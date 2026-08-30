"""web_node — browser ⇄ ROS-graph bridge.

Runs the legacy FastAPI app (same routes, same websocket contract) alongside
rclpy. The legacy COMMAND_TRANSLATOR string→enum table becomes a
string→service-client table of the same shape.

Outgoing websocket messages (app.js contract preserved):
    {"type": "camera_frame", "rgb": <b64 jpg>, "depth": <b64 jpg>}
    {"type": "command_result", "command": ..., "result": ...}
plus new feedback types the legacy UI never had:
    {"type": "targets", ...} {"type": "cobot_status", ...} {"type": "layer_report", ...}
    {"type": "detections", ...}   latest /vision/detections_px (pixel OBBs → overlay)
    {"type": "health", ...}       latest /health/summary (per-node level + metrics)
    {"type": "pick_event", ...}   one per can from /task/pick_event (pick monitor)
    {"type": "manual_dot_request", "waiting": bool}   (temporary, see below)

TEMPORARY manual centre select: while localization publishes
/vision/manual_dot_request=true the UI prompts for a click on the color feed;
the ("VISION", "click_center") command republishes it as a PointStamped on
/vision/manual_dot_click, and ("VISION", "skip_center") publishes Empty on
/vision/manual_dot_skip to drop a pose whose marker is out of frame. Remove
with the localization-side flag.

Threading model: rclpy spins in a background thread; uvicorn owns the main
thread's asyncio loop. ROS callbacks hand payloads to the loop via
run_coroutine_threadsafe.
"""

import asyncio
import base64
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from ament_index_python.packages import get_package_share_directory

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger

from cobot_interfaces.msg import (
    CanTargetArray,
    HealthSummary,
    Heartbeat,
    LayerReport,
    NodeHealth,
    ObbDetectionArray,
    PickEvent,
    TaskStatus,
)

from cobot_health.heartbeat import SUMMARY_TOPIC, HeartbeatPublisher, RateMeter


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Legacy COMMAND_TRANSLATOR, reshaped: (receiver, command) → Trigger service.
# Operations go to the task manager — it owns the state machine and drives
# cobot_node/vision itself. Only display-only commands hit nodes directly.
COMMAND_ROUTES = {
    ("VISION", "detect"): "/vision/scan",  # display-only scan (nothing moves)
    ("COBOT", "start"): "/task/start",
    ("COBOT", "pickup"): "/task/start",  # pickup == start the scan→pick cycle
    ("COBOT", "stop"): "/task/stop",
    ("COBOT", "calibrate"): "/task/calibrate",
    ("CAMERA", "save"): "/camera/save_frames",  # capture pair → vision_workspace
}

HEALTH_LEVEL_NAMES = {
    NodeHealth.LEVEL_OK: "ok",
    NodeHealth.LEVEL_WARN: "warn",
    NodeHealth.LEVEL_ERROR: "error",
    NodeHealth.LEVEL_STALE: "stale",
    NodeHealth.LEVEL_UNKNOWN: "unknown",
}

# Pick events kept for replay to a browser that connects mid-layer.
MAX_PICK_EVENTS = 64

TASK_STATE_NAMES = {
    TaskStatus.STATE_DISCONNECTED: "disconnected",
    TaskStatus.STATE_IDLE: "idle",
    TaskStatus.STATE_SCANNING: "scanning",
    TaskStatus.STATE_PICKING: "picking",
    TaskStatus.STATE_CALIBRATING: "calibrating",
    TaskStatus.STATE_STOPPING: "stopping",
    TaskStatus.STATE_ERROR: "error",
}


def _encode_jpeg_b64(img: np.ndarray | None) -> str | None:
    if img is None:
        return None
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 255).astype(np.uint8)
    ok, buffer = cv2.imencode(".jpg", img)
    if not ok:
        return None
    return base64.b64encode(buffer).decode("ascii")


class WebNode(Node):
    def __init__(self):
        super().__init__("web_node")

        self.declare_parameter("port", 8000)
        self.declare_parameter("web_stream_hz", 15.0)
        # The bringup params YAML — source of truth for GET/POST /api/config.
        self.declare_parameter("params_file", "")
        self.declare_parameter("results_dir", "/ws/vision_workspace/operation/results")

        self.port = int(self.get_parameter("port").value)
        self.params_file = self.get_parameter("params_file").value
        self.results_dir = Path(self.get_parameter("results_dir").value)

        # Latest frames cached by the ROS side, encoded/broadcast on a timer.
        self._frame_lock = threading.Lock()
        self._latest_color: np.ndarray | None = None
        self._latest_depth: np.ndarray | None = None

        # Set by FastAPI on startup; ROS callbacks broadcast through it.
        self.ws_loop: asyncio.AbstractEventLoop | None = None
        self.ws_clients: list[WebSocket] = []

        # Last value of latched/edge-triggered feedback, replayed to each newly
        # connected browser. Broadcasts are fire-and-forget to whoever is
        # connected AT THE TIME; a page that opens or refreshes later would
        # otherwise miss a state set before it connected (e.g. a calibration
        # already waiting for a marker click). Cached on the ROS thread, read
        # on the asyncio thread — a plain assignment, safe without a lock.
        self._last_status_payload: dict | None = None
        self._manual_waiting = False
        self._last_detections_payload: dict | None = None
        self._last_targets_payload: dict | None = None
        self._last_health_payload: dict | None = None
        self._last_health_mono: float | None = None
        self._pick_events: list[dict] = []
        self._stream_rate = RateMeter()

        self.create_subscription(Image, "/camera/color/image_raw", self._on_color, SENSOR_QOS)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, SENSOR_QOS)
        self.create_subscription(CanTargetArray, "/vision/targets", self._on_targets, 10)
        self.create_subscription(TaskStatus, "/task/status", self._on_status, LATCHED_QOS)
        self.create_subscription(LayerReport, "/task/layer_report", self._on_report, 10)
        self.create_subscription(
            ObbDetectionArray, "/vision/detections_px", self._on_detections, 10
        )
        self.create_subscription(HealthSummary, SUMMARY_TOPIC, self._on_health, LATCHED_QOS)
        self.create_subscription(PickEvent, "/task/pick_event", self._on_pick_event, 10)

        # TEMPORARY manual centre select: localization asks for a click, the
        # browser answers with a COLOR-image pixel.
        self.create_subscription(
            Bool, "/vision/manual_dot_request", self._on_manual_request, LATCHED_QOS
        )
        self._manual_click_pub = self.create_publisher(
            PointStamped, "/vision/manual_dot_click", 10
        )
        self._manual_skip_pub = self.create_publisher(
            Empty, "/vision/manual_dot_skip", 10
        )

        self._trigger_clients = {
            route: self.create_client(Trigger, route)
            for route in set(COMMAND_ROUTES.values())
        }

        stream_hz = float(self.get_parameter("web_stream_hz").value)
        self._stream_hz_target = stream_hz
        self.create_timer(1.0 / stream_hz, self._stream_frames)

        self._hb = HeartbeatPublisher(self, self._health_report)

        self.get_logger().info(f"web_node ready — UI on port {self.port}")

    # ------------------------------------------------------------------
    # ROS → browser
    # ------------------------------------------------------------------

    def _on_color(self, msg: Image):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        with self._frame_lock:
            self._latest_color = frame

    def _on_depth(self, msg: Image):
        depth = np.frombuffer(msg.data, dtype=np.int16).reshape(msg.height, msg.width)
        with self._frame_lock:
            self._latest_depth = depth

    def _stream_frames(self):
        if not self.ws_clients:
            return
        payload = self._camera_frame_payload()
        if payload["rgb"] is None and payload["depth"] is None:
            return  # nothing received from the camera yet
        self._stream_rate.tick()
        self.broadcast(payload)

    def _camera_frame_payload(self) -> dict:
        with self._frame_lock:
            color = self._latest_color
            depth = self._latest_depth

        color_bgr = cv2.cvtColor(color, cv2.COLOR_RGB2BGR) if color is not None else None

        depth_vis = None
        if depth is not None:
            depth_norm = cv2.normalize(depth.astype(np.float32), None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = depth_norm.astype(np.uint8)

        return {
            "type": "camera_frame",
            "rgb": _encode_jpeg_b64(color_bgr),
            "depth": _encode_jpeg_b64(depth_vis),
        }

    def _on_targets(self, msg: CanTargetArray):
        payload = {
                "type": "targets",
                "count": len(msg.targets),
                "targets": [
                    {
                        "detection_index": t.detection_index,
                        "real_x": t.real_x,
                        "real_y": t.real_y,
                        "real_z": t.real_z,
                        "real_rz": t.real_rz,
                        "confidence": t.confidence,
                    }
                    for t in msg.targets
                ],
            }
        self._last_targets_payload = payload
        self.broadcast(payload)

    def _on_detections(self, msg: ObbDetectionArray):
        # A new scan invalidates the previous overlay and its pick colors.
        payload = {
            "type": "detections",
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "image_width": int(msg.image_width),
            "image_height": int(msg.image_height),
            "detections": [
                {
                    "index": i,
                    "class_id": d.class_id,
                    "confidence": float(d.confidence),
                    "center_x": float(d.center_x),
                    "center_y": float(d.center_y),
                    "corners_px": [float(v) for v in d.corners_px],
                }
                for i, d in enumerate(msg.detections)
            ],
        }
        self._last_detections_payload = payload
        self._pick_events = []
        self._last_targets_payload = None
        self.broadcast(payload)

    def _on_health(self, msg: HealthSummary):
        self._last_health_mono = time.monotonic()
        self._last_health_payload = self._health_payload(msg)
        self.broadcast(self._last_health_payload)

    def _health_payload(self, msg: HealthSummary) -> dict:
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_s = self.get_clock().now().nanoseconds * 1e-9
        return {
            "type": "health",
            "level": HEALTH_LEVEL_NAMES.get(msg.level, "unknown"),
            # Age from the message stamp (not receive time): the topic is
            # latched, so a restarted web_node must not show an old summary as fresh.
            "summary_age_s": max(0.0, now_s - stamp_s),
            "nodes": [
                {
                    "name": n.node_name,
                    "level": HEALTH_LEVEL_NAMES.get(n.level, "unknown"),
                    "detail": n.detail,
                    "age_s": float(n.age_s),
                    "metrics": {kv.key: kv.value for kv in n.metrics},
                }
                for n in msg.nodes
            ],
        }

    def _on_pick_event(self, msg: PickEvent):
        payload = {
            "type": "pick_event",
            "index": int(msg.index),
            "total": int(msg.total),
            "detection_index": int(msg.detection_index),
            "outcome": msg.outcome,
            "verify_ran": bool(msg.verify_ran),
            "verified": bool(msg.verified),
            "measured_z": float(msg.measured_z),
            "message": msg.message,
            "target": {
                "real_x": msg.target.real_x,
                "real_y": msg.target.real_y,
                "real_z": msg.target.real_z,
                "real_rz": msg.target.real_rz,
            },
        }
        self._pick_events = (self._pick_events + [payload])[-MAX_PICK_EVENTS:]
        self.broadcast(payload)

    def _health_report(self):
        stream_hz, _ = self._stream_rate.read()
        clients = len(self.ws_clients)
        health_age = (
            None if self._last_health_mono is None else time.monotonic() - self._last_health_mono
        )
        with self._frame_lock:
            have_frames = self._latest_color is not None or self._latest_depth is not None
        level = Heartbeat.LEVEL_OK if self.ws_loop is not None else Heartbeat.LEVEL_WARN
        detail = f"{clients} browser(s)"
        if clients and have_frames:
            detail += f" · stream {stream_hz:.1f} Hz"
        elif not have_frames:
            detail += " · no camera frames"
        return level, detail, {
            "ws_clients": clients,
            "stream_hz_target": self._stream_hz_target,
            "stream_hz_actual": stream_hz,
            "health_summary_age_s": health_age,
            "port": self.port,
        }

    def _on_status(self, msg: TaskStatus):
        # Websocket type stays "cobot_status" — app.js contract unchanged.
        # progress/active_task are additive; the UI can ignore them.
        payload = {
            "type": "cobot_status",
            "state": TASK_STATE_NAMES.get(msg.state, "unknown"),
            "detail": msg.detail,
            "active_task": msg.active_task,
            "progress_current": msg.progress_current,
            "progress_total": msg.progress_total,
        }
        self._last_status_payload = payload  # replayed to late-connecting browsers
        self.broadcast(payload)

    def _on_report(self, msg: LayerReport):
        self.broadcast(
            {
                "type": "layer_report",
                "successes": msg.successes,
                "failures": msg.failures,
                "overall_ok": msg.overall_ok,
            }
        )

    def _on_manual_request(self, msg: Bool):
        # TEMPORARY manual centre select — the UI prompts while this is true.
        self._manual_waiting = bool(msg.data)  # replayed to late-connecting browsers
        self.broadcast({"type": "manual_dot_request", "waiting": self._manual_waiting})

    def broadcast(self, payload: dict):
        """Thread-safe: callable from ROS callbacks."""
        if self.ws_loop is None:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast_async(payload), self.ws_loop)

    async def _broadcast_async(self, payload: dict):
        disconnected = []
        for ws in self.ws_clients:
            try:
                await ws.send_json(payload)
            except Exception:  # noqa: BLE001
                disconnected.append(ws)
        for ws in disconnected:
            if ws in self.ws_clients:
                self.ws_clients.remove(ws)

    # ------------------------------------------------------------------
    # Browser → ROS
    # ------------------------------------------------------------------

    async def handle_web_command(self, receiver: str, command: str, payload) -> dict:
        if receiver == "CAMERA" and command == "snapshot":
            # Frames stream continuously now; snapshot = push one immediately.
            await self._broadcast_async(self._camera_frame_payload())
            return {"result": "ok", "message": "frame pushed"}

        if receiver == "VISION" and command == "click_center":
            # TEMPORARY manual centre select — a topic, not a Trigger service.
            return self._publish_manual_click(payload)

        if receiver == "VISION" and command == "skip_center":
            # TEMPORARY manual centre select — drop this calibration pose.
            self._manual_skip_pub.publish(Empty())
            return {"result": "ok", "message": "pose skipped"}

        route = COMMAND_ROUTES.get((receiver, command))
        if route is None:
            raise ValueError(f"Invalid command '{command}' for receiver '{receiver}'")

        client = self._trigger_clients[route]
        if not client.service_is_ready():
            raise RuntimeError(f"service {route} not available")

        result = await self._call_trigger(client)
        return {"result": result.success, "message": result.message}

    def _publish_manual_click(self, payload) -> dict:
        """TEMPORARY manual centre select: payload {"u": px, "v": px} in COLOR
        image pixels (the browser scales its click back to natural size)."""
        if not isinstance(payload, dict) or "u" not in payload or "v" not in payload:
            raise ValueError('click_center needs a payload of {"u": px, "v": px}')
        try:
            u, v = float(payload["u"]), float(payload["v"])
        except (TypeError, ValueError) as e:
            raise ValueError(f"click_center u/v must be numbers: {e}") from e

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color"
        msg.point.x = u
        msg.point.y = v
        self._manual_click_pub.publish(msg)
        return {"result": "ok", "message": f"marker click at ({u:.0f}, {v:.0f})"}

    async def _call_trigger(self, client, timeout: float = 10.0):
        """Bridge an rclpy future (resolved on the executor thread) to asyncio."""
        loop = asyncio.get_running_loop()
        aio_future = loop.create_future()

        def _done(ros_future):
            try:
                loop.call_soon_threadsafe(aio_future.set_result, ros_future.result())
            except Exception as e:  # noqa: BLE001
                loop.call_soon_threadsafe(aio_future.set_exception, e)

        client.call_async(Trigger.Request()).add_done_callback(_done)
        return await asyncio.wait_for(aio_future, timeout=timeout)

    # ------------------------------------------------------------------
    # Config editing (POST /api/config): write YAML + live set_parameters
    # ------------------------------------------------------------------

    def apply_config(self, config: dict) -> dict:
        """config = {node_name: {param: value}} — the params YAML shape."""
        outcomes = {}
        for node_name, section in config.items():
            params = section.get("ros__parameters", section)
            if not isinstance(params, dict):
                continue
            outcomes[node_name] = self._set_node_parameters(node_name, params)
        return outcomes

    def _set_node_parameters(self, node_name: str, params: dict) -> str:
        client = self.create_client(SetParameters, f"/{node_name}/set_parameters")
        if not client.wait_for_service(timeout_sec=1.0):
            self.destroy_client(client)
            return "node not reachable — saved to file only, applies on restart"

        request = SetParameters.Request()
        for name, value in params.items():
            request.parameters.append(Parameter(name=name, value=_to_parameter_value(value)))

        future = client.call_async(request)
        # Called from the uvicorn thread; the executor thread resolves it.
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=5.0):
            self.destroy_client(client)
            return "timeout"

        results = future.result().results
        self.destroy_client(client)
        rejected = [r.reason for r in results if not r.successful]
        return f"rejected: {rejected}" if rejected else "applied"


def _to_parameter_value(value) -> ParameterValue:
    pv = ParameterValue()
    if isinstance(value, bool):
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = value
    elif isinstance(value, int):
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = value
    elif isinstance(value, float):
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = value
    elif isinstance(value, list) and value and all(isinstance(v, (int, float)) for v in value):
        pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        pv.double_array_value = [float(v) for v in value]
    else:
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = str(value)
    return pv


# ----------------------------------------------------------------------
# FastAPI app (legacy routes preserved)
# ----------------------------------------------------------------------


def create_app(node: WebNode) -> FastAPI:
    share_dir = Path(get_package_share_directory("cobot_web"))

    app = FastAPI()
    templates = Jinja2Templates(directory=share_dir / "templates")

    app.mount("/static", StaticFiles(directory=share_dir / "static"), name="static")

    node.results_dir.mkdir(parents=True, exist_ok=True)
    app.mount("/results", StaticFiles(directory=node.results_dir), name="results")

    @app.on_event("startup")
    async def _capture_loop():
        node.ws_loop = asyncio.get_running_loop()

    @app.get("/", response_class=HTMLResponse)
    def home(request: Request):
        return templates.TemplateResponse(request=request, name="index.html")

    @app.get("/api/config")
    def get_config():
        if not node.params_file:
            return {}
        with open(node.params_file, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    @app.post("/api/config")
    def save_config(config: dict):
        outcomes = {}
        if node.params_file:
            with open(node.params_file, "w", encoding="utf-8") as f:
                yaml.safe_dump(config, f, sort_keys=False)
            outcomes = node.apply_config(config)

        return {
            "status": "saved",
            "saved_to": str(node.params_file),
            "applied": outcomes,
            "config": config,
        }

    @app.get("/api/latest-image")
    def latest_image():
        image_files = (
            list(node.results_dir.glob("*.jpg"))
            + list(node.results_dir.glob("*.jpeg"))
            + list(node.results_dir.glob("*.png"))
        )
        if not image_files:
            return {"image": None}
        newest = max(image_files, key=lambda p: p.stat().st_mtime)
        return {"image": f"/results/{newest.name}", "filename": newest.name}

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        node.ws_clients.append(websocket)
        await websocket.send_json({"type": "connection", "message": "WebSocket connected"})

        # Replay the current state so a page opened or refreshed mid-operation
        # is immediately correct — otherwise it would sit blank until the next
        # change. Most important for the manual-click prompt: a calibration
        # already waiting must show it right away.
        if node._last_status_payload is not None:
            await websocket.send_json(node._last_status_payload)
        await websocket.send_json(
            {"type": "manual_dot_request", "waiting": node._manual_waiting}
        )
        if node._last_health_payload is not None:
            await websocket.send_json(node._last_health_payload)
        if node._last_detections_payload is not None:
            await websocket.send_json(node._last_detections_payload)
        if node._last_targets_payload is not None:
            await websocket.send_json(node._last_targets_payload)
        for event in list(node._pick_events):
            await websocket.send_json(event)

        try:
            while True:
                data = await websocket.receive_json()

                receiver = data.get("receiver")
                command = data.get("command")
                payload = data.get("payload")

                try:
                    result = await node.handle_web_command(receiver, command, payload)
                except Exception as e:  # noqa: BLE001
                    node.get_logger().error(f"web command failed: {e}")
                    result = {"result": str(e), "message": "websocket command failed"}

                await websocket.send_json(
                    {"type": "command_result", "command": command, "result": result}
                )

        except WebSocketDisconnect:
            pass
        finally:
            if websocket in node.ws_clients:
                node.ws_clients.remove(websocket)

    return app


def main(args=None):
    rclpy.init(args=args)
    node = WebNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    app = create_app(node)
    try:
        uvicorn.run(app, host="0.0.0.0", port=node.port, log_level="info")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
