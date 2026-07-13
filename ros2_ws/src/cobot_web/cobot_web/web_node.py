"""web_node — browser ⇄ ROS-graph bridge.

Runs the legacy FastAPI app (same routes, same websocket contract) alongside
rclpy. The legacy COMMAND_TRANSLATOR string→enum table becomes a
string→service-client table of the same shape.

Outgoing websocket messages (app.js contract preserved):
    {"type": "camera_frame", "rgb": <b64 jpg>, "depth": <b64 jpg>}
    {"type": "command_result", "command": ..., "result": ...}
plus new feedback types the legacy UI never had:
    {"type": "targets", ...} {"type": "cobot_status", ...} {"type": "layer_report", ...}

Threading model: rclpy spins in a background thread; uvicorn owns the main
thread's asyncio loop. ROS callbacks hand payloads to the loop via
run_coroutine_threadsafe.
"""

import asyncio
import base64
import json
import threading
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

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from cobot_interfaces.msg import CanTargetArray, CobotStatus, LayerReport


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Legacy COMMAND_TRANSLATOR, reshaped: (receiver, command) → Trigger service.
COMMAND_ROUTES = {
    ("VISION", "detect"): "/vision/scan",
    ("COBOT", "start"): "/cobot/start",
    ("COBOT", "pickup"): "/cobot/start",  # pickup == start the scan→pick cycle
    ("COBOT", "stop"): "/cobot/stop",
    ("COBOT", "calibrate"): "/cobot/calibrate",
}

COBOT_STATE_NAMES = {
    CobotStatus.STATE_DISCONNECTED: "disconnected",
    CobotStatus.STATE_IDLE: "idle",
    CobotStatus.STATE_PICKING: "picking",
    CobotStatus.STATE_STOPPING: "stopping",
    CobotStatus.STATE_ERROR: "error",
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
        self.declare_parameter("web_stream_hz", 5.0)
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

        self.create_subscription(Image, "/camera/color/image_raw", self._on_color, SENSOR_QOS)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, SENSOR_QOS)
        self.create_subscription(CanTargetArray, "/vision/targets", self._on_targets, 10)
        self.create_subscription(CobotStatus, "/cobot/status", self._on_status, LATCHED_QOS)
        self.create_subscription(LayerReport, "/cobot/layer_report", self._on_report, 10)

        self._trigger_clients = {
            route: self.create_client(Trigger, route)
            for route in set(COMMAND_ROUTES.values())
        }

        stream_hz = float(self.get_parameter("web_stream_hz").value)
        self.create_timer(1.0 / stream_hz, self._stream_frames)

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
        self.broadcast(
            {
                "type": "targets",
                "count": len(msg.targets),
                "targets": [
                    {
                        "real_x": t.real_x,
                        "real_y": t.real_y,
                        "real_z": t.real_z,
                        "real_rz": t.real_rz,
                        "confidence": t.confidence,
                    }
                    for t in msg.targets
                ],
            }
        )

    def _on_status(self, msg: CobotStatus):
        self.broadcast(
            {
                "type": "cobot_status",
                "state": COBOT_STATE_NAMES.get(msg.state, "unknown"),
                "detail": msg.detail,
            }
        )

    def _on_report(self, msg: LayerReport):
        self.broadcast(
            {
                "type": "layer_report",
                "successes": msg.successes,
                "failures": msg.failures,
                "overall_ok": msg.overall_ok,
            }
        )

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

        route = COMMAND_ROUTES.get((receiver, command))
        if route is None:
            raise ValueError(f"Invalid command '{command}' for receiver '{receiver}'")

        client = self._trigger_clients[route]
        if not client.service_is_ready():
            raise RuntimeError(f"service {route} not available")

        result = await self._call_trigger(client)
        return {"result": result.success, "message": result.message}

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
