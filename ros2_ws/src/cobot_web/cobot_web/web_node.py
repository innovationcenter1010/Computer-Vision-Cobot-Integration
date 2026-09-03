"""web_node — browser ⇄ ROS-graph bridge.

Runs the legacy FastAPI app (same routes, same websocket contract) alongside
rclpy. The legacy COMMAND_TRANSLATOR string→enum table becomes a
string→service-client table of the same shape.

Outgoing websocket messages (app.js contract preserved):
    {"type": "camera_frame", "rgb": <b64 jpg>, "depth": <b64 jpg>}
    {"type": "command_result", "command": ..., "result": ...}
plus new feedback types the legacy UI never had:
    {"type": "targets", ...} {"type": "cobot_status", ...} {"type": "layer_report", ...}
    {"type": "detections", ...}   latest /vision/detections_px (pixel OBBs → overlay;
                                  "calibration": true for marker-model runs, which
                                  do not disturb the can overlay / pick monitor)
    {"type": "calibration_marker", ...}  /vision/calibration_marker: one per
                                  calibration sample (accepted / rejected + where)
    {"type": "health", ...}       latest /health/summary (per-node level + metrics)
    {"type": "pick_event", ...}   one per can from /task/pick_event (pick monitor)
    {"type": "workflows", ...}    latched /task/workflows (dashboard selector)
    {"type": "system_state", ...} latched /task/system_state (capability +
                                  individual robot/vision facts, derived by
                                  task_manager — read-only)
    {"type": "manual_dot_request", "waiting": bool}   (temporary, see below)
    {"type": "tool_profile", ...}  latched /cobot/profile: the YRC1000 read once
                                  after connect (system info, TOOL.CND tool
                                  files, UFRAME.CND user frames — READ-ONLY) +
                                  the configured tool names + the active tool
    {"type": "cobot_state", ...}   full /cobot/state digest for the Cobot panel
                                   (link/servo/hold/alarms/mode/IO + joints),
                                   throttled to cobot_state_hz, replayed on connect
    {"type": "tool_state", ...}    active tool / pose / joints from /cobot/state,
                                  throttled to <= 2 Hz and only on change
    {"type": "recording", ...}     session recorder status (active/duration/
                                  export link), broadcast on change + on connect.
                                  ("RECORD", start|stop|status) arms/stops the
                                  server-side recorder (session_recorder.py);
                                  a stop exports a standalone replay HTML into
                                  results_dir, downloadable at /results/.
Frame opt-out: a client may connect as /ws?frames=off to receive every message
type EXCEPT camera_frame. The dev panel (/dev) does this — it has no feeds, so
its socket never carries frame payloads and tuning there cannot degrade the
operator panel's stream. Its /api/param saves write cobot_params.yaml AND
live-apply via /{node}/set_parameters (the physical tunables are read at point
of use, or synced by a parameter callback in cobot_control), so edits take
effect without a restart.

Commands: ("COBOT", "set_active_tool") {name} → /cobot/set_active_tool (the
remembered positions are then converted automatically by cobot_operation);
("COBOT", "convert_trajectories") → /cobot/convert_trajectories (manual retry);
("COBOT", "teach_point") {key, index} → /cobot/save_teach_point.

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
    CalibrationMarker,
    CobotState,
    ControllerProfile,
    TargetArray,
    HealthSummary,
    Heartbeat,
    LayerReport,
    NodeHealth,
    ObbDetectionArray,
    PickEvent,
    SystemState,
    TaskStatus,
    WorkflowList,
)
from cobot_interfaces.srv import (
    ConvertTrajectories,
    SaveTeachPoint,
    SetActiveTool,
    SetTeachPoint,
    StartWorkflow,
)

from cobot_health.heartbeat import SUMMARY_TOPIC, HeartbeatPublisher, RateMeter
from cobot_web.session_recorder import SessionRecorder


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Legacy COMMAND_TRANSLATOR, reshaped: (receiver, command) → Trigger service.
# Operations go to the task manager — it owns the state machine and drives
# cobot_operation/vision itself. The dashboard's main path is
# ("COBOT", "start_workflow") with payload {"workflow": id} → /task/start_workflow
# (StartWorkflow, handled in handle_web_command); the routes below are the
# Trigger aliases task_manager still serves.
# NOTE: /vision/scan is now a Scan ACTION (localization_node); the old
# ("VISION", "detect") Trigger route was removed — re-add with an ActionClient.
COMMAND_ROUTES = {
    ("COBOT", "start"): "/task/start",       # alias: default_workflow
    ("COBOT", "pickup"): "/task/start",      # alias: default_workflow
    ("COBOT", "stop"): "/task/stop",
    ("COBOT", "calibrate"): "/task/calibrate",  # alias: "calibration" workflow
    ("CAMERA", "save"): "/camera/save_frames",  # capture pair → vision_workspace
}

COBOT_CONTROL_MODES = {
    CobotState.CONTROL_MODE_TEACH: "teach",
    CobotState.CONTROL_MODE_PLAY: "play",
    CobotState.CONTROL_MODE_REMOTE: "remote",
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
        # Developer dashboard (/dev): cell setup YAML (per-param edits via
        # POST /api/setup-param) and the runtime trajectories.json (read-only
        # here — writes go through cobot_operation's services so its in-memory
        # store stays authoritative).
        self.declare_parameter("setup_file", "")
        self.declare_parameter("trajectories_file", "")
        self.declare_parameter("results_dir", "/ws/vision_workspace/operation/results")
        # Cobot panel: how often the /cobot/state digest is pushed (the topic
        # itself is 10 Hz) and the per-joint range the joint bars are drawn
        # against — [S_min, S_max, L_min, L_max, U, U, R, R, B, B, T, T] degrees.
        # Set to the controller's soft limits for the arm model in use.
        self.declare_parameter("cobot_state_hz", 5.0)
        # Session recorder: camera_frame decimation rate in the recording and
        # the auto-stop safety caps (a stop always triggers the replay export).
        self.declare_parameter("record_frame_hz", 3.0)
        self.declare_parameter("record_max_minutes", 10.0)
        self.declare_parameter("record_max_mb", 200.0)
        # Detector model ids used to find the calibration marker. Their
        # detections are shown as marker hits and never reset the can
        # overlay / pick monitor (every detect publishes on /vision/detections_px).
        self.declare_parameter(
            "calibration_model_ids", ["red_dot", "green_light", "green_light_v2"]
        )
        self.declare_parameter(
            "joint_limits_deg",
            [-180.0, 180.0, -180.0, 180.0, -180.0, 180.0,
             -180.0, 180.0, -180.0, 180.0, -180.0, 180.0],
        )

        self.port = int(self.get_parameter("port").value)
        self.params_file = self.get_parameter("params_file").value
        self.setup_file = self.get_parameter("setup_file").value
        self.trajectories_file = self.get_parameter("trajectories_file").value
        self.results_dir = Path(self.get_parameter("results_dir").value)

        # Session recorder: broadcast() tees every payload into it while armed
        # (see session_recorder.py). Status changes are pushed back through
        # broadcast() itself — record() skips "recording" payloads.
        self._recorder = SessionRecorder(
            results_dir=self.results_dir,
            share_dir_resolver=lambda: Path(get_package_share_directory("cobot_web")),
            frame_hz=float(self.get_parameter("record_frame_hz").value),
            max_seconds=float(self.get_parameter("record_max_minutes").value) * 60.0,
            max_bytes=int(float(self.get_parameter("record_max_mb").value) * 1e6),
            on_status_change=self.broadcast,
            logger=self.get_logger(),
        )

        # Latest frames cached by the ROS side, encoded/broadcast on a timer.
        self._frame_lock = threading.Lock()
        self._latest_color: np.ndarray | None = None
        self._latest_depth: np.ndarray | None = None

        # Set by FastAPI on startup; ROS callbacks broadcast through it.
        self.ws_loop: asyncio.AbstractEventLoop | None = None
        self.ws_clients: list[WebSocket] = []
        # Subset of ws_clients that receives camera frames. The dev panel
        # connects with /ws?frames=off (it has no feeds), so its socket never
        # carries the ~100s-of-KB frame payloads that starve the operator
        # panel's stream — it still gets every non-frame broadcast.
        self.ws_frame_clients: list[WebSocket] = []

        # Last value of latched/edge-triggered feedback, replayed to each newly
        # connected browser. Broadcasts are fire-and-forget to whoever is
        # connected AT THE TIME; a page that opens or refreshes later would
        # otherwise miss a state set before it connected (e.g. a calibration
        # already waiting for a marker click). Cached on the ROS thread, read
        # on the asyncio thread — a plain assignment, safe without a lock.
        self._last_status_payload: dict | None = None
        self._last_workflows_payload: dict | None = None
        self._last_system_state_payload: dict | None = None
        self._manual_waiting = False
        self._last_detections_payload: dict | None = None
        self._calibration_models = set(
            str(v) for v in self.get_parameter("calibration_model_ids").value
        )
        # Calibration verification overlay: accepted markers of the current
        # run (cleared by reset) + the last result, replayed on connect.
        self._cal_markers: list[dict] = []
        self._last_marker_payload: dict | None = None
        self._last_targets_payload: dict | None = None
        self._last_health_payload: dict | None = None
        self._last_health_mono: float | None = None
        self._pick_events: list[dict] = []
        self._stream_rate = RateMeter()
        # camera_frame coalescing: while a broadcast is in flight, the timer
        # REPLACES the pending frame instead of scheduling another send, so a
        # slow client costs dropped frames — never a growing backlog of stale
        # ones. Guarded by a lock: written on the ROS timer thread, drained on
        # the asyncio thread.
        self._frame_send_lock = threading.Lock()
        self._pending_frame: dict | None = None
        self._frame_send_running = False
        # Tooling: latched controller profile + throttled active-tool/pose state.
        self._last_profile_payload: dict | None = None
        self._last_cobot_state: CobotState | None = None
        self._last_cobot_state_mono: float | None = None
        self._last_tool_state_payload: dict | None = None
        # Cobot panel digest (throttled, replayed on connect).
        self._last_cobot_state_payload: dict | None = None
        limits = [float(v) for v in self.get_parameter("joint_limits_deg").value]
        self._joint_limits = [limits[i:i + 2] for i in range(0, len(limits) - 1, 2)][:6]
        cobot_state_hz = max(float(self.get_parameter("cobot_state_hz").value), 0.5)
        self.create_timer(1.0 / cobot_state_hz, self._tick_cobot_state)

        self.create_subscription(Image, "/camera/color/image_raw", self._on_color, SENSOR_QOS)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, SENSOR_QOS)
        self.create_subscription(TargetArray, "/vision/targets", self._on_targets, 10)
        self.create_subscription(TaskStatus, "/task/status", self._on_status, LATCHED_QOS)
        self.create_subscription(WorkflowList, "/task/workflows", self._on_workflows, LATCHED_QOS)
        self.create_subscription(
            SystemState, "/task/system_state", self._on_system_state, LATCHED_QOS
        )
        self.create_subscription(LayerReport, "/task/layer_report", self._on_report, 10)
        self.create_subscription(
            ObbDetectionArray, "/vision/detections_px", self._on_detections, 10
        )
        self.create_subscription(HealthSummary, SUMMARY_TOPIC, self._on_health, LATCHED_QOS)
        self.create_subscription(
            CalibrationMarker, "/vision/calibration_marker", self._on_cal_marker, LATCHED_QOS
        )
        self.create_subscription(PickEvent, "/task/pick_event", self._on_pick_event, 10)
        self.create_subscription(ControllerProfile, "/cobot/profile", self._on_profile, LATCHED_QOS)
        # 10 Hz topic: cache only; _tick_tool_state forwards a change-only digest.
        self.create_subscription(CobotState, "/cobot/state", self._on_cobot_state, 1)
        self.create_timer(0.5, self._tick_tool_state)

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
        self._workflow_client = self.create_client(StartWorkflow, "/task/start_workflow")
        self._set_tool_client = self.create_client(SetActiveTool, "/cobot/set_active_tool")
        self._convert_client = self.create_client(
            ConvertTrajectories, "/cobot/convert_trajectories"
        )
        self._teach_client = self.create_client(SaveTeachPoint, "/cobot/save_teach_point")
        self._set_point_client = self.create_client(SetTeachPoint, "/cobot/set_teach_point")

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
        # Encode even with zero browsers while a recording wants a frame —
        # wants_frame() opens at record_frame_hz, so the idle-recording cost
        # stays at the decimated rate, not the full stream rate.
        if not self.ws_frame_clients and not self._recorder.wants_frame():
            return
        payload = self._camera_frame_payload()
        if payload["rgb"] is None and payload["depth"] is None:
            return  # nothing received from the camera yet
        # Recording tap at production time (the recorder decimates itself), so
        # a recording keeps every frame the websocket path may drop below.
        self._recorder.record(payload)
        if not self.ws_frame_clients or self.ws_loop is None:
            return
        with self._frame_send_lock:
            self._pending_frame = payload  # newest wins; a stale pending frame is dropped
            if self._frame_send_running:
                return  # the in-flight drain loop picks it up
            self._frame_send_running = True
        asyncio.run_coroutine_threadsafe(self._send_pending_frames(), self.ws_loop)

    async def _send_pending_frames(self):
        """Send pending camera frames one broadcast at a time.

        Runs until no frame is pending, so the timer never stacks a second
        broadcast behind a slow send — clients that can't keep up see a lower
        frame rate, never a growing backlog of stale frames.
        """
        while True:
            with self._frame_send_lock:
                payload = self._pending_frame
                self._pending_frame = None
                if payload is None:
                    self._frame_send_running = False
                    return
            self._stream_rate.tick()  # frames actually sent, not merely produced
            await self._broadcast_async(payload, self.ws_frame_clients)

    def _camera_frame_payload(self) -> dict:
        with self._frame_lock:
            color = self._latest_color
            depth = self._latest_depth

        color_bgr = cv2.cvtColor(color, cv2.COLOR_RGB2BGR) if color is not None else None

        depth_vis = None
        if depth is not None:
            depth_norm = cv2.normalize(depth.astype(np.float32), None, 0, 255, cv2.NORM_MINMAX)
            # RGB colormap instead of grayscale: JET runs blue → cyan → yellow
            # → red, so with the min-max normalize above near pixels are blue
            # and far pixels red. applyColorMap returns BGR, which is exactly
            # what imencode wants.
            depth_vis = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_JET)

        return {
            "type": "camera_frame",
            "rgb": _encode_jpeg_b64(color_bgr),
            "depth": _encode_jpeg_b64(depth_vis),
        }

    def _on_targets(self, msg: TargetArray):
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
        # A new SCAN invalidates the previous overlay and its pick colors;
        # a calibration-marker detect is shown separately and leaves them alone.
        calibration = msg.model_id in self._calibration_models
        payload = {
            "type": "detections",
            "model_id": msg.model_id,
            "calibration": calibration,
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
        if not calibration:
            self._last_detections_payload = payload
            self._pick_events = []
            self._last_targets_payload = None
        self.broadcast(payload)

    def _on_cal_marker(self, msg: CalibrationMarker):
        payload = {
            "type": "calibration_marker",
            "reset": bool(msg.reset),
            "mode": msg.mode,
            "model_id": msg.model_id,
            "sample_index": int(msg.sample_index),
            "accepted": bool(msg.accepted),
            "message": msg.message,
            "u": float(msg.u),
            "v": float(msg.v),
            "depth_mm": float(msg.depth_mm),
            "confidence": float(msg.confidence),
            "hits": int(msg.hits),
            "reads": int(msg.reads),
            "tcp": [float(msg.tcp_x), float(msg.tcp_y), float(msg.tcp_z)],
            "image_width": int(msg.image_width),
            "image_height": int(msg.image_height),
            "detections": [
                {
                    "class_id": d.class_id,
                    "confidence": float(d.confidence),
                    "center_x": float(d.center_x),
                    "center_y": float(d.center_y),
                    "corners_px": [float(v) for v in d.corners_px],
                }
                for d in msg.detections
            ],
        }
        if msg.reset:
            self._cal_markers = []
        elif msg.accepted:
            self._cal_markers = (self._cal_markers + [payload])[-256:]
        self._last_marker_payload = payload
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
            "verify_delta_mm": float(msg.verify_delta_mm),
            "verify_message": msg.verify_message,
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
            "recording": self._recorder.status()["active"],
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

    def _on_workflows(self, msg: WorkflowList):
        payload = {
            "type": "workflows",
            "default": msg.default_workflow,
            "workflows": [
                {
                    "id": w.id,
                    "label": w.label,
                    "description": w.description,
                    "ready": bool(w.ready),
                    "reason": w.not_ready_reason,
                    "requires": w.requires_name,
                }
                for w in msg.workflows
            ],
        }
        self._last_workflows_payload = payload  # replayed to late-connecting browsers
        self.broadcast(payload)

    def _on_system_state(self, msg: SystemState):
        payload = {
            "type": "system_state",
            "capability": msg.capability_name,
            "capability_bits": int(msg.capability),
            "robot_state_fresh": bool(msg.robot_state_fresh),
            "connected": bool(msg.connected),
            "servo_on": bool(msg.servo_on),
            "alarming": bool(msg.alarming),
            "erroring": bool(msg.erroring),
            "in_hold": bool(msg.in_hold),
            "paused": bool(msg.paused),
            "busy": bool(msg.busy),
            "control_mode": msg.control_mode,
            "robot_state_age_s": float(msg.robot_state_age_s),
            "health_fresh": bool(msg.health_fresh),
            "camera_ok": bool(msg.camera_ok),
            "calibrated": bool(msg.calibrated),
            "depth_fresh": bool(msg.depth_fresh),
            "detector_ok": bool(msg.detector_ok),
            "health_age_s": float(msg.health_age_s),
            "move_blockers": list(msg.move_blockers),
            "see_blockers": list(msg.see_blockers),
        }
        self._last_system_state_payload = payload  # replayed to late-connecting browsers
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

    def _on_profile(self, msg: ControllerProfile):
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        payload = {
            "type": "tool_profile",
            "read_count": int(msg.read_count),
            "loaded": bool(msg.loaded),
            "source": msg.source,
            "robot_ip": msg.robot_ip,
            "read_at": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(stamp_s)) if stamp_s > 0 else "",
            "model_name": msg.model_name,
            "software_version": msg.software_version,
            "tools": [
                {
                    "number": int(t.number),
                    "name": t.name,
                    "x": float(t.x), "y": float(t.y), "z": float(t.z),
                    "rx": float(t.rx), "ry": float(t.ry), "rz": float(t.rz),
                    "weight_kg": float(t.weight_kg),
                    "cog_mm": [float(v) for v in t.cog_mm],
                    "is_identity": bool(t.is_identity),
                }
                for t in msg.tools
            ],
            "user_frames": [
                {
                    "number": int(f.number),
                    "name": f.name,
                    "tool_number": int(f.tool_number),
                    "group": int(f.group),
                    "pose": [float(v) for v in f.pose],
                    "defined": bool(f.defined),
                }
                for f in msg.user_frames
            ],
            "cnd_files": list(msg.cnd_files),
            "errors": list(msg.errors),
            "warnings": list(msg.warnings),
            "configured": [
                {"name": n, "tool_number": int(k)}
                for n, k in zip(msg.configured_tool_names, msg.configured_tool_numbers)
            ],
            "active_tool_number": int(msg.active_tool_number),
            "active_tool_name": msg.active_tool_name,
            "active_tool_source": msg.active_tool_source,
            "active_tool_verified": bool(msg.active_tool_verified),
        }
        self._last_profile_payload = payload  # replayed to late-connecting browsers
        self.broadcast(payload)

    def _on_cobot_state(self, msg: CobotState):
        self._last_cobot_state = msg  # 10 Hz — digested by the tick timers
        self._last_cobot_state_mono = time.monotonic()

    def _tick_cobot_state(self):
        """Cobot panel: everything the controller reports, at cobot_state_hz.

        Sent even when unchanged (the panel shows the sample age), unlike
        tool_state which is change-only.
        """
        msg = self._last_cobot_state
        if msg is None:
            return
        age = (
            None if self._last_cobot_state_mono is None
            else time.monotonic() - self._last_cobot_state_mono
        )
        payload = {
            "type": "cobot_state",
            "age_s": age,
            "connected": bool(msg.connected),
            "servo_on": bool(msg.servo_on),
            "running": bool(msg.running),
            "in_hold": bool(msg.in_hold),
            "alarming": bool(msg.alarming),
            "erroring": bool(msg.erroring),
            "control_mode": COBOT_CONTROL_MODES.get(int(msg.control_mode), "unknown"),
            "paused": bool(msg.paused),
            "busy": bool(msg.busy),
            "poll_ms": round(float(msg.poll_ms), 1),
            "active_tool_number": int(msg.active_tool_number),
            "active_tool_name": msg.active_tool_name,
            "pose_tool_number": int(msg.pose_tool_number),
            "pose": [round(float(v), 2) for v in (msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz)],
            "joints_deg": [round(float(v), 2) for v in msg.joints_deg],
            "joint_limits_deg": self._joint_limits,
            "sensors": {n: bool(v) for n, v in zip(msg.sensor_names, msg.sensor_values)},
            "outputs": {n: bool(v) for n, v in zip(msg.output_names, msg.output_values)},
            "alarms": [
                {"code": int(a.code), "subcode": int(a.subcode), "name": a.name, "time": a.time}
                for a in msg.alarms
            ],
        }
        self._last_cobot_state_payload = payload  # replayed to late-connecting browsers
        self.broadcast(payload)

    def _tick_tool_state(self):
        msg = self._last_cobot_state
        if msg is None:
            return
        payload = {
            "type": "tool_state",
            "connected": bool(msg.connected),
            "busy": bool(msg.busy),
            "active_tool_number": int(msg.active_tool_number),
            "active_tool_name": msg.active_tool_name,
            "pose_tool_number": int(msg.pose_tool_number),
            "profile_loaded": bool(msg.profile_loaded),
            "pose": [round(float(v), 1) for v in (msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz)],
            "joints_deg": [round(float(v), 2) for v in msg.joints_deg],
        }
        if payload == self._last_tool_state_payload:
            return  # change-only; the pose is rounded so idle ticks stay quiet
        self._last_tool_state_payload = payload
        self.broadcast(payload)

    def _on_manual_request(self, msg: Bool):
        # TEMPORARY manual centre select — the UI prompts while this is true.
        self._manual_waiting = bool(msg.data)  # replayed to late-connecting browsers
        self.broadcast({"type": "manual_dot_request", "waiting": self._manual_waiting})

    def _latched_snapshot(self) -> list[dict]:
        """The current latched/edge-triggered state, in the order a newly
        connected browser must receive it. Single source of truth for both the
        websocket connect replay and the recorder's t=0 snapshot (a recording
        armed mid-session must open with correct initial state)."""
        payloads = [
            p for p in (
                self._last_workflows_payload,
                self._last_system_state_payload,
                self._last_status_payload,
            ) if p is not None
        ]
        # Most important for the manual-click prompt: a calibration already
        # waiting must show it right away.
        payloads.append({"type": "manual_dot_request", "waiting": self._manual_waiting})
        payloads.extend(
            p for p in (
                self._last_health_payload,
                self._last_profile_payload,
                self._last_tool_state_payload,
                self._last_cobot_state_payload,
                self._last_detections_payload,
            ) if p is not None
        )
        # Calibration verification overlay: a reset, every accepted marker
        # of the run, then the latest result (which may be a rejection).
        if self._last_marker_payload is not None:
            payloads.append({"type": "calibration_marker", "reset": True})
            payloads.extend(list(self._cal_markers))
            if not self._last_marker_payload.get("accepted"):
                payloads.append(self._last_marker_payload)
        if self._last_targets_payload is not None:
            payloads.append(self._last_targets_payload)
        payloads.extend(list(self._pick_events))
        # Recorder status last — record() drops "recording" payloads, so this
        # reaches browsers but never the recording itself.
        payloads.append(self._recorder.status())
        return payloads

    def broadcast(self, payload: dict):
        """Thread-safe: callable from ROS callbacks."""
        # Recording tap first: capture regardless of ws_loop/client state.
        self._recorder.record(payload)
        if self.ws_loop is None:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast_async(payload), self.ws_loop)

    async def _broadcast_async(self, payload: dict, clients: list | None = None):
        """Send to every client in `clients` (default: all). Camera frames pass
        ws_frame_clients so frame-opted-out sockets (the dev panel) are skipped."""
        targets = self.ws_clients if clients is None else clients
        disconnected = []
        for ws in list(targets):
            try:
                await ws.send_json(payload)
            except Exception:  # noqa: BLE001
                disconnected.append(ws)
        for ws in disconnected:
            for pool in (self.ws_clients, self.ws_frame_clients):
                if ws in pool:
                    pool.remove(ws)

    # ------------------------------------------------------------------
    # Browser → ROS
    # ------------------------------------------------------------------

    async def handle_web_command(self, receiver: str, command: str, payload) -> dict:
        if receiver == "CAMERA" and command == "snapshot":
            # Frames stream continuously now; snapshot = push one immediately.
            frame = self._camera_frame_payload()
            self._recorder.record(frame)  # bypasses broadcast(); still decimated
            await self._broadcast_async(frame, self.ws_frame_clients)
            return {"result": "ok", "message": "frame pushed"}

        if receiver == "RECORD":
            # Session recorder: server-side, so it survives page refreshes and
            # records with zero browsers connected.
            if command == "start":
                result = self._recorder.start(self._latched_snapshot())
            elif command == "stop":
                result = self._recorder.stop()
            elif command == "status":
                return {**self._recorder.status(), "result": True}
            else:
                raise ValueError(f"Invalid RECORD command '{command}'")
            self.broadcast(self._recorder.status())  # flip the pill everywhere
            return result

        if receiver == "VISION" and command == "click_center":
            # TEMPORARY manual centre select — a topic, not a Trigger service.
            return self._publish_manual_click(payload)

        if receiver == "VISION" and command == "skip_center":
            # TEMPORARY manual centre select — drop this calibration pose.
            self._manual_skip_pub.publish(Empty())
            return {"result": "ok", "message": "pose skipped"}

        if receiver == "COBOT" and command == "start_workflow":
            workflow = payload.get("workflow", "") if isinstance(payload, dict) else ""
            if not self._workflow_client.service_is_ready():
                raise RuntimeError("service /task/start_workflow not available")
            result = await self._call_service(
                self._workflow_client, StartWorkflow.Request(workflow=str(workflow or ""))
            )
            return {"result": result.success, "message": result.message}

        if receiver == "COBOT" and command == "set_active_tool":
            name = payload.get("name", "") if isinstance(payload, dict) else ""
            if not name:
                raise ValueError('set_active_tool needs a payload of {"name": <tools key | none>}')
            if not self._set_tool_client.service_is_ready():
                raise RuntimeError("service /cobot/set_active_tool not available")
            req = SetActiveTool.Request(name=str(name), tool_number=-1, force=False)
            result = await self._call_service(self._set_tool_client, req)
            message = result.message
            if result.success:
                message += " — remembered positions are converted automatically (watch the pill)"
            return {
                "result": result.success,
                "message": message,
                "tool_number": int(result.tool_number),
                "name": result.name,
                "status_code": int(result.status_code),
            }

        if receiver == "COBOT" and command == "convert_trajectories":
            if not self._convert_client.service_is_ready():
                raise RuntimeError("service /cobot/convert_trajectories not available")
            result = await self._call_service(
                self._convert_client, ConvertTrajectories.Request(to_tool=-1), timeout=60.0
            )
            return {"result": result.success, "message": result.message,
                    "converted": int(result.converted), "to_tool": int(result.to_tool)}

        if receiver == "COBOT" and command == "teach_point":
            key = payload.get("key", "") if isinstance(payload, dict) else ""
            index = int(payload.get("index", 0)) if isinstance(payload, dict) else 0
            if not key:
                raise ValueError('teach_point needs a payload of {"key": str, "index": int}')
            if not self._teach_client.service_is_ready():
                raise RuntimeError("service /cobot/save_teach_point not available")
            result = await self._call_service(
                self._teach_client, SaveTeachPoint.Request(key=str(key), index=index)
            )
            return {"result": result.success, "message": result.message}

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
        return await self._call_service(client, Trigger.Request(), timeout)

    async def _call_service(self, client, request, timeout: float = 10.0):
        """Bridge an rclpy future (resolved on the executor thread) to asyncio."""
        loop = asyncio.get_running_loop()
        aio_future = loop.create_future()

        def _done(ros_future):
            try:
                loop.call_soon_threadsafe(aio_future.set_result, ros_future.result())
            except Exception as e:  # noqa: BLE001
                loop.call_soon_threadsafe(aio_future.set_exception, e)

        client.call_async(request).add_done_callback(_done)
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


# rcl's C YAML parser follows full YAML 1.1, where BARE y/n/yes/no/on/off are
# BOOLEANS and bare numerals are numbers — but PyYAML's safe_dump writes such
# strings unquoted (it doesn't treat y/n as bool itself). A round-tripped
# pathing_row_axis: "y" would come back as `y`, load as True, and kill the
# node at startup with a type mismatch. Quote every ambiguous string on dump.
_RCL_BOOL_WORDS = frozenset(
    variant
    for word in ("y", "n", "yes", "no", "true", "false", "on", "off")
    for variant in (word, word.upper(), word.capitalize())
)


class _RclSafeDumper(yaml.SafeDumper):
    """safe_dump whose str output always re-reads as a string under rcl."""


def _represent_str(dumper, value: str):
    ambiguous = value in _RCL_BOOL_WORDS
    if not ambiguous:
        try:
            float(value)  # "1", "1.5", "1e5", … would re-read as a number
            ambiguous = True
        except ValueError:
            pass
    return dumper.represent_scalar(
        "tag:yaml.org,2002:str", value, style='"' if ambiguous else None)


_RclSafeDumper.add_representer(str, _represent_str)


def _dump_yaml(config: dict, f) -> None:
    yaml.dump(config, f, Dumper=_RclSafeDumper, sort_keys=False)


def _coerce_like(old, new):
    """Keep the YAML type of an edited value stable: a float param edited to
    '15' must stay a float (set_parameters and some loaders are type-strict,
    e.g. calibration_z_levels_mm MUST be floats)."""
    if isinstance(old, bool):
        return bool(new)
    if isinstance(old, float) and isinstance(new, (int, float)) and not isinstance(new, bool):
        return float(new)
    if (
        isinstance(old, int) and not isinstance(old, bool)
        and isinstance(new, (int, float)) and not isinstance(new, bool)
        and float(new).is_integer()
    ):
        return int(new)
    if isinstance(old, list) and isinstance(new, list):
        if old and all(isinstance(v, float) for v in old):
            return [float(v) for v in new]
        if old and all(isinstance(v, int) and not isinstance(v, bool) for v in old):
            return [int(v) for v in new]
    return new


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
    elif (
        isinstance(value, list) and value
        and all(isinstance(v, int) and not isinstance(v, bool) for v in value)
    ):
        # int lists stay integer arrays — nodes declare e.g. calibration_grid_size
        # as INTEGER_ARRAY and would reject a double array (rclpy static typing).
        pv.type = ParameterType.PARAMETER_INTEGER_ARRAY
        pv.integer_array_value = list(value)
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
                _dump_yaml(config, f)
            outcomes = node.apply_config(config)

        return {
            "status": "saved",
            "saved_to": str(node.params_file),
            "applied": outcomes,
            "config": config,
        }

    # ------------------------------------------------------------------
    # Developer dashboard (/dev): per-param saves + trajectories editing.
    # ------------------------------------------------------------------

    @app.get("/dev", response_class=HTMLResponse)
    def dev(request: Request):
        return templates.TemplateResponse(request=request, name="dev.html")

    @app.post("/api/param")
    def save_param(body: dict):
        """Save ONE parameter of ONE node to cobot_params.yaml and live-apply
        it via /{node}/set_parameters. body = {node, name, value}."""
        node_name = str(body.get("node", ""))
        name = str(body.get("name", ""))
        if not node_name or not name or "value" not in body:
            return {"status": "error", "message": "need {node, name, value}"}
        if not node.params_file:
            return {"status": "error", "message": "web_node has no params_file"}
        with open(node.params_file, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
        section = config.setdefault(node_name, {}).setdefault("ros__parameters", {})
        value = _coerce_like(section.get(name), body["value"])
        section[name] = value
        with open(node.params_file, "w", encoding="utf-8") as f:
            _dump_yaml(config, f)
        applied = node._set_node_parameters(node_name, {name: value})
        return {
            "status": "saved",
            "saved_to": str(node.params_file),
            "node": node_name,
            "name": name,
            "value": value,
            "applied": applied,
        }

    @app.get("/api/setup")
    def get_setup():
        if not node.setup_file or not Path(node.setup_file).exists():
            return {}
        with open(node.setup_file, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    @app.post("/api/setup-param")
    def save_setup_param(body: dict):
        """Save ONE value of cobot_setup.yaml. body = {path: [keys...], value}.
        cobot_control / cobot_operation load this file at startup — the save
        persists to disk and applies on the next restart."""
        path = body.get("path")
        if not isinstance(path, list) or not path or "value" not in body:
            return {"status": "error", "message": "need {path: [keys...], value}"}
        if not node.setup_file:
            return {"status": "error", "message": "web_node has no setup_file"}
        with open(node.setup_file, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
        cursor = config
        for key in path[:-1]:
            cursor = cursor.get(str(key)) if isinstance(cursor, dict) else None
            if not isinstance(cursor, dict):
                return {"status": "error", "message": f"unknown setup path {path}"}
        leaf = str(path[-1])
        if leaf not in cursor:
            return {"status": "error", "message": f"unknown setup key {path}"}
        cursor[leaf] = _coerce_like(cursor.get(leaf), body["value"])
        with open(node.setup_file, "w", encoding="utf-8") as f:
            _dump_yaml(config, f)
        return {
            "status": "saved",
            "saved_to": str(node.setup_file),
            "value": cursor[leaf],
            "applied": "saved to file — loaded at startup, restart cobot nodes to apply",
        }

    @app.get("/api/trajectories")
    def get_trajectories():
        path = node.trajectories_file
        if not path or not Path(path).exists():
            return {"path": str(path or ""), "exists": False, "points": {}}
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        return {
            "path": str(path),
            "exists": True,
            "version": raw.get("version"),
            "updated_at": raw.get("updated_at"),
            "points": raw.get("points", {}),
        }

    @app.post("/api/trajectories/capture")
    async def capture_teach_point(body: dict):
        """Capture the CURRENT robot pose + joints into points.<key>[index]
        via cobot_operation (/cobot/save_teach_point) — saved to disk there."""
        key = str(body.get("key", ""))
        index = int(body.get("index", 0))
        if not key:
            return {"success": False, "message": "need {key, index}"}
        if not node._teach_client.service_is_ready():
            return {"success": False, "message": "service /cobot/save_teach_point not available"}
        result = await node._call_service(
            node._teach_client, SaveTeachPoint.Request(key=key, index=index)
        )
        return {
            "success": bool(result.success),
            "message": result.message,
            "tool_number": int(result.tool_number),
            "pose": [result.x, result.y, result.z, result.rx, result.ry, result.rz],
        }

    @app.post("/api/trajectories/save")
    async def save_teach_point_edit(body: dict):
        """Save a HAND-EDITED pose into points.<key>[index] via
        cobot_operation (/cobot/set_teach_point) — saved to disk there."""
        try:
            req = SetTeachPoint.Request(
                key=str(body.get("key", "")),
                index=int(body.get("index", 0)),
                x=float(body["x"]), y=float(body["y"]), z=float(body["z"]),
                rx=float(body["rx"]), ry=float(body["ry"]), rz=float(body["rz"]),
                tool_number=int(body.get("tool_number", -1)),
            )
        except (KeyError, TypeError, ValueError) as e:
            return {"success": False, "message": f"bad payload: {e}"}
        if not req.key:
            return {"success": False, "message": "need a key"}
        if not node._set_point_client.service_is_ready():
            return {"success": False, "message": "service /cobot/set_teach_point not available"}
        result = await node._call_service(node._set_point_client, req)
        return {
            "success": bool(result.success),
            "message": result.message,
            "tool_number": int(result.tool_number),
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
        # /ws?frames=off (the dev panel) never receives camera frames — the
        # heavy payloads stay off that socket entirely, server-side.
        wants_frames = websocket.query_params.get("frames", "on").lower() not in (
            "off", "0", "false", "no")
        if wants_frames:
            node.ws_frame_clients.append(websocket)
        await websocket.send_json({"type": "connection", "message": "WebSocket connected"})

        # Replay the current state so a page opened or refreshed mid-operation
        # is immediately correct — otherwise it would sit blank until the next
        # change. Same list the recorder writes as its t=0 snapshot.
        for payload in node._latched_snapshot():
            await websocket.send_json(payload)

        try:
            while True:
                data = await websocket.receive_json()

                receiver = data.get("receiver")
                command = data.get("command")
                payload = data.get("payload")

                # Audit trail: inbound commands land in the recording as
                # "_recorded_command" (harmless at replay — falls through to
                # the dispatch default). RECORD commands are excluded so a
                # recording does not end with its own stop machinery.
                if receiver != "RECORD":
                    node._recorder.record({
                        "type": "_recorded_command",
                        "t_wall": time.time(),
                        "receiver": receiver,
                        "command": command,
                        "payload": payload,
                    })

                try:
                    result = await node.handle_web_command(receiver, command, payload)
                except Exception as e:  # noqa: BLE001
                    node.get_logger().error(f"web command failed: {e}")
                    result = {"result": str(e), "message": "websocket command failed"}

                result_msg = {"type": "command_result", "command": command, "result": result}
                if receiver != "RECORD":
                    # Reproduces what the operator saw in the status lines.
                    node._recorder.record(result_msg)
                await websocket.send_json(result_msg)

        except WebSocketDisconnect:
            pass
        finally:
            for pool in (node.ws_clients, node.ws_frame_clients):
                if websocket in pool:
                    pool.remove(websocket)

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
