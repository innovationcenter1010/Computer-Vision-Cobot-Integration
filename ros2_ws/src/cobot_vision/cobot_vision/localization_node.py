"""localization_node — pure geometry. No ML, no robot commands.

Owns the ONE "look at the scene" entry point of the vision stack and turns
pixel-space detections into robot-space targets using the fitted camera→robot
calibration (see calibration.py):

  action /vision/scan (Scan)     — goal model_id ("" → default_model_id):
                                   1. call /vision/detect on cobot_detector
                                   2. match the color stamp to a depth frame
                                   3. px + depth → robot base xyz, rz from corners
                                   4. snake-order → result.targets, also
                                      mirrored on /vision/targets
  in:  /camera/depth/image_raw (Image, 16SC1 mm)  — ring buffer, stamp-matched
       /camera/color/camera_info (CameraInfo)      — camera-SDK intrinsics
  out: /vision/targets (TargetArray)              — ordered pick layer;
                                                     consumed by task_manager + web
  uses /vision/detect (Detect action, cobot_detector) — the only ML in the graph

NO CALIBRATION, NO TARGETS: if calibration_file is missing the node refuses to
localize (detections are dropped, verify_target errors) until a calibration
run has completed. There is deliberately no fallback: an unmeasured transform
must never move the robot.

Spatial setup data (can height) comes from cobot_params.yaml — parameter
defaults here are deliberately zeros, and zero-valued spatial params mean
"unconfigured": the dependent feature refuses to run instead of guessing.
The red-dot marker offset is PER TOOL FILE: cobot_setup.yaml
tools.<name>.marker_offset_mm (setup_file param), selected by the ACTIVE tool
cobot_control reports on /cobot/state. calibration_begin snapshots that tool,
every sample refuses if it changes mid-run, and the fitted calibration.yaml
records it (tool_number / tool_name) so a later mismatch can be flagged.
Nothing here depends on bin-corner coordinates: pick order is a robot-frame
snake (pathing.py) and layer height is sampled at the live detection pixels,
so both stay correct when the bin moves.

Calibration split: task_manager drives the routine (/task/calibrate) by
sending MoveToPose goals to cobot_node; this node owns every millimeter of
math. Between poses the task manager calls:

  /vision/calibration_begin    — clear the sample buffer, warm the dot model
  /vision/calibration_sample   — "marker parked at TCP (x,y,z), measure now":
                                 locate the dot (/vision/detect with
                                 dot_model_id, best-confidence box centre),
                                 sample own depth buffer, add marker offset,
                                 store the matched pair. With
                                 manual_dot_selection:=true the dot model is
                                 bypassed and (u,v) comes from an operator
                                 click on the web UI instead — see below.
  /vision/calibration_compute  — fit the 3x4 transform, validate RMS, persist
                                 to calibration_file, hot-swap the live model.
                                 With camera intrinsics known (CameraInfo) this
                                 is the small extrinsic-only (Kabsch) fit; with
                                 require_intrinsics:=false and no intrinsics it
                                 falls back to the full monolithic fit.

TEMPORARY manual centre select (manual_dot_selection):
  out: /vision/manual_dot_request (Bool, latched) — true while input is wanted
  in:  /vision/manual_dot_click (PointStamped)    — point.x/y = COLOR image px
       /vision/manual_dot_skip  (Empty)           — drop this pose (marker not
                                                     in frame), no wait for the
                                                     timeout
Only the source of (u,v) changes; depth sampling, the marker offset and the
fit are identical either way. Delete these topics once the dot model is trusted.

Tier-1 depth services (no ML):
  /vision/verify_target    — is a can still at this target's position?
  /vision/get_layer_height — median depth at the last scan's detection pixels
                             → layer index
"""

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger

from cobot_interfaces.action import Detect, Scan
from cobot_interfaces.msg import CalibrationMarker, Heartbeat, ObbDetectionArray, Target, TargetArray
from cobot_interfaces.msg import CobotState
from cobot_interfaces.srv import (
    CalibrationBegin,
    CalibrationSample,
    GetLayerHeight,
    VerifyTarget,
)

from cobot_health.heartbeat import HeartbeatPublisher
from cobot_vision.calibration import CalibrationError, CameraRobotCalibration
from cobot_vision.tooling import load_marker_offsets, select_marker_offset
from cobot_vision.pathing import order_detections_snake
from cobot_vision.rotation import calculate_real_rz
from cobot_vision.verify import classify_presence


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Match camera_node's latched CameraInfo so we get the last value on subscribe.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# The first detect with the dot model lazy-loads it inside cobot_detector
# (handled in _begin); per-sample calls after that are one inference.
DOT_WARMUP_TIMEOUT_SEC = 110.0
DOT_TIMEOUT_SEC = 20.0


@dataclass
class _DotHit:
    """Best marker detection of one /vision/detect run (or found=False)."""

    found: bool = False
    u: float = 0.0
    v: float = 0.0
    confidence: float = 0.0
    message: str = ""
    detections: object | None = None  # raw ObbDetectionArray of the read


class _DetectError(RuntimeError):
    """/vision/detect rejected, aborted, canceled, or timed out."""


@dataclass
class _Target:
    """Internal working object; duck-types the attrs pathing.py expects."""

    detection_index: int
    class_id: int
    confidence: float
    center_x: float
    center_y: float
    depth_mm: float = -1.0
    real_x: float | None = None
    real_y: float | None = None
    real_z: float | None = None
    real_rz: float | None = None
    path_across_mm: float = field(default=0.0)
    path_down_mm: float = field(default=0.0)


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization_node")

        # ---- pick ordering: robot-frame snake, no bin geometry ----
        # Rows are grouped along this robot axis ("x" or "y"); the arm
        # serpentines along the other axis within each row. No bin corners:
        # ordering must survive the bin shifting.
        self.declare_parameter("pathing_row_axis", "y")
        self.declare_parameter("pathing_row_tolerance_mm", 50.0)

        self.declare_parameter("can_height_mm", 0.0)

        # ---- depth sampling ----
        self.declare_parameter("depth_buffer_size", 15)
        self.declare_parameter("depth_sample_window_px", 10)
        # |measured - expected| below this → target still present (raw depth mm)
        self.declare_parameter("verify_tolerance_mm", 40.0)
        # verify_target refuses a depth frame older than this (a frozen camera
        # must not confirm picks). 0 = no age check.
        self.declare_parameter("verify_max_depth_age_s", 2.0)
        # Depth reading (mm) when the TOP layer is present. 0.0 = uncalibrated;
        # get_layer_height then returns layer_index = -1. Calibrate on hardware.
        self.declare_parameter("depth_at_top_layer_mm", 0.0)

        # ---- camera→robot calibration (math side; the pose grid that drives
        # the robot lives in task_manager) ----
        # Where the fitted transform lives; injected by robot.launch.py into
        # the bind-mounted config dir so it survives container rebuilds.
        self.declare_parameter("calibration_file", "")
        # cobot_setup.yaml — the per-tool marker offsets (tools.<name>.
        # marker_offset_mm: red dot relative to THAT tool file's TCP, robot
        # base axes at the calibration orientation). Injected by launch.
        self.declare_parameter("setup_file", "")
        self.declare_parameter("dot_samples_per_pose", 3)
        self.declare_parameter("calibration_min_samples", 8)
        self.declare_parameter("calibration_max_rms_mm", 5.0)
        # Intrinsics come from the camera SDK via /camera/color/camera_info.
        # True (default): calibration is the small extrinsic-only (Kabsch) fit
        # and refuses to run until intrinsics arrive. False: fall back to the
        # full monolithic fit that solves intrinsics + extrinsics together
        # (needs the wide depth-span grid).
        self.declare_parameter("require_intrinsics", True)

        # ---- TEMPORARY: manual centre select ----
        # True: calibration_sample does NOT run the red-dot model. It publishes
        # /vision/manual_dot_request and blocks until the operator clicks the
        # marker centre on the web UI's color feed (/vision/manual_dot_click).
        # Everything downstream (depth sampling, marker offset, the fit) is
        # unchanged — only where (u, v) comes from differs. Remove once the
        # red-dot model is trusted.
        self.declare_parameter("manual_dot_selection", False)
        self.declare_parameter("manual_click_timeout_sec", 120.0)

        # ---- detector hand-off (models.yaml ids served by cobot_detector) ----
        # Scan goals with an empty model_id use default_model_id; the
        # calibration routine always uses dot_model_id.
        self.declare_parameter("default_model_id", "cans")
        self.declare_parameter("dot_model_id", "red_dot")
        # Upper bound for one /vision/detect round trip (includes a lazy model
        # load on first use).
        self.declare_parameter("detect_timeout_sec", 60.0)

        # One event, two outcomes: a click carries (u, v); a skip carries the
        # SKIP sentinel (operator says the marker isn't in frame → drop this
        # pose without a sample, don't wait out the timeout).
        self._manual_lock = threading.Lock()
        self._manual_result: tuple[float, float] | str | None = None
        self._manual_event = threading.Event()

        # Latest camera intrinsics {fx, fy, cx, cy} or None until CameraInfo.
        self._intrinsics_lock = threading.Lock()
        self._intrinsics: dict | None = None

        self._calibration: CameraRobotCalibration | None = None
        self._load_calibration()

        # Per-tool marker offsets + the active tool (from /cobot/state).
        self._marker_offsets: dict = {}
        self._load_marker_offsets()
        self._tool_lock = threading.Lock()
        self._active_tool_sample: tuple[int, str, float] | None = None  # (number, name, monotonic)
        self._cal_tool: tuple[int, str] | None = None  # snapshot taken by calibration_begin
        self.create_subscription(CobotState, "/cobot/state", self._on_cobot_state, 1)

        self._depth_lock = threading.Lock()
        self._depth_buffer: deque = deque(
            maxlen=int(self.get_parameter("depth_buffer_size").value)
        )

        # Pixel centers of the most recent scan's detections — get_layer_height
        # samples the live depth frame here so the height check follows the
        # cans (and the bin) instead of a fixed image region.
        self._detection_px_lock = threading.Lock()
        self._last_detection_px: list[tuple[float, float]] = []

        # Calibration sample buffer, filled between begin and compute.
        self._samples_uvd: list[list[float]] = []
        self._samples_xyz: list[list[float]] = []

        self.create_subscription(
            Image, "/camera/depth/image_raw", self._on_depth, SENSOR_QOS
        )
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self._on_camera_info, LATCHED_QOS
        )

        self._targets_pub = self.create_publisher(TargetArray, "/vision/targets", 10)

        # Detector client + Scan server. Both live in a reentrant group: the
        # Scan execute callback blocks on a Detect future that the executor
        # must resolve on another thread (MultiThreadedExecutor in main()).
        self._detect_group = ReentrantCallbackGroup()
        self._detect_client = ActionClient(
            self, Detect, "/vision/detect", callback_group=self._detect_group
        )
        self._scan_lock = threading.Lock()  # one scan at a time
        self._scan_in_flight = False
        self._last_scan_model_id: str | None = None
        self._scan_server = ActionServer(
            self,
            Scan,
            "/vision/scan",
            execute_callback=self._execute_scan,
            goal_callback=self._on_scan_goal,
            cancel_callback=self._on_scan_cancel,
            callback_group=self._detect_group,
        )

        # Manual centre select (temporary): latched request so a browser that
        # connects mid-pose still sees that a click is wanted.
        self._manual_request_pub = self.create_publisher(
            Bool, "/vision/manual_dot_request", LATCHED_QOS
        )
        self._manual_request_active = bool(False)
        self._manual_request_pub.publish(Bool(data=False))
        self.create_subscription(
            PointStamped, "/vision/manual_dot_click", self._on_manual_click, 10
        )
        self.create_subscription(
            Empty, "/vision/manual_dot_skip", self._on_manual_skip, 10
        )

        self.create_service(VerifyTarget, "/vision/verify_target", self._on_verify_target)
        self.create_service(
            GetLayerHeight, "/vision/get_layer_height", self._on_get_layer_height
        )

        # Calibration services share one group (serialized among themselves,
        # a sample can't race a compute) but must not block the default group:
        # their handlers wait on /vision/detect futures that the executor
        # resolves on other threads (MultiThreadedExecutor in main()).
        cal_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            CalibrationBegin, "/vision/calibration_begin", self._on_cal_begin,
            callback_group=cal_group,
        )
        # Per-run marker-finding mode, pinned by calibration_begin (None =
        # legacy: the manual_dot_selection parameter decides).
        self._cal_manual: bool | None = None
        self._cal_model_id = ""
        self._cal_class_id = -1
        self._cal_reads = 0
        # Every sample (accepted or not) is echoed here for the dashboard's
        # verification overlay; latched so a page opened mid-run sees the last.
        self._marker_pub = self.create_publisher(
            CalibrationMarker, "/vision/calibration_marker", LATCHED_QOS
        )
        self.create_service(
            CalibrationSample, "/vision/calibration_sample", self._on_cal_sample,
            callback_group=cal_group,
        )
        self.create_service(
            Trigger, "/vision/calibration_compute", self._on_cal_compute,
            callback_group=cal_group,
        )
        # Health (heartbeat timer runs in its OWN callback group — never in
        # cal_group, whose handlers block for up to 120 s).
        self._last_targets_count: int | None = None
        self._hb = HeartbeatPublisher(self, self._health_report)

        self.get_logger().info("localization_node ready")

    # ------------------------------------------------------------------
    # Calibration state
    # ------------------------------------------------------------------

    def _load_calibration(self):
        path = self.get_parameter("calibration_file").value
        if not path:
            self.get_logger().error(
                "calibration_file parameter is empty — cannot persist or load "
                "a calibration (robot.launch.py normally injects it)"
            )
            return
        if not Path(path).exists():
            self.get_logger().warn(
                f"NOT CALIBRATED — {path} does not exist. /vision/targets stays "
                "silent until a calibration run completes (/task/calibrate)."
            )
            return
        try:
            self._calibration = CameraRobotCalibration.load(path)
        except CalibrationError as e:
            self.get_logger().error(f"NOT CALIBRATED — {e}")
            return
        # Seed intrinsics from the saved block so a startup extrinsic fit can
        # run even before CameraInfo arrives (the live topic overrides later).
        if self._calibration.intrinsics:
            with self._intrinsics_lock:
                self._intrinsics = dict(self._calibration.intrinsics)
        rms = self._calibration.rms_mm
        self.get_logger().info(
            f"calibration loaded ({self._calibration.created}, source="
            f"{self._calibration.source}, {self._calibration.sample_count} samples, "
            f"rms x={rms[0]:.1f} y={rms[1]:.1f} z={rms[2]:.1f} mm)"
        )

    def _load_marker_offsets(self):
        path = self.get_parameter("setup_file").value
        if not path:
            self.get_logger().error(
                "setup_file parameter is empty — no per-tool marker offsets; calibration "
                "will refuse to start (robot.launch.py normally injects it)"
            )
            return
        try:
            self._marker_offsets = load_marker_offsets(path)
        except (OSError, ValueError) as e:
            self.get_logger().error(f"cannot load tools from {path}: {e}")
            return
        self.get_logger().info(
            "marker offsets: " + ", ".join(
                f"{m.name}(tool {n})={list(m.offset_mm)}" for n, m in sorted(self._marker_offsets.items())
            )
        )

    def _on_cobot_state(self, msg: CobotState):
        with self._tool_lock:
            self._active_tool_sample = (
                int(msg.active_tool_number), str(msg.active_tool_name), time.monotonic()
            )

    def _active_tool(self, max_age_s: float = 1.5) -> tuple[int, str] | None:
        """(tool_number, name) from a fresh /cobot/state, else None."""
        with self._tool_lock:
            sample = self._active_tool_sample
        if sample is None or time.monotonic() - sample[2] > max_age_s:
            return None
        return sample[0], sample[1]

    def _on_camera_info(self, msg: CameraInfo):
        """Cache intrinsics from the camera SDK (relayed by camera_node)."""
        fx, cx, fy, cy = msg.k[0], msg.k[2], msg.k[4], msg.k[5]
        if fx == 0.0 or fy == 0.0:
            return  # placeholder/empty CameraInfo — ignore
        intrinsics = {"fx": float(fx), "fy": float(fy), "cx": float(cx), "cy": float(cy)}
        with self._intrinsics_lock:
            first = self._intrinsics is None
            self._intrinsics = intrinsics
        if first:
            self.get_logger().info(
                f"camera intrinsics received: fx={fx:.1f} fy={fy:.1f} "
                f"cx={cx:.1f} cy={cy:.1f}"
            )

    # ------------------------------------------------------------------
    # Depth handling
    # ------------------------------------------------------------------

    def _on_depth(self, msg: Image):
        depth = np.frombuffer(msg.data, dtype=np.int16).reshape(msg.height, msg.width)
        with self._depth_lock:
            self._depth_buffer.append((_stamp_to_sec(msg.header.stamp), depth))

    def _depth_frame_near(self, stamp) -> np.ndarray | None:
        """Depth frame closest in time to the given stamp (or latest if none)."""
        with self._depth_lock:
            if not self._depth_buffer:
                return None
            if stamp is None:
                return self._depth_buffer[-1][1]
            t = _stamp_to_sec(stamp)
            return min(self._depth_buffer, key=lambda item: abs(item[0] - t))[1]

    def _sample_depth_mm(self, depth: np.ndarray, px: float, py: float) -> float | None:
        """Median depth (mm) in a small window around the pixel; None if invalid."""
        window = int(self.get_parameter("depth_sample_window_px").value)
        h, w = depth.shape
        x, y = int(round(px)), int(round(py))
        if not (0 <= x < w and 0 <= y < h):
            return None

        x0, x1 = max(0, x - window), min(w, x + window + 1)
        y0, y1 = max(0, y - window), min(h, y + window + 1)
        patch = depth[y0:y1, x0:x1]
        valid = patch[patch > 0]  # 0 = no reading
        if valid.size == 0:
            return None
        return float(np.median(valid))

    # ------------------------------------------------------------------
    # Detector hand-off
    # ------------------------------------------------------------------

    def _call_detect(self, model_id: str, timeout: float, image_source: str = "") -> Detect.Result:
        """Blocking /vision/detect round trip from a handler thread. Other
        executor threads resolve the goal/result futures (everything here
        lives in reentrant groups). Raises _DetectError on any non-success.
        image_source != "" makes the detector run on that file instead of
        its cached camera frame."""
        if not self._detect_client.server_is_ready():
            raise _DetectError("/vision/detect not available (is detector_node up?)")

        goal = Detect.Goal()
        goal.model_id = model_id
        goal.image_source = image_source
        deadline = time.monotonic() + timeout

        sent = threading.Event()
        goal_future = self._detect_client.send_goal_async(goal)
        goal_future.add_done_callback(lambda _: sent.set())
        if not sent.wait(timeout=max(0.0, deadline - time.monotonic())):
            raise _DetectError(f"/vision/detect did not answer the goal within {timeout:.0f}s")
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise _DetectError(
                f"/vision/detect rejected model '{model_id}' "
                "(unknown id, no camera frame, missing image_source, or a "
                "detection already running)"
            )

        done = threading.Event()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=max(0.0, deadline - time.monotonic())):
            goal_handle.cancel_goal_async()
            raise _DetectError(f"/vision/detect '{model_id}' timed out after {timeout:.0f}s")

        wrapped = result_future.result()
        result = wrapped.result
        if not result.success:
            raise _DetectError(f"/vision/detect '{model_id}' failed: {result.message}")
        return result

    # ------------------------------------------------------------------
    # Scan action: detect → localize → ordered targets
    # ------------------------------------------------------------------

    def _on_scan_goal(self, goal_request):
        if goal_request.localize:
            # Only a localized scan needs the geometry. A detect-only scan
            # (localize=false: bench tests, dataset sweeps) needs the detector.
            if self._calibration is None:
                self.get_logger().error("scan rejected: NOT CALIBRATED — run /task/calibrate")
                return GoalResponse.REJECT
            if self._depth_frame_near(None) is None:
                self.get_logger().warn("scan rejected: no depth frames yet")
                return GoalResponse.REJECT
        if not self._detect_client.server_is_ready():
            self.get_logger().warn("scan rejected: /vision/detect not available")
            return GoalResponse.REJECT
        if self._scan_in_flight:
            self.get_logger().warn("scan rejected: a scan is already running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_scan_cancel(self, goal_handle):
        # A scan is one detect round trip plus microseconds of math; the
        # detect goal is canceled downstream on timeout only. Accept so the
        # caller isn't left hanging, and report canceled after the detect.
        return CancelResponse.ACCEPT

    def _execute_scan(self, goal_handle):
        result = Scan.Result()
        if not self._scan_lock.acquire(blocking=False):
            result.success = False
            result.message = "a scan is already running"
            goal_handle.abort()
            return result
        self._scan_in_flight = True
        try:
            request = goal_handle.request
            model_id = request.model_id or str(
                self.get_parameter("default_model_id").value
            )
            image_source = str(request.image_source)
            localize = bool(request.localize)
            timeout = float(self.get_parameter("detect_timeout_sec").value)

            feedback = Scan.Feedback()
            feedback.stage = "detecting"
            goal_handle.publish_feedback(feedback)
            try:
                detect = self._call_detect(model_id, timeout, image_source)
            except _DetectError as e:
                result.success = False
                result.message = str(e)
                self.get_logger().error(f"scan failed: {result.message}")
                goal_handle.abort()
                return result

            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = "scan canceled"
                goal_handle.canceled()
                return result

            n_detections = len(detect.detections.detections)
            result.detection_count = n_detections
            if not localize:
                # Detect-only scan: no geometry, and nothing on /vision/targets
                # (that topic and the heartbeat counters describe LIVE scans).
                self._last_scan_model_id = model_id
                self.get_logger().info(
                    f"scan '{model_id}' (detect only"
                    f"{', ' + image_source if image_source else ''}): "
                    f"{n_detections} detections (inference {detect.inference_ms:.0f} ms)"
                )
                result.success = True
                result.message = f"{n_detections} detection(s) (localization skipped)"
                result.targets.header = detect.detections.header
                result.targets.model_id = detect.detections.model_id
                goal_handle.succeed()
                return result

            feedback.stage = "localizing"
            goal_handle.publish_feedback(feedback)
            targets, why = self._localize(detect.detections)
            if targets is None:
                result.success = False
                result.message = why
                self.get_logger().error(f"scan failed: {why}")
                goal_handle.abort()
                return result

            self._targets_pub.publish(targets)
            self._last_targets_count = len(targets.targets)
            self._last_scan_model_id = model_id
            self.get_logger().info(
                f"scan '{model_id}': {len(detect.detections.detections)} detections → "
                f"{len(targets.targets)} ordered targets "
                f"(inference {detect.inference_ms:.0f} ms)"
            )
            result.success = True
            result.message = f"{len(targets.targets)} target(s)"
            result.targets = targets
            goal_handle.succeed()
            return result
        finally:
            self._scan_in_flight = False
            self._scan_lock.release()

    # ------------------------------------------------------------------
    # Detections → targets (the transform math; unchanged)
    # ------------------------------------------------------------------

    def _localize(self, msg: ObbDetectionArray) -> tuple[TargetArray | None, str]:
        """Pixel detections + stamp-matched depth → ordered robot-frame targets.
        Returns (None, reason) instead of raising so the Scan action can
        report the reason verbatim."""
        calibration = self._calibration
        if calibration is None:
            return None, "NOT CALIBRATED — run /task/calibrate"

        depth = self._depth_frame_near(msg.header.stamp)
        if depth is None:
            return None, "no depth frame yet"

        targets: list[_Target] = []

        for det_index, det in enumerate(msg.detections):
            d = self._sample_depth_mm(depth, det.center_x, det.center_y)
            if d is None:
                self.get_logger().warn(
                    f"skipping detection at px ({det.center_x:.0f}, "
                    f"{det.center_y:.0f}): no valid depth"
                )
                continue

            target = _Target(
                detection_index=det_index,
                class_id=det.class_id,
                confidence=det.confidence,
                center_x=det.center_x,
                center_y=det.center_y,
                depth_mm=d,
            )

            x, y, z = calibration.transform(det.center_x, det.center_y, d)
            target.real_x = x
            target.real_y = y
            target.real_z = z

            # rz from the OBB left edge: corners are TL,TR,BR,BL → TL=0, BL=3.
            # The can top is planar, so both corners share the center's depth.
            corners = det.corners_px
            tl_x, tl_y, _ = calibration.transform(corners[0], corners[1], d)
            bl_x, bl_y, _ = calibration.transform(corners[6], corners[7], d)
            target.real_rz = float(
                calculate_real_rz(real_TL=[tl_x, tl_y], real_BL=[bl_x, bl_y])
            )

            targets.append(target)

        ordered = order_detections_snake(
            detections=targets,
            row_axis=self.get_parameter("pathing_row_axis").value,
            row_tolerance_mm=self.get_parameter("pathing_row_tolerance_mm").value,
        )

        # Remember where the cans were seen so get_layer_height can re-sample
        # the live depth frame at those pixels (follows the bin if it moves).
        with self._detection_px_lock:
            self._last_detection_px = [(t.center_x, t.center_y) for t in ordered]

        out = TargetArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "robot_base"
        out.model_id = msg.model_id
        for target in ordered:
            t = Target()
            t.detection_index = int(target.detection_index)
            t.class_id = target.class_id
            t.confidence = target.confidence
            t.real_x = target.real_x
            t.real_y = target.real_y
            t.real_z = target.real_z
            t.real_rz = target.real_rz
            t.pixel_x = float(target.center_x)
            t.pixel_y = float(target.center_y)
            t.depth_mm = float(target.depth_mm)
            out.targets.append(t)

        return out, ""

    # ------------------------------------------------------------------
    # Health
    # ------------------------------------------------------------------

    def _health_report(self):
        calibration = self._calibration  # replaced wholesale on compute; read once
        with self._depth_lock:
            depth_len = len(self._depth_buffer)
            depth_stamp = self._depth_buffer[-1][0] if self._depth_buffer else None
        with self._intrinsics_lock:
            intrinsics_ok = self._intrinsics is not None
        now_s = self.get_clock().now().nanoseconds * 1e-9
        depth_age = None if depth_stamp is None else max(0.0, now_s - depth_stamp)
        require_intrinsics = bool(self.get_parameter("require_intrinsics").value)

        problems = []
        level = Heartbeat.LEVEL_OK
        if calibration is None:
            level = Heartbeat.LEVEL_ERROR
            problems.append("NOT CALIBRATED")
        if depth_age is None:
            level = max(level, Heartbeat.LEVEL_WARN)
            problems.append("no depth frames yet")
        elif depth_age > 2.0:
            level = max(level, Heartbeat.LEVEL_WARN)
            problems.append(f"depth stale {depth_age:.1f}s")
        if require_intrinsics and not intrinsics_ok:
            level = max(level, Heartbeat.LEVEL_WARN)
            problems.append("no intrinsics")

        rms = calibration.rms_mm if calibration is not None else (None, None, None)
        if problems:
            detail = " · ".join(problems)
        else:
            detail = f"calibrated · rms {rms[0]:.1f} / {rms[1]:.1f} / {rms[2]:.1f} mm"

        return level, detail, {
            "calibrated": calibration is not None,
            "rms_x_mm": rms[0],
            "rms_y_mm": rms[1],
            "rms_z_mm": rms[2],
            "calibration_source": None if calibration is None else calibration.source,
            "calibration_samples": None if calibration is None else calibration.sample_count,
            "calibration_tool_number": (
                None if calibration is None or calibration.tool_number is None
                else calibration.tool_number
            ),
            "calibration_tool_name": None if calibration is None else calibration.tool_name,
            "marker_offset_tools": ",".join(str(n) for n in sorted(self._marker_offsets)),
            "intrinsics_ok": intrinsics_ok,
            "depth_buffer_len": depth_len,
            "depth_buffer_age_s": depth_age,
            "last_targets_count": self._last_targets_count,
            "last_scan_model_id": self._last_scan_model_id,
            "scan_in_flight": self._scan_in_flight,
            "detector_ready": self._detect_client.server_is_ready(),
            "manual_click_waiting": self._manual_event is not None and not self._manual_event.is_set() and self._manual_result is None and self._manual_waiting(),
        }

    def _manual_waiting(self) -> bool:
        # True while a calibration sample is blocked on an operator click.
        return bool(getattr(self, "_manual_request_active", False))

    # ------------------------------------------------------------------
    # Tier-1 depth services
    # ------------------------------------------------------------------

    def _on_verify_target(self, request, response):
        """Post-place check (task manager, once the arm is clear of the
        camera): re-sample the live depth at the pixel the target was seen at
        and compare with the depth measured there at scan time. Every path
        that cannot measure returns success=False so the caller never reads
        a missing reading as "can gone"."""
        response.success = False
        response.present = False
        response.measured_depth_mm = -1.0
        response.expected_depth_mm = -1.0
        response.delta_mm = 0.0
        response.measured_z = -1.0

        with self._depth_lock:
            latest = self._depth_buffer[-1] if self._depth_buffer else None
        if latest is None:
            response.message = "no depth frame"
            self.get_logger().warn(f"verify_target: {response.message}")
            return response
        stamp_s, depth = latest
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp_s
        max_age = float(self.get_parameter("verify_max_depth_age_s").value)
        if max_age > 0.0 and age > max_age:
            response.message = f"depth stale ({age:.1f} s)"
            self.get_logger().warn(f"verify_target: {response.message}")
            return response

        calibration = self._calibration
        target = request.target
        if target.pixel_x >= 0.0 and target.pixel_y >= 0.0 and target.depth_mm > 0.0:
            px, py = float(target.pixel_x), float(target.pixel_y)
            expected_d = float(target.depth_mm)
            source = "scan pixel"
        else:
            # Hand-built target without a scan pixel: go backwards through the
            # calibration (carries its fit error into the verdict).
            if calibration is None:
                response.message = "target has no scan pixel and NOT CALIBRATED"
                self.get_logger().error(f"verify_target: {response.message}")
                return response
            try:
                px, py, expected_d = calibration.inverse(
                    target.real_x, target.real_y, target.real_z
                )
            except CalibrationError as e:
                response.message = f"cannot invert calibration: {e}"
                self.get_logger().error(f"verify_target: {response.message}")
                return response
            source = "calibration inverse"

        measured_d = self._sample_depth_mm(depth, px, py)
        if measured_d is None:
            response.message = f"no valid depth at px ({px:.0f}, {py:.0f})"
            self.get_logger().warn(f"verify_target: {response.message}")
            return response

        tolerance = float(self.get_parameter("verify_tolerance_mm").value)
        present, delta = classify_presence(expected_d, measured_d, tolerance)
        response.success = True
        response.present = present
        response.measured_depth_mm = measured_d
        response.expected_depth_mm = expected_d
        response.delta_mm = delta
        if calibration is not None:
            # Robot-frame Z of whatever is actually there.
            response.measured_z = calibration.transform(px, py, measured_d)[2]
        self.get_logger().info(
            f"verify px=({px:.0f}, {py:.0f}) [{source}] expected {expected_d:.0f} mm, "
            f"measured {measured_d:.0f} mm, delta {delta:+.0f} mm → "
            f"{'still present' if present else 'gone'}"
        )
        return response

    def _on_get_layer_height(self, request, response):
        depth = self._depth_frame_near(None)
        if depth is None:
            response.median_z = -1.0
            response.layer_index = -1
            return response

        # Sample the live depth frame where the last scan saw cans — no fixed
        # bin ROI, so the measurement tracks the cans (and the bin) if it moves.
        with self._detection_px_lock:
            centers = list(self._last_detection_px)
        if not centers:
            self.get_logger().warn(
                "get_layer_height: no recent detections to sample — run a scan first"
            )
            response.median_z = -1.0
            response.layer_index = -1
            return response

        samples = [
            d for d in (self._sample_depth_mm(depth, px, py) for px, py in centers)
            if d is not None
        ]
        if not samples:
            response.median_z = -1.0
            response.layer_index = -1
            return response

        response.median_z = float(np.median(samples))

        depth_at_top = self.get_parameter("depth_at_top_layer_mm").value
        can_height = self.get_parameter("can_height_mm").value
        if depth_at_top > 0.0 and can_height > 0.0:
            # Larger depth reading = deeper into the bin = lower layer.
            response.layer_index = int(
                round((response.median_z - depth_at_top) / can_height)
            )
        else:
            # unconfigured: set depth_at_top_layer_mm + can_height_mm
            response.layer_index = -1

        return response

    # ------------------------------------------------------------------
    # Calibration math services (task_manager drives the robot between calls)
    # ------------------------------------------------------------------

    def _cal_model(self) -> str:
        return self._cal_model_id or str(self.get_parameter("dot_model_id").value)

    def _call_dot(self, timeout: float) -> _DotHit:
        """One /vision/detect run with the run's marker model; the
        highest-confidence box (of the run's class, if pinned) is the marker.
        Raises TimeoutError (kept for the callers' existing handling) on any
        detect failure. The raw ObbDetectionArray is kept on the hit so the
        dashboard can show what the detector saw."""
        model_id = self._cal_model()
        try:
            result = self._call_detect(model_id, timeout)
        except _DetectError as e:
            raise TimeoutError(str(e)) from e

        array = result.detections
        dets = list(array.detections)
        if self._cal_class_id >= 0:
            dets = [d for d in dets if int(d.class_id) == self._cal_class_id]
        if not dets:
            why = (
                f"no class-{self._cal_class_id} detection" if self._cal_class_id >= 0
                else "no marker detected"
            )
            return _DotHit(found=False, message=why, detections=array)
        best = max(dets, key=lambda d: d.confidence)
        return _DotHit(
            found=True,
            u=float(best.center_x),
            v=float(best.center_y),
            confidence=float(best.confidence),
            message=f"{len(dets)} detection(s)",
            detections=array,
        )

    def _publish_marker(self, **fields):
        """Echo one calibration sample (or a reset) for the dashboard."""
        msg = CalibrationMarker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color"
        msg.mode = "manual" if self._is_manual() else "automatic"
        msg.model_id = "" if self._is_manual() else self._cal_model()
        msg.depth_mm = -1.0
        array = fields.pop("detections", None)
        if array is not None:
            msg.image_width = int(array.image_width)
            msg.image_height = int(array.image_height)
            msg.detections = list(array.detections)
        for key, value in fields.items():
            setattr(msg, key, value)
        self._marker_pub.publish(msg)

    def _is_manual(self) -> bool:
        if self._cal_manual is None:
            return bool(self.get_parameter("manual_dot_selection").value)
        return self._cal_manual

    # SKIP sentinel returned by _await_manual_click when the operator drops a pose.
    _SKIP = "skip"

    def _on_manual_click(self, msg: PointStamped):
        """Operator clicked the marker centre on the web UI's color feed.
        point.x / point.y are COLOR image pixels."""
        with self._manual_lock:
            self._manual_result = (float(msg.point.x), float(msg.point.y))
        self._manual_event.set()

    def _on_manual_skip(self, msg: Empty):
        """Operator says the marker is not in frame — drop this pose."""
        with self._manual_lock:
            self._manual_result = self._SKIP
        self._manual_event.set()

    def _await_manual_click(self, timeout: float):
        """Ask for a click and block until the operator clicks or skips.

        Returns (u, v) on a click, self._SKIP if the pose was skipped, or None
        on timeout. Clears any stale click/skip first, so an input sent while
        the arm was still moving can never be consumed as this pose's answer.
        """
        with self._manual_lock:
            self._manual_result = None
        self._manual_event.clear()
        self._manual_request_active = bool(True)
        self._manual_request_pub.publish(Bool(data=True))
        self.get_logger().info(
            "[calibration] waiting for a marker click (or skip) on the web UI"
        )
        try:
            if not self._manual_event.wait(timeout=timeout):
                return None
            with self._manual_lock:
                return self._manual_result
        finally:
            self._manual_request_active = bool(False)
            self._manual_request_pub.publish(Bool(data=False))

    def _on_cal_begin(self, request, response):
        if not self.get_parameter("calibration_file").value:
            response.success = False
            response.message = "calibration_file parameter is empty"
            return response

        # Extrinsic mode needs the camera intrinsics first — refuse here so the
        # task manager never moves the arm for a fit that can't complete.
        if self.get_parameter("require_intrinsics").value:
            with self._intrinsics_lock:
                have_intrinsics = self._intrinsics is not None
            if not have_intrinsics:
                response.success = False
                response.message = (
                    "no camera intrinsics yet (waiting on /camera/color/camera_info "
                    "— check the streamer's intrinsics file)"
                )
                return response

        if self._depth_frame_near(None) is None:
            response.success = False
            response.message = "no depth frames received yet"
            return response

        # The marker offset depends on which tool file the commanded TCPs
        # are expressed in — snapshot the active tool for the whole run.
        tool = self._active_tool()
        if tool is None:
            response.success = False
            response.message = (
                "no fresh /cobot/state — cannot tell which tool file is active "
                "(is cobot_control running?)"
            )
            return response
        marker = select_marker_offset(self._marker_offsets, tool[0])
        if marker is None:
            response.success = False
            response.message = (
                f"active tool '{tool[1]}' is tool file {tool[0]} but cobot_setup.yaml tools has "
                f"no entry for it (marker_offset_mm unknown)"
            )
            return response
        if marker.is_zero:
            self.get_logger().warn(
                f"[calibration] tools.{marker.name}.marker_offset_mm is all zero — the fit "
                "assumes the dot sits exactly on the TCP"
            )
        self._cal_tool = tool

        # Pin how the marker pixel is found for this whole run. MODE_DEFAULT
        # keeps the legacy behaviour (the manual_dot_selection parameter).
        if request.mode == CalibrationBegin.Request.MODE_MANUAL:
            self._cal_manual = True
        elif request.mode == CalibrationBegin.Request.MODE_AUTOMATIC:
            self._cal_manual = False
        else:
            self._cal_manual = None
        self._cal_model_id = str(request.model_id or "")
        self._cal_class_id = int(request.class_id)
        self._cal_reads = int(request.samples_per_pose)
        manual = self._is_manual()

        # Manual mode never touches the marker model — skip the client check
        # and the (slow) lazy-load warm-up entirely.
        if not manual:
            if not self._detect_client.wait_for_server(timeout_sec=5.0):
                response.success = False
                response.message = "/vision/detect not available"
                return response

            # Warm-up: the first call makes cobot_detector lazy-load the dot
            # model, so per-pose sampling is fast once the robot starts moving.
            try:
                self._call_dot(DOT_WARMUP_TIMEOUT_SEC)
            except TimeoutError as e:
                response.success = False
                response.message = f"marker model '{self._cal_model()}' warm-up failed: {e}"
                return response

        self._samples_uvd.clear()
        self._samples_xyz.clear()
        self._manual_request_active = bool(False)
        self._manual_request_pub.publish(Bool(data=False))
        mode = "MANUAL clicks" if manual else f"AUTOMATIC ({self._cal_model()} warm)"
        self.get_logger().info(
            f"[calibration] buffer cleared, {mode}, tool '{tool[1]}' (file {tool[0]}), "
            f"marker offset {list(marker.offset_mm)}"
        )
        self._publish_marker(reset=True, message="calibration started")
        response.success = True
        response.message = f"ready ({mode}, tool file {tool[0]})"
        response.mode_name = "manual" if manual else "automatic"
        response.model_id = "" if manual else self._cal_model()
        return response

    def _on_cal_sample(self, request, response):
        response.sample_count = len(self._samples_uvd)
        tcp = dict(tcp_x=float(request.x), tcp_y=float(request.y), tcp_z=float(request.z))
        hits: list[tuple[float, float]] = []
        reads = 0
        confidence = 0.0
        last_array = None

        if self._is_manual():
            # TEMPORARY manual mode: one operator click replaces N inferences.
            timeout = float(self.get_parameter("manual_click_timeout_sec").value)
            click = self._await_manual_click(timeout)
            if click is None:
                response.success = False
                response.message = f"no marker click within {timeout:.0f}s"
                self._publish_marker(accepted=False, message=response.message, **tcp)
                return response
            if click == self._SKIP:
                # Operator dropped this pose (marker out of frame). Same
                # success=False path the task manager already skips on, but
                # deliberate — say so instead of implying a failure.
                response.success = False
                response.message = "pose skipped by operator (marker not in frame)"
                self.get_logger().info(f"[calibration] {response.message}")
                self._publish_marker(accepted=False, message=response.message, **tcp)
                return response
            u, v = click
        else:
            reads = self._cal_reads or int(self.get_parameter("dot_samples_per_pose").value)
            reads = max(int(reads), 1)
            confidences: list[float] = []
            try:
                for _ in range(reads):
                    dot = self._call_dot(DOT_TIMEOUT_SEC)
                    last_array = dot.detections
                    if dot.found:
                        hits.append((dot.u, dot.v))
                        confidences.append(dot.confidence)
                    time.sleep(0.1)
            except TimeoutError as e:
                response.success = False
                response.message = str(e)
                self._publish_marker(accepted=False, message=response.message,
                                     hits=len(hits), reads=reads, **tcp)
                return response

            if len(hits) < (reads // 2) + 1:
                response.success = False
                response.message = f"marker seen {len(hits)}/{reads} times"
                self._publish_marker(accepted=False, message=response.message,
                                     hits=len(hits), reads=reads, detections=last_array, **tcp)
                return response

            u = float(np.median([h[0] for h in hits]))
            v = float(np.median([h[1] for h in hits]))
            confidence = float(np.median(confidences)) if confidences else 0.0

        depth = self._depth_frame_near(None)
        d = self._sample_depth_mm(depth, u, v) if depth is not None else None
        if d is None:
            response.success = False
            response.message = f"no valid depth at marker px ({u:.0f}, {v:.0f})"
            self._publish_marker(accepted=False, message=response.message, u=float(u), v=float(v),
                                 confidence=confidence, hits=len(hits), reads=reads,
                                 detections=last_array, **tcp)
            return response

        # The commanded TCP is expressed in the tool file that was active at
        # calibration_begin; refuse if it changed mid-run (the ground truth
        # would silently mix two TCPs).
        tool = self._active_tool()
        if self._cal_tool is None or tool is None or tool[0] != self._cal_tool[0]:
            response.success = False
            response.message = (
                f"active tool changed mid-calibration (began with tool file "
                f"{None if self._cal_tool is None else self._cal_tool[0]}, now "
                f"{None if tool is None else tool[0]}) — restart the calibration"
            )
            self.get_logger().error(f"[calibration] {response.message}")
            return response
        marker = select_marker_offset(self._marker_offsets, self._cal_tool[0])
        offset = list(marker.offset_mm) if marker is not None else [0.0, 0.0, 0.0]
        self._samples_uvd.append([u, v, d])
        self._samples_xyz.append(
            [request.x + offset[0], request.y + offset[1], request.z + offset[2]]
        )

        response.sample_count = len(self._samples_uvd)
        response.success = True
        response.message = f"px ({u:.0f}, {v:.0f}) d={d:.0f} mm"
        self.get_logger().info(
            f"[calibration] sample {response.sample_count}: {response.message} "
            f"for TCP ({request.x:.0f}, {request.y:.0f}, {request.z:.0f})"
        )
        self._publish_marker(
            accepted=True, sample_index=int(response.sample_count), message=response.message,
            u=float(u), v=float(v), depth_mm=float(d), confidence=confidence,
            hits=len(hits), reads=reads, detections=last_array, **tcp,
        )
        return response

    def _reject_fit(self, response, message: str):
        """Refuse a fit and say so in the LOG, not just the service response.
        A rejected calibration is the single most important thing to be able
        to find afterwards; /task/status detail scrolls away."""
        response.success = False
        response.message = f"{message} — transform unchanged"
        self.get_logger().error(f"[calibration] REJECTED: {response.message}")
        return response

    def _on_cal_compute(self, request, response):
        min_samples = int(self.get_parameter("calibration_min_samples").value)
        max_rms = float(self.get_parameter("calibration_max_rms_mm").value)
        cal_file = self.get_parameter("calibration_file").value
        require_intrinsics = self.get_parameter("require_intrinsics").value

        with self._intrinsics_lock:
            intrinsics = dict(self._intrinsics) if self._intrinsics else None

        if len(self._samples_uvd) < min_samples:
            return self._reject_fit(
                response,
                f"only {len(self._samples_uvd)} usable samples (need {min_samples})",
            )

        # Fit-mode selection is driven by require_intrinsics, NOT merely by
        # whether a CameraInfo happens to be present:
        #   False -> we do NOT trust the camera's intrinsics; run the full
        #            monolithic fit that solves intrinsics + extrinsics together
        #            (needs the wide depth-span grid). Use this when the SDK's
        #            reported fx/fy are wrong/inconsistent.
        #   True  -> trust the camera's intrinsics; run the small extrinsic-only
        #            (Kabsch) fit, and refuse if no intrinsics have arrived.
        try:
            if not require_intrinsics:
                fitted = CameraRobotCalibration.fit(self._samples_uvd, self._samples_xyz)
            elif intrinsics is not None:
                fitted = CameraRobotCalibration.fit_extrinsic(
                    self._samples_uvd, self._samples_xyz, intrinsics
                )
            else:
                return self._reject_fit(
                    response,
                    "no camera intrinsics — extrinsic fit impossible "
                    "(set require_intrinsics:=false to allow a full fit)",
                )
        except CalibrationError as e:
            return self._reject_fit(response, str(e))

        rms = fitted.rms_mm
        rms_text = f"rms x={rms[0]:.1f} y={rms[1]:.1f} z={rms[2]:.1f} mm"
        if max(rms) > max_rms:
            # Log the samples too — without them a rejection is undebuggable
            # once the buffer is cleared by the next run.
            self.get_logger().error(
                f"[calibration] {fitted.source} fit over "
                f"{fitted.sample_count} samples:\n"
                + "\n".join(
                    f"    px({u:6.1f},{v:6.1f}) d={d:6.1f}  ->  "
                    f"target ({x:8.1f},{y:9.1f},{z:8.1f})"
                    for (u, v, d), (x, y, z) in zip(self._samples_uvd, self._samples_xyz)
                )
            )
            return self._reject_fit(
                response, f"{rms_text} exceeds calibration_max_rms_mm={max_rms:.1f}"
            )

        if self._cal_tool is not None:
            marker = select_marker_offset(self._marker_offsets, self._cal_tool[0])
            fitted.tool_number = int(self._cal_tool[0])
            fitted.tool_name = marker.name if marker is not None else str(self._cal_tool[1])
            fitted.marker_offset_mm = marker.offset_mm if marker is not None else None
        fitted.save(cal_file)
        self._calibration = fitted
        self._samples_uvd.clear()
        self._samples_xyz.clear()

        response.success = True
        response.message = (
            f"{fitted.source} fit, {fitted.sample_count} samples, "
            f"{rms_text}, saved to {cal_file}"
        )
        self.get_logger().info(f"[calibration] COMPLETE: {response.message}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    # Multi-threaded: Scan / calibration handlers block on /vision/detect
    # futures that the executor must keep resolving, and depth frames must
    # keep flowing.
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
