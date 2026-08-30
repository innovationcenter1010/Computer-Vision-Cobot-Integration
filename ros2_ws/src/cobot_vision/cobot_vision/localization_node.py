"""localization_node — pure geometry. No ML, no robot commands.

Turns pixel-space detections into robot-space pick targets using the fitted
camera→robot calibration (see calibration.py):

  in:  /vision/detections_px (ObbDetectionArray)  — from yolo_node
       /camera/depth/image_raw (Image, 16SC1 mm)  — ring buffer, stamp-matched
       /camera/color/camera_info (CameraInfo)      — camera-SDK intrinsics
  out: /vision/targets (CanTargetArray)           — ordered pick layer;
                                                     consumed by task_manager + web

NO CALIBRATION, NO TARGETS: if calibration_file is missing the node refuses to
localize (detections are dropped, verify_target errors) until a calibration
run has completed. There is deliberately no fallback: an unmeasured transform
must never move the robot.

Spatial setup data (can height, marker offset) comes from cobot_params.yaml —
parameter defaults here are deliberately zeros, and zero-valued spatial params
mean "unconfigured": the dependent feature refuses to run instead of guessing.
Nothing here depends on bin-corner coordinates: pick order is a robot-frame
snake (pathing.py) and layer height is sampled at the live detection pixels,
so both stay correct when the bin moves.

Calibration split: task_manager drives the routine (/task/calibrate) by
sending MoveToPose goals to cobot_node; this node owns every millimeter of
math. Between poses the task manager calls:

  /vision/calibration_begin    — clear the sample buffer, warm the dot model
  /vision/calibration_sample   — "marker parked at TCP (x,y,z), measure now":
                                 locate the dot (yolo_node /vision/locate_dot),
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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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

from cobot_interfaces.msg import Heartbeat, CanTarget, CanTargetArray, ObbDetectionArray
from cobot_interfaces.srv import (
    CalibrationSample,
    GetLayerHeight,
    LocateDot,
    VerifyTarget,
)

from cobot_health.heartbeat import HeartbeatPublisher
from cobot_vision.calibration import CalibrationError, CameraRobotCalibration
from cobot_vision.pathing import order_detections_snake
from cobot_vision.rotation import calculate_real_rz


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

# locate_dot's first call lazy-loads the red-dot model (handled in _begin);
# per-sample calls after that are one inference.
DOT_WARMUP_TIMEOUT_SEC = 110.0
DOT_TIMEOUT_SEC = 20.0


@dataclass
class _Target:
    """Internal working object; duck-types the attrs pathing.py expects."""

    detection_index: int
    class_id: int
    confidence: float
    center_x: float
    center_y: float
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
        # Depth reading (mm) when the TOP layer is present. 0.0 = uncalibrated;
        # get_layer_height then returns layer_index = -1. Calibrate on hardware.
        self.declare_parameter("depth_at_top_layer_mm", 0.0)

        # ---- camera→robot calibration (math side; the pose grid that drives
        # the robot lives in task_manager) ----
        # Where the fitted transform lives; injected by robot.launch.py into
        # the bind-mounted config dir so it survives container rebuilds.
        self.declare_parameter("calibration_file", "")
        # Red dot relative to the commanded TCP, measured along ROBOT BASE
        # axes with the arm at the calibration orientation. Measure to the dot
        # surface the camera actually sees.
        self.declare_parameter("marker_offset_mm", [0.0, 0.0, 0.0])
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
            ObbDetectionArray, "/vision/detections_px", self._on_detections, 10
        )
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self._on_camera_info, LATCHED_QOS
        )

        self._targets_pub = self.create_publisher(CanTargetArray, "/vision/targets", 10)

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
        # their handlers wait on /vision/locate_dot futures that the executor
        # resolves on other threads (MultiThreadedExecutor in main()).
        cal_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            Trigger, "/vision/calibration_begin", self._on_cal_begin,
            callback_group=cal_group,
        )
        self.create_service(
            CalibrationSample, "/vision/calibration_sample", self._on_cal_sample,
            callback_group=cal_group,
        )
        self.create_service(
            Trigger, "/vision/calibration_compute", self._on_cal_compute,
            callback_group=cal_group,
        )
        self._dot_client = self.create_client(LocateDot, "/vision/locate_dot")

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
    # Detections → targets
    # ------------------------------------------------------------------

    def _on_detections(self, msg: ObbDetectionArray):
        calibration = self._calibration
        if calibration is None:
            self.get_logger().error(
                "detections dropped: NOT CALIBRATED — run /task/calibrate",
                throttle_duration_sec=5.0,
            )
            return

        depth = self._depth_frame_near(msg.header.stamp)
        if depth is None:
            self.get_logger().warn("detections dropped: no depth frame yet")
            return

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

        out = CanTargetArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "robot_base"
        for target in ordered:
            can = CanTarget()
            can.detection_index = int(target.detection_index)
            can.class_id = target.class_id
            can.confidence = target.confidence
            can.real_x = target.real_x
            can.real_y = target.real_y
            can.real_z = target.real_z
            can.real_rz = target.real_rz
            out.targets.append(can)

        self._targets_pub.publish(out)
        self._last_targets_count = len(out.targets)
        self.get_logger().info(f"Published {len(out.targets)} ordered targets")

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
            "intrinsics_ok": intrinsics_ok,
            "depth_buffer_len": depth_len,
            "depth_buffer_age_s": depth_age,
            "last_targets_count": self._last_targets_count,
            "manual_click_waiting": self._manual_event is not None and not self._manual_event.is_set() and self._manual_result is None and self._manual_waiting(),
        }

    def _manual_waiting(self) -> bool:
        # True while a calibration sample is blocked on an operator click.
        return bool(getattr(self, "_manual_request_active", False))

    # ------------------------------------------------------------------
    # Tier-1 depth services
    # ------------------------------------------------------------------

    def _on_verify_target(self, request, response):
        calibration = self._calibration
        if calibration is None:
            self.get_logger().error("verify_target refused: NOT CALIBRATED")
            response.present = False
            response.measured_z = -1.0
            return response

        depth = self._depth_frame_near(None)
        if depth is None:
            response.present = False
            response.measured_z = -1.0
            return response

        try:
            px, py, expected_d = calibration.inverse(
                request.target.real_x, request.target.real_y, request.target.real_z
            )
        except CalibrationError as e:
            self.get_logger().error(f"verify_target: {e}")
            response.present = False
            response.measured_z = -1.0
            return response

        measured_d = self._sample_depth_mm(depth, px, py)
        if measured_d is None:
            response.present = False
            response.measured_z = -1.0
            return response

        tolerance = self.get_parameter("verify_tolerance_mm").value
        # Report robot-frame Z of whatever is actually there.
        response.measured_z = calibration.transform(px, py, measured_d)[2]
        response.present = abs(measured_d - expected_d) <= tolerance
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

    def _call_dot(self, timeout: float):
        """Blocking locate_dot call from a service handler; other executor
        threads resolve the future (handlers live in cal_group, the client
        response comes back on the default group)."""
        future = self._dot_client.call_async(LocateDot.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout):
            future.cancel()
            raise TimeoutError(f"/vision/locate_dot timed out after {timeout:.0f}s")
        return future.result()

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

        # Manual mode never touches the red-dot model — skip the client check
        # and the (slow) lazy-load warm-up entirely.
        manual = self.get_parameter("manual_dot_selection").value
        if not manual:
            if not self._dot_client.wait_for_service(timeout_sec=5.0):
                response.success = False
                response.message = "/vision/locate_dot not available"
                return response

            # Warm-up: the first call makes yolo_node lazy-load the red-dot
            # model, so per-pose sampling is fast once the robot starts moving.
            try:
                self._call_dot(DOT_WARMUP_TIMEOUT_SEC)
            except TimeoutError as e:
                response.success = False
                response.message = f"red-dot model warm-up failed: {e}"
                return response

        self._samples_uvd.clear()
        self._samples_xyz.clear()
        self._manual_request_active = bool(False)
        self._manual_request_pub.publish(Bool(data=False))
        mode = "MANUAL clicks" if manual else "dot model warm"
        self.get_logger().info(f"[calibration] buffer cleared, {mode}")
        response.success = True
        response.message = f"ready ({mode})"
        return response

    def _on_cal_sample(self, request, response):
        response.sample_count = len(self._samples_uvd)

        if self.get_parameter("manual_dot_selection").value:
            # TEMPORARY manual mode: one operator click replaces N inferences.
            timeout = float(self.get_parameter("manual_click_timeout_sec").value)
            click = self._await_manual_click(timeout)
            if click is None:
                response.success = False
                response.message = f"no marker click within {timeout:.0f}s"
                return response
            if click == self._SKIP:
                # Operator dropped this pose (marker out of frame). Same
                # success=False path the task manager already skips on, but
                # deliberate — say so instead of implying a failure.
                response.success = False
                response.message = "pose skipped by operator (marker not in frame)"
                self.get_logger().info(f"[calibration] {response.message}")
                return response
            u, v = click
        else:
            reads = int(self.get_parameter("dot_samples_per_pose").value)
            hits = []
            try:
                for _ in range(reads):
                    dot = self._call_dot(DOT_TIMEOUT_SEC)
                    if dot.found:
                        hits.append((dot.u, dot.v))
                    time.sleep(0.1)
            except TimeoutError as e:
                response.success = False
                response.message = str(e)
                return response

            if len(hits) < (reads // 2) + 1:
                response.success = False
                response.message = f"dot seen {len(hits)}/{reads} times"
                return response

            u = float(np.median([h[0] for h in hits]))
            v = float(np.median([h[1] for h in hits]))

        depth = self._depth_frame_near(None)
        d = self._sample_depth_mm(depth, u, v) if depth is not None else None
        if d is None:
            response.success = False
            response.message = f"no valid depth at dot px ({u:.0f}, {v:.0f})"
            return response

        offset = list(self.get_parameter("marker_offset_mm").value)
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
    # Multi-threaded: calibration handlers block on locate_dot futures that
    # the executor must keep resolving, and depth frames must keep flowing.
    executor = MultiThreadedExecutor()
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
