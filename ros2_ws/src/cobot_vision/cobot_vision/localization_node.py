"""localization_node — pure geometry. No ML, no robot commands.

Turns pixel-space detections into robot-space pick targets using the fitted
camera→robot calibration (see calibration.py):

  in:  /vision/detections_px (ObbDetectionArray)  — from yolo_node
       /camera/depth/image_raw (Image, 16SC1 mm)  — ring buffer, stamp-matched
  out: /vision/targets (CanTargetArray)           — ordered pick layer;
                                                     consumed by task_manager + web

NO CALIBRATION, NO TARGETS: if calibration_file is missing the node refuses to
localize (detections are dropped, verify_target errors) until a calibration
run has completed. There is deliberately no fallback: an unmeasured transform
must never move the robot.

All spatial setup data (bin outlines, can height, marker offset) comes from
cobot_params.yaml — parameter defaults here are deliberately zeros, and
zero-valued spatial params mean "unconfigured": the dependent feature refuses
to run instead of guessing.

Calibration split: task_manager drives the routine (/task/calibrate) by
sending MoveToPose goals to cobot_node; this node owns every millimeter of
math. Between poses the task manager calls:

  /vision/calibration_begin    — clear the sample buffer, warm the dot model
  /vision/calibration_sample   — "marker parked at TCP (x,y,z), measure now":
                                 locate the dot (yolo_node /vision/locate_dot),
                                 sample own depth buffer, add marker offset,
                                 store the matched pair
  /vision/calibration_compute  — fit the 3x4 transform, validate RMS, persist
                                 to calibration_file, hot-swap the live model

Tier-1 depth services (no ML):
  /vision/verify_target    — is a can still at this target's position?
  /vision/get_layer_height — median depth over the bin ROI → layer index
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from cobot_interfaces.msg import CanTarget, CanTargetArray, ObbDetectionArray
from cobot_interfaces.srv import (
    CalibrationSample,
    GetLayerHeight,
    LocateDot,
    VerifyTarget,
)

from cobot_vision.calibration import CalibrationError, CameraRobotCalibration
from cobot_vision.pathing import order_obb_detections_by_real_position
from cobot_vision.rotation import calculate_real_rz


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# locate_dot's first call lazy-loads the red-dot model (handled in _begin);
# per-sample calls after that are one inference.
DOT_WARMUP_TIMEOUT_SEC = 110.0
DOT_TIMEOUT_SEC = 20.0


@dataclass
class _Target:
    """Internal working object; duck-types the attrs pathing.py expects."""

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

        # ---- workspace geometry: values live in cobot_params.yaml ----
        # Bin outline in pixels, flattened 4x2 TL,TR,BR,BL — only used as the
        # ROI for get_layer_height. All zeros = unconfigured.
        self.declare_parameter("bin_image_corners_px", [0.0] * 8)
        # Bin outline in robot mm, flattened 4x2 TL,TR,BR,BL — defines the
        # bin-local axes for snake pick ordering (pathing.py). All zeros =
        # unconfigured; detections are refused until set.
        self.declare_parameter("bin_corners_mm", [0.0] * 8)

        self.declare_parameter("can_height_mm", 0.0)
        self.declare_parameter("pathing_row_tolerance_mm", 50.0)

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

        # Early, loud signal — the values normally arrive via the params YAML.
        if self._corners_param("bin_corners_mm") is None:
            self.get_logger().error(
                "bin_corners_mm not configured — detections will be refused "
                "(set it in cobot_params.yaml)"
            )

        self._calibration: CameraRobotCalibration | None = None
        self._load_calibration()

        self._depth_lock = threading.Lock()
        self._depth_buffer: deque = deque(
            maxlen=int(self.get_parameter("depth_buffer_size").value)
        )

        # Calibration sample buffer, filled between begin and compute.
        self._samples_uvd: list[list[float]] = []
        self._samples_xyz: list[list[float]] = []

        self.create_subscription(
            Image, "/camera/depth/image_raw", self._on_depth, SENSOR_QOS
        )
        self.create_subscription(
            ObbDetectionArray, "/vision/detections_px", self._on_detections, 10
        )

        self._targets_pub = self.create_publisher(CanTargetArray, "/vision/targets", 10)

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

        self.get_logger().info("localization_node ready")

    def _corners_param(self, name: str) -> list[list[float]] | None:
        """4x[x,y] corners from a flattened param; None while unconfigured.

        Read live (not cached) so /api/config edits apply without restart.
        """
        flat = list(self.get_parameter(name).value)
        if len(flat) != 8:
            raise ValueError(f"{name} must be 8 floats (4 corners x/y), got {len(flat)}")
        if all(v == 0.0 for v in flat):
            return None
        return [[flat[i], flat[i + 1]] for i in range(0, 8, 2)]

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
        rms = self._calibration.rms_mm
        self.get_logger().info(
            f"calibration loaded ({self._calibration.created}, "
            f"{self._calibration.sample_count} samples, "
            f"rms x={rms[0]:.1f} y={rms[1]:.1f} z={rms[2]:.1f} mm)"
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

        bin_corners = self._corners_param("bin_corners_mm")
        if bin_corners is None:
            self.get_logger().error(
                "detections dropped: bin_corners_mm not configured "
                "(set it in cobot_params.yaml)",
                throttle_duration_sec=5.0,
            )
            return

        depth = self._depth_frame_near(msg.header.stamp)
        if depth is None:
            self.get_logger().warn("detections dropped: no depth frame yet")
            return

        targets: list[_Target] = []

        for det in msg.detections:
            d = self._sample_depth_mm(depth, det.center_x, det.center_y)
            if d is None:
                self.get_logger().warn(
                    f"skipping detection at px ({det.center_x:.0f}, "
                    f"{det.center_y:.0f}): no valid depth"
                )
                continue

            target = _Target(
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

        ordered = order_obb_detections_by_real_position(
            detections=targets,
            real_world_corners_mm=bin_corners,
            row_tolerance_mm=self.get_parameter("pathing_row_tolerance_mm").value,
        )

        out = CanTargetArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "robot_base"
        for target in ordered:
            can = CanTarget()
            can.class_id = target.class_id
            can.confidence = target.confidence
            can.real_x = target.real_x
            can.real_y = target.real_y
            can.real_z = target.real_z
            can.real_rz = target.real_rz
            out.targets.append(can)

        self._targets_pub.publish(out)
        self.get_logger().info(f"Published {len(out.targets)} ordered targets")

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

        # Bin ROI = bounding box of the bin's pixel corners.
        pixel_corners = self._corners_param("bin_image_corners_px")
        if pixel_corners is None:
            self.get_logger().warn(
                "get_layer_height refused: bin_image_corners_px not configured"
            )
            response.median_z = -1.0
            response.layer_index = -1
            return response

        corners = np.array(pixel_corners)
        h, w = depth.shape
        x0 = max(0, int(corners[:, 0].min()))
        x1 = min(w, int(corners[:, 0].max()))
        y0 = max(0, int(corners[:, 1].min()))
        y1 = min(h, int(corners[:, 1].max()))

        roi = depth[y0:y1, x0:x1]
        valid = roi[roi > 0]
        if valid.size == 0:
            response.median_z = -1.0
            response.layer_index = -1
            return response

        response.median_z = float(np.median(valid))

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

    def _on_cal_begin(self, request, response):
        if not self.get_parameter("calibration_file").value:
            response.success = False
            response.message = "calibration_file parameter is empty"
            return response

        if self._depth_frame_near(None) is None:
            response.success = False
            response.message = "no depth frames received yet"
            return response

        if not self._dot_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = "/vision/locate_dot not available"
            return response

        # Warm-up: the first call makes yolo_node lazy-load the red-dot model,
        # so per-pose sampling is fast once the robot starts moving.
        try:
            self._call_dot(DOT_WARMUP_TIMEOUT_SEC)
        except TimeoutError as e:
            response.success = False
            response.message = f"red-dot model warm-up failed: {e}"
            return response

        self._samples_uvd.clear()
        self._samples_xyz.clear()
        self.get_logger().info("[calibration] buffer cleared, dot model warm")
        response.success = True
        response.message = "ready"
        return response

    def _on_cal_sample(self, request, response):
        response.sample_count = len(self._samples_uvd)

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

    def _on_cal_compute(self, request, response):
        min_samples = int(self.get_parameter("calibration_min_samples").value)
        max_rms = float(self.get_parameter("calibration_max_rms_mm").value)
        cal_file = self.get_parameter("calibration_file").value

        if len(self._samples_uvd) < min_samples:
            response.success = False
            response.message = (
                f"only {len(self._samples_uvd)} usable samples "
                f"(need {min_samples}) — transform unchanged"
            )
            return response

        try:
            fitted = CameraRobotCalibration.fit(self._samples_uvd, self._samples_xyz)
        except CalibrationError as e:
            response.success = False
            response.message = f"{e} — transform unchanged"
            return response

        rms = fitted.rms_mm
        rms_text = f"rms x={rms[0]:.1f} y={rms[1]:.1f} z={rms[2]:.1f} mm"
        if max(rms) > max_rms:
            response.success = False
            response.message = (
                f"{rms_text} exceeds calibration_max_rms_mm={max_rms:.1f} "
                "— transform unchanged"
            )
            return response

        fitted.save(cal_file)
        self._calibration = fitted
        self._samples_uvd.clear()
        self._samples_xyz.clear()

        response.success = True
        response.message = f"{fitted.sample_count} samples, {rms_text}, saved to {cal_file}"
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
