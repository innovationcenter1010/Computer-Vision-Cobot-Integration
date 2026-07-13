"""localization_node — pure geometry. No ML.

Turns pixel-space detections into robot-space pick targets:

  in:  /vision/detections_px (ObbDetectionArray)  — from yolo_node
       /camera/depth/image_raw (Image, 16SC1 mm)  — ring buffer, stamp-matched
  out: /vision/targets (CanTargetArray)           — ordered pick layer;
                                                     consumed by cobot_node + web

Per detection: homography px→real x/y, real_rz from the OBB left edge,
real_z sampled from the depth image at the can center.

Also serves the cheap tier-1 depth checks (no YOLO involved):
  /vision/verify_target    — is a can still at this target's position?
  /vision/get_layer_height — median depth over the bin ROI → layer index
"""

import threading
from collections import deque
from dataclasses import dataclass, field

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image

from cobot_interfaces.msg import CanTarget, CanTargetArray, ObbDetectionArray
from cobot_interfaces.srv import GetLayerHeight, VerifyTarget

from cobot_vision.homography import HomographyTranslator
from cobot_vision.pathing import order_obb_detections_by_real_position
from cobot_vision.rotation import calculate_real_rz


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


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

        # Homography calibration: flattened 4x2 [x1,y1, x2,y2, x3,y3, x4,y4],
        # corner order TL, TR, BR, BL (matches legacy bin_configs).
        self.declare_parameter(
            "image_corners",
            [3744.0, 1178.0, 4114.0, 4524.0, 492.0, 4408.0, 904.0, 1241.0],
        )
        self.declare_parameter(
            "real_world_corners_mm",
            [448.22, -1600.0, -677.22, -1600.0, -677.22, -565.96, 448.22, -565.96],
        )

        self.declare_parameter("can_height_mm", 87.0)
        self.declare_parameter("pathing_row_tolerance_mm", 50.0)

        # Depth sampling
        self.declare_parameter("depth_buffer_size", 15)
        self.declare_parameter("depth_sample_window_px", 10)
        # |measured - expected| below this → target still present
        self.declare_parameter("verify_tolerance_mm", 40.0)
        # Depth reading (mm) when the TOP layer is present. 0.0 = uncalibrated;
        # get_layer_height then returns layer_index = -1. Calibrate on hardware.
        self.declare_parameter("depth_at_top_layer_mm", 0.0)

        image_corners = self._corners_param("image_corners")
        self._real_corners = self._corners_param("real_world_corners_mm")

        self._translator = HomographyTranslator(image_corners, self._real_corners)
        self._inverse_homography = np.linalg.inv(self._translator.homography_matrix)

        self._depth_lock = threading.Lock()
        self._depth_buffer: deque = deque(
            maxlen=int(self.get_parameter("depth_buffer_size").value)
        )

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

        self.get_logger().info("localization_node ready")

    def _corners_param(self, name: str) -> list[list[float]]:
        flat = list(self.get_parameter(name).value)
        if len(flat) != 8:
            raise ValueError(f"{name} must be 8 floats (4 corners x/y), got {len(flat)}")
        return [[flat[i], flat[i + 1]] for i in range(0, 8, 2)]

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

    def _real_to_pixel(self, real_x: float, real_y: float) -> tuple[float, float]:
        """Inverse homography: robot-space mm → image pixel."""
        point = self._inverse_homography @ np.array([real_x, real_y, 1.0])
        return point[0] / point[2], point[1] / point[2]

    # ------------------------------------------------------------------
    # Detections → targets
    # ------------------------------------------------------------------

    def _on_detections(self, msg: ObbDetectionArray):
        depth = self._depth_frame_near(msg.header.stamp)
        if depth is None:
            self.get_logger().warn("Detections received but no depth frame yet")

        targets: list[_Target] = []

        for det in msg.detections:
            target = _Target(
                class_id=det.class_id,
                confidence=det.confidence,
                center_x=det.center_x,
                center_y=det.center_y,
            )

            real_x, real_y = self._translator.translate_point(det.center_x, det.center_y)
            target.real_x = float(real_x)
            target.real_y = float(real_y)

            # rz from the OBB left edge: corners are TL,TR,BR,BL → TL=0, BL=3
            corners = det.corners_px
            real_tl = self._translator.translate_point(corners[0], corners[1])
            real_bl = self._translator.translate_point(corners[6], corners[7])
            target.real_rz = float(
                calculate_real_rz(real_TL=list(real_tl), real_BL=list(real_bl))
            )

            if depth is not None:
                measured = self._sample_depth_mm(depth, det.center_x, det.center_y)
                target.real_z = measured if measured is not None else 0.0
            else:
                target.real_z = 0.0

            targets.append(target)

        ordered = order_obb_detections_by_real_position(
            detections=targets,
            real_world_corners_mm=self._real_corners,
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
        depth = self._depth_frame_near(None)
        if depth is None:
            response.present = False
            response.measured_z = -1.0
            return response

        px, py = self._real_to_pixel(request.target.real_x, request.target.real_y)
        measured = self._sample_depth_mm(depth, px, py)

        if measured is None:
            response.present = False
            response.measured_z = -1.0
            return response

        tolerance = self.get_parameter("verify_tolerance_mm").value
        response.measured_z = measured
        response.present = abs(measured - request.target.real_z) <= tolerance
        return response

    def _on_get_layer_height(self, request, response):
        depth = self._depth_frame_near(None)
        if depth is None:
            response.median_z = -1.0
            response.layer_index = -1
            return response

        # Bin ROI = bounding box of the homography image corners.
        corners = np.array(self._corners_param("image_corners"))
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
        if depth_at_top > 0.0:
            # Larger depth reading = deeper into the bin = lower layer.
            response.layer_index = int(
                round((response.median_z - depth_at_top) / can_height)
            )
        else:
            response.layer_index = -1  # uncalibrated: set depth_at_top_layer_mm

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
