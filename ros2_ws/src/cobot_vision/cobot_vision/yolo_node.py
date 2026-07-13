"""yolo_node — pure perception. No geometry, no millimeters.

Subscribes /camera/color/image_raw (best-effort, latest wins) and caches the
newest frame. A /vision/scan Trigger kicks YOLO-OBB off in a worker thread and
returns immediately; results flow on /vision/detections_px, header-stamped
with the color frame the model actually ran on so localization_node can match
the corresponding depth frame.

Also serves /vision/locate_dot for camera→robot calibration: a second model
(red_dot_best.pt) finds the marker mounted on the cobot. Lazy-loaded on first
call so normal startup stays fast; localization_node warms it up before the
robot starts moving.

Only node in the graph that imports ultralytics.
"""

import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from cobot_interfaces.msg import ObbDetection, ObbDetectionArray
from cobot_interfaces.srv import LocateDot

from cobot_vision.yolo_obb_detector import (
    YoloOBBDetector,
    remove_duplicate_detections,
)


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        self.declare_parameter("model_path", "/ws/vision_workspace/models/cans_obb_best.pt")
        self.declare_parameter("confidence", 0.50)
        self.declare_parameter("iou", 0.4)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("remove_duplicates", True)
        self.declare_parameter("duplicate_center_distance_px", 50.0)

        # Calibration marker model (see /vision/locate_dot).
        self.declare_parameter("dot_model_path", "/ws/vision_workspace/models/red_dot_best.pt")
        self.declare_parameter("dot_confidence", 0.5)

        model_path = self.get_parameter("model_path").value

        self._detector = YoloOBBDetector(
            model_path=model_path,
            conf=self.get_parameter("confidence").value,
            imgsz=self.get_parameter("image_size").value,
            iou=self.get_parameter("iou").value,
            max_det=self.get_parameter("max_detections").value,
        )
        self.get_logger().info(f"YOLO model loaded: {model_path}")

        # Live tuning: accept scalar changes without restart.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._frame_lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._latest_stamp = None

        self._scan_thread: threading.Thread | None = None

        # Red-dot model: loaded on first locate_dot call, guarded because the
        # load takes seconds and calls may race.
        self._dot_lock = threading.Lock()
        self._dot_model = None

        self.create_subscription(Image, "/camera/color/image_raw", self._on_color, SENSOR_QOS)
        self._detections_pub = self.create_publisher(
            ObbDetectionArray, "/vision/detections_px", 10
        )
        self.create_service(Trigger, "/vision/scan", self._on_scan)
        self.create_service(LocateDot, "/vision/locate_dot", self._on_locate_dot)

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def _on_color(self, msg: Image):
        if msg.encoding != "rgb8":
            self.get_logger().warn(f"Unexpected color encoding: {msg.encoding}", once=True)
            return

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

        with self._frame_lock:
            self._latest_frame = frame
            self._latest_stamp = msg.header.stamp

    def _on_set_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == "confidence":
                self._detector.conf = float(param.value)
            elif param.name == "iou":
                self._detector.iou = float(param.value)
            elif param.name == "max_detections":
                self._detector.max_det = int(param.value)
            elif param.name == "image_size":
                self._detector.imgsz = int(param.value)
            elif param.name in ("model_path", "dot_model_path"):
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} cannot change at runtime — restart yolo_node",
                )

        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Scan
    # ------------------------------------------------------------------

    def _on_scan(self, request, response):
        with self._frame_lock:
            frame = self._latest_frame
            stamp = self._latest_stamp

        if frame is None:
            response.success = False
            response.message = "no camera frame received yet"
            return response

        if self._scan_thread is not None and self._scan_thread.is_alive():
            response.success = False
            response.message = "scan already running"
            return response

        # Run inference off the executor thread; results are published on
        # /vision/detections_px when done.
        self._scan_thread = threading.Thread(
            target=self._run_scan, args=(frame, stamp), daemon=True
        )
        self._scan_thread.start()

        response.success = True
        response.message = "scan started"
        return response

    def _run_scan(self, frame: np.ndarray, stamp):
        try:
            detections = self._detector.detect(image=frame, save_visual=False)

            if self.get_parameter("remove_duplicates").value:
                detections = remove_duplicate_detections(
                    detections,
                    min_center_distance_px=self.get_parameter(
                        "duplicate_center_distance_px"
                    ).value,
                )

            msg = ObbDetectionArray()
            msg.header.stamp = stamp
            msg.header.frame_id = "camera_color"
            msg.image_height, msg.image_width = frame.shape[:2]

            for det in detections:
                out = ObbDetection()
                out.class_id = det.class_id
                out.confidence = det.confidence
                out.center_x = float(det.center_x)
                out.center_y = float(det.center_y)
                out.corners_px = [
                    float(v) for corner in det.pixel_corners for v in corner
                ]
                msg.detections.append(out)

            self._detections_pub.publish(msg)
            self.get_logger().info(f"Scan complete: {len(msg.detections)} detections")

        except Exception as e:  # noqa: BLE001 — a failed scan must not kill the node
            self.get_logger().error(f"Scan failed: {e}")

    # ------------------------------------------------------------------
    # Calibration marker
    # ------------------------------------------------------------------

    def _get_dot_model(self):
        with self._dot_lock:
            if self._dot_model is None:
                from ultralytics import YOLO

                path = self.get_parameter("dot_model_path").value
                self.get_logger().info(f"Loading red-dot model: {path}")
                self._dot_model = YOLO(path)
            return self._dot_model

    def _on_locate_dot(self, request, response):
        response.found = False
        response.u = 0.0
        response.v = 0.0
        response.confidence = 0.0

        with self._frame_lock:
            frame = self._latest_frame

        if frame is None:
            response.message = "no camera frame received yet"
            return response

        try:
            model = self._get_dot_model()
            results = model.predict(
                source=frame,
                conf=self.get_parameter("dot_confidence").value,
                verbose=False,
            )[0]
        except Exception as e:  # noqa: BLE001 — a bad model file must not kill the node
            response.message = f"red-dot inference failed: {e}"
            self.get_logger().error(response.message)
            return response

        # Accept either head type: axis-aligned boxes or OBB, whichever the
        # red-dot model was trained with. Take the highest-confidence hit.
        centers, confs = [], []
        if getattr(results, "obb", None) is not None and len(results.obb) > 0:
            centers = results.obb.xywhr[:, :2].tolist()
            confs = results.obb.conf.tolist()
        elif getattr(results, "boxes", None) is not None and len(results.boxes) > 0:
            centers = results.boxes.xywh[:, :2].tolist()
            confs = results.boxes.conf.tolist()

        if not centers:
            response.message = "no dot detected"
            return response

        best = max(range(len(confs)), key=lambda i: confs[i])
        response.found = True
        response.u = float(centers[best][0])
        response.v = float(centers[best][1])
        response.confidence = float(confs[best])
        response.message = f"{len(centers)} detection(s)"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
