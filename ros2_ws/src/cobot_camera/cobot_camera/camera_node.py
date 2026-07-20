"""camera_node — bridges the external ZMQ camera streamer into the ROS graph.

Wraps CameraStream (two background SUB threads, latest-frame cache) and
republishes on a fixed-rate timer:

    /camera/color/image_raw   sensor_msgs/Image, rgb8
    /camera/depth/image_raw   sensor_msgs/Image, 16SC1 (int16 mm, matches streamer)
    /camera/color/camera_info sensor_msgs/CameraInfo, latched (see below)

Camera intrinsics come from the camera's own SDK, not a calibration: the
external streamer writes a small JSON next to the frames (intrinsics_file, in
the bind-mounted vision_workspace) and this node relays it as a latched
CameraInfo. localization_node consumes it for the extrinsic-only startup
calibration. Static data → published once (and again only if the file
changes), not per frame. Schema:

    {"fx": ..., "fy": ..., "cx": ..., "cy": ..., "width": 640, "height": 480}

Service /camera/save_frames (std_srvs/Trigger) writes the current color+depth
pair to capture_dir as a timestamped PNG pair (color 8-bit, depth 16-bit mm).
The directory sits inside the bind-mounted vision_workspace, so captures land
on the host — useful for collecting training images and debugging.

QoS: images best-effort, keep-last(1) — freshest frame wins, no backlog.
CameraInfo is reliable + transient-local so late subscribers still get it.
The ZMQ endpoints are parameters; the legacy hardcoded defaults are kept.
"""

import json
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from cobot_camera.camera_streamer import CameraStream, Frame


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Latched so localization_node gets the intrinsics whenever it subscribes.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.declare_parameter("color_endpoint", "tcp://host.docker.internal:5550")
        self.declare_parameter("depth_endpoint", "tcp://host.docker.internal:5551")
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("color_frame_id", "camera_color")
        self.declare_parameter("depth_frame_id", "camera_depth")
        # Inside the vision_workspace bind mount → files appear on the host.
        self.declare_parameter("capture_dir", "/ws/vision_workspace/operation/captures")
        # JSON the streamer writes from the camera SDK; relayed as CameraInfo.
        self.declare_parameter(
            "intrinsics_file", "/ws/vision_workspace/operation/camera_intrinsics.json"
        )

        color_endpoint = self.get_parameter("color_endpoint").value
        depth_endpoint = self.get_parameter("depth_endpoint").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self._color_frame_id = self.get_parameter("color_frame_id").value
        self._depth_frame_id = self.get_parameter("depth_frame_id").value

        self._color_pub = self.create_publisher(Image, "/camera/color/image_raw", SENSOR_QOS)
        self._depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", SENSOR_QOS)
        self._info_pub = self.create_publisher(
            CameraInfo, "/camera/color/camera_info", LATCHED_QOS
        )
        self._intrinsics_mtime: float | None = None
        # Publish now if the file already exists; a slow timer picks up changes.
        self._publish_intrinsics()
        self.create_timer(5.0, self._publish_intrinsics)

        self._stream = CameraStream(
            color_endpoint=color_endpoint,
            depth_endpoint=depth_endpoint,
        )
        self._stream.start()
        self.get_logger().info(
            f"CameraStream started (color={color_endpoint}, depth={depth_endpoint})"
        )

        # Publish from a timer instead of the ZMQ threads: keeps all ROS calls
        # on the executor thread and gives a natural throttle parameter.
        self._last_color_index = -1
        self._last_depth_index = -1
        self.create_timer(1.0 / publish_rate_hz, self._publish_latest)

        self.create_service(Trigger, "/camera/save_frames", self._on_save_frames)

    def _publish_intrinsics(self):
        """Relay the streamer's intrinsics JSON as a latched CameraInfo.

        Reads only when the file changes (mtime), so this is cheap on the
        5 s timer. Missing/partial file → skip quietly; a later write is
        picked up automatically.
        """
        path = Path(self.get_parameter("intrinsics_file").value)
        try:
            mtime = path.stat().st_mtime
        except OSError:
            return  # not written yet
        if mtime == self._intrinsics_mtime:
            return  # unchanged since last publish

        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            fx = float(data["fx"])
            fy = float(data["fy"])
            cx = float(data["cx"])
            cy = float(data["cy"])
            width = int(data.get("width", 0))
            height = int(data.get("height", 0))
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f"bad intrinsics file {path}: {e}")
            return

        info = CameraInfo()
        info.header.frame_id = self._color_frame_id
        info.header.stamp = self.get_clock().now().to_msg()
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # distortion absorbed as fit residual
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self._info_pub.publish(info)
        self._intrinsics_mtime = mtime
        self.get_logger().info(
            f"published camera intrinsics fx={fx:.1f} fy={fy:.1f} "
            f"cx={cx:.1f} cy={cy:.1f} ({width}x{height}) from {path}"
        )

    def _publish_latest(self):
        color = self._stream.latest_color
        depth = self._stream.latest_depth
        now = self.get_clock().now().to_msg()

        if color is not None and color.index != self._last_color_index:
            msg = self._frame_to_image(color, encoding="rgb8", frame_id=self._color_frame_id)
            msg.header.stamp = now
            self._color_pub.publish(msg)
            self._last_color_index = color.index

        if depth is not None and depth.index != self._last_depth_index:
            msg = self._frame_to_image(depth, encoding="16SC1", frame_id=self._depth_frame_id)
            msg.header.stamp = now
            self._depth_pub.publish(msg)
            self._last_depth_index = depth.index

    def _on_save_frames(self, request, response):
        """Write the freshest color+depth pair to capture_dir.

        color_<stamp>.png  — 8-bit BGR (lossless)
        depth_<stamp>.png  — 16-bit grayscale, value = millimeters
        """
        color = self._stream.latest_color
        depth = self._stream.latest_depth

        if color is None and depth is None:
            response.success = False
            response.message = "no frames received from the streamer yet"
            return response

        capture_dir = Path(self.get_parameter("capture_dir").value)
        try:
            capture_dir.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            response.success = False
            response.message = f"cannot create {capture_dir}: {e}"
            return response

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        written = []

        if color is not None:
            path = capture_dir / f"color_{stamp}.png"
            # Stream delivers RGB; OpenCV writes BGR.
            cv2.imwrite(str(path), cv2.cvtColor(color.data, cv2.COLOR_RGB2BGR))
            written.append(path.name)

        if depth is not None:
            path = capture_dir / f"depth_{stamp}.png"
            # int16 mm → uint16 PNG (negative/invalid readings clip to 0).
            cv2.imwrite(str(path), np.clip(depth.data, 0, 65535).astype(np.uint16))
            written.append(path.name)

        missing = "" if len(written) == 2 else " (other stream had no frame yet)"
        response.success = True
        response.message = f"saved {', '.join(written)} to {capture_dir}{missing}"
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _frame_to_image(frame: Frame, encoding: str, frame_id: str) -> Image:
        msg = Image()
        msg.header.frame_id = frame_id
        msg.height = frame.height
        msg.width = frame.width
        msg.encoding = encoding
        msg.is_bigendian = False
        data = frame.data.tobytes()
        msg.step = len(data) // frame.height
        msg.data = data
        return msg

    def destroy_node(self):
        self._stream.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
