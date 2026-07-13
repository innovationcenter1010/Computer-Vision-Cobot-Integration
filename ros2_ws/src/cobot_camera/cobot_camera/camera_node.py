"""camera_node — bridges the external ZMQ camera streamer into the ROS graph.

Wraps CameraStream (two background SUB threads, latest-frame cache) and
republishes on a fixed-rate timer:

    /camera/color/image_raw   sensor_msgs/Image, rgb8
    /camera/depth/image_raw   sensor_msgs/Image, 16SC1 (int16 mm, matches streamer)

QoS: best-effort, keep-last(1) — freshest frame wins, no backlog.
The ZMQ endpoints are parameters; the legacy hardcoded defaults are kept.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image

from cobot_camera.camera_streamer import CameraStream, Frame


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.declare_parameter("color_endpoint", "tcp://host.docker.internal:5550")
        self.declare_parameter("depth_endpoint", "tcp://host.docker.internal:5551")
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("color_frame_id", "camera_color")
        self.declare_parameter("depth_frame_id", "camera_depth")

        color_endpoint = self.get_parameter("color_endpoint").value
        depth_endpoint = self.get_parameter("depth_endpoint").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self._color_frame_id = self.get_parameter("color_frame_id").value
        self._depth_frame_id = self.get_parameter("depth_frame_id").value

        self._color_pub = self.create_publisher(Image, "/camera/color/image_raw", SENSOR_QOS)
        self._depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", SENSOR_QOS)

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
