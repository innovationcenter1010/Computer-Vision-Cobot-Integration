from cobot_application.camera import astra_cpp

from dataclasses import dataclass
import numpy as np


@dataclass
class RGBDImage:
    timestamp: int

    color_frame_index: int
    depth_frame_index: int

    rgb: np.ndarray      # (H, W, 3), uint8
    depth: np.ndarray    # (H, W), int16

    color_width: int
    color_height: int

    depth_width: int
    depth_height: int

    def yolo_image(self) -> np.ndarray:
        """
        Returns image in standard HWC uint8 format expected by Ultralytics YOLO.
        """
        return self.rgb

    def depth_at(self, x: int, y: int) -> int:
        """
        Return depth value at pixel location.
        """
        return int(self.depth[y, x])


def rgbdframe_to_numpy(frame: astra_cpp.RGBDFrame) -> RGBDImage:
    """
    Convert pybind RGBDFrame -> RGBDImage
    """

    # RGB vector<uint8_t> -> numpy
    rgb = np.asarray(frame.rgb, dtype=np.uint8)

    expected_rgb_size = (
        frame.color_height
        * frame.color_width
        * 3
    )

    if rgb.size != expected_rgb_size:
        raise ValueError(
            f"RGB size mismatch. "
            f"Expected {expected_rgb_size}, got {rgb.size}"
        )

    rgb = rgb.reshape(
        frame.color_height,
        frame.color_width,
        3
    )

    # depth vector<int16_t> -> numpy
    depth = np.asarray(frame.depth, dtype=np.int16)

    expected_depth_size = (
        frame.depth_height
        * frame.depth_width
    )

    if depth.size != expected_depth_size:
        raise ValueError(
            f"Depth size mismatch. "
            f"Expected {expected_depth_size}, got {depth.size}"
        )

    depth = depth.reshape(
        frame.depth_height,
        frame.depth_width
    )

    return RGBDImage(
        timestamp=frame.timestamp,

        color_frame_index=frame.color_frame_index,
        depth_frame_index=frame.depth_frame_index,

        rgb=rgb,
        depth=depth,

        color_width=frame.color_width,
        color_height=frame.color_height,

        depth_width=frame.depth_width,
        depth_height=frame.depth_height,
    )