import asyncio
import time
import numpy as np
import cv2
from utils import frame_to_bgr_image


from pyorbbecsdk import (
    Pipeline,
    Config,
    OBSensorType,
    OBFormat,
    OBFrameAggregateOutputMode,
)

from cobot_application.third_party.pyorbbecsdk import Pipeline


class orbec_snapshot():
    def __init__(self):
        self.status: bool
        self.timestamp: float
        self.rgb_array: np.array
        self.depth_array = np.array


class OrbecCamera():
    def __init__(self):
        self.pipeline = None
        self.config = None
        self.connected = False


    def connect(self):
        self.pipeline = Pipeline()
        self.config = Config()

        # Enable COLOR stream
        color_profiles = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = color_profiles.get_video_stream_profile(
            0, 0, OBFormat.RGB, 0
        )
        self.config.enable_stream(color_profile)

        # Enable DEPTH stream
        depth_profiles = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        depth_profile = depth_profiles.get_default_video_stream_profile()
        self.config.enable_stream(depth_profile)

        # Optional: ensures frames are paired correctly
        self.config.set_frame_aggregate_output_mode(
            OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE
        )

        self.pipeline.start(self.config)
        self.connected = True

    def _get_snapshot_sync(self):
        frames = self.pipeline.wait_for_frames(1000)
        if not frames:
            return None

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None

        snap = orbec_snapshot()
        snap.timestamp = time.time()

        # -------------------------
        # COLOR (SDK recommended conversion)
        # -------------------------
        color_image = frame_to_bgr_image(color_frame)
        if color_image is None:
            return None
        snap.rgb_array = color_image

        # -------------------------
        # DEPTH (correct SDK method)
        # -------------------------
        depth_data = np.frombuffer(
            depth_frame.get_data(),
            dtype=np.uint16
        ).reshape(
            depth_frame.get_height(),
            depth_frame.get_width()
        )

        # Apply depth scale (IMPORTANT from SDK example)
        depth_data = depth_data.astype(np.float32) * depth_frame.get_depth_scale()

        snap.depth_array = depth_data

        return snap

    # -----------------------------
    # ASYNC PUBLIC API
    # -----------------------------
    async def getsnapshot(self):
        if not self.connected:
            raise RuntimeError("Camera not connected. Call connect() first.")

        snap = await asyncio.to_thread(self._get_snapshot_sync)
        return snap