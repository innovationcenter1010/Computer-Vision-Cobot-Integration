import cv2
import numpy as np
from pathlib import Path
from datetime import datetime
 
from cobot_application.camera.camera_streamer import DualFrame
 
SAVE_DIR = Path("/app/images")
 
 
def save_dual_frame(dual_frame: DualFrame) -> None:
    """
    Saves a DualFrame to /app/images as two JPEGs:
      - <timestamp>_color.jpg       — raw RGB color frame
      - <timestamp>_depth.jpg       — depth frame colormapped with COLORMAP_JET
    """
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
 
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
 
    # ---- Color ----
    # Camera streams RGB, OpenCV expects BGR
    color_bgr = cv2.cvtColor(dual_frame.color_frame.data, cv2.COLOR_RGB2BGR)
    color_path = SAVE_DIR / f"{timestamp}_color.jpg"
    cv2.imwrite(str(color_path), color_bgr)
 
    # ---- Depth ----
    # int16 mm values — normalize to 0-255 then apply colormap
    depth = dual_frame.depth_frame.data.astype(np.float32)
    depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
    depth_uint8 = depth_norm.astype(np.uint8)
    depth_colormap = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
    depth_path = SAVE_DIR / f"{timestamp}_depth.jpg"
    cv2.imwrite(str(depth_path), depth_colormap)
 
    print(f"[frame_saver] saved {color_path.name} + {depth_path.name}", flush=True)