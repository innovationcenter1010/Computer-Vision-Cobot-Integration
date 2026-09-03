# One detection, PIXEL SPACE ONLY. Emitted by cobot_detector for both model
# types: an OBB model fills a rotated box, a plain bbox model fills an
# axis-aligned rectangle with angle = 0. Real-world conversion is
# localization_node's job (Target).

int32 class_id
float32 confidence

# Box center in pixels
float32 center_x
float32 center_y

# OBB rotation in radians (ultralytics convention, 0 for bbox models).
# corners_px is the source of truth; angle is informational.
float32 angle

# Four corners in pixels: x1,y1, x2,y2, x3,y3, x4,y4
# Order follows the ultralytics xyxyxyxy convention (TL, TR, BR, BL).
float32[8] corners_px
