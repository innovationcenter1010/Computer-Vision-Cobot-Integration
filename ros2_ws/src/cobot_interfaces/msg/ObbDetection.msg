# One YOLO oriented-bounding-box detection, PIXEL SPACE ONLY.
# Real-world conversion is localization_node's job (CanTarget).

int32 class_id
float32 confidence

# OBB center in pixels
float32 center_x
float32 center_y

# Four OBB corners in pixels: x1,y1, x2,y2, x3,y3, x4,y4
# Order follows ultralytics xyxyxyxy convention (TL, TR, BR, BL).
float32[8] corners_px
