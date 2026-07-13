# All detections from one YOLO scan.
# header.stamp = the timestamp of the color frame the model ran on —
# localization_node uses it to pick the matching depth frame.

std_msgs/Header header

uint32 image_width
uint32 image_height

ObbDetection[] detections
