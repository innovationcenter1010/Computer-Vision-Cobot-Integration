# All detections from one detector run.
# header.stamp = the timestamp of the color frame the model ran on —
# localization_node uses it to pick the matching depth frame.

std_msgs/Header header
string model_id      # models.yaml id of the model that produced these

uint32 image_width
uint32 image_height

ObbDetection[] detections
