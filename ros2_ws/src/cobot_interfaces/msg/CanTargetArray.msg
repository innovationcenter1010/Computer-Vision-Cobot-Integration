# One ordered pick layer (snake ordering applied by localization_node).
# Publishing this to /vision/targets is what triggers the cobot's pick cycle.

std_msgs/Header header

CanTarget[] targets
