# One ordered pick layer (snake ordering applied by localization_node).
# Returned by the /vision/scan action and mirrored on /vision/targets.
# It is DATA, not a trigger: nothing moves because this was published.

std_msgs/Header header
string model_id      # models.yaml id the detections came from

Target[] targets
