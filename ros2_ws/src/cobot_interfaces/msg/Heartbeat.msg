# Low-frequency liveness + metrics beacon. Every node publishes one on
# /health/heartbeat every period_s (see cobot_health.heartbeat); the
# health_monitor node aggregates them into /health/summary.
uint8 LEVEL_OK = 0
uint8 LEVEL_WARN = 1
uint8 LEVEL_ERROR = 2

std_msgs/Header header
string node_name          # matches the launch `name` (e.g. "camera_node")
uint8 level
string detail             # one-line human summary
float32 period_s          # sender's own period; the monitor scales staleness by it
KeyValue[] metrics
