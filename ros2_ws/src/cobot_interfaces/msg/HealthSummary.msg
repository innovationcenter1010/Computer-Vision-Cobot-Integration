# Aggregated system health, published latched on /health/summary at ~1 Hz.
std_msgs/Header header
uint8 level               # worst node level (STALE / UNKNOWN count as ERROR severity)
NodeHealth[] nodes
