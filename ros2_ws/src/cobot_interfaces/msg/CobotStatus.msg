# High-level cobot state, published on /cobot/status (reliable, latched).

uint8 STATE_DISCONNECTED = 0
uint8 STATE_IDLE = 1
uint8 STATE_PICKING = 2
uint8 STATE_STOPPING = 3
uint8 STATE_ERROR = 4

std_msgs/Header header

uint8 state
string detail
