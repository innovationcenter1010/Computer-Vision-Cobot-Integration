# Hardware/connection status of the arm, published on /cobot/status
# (reliable, latched) by cobot_node. This is NOT the operational task state —
# picking/calibrating/etc live in TaskStatus on /task/status (task_manager).

uint8 STATE_DISCONNECTED = 0
uint8 STATE_IDLE = 1      # connected, no motion goal executing
uint8 STATE_BUSY = 2      # executing a PickCan / MoveToPose goal
uint8 STATE_ERROR = 3

std_msgs/Header header

uint8 state
string detail
