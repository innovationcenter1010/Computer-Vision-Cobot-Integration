# THE operational state of the system, published latched on /task/status by
# task_manager — the single source of truth for "what is the robot doing".
# (Hardware/connection state of the arm itself is CobotStatus on /cobot/status;
# task_manager mirrors DISCONNECTED from there when no task is running.)

uint8 STATE_DISCONNECTED = 0
uint8 STATE_IDLE = 1
uint8 STATE_SCANNING = 2
uint8 STATE_PICKING = 3
uint8 STATE_CALIBRATING = 4
uint8 STATE_STOPPING = 5
uint8 STATE_ERROR = 6

std_msgs/Header header

uint8 state
string detail

# Progress through the active task: can 3 of 12, calibration pose 5 of 12.
# Both zero when no task is running.
string active_task
uint32 progress_current
uint32 progress_total
