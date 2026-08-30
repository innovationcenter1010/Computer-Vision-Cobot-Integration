# Live controller state, published by cobot_control on /cobot/state at
# poll_period_s (default 10 Hz). ONE source of truth for everything that is
# read from the YRC1000: liveness, hold/alarm/servo state, every named
# sensor/output from cobot_setup.yaml, and the current TCP pose.
# Consumers (cobot_operation, task_manager, web) read from here instead of
# calling services for reads.

std_msgs/Header header

bool connected            # YMConnect link up (false => every other field stale)
bool servo_on
bool running              # controller reports motion in progress
bool in_hold
bool alarming
bool erroring
uint8 CONTROL_MODE_TEACH = 0
uint8 CONTROL_MODE_PLAY = 1
uint8 CONTROL_MODE_REMOTE = 2
uint8 control_mode
bool paused               # cobot_control pause latch (see /cobot/pause)
bool busy                 # a MoveToPose goal is executing

# io.sensors / io.outputs from cobot_setup.yaml, by name. Parallel arrays.
string[] sensor_names
bool[] sensor_values
string[] output_names
bool[] output_values

# Current TCP pose, robot base frame, mm / degrees (active tool file).
float64 x
float64 y
float64 z
float64 rx
float64 ry
float64 rz

Alarm[] alarms            # only filled while alarming
float32 poll_ms           # wall time the last poll tick spent talking to the controller
