# Derived, read-only system capability, published latched on /task/system_state
# by task_manager (cobot_tasks/system_state.py holds the exact predicates).
# Aggregated from /cobot/state and /health/summary — the task planner never
# sets or changes it; it only reads what other nodes publish.
#
# capability is a BITMASK: OPERATE == SEE | MOVE. A workflow declares which
# bits it needs (WorkflowInfo.requires) and is ready only when
# (capability & requires) == requires.
std_msgs/Header header

uint8 CAP_NONE = 0
uint8 CAP_SEE = 1        # vision usable (camera + calibration); robot not required
uint8 CAP_MOVE = 2       # robot motion usable; vision not required
uint8 CAP_OPERATE = 3    # SEE | MOVE
uint8 capability
string capability_name   # "none" / "see" / "move" / "operate"

# --- individual robot facts (from /cobot/state; all false when not fresh) ---
bool robot_state_fresh   # a /cobot/state sample newer than ~1 s
bool connected
bool servo_on
bool alarming
bool erroring
bool in_hold
bool paused
bool busy
string control_mode      # "teach" / "play" / "remote"; "-" when unknown
float32 robot_state_age_s   # -1 = never heard

# --- individual vision facts (from /health/summary; all false when not fresh) ---
bool health_fresh        # a /health/summary newer than ~3 s
bool camera_ok           # camera_node level ok/warn (streams flowing)
bool calibrated          # localization's camera->robot calibration fitted
bool depth_fresh         # localization depth buffer younger than ~2 s (informational)
bool detector_ok         # detector_node level ok/warn (informational)
float32 health_age_s     # -1 = never heard

string[] move_blockers   # human reasons; empty when the MOVE bit is set
string[] see_blockers    # human reasons; empty when the SEE bit is set

# --- tooling facts (from /cobot/state + localization health) ---
int32 active_tool_number -1        # -1 = unknown / robot not fresh
string active_tool_name
int32 calibration_tool_number -1   # tool the saved calibration was captured with; -1 unknown
bool calibration_tool_mismatch     # both known and different (WARNING, not a blocker)
