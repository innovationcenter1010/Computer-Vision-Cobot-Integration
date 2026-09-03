# Static description of the connected YRC1000, read ONCE after every
# successful connect (and on /cobot/reload_profile) and published LATCHED on
# /cobot/profile by cobot_control. Everything here is read-only controller
# data plus the currently selected (active) tool. Per-tool CELL values
# (marker offset, pick heights...) live in cobot_setup.yaml `tools`, not here.
#
# header.stamp = read_at. read_count increments on every (re)read so a
# consumer can tell a reload from a duplicate.
std_msgs/Header header
uint32 read_count

bool loaded               # true when at least one controller section was read successfully
string source             # "controller" | "fixture" | "none"
string robot_ip

# ReadSystemInformation(R1)
string model_name
string software_version

ToolFile[] tools          # TOOL.CND, ascending by number; empty when that section failed
UserFrame[] user_frames   # UFRAME.CND, ascending by number; empty when that section failed
string[] cnd_files        # ListFiles(Condition_CND) - diagnostics

# Per-section failures ("system_info: [code] msg", "tool_files: ...",
# "user_frames: ...", "cnd_list: ..."). A failed section is non-fatal.
string[] errors
string[] warnings         # parser warnings (unrecognised lines, odd counts)

# Tooling names configured for THIS cell in cobot_setup.yaml `tools`
# (parallel arrays). Lets the dashboard show which controller tool files
# have a cell profile behind them.
string[] configured_tool_names
int32[] configured_tool_numbers

# Active tool = the tool file (TCP) every pose on /cobot/state,
# /cobot/get_current_pose and every MoveToPose goal without an explicit
# tool_number is expressed in. Republished whenever it changes.
int32 active_tool_number
string active_tool_name       # "none" (tool 0 default) or a cobot_setup.yaml tools key
string active_tool_source     # "setup_default" | "state_file" | "service"
bool active_tool_verified     # tool number was found in the parsed TOOL.CND
