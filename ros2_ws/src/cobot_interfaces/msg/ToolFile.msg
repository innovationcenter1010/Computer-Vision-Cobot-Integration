# One YRC1000 tool file, parsed from TOOL.CND by cobot_control after connect.
# READ-ONLY snapshot of what the controller already knows; cobot_control never
# writes tool data. Field names follow the pendant's TOOL screen.
int32 number              # tool file index (0-63). Tool 0 is the "none" default.
string name               # NAME line ("" when the controller has none)
float64 x                 # TCP offset from the flange, mm, tool frame
float64 y
float64 z
float64 rx                # TCP orientation, degrees
float64 ry
float64 rz
float64 weight_kg         # W
float64[] cog_mm          # XG YG ZG (3 values, or empty when absent)
float64[] inertia         # IX IY IZ (3 values, or empty when absent)
bool is_identity          # x=y=z=rx=ry=rz=0 -> behaves like tool 0 (an unused slot)
KeyValue[] extra          # any other "///KEY value" lines the parser saw (raw text)
