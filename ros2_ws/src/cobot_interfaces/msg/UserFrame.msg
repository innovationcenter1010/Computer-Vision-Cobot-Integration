# One YRC1000 user frame, parsed from UFRAME.CND. Display / groundwork only:
# nothing in the stack commands poses in user coordinates yet.
int32 number              # user frame index
string name               # NAME line
int32 tool_number         # TOOL line: tool file the frame was taught with (-1 unknown)
int32 group               # GROUP line (-1 unknown)
float64[] pose            # BUSER x y z rx ry rz in base coordinates (6 values, or empty)
bool defined              # at least one of pose / name / RORG-RXX-RXY was present
KeyValue[] extra          # BASICFRM, BASEP, PULSE, RORG, RXX, RXY, ... as raw text
