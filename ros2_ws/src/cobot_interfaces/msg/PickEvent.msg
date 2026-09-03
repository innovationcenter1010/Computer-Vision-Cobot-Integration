# One per PickCan attempt inside a pick_layer cycle. Published by task_manager
# on /task/pick_event after the action result AND the post-pick depth check
# ("second depth scan" via /vision/verify_target). The UI colors the matching
# detection box; detection_index links back to the scan's ObbDetectionArray.
string OUTCOME_PICKED = "picked"
string OUTCOME_FAILED = "failed"
string OUTCOME_SKIPPED = "skipped"

std_msgs/Header header
uint32 index               # 1-based position in the layer
uint32 total
int32 detection_index      # index into the scan's ObbDetectionArray; -1 unknown
Target target
string outcome
bool verify_ran            # verify_target answered in time AND took a measurement (success)
bool verified              # verify ran AND reported the can gone (present == false)
float64 measured_z         # robot-frame z now measured at the target; -1 if none
float64 verify_delta_mm    # measured - scan-time depth at the target pixel; 0 if verify did not run
string verify_message      # why verify could not measure (unavailable / timeout / service message); "" otherwise
string message
