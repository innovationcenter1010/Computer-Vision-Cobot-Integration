# One hardware primitive executed during a choreography (one row of the
# operation report carried in PickCan.Result).
string job            # move / engage_gripper / disengage_gripper / sensor_read / sensor_assert
bool ok
int32 status_code     # 0 ok; 1 not connected; -1 transport (timeout / rejected); else YMConnect code
string message
string payload        # free-form: pose text, bit value, ...
