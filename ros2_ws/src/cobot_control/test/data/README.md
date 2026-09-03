# cobot_control parser fixtures

`TOOL.CND`, `UFRAME.CND` and `system_info.yaml` here drive the
`test_controller_profile` gtests and the zero-hardware `profile_source: fixture`
mode of the node.

**STATUS: SYNTHETIC.** These files were hand-written from the documented field
lists (TOOL: NAME X Y Z RX RY RZ W XG YG ZG IX IY IZ; UFRAME: NAME TOOL GROUP
BASICFRM BASEP PULSE RORG RXX RXY BUSER). Replace them with real dumps as soon
as the controller is reachable (step 0 of the tooling plan):

    ros2 service call /cobot/reload_profile std_srvs/srv/Trigger
    cp runtime_config/controller_profile/{TOOL.CND,UFRAME.CND,system_info.yaml} \
       ros2_ws/src/cobot_control/test/data/

then update the expected values in `test/test_controller_profile.cpp`. If the
service reports an RS7 error, set parameter RS7 = 2 on the pendant
(management mode) — file reads outside REMOTE mode are refused otherwise.
