# motoman_cpp — RETIRED

The pybind11 wrapper is no longer part of the ROS graph. `cobot_control` is now
a C++ ROS 2 node that links YMConnect directly (see `../cobot_control`).

Kept here, un-built (`COLCON_IGNORE`), for scripting against a live robot
(`vision_workspace/tests/*.py`) and as the reference for the port:

- `src/MotomanRobotBindings.cpp` — the old Python surface.
- `retired_python/` — the previous `cobot_control` Python node and choreography
  (`cobot.py`, `cobot_node.py`). Superseded by `cobot_operation`.
- `MotomanRobot.{hpp,cpp}` moved (with history) to
  `cobot_control/include/cobot_control/motoman_robot.hpp` and
  `cobot_control/src/motoman_robot.cpp`.

To build the extension again, delete `COLCON_IGNORE`; CMake resolves YMConnect
from `../cobot_control/third_party/YMConnect` and the sources from
`../cobot_control`.
