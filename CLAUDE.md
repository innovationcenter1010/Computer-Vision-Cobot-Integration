# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 (Humble) bin-picking stack: an Orbbec RGB-D camera + ONNX detection models find objects, a fitted camera→robot calibration turns pixels into robot-base millimetres, and a Yaskawa Motoman cobot (YMConnect SDK) picks and places them. Everything runs as one Docker container (eight nodes, one launch file); the camera streamer and the robot controller are external, reached over TCP. [README.md](README.md) is the detailed operator/developer reference; [docs/ros2_architecture.md](docs/ros2_architecture.md) is the design rationale (ignore its "design draft — nothing implemented yet" header; the stack is implemented).

## Commands

Build and run the whole stack (Docker Desktop on Windows is the deployment host):

```bash
docker compose up --build          # builds the image AND runs the C++ unit tests
                                   # (a gtest regression fails the image build)
# UI at http://localhost:8000
```

Inside the container (or any Humble env with the pip deps):

```bash
cd /ws
colcon build --merge-install --symlink-install   # symlink: Python edits are live
source install/setup.bash
ros2 launch cobot_bringup robot.launch.py cobot:=false   # args: camera/vision/cobot/tasks/web/health

# C++ unit tests (detector decode/NMS/letterbox/registry, control profile/setup/tooling parsers)
colcon test --merge-install --packages-select cobot_detector cobot_control --event-handlers console_direct+
colcon test-result --verbose

# Python unit tests — plain pytest, no rclpy, but they need the built
# cobot_interfaces overlay sourced first (they build real messages):
python3 -m pytest src/cobot_operation/test src/cobot_tasks/test src/cobot_vision/test
python3 -m pytest src/cobot_operation/test/test_choreography.py -k <name>   # single test
```

Model training/export runs on the dev PC in `vision_workspace/.venv` — never in the image (see `vision_workspace/README.md`). **Never start a training run without asking the user first**; evaluating and exporting are fine. Exports land in `runtime_models/<name>/<name>.onnx` + `manifest.yaml`, registered in `runtime_models/models.yaml` (bind-mounted; no rebuild to swap a model). `.pt` weights are gitignored and never enter the container.

## Architecture

Data flow: `camera_node` (ZMQ→ROS bridge) → `detector_node` (C++, ONNX Runtime, action `/vision/detect`, pixel space only) → `localization_node` (action `/vision/scan`: depth lookup + fitted 3×4 pixel→robot transform + snake ordering) → `task_manager` (workflows) → `cobot_operation` (pick choreography, action `/cobot/pick_can`) → `cobot_control` (C++, the only process that talks to the robot). `web_node` bridges the browser (FastAPI + websocket); `health_monitor` aggregates per-node heartbeats into `/health/summary`.

Ownership rules the code enforces (violating them is a design bug, not a style choice):

- **`task_manager` decides, never executes or computes.** It runs one workflow at a time from a registry (`ros2_ws/src/cobot_tasks/cobot_tasks/workflows/`). To add a task: write `workflows/<name>.py` subclassing `Workflow` (declares `params`, `clients`, `requires` capability), register it in `workflows/__init__.py` `REGISTRY`. Workflows must not import rclpy — they talk to ROS only through `WorkflowContext` (`context.py`), which is what keeps them unit-testable with a fake.
- **The cobot never sees pixels; the detector never sees millimetres.** `ObbDetection` is pixel-only, `Target` robot-frame-only; only `localization_node` converts. (`Target` carries `pixel_x/y`, `depth_mm` as scan-time provenance for `/vision/verify_target` — the choreography ignores them.)
- **Post-pick depth verification belongs to the workflow (`pick_layer`), never the choreography**, and only after the PickCan result — the arm occludes the camera mid-pick.
- **`localization_node` refuses to work uncalibrated** (no `calibration.yaml` → detections dropped). There is deliberately no fallback; don't add one. A fit whose RMS exceeds `calibration_max_rms_mm` is rejected and the old transform kept.
- **`/vision/targets` is data, not a trigger** — nothing moves because a topic was published; motion happens only via action goals from a running workflow.
- Dependency rule: every package depends only on `cobot_interfaces` (+ stock msgs); `cobot_tasks` additionally on `cobot_health`; nothing depends on `cobot_bringup`.

Tooling model: `cobot_control` reads the YRC1000 profile (tool files, user frames) once per connect and publishes it latched on `/cobot/profile` — strictly read-only. Every pose carries an explicit tool file number; per-tool cell values (marker offset, pick/approach heights) live in `cobot_setup.yaml` `tools.<name>`. Only `tools.none` (tool 0) may have a `tip_offset_mm` — with a real tool file active the controller TCP already is the tip, and the TCP math must never be duplicated in Python.

## Configuration and state — three distinct places

| Where | What | Nature |
|---|---|---|
| `ros2_ws/src/cobot_bringup/config/` (bind-mounted) | `cobot_params.yaml` (all node params), `cobot_setup.yaml` (named IO, motion profiles, tools), `trajectories.json` (v2 seed), generated `calibration.yaml` | editable config; edits apply on container restart, no rebuild |
| `runtime_config/` (bind-mounted, gitignored) | live `trajectories.json`, `tooling_state.yaml` (active tool), `controller_profile/` dumps | runtime-WRITTEN per-cell state — never edit while the stack runs |
| `runtime_models/` (bind-mounted) | `models.yaml` registry + ONNX exports | what `detector_node` loads |

Gotchas: `calibration_z_levels_mm` and friends must be float arrays (`[500.0, ...]` — an int array fails ROS param loading). `ros2_ws/src/motoman_cpp` is retired (`COLCON_IGNORE`, kept for scripting); `legacy/` is the pre-ROS monolith, reference only — don't build or extend either. Hardware-facing behaviour (kinematics, tool-file layout, IO addresses) is largely unverified against the real cell; the shipped `io.sensors` addresses are placeholders.

## Keeping docs honest

README.md documents behaviour in detail — when you change interfaces, workflows, parameters, or the web contract, update the matching README section (and `docs/ros2_architecture.md` if the design changes). Node docstrings (`task_manager_node.py`, `web_node.py`, etc.) double as interface contracts; keep them current.
