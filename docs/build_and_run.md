# Build & run

Quick reference. For full details see [README.md](../README.md).

## Prerequisites

- Docker Desktop
- External camera streamer running on the host (writes ZMQ frames + `vision_workspace/operation/camera_intrinsics.json`)
- Robot reachable at the IP in `cobot_params.yaml` (or run with `cobot:=false`)

## Build and run

```bash
docker compose up --build
```

- Builds the image (a failing C++ unit test fails the build) and starts the one-container stack.
- UI: http://localhost:8000

To rebuild without cache: `docker compose build --no-cache`.

Run without the robot attached:

```bash
ros2 launch cobot_bringup robot.launch.py cobot:=false
```

## Models

Detection models live in `runtime_models/`, bind-mounted into the container — swapping a model **never needs an image rebuild**, just restart `detector_node`.

- `runtime_models/models.yaml` — registry: model id → path, type (`obb`/`bbox`), thresholds
- `runtime_models/<name>/<name>.onnx` + `manifest.yaml` — one directory per exported model
- Only `.onnx` files belong here. `.pt` weights stay on the dev PC (`vision_workspace/weights/`, gitignored) and are exported with `vision_workspace/tools/export_to_onnx.py` (or `export_yoloe.py` for YOLOE models) — see `vision_workspace/README.md`.
- Training never happens in the container.

## Tests

```bash
colcon test --merge-install --packages-select cobot_detector cobot_control --event-handlers console_direct+
colcon test-result --verbose

python3 -m pytest src/cobot_operation/test src/cobot_tasks/test src/cobot_vision/test
```
