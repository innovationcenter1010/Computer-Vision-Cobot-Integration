# ROS2 Architecture — Computer Vision Cobot Integration

Design document for migrating the `cobot_application` asyncio monolith to a ROS2 (Humble)
node graph. Covers the target architecture, message contracts, the bin-picking operating
cycle, configuration strategy, and the Docker build.

Status: **design draft — nothing implemented yet.**

---

## 1. Current system (migration baseline)

The legacy application (`legacy/main.py`, `legacy/shared_data.py`) runs four controllers as
asyncio tasks in one process, communicating through `Py_network` — one `asyncio.Queue` per
module, carrying `P2P_command(sender, receiver, message_enum, value)` envelopes.

| Module | Role | Key implementation |
|---|---|---|
| camera | ZMQ SUB bridge to external color/depth streamer (5550/5551), latest-frame cache | `camera/camera_streamer.py` |
| vision | YOLO-OBB detection + homography px→mm + rz + pick ordering | `vision_system/run_operation_scan.py` |
| cobot | Motoman control via **pybind** `motoman_cpp` (YMConnect), pick-and-place choreography | `cobot/cobot.py` |
| web | FastAPI + websocket UI, command injection, config editor | `web_app/webpage_controller.py` |

Runtime choreography:

```
Browser ──ws {receiver,command,payload}──▶ web
   web ──snapshot──▶ camera ──update_image(DualFrame)──▶ web ──base64 ws──▶ Browser
   web ──detect────▶ vision ──pick_layer(list[OBBDetection])──▶ cobot ──▶ motoman_cpp (pybind)
camera ZMQ SUB ◀── external color/depth streamer
```

Three payload shapes cross the queues: scalars (UI), `DualFrame` (two numpy images),
`list[OBBDetection]` (pose-bearing detections).

---

## 2. Target node graph

Six nodes plus a bringup package. Design decisions locked in:

- **The cobot is two nodes** — `cobot_control` (C++, the only YMConnect link: hardware
  primitives, 10 Hz state poll, pause / cancel) and `cobot_operation` (Python choreography).
  The pybind extension `motoman_cpp` is retired (§7).
- **Vision splits into two nodes** — ML (YOLO) and geometry (localization) are separate.
- **A task manager owns the state machine** (§11) — the cobot nodes are action servers;
  nothing moves because a topic was published. `/vision/targets` is data, not a trigger.
- **All frames flow as topics** — vision *and* the web UI subscribe to the same camera streams.

```
              ZMQ 5550/5551
 external ──▶ camera_node ─▶ /camera/color/image_raw ──┬─▶ yolo_node
                          └▶ /camera/depth/image_raw ──┼────────────────▶ localization_node
                                                       │                    ▲            │
                                                       └─▶ web_node         │            │
                                                                            │            │
 browser ══ws══▶ web_node                                                   │            │
                    │◀──────────── /vision/detections_px (pixel space) ─────┘            │
                    │◀──┬───────── /vision/targets (robot space) ◀────────────────────┬──┘
                    │   │                                                             │
                    ├── /task/start|stop|calibrate (Trigger) ──▶ task_manager ◀───────┘
                    │◀──────── /task/status + /task/layer_report ──┤ │
                    │                              /vision/scan ◀──┘ │ (+ calibration srvs)
                    │                                                ▼
                    │              PickCan action ──▶ cobot_operation ─▶ cobot_control ─▶ robot
                    │              MoveToPose action ───────────────────▶ (YMConnect)
                    │◀──────────── /cobot/status (latched) + /cobot/state (10 Hz) ─┘
```

### Workspace layout

```
ros2_ws/src/
├── cobot_interfaces/      # msgs/srvs/actions only — depends on nothing local
├── cobot_camera/          # ament_python: camera_streamer.py + camera_node
├── cobot_vision/          # ament_python: yolo_node + localization_node (two executables)
├── cobot_control/         # ament_cmake: C++ node on YMConnect, SDK vendored in third_party/ (§7)
├── cobot_operation/       # ament_python: pick choreography (PickCan) over cobot_control primitives
├── motoman_cpp/           # RETIRED pybind11 module (COLCON_IGNORE) + old Python cobot node
├── cobot_tasks/           # ament_python: task_manager — state machine + sequencing (§11)
├── cobot_web/             # ament_python: FastAPI app + web_node
└── cobot_bringup/         # launch files + config YAMLs
```

Dependency rule: every package depends only on `cobot_interfaces` + standard messages
(`sensor_msgs`, `std_srvs`). `cobot_control` is C++ and links the vendored YMConnect SDK.
`cobot_tasks` depends on `cobot_interfaces` + rclpy ONLY (§11). Nothing depends on
`cobot_bringup`.

---

## 3. Node specifications

### 3.1 `camera_node` (pkg `cobot_camera`)

`CameraStream` (ZMQ SUB threads) moves in unchanged; its `on_color`/`on_depth` callbacks
become publisher calls.

- Publishes `/camera/color/image_raw` — `sensor_msgs/Image`, `rgb8`
- Publishes `/camera/depth/image_raw` — `sensor_msgs/Image`, `mono16` (depth is already int16 mm)
- Publishes `/camera/color/camera_info` — `sensor_msgs/CameraInfo`, latched. Relays the
  camera-SDK **intrinsics** the streamer writes to a small JSON (`intrinsics_file`, in the
  bind-mounted vision_workspace). Static data → published once and re-read only on file
  change, not per frame. localization consumes it for the extrinsic startup calibration (§10)
- QoS: images best-effort keep-last(1) (freshest wins); CameraInfo reliable transient-local
- ZMQ endpoints become **parameters** (they are currently hardcoded in `camera_streamer.py`;
  the `camera` section of `config.JSON` was never actually read)
- The legacy `snapshot` / `get_stream_provider` commands disappear — consumers subscribe.
  `frame_saver` can return later as an optional `/camera/save_frame` Trigger service.

### 3.2 `yolo_node` (pkg `cobot_vision`) — pure perception, no geometry

- Subscribes `/camera/color/image_raw`, caches the latest frame
- Service `/vision/scan` (`std_srvs/Trigger`) → runs YOLO-OBB on the cached frame in a
  worker thread (replaces `asyncio.to_thread`), returns immediately; results flow on the topic
- Publishes `/vision/detections_px` (`ObbDetectionArray`) — class, confidence, pixel corners,
  pixel center. **Header stamp = the color frame the model ran on.**
- Pixel-distance duplicate removal (`remove_duplicate_detections`) happens here
- Takes: `yolo_obb_detector.py`, annotated-image saving. `train_yolo_obb.py` is an
  **offline tool** in `vision_workspace/tools/`, not a node.
- Service `/vision/locate_dot` (`LocateDot`) — finds the red calibration marker with a
  second model (`red_dot_best.pt`), lazy-loaded on first call so normal startup stays fast.
  Accepts either detection head (OBB or axis-aligned boxes). See §10.
- Only node that imports `ultralytics` — the heavy dependency stays quarantined
- Upgrade path: promote `/vision/scan` to an action only if progress/cancel is ever needed

### 3.3 `localization_node` (pkg `cobot_vision`) — pure geometry, no ML

- Subscribes `/vision/detections_px` + `/camera/depth/image_raw` (small ring buffer of
  recent depth frames)
- On detections: pick the depth frame nearest the detection stamp (start with "latest" —
  the scene is static during a scan; nearest-stamp is a drop-in upgrade since the stamp is
  already carried)
- **Requires a fitted camera→robot calibration** (`calibration.yaml`, see §10). Uncalibrated
  → detections are dropped with a loud log and `verify_target` errors. There is deliberately
  no homography fallback: an unmeasured transform must not move the robot.
- Per detection:
  - depth at the can center + fitted transform → real x/y/z (`calibration.py`) — one
    consistent 3D model replaces the legacy planar homography AND the hardcoded
    `LAYER_Z1..Z7` table
  - `real_rz` from OBB corners (`rotation.py`), corners mapped through the same transform
  - pick ordering (`pathing.py`) — a **robot-frame snake**: rows grouped along one robot
    axis (`pathing_row_axis`), serpentine along the other. No bin-corner coordinates, so
    the order survives the bin shifting
- Publishes `/vision/targets` (`CanTargetArray`) — consumed by `task_manager` (data for the
  pick cycle it requested) and `web_node` (display)
- Services (cheap depth reads, no ML — see §5):
  - `/vision/verify_target` — is a can still at (x, y), and at what measured z?
  - `/vision/get_layer_height` — median live depth **at the last scan's detection pixels**
    → current layer index. Sampling where the cans were seen (not a fixed image ROI) keeps
    the height check valid when the bin moves
- Calibration math services (`task_manager` drives the robot between calls — see §10):
  - `/vision/calibration_begin` · `/vision/calibration_sample` · `/vision/calibration_compute`

### 3.4a `cobot_control` (pkg `cobot_control`) — hardware primitives, C++, YMConnect

The only process on the YMConnect link. No decisions, no choreography, no task state.
The SDK wrapper (`motoman_robot.cpp`) has no ROS in it and serialises every SDK call on
one mutex; the node owns the timers and the move thread.

- Action `/cobot/move_to_pose` (`MoveToPose`) — one supervised move. Named motion
  profile from `cobot_setup.yaml` (joint / **linear**, speed, accel/decel, accuracy,
  tool file) + per-goal overrides; IK reachability pre-check; feedback
  `progress_percent` / `steps_remaining` / `paused`. **Cancel is immediate** —
  `MotionStop(true)`, trajectory cleared, goal CANCELED.
- `/cobot/pause`, `/cobot/resume` (Trigger) — hold the arm in place (`MotionStop(false)`,
  resumed with `MotionStart()`, the SDK's documented pause) or, with no move in flight,
  make the next move wait. `pause_uses_hold` switches to the controller HOLD signal.
- `/cobot/motion_stop` — immediate controlled stop (not an e-stop).
- Services for every other primitive: `set_output_bit` / `read_io_bit` (by **name** from
  `cobot_setup.yaml` or raw address), `set_motion_profile`, `get_current_pose`,
  `get_active_alarms`, `clear_faults`, `clear_trajectory`, `servo_on/off`, `reconnect`,
  int / position variables. Every response: `success`, `status_code` (0 ok; 1 not
  connected; else the YMConnect code), `message`.
- **The 10 Hz poll** (`poll_period_s`) publishes `/cobot/state` (`CobotState`): `ReadState`
  (servo / hold / alarm / erroring / Teach-Play-Remote), one `ReadByte` per distinct IO group
  so every named sensor and output costs one round-trip, the current TCP pose, active
  alarms while alarming, `poll_ms`. The move loop consumes this instead of polling the
  controller itself. The same timer is the reconnect loop.
- One motion at a time: a goal arriving while another executes is REJECTED, as is any goal
  while alarming, servos off or in Teach mode.
- Publishes `/cobot/status` (`CobotStatus`, latched) on transitions — hardware status only.
- Threading: `MultiThreadedExecutor`; poll timer (exclusive group), services + action
  (reentrant group), one `std::thread` per executing move (joined before the next).

### 3.4b `cobot_operation` (pkg `cobot_operation`) — pick choreography, Python

`rclpy` + `cobot_interfaces` only — never imports robot code. Owns what the legacy
`cobot.py` owned above the hardware line: the phase order, tool profiles
(`cobot_setup.yaml` `tools`), teach-point trajectories, `last_placed`, the report.

- Action `/cobot/pick_can` (`PickCan`) — pick + place ONE can by chaining `MoveToPose`,
  `set_output_bit` and `/cobot/state` sensor reads; feedback = phase; the result carries the
  full report (`jobs[]`, `sensor_asserts[]`, `phase`). **Cancel is immediate**: the in-flight
  `MoveToPose` is canceled (controller stop) and the choreography returns at its next
  boundary — the can may still be gripped.
- Pause: `cobot_control`'s `/cobot/pause` holds the arm; the choreography sees
  `CobotState.paused` and waits before its next primitive. One service pauses both layers.
- Execute callback is a coroutine awaiting the client futures (one reentrant group on a
  `MultiThreadedExecutor`) — no thread is parked, so goal / cancel stay serviceable.
- The choreography runs against a `Primitives` protocol; `test/test_choreography.py`
  drives it with a fake (phase order, abort points, place selection, gating).


### 3.5 `task_manager` (pkg `cobot_tasks`) — state machine + sequencing

See §11. Owns start/stop/calibrate, the pick-layer and calibration loops, and
`/task/status` — the single source of operational truth.

### 3.6 `web_node` (pkg `cobot_web`)

FastAPI app, templates, static files, `/api/config`, `/ws` — all preserved. The node is the
browser⇄graph bridge:

- The legacy `COMMAND_TRANSLATOR` string→enum table becomes a string→(service client) table
  of the same shape — operations route to `/task/*` (the task manager), display-only
  commands (`/vision/scan`, `/camera/save_frames`) go direct
- Subscribes both image topics; throttles to ~5–10 Hz, JPEG-encodes, base64 over the existing
  websocket (the old `update_image` handler, fed by subscription instead of queue)
- Subscribes `/vision/detections_px`, `/vision/targets`, `/task/status`,
  `/task/layer_report` and forwards to the browser — the UI finally gets operation feedback
  (legacy reports only went to stdout)
- Config editor becomes a parameter client (see §6)
- Integration note: uvicorn's asyncio loop and the rclpy executor coexist by spinning rclpy
  in a background thread

---

## 4. Interface package — `cobot_interfaces`

```
msg/ObbDetection.msg        # class_id, confidence, center px, corners px    (pixel space)
msg/ObbDetectionArray.msg   # header (stamp = source color frame) + image w/h + detections
msg/CanTarget.msg           # real_x, real_y, real_z, real_rz + class_id, confidence
msg/CanTargetArray.msg      # header + targets
msg/CobotStatus.msg         # HARDWARE status: disconnected/idle/busy/error + detail
msg/TaskStatus.msg          # OPERATIONAL state: idle/scanning/picking/calibrating/
                            # stopping/error + detail + active_task + progress   (§11)
msg/LayerReport.msg         # successes, failures, overall_ok
srv/VerifyTarget.srv        # CanTarget → bool present, float64 measured_z
srv/GetLayerHeight.srv      # (samples last detection pixels) → float64 median_z, int8 layer_index
srv/LocateDot.srv           # () → found, dot center px (u, v), confidence     (§10)
srv/CalibrationSample.srv   # commanded TCP (x, y, z) → success, sample_count  (§10)
action/PickCan.action       # goal CanTarget → success/message; feedback phase (§11)
action/MoveToPose.action    # goal x,y,z,rx,ry,rz → success/message            (§11)
```

Everything else uses stock types: `sensor_msgs/Image`, `std_srvs/Trigger`. The pixel/robot
split in the messages mirrors the node split — **the cobot never sees pixels, YOLO never
sees millimeters**.

QoS contract: images best-effort keep-last(1); detections/targets/status/reports reliable.

---

## 5. Bin-picking operating cycle — tiered sensing

YOLO must not run per pick. The design uses three tiers of sensing cost; the depth stream
(already continuous, already free) supplies the missing middle tier:

| Tier | Sensor | Cost | Question |
|---|---|---|---|
| 0 | Gripper sensors (`existance_check`) | free | "Did I actually grab something?" |
| 1 | **Depth sampling** (localization_node srvs) | ~µs, no ML | "Is a can still at (x,y)? What height is the layer, sampled where the cans were seen?" |
| 2 | **YOLO scan** (yolo_node) | expensive | "Where is everything and how is it rotated?" |

The task manager drives the cycle (pick execution = one `PickCan` goal per can):

```
  layer start ──▶ call /vision/scan ──▶ receive /vision/targets (N cans, ordered)
                        │
      ┌─────────────────▼──────────────────────┐
      │ for each target in layer:              │
      │   1. /vision/verify_target (depth)  ───┼──▶ can still at expected x,y,z?
      │        └─ mismatch? ──▶ RESCAN (tier 2)│
      │   2. pick  (gripper existance_check)   │
      │        └─ grab failed? ──▶ RESCAN      │
      │   3. place (bin is UNOCCLUDED now —    │
      │      free window for any rescan)       │
      └─────────────────┬──────────────────────┘
                        ▼
  targets exhausted ──▶ /vision/get_layer_height (depth at last detection pixels)
                        ├─ dropped ~can_height? ──▶ scan next layer
                        ├─ leftovers at old height? ──▶ rescan same layer
                        └─ at bin floor? ──▶ BIN_COMPLETE
```

Properties:

- **YOLO runs once per layer in the happy path.** Rescans are event-driven (verification
  mismatch, failed grab), not scheduled.
- **Every pick is verified against live depth** before the gripper commits — stale
  detections are caught before, not after. This fixes the failure mode of picking a layer
  blind: cans shift when neighbors are pulled.
- **Layer completion is measured, not inferred** — the depth plane dropping by
  `can_height_mm` replaces the legacy 70 %-count threshold heuristic (`bin.py`).
- **Rescan latency hides inside the place motion** — the controller requests scans while the
  arm is away from the bin, which also solves occlusion: vision never needs to detect
  whether an elbow is in frame.
- Known wiring gap to close during implementation: `cobot.py` currently hardcodes pick z at
  300/401 mm and ignores `real_z` — measured depth should feed the pick height.

---

## 6. Configuration strategy

Legacy: one `Config` object loaded from `config.JSON`, passed by reference everywhere; the
web app edits it at runtime (write file + in-memory `update()`). Live updates only reached
modules that re-sliced per operation — vision picked them up, the cobot silently kept stale
trajectories.

`config.JSON` holds three kinds of data, mapped differently:

| Kind | Examples | ROS2 home |
|---|---|---|
| Tunable scalars | YOLO conf/iou/max_det, `can_height_mm`, `layer_threshold`, row tolerance, `robot_ip`, tool-head offsets | **Declared ROS parameters** per owning node |
| Structured data | `cobot_place_trajectories` (named waypoint lists) | trajectories stay a **JSON file** referenced by a path param (bin-corner matrices retired — pick order is a robot-frame snake, layer height samples the detection pixels) |
| Offline / dead | `TRAIN_MODEL`, `TRAINING_*` (offline tool); `camera` section (never read — endpoints were hardcoded) | Not ROS params. Training config goes with the offline tool; camera endpoints become real params for the first time |

One YAML in `cobot_bringup` replaces the global object:

```yaml
# cobot_bringup/config/cobot_params.yaml
camera_node:
  ros__parameters:
    color_endpoint: "tcp://host.docker.internal:5550"
    depth_endpoint: "tcp://host.docker.internal:5551"

yolo_node:
  ros__parameters:
    model_path: "/ws/vision_workspace/weights/cans_obb_best.pt"
    confidence: 0.50
    iou: 0.4
    max_detections: 20
    image_size: 640
    duplicate_center_distance_px: 50.0

localization_node:
  ros__parameters:
    # Spatial setup data lives HERE, never as code defaults (defaults are
    # zeros = unconfigured; dependent features refuse to run until set).
    # No bin corners: pick order is a robot-frame snake and layer height is
    # sampled at the live detection pixels, so both survive the bin moving.
    pathing_row_axis: "y"               # rows along robot Y, serpentine along X
    pathing_row_tolerance_mm: 50.0
    can_height_mm: 87.0
    marker_offset_mm: [0.0, 0.0, 0.0]   # red dot off the toolhead point (§10)

cobot_control:
  ros__parameters:
    robot_ip: "192.168.1.31"
    tool_offset_mm: [0.0, 0.0, 0.0]     # tool TIP off the toolhead point (§10)
    tool_head_compression_distance_mm: 50.0
    trajectories_file: "config/trajectories.json"

task_manager:
  ros__parameters:
    scan_timeout_sec: 30.0
    # calibration pose grid (§10/§11) — zeros = unconfigured, calibrate refuses
    calibration_x_range_mm: [0.0, 0.0]
    calibration_y_range_mm: [0.0, 0.0]
    calibration_z_levels_mm: [0.0]
```

### Web config editor

`web_node` becomes a **parameter client** instead of a shared-object mutator:

- `GET /api/config` → aggregate each node's built-in `list_parameters`/`get_parameters`
  services (free on every rclpy node)
- `POST /api/config` → diff, then `set_parameters` on the owning node. Each node registers
  an on-set-parameter callback that **applies live or explicitly rejects** (e.g. YOLO
  thresholds: live; `robot_ip`: reject/restart) — making update semantics explicit fixes
  the legacy stale-reference problem
- Persistence: web_node writes accepted changes back to `cobot_params.yaml` on the
  bind-mounted config volume

```
LEGACY:  browser → POST → write config.JSON → Config.update() → (maybe) picked up
ROS2:    browser → POST → web_node → set_parameters on owning node → callback applies/rejects
                                   └→ write cobot_params.yaml (persistence)
```

Bonus: `ros2 param set /yolo_node confidence 0.7` works from any terminal, typed and logged.

---

## 7. The YMConnect SDK inside colcon

`cobot_control` is an `ament_cmake` package that vendors the SDK
(`third_party/YMConnect/{YMConnect.h, libYMConnect.so}`, moved with history from
`motoman_cpp`) as an IMPORTED shared library. `libYMConnect.so` is installed next to
`cobot_control_node` (`lib/cobot_control/`) and resolved through an `$ORIGIN` RPATH — the
same trick the pybind build used, now for an executable. `yaml-cpp` reads
`cobot_setup.yaml`.

The pybind module `motoman_cpp` is **retired, not deleted**: the package directory carries a
`COLCON_IGNORE`, its CMake points at the SDK and wrapper sources in `cobot_control`, and the
old Python `cobot_control.py` / `cobot.py` live under `motoman_cpp/retired_python/`. Delete the
`COLCON_IGNORE` to build the module for the standalone scripts in `vision_workspace/tests/`.

What the port changed versus the pybind wrapper (all in `motoman_robot.cpp`):

- no watchdog thread (the node's poll timer supervises the link); no `std::any`; no `std::cout`
- `OpenConnection` result checked before use; every call goes through a lock + null check
- IO reads say which `IOType` they read (the old `read_general_output_bit` read inputs)
- joint vs linear moves via the SDK's `JointMotion` / `LinearMotion`, so the speed unit
  always matches the `SpeedType`; accuracy and tool file per profile
- `ReadState`, `ReadByte`, `ReadPositionData`, `GetActiveAlarms`, `ClearAllFaults`, `SetHold`,
  `DisplayStringToPendant`, `ConvertPositionFromCartesian` (fixed `IsOk` inversion) are used
- `MotionStop(false)` + `MotionStart()` implement pause / resume

---


## 8. Docker build

### 8.1 Layer-cache policy (carried over from the legacy dockerfile, adapted)

Ordering rule, most stable → least stable. **The ultralytics layer is by far the longest
build step (it pulls torch) and rarely changes — it is pinned as the FIRST pip layer, before
any manifest or source copy, so no package.xml edit or source change ever re-downloads it:**

1. apt packages — never change
2. **`pip install ultralytics` — own layer, earliest possible, always cached**
3. remaining pip deps — rarely change
4. `package.xml` manifests → `rosdep install` — change when a package's deps change
5. workspace source → `colcon build` — changes constantly

Pin the ultralytics version (e.g. `ultralytics==X.Y.Z`) so that when the cache *is* busted,
the rebuild is reproducible.

### 8.2 Dockerfile shape

```dockerfile
# ============================================================
# Stage 1: builder
# ============================================================
FROM ros:humble AS builder
ENV DEBIAN_FRONTEND=noninteractive

# ---- 1. apt: never changes ----
RUN apt-get update && apt-get install -y \
    build-essential cmake git \
    python3-pip python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ---- 2. ultralytics: heaviest layer, cached before everything else ----
RUN pip3 install --no-cache-dir ultralytics

# ---- 3. remaining pip deps ----
RUN pip3 install --no-cache-dir \
    opencv-python-headless fastapi "uvicorn[standard]" pyzmq jinja2

WORKDIR /ws

# ---- 4. rosdep layer, keyed on manifests only ----
COPY ros2_ws/src/cobot_interfaces/package.xml   src/cobot_interfaces/package.xml
COPY ros2_ws/src/cobot_camera/package.xml       src/cobot_camera/package.xml
COPY ros2_ws/src/cobot_vision/package.xml       src/cobot_vision/package.xml
COPY ros2_ws/src/cobot_control/package.xml      src/cobot_control/package.xml
COPY ros2_ws/src/cobot_web/package.xml          src/cobot_web/package.xml
COPY ros2_ws/src/cobot_bringup/package.xml      src/cobot_bringup/package.xml
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf src

# ---- 5. source + build ----
COPY ros2_ws/src src
RUN . /opt/ros/humble/setup.sh && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ============================================================
# Stage 2: runtime
# ============================================================
FROM ros:humble-ros-core AS runtime
ENV DEBIAN_FRONTEND=noninteractive

# ---- 1. apt runtime libs ----
RUN apt-get update && apt-get install -y \
    libgl1 libglib2.0-0 python3-pip \
    ros-humble-cv-bridge ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# ---- 2. ultralytics: same earliest-layer rule as builder ----
RUN pip3 install --no-cache-dir ultralytics

# ---- 3. remaining pip deps ----
RUN pip3 install --no-cache-dir \
    opencv-python-headless fastapi "uvicorn[standard]" pyzmq jinja2

# ---- install tree only: no compilers, no source ----
COPY --from=builder /ws/install /ws/install
COPY docker/entrypoint.sh /entrypoint.sh   # sources /opt/ros + /ws/install, exec "$@"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONUNBUFFERED=1 \
    YOLO_CONFIG_DIR=/tmp/Ultralytics

EXPOSE 8000
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "cobot_bringup", "robot.launch.py"]
```

Notes:

- ROS Humble pins Python **3.10** (legacy image was 3.11) — all current deps are compatible.
- Only the `install/` tree crosses to runtime: launch files, message definitions, and the
  compiled `.so` in one artifact. Same lean-image outcome as the legacy two-stage build.
- **Do not bake model weights.** The legacy image baked `cans_obb_best.pt`, but the
  `vision_workspace` bind mount shadowed it at runtime anyway — the volume is the source of
  truth; `model_path` is a parameter.

### 8.3 Compose

One service running the bringup launch file — a drop-in evolution of the legacy compose:

```yaml
services:
  cobot-ros:
    build: { context: ., dockerfile: docker/Dockerfile }
    image: cobot-ros
    pull_policy: never
    restart: unless-stopped
    command: ["ros2", "launch", "cobot_bringup", "robot.launch.py"]
    ports: ["8000:8000"]                      # web only — 5550/5551 were never listeners
    volumes:
      - ./vision_workspace:/ws/vision_workspace
      - ./ros2_ws/src/cobot_bringup/config:/ws/config   # params editable without rebuild
    environment:
      RMW_IMPLEMENTATION: rmw_cyclonedds_cpp
      ROS_DOMAIN_ID: "0"
```

- **Single container, not per-node services.** Deployment host is Docker Desktop for Windows
  (`host.docker.internal`), where `network_mode: host` — the standard DDS-in-containers fix —
  is unavailable. Per-node crash isolation comes from `respawn=True` per node in the launch
  file instead. Split into multiple services only if nodes move to different machines
  (container↔container DDS on a shared bridge network does work if ever needed).
- The ZMQ camera source stays an external host process; `camera_node` bridges it into the
  graph via `host.docker.internal`.
- Legacy `privileged: true`, `/dev/bus/usb`, and the AstraSDK runtime libs were leftovers
  from the retired `astra_cpp` USB-camera path — all droppable (camera is ZMQ, Motoman is TCP).

### 8.4 Dev loop

Legacy `pip install -e` ⇒ `colcon build --symlink-install` inside the container with
`ros2_ws/src` bind-mounted (devcontainer on the same image). Python edits are live without
an image rebuild.

---

## 9. Migration map

### Code that moves (mostly unchanged)

| Legacy | Destination |
|---|---|
| `camera/camera_streamer.py` | `cobot_camera` (endpoints become params) |
| `camera/frame_saver.py` | `cobot_camera` (optional save service, later) |
| `vision_system/yolo_obb_detector.py` | `cobot_vision` / yolo_node |
| `vision_system/rotation.py`, `pathing.py` | `cobot_vision` / localization_node |
| `vision_system/homography.py` | retired — replaced by the fitted 3D calibration (`calibration.py`, §10) |
| `vision_system/train_yolo_obb.py` | `vision_workspace/tools/train_yolo_obb.py` (offline tool, dev-PC .venv) |
| `cobot/cobot.py` choreography | `cobot_operation/choreography.py` (against a `Primitives` protocol) |
| `cpp/` `MotomanRobot.cpp` + YMConnect | `cobot_control` C++ node (`motoman_robot.cpp`, `third_party/YMConnect`) |
| `cpp/` pybind bindings | retired in `motoman_cpp/` (`COLCON_IGNORE`) |
| `web_app/` templates, static, FastAPI routes | `cobot_web` |

Required code changes found during analysis:

- `run_YOLO(image, …)` ignores its `image` argument and always reads the inbox folder —
  must actually pass the subscribed frame through.
- `cobot.py` hardcodes pick z (300/401 mm) — wire measured `real_z` in.
- the ROS graph no longer imports `motoman_cpp`; `cobot_operation` talks to `cobot_control` over actions/services/topics.

### Code that dies

| Legacy | Replaced by |
|---|---|
| `Py_network`, `P2P_command`, command enums, `COMMAND_REGISTRY` | the ROS graph |
| `main.py` | `robot.launch.py` |
| `web_app/web_server.py` | already-dead earlier draft of `webpage_controller.py` |
| camera `snapshot`/`get_stream_provider`, vision `convert_to_jpeg` | frame topics |
| `bin.py` layer heuristics (stale imports, `LAYER_Z` table) | measured depth (§5) + bin state in `task_manager` |
| `Config` shared object | per-node parameters + bringup YAML (§6) |

---

## 10. Camera→robot calibration

Replaces the hand-entered 4-corner homography and the separate `depth_at_top_layer`
guess with **one fitted 3D transform**, measured by the robot itself: a red dot is
mounted near the tool head, the robot parks it at known positions, the camera reports
where it sees it, and localization solves for the mapping. Model: `red_dot_best.pt`.

### 10.1 The math (`cobot_vision/calibration.py` — pure, no ROS imports)

A pinhole camera composed with a rigid camera→robot transform is **exactly linear in
the features `[u·d, v·d, d, 1]`** (back-projection is `X_cam = (u−cx)·d/fx`). One 3×4
matrix `M` maps pixel + depth straight to robot base mm:

```
[X, Y, Z]ᵀ = M · [u·d,  v·d,  d,  1]ᵀ        M = [ R·B | t ]
```

`M` factors into the two physical parts: **intrinsics** `B` (a static 3×3 holding
`fx, fy, cx, cy`) and **extrinsics** `R, t` (the camera pose vs the cobot base — the
only part that drifts when the camera is bumped). Two fitters:

- **`fit()` — full monolithic.** Solves all 12 numbers at once, absorbing intrinsics; no
  CameraInfo needed. **Requires depth variation** (≥30 mm span) or it is rejected — with
  one height the feature matrix is rank-3 and Z is unconstrained ("sample at multiple
  heights", enforced in math).
- **`fit_extrinsic()` — Kabsch, with known intrinsics.** Back-projects each sample to a
  camera-frame 3D point using the fixed intrinsics, then solves the rigid transform
  `{P_cam}→{P_robot}` in closed form (SVD). Only **6 unknowns**, so ~4 poses suffice and
  the depth-span rule **relaxes** (Z = d is measured, not fit). Returns a *pure rotation*,
  so it can't drift into unphysical scale/shear. This is the small startup fit.

Per-axis RMS residuals are the honest quality measure either way; a fit above
`calibration_max_rms_mm` is rejected and the old transform kept. The inverse (robot xyz →
expected pixel + depth) is closed-form and powers `/vision/verify_target`.

### 10.1a Best of both worlds — static intrinsics, dynamic extrinsics

Intrinsics are bolted into the sensor; only the camera *pose* moves. So they are sourced
**once, statically** and the pose is re-fit **cheaply, often**:

- **Intrinsics (static):** the camera SDK already knows `fx, fy, cx, cy`. The external
  streamer writes them to `intrinsics_file`; `camera_node` relays them as latched
  `/camera/color/camera_info`; localization caches them and also persists them into
  `calibration.yaml` (`camera_intrinsics` block) so they survive a silent camera.
- **Extrinsics (dynamic):** every boot, `task_manager` auto-runs a small extrinsic grid
  (`fit_extrinsic`) to correct for camera drift — see §11. No intrinsics re-measurement.

`require_intrinsics` (default true) makes calibration the extrinsic-only fit and refuses
until intrinsics arrive; set it false to allow the full monolithic fit when the camera
can't report intrinsics.

> **Depth must be aligned to color.** localization samples depth at the *color* pixel of
> each detection/dot and back-projects with the *color* intrinsics. That is only valid
> when the streamer has **depth→color registration ON**; otherwise the depth reading is
> from the wrong physical ray. Enable alignment in the streamer and publish the color
> intrinsics.

### 10.2 The two offsets from the toolhead point

The robot controller commands one pose — the **official toolhead point**. Two things
hang off it and neither sits on it, so each gets its own 3-vector, measured separately
in mm along **robot base axes** at the fixed pick/calibration orientation
(`calibration_orientation_deg`, default 180/0/0):

| offset | param | owner | applied to |
|---|---|---|---|
| red calibration dot | `localization_node.marker_offset_mm` | localization | each calibration sample's ground truth: `commanded_TCP + marker_offset_mm` |
| end of the tooling | `cobot_setup.yaml` `tools.<name>.tip_offset_mm` | `cobot_operation/geometry.py` (`ToolOffset`) | vision-derived pick poses: commanded pose = can position − tool offset |

The marker offset is applied by localization **before** fitting and is never absorbed
into the fit — the transform must localize cans, not tool flanges. Because it is
accounted for during the fit, `/vision/targets` coordinates are true robot-base
positions of the **can**; `cobot_operation` then subtracts its own tool offset so the tool
*tip* is what lands there. The XY part rotates with the commanded `rz` (the offset is
measured at `rz = 0`); Z does not, since `rx`/`ry` are fixed.

Scope: the tool offset touches vision-derived pick poses only. Teach points
(`trajectories.json`) and `/cobot/move_to_pose` stay raw commanded-toolhead poses — the
calibration grid in particular *must* command the toolhead, because the vision side adds
`marker_offset_mm` to that same commanded pose.

`tool_offset_mm: [0,0,0]` means "tip is the toolhead point" and reproduces the behavior
from before the offset existed. A non-zero Z also redefines the pick and approach
heights (`PICK_TIP_Z_MM` / `APPROACH_TIP_Z_MM` in `cobot.py`) as **tip** heights —
re-measure them if you set one.

### 10.3 Ownership split — movements vs math

| | owner | does |
|---|---|---|
| `/task/calibrate` | `task_manager` | pose grid from params, sequences the run (holds the one-task lock), sends one `MoveToPose` goal per pose to `cobot_control`, reports progress on `/task/status` (`calibrating`), stop cancels the active move |
| `/cobot/move_to_pose` | `cobot_control` | executes each grid move (the only node touching the robot) |
| `/vision/calibration_begin` | `localization_node` | clears the sample buffer, warms the red-dot model |
| `/vision/calibration_sample` | `localization_node` | "marker parked at TCP (x,y,z), measure now": N `locate_dot` reads → median (u,v), median-window depth from its own buffer, add marker offset, store the pair |
| `/vision/calibration_compute` | `localization_node` | mode-select (extrinsic if intrinsics known, else full) → validate RMS → write `calibration.yaml` → hot-swap the live transform |
| `/vision/locate_dot` | `yolo_node` | red-dot inference on the latest color frame (only ultralytics importer) — **skipped entirely in manual mode**, see §10.5 |
| `/camera/color/camera_info` | `camera_node` | relays the SDK intrinsics (from the streamer's JSON) that make the extrinsic fit possible |

Sequence: web **Calibrate** → `/task/calibrate` → `begin` → {`MoveToPose` → settle →
`sample`} × poses → `compute` → result in `/task/status` detail. Poses that fail
(unreachable, dot occluded, bad depth) are skipped; the fit is overdetermined.

The same sequence runs **automatically at startup** on a small grid
(`startup_calibration_*`) to re-fit extrinsics for camera drift — see §11. `begin`
refuses (no motion) until CameraInfo intrinsics have arrived, so a boot with no camera
never moves the arm for a fit that can't complete.

### 10.4 Persistence & the no-fallback rule

`calibration.yaml` is written into the bringup config dir, which docker-compose
bind-mounts — calibration survives container rebuilds. It is per-machine and
gitignored. On startup localization loads it if present; **if absent the node refuses
to localize** (detections dropped loudly, `verify_target` errors). No homography
fallback exists: an unmeasured transform must never move the robot.

Prerequisites before running a calibration (all parameters):

- `task_manager.calibration_{x,y}_range_mm`, `calibration_z_levels_mm` — safe, reachable,
  in-frame bounds; `[0,0]` means unconfigured and `/task/calibrate` refuses
- `localization_node.marker_offset_mm` — measured with a ruler after mounting the dot
  (and `tools.<name>.tip_offset_mm` in `cobot_setup.yaml` for the tooling — same reference point, §10.2)
- `yolo_node.dot_model_path` — `red_dot_best.pt` in the models mount
- Camera intrinsics available: the streamer writes `intrinsics_file` (color intrinsics,
  depth→color alignment ON), relayed as `/camera/color/camera_info`. With
  `require_intrinsics:=true` (default) calibration is extrinsic-only and waits for this

---

### 10.5 TEMPORARY: manual centre select

`localization_node.manual_dot_selection: true` replaces the red-dot model with an
operator click, for use while the model is still being trained. **Only the source of
`(u, v)` changes** — depth sampling, `marker_offset_mm`, the fit, the RMS gate and
persistence are the same code either way, so a manual calibration is as valid as an
automatic one (as accurate as your clicking).

```
localization  --/vision/manual_dot_request (Bool, latched)-->  web_node --ws--> browser
browser --ws("VISION","click_center",{u,v})--> web_node --/vision/manual_dot_click--> localization
browser --ws("VISION","skip_center")-------->  web_node --/vision/manual_dot_skip (Empty)--> localization
```

- `calibration_begin` skips the `locate_dot` availability check and the model warm-up
  (so it returns instantly instead of after the ~110 s lazy load).
- `calibration_sample` publishes `manual_dot_request=true`, clears any stale click, and
  blocks up to `manual_click_timeout_sec`. A stale click sent while the arm was still
  moving can never be consumed as the next pose's sample.
- On timeout the pose fails cleanly and is skipped, like any other bad sample.
- **Skip:** if the marker is out of frame at a pose, the operator clicks *"Marker not
  in frame — skip pose"* instead of the feed. That publishes `manual_dot_skip`, and
  `calibration_sample` returns `success=False` immediately (no waiting out the timeout).
  The task manager drops that pose and moves on; the fit is overdetermined, so a few
  skips are harmless.
- The browser clicks `#rgb_image` and scales by `naturalWidth/Height`, so the pixel sent
  is a true COLOR-frame pixel regardless of CSS size.

**`task_manager.calibration_sample_timeout_sec` must stay above
`manual_click_timeout_sec`** (180 vs 120 by default) or the task manager gives up on the
service call while you are still looking at the screen, and the whole run aborts.

To remove: drop the flag, all three topics, `handle_web_command`'s `click_center` /
`skip_center` branches, and the `TEMPORARY` blocks in `app.js` / `index.html` /
`style.css`.

## 11. Task manager — state machine + sequencing (`cobot_tasks`)

Why it exists: in the first migration draft, `/vision/targets` being *published* was what
*triggered* picking (cobot_control subscribed and immediately ran the layer). A data topic
doubling as a command is fragile. The task manager makes "what should happen next" an
explicit, single-owner decision, and turns cobot_control into a dumb, reliable motion server.

**Ownership rule** — mirrors the calibration split (§10.3), one row per node:

| node | owns | never does |
|---|---|---|
| `task_manager` | state machine, task loops, gating, progress | geometry math, motoman/camera access |
| `cobot_control` | motion execution (actions) | decisions, vision calls, task state |
| `localization_node` | geometry + calibration math | robot commands |
| `yolo_node` | ML inference | geometry, robot commands |

Dependency rule enforced in review: `cobot_tasks` depends on `rclpy` + `cobot_interfaces`
ONLY. If a change makes it compute geometry or import robot/camera code, it has become a
god-node — reject it. (It handles only small structured data: targets, poses, states;
never a numpy array.)

**Interface:**

- `/task/start` · `/task/stop` · `/task/calibrate` (`std_srvs/Trigger`) — web routes here
- `/task/status` (`TaskStatus`, latched) — THE operational state; web shows this
- `/task/layer_report` (`LayerReport`) — task manager aggregates per-goal results
- Action clients → `/cobot/pick_can`, `/cobot/move_to_pose`; service clients →
  `/vision/scan`, `/vision/calibration_*`
- Subscribes `/vision/targets` (data: consumed only while a scan it requested is in
  flight, guarded by a timeout) and `/cobot/status` (hardware state, mirrored into
  `/task/status` when idle)

**State machine:**

```
IDLE ──start──▶ SCANNING ──targets──▶ PICKING ──layer done──▶ IDLE
  │                 └─ timeout/none ──▶ ERROR / IDLE
  └─calibrate─▶ CALIBRATING ── compute ok ─▶ IDLE   (fit rejected ─▶ ERROR)
any task ──stop──▶ STOPPING ── active goal canceled ─▶ IDLE
```

One task at a time (`_operation_lock`, held for the whole task); anything but `stop` is
refused while a task runs. Stop cancels the in-flight action goal **natively** — a pick
stops at the next safe phase boundary (never between grip and lift) instead of waiting
for the layer's next can, which is why the motion interface is actions, not services.

**Auto extrinsic calibration at startup.** When `auto_calibrate_on_startup` is set, a
background thread waits for the robot + services (up to `startup_calibration_wait_sec`),
then runs the same calibration loop on a small grid (`startup_calibration_grid_size` /
`startup_calibration_z_levels_mm`) to re-fit the camera *pose* for drift (§10.1a). It is
the `CALIBRATING` state at boot. Safety: it **skips without moving** if the grid is
unconfigured, the robot/services never come up, or (via `begin`) the camera intrinsics
haven't arrived; a failed fit keeps the previously saved transform. The full-grid
`/task/calibrate` remains for a deliberate, denser re-fit.

Bin-cycle state (layer index, `BIN_COMPLETE`, tier-1 rescan decisions — §5) belongs here
when it gets wired in: it is sequencing state, not perception or motion state.

## 12. System health + pick monitor (`cobot_health`)

**Heartbeats.** Every node embeds `cobot_health.heartbeat.HeartbeatPublisher`
and publishes one `Heartbeat` on `/health/heartbeat` every
`heartbeat_period_s` (default 2 s): `level` (OK / WARN / ERROR), a one-line
`detail`, and string `KeyValue[] metrics`. The report callback only reads
cached values and runs in its own callback group, so it never blocks and is
never starved by a long handler (localization's 120 s calibration waits).

| node | level rule | key metrics |
|---|---|---|
| `camera_node` | color and depth judged **identically**: ERROR if either never delivered or `age > 2 s`; WARN if either < 50 % of `publish_rate_hz` | `color_hz`, `depth_hz`, `*_age_s`, `*_dropped` (ZMQ index gaps), `intrinsics_published` |
| `yolo_node` | WARN color frame stale | `scan_in_flight`, `last_scan_ms`, `last_detection_count`, `dot_model_loaded` |
| `localization_node` | ERROR not calibrated; WARN depth stale / no intrinsics | `calibrated`, `rms_*_mm`, `depth_buffer_age_s`, `manual_click_waiting` |
| `cobot_control` | ERROR not connected / alarming, WARN servos off | `connected`, `busy`, `paused`, `servo_on`, `alarming`, `poll_ms` |
| `cobot_operation` | ERROR primitives missing / robot disconnected | `primitives_ready`, `busy`, `paused`, `last_placed`, `active_tool` |
| `task_manager` | WARN robot unavailable / task error | `state`, `progress`, `verify_ready`, `pick_ready` |
| `web_node` | — | `ws_clients`, `stream_hz_actual` |

`cobot_control`'s 10 Hz poll mirrors a link change
into `/cobot/status` (DISCONNECTED / IDLE), which task_manager already
mirrors into `/task/status` — a mid-session link loss is no longer invisible.

**health_monitor node** subscribes `/health/heartbeat`, and publishes a
latched `HealthSummary` on `/health/summary` at 1 Hz: per expected node
(`expected_nodes` param) UNKNOWN if never seen, STALE if silent for
`stale_factor × period_s`, else the node's own level; unexpected publishers
are appended. Level transitions are logged once. `web_node` forwards the
summary to the browser as `{"type": "health"}` (age computed from the header
stamp because the topic is latched); the dashboard flags the monitor itself
as offline if no summary arrives for 3.5 s.

**Pick monitor.** `task_manager` publishes one `PickEvent` on
`/task/pick_event` per `PickCan` attempt with `outcome`
(picked / failed / skipped) and — the "second depth scan" — the result of
calling `/vision/verify_target` after a successful pick (`verify_ran`,
`verified = not present`, `measured_z`). `CanTarget.detection_index` (set by
localization) links each event back to the scan's `ObbDetectionArray` entry
so the browser recolors the right box on the color-feed overlay. The verdict
is reported only; it does not yet gate the layer report or trigger a rescan
(§5), and the UI's `USE_VERIFY_GATE` flag in `app.js` keeps the strict
coloring off until the check is trusted on the real cell.

Dependency note: `cobot_tasks` may depend on `cobot_health` — it is itself
rclpy + `cobot_interfaces` only and carries no geometry/robot/camera code, so
the §11 god-node rule is preserved.
