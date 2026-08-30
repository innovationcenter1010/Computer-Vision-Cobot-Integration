# Computer-Vision-Cobot-Integration

A ROS 2 (Humble) bin-picking stack: an Orbbec RGB-D camera finds cans with a
YOLO oriented-bounding-box model, a fitted camera→robot calibration turns pixels
into robot-base millimetres, and a Yaskawa Motoman cobot (driven through the
YMConnect SDK from a C++ ROS node) picks each can and places it. A FastAPI
web UI streams the camera feeds, shows live status, and issues the
start / stop / calibrate commands.

Everything runs as **one Docker container** (`docker compose up`) holding six
ROS nodes launched by a single launch file. The camera streamer and the robot
controller are external to the container and reached over TCP.

---

## Table of contents

1. [System topology](#1-system-topology)
2. [Quick start](#2-quick-start)
3. [ROS graph at a glance](#3-ros-graph-at-a-glance)
4. [Packages and nodes](#4-packages-and-nodes)
5. [How an operation actually runs](#5-how-an-operation-actually-runs)
6. [Configuration](#6-configuration)
7. [Web UI and API](#7-web-ui-and-api)
8. [Docker build](#8-docker-build)
9. [Repository layout](#9-repository-layout)
10. [Development workflow](#10-development-workflow)
11. [Training the vision models (offline)](#11-training-the-vision-models-offline)
12. [Known gaps and things to watch](#12-known-gaps-and-things-to-watch)

---

## 1. System topology

```
┌──────────────────────── host machine (Docker Desktop / Windows) ───────────────────────┐
│                                                                                        │
│   Orbbec camera ──USB──▶ external streamer process                                     │
│                          ZMQ PUB tcp://*:5550 (color, rgb8)                            │
│                          ZMQ PUB tcp://*:5551 (depth, int16 mm, aligned to color)      │
│                          writes vision_workspace/operation/camera_intrinsics.json      │
│                                        │                                               │
│   ┌────────────── container: cobot-ros ─┼───────────────────────────────────────────┐  │
│   │  ros2 launch cobot_bringup robot.launch.py                                      │  │
│   │                                     ▼                                           │  │
│   │  camera_node ─▶ yolo_node ─▶ localization_node ─▶ task_manager ─▶ cobot_ctrl ───┼──┼──TCP──▶ Motoman controller
│   │       │                                              ▲                          │  │        (YMConnect, 192.168.1.31)
│   │       └──────────────────▶ web_node ◀────────────────┘  :8000                   │  │
│   └─────────────────────────────────────┬───────────────────────────────────────────┘  │
│                                         │ http / websocket                             │
│   browser ◀─────────────────────────────┘                                              │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

- **Camera** — an Orbbec Astra-class RGB-D sensor. It is *not* opened inside the
  container; a separate streamer publishes color and depth frames over ZMQ and
  the container connects out to `host.docker.internal:5550/5551`.
- **Robot** — a Yaskawa Motoman cobot on the LAN, controlled through the
  vendored `libYMConnect.so` SDK. Only `cobot_control` ever talks to it.
- **Container** — single service, single launch file, `rmw_cyclonedds_cpp`,
  every node `respawn=True` for crash isolation. Port 8000 is the only one
  published.

Bind mounts (see [docker-compose.yaml](docker-compose.yaml)):

| Host path | Container path | Purpose |
|---|---|---|
| `./vision_workspace` | `/ws/vision_workspace` | model weights (`*.pt`, gitignored), camera intrinsics JSON, captures, results |
| `./ros2_ws/src/cobot_bringup/config` | `/ws/install/share/cobot_bringup/config` | `cobot_params.yaml`, `trajectories.json`, and the per-machine `calibration.yaml` — editable and persisted without an image rebuild |

---

## 2. Quick start

Prerequisites: Docker Desktop, the external camera streamer running on the
host, and the robot reachable at the IP in `cobot_params.yaml`.

```bash
# 1. Put the model weights in place (they are gitignored)
#    vision_workspace/weights/cans_obb_best.pt
#    vision_workspace/weights/red_dot_best.pt     (only for automatic dot detection)

# 2. Make sure the streamer has written vision_workspace/operation/camera_intrinsics.json

# 3. Build and run
docker compose up --build

# 4. Open the UI
#    http://localhost:8000
```

Bring up a partial stack (e.g. no robot attached) by overriding the compose
command or running the launch file directly:

```bash
ros2 launch cobot_bringup robot.launch.py cobot:=false tasks:=false
#   launch args: camera / vision / cobot / tasks / web   (all default true)
```

> **Safety:** `task_manager.auto_calibrate_on_startup` is `true` in the shipped
> config. **The arm moves at launch** once the robot and vision services are
> up. Set it to `false` if unattended motion at boot is not safe for your cell.

First-time bring-up order:

1. Measure and set `localization_node.marker_offset_mm` and
   `tools.<name>.tip_offset_mm` in `cobot_setup.yaml` (see [§6](#6-configuration)).
2. Set a safe, reachable, in-frame calibration grid
   (`task_manager.calibration_{x,y}_range_mm`, `calibration_z_levels_mm`).
3. Press **Calibrate** in the UI (or `ros2 service call /task/calibrate std_srvs/srv/Trigger`).
   Until `calibration.yaml` exists, `localization_node` refuses to publish
   targets and nothing will pick.
4. Press **Start** to scan and pick one layer.

---

## 3. ROS graph at a glance

```
               ZMQ 5550/5551 + intrinsics.json
 streamer ───▶ camera_node ─┬─▶ /camera/color/image_raw ──────┬─▶ yolo_node
                            ├─▶ /camera/depth/image_raw ──────┼──────────────▶ localization_node
                            └─▶ /camera/color/camera_info ────┼──────────────▶ localization_node
                                (latched)                     └─▶ web_node
                                                                                │
   yolo_node ── /vision/detections_px (pixel space) ────────────────────────────┘
        ▲  srv /vision/scan, /vision/locate_dot
        │
   localization_node ── /vision/targets (robot space) ──┬─▶ task_manager
        srv /vision/verify_target                       └─▶ web_node
        srv /vision/get_layer_height
        srv /vision/calibration_begin|sample|compute
        topics /vision/manual_dot_request|click|skip  (temporary manual mode)

   browser ══ws══▶ web_node ── srv /task/start|stop|calibrate ──▶ task_manager
                      ▲                                             │
                      ├── /task/status (latched), /task/layer_report ┘
                      │
   task_manager ── action /cobot/pick_can ──▶ cobot_operation ── MoveToPose / set_output_bit ──▶ cobot_control ──▶ robot
                ── action /cobot/move_to_pose ─────────────────────────────────────────▶ cobot_control
                      ◀────────────── /cobot/status (latched) ───────┘
```

Design rules that the graph enforces (from [docs/ros2_architecture.md](docs/ros2_architecture.md)):

| Node | Owns | Never does |
|---|---|---|
| `task_manager` | the state machine, task loops, gating, progress reporting | geometry, robot/camera access |
| `cobot_operation` | pick choreography (`/cobot/pick_can`) | decisions, vision calls, task state, robot code |
| `cobot_control` | hardware primitives (YMConnect) | choreography, decisions |
| `localization_node` | geometry + calibration math | robot commands, ML |
| `yolo_node` | ML inference (the only `ultralytics` importer) | geometry, robot commands |
| `camera_node` | ZMQ→ROS bridging | anything else |
| `web_node` | browser bridge | logic — every command is forwarded to a service |

Two consequences worth internalising:

- **The cobot never sees pixels, YOLO never sees millimetres.** `ObbDetection`
  is pixel-only; `CanTarget` is robot-frame only. Only `localization_node`
  converts between them.
- **`/vision/targets` is data, not a trigger.** Nothing moves because a topic was
  published. `task_manager` only consumes a targets message while a scan *it*
  requested is in flight, and it drives `cobot_control` through action goals.

QoS contract: images are best-effort keep-last(1) (freshest wins);
`camera_info`, `/cobot/status`, `/task/status`, `/vision/manual_dot_request` are
reliable + transient-local (latched); everything else is reliable depth 10.

---

## 4. Packages and nodes

All packages live under [ros2_ws/src/](ros2_ws/src/). Dependency rule: every
package depends only on `cobot_interfaces` and stock messages; `cobot_control`
is C++ and links YMConnect; `cobot_tasks` depends on `rclpy` +
`cobot_interfaces` *only*; nothing depends on `cobot_bringup`.

### 4.1 `cobot_interfaces` — message contracts

| Type | Name | Fields / purpose |
|---|---|---|
| msg | `ObbDetection` | `class_id`, `confidence`, `center_x/y` (px), `corners_px[8]` (TL,TR,BR,BL) — **pixel space only** |
| msg | `ObbDetectionArray` | `header` (stamp = the color frame the model ran on), `image_width/height`, `detections[]` |
| msg | `CanTarget` | `class_id`, `confidence`, `real_x/y/z` (mm, robot base), `real_rz` (deg) — **robot space only** |
| msg | `CanTargetArray` | `header`, `targets[]` — one ordered pick layer |
| msg | `CobotStatus` | **hardware** state: `DISCONNECTED / IDLE / BUSY / ERROR` + `detail` |
| msg | `TaskStatus` | **operational** state: `DISCONNECTED / IDLE / SCANNING / PICKING / CALIBRATING / STOPPING / ERROR` + `detail`, `active_task`, `progress_current/total` |
| msg | `LayerReport` | `successes`, `failures`, `overall_ok` |
| srv | `VerifyTarget` | `CanTarget target → bool present, float64 measured_z` (tier-1 depth check) |
| srv | `GetLayerHeight` | `() → float64 median_z, int8 layer_index` (tier-1 depth check) |
| srv | `LocateDot` | `() → found, u, v, confidence, message` (red calibration marker) |
| srv | `CalibrationSample` | `x, y, z (commanded TCP) → success, sample_count, message` |
| action | `PickCan` | goal `CanTarget target` → result `success, message`; feedback `phase` (check/approach/grip/lift/place) |
| action | `MoveToPose` | goal `x,y,z,rx,ry,rz` → result `success, message` |

Stock types used elsewhere: `sensor_msgs/Image`, `sensor_msgs/CameraInfo`,
`std_srvs/Trigger`, `std_msgs/Bool`, `std_msgs/Empty`, `geometry_msgs/PointStamped`.

### 4.2 `cobot_camera` / `camera_node`

Bridges the external ZMQ streamer into the graph.
Source: [camera_node.py](ros2_ws/src/cobot_camera/cobot_camera/camera_node.py),
[camera_streamer.py](ros2_ws/src/cobot_camera/cobot_camera/camera_streamer.py).

- `CameraStream` runs two ZMQ SUB threads (color, depth) with a latest-frame
  cache. It accepts three wire framings (multipart / combined / legacy two-part)
  so a publisher change never blacks out the stream, keeps `RCVHWM=2` and
  drains to the newest frame, and deliberately avoids `ZMQ_CONFLATE` (it aborts
  the process on multipart input). Frame header: `struct "<BIHHBI"` =
  `stream_type, frame_index, width, height, channels, payload_size`.
- A timer at `publish_rate_hz` republishes whichever cached frame is new:
  - `/camera/color/image_raw` — `sensor_msgs/Image`, `rgb8`
  - `/camera/depth/image_raw` — `sensor_msgs/Image`, `16SC1` (int16 mm, as the streamer sends it)
- `/camera/color/camera_info` (latched) — relays the streamer's
  `camera_intrinsics.json` (`fx, fy, cx, cy, width, height`) as a `CameraInfo`.
  Re-read only when the file's mtime changes (5 s timer).
- Service `/camera/save_frames` (`Trigger`) — writes a timestamped
  `color_*.png` (8-bit BGR) + `depth_*.png` (16-bit mm) pair to `capture_dir`,
  which is bind-mounted so the files land on the host. This is how training
  images are collected.

Parameters: `color_endpoint`, `depth_endpoint`, `publish_rate_hz`,
`color_frame_id`, `depth_frame_id`, `capture_dir`, `intrinsics_file`.

### 4.3 `cobot_vision` / `yolo_node`

Pure perception. Source: [yolo_node.py](ros2_ws/src/cobot_vision/cobot_vision/yolo_node.py),
[yolo_obb_detector.py](ros2_ws/src/cobot_vision/cobot_vision/yolo_obb_detector.py).

- Subscribes `/camera/color/image_raw`, caches the latest frame + stamp.
- Service `/vision/scan` (`Trigger`) — starts YOLO-OBB (`cans_obb_best.pt`) on
  the cached frame in a worker thread and returns immediately (`"scan started"`,
  or refuses with `"scan already running"` / `"no camera frame received yet"`).
  When done it publishes `/vision/detections_px` (`ObbDetectionArray`) with
  `header.stamp` = the color frame's stamp. Optional pixel-distance duplicate
  removal (`remove_duplicates`, `duplicate_center_distance_px`) keeps the
  highest-confidence of any two detections closer than the threshold.
- Service `/vision/locate_dot` (`LocateDot`) — runs a second model
  (`red_dot_best.pt`, lazy-loaded on first call) on the latest frame and returns
  the highest-confidence hit. Accepts either an OBB or an axis-aligned head.
- Live-tunable parameters via `ros2 param set` / the web config editor:
  `confidence`, `iou`, `max_detections`, `image_size`. `model_path` and
  `dot_model_path` are rejected at runtime (restart the node).

### 4.4 `cobot_vision` / `localization_node`

Pure geometry. Source: [localization_node.py](ros2_ws/src/cobot_vision/cobot_vision/localization_node.py),
plus the pure-math helpers [calibration.py](ros2_ws/src/cobot_vision/cobot_vision/calibration.py),
[pathing.py](ros2_ws/src/cobot_vision/cobot_vision/pathing.py),
[rotation.py](ros2_ws/src/cobot_vision/cobot_vision/rotation.py). Runs on a
`MultiThreadedExecutor` because its calibration handlers block on futures.

**Inputs:** `/vision/detections_px`, `/camera/depth/image_raw` (ring buffer of
`depth_buffer_size` frames, stamp-matched), `/camera/color/camera_info`.

**Refuses to work uncalibrated.** On start it loads
`calibration_file` (`config/calibration.yaml`, injected by the launch file). If
the file is missing or unreadable, detections are dropped with a throttled
error and `verify_target` returns `present=false`. There is deliberately no
fallback — an unmeasured transform must never move the robot.

**Detections → targets** (`_on_detections`): for each detection, pick the depth
frame nearest the detection stamp, take the median depth in a
`depth_sample_window_px` window around the OBB centre (zeros = no reading are
ignored), and map `(u, v, d)` → robot `(x, y, z)` through the fitted 3×4
matrix. `real_rz` is the angle of the OBB's left edge (TL→BL, both corners
mapped through the same transform at the centre's depth) against robot +Y.
Targets are then ordered by `order_detections_snake`: rows grouped along
`pathing_row_axis` within `pathing_row_tolerance_mm`, serpentine along the other
axis. No bin-corner coordinates are involved, so the order survives the bin
shifting. Published on `/vision/targets` (`CanTargetArray`,
`frame_id="robot_base"`). The pixel centres of the last scan are remembered for
`get_layer_height`.

**The calibration model** (`calibration.py`, no ROS imports): a pinhole camera
composed with a rigid camera→robot transform is exactly linear in
`[u·d, v·d, d, 1]`, so one 3×4 matrix `M` maps pixel + depth to robot mm:

```
[X, Y, Z]ᵀ = M · [u·d, v·d, d, 1]ᵀ        M = [ R·B | t ]
```

Two fitters:

| Fitter | Unknowns | Needs | Used when |
|---|---|---|---|
| `fit()` (full, monolithic least squares) | 12 | ≥ 6 samples, ≥ 30 mm depth span, non-degenerate geometry | `require_intrinsics: false` |
| `fit_extrinsic()` (Kabsch/Umeyama, known intrinsics) | 6 | ≥ 4 samples, intrinsics from `CameraInfo` (or the saved `camera_intrinsics` block) | `require_intrinsics: true` |

Both report per-axis RMS residuals; a fit with any axis above
`calibration_max_rms_mm` is **rejected and the old transform kept** (the samples
are dumped to the log so the rejection is debuggable). `inverse()` maps robot
xyz back to the expected `(u, v, d)` and powers `verify_target`.

**Services:**

| Service | What it does |
|---|---|
| `/vision/verify_target` | Project the target's xyz to the expected pixel+depth, sample live depth there, `present = abs(measured − expected) ≤ verify_tolerance_mm`; returns the robot-frame Z of whatever is actually there |
| `/vision/get_layer_height` | Median live depth at the last scan's detection pixels; `layer_index = round((median − depth_at_top_layer_mm) / can_height_mm)`, or −1 if either param is 0 |
| `/vision/calibration_begin` | Refuse if no `calibration_file`, no depth frames, or (with `require_intrinsics`) no intrinsics yet. In automatic mode also waits for `/vision/locate_dot` and warms the red-dot model (≈ 110 s budget). Clears the sample buffer |
| `/vision/calibration_sample` | "Marker is parked at TCP `(x,y,z)` — measure now." Gets `(u,v)` either from `dot_samples_per_pose` `locate_dot` reads (median, majority must hit) **or** from an operator click (manual mode), samples its own depth buffer at that pixel, adds `marker_offset_mm` to the commanded TCP, stores the pair |
| `/vision/calibration_compute` | Needs ≥ `calibration_min_samples`; picks the fitter by `require_intrinsics`; validates RMS; writes `calibration.yaml`; hot-swaps the live transform |

**Temporary manual centre select** (`manual_dot_selection: true`, the shipped
default): `calibration_sample` publishes `/vision/manual_dot_request=true`
(latched), then blocks up to `manual_click_timeout_sec` for either a
`/vision/manual_dot_click` (`PointStamped`, colour-image pixels) or a
`/vision/manual_dot_skip` (`Empty`, "marker not in frame — drop this pose").
Only the source of `(u, v)` changes; depth sampling, offset and fit are the
same code path. Stale clicks are cleared before each request so an input sent
while the arm was still moving is never consumed as the next pose's sample.

### 4.5 `cobot_control` — C++ node on the YMConnect link

The only process that talks to the Motoman. `ament_cmake`, `rclcpp` +
`rclcpp_action`, links the vendored `libYMConnect.so`
([third_party/YMConnect](ros2_ws/src/cobot_control/third_party/YMConnect))
which is installed beside the executable and found via `$ORIGIN` RPATH.
Source: [motoman_robot.cpp](ros2_ws/src/cobot_control/src/motoman_robot.cpp)
(SDK wrapper, no ROS, one mutex around every SDK call) and
[cobot_control_node.cpp](ros2_ws/src/cobot_control/src/cobot_control_node.cpp).
It replaces the pybind11 extension `motoman_cpp`, which is retired in place
(`COLCON_IGNORE`, kept for scripting — see
[motoman_cpp/README.md](ros2_ws/src/motoman_cpp/README.md)).

Hardware primitives only — no choreography, no task state:

| Interface | Behaviour |
|---|---|
| action `/cobot/move_to_pose` | One supervised move. Goal picks a named motion profile from `cobot_setup.yaml` (joint or **linear** interpolation, speed, accel/decel, accuracy, tool file) with per-goal overrides. IK reachability check, `AddPointToTrajectory` → `MotionStart`, then the move loop waits on the poll (below) for progress. Feedback `progress_percent`, `steps_remaining`, `paused`. **Cancel is immediate**: `MotionStop(true)`, trajectory cleared |
| srv `/cobot/pause`, `/cobot/resume` | Hold the arm where it is (`MotionStop(false)`; `MotionStart()` resumes the same trajectory — the SDK's documented pause) and continue. With no move in flight the latch makes the next move wait. `pause_uses_hold` switches to the controller HOLD signal |
| srv `/cobot/motion_stop` | Immediate controlled stop (not an e-stop), optionally clearing the trajectory |
| srv `/cobot/set_output_bit`, `/cobot/read_io_bit` | Write / read one bit by **name** (`io.outputs` / `io.sensors` in `cobot_setup.yaml`) or raw `(io_type, group, bit)` |
| srv `/cobot/set_motion_profile`, `/cobot/get_current_pose`, `/cobot/get_active_alarms`, `/cobot/clear_faults`, `/cobot/clear_trajectory`, `/cobot/servo_on`, `/cobot/servo_off`, `/cobot/reconnect`, `/cobot/read|write_int_var`, `/cobot/read|write_position_var` | thin wrappers; every response carries `success`, `status_code` (0 ok, 1 not connected, else the YMConnect code) and `message` |
| topic `/cobot/state` (`CobotState`, 10 Hz) | The **poll**: one tick reads `ReadState` (servo / hold / alarm / Teach-Play-Remote), one `ReadByte` per distinct IO group (so all named sensors cost one round-trip), the current TCP pose and — while alarming — the active alarms. Every named sensor and output is published by name. `poll_ms` is the tick's cost; watch it against `poll_period_s` |
| topic `/cobot/status` (latched) | `DISCONNECTED` / `IDLE` / `BUSY` / `ERROR` (alarm, servos off, Teach mode) — hardware status only |

Connection supervision is the same timer: not connected → one `OpenConnection`
attempt per `connect_retry_period_s`; connected → a failed `ReadState` marks
the link lost, aborts any move, and the retry loop takes over. On connect the
node arms the servos and **clears the controller's trajectory buffer** so a
stale point from a crashed session can never execute first.

### 4.6 `cobot_operation` — pick choreography

Python, `rclpy` + `cobot_interfaces` only — it never imports robot code.
Source: [choreography.py](ros2_ws/src/cobot_operation/cobot_operation/choreography.py)
(phase order, tested against a fake in
[test/](ros2_ws/src/cobot_operation/test/)),
[primitives.py](ros2_ws/src/cobot_operation/cobot_operation/primitives.py)
(action / service clients + the `/cobot/state` cache) and
[operation_node.py](ros2_ws/src/cobot_operation/cobot_operation/operation_node.py).

| Interface | Behaviour |
|---|---|
| action `/cobot/pick_can` | **check** pose → **approach** (above-can, then descend with the tool's `descend_profile`) → **grip** (`set_output_bit gripper`) → **lift** → back to check → sensor reads from `/cobot/state` (`existance_check`, `divot_check`, `right_complete`, `left_complete`) → **place** on the side opposite the last placement, choosing `right/right_flip/left/left_flip` from `divot_check` → gripper off → existence assert. Feedback `phase`; the result carries the full report (`jobs[]`, `sensor_asserts[]`, `phase`). **Cancel is immediate** — the in-flight `MoveToPose` is canceled (controller stop) and the choreography returns at its next boundary; the can may still be gripped |
| pause | `cobot_control`'s `/cobot/pause` holds the arm and the choreography sees `CobotState.paused`, so it also waits before its next primitive. One service pauses both layers |

The execute callback is a coroutine: it `await`s the action / service futures,
so no executor thread is ever parked and goal / cancel requests stay
serviceable mid-pick. Sensor assertions wait up to `sensor_assert_timeout_s`
for a matching `/cobot/state` sample (the old 5 × 0.2 s re-read loop).

Tool profiles live in [cobot_setup.yaml](ros2_ws/src/cobot_bringup/config/cobot_setup.yaml)
(`tools.<name>`): `tip_offset_mm` (the former `tool_offset_mm`), the YRC1000
`tool_number`, `pick_tip_z_mm` / `approach_tip_z_mm` and the descent motion
profile. `active_tool` selects one; `cobot_operation.active_tool` overrides it.
`ToolOffset` converts a vision-derived *can* position into the *toolhead* pose
to command so that the tool **tip** lands on the can; the XY part rotates with
the commanded `rz`, Z does not. It applies to vision-derived pick poses only —
teach points in `trajectories.json` and `move_to_pose` goals stay raw.


### 4.7 `cobot_tasks` / `task_manager`

The decision layer — owns *what* the system is doing.
Source: [task_manager_node.py](ros2_ws/src/cobot_tasks/cobot_tasks/task_manager_node.py).
Depends on `rclpy` + `cobot_interfaces` only; it never touches numpy, the
camera or the robot.

- Services `/task/start`, `/task/stop`, `/task/calibrate` (`Trigger`) live in
  their own callback group so `stop` is reachable while a task runs.
- Publishes `/task/status` (`TaskStatus`, latched) — **the** operational state
  the UI shows — and `/task/layer_report`.
- Subscribes `/vision/targets` (consumed only while `_awaiting_targets`) and
  `/cobot/status` (mirrored into `/task/status` as `DISCONNECTED` / `IDLE`
  when no task is running).
- Action clients to `/cobot/pick_can`, `/cobot/move_to_pose`; service clients
  to `/vision/scan` and the three `/vision/calibration_*` services.

State machine:

```
IDLE ──start──▶ SCANNING ──targets──▶ PICKING ──layer done──▶ IDLE
  │                 └─ timeout / refused ──▶ ERROR      └─ no cans ──▶ IDLE
  └─calibrate─▶ CALIBRATING ── compute ok ─▶ IDLE   (fit rejected ─▶ ERROR)
any task ──stop──▶ STOPPING ── active goal cancelled ─▶ IDLE
```

One task at a time (`_operation_lock` held for the whole task, on a background
worker thread). While a goal is in flight the worker polls `_stop_requested`
every 200 ms and cancels the goal natively, so a stop interrupts a pick at its
next safe phase boundary instead of waiting for the next can. Timeouts:
goal ack 10 s, one pick 300 s, one calibration move 120 s, `calibration_begin`
120 s, `calibration_compute` 60 s, `calibration_sample` = parameter.

### 4.8 `cobot_web` / `web_node`

Browser ⇄ graph bridge. Source: [web_node.py](ros2_ws/src/cobot_web/cobot_web/web_node.py),
[index.html](ros2_ws/src/cobot_web/templates/index.html),
[app.js](ros2_ws/src/cobot_web/static/app.js),
[commands.js](ros2_ws/src/cobot_web/static/commands.js).

rclpy spins on a background thread; uvicorn owns the main thread's asyncio
loop; ROS callbacks push to websocket clients through
`run_coroutine_threadsafe`.

- Subscribes both image topics, JPEG-encodes at `web_stream_hz` and broadcasts
  `{"type":"camera_frame","rgb":<b64>,"depth":<b64>}` (depth min-max normalised
  to 8-bit for display).
- Forwards `/vision/targets`, `/task/status` (as websocket type
  `cobot_status`), `/task/layer_report` and `/vision/manual_dot_request` to
  the browser. The last status and the manual-click flag are replayed to every
  newly connected page.
- Routes websocket commands `{"receiver","command","payload"}` through a
  string→service table (`COMMAND_ROUTES`) — see [§7](#7-web-ui-and-api).
- `GET/POST /api/config` reads/writes `cobot_params.yaml` and, on POST, also
  calls each node's `set_parameters` so accepted changes apply live.

### 4.9 `cobot_bringup` — launch + config

- [robot.launch.py](ros2_ws/src/cobot_bringup/launch/robot.launch.py) — starts
  the six nodes, all reading `config/cobot_params.yaml`, with per-node
  `respawn=True` and launch args `camera / vision / cobot / tasks / web`. It
  also injects three paths: `localization_node.calibration_file`,
  `cobot_operation.trajectories_file`, `*.setup_file`, `web_node.params_file` — all in the
  bind-mounted config dir.
- [cobot_params.yaml](ros2_ws/src/cobot_bringup/config/cobot_params.yaml) —
  every runtime parameter (see [§6](#6-configuration)).
- [trajectories.json](ros2_ws/src/cobot_bringup/config/trajectories.json) —
  named teach points used by `cobot.py`: `check` (single pose), and two-point
  `right`, `right_flip`, `left`, `left_flip` place trajectories (`await` is
  present but unused).
- `config/calibration.yaml` — **generated**, gitignored, per machine. Written by
  `/vision/calibration_compute`.

---

## 5. How an operation actually runs

### 5.1 Pick cycle (`/task/start`)

```
browser "Start"  ──ws──▶ web_node ──▶ /task/start
task_manager: robot available? /vision/scan ready? /cobot/pick_can ready? lock free?
  ├─ publish SCANNING, set _awaiting_targets
  ├─ call /vision/scan ──▶ yolo_node runs YOLO-OBB in a thread
  │                          └─▶ /vision/detections_px ──▶ localization_node
  │                                └─▶ depth lookup + M transform + rz + snake order
  │                                      └─▶ /vision/targets (N ordered CanTargets)
  ├─ wait ≤ scan_timeout_sec for targets (ERROR on timeout, IDLE on zero cans)
  ├─ for each target (PICKING, progress i/N):
  │     send PickCan goal ──▶ cobot_operation.pick_and_place  (check→approach→grip→lift→place)
  │     count success / failure; stop request cancels the goal at a safe boundary
  ├─ publish /task/layer_report {successes, failures, overall_ok}
  └─ publish IDLE "layer done: X ok / Y failed"
```

YOLO runs **once per layer**. The `verify_target` and `get_layer_height`
services exist for a tiered multi-layer loop (verify each can against live
depth before gripping, measure when a layer is finished) but the task manager
does not call them yet — see [§12](#12-known-gaps-and-things-to-watch).

### 5.2 Camera→robot calibration (`/task/calibrate`)

A red marker is mounted near the tool head. The robot parks it at a grid of
known poses, the camera reports where it sees it, and `localization_node`
solves for the transform.

```
task_manager (sequencing)                      localization_node (math)          cobot_control
────────────────────────                       ────────────────────────          ──────────
check grid configured, robot + services ready
publish CALIBRATING 0/N
call /vision/calibration_begin ──────────────▶ clear buffer; check intrinsics/depth;
                                               (auto mode: warm red-dot model)
for each pose (x,y,z) at calibration_orientation_deg:
   send MoveToPose ─────────────────────────────────────────────────────────────▶ move, settle
   sleep calibration_settle_sec
   call /vision/calibration_sample(x,y,z) ───▶ (u,v) from locate_dot or a UI click,
                                               d from depth buffer,
                                               store [u,v,d] ↔ TCP + marker_offset_mm
   (move or sample failure → pose skipped; fit is overdetermined)
call /vision/calibration_compute ────────────▶ fit (extrinsic or full) → RMS gate
                                               → write calibration.yaml → hot-swap
publish IDLE "complete: …" or ERROR "failed: …"
```

Grid: `calibration_grid_size = [nx, ny]` poses spread over
`calibration_x_range_mm × calibration_y_range_mm` at each of
`calibration_z_levels_mm` (e.g. 3×2×3 = 18 poses). The full fit needs ≥ 30 mm
of depth span, so use at least two heights.

**Manual centre-select mode** (shipped default `manual_dot_selection: true`):
at each pose the UI shows a prompt over the colour feed; the operator clicks
the marker centre (the browser scales the click to natural image pixels) or
presses *"Marker not in frame — skip pose"*. Keep
`task_manager.calibration_sample_timeout_sec` (180) above
`localization_node.manual_click_timeout_sec` (120) or the task manager gives up
while you are still looking at the screen.

**Two offsets from the same reference point.** The controller commands the
*official toolhead point*; two things hang off it and neither sits on it:

| Offset | Parameter | Applied by | To |
|---|---|---|---|
| red marker | `localization_node.marker_offset_mm` | localization, *before* fitting | each sample's ground truth = commanded TCP + offset |
| tool tip | `cobot_setup.yaml` `tools.<name>.tip_offset_mm` | `cobot_operation/geometry.py` `ToolOffset` | vision-derived pick poses: commanded = can − offset |

Because the marker offset is accounted for during the fit, `/vision/targets`
are true robot-base positions of the **can**; `cobot_operation` then subtracts its
own tool offset so the tool tip lands there. Measure both with a ruler, along
robot base axes, at the calibration orientation (`rx=180, ry=0, rz=0`).

**Intrinsics.** With `require_intrinsics: true` the small extrinsic-only
(Kabsch) fit is used against the SDK intrinsics relayed on
`/camera/color/camera_info`. The shipped config sets it to **false** because
this camera's reported colour `fx/fy` are internally inconsistent
(`camera_intrinsics.json` documents the problem and the recipe to recover true
intrinsics from a full fit's matrix).

### 5.3 Automatic extrinsic calibration at boot

With `auto_calibrate_on_startup: true`, a background thread in `task_manager`
waits up to `startup_calibration_wait_sec` for the robot and the vision
services, then runs the same calibration loop on a smaller grid
(`startup_calibration_grid_size` × `startup_calibration_z_levels_mm`) to
correct for camera drift. It skips without moving if the grid is unconfigured,
the robot/services never come up, or `calibration_begin` refuses (e.g. no
intrinsics). A rejected fit keeps the previously saved transform.

### 5.4 Stop (`/task/stop`)

Sets `_stop_requested`, publishes `STOPPING`, and the worker cancels the
in-flight action goal. `PickCan` stops at its next safe phase boundary (a
cancel can leave the arm holding a can at the check pose — the result message
says which phase); `MoveToPose` finishes the current move first. Calibration
aborts at the next pose. `stop` when nothing is running returns success with
`"nothing running"`.

---

## 6. Configuration

Everything lives in [cobot_params.yaml](ros2_ws/src/cobot_bringup/config/cobot_params.yaml)
(standard ROS 2 params layout, one section per node) and is bind-mounted, so
edits apply on the next container start without a rebuild. Scalars can also
be changed live with `ros2 param set /<node> <name> <value>` or the web
config editor.

Parameters that **must be measured/set before the robot moves**:

| Parameter | Meaning |
|---|---|
| `cobot_control.robot_ip` | Motoman controller address |
| `cobot_setup.yaml` → `io.sensors` | real YRC1000 addresses of the four pick sensors (the shipped values are placeholders) |
| `localization_node.marker_offset_mm` | red marker relative to the commanded TCP, robot-base axes |
| `cobot_setup.yaml` → `tools.<name>.tip_offset_mm` | tool tip relative to the commanded TCP, robot-base axes (was `cobot_control.tool_offset_mm`) |
| `task_manager.calibration_x_range_mm`, `calibration_y_range_mm` | safe, reachable, in-frame calibration bounds (`[0,0]` = unconfigured → calibrate refuses) |
| `task_manager.calibration_z_levels_mm`, `startup_calibration_z_levels_mm` | heights for the full and boot grids — **floats with `.0`** (an int array fails to load) |
| `task_manager.auto_calibrate_on_startup` | arm moves at launch when true |

Other notable parameters:

| Node | Parameter | Notes |
|---|---|---|
| `camera_node` | `color_endpoint`, `depth_endpoint` | ZMQ SUB endpoints (`tcp://host.docker.internal:5550/5551`) |
| `camera_node` | `intrinsics_file`, `capture_dir` | in the `vision_workspace` mount |
| `yolo_node` | `model_path`, `confidence`, `iou`, `max_detections`, `image_size`, `remove_duplicates`, `duplicate_center_distance_px`, `dot_model_path`, `dot_confidence` | thresholds are live-tunable |
| `localization_node` | `pathing_row_axis` (`"x"`/`"y"`), `pathing_row_tolerance_mm` | snake ordering |
| `localization_node` | `can_height_mm`, `depth_at_top_layer_mm` | for `get_layer_height`; 0 = unconfigured |
| `localization_node` | `depth_buffer_size`, `depth_sample_window_px`, `verify_tolerance_mm` | depth sampling |
| `localization_node` | `dot_samples_per_pose`, `calibration_min_samples`, `calibration_max_rms_mm`, `require_intrinsics` | fit gating |
| `localization_node` | `manual_dot_selection`, `manual_click_timeout_sec` | temporary manual mode |
| `cobot_control` | `poll_period_s`, `connect_retry_period_s`, `motion_timeout_s`, `pause_uses_hold`, `check_reachability`, `pendant_messages` | 10 Hz poll, reconnect, move supervision |
| `cobot_operation` | `active_tool`, `move_timeout_s`, `service_timeout_s`, `sensor_assert_timeout_s` | tool profile override, primitive budgets |
| `cobot_setup.yaml` | `io.*`, `motion_profiles.*`, `tools.*`, `active_tool` | named IO, joint/linear motion profiles, tooling profiles (`compression_mm` declared, **not yet applied**) |
| `task_manager` | `scan_timeout_sec`, `calibration_grid_size`, `calibration_orientation_deg`, `calibration_settle_sec`, `calibration_sample_timeout_sec`, `startup_calibration_grid_size`, `startup_calibration_wait_sec` | |
| `web_node` | `port`, `web_stream_hz`, `results_dir` | |

Generated / per-machine files (gitignored): `config/calibration.yaml`,
`vision_workspace/operation/camera_intrinsics.json`, all `*.pt` weights,
`vision_workspace/operation/{captures,results,inbox,debug_images}`.

---

## 7. Web UI and API

`http://localhost:8000` — a single page with the colour and depth feeds, a
**Save frames** button, **Start / Calibrate / Stop**, a live status box fed by
`/task/status`, the manual marker-click prompt, and a "latest result image"
panel.

Websocket `/ws` — browser sends `{"receiver", "command", "payload"}`:

| receiver / command | Routed to | Effect |
|---|---|---|
| `COBOT` / `start`, `pickup` | `/task/start` | scan + pick one layer |
| `COBOT` / `stop` | `/task/stop` | cancel the active goal |
| `COBOT` / `calibrate` | `/task/calibrate` | full-grid calibration |
| `VISION` / `detect` | `/vision/scan` | display-only scan, nothing moves (targets are ignored by the task manager) |
| `CAMERA` / `save` | `/camera/save_frames` | write a PNG pair to `vision_workspace/operation/captures` |
| `CAMERA` / `snapshot` | (web_node) | push one frame immediately |
| `VISION` / `click_center` `{u,v}` | topic `/vision/manual_dot_click` | manual calibration click |
| `VISION` / `skip_center` | topic `/vision/manual_dot_skip` | drop the current calibration pose |

Server → browser message types: `connection`, `camera_frame`, `command_result`,
`targets`, `cobot_status` (carries `state`, `detail`, `active_task`,
`progress_current/total` from `TaskStatus`), `layer_report`,
`manual_dot_request`.

HTTP: `GET /` (UI), `GET /api/config` (params YAML as JSON),
`POST /api/config` (write YAML + live `set_parameters`, returns per-node
outcomes: `applied` / `rejected: [...]` / `node not reachable`),
`GET /api/latest-image` (newest file in `results_dir`), `/static/*`, `/results/*`.

---

## 8. Docker build

[docker/Dockerfile](docker/Dockerfile) is a two-stage build:

1. **builder** (`ros:humble`) — apt build tools + `pybind11_vendor`; pip
   `ultralytics` pinned as the *first* pip layer (it pulls torch and is by far
   the longest step, so nothing downstream ever busts it); remaining pip deps
   (`opencv-python-headless`, `fastapi`, `uvicorn[standard]`, `pyzmq`,
   `jinja2`); `setuptools<80` + `packaging>=22` pin for colcon compatibility;
   `package.xml` manifests copied alone → `rosdep install` (cached on manifest
   changes only); then the source → `colcon build --merge-install`.
2. **runtime** (`ros:humble-ros-core`) — runtime libs, `rmw_cyclonedds_cpp`,
   the same pip layers, and only the `/ws/install` tree from the builder (launch
   files, message definitions, Python packages, the compiled `cobot_control_node` +
   `libYMConnect.so`). No compilers, no source, no baked model weights.

[docker/entrypoint.sh](docker/entrypoint.sh) sources `/opt/ros/humble/setup.bash`
and `/ws/install/setup.bash` then `exec "$@"`; CRs are stripped at build time
so a Windows checkout cannot break the shebang.

Why one container: the deployment host is Docker Desktop for Windows, where
`network_mode: host` (the usual DDS-in-containers fix) is unavailable. Per-node
crash isolation comes from `respawn=True` in the launch file instead.

---

## 9. Repository layout

```
.
├── docker-compose.yaml            # single cobot-ros service, bind mounts, port 8000
├── docker/                        # Dockerfile (2-stage) + entrypoint.sh
├── docs/
│   ├── ros2_architecture.md       # full design document (the "why" behind every node)
│   └── dependancies/orbbec_devices/AstraSDK_Doc_en.chm
├── ros2_ws/src/
│   ├── cobot_interfaces/          # msg / srv / action definitions
│   ├── cobot_camera/              # camera_node, camera_streamer (ZMQ), frame_saver (legacy helper)
│   ├── cobot_vision/              # yolo_node, localization_node, calibration/pathing/rotation math
│   ├── cobot_control/             # C++ node on YMConnect (ament_cmake) + vendored SDK in third_party/
│   ├── cobot_operation/           # pick choreography node (Python) + unit tests
│   ├── motoman_cpp/               # RETIRED pybind11 module (COLCON_IGNORE) + the old Python cobot node
│   ├── cobot_tasks/               # task_manager_node
│   ├── cobot_web/                 # web_node, templates/, static/
│   └── cobot_bringup/             # robot.launch.py, cobot_params.yaml, cobot_setup.yaml, trajectories.json
├── vision_workspace/              # bind-mounted at /ws/vision_workspace — see its README.md
│   ├── .venv/                     # dev-PC training/export env (gitignored) — NOT in the image
│   ├── tools/                     # train_yolo_obb.py, train_green_light.py, export_to_onnx.py, export_yoloe.py
│   ├── weights/                   # trained + base .pt (gitignored); training_runs/ beside it
│   ├── models/                    # ONNX exports ONLY (<name>/<name>.onnx + manifest.yaml)
│   ├── datasets/                  # cans_obb (YOLO-OBB), green_light (Roboflow export)
│   ├── operation/                 # camera_intrinsics.json, captures/, results/, inbox/
│   ├── calibration/images/
│   └── tests/                     # standalone motoman_cpp smoke scripts (need the retired module built)
└── legacy/                        # the pre-ROS asyncio monolith this stack replaced (reference only)
```

---

## 10. Development workflow

Inside the container (or any Humble environment with the pip deps installed):

```bash
cd /ws
colcon build --merge-install --symlink-install     # symlink: Python edits are live
source install/setup.bash
ros2 launch cobot_bringup robot.launch.py cobot:=false

# Poke individual pieces
ros2 topic hz /camera/color/image_raw
ros2 service call /vision/scan std_srvs/srv/Trigger
ros2 topic echo /vision/targets
ros2 param set /yolo_node confidence 0.7
ros2 action send_goal --feedback /cobot/move_to_pose cobot_interfaces/action/MoveToPose "{x: 0, y: -800, z: 600, rx: 180, ry: 0, rz: 90, motion_profile: descend}"
ros2 topic hz /cobot/state                         # the 10 Hz poll
ros2 topic echo /cobot/state --once                # named sensors, pose, alarms, poll_ms
ros2 service call /cobot/set_output_bit cobot_interfaces/srv/SetOutputBit "{name: gripper, value: true}"
ros2 service call /cobot/pause std_srvs/srv/Trigger   # then /cobot/resume
ros2 topic echo /task/status
```

[vision_workspace/tests/](vision_workspace/tests/) holds standalone scripts
that exercise the **retired** `motoman_cpp` pybind module directly against a
live controller (`192.168.1.31`). They need that module built (delete
`ros2_ws/src/motoman_cpp/COLCON_IGNORE`) and they move the arm; run them only
in a safe cell. The ROS stack itself no longer uses the module.

The [legacy/](legacy/) tree is the asyncio monolith the ROS stack replaced. It
is kept for reference (the choreography in `cobot.py`, the ZMQ receiver and the
web templates were ported from it) and is not built.

---

## 11. Training and exporting the vision models (offline)

All training and export happens on the dev PC in `vision_workspace/.venv` —
**never inside the docker image**, which only runs inference. The full
workflow (setup, train, YOLOE prompt export, ONNX export) is documented in
[vision_workspace/README.md](vision_workspace/README.md). In short:

```powershell
cd vision_workspace
.venv\Scripts\python tools	rain_yolo_obb.py          # datasets/cans_obb   -> weights/cans_obb_best.pt
.venv\Scripts\python tools	rain_green_light.py       # datasets/green_light -> weights/green_light_best.pt
.venv\Scripts\python tools\export_to_onnx.py cans_obb_best   # weights/*.pt -> models/<name>/<name>.onnx + manifest
.venv\Scripts\python tools\export_yoloe.py --prompts "can" "green light"   # open-vocab YOLOE -> models/yoloe_cobot_v1/
```

`vision_workspace/models/` holds ONNX exports only; `.pt` weights live in
`vision_workspace/weights/` (gitignored). `yolo_node` currently still loads
`weights/cans_obb_best.pt` — switching it to the ONNX manifests is a later step.

Collect more training frames with the UI's **Save frames** button
(`/camera/save_frames`) — they land in `vision_workspace/operation/captures`.

---

## 12. Known gaps and things to watch

- **Single-layer pick cycle.** `task_manager` scans once and picks every
  target; it does not yet call `/vision/verify_target` before each grip or
  `/vision/get_layer_height` to decide whether to rescan / move to the next
  layer / declare the bin complete. The services and the tier-1 sensing
  design ([docs/ros2_architecture.md](docs/ros2_architecture.md) §5) are ready
  for it.
- **Pick height is constant.** `cobot.py` uses `PICK_TIP_Z_MM = 300` /
  `APPROACH_TIP_Z_MM = 401` and ignores `CanTarget.real_z`; measured depth is
  not yet wired into the pick height. `tool_head_compression_distance_mm` is
  declared but unused.
- **Camera intrinsics are provisional.** The streamer's colour `fx/fy`
  disagree by 33 %; a rigid fit against them came out at ~95 mm RMS, so the
  shipped config uses the full monolithic fit (`require_intrinsics: false`).
  `camera_intrinsics.json` documents how to recover true intrinsics from a
  good full fit and switch back to the fast extrinsic-only boot fit.
- **Manual marker clicking is the default** (`manual_dot_selection: true`)
  because the red-dot model is not yet trusted. The automatic path
  (`/vision/locate_dot`, `red_dot_best.pt`) is implemented and selected by
  flipping that flag.
- **`docs/ros2_architecture.md` still says "design draft — nothing implemented
  yet"** in its header; the document is otherwise accurate and is the best
  explanation of the reasoning behind each node.
- `.gitmodules` refers to paths from the legacy layout (`cpp/third_party/pybind`,
  `python/cobot_application/...`) that no longer exist; the ROS build uses
  Humble's `pybind11_vendor` instead and does not need the submodules.
- Depth must be **registered to colour** in the streamer: localization samples
  depth at colour pixels and back-projects with colour intrinsics.
