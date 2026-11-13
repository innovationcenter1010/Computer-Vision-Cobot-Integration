# Computer-Vision-Cobot-Integration

A ROS 2 (Humble) bin-picking stack: an Orbbec RGB-D camera finds cans with a
YOLO oriented-bounding-box model, a fitted camera→robot calibration turns pixels
into robot-base millimetres, and a Yaskawa Motoman cobot (driven through the
YMConnect SDK from a C++ ROS node) picks each can and places it. A FastAPI
web UI streams the camera feeds, shows live status, and issues the
start / stop / calibrate commands.

Everything runs as **one Docker container** (`docker compose up`) holding eight
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
│   │  camera_node ─▶ detector_node ─▶ localization_node ─▶ task_manager ─▶ cobot_ctrl ─┼──┼──TCP──▶ Motoman controller
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
  published. A `health_monitor` node aggregates per-node heartbeats into
  `/health/summary` for the dashboard.

Bind mounts (see [docker-compose.yaml](docker-compose.yaml)):

| Host path | Container path | Purpose |
|---|---|---|
| `./runtime_models` | `/ws/runtime_models` | `models.yaml` registry + ONNX exports — what `detector_node` loads |
| `./vision_workspace` | `/ws/vision_workspace` | camera intrinsics JSON, captures, results |
| `./ros2_ws/src/cobot_bringup/config` | `/ws/install/share/cobot_bringup/config` | `cobot_params.yaml`, `cobot_setup.yaml`, the packaged `trajectories.json` seed, and the per-machine `calibration.yaml` — editable and persisted without an image rebuild |
| `./runtime_config` | `/ws/runtime_config` | runtime-WRITABLE per-cell state (gitignored): the live `trajectories.json` (remembered positions, each with its tool file), `tooling_state.yaml` (active tool), `controller_profile/` (raw TOOL.CND / UFRAME.CND dumps) — see `runtime_config/README.md` |

---

## 2. Quick start

Prerequisites: Docker Desktop, the external camera streamer running on the
host, and the robot reachable at the IP in `cobot_params.yaml`.

```bash
# 1. Make sure runtime_models/models.yaml points at exported ONNX models
#    (runtime_models/<name>/<name>.onnx + manifest.yaml — exported on the dev
#    PC, see vision_workspace/README.md; .pt weights never enter the container)

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
#   launch args: camera / vision / cobot / tasks / web / health   (all default true)
```

> **Safety:** `task_manager.auto_calibrate_on_startup` is `false` in the shipped
> config — nothing moves at boot; the operator starts calibration from the
> dashboard. Setting it to `true` re-enables the automatic boot run, and then
> **the arm moves at launch** once the robot and vision services are up.

First-time bring-up order:

1. Bring the robot up once, open the dashboard's **Tooling** card (or
   `ros2 topic echo /cobot/profile --once`) and note which YRC1000 tool files
   exist. Put their numbers into `cobot_setup.yaml` `tools.<name>.tool_number`,
   measure `tools.<name>.marker_offset_mm` for each tool you will calibrate with
   (and `tools.none.tip_offset_mm` for tool 0) — see [§6](#6-configuration). If
   `/cobot/profile` reports an RS7 error, set parameter **RS7 = 2** on the
   pendant (file reads outside REMOTE mode).
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
 streamer ───▶ camera_node ─┬─▶ /camera/color/image_raw ──────┬─▶ detector_node (C++ / ONNX Runtime)
                            ├─▶ /camera/depth/image_raw ──────┼──────────────▶ localization_node
                            └─▶ /camera/color/camera_info ────┼──────────────▶ localization_node
                                (latched)                     └─▶ web_node
                                                                                │
   detector_node ── /vision/detections_px (pixel space, mirror) ────────────────┘
        ▲  action /vision/detect {model_id, image_source}   (models.yaml: runtime / calibration)
        │
   localization_node ── action /vision/scan {model_id, image_source, localize} → TargetArray
        │             ── /vision/targets (robot space) ──┬─▶ task_manager
        srv /vision/verify_target                       └─▶ web_node
        srv /vision/get_layer_height
        srv /vision/calibration_begin|sample|compute
        topics /vision/manual_dot_request|click|skip  (temporary manual mode)

   browser ══ws══▶ web_node ── srv /task/start_workflow {id} ──▶ task_manager
                      ▲          (and /task/start|stop|calibrate aliases)
                      ├── /task/status, /task/workflows, /task/system_state (latched)
                      ├── /task/layer_report, /task/pick_event
                      │
   every node ── /health/heartbeat ──▶ health_monitor ── /health/summary (latched) ──▶ web_node, task_manager
                      │
   task_manager ── action /cobot/pick_can ──▶ cobot_operation ── MoveToPose / set_output_bit ──▶ cobot_control ──▶ robot
                ── action /cobot/hide ─────────────────────▶ cobot_operation ── MoveToPose ──▶ cobot_control
                ── action /cobot/move_to_pose ─────────────────────────────────────────▶ cobot_control  (calibration grid only)
                ── srv /vision/verify_target (after each place)
                      ◀────────────── /cobot/status (latched) ───────┘
```

Design rules that the graph enforces (from [docs/ros2_architecture.md](docs/ros2_architecture.md)). `task_manager` never talks to `cobot_control` for anything the arm's remembered positions / tooling can affect — those moves are always routed through `cobot_operation` (`/cobot/pick_can`, `/cobot/hide`); the sole exception is the calibration workflow's grid, which drives the marker through an unmeasured, config-defined sweep that has no teach point to own:

| Node | Owns | Never does |
|---|---|---|
| `task_manager` | the workflow registry, task sequencing, gating, progress reporting | geometry, robot/camera access |
| `cobot_operation` | pick choreography (`/cobot/pick_can`), the "hide" park move (`/cobot/hide`) | decisions, vision calls, task state, robot code |
| `cobot_control` | hardware primitives (YMConnect) | choreography, decisions |
| `localization_node` | geometry + calibration math | robot commands, ML |
| `detector_node` | ML inference (C++, ONNX Runtime; any model in `models.yaml`) | geometry, robot commands |
| `camera_node` | ZMQ→ROS bridging | anything else |
| `web_node` | browser bridge | logic — every command is forwarded to a service |
| `health_monitor` | heartbeat aggregation (`/health/summary`) | anything else |

Two consequences worth internalising:

- **The cobot never sees pixels, the detector never sees millimetres.** `ObbDetection`
  is pixel-only; `Target` is robot-frame only. Only `localization_node`
  converts between them (its `/vision/scan` action is the one entry point).
  A `Target` does carry its scan-time `pixel_x/y` + `depth_mm` as provenance —
  but only so `/vision/verify_target` can re-sample the exact pixel later;
  the choreography never reads them.
- **`/vision/targets` is data, not a trigger.** Nothing moves because a topic was
  published. `task_manager` only consumes a targets message while a scan *it*
  requested is in flight, and it drives `cobot_control` through action goals.

QoS contract: images are best-effort keep-last(1) (freshest wins);
`camera_info`, `/cobot/status`, `/task/status`, `/vision/manual_dot_request` are
reliable + transient-local (latched); everything else is reliable depth 10.

---

## 4. Packages and nodes

All packages live under [ros2_ws/src/](ros2_ws/src/). Dependency rule: every
package depends only on `cobot_interfaces` and stock messages (plus
`cobot_health` for the heartbeat helper); `cobot_control` is C++ and links
YMConnect; `cobot_tasks` depends on `rclpy` + `cobot_interfaces` +
`cobot_health` *only*; nothing depends on `cobot_bringup`.

### 4.1 `cobot_interfaces` — message contracts

| Type | Name | Fields / purpose |
|---|---|---|
| msg | `ObbDetection` | `class_id`, `confidence`, `center_x/y` (px), `corners_px[8]` (TL,TR,BR,BL) — **pixel space only** |
| msg | `ObbDetectionArray` | `header` (stamp = the color frame the model ran on), `image_width/height`, `detections[]` |
| msg | `Target` | `class_id`, `confidence`, `real_x/y/z` (mm, robot base), `real_rz` (deg) — **robot space only** — plus scan-time provenance: `detection_index`, `pixel_x/y`, `depth_mm` (for `verify_target`; −1 = unknown) |
| msg | `TargetArray` | `header`, `model_id`, `targets[]` — one ordered pick layer |
| msg | `CobotStatus` | **hardware** state: `DISCONNECTED / IDLE / BUSY / ERROR` + `detail` |
| msg | `TaskStatus` | **operational** state: `DISCONNECTED / IDLE / SCANNING / PICKING / CALIBRATING / STOPPING / ERROR` + `detail`, `active_task`, `progress_current/total` |
| msg | `LayerReport` | `successes`, `failures`, `overall_ok` |
| msg | `PickEvent` | one per can on `/task/pick_event`: `index/total`, `target`, `outcome` (picked/failed/skipped), `verify_ran`, `verified`, `measured_z` |
| msg | `SystemState` | derived capability (`CAP_NONE/SEE/MOVE/OPERATE`) + individual robot/vision facts, latched on `/task/system_state` |
| msg | `WorkflowInfo` / `WorkflowList` | registry entry (`id`, `label`, `ready`, `not_ready_reason`, `requires`) for the dashboard selector |
| msg | `Heartbeat` / `NodeHealth` / `HealthSummary` | per-node health on `/health/heartbeat`, aggregated on `/health/summary` |
| msg | `CalibrationMarker` | one per calibration sample (accepted/rejected, `u/v/depth_mm`, mode) — feeds the dashboard overlay |
| srv | `StartWorkflow` | `workflow (id) → success, message` — the one entry point for tasks |
| srv | `VerifyTarget` | `Target target → bool present, float64 measured_z` — re-samples depth at the target's remembered scan pixel |
| srv | `GetLayerHeight` | `() → float64 median_z, int8 layer_index` (tier-1 depth check) |
| srv | `CalibrationBegin` | `mode (default/manual/automatic), model_id, … → success, mode_name` — per-run marker-source override |
| srv | `CalibrationSample` | `x, y, z (commanded TCP) → success, sample_count, message` |
| action | `Detect` | goal `model_id, image_source` → result `ObbDetectionArray, inference_ms`; feedback `stage` |
| action | `Scan` | goal `model_id, image_source, localize` → result `TargetArray` (detect + depth + transform + ordering) |
| action | `PickCan` | goal `Target target` → result `success, message`; feedback `phase` (check/approach/grip/lift/place) |
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
- Heartbeat health is judged on the *measured* color/depth Hz against
  `publish_rate_warn_hz` / `publish_rate_error_hz`, independently of
  `publish_rate_hz` — retuning the thresholds never changes how often the
  camera is actually polled. A stream older than 2 s (or that has never
  produced a frame) is always ERROR regardless of these thresholds.

Parameters: `color_endpoint`, `depth_endpoint`, `publish_rate_hz`,
`publish_rate_warn_hz`, `publish_rate_error_hz`,
`color_frame_id`, `depth_frame_id`, `capture_dir`, `intrinsics_file`.

### 4.3 `cobot_detector` / `detector_node` (C++)

Pure perception on **ONNX Runtime**. Source: [ros2_ws/src/cobot_detector/](ros2_ws/src/cobot_detector/)
(package README explains ORT and the decode path).

- Subscribes `/camera/color/image_raw`, caches the latest frame + stamp.
- Action `/vision/detect` (`Detect`) — goal `model_id`, a key of the bind-mounted
  [`models.yaml`](ros2_ws/src/cobot_detector/config/models.example.yaml)
  (`runtime:` e.g. `cans`, `calibration:` e.g. `red_dot`). Runs on a worker
  thread with stage feedback; the result carries the `ObbDetectionArray`
  (`header.stamp` = the color frame's stamp, `model_id` echoed) and
  `inference_ms`. Goal `image_source` (optional container path) runs the model
  on that file instead of the cached frame — no camera needed (bench tests, the
  `vision_test` workflow). Rejected when the id is unknown, no frame has
  arrived (and no `image_source`), the file does not exist, or a detection is
  already running. Also mirrored on `/vision/detections_px`.
- OBB and plain-bbox models share the message: bbox → axis-aligned corners,
  `angle = 0`.
- Per-model thresholds (`confidence`, `iou`, `max_detections`, duplicate
  filter) live in `models.yaml`; models lazy-load on first use unless in
  `preload`. No ultralytics / torch in the container.

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
shifting. Published on `/vision/targets` (`TargetArray`,
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
| `/vision/calibration_begin` | Refuse if no `calibration_file`, no depth frames, or (with `require_intrinsics`) no intrinsics yet. In automatic mode also waits for `/vision/detect` and warms the dot model (`dot_model_id`) (≈ 110 s budget). Clears the sample buffer |
| `/vision/calibration_sample` | "Marker is parked at TCP `(x,y,z)` — measure now." Gets `(u,v)` either from `dot_samples_per_pose` `/vision/detect` (dot model) reads (median, majority must hit) **or** from an operator click (manual mode), samples its own depth buffer at that pixel, adds `marker_offset_mm` to the commanded TCP, stores the pair |
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
| action `/cobot/move_to_pose` | One supervised move. Goal picks a named motion profile from `cobot_setup.yaml` (joint or **linear** interpolation, speed, accel/decel, accuracy) with per-goal overrides; the pose is expressed in `goal.tool_number` (−1 = the **active tool**). IK reachability check, `AddPointToTrajectory` → `MotionStart`, then the move loop waits on the poll (below) for progress. Feedback `progress_percent`, `steps_remaining`, `paused`. **Cancel is immediate**: `MotionStop(true)`, trajectory cleared |
| topic `/cobot/profile` (latched) | The YRC1000 profile, read **once** after every connect and strictly **read-only**: model + software version, every tool file from `TOOL.CND` (TCP, weight, COG), user frames from `UFRAME.CND`, per-section errors, the configured tool names and the active tool. `/cobot/reload_profile` re-reads on demand; `profile_source: fixture` serves `test/data` without a robot |
| srv `/cobot/set_active_tool` | Select the tool file every pose is expressed in: `none` (tool 0) or a `cobot_setup.yaml tools` key (validated against `TOOL.CND`, refused mid-move). Persisted in `runtime_config/tooling_state.yaml`, restored at boot, visible on `/cobot/state` (`active_tool_*`, `pose_tool_number`, `joints_deg`) |
| srv `/cobot/convert_pose` | `cart(tool A)` → joints → `cart(tool B)` through the controller kinematics — no motion, nothing written. Used to migrate remembered positions between tool files; returns the joints too |
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
| action `/cobot/hide` | One `MoveToPose` to the remembered `hide` teach point (`trajectories.json`) — parks the arm 100% out of the camera's view. `pick_layer` calls this before every scan instead of talking to `cobot_control` directly, so the same tool/trajectory ownership and busy-lock serialization as `/cobot/pick_can` apply. Rejected if no `hide` point is configured yet (`/cobot/set_teach_point` adds one) or the stored positions don't match the active tool. Empty goal; result is `success`/`message`. Cancel is immediate, same as a pick |
| pause | `cobot_control`'s `/cobot/pause` holds the arm and the choreography sees `CobotState.paused`, so it also waits before its next primitive. One service pauses both layers |

The execute callback is a coroutine: it `await`s the action / service futures,
so no executor thread is ever parked and goal / cancel requests stay
serviceable mid-pick. Sensor assertions wait up to `sensor_assert_timeout_s`
for a matching `/cobot/state` sample (the old 5 × 0.2 s re-read loop).

A raw IO sensor bit is **not** treated as ground truth for a pick's
success/failure: a mismatched assert (e.g. `existance_check` reading `false`
after lift or place) is always recorded on the result (`sensor_asserts[]`)
and in the report/UI, but by default does not fail the `PickCan` goal —
sensors can be misaligned or flaky, and the actual post-pick confirmation is
`pick_layer`'s vision depth check (`/vision/verify_target`), which runs after
the result. Set `sensor_assert_authoritative: true` in `cobot_params.yaml`
(bind-mounted, no rebuild) to restore the old behaviour of failing the pick
on a sensor mismatch.

Per-tool cell values live in [cobot_setup.yaml](ros2_ws/src/cobot_bringup/config/cobot_setup.yaml)
(`tools.<name>`): the YRC1000 `tool_number`, `marker_offset_mm` (calibration
dot relative to that tool's TCP), `pick_tip_z_mm` / `approach_tip_z_mm` and the
descent motion profile. `none` (tool 0) is mandatory and is the only entry
allowed a `tip_offset_mm`: `ToolOffset` then converts a vision-derived *can*
position into the pose to command so that the tool **tip** lands on the can.
With a real tool file active the controller's TCP already is the tip, so no
Python offset is applied (a non-zero `tip_offset_mm` there is a load error — the
TCP math is never duplicated). The **active tool** is selected on the dashboard
(`/cobot/set_active_tool`) and read from `/cobot/state`; every pose
`cobot_operation` sends carries an explicit tool file number.

`cobot_setup.yaml` → `mount_yaw_offset_deg` (top-level, default `0.0`) is added to
every commanded `MoveToPose` `rz` in `cobot_control::executeMove`, just before the
pose is handed to YMConnect — it compensates for the physical toolhead being
clocked off the taught/commanded convention (currently `90.0`: the toolhead sits
90° rotated from every taught and vision-derived pose). The sum is passed through
`normalizeDeg180()` (`motoman_robot.hpp`) so it always lands in `[-180, 180)` —
the wrist's (T-axis) periodic Euler convention — instead of an out-of-range value
like `190` if the taught `rz` plus the offset would otherwise overflow; `rx`/`ry`
and the other joints are never touched by this, only `rz`. It applies to *every*
move regardless of caller (`cobot_operation` choreography **and** the
`task_manager` calibration grid), and only on the outbound command — read-back
paths (`/cobot/state`, `ReadPositionVar`, `ConvertPose`) are untouched, so
teaching new positions and `trajectories.json` keep the same convention they
always have. It is a pure ROS-layer correction: the YRC1000's own tool
files/frames are never touched. Because the wrapped, corrected pose is built
before the `check_reachability`/`jointLimitsOk` check, joint limits are enforced
against the true corrected angle, not the raw taught value.

Remembered positions live in `runtime_config/trajectories.json` (v2 — each pose
has `frame`, `tool_number`, canonical `joints_deg`; a v1 file is migrated with a
`.v1.bak`). When the active tool changes, `cobot_operation` converts every stored
pose through `/cobot/convert_pose` and rewrites the file (all-or-nothing;
`/cobot/convert_trajectories` retries it manually, `/cobot/save_teach_point`
overwrites one position with the current pose + joints). A pick is refused
until the stored positions match the active tool.


### 4.7 `cobot_tasks` / `task_manager`

The decision layer — owns *what* the system is doing. It hosts a registry of
**workflows** and runs one at a time; it never does geometry or touches the
camera / robot directly.
Source: [task_manager_node.py](ros2_ws/src/cobot_tasks/cobot_tasks/task_manager_node.py),
[workflows/](ros2_ws/src/cobot_tasks/cobot_tasks/workflows/),
[context.py](ros2_ws/src/cobot_tasks/cobot_tasks/context.py),
[system_state.py](ros2_ws/src/cobot_tasks/cobot_tasks/system_state.py).
Depends on `rclpy` + `cobot_interfaces` + `cobot_health` only.

**Workflows.** Each task the operator can start is a `Workflow` subclass in
`workflows/<name>.py`, registered in `workflows/__init__.py` `REGISTRY`
(order = dashboard order): `pick_layer`, the calibration variants
(manual / automatic marker source, full and startup grids) and `vision_test`.
A workflow declares its ROS parameters (`ParamSpec`), the action/service
clients it needs (`ClientSpec` — the node creates one client per key, shared
across workflows) and a prerequisite capability (`requires`). Workflows never
import rclpy — all sequencing goes through a `WorkflowContext`, which keeps
them unit-testable with a fake ([test/test_workflows.py](ros2_ws/src/cobot_tasks/test/test_workflows.py)).
Adding a task changes nothing in the node itself.

**Capability derivation** (`system_state.py`): `/cobot/state` +
`/health/summary` are re-derived every second into a `SystemState`
(`none / see / move / operate`) published latched on `/task/system_state`;
each workflow's `requires` is checked against it and the result is published
as the readiness flag on `/task/workflows` — the dashboard greys out what
cannot run and shows why.

**Interfaces:**

- `/task/start_workflow` (`StartWorkflow`) runs one workflow by id;
  `/task/start` and `/task/calibrate` (`Trigger`) are aliases for the
  `default_workflow` parameter (`pick_layer`) and `calibration`. `/task/stop`
  lives in its own callback group so it is reachable while a workflow runs.
- Publishes `/task/status` (`TaskStatus`, latched) — **the** operational state
  the UI shows (`active_task` = workflow id) — `/task/system_state`,
  `/task/workflows`, `/task/layer_report` and `/task/pick_event` (one
  `PickEvent` per can, carrying the post-place verify verdict).
- `/cobot/status` is mirrored into `/task/status` as `DISCONNECTED` / `IDLE`
  when no workflow is running.

State machine (states carried by `TaskStatus`):

```
IDLE ──start──▶ SCANNING ──targets──▶ PICKING ──layer done──▶ IDLE
  │                 └─ timeout / refused ──▶ ERROR      └─ no cans ──▶ IDLE
  └─calibrate─▶ CALIBRATING ── compute ok ─▶ IDLE   (fit rejected ─▶ ERROR)
any task ──stop──▶ STOPPING ── active goal cancelled ─▶ IDLE
```

One workflow at a time (`_operation_lock` held for the whole run, on a
background worker thread). While a goal is in flight the worker polls
`stop_requested` and cancels the goal natively, so a stop interrupts a pick at
its next safe phase boundary instead of waiting for the next can.

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
- Forwards the graph to the browser as typed websocket messages:
  `/vision/targets`, `/task/status` (as type `cobot_status`),
  `/task/layer_report`, `/task/pick_event`, `/task/workflows`,
  `/task/system_state`, `/health/summary`, `/vision/detections_px` (pixel OBBs
  for the overlay), `/vision/calibration_marker`, `/cobot/profile`
  (`tool_profile`), a throttled `/cobot/state` digest (`cobot_state`,
  `tool_state`) and `/vision/manual_dot_request`. Latched topics are replayed
  to every newly connected page.
- A server-side **session recorder** (`session_recorder.py`, `RECORD`
  start/stop commands) captures a session and exports a standalone replay
  HTML into `results_dir`, downloadable at `/results/`.
- Routes websocket commands `{"receiver","command","payload"}` through a
  string→service table (`COMMAND_ROUTES`) — see [§7](#7-web-ui-and-api).
- `GET/POST /api/config` reads/writes `cobot_params.yaml` and, on POST, also
  calls each node's `set_parameters` so accepted changes apply live.

### 4.9 `cobot_bringup` — launch + config

- [robot.launch.py](ros2_ws/src/cobot_bringup/launch/robot.launch.py) — starts
  the six nodes, all reading `config/cobot_params.yaml`, with per-node
  `respawn=True` and launch args `camera / vision / cobot / tasks / web`. It
  also injects the paths: `localization_node.calibration_file`, `*.setup_file`,
  `web_node.params_file` (bind-mounted config dir), `cobot_control.runtime_dir` and
  `cobot_operation.trajectories_file` (`runtime_dir:=/ws/runtime_config`, the
  runtime-writable mount) plus `cobot_operation.trajectories_default_file` (the seed).
- [cobot_params.yaml](ros2_ws/src/cobot_bringup/config/cobot_params.yaml) —
  every runtime parameter (see [§6](#6-configuration)).
- [trajectories.json](ros2_ws/src/cobot_bringup/config/trajectories.json) —
  the v2 **seed** for the remembered positions (`check`, and two-point `right`,
  `right_flip`, `left`, `left_flip` place trajectories, each pose with its tool
  file); copied to `runtime_config/trajectories.json` on first boot, which is
  the live, runtime-rewritten file.
- `config/calibration.yaml` — **generated**, gitignored, per machine. Written by
  `/vision/calibration_compute`.

### 4.10 `cobot_health` / `health_monitor`

Source: [health_monitor_node.py](ros2_ws/src/cobot_health/cobot_health/health_monitor_node.py),
[heartbeat.py](ros2_ws/src/cobot_health/cobot_health/heartbeat.py).

Every node publishes a `Heartbeat` on `/health/heartbeat` every ~2 s (via the
shared `HeartbeatPublisher` helper — level, one-line detail, per-node metrics).
`health_monitor` aggregates them into a latched `HealthSummary` on
`/health/summary` once per second: a node never seen is `UNKNOWN`, one whose
heartbeat is older than `stale_factor ×` its own period is `STALE` (both count
as ERROR severity in the summary level), otherwise its self-reported level is
used. `expected_nodes` lists what must appear; anything else that publishes is
appended, so a new node shows up without a config change. The summary feeds
the dashboard's health panel and `task_manager`'s capability derivation.

---

## 5. How an operation actually runs

### 5.1 Pick cycle (the `pick_layer` workflow)

```
browser "Start"  ──ws──▶ web_node ──▶ /task/start_workflow {pick_layer}
task_manager: capability = operate? /vision/scan + /cobot/pick_can + /cobot/hide ready? lock free?
  ├─ publish SCANNING "moving to hide position"
  │    action /cobot/hide ──▶ cobot_operation ── MoveToPose ──▶ cobot_control
  │    (ERROR if the move fails — never scan with the arm possibly in frame)
  ├─ publish SCANNING "requesting scan"
  ├─ action /vision/scan ──▶ localization_node ──▶ action /vision/detect ──▶ detector_node (ONNX)
  │                                │◀─────────────── ObbDetectionArray ◀────────────┘
  │                                └─▶ depth lookup + M transform + rz + snake order
  │                                      └─▶ result TargetArray + /vision/targets (N ordered Targets)
  │    (ERROR on scan failure, IDLE on zero cans)
  ├─ for each target (PICKING, progress i/N):
  │     send PickCan goal ──▶ cobot_operation.pick_and_place  (check→approach→grip→lift→place)
  │     after the result: srv /vision/verify_target ("second depth scan") —
  │       re-sample depth at the target's remembered scan pixel; the can gone = verified
  │     publish /task/pick_event {outcome, verify_ran, verified, measured_z}
  │     count success / failure; stop request cancels the goal at a safe boundary
  ├─ publish /task/layer_report {successes, failures, overall_ok}
  └─ publish IDLE "layer done: X ok / Y failed"
```

The detector runs **once per layer**. The post-place verify runs only *after*
the PickCan result — during the pick the arm covers the camera — and its
verdict is reported on `/task/pick_event` only; it does not yet gate the layer
or trigger a rescan, and `get_layer_height` (for a tiered multi-layer loop) is
still uncalled — see [§12](#12-known-gaps-and-things-to-watch).

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
   call /vision/calibration_sample(x,y,z) ───▶ (u,v) from /vision/detect (dot model) or a UI click,
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

**Four workflow variants** share this loop
([workflows/calibration.py](ros2_ws/src/cobot_tasks/cobot_tasks/workflows/calibration.py)):
`calibration` / `startup_calibration` (manual — the operator clicks the marker
at each pose) and their automatic twins `auto_calibration` /
`auto_startup_calibration` (the detector finds the marker; no operator input,
so the grid is denser). How the marker pixel is found is pinned **per run** in
the `CalibrationBegin` request (mode / model_id / class_id /
samples_per_pose), so the variants coexist without touching localization's
`manual_dot_selection` parameter. Every sample is echoed on
`/vision/calibration_marker` for the dashboard's verification overlay.

**Manual centre-select mode**:
at each pose the UI shows a prompt over the colour feed; the operator clicks
the marker centre (the browser scales the click to natural image pixels) or
presses *"Marker not in frame — skip pose"*. Keep
`task_manager.calibration_sample_timeout_sec` (180) above
`localization_node.manual_click_timeout_sec` (120) or the task manager gives up
while you are still looking at the screen.

**Offsets and tool files.** The controller commands the TCP of the **active tool
file**; the calibration is commanded in that tool and records it:

| Offset | Where | Applied by | To |
|---|---|---|---|
| red marker | `cobot_setup.yaml` `tools.<name>.marker_offset_mm` (per tool file) | localization, *before* fitting (active tool from `/cobot/state`) | each sample's ground truth = commanded TCP + that tool's offset |
| tool tip | `cobot_setup.yaml` `tools.none.tip_offset_mm` (tool 0 only) | `cobot_operation/geometry.py` `ToolOffset` | vision-derived pick poses while `none` is active: commanded = can − offset |

Because the marker offset is accounted for during the fit, `/vision/targets`
are true robot-base positions of the **can** and the fit is tool-independent.
`calibration.yaml` records the tool it was captured with; active tool ≠
calibration tool shows as a warning on the dashboard. Measure both offsets with
a ruler, along robot base axes, at the calibration orientation (`rx=180, ry=0, rz=0`).

**Intrinsics.** With `require_intrinsics: true` the small extrinsic-only
(Kabsch) fit is used against the SDK intrinsics relayed on
`/camera/color/camera_info`. The shipped config sets it to **false** because
this camera's reported colour `fx/fy` are internally inconsistent
(`camera_intrinsics.json` documents the problem and the recipe to recover true
intrinsics from a full fit's matrix).

### 5.3 Automatic extrinsic calibration at boot (opt-in, off by default)

Calibration is normally operator-selected from the dashboard's workflow list.
With `auto_calibrate_on_startup: true`, a background thread in `task_manager`
waits up to `startup_calibration_wait_sec` for the robot and the vision
services, then runs the configured startup workflow
(`startup_calibration_workflow`, default `startup_calibration`; set it to
`auto_startup_calibration` for the detector-driven variant) on a smaller grid
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
| `cobot_setup.yaml` → `tools.<name>.tool_number` | the YRC1000 tool file behind each cell profile (read the numbers off the dashboard's Tooling card) |
| `cobot_setup.yaml` → `tools.<name>.marker_offset_mm` | red marker relative to THAT tool file's TCP, robot-base axes (replaces `localization_node.marker_offset_mm`) |
| `cobot_setup.yaml` → `tools.none.tip_offset_mm` | tool tip relative to the tool-0 TCP, robot-base axes (only for `none`) |
| `task_manager.calibration_x_range_mm`, `calibration_y_range_mm` | safe, reachable, in-frame calibration bounds (`[0,0]` = unconfigured → calibrate refuses) |
| `task_manager.calibration_z_levels_mm`, `startup_calibration_z_levels_mm` | heights for the full and boot grids — **floats with `.0`** (an int array fails to load) |
| `task_manager.auto_calibrate_on_startup` | arm moves at launch when true |

Other notable parameters:

| Node | Parameter | Notes |
|---|---|---|
| `camera_node` | `color_endpoint`, `depth_endpoint` | ZMQ SUB endpoints (`tcp://host.docker.internal:5550/5551`) |
| `camera_node` | `intrinsics_file`, `capture_dir` | in the `vision_workspace` mount |
| `detector_node` | `models_yaml`, `preload`, `intra_op_threads` | per-model thresholds live in `models.yaml` |
| `localization_node` | `default_model_id`, `dot_model_id`, `detect_timeout_sec` | which `models.yaml` ids a scan / the calibration use |
| `localization_node` | `pathing_row_axis` (`"x"`/`"y"`), `pathing_row_tolerance_mm` | snake ordering |
| `localization_node` | `can_height_mm`, `depth_at_top_layer_mm` | for `get_layer_height`; 0 = unconfigured |
| `localization_node` | `depth_buffer_size`, `depth_sample_window_px`, `verify_tolerance_mm` | depth sampling |
| `localization_node` | `dot_samples_per_pose`, `calibration_min_samples`, `calibration_max_rms_mm`, `require_intrinsics` | fit gating |
| `localization_node` | `manual_dot_selection`, `manual_click_timeout_sec` | temporary manual mode |
| `cobot_control` | `poll_period_s`, `connect_retry_period_s`, `motion_timeout_s`, `pause_uses_hold`, `check_reachability`, `pendant_messages`, `poll_joints` | 10 Hz poll, reconnect, move supervision |
| `cobot_control` | `profile_source`, `profile_fixture_dir`, `profile_tool_file`, `profile_uframe_file`, `profile_dump` (+ `runtime_dir` from launch) | read-once controller profile (`fixture` = zero-hardware), raw dumps, `tooling_state.yaml` |
| `cobot_operation` | `move_timeout_s`, `service_timeout_s`, `sensor_assert_timeout_s`, `sensor_assert_authoritative`, `auto_convert_on_tool_change` | primitive budgets, automatic conversion of remembered positions; `sensor_assert_authoritative` (default `false`) gates whether a sensor-assert mismatch fails the pick |
| `cobot_setup.yaml` | `io.*`, `motion_profiles.*`, `tools.*` | named IO, joint/linear motion profiles, per-tool cell values (`none` mandatory; `compression_mm` declared, **not yet applied**). The active tool is runtime state (`runtime_config/tooling_state.yaml`), not a key here |
| `cobot_setup.yaml` | `mount_yaw_offset_deg` | added to every commanded `rz` in `cobot_control` (all tools, all callers) to compensate for the physical toolhead's mounting rotation — software-only, never written to the YRC1000 |
| `task_manager` | `default_workflow`, `scan_timeout_sec`, `calibration_grid_size`, `calibration_orientation_deg`, `calibration_settle_sec`, `calibration_sample_timeout_sec`, `startup_calibration_workflow`, `startup_calibration_grid_size`, `startup_calibration_wait_sec` | `/task/start` runs `default_workflow` (`pick_layer`); auto variants read `auto_calibration_*` / `auto_startup_calibration_*` grids |
| `health_monitor` | `expected_nodes`, `stale_factor`, `summary_period_s` | |
| `web_node` | `port`, `web_stream_hz`, `web_jpeg_quality` (70), `web_depth_scale` (0.5), `results_dir` | quality/scale affect the browser preview only |

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
| `COBOT` / `start_workflow` `{workflow}` | `/task/start_workflow` | run one workflow by id (the dashboard selector) |
| `COBOT` / `start`, `pickup` | `/task/start` | alias: the `default_workflow` (`pick_layer`) |
| `COBOT` / `stop` | `/task/stop` | cancel the active goal |
| `COBOT` / `calibrate` | `/task/calibrate` | alias: the `calibration` workflow |
| `COBOT` / `set_active_tool` `{name}` | `/cobot/set_active_tool` | select the tool file (remembered positions auto-convert) |
| `COBOT` / `convert_trajectories` | `/cobot/convert_trajectories` | manual retry of the position conversion |
| `COBOT` / `teach_point` `{key, index}` | `/cobot/save_teach_point` | overwrite one remembered position with the current pose |
| `CAMERA` / `save` | `/camera/save_frames` | write a PNG pair to `vision_workspace/operation/captures` |
| `CAMERA` / `snapshot` | (web_node) | push one frame immediately |
| `VISION` / `click_center` `{u,v}` | topic `/vision/manual_dot_click` | manual calibration click |
| `VISION` / `skip_center` | topic `/vision/manual_dot_skip` | drop the current calibration pose |
| `RECORD` / `start`, `stop`, `status` | (web_node) | session recorder; a stop exports a replay HTML into `results_dir` |

Server → browser message types: `connection`, `camera_frame`, `command_result`,
`targets`, `detections` (pixel OBB overlay), `cobot_status` (carries `state`,
`detail`, `active_task`, `progress_current/total` from `TaskStatus`),
`layer_report`, `pick_event`, `workflows`, `system_state`, `health`,
`calibration_marker`, `tool_profile`, `cobot_state`, `tool_state`,
`recording`, `manual_dot_request`.

HTTP: `GET /` (UI), `GET /api/config` (params YAML as JSON),
`POST /api/config` (write YAML + live `set_parameters`, returns per-node
outcomes: `applied` / `rejected: [...]` / `node not reachable`),
`GET /api/latest-image` (newest file in `results_dir`), `/static/*`, `/results/*`.

---

## 8. Docker build

[docker/Dockerfile](docker/Dockerfile) is a two-stage build:

1. **builder** (`ros:humble`) — apt build tools; the pinned ONNX Runtime
   release tarball unpacked to `/opt/onnxruntime` (`cobot_detector` links it —
   **ultralytics / torch are not in the image**: training and the `.pt` →
   `.onnx` export happen on the dev PC, see [§11](#11-training-and-exporting-the-vision-models-offline));
   pip deps (`opencv-python-headless`, `fastapi`, `uvicorn[standard]`,
   `pyzmq`, `jinja2`); `setuptools<80` + `packaging>=22` pin for colcon
   compatibility; `package.xml` manifests copied alone → `rosdep install`
   (cached on manifest changes only); then the source →
   `colcon build --merge-install`, followed by
   `colcon test --packages-select cobot_detector cobot_control` — the C++ unit
   tests (decode / NMS / letterbox / model registry, controller-profile and
   setup parsers) run at build time, so a regression fails the image, not the
   robot.
2. **runtime** (`ros:humble-ros-core`) — runtime libs, `rmw_cyclonedds_cpp`,
   the same pip layers, and only the `/ws/install` tree from the builder
   (launch files, message definitions, Python packages, the compiled
   `cobot_control_node` + `libYMConnect.so` and `detector_node` +
   `libonnxruntime.so` beside them). No compilers, no source, no baked model
   weights.

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
├── runtime_models/                # models.yaml registry + ONNX exports (mounted at /ws/runtime_models)
├── runtime_config/                # runtime-WRITABLE per-cell state, gitignored (mounted at /ws/runtime_config)
├── docs/
│   ├── ros2_architecture.md       # full design document (the "why" behind every node)
│   └── dependancies/orbbec_devices/AstraSDK_Doc_en.chm
├── ros2_ws/src/
│   ├── cobot_interfaces/          # msg / srv / action definitions
│   ├── cobot_camera/              # camera_node, camera_streamer (ZMQ), frame_saver (legacy helper)
│   ├── cobot_vision/              # localization_node (/vision/scan action), calibration/pathing/rotation math
│   ├── cobot_detector/            # C++ detector_node on ONNX Runtime (/vision/detect action)
│   ├── cobot_control/             # C++ node on YMConnect (ament_cmake) + vendored SDK in third_party/
│   ├── cobot_operation/           # pick choreography node (Python) + unit tests
│   ├── motoman_cpp/               # RETIRED pybind11 module (COLCON_IGNORE) + the old Python cobot node
│   ├── cobot_tasks/               # task_manager_node + workflows/ (the registry) + unit tests
│   ├── cobot_health/              # health_monitor node + the shared HeartbeatPublisher helper
│   ├── cobot_web/                 # web_node, session_recorder, templates/, static/
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
ros2 action send_goal --feedback /vision/scan cobot_interfaces/action/Scan "{model_id: cans}"
ros2 action send_goal --feedback /vision/detect cobot_interfaces/action/Detect "{model_id: red_dot}"
# detect-only scan on a file: no camera, depth or calibration needed
ros2 action send_goal /vision/scan cobot_interfaces/action/Scan   "{model_id: cans, image_source: /ws/vision_workspace/datasets/cans_obb/images/train/IMG_001.jpeg, localize: false}"
ros2 service call /task/start_workflow cobot_interfaces/srv/StartWorkflow "{workflow: vision_test}"
ros2 topic echo /vision/targets
ros2 action send_goal --feedback /cobot/move_to_pose cobot_interfaces/action/MoveToPose "{x: 0, y: -800, z: 600, rx: 180, ry: 0, rz: 90, motion_profile: descend}"
ros2 topic hz /cobot/state                         # the 10 Hz poll
ros2 topic echo /cobot/state --once                # named sensors, pose, alarms, poll_ms
ros2 service call /cobot/set_output_bit cobot_interfaces/srv/SetOutputBit "{name: gripper, value: true}"
ros2 service call /cobot/pause std_srvs/srv/Trigger   # then /cobot/resume
ros2 topic echo /task/status
ros2 topic echo /health/summary --once             # per-node health
ros2 topic echo /task/workflows --once             # registry + readiness
ros2 service call /task/start_workflow cobot_interfaces/srv/StartWorkflow "{workflow: pick_layer}"
```

Tests (the C++ ones also run automatically during the image build):

```bash
# C++: detector decode/NMS/letterbox/registry, control profile/setup/tooling parsers
colcon test --merge-install --packages-select cobot_detector cobot_control --event-handlers console_direct+
colcon test-result --verbose

# Python: plain pytest, no rclpy — but the built cobot_interfaces overlay must
# be sourced (the tests build real messages)
python3 -m pytest src/cobot_operation/test src/cobot_tasks/test src/cobot_vision/test
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
.venv\Scripts\python tools\train_yolo_obb.py          # datasets/cans_obb   -> weights/cans_obb_best.pt
.venv\Scripts\python tools\train_green_light.py       # datasets/green_light -> weights/green_light_best.pt
.venv\Scripts\python tools\export_to_onnx.py cans_obb_best   # weights/*.pt -> models/<name>/<name>.onnx + manifest
.venv\Scripts\python tools\export_yoloe.py --prompts "can" "green light"   # open-vocab YOLOE -> models/yoloe_cobot_v1/
```

`runtime_models/` (repo root, mounted at `/ws/runtime_models`) holds the ONNX
exports + `models.yaml` (the registry `detector_node` reads); `.pt` weights
live in `vision_workspace/weights/` (gitignored) and never enter the container.

Collect more training frames with the UI's **Save frames** button
(`/camera/save_frames`) — they land in `vision_workspace/operation/captures`.

---

## 12. Known gaps and things to watch

- **Single-layer pick cycle.** `pick_layer` scans once and picks every
  target. `/vision/verify_target` now runs after each place ("second depth
  scan") and its verdict rides on `/task/pick_event`, but it does not yet gate
  the layer or trigger a rescan, and `/vision/get_layer_height` (rescan / next
  layer / bin complete decisions) is still uncalled. The tier-1 sensing
  design ([docs/ros2_architecture.md](docs/ros2_architecture.md) §5) is ready
  for it.
- **Pick height is constant per tool.** The choreography commands
  `tools.<name>.pick_tip_z_mm` / `approach_tip_z_mm` from `cobot_setup.yaml`
  and ignores `Target.real_z`; measured depth is not yet wired into the pick
  height. `compression_mm` is declared but unused.
- **Camera intrinsics are provisional.** The streamer's colour `fx/fy`
  disagree by 33 %; a rigid fit against them came out at ~95 mm RMS, so the
  shipped config uses the full monolithic fit (`require_intrinsics: false`).
  `camera_intrinsics.json` documents how to recover true intrinsics from a
  good full fit and switch back to the fast extrinsic-only boot fit.
- **Manual marker clicking is the default** because the red-dot model is not
  yet trusted. The automatic path (`/vision/detect` with the dot model) is
  implemented — run the `auto_calibration` / `auto_startup_calibration`
  workflows, which pin the mode per run through `CalibrationBegin` without
  touching the legacy `manual_dot_selection` flag.
- **`docs/ros2_architecture.md` still says "design draft — nothing implemented
  yet"** in its header; the document is otherwise accurate and is the best
  explanation of the reasoning behind each node.
- `.gitmodules` refers to paths from the legacy layout (`cpp/third_party/pybind`,
  `python/cobot_application/...`) that no longer exist; the ROS build uses
  Humble's `pybind11_vendor` instead and does not need the submodules.
- Depth must be **registered to colour** in the streamer: localization samples
  depth at colour pixels and back-projects with colour intrinsics.
