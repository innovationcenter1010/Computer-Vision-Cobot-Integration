# ROS2 Architecture â Computer Vision Cobot Integration

Design document for migrating the `cobot_application` asyncio monolith to a ROS2 (Humble)
node graph. Covers the target architecture, message contracts, the bin-picking operating
cycle, configuration strategy, and the Docker build.

Status: **design draft â nothing implemented yet.**

---

## 1. Current system (migration baseline)

The legacy application (`legacy/main.py`, `legacy/shared_data.py`) runs four controllers as
asyncio tasks in one process, communicating through `Py_network` â one `asyncio.Queue` per
module, carrying `P2P_command(sender, receiver, message_enum, value)` envelopes.

| Module | Role | Key implementation |
|---|---|---|
| camera | ZMQ SUB bridge to external color/depth streamer (5550/5551), latest-frame cache | `camera/camera_streamer.py` |
| vision | YOLO-OBB detection + homography pxâmm + rz + pick ordering | `vision_system/run_operation_scan.py` |
| cobot | Motoman control via **pybind** `motoman_cpp` (YMConnect), pick-and-place choreography | `cobot/cobot.py` |
| web | FastAPI + websocket UI, command injection, config editor | `web_app/webpage_controller.py` |

Runtime choreography:

```
Browser ââws {receiver,command,payload}âââ¶ web
   web ââsnapshotâââ¶ camera ââupdate_image(DualFrame)âââ¶ web ââbase64 wsâââ¶ Browser
   web ââdetectâââââ¶ vision ââpick_layer(list[OBBDetection])âââ¶ cobot âââ¶ motoman_cpp (pybind)
camera ZMQ SUB âââ external color/depth streamer
```

Three payload shapes cross the queues: scalars (UI), `DualFrame` (two numpy images),
`list[OBBDetection]` (pose-bearing detections).

---

## 2. Target node graph

Six nodes plus a bringup package. Design decisions locked in:

- **The cobot is two nodes** â `cobot_control` (C++, the only YMConnect link: hardware
  primitives, 10 Hz state poll, pause / cancel) and `cobot_operation` (Python choreography).
  The pybind extension `motoman_cpp` is retired (Â§7).
- **Vision splits into two nodes** â ML (`detector_node`, C++ on ONNX Runtime) and geometry
  (`localization_node`, Python) are separate packages; `localization_node` owns the single
  `/vision/scan` action and calls `/vision/detect` for pixels.
- **A task manager owns the state machine** (Â§11) â the cobot nodes are action servers;
  nothing moves because a topic was published. `/vision/targets` is data, not a trigger.
- **All frames flow as topics** â vision *and* the web UI subscribe to the same camera streams.

```
              ZMQ 5550/5551
 external âââ¶ camera_node ââ¶ /camera/color/image_raw âââ¬ââ¶ detector_node (C++/ONNX)
                          ââ¶ /camera/depth/image_raw âââ¼âââââââââââââââââ¶ localization_node
                                                       â                    â²            â
                                                       âââ¶ web_node         â            â
                                                                            â            â
 browser ââwsâââ¶ web_node                                                   â            â
                    ââââââââââââââ /vision/detections_px (pixel space) ââââââ            â
                    âââââ¬âââââââââ /vision/targets (robot space) ââââââââââââââââââââââ¬âââ
                    â   â                                                             â
                    âââ /task/start|stop|calibrate (Trigger) âââ¶ task_manager âââââââââ
                    ââââââââââ /task/status + /task/layer_report âââ¤ â
                    â                              /vision/scan ââââ â (+ calibration srvs)
                    â                                                â¼
                    â              PickCan action âââ¶ cobot_operation ââ¶ cobot_control ââ¶ robot
                    â              MoveToPose action ââââââââââââââââââââ¶ (YMConnect)
                    ââââââââââââââ /cobot/status (latched) + /cobot/state (10 Hz) ââ
```

### Workspace layout

```
ros2_ws/src/
âââ cobot_interfaces/      # msgs/srvs/actions only â depends on nothing local
âââ cobot_camera/          # ament_python: camera_streamer.py + camera_node
âââ cobot_detector/        # ament_cmake: C++ detector_node on ONNX Runtime (/vision/detect action)
âââ cobot_vision/          # ament_python: localization_node (/vision/scan action + calibration math)
âââ cobot_control/         # ament_cmake: C++ node on YMConnect, SDK vendored in third_party/ (Â§7)
âââ cobot_operation/       # ament_python: pick choreography (PickCan) over cobot_control primitives
âââ motoman_cpp/           # RETIRED pybind11 module (COLCON_IGNORE) + old Python cobot node
âââ cobot_tasks/           # ament_python: task_manager â state machine + sequencing (Â§11)
âââ cobot_web/             # ament_python: FastAPI app + web_node
âââ cobot_bringup/         # launch files + config YAMLs
```

Dependency rule: every package depends only on `cobot_interfaces` + standard messages
(`sensor_msgs`, `std_srvs`). `cobot_control` is C++ and links the vendored YMConnect SDK.
`cobot_tasks` depends on `cobot_interfaces` + rclpy ONLY (Â§11). Nothing depends on
`cobot_bringup`.

---

## 3. Node specifications

### 3.1 `camera_node` (pkg `cobot_camera`)

`CameraStream` (ZMQ SUB threads) moves in unchanged; its `on_color`/`on_depth` callbacks
become publisher calls.

- Publishes `/camera/color/image_raw` â `sensor_msgs/Image`, `rgb8`
- Publishes `/camera/depth/image_raw` â `sensor_msgs/Image`, `mono16` (depth is already int16 mm)
- Publishes `/camera/color/camera_info` â `sensor_msgs/CameraInfo`, latched. Relays the
  camera-SDK **intrinsics** the streamer writes to a small JSON (`intrinsics_file`, in the
  bind-mounted vision_workspace). Static data â published once and re-read only on file
  change, not per frame. localization consumes it for the extrinsic startup calibration (Â§10)
- QoS: images best-effort keep-last(1) (freshest wins); CameraInfo reliable transient-local
- ZMQ endpoints become **parameters** (they are currently hardcoded in `camera_streamer.py`;
  the `camera` section of `config.JSON` was never actually read)
- The legacy `snapshot` / `get_stream_provider` commands disappear â consumers subscribe.
  `frame_saver` can return later as an optional `/camera/save_frame` Trigger service.

### 3.2 `detector_node` (pkg `cobot_detector`, C++) â pure perception, no geometry

- Runs YOLO models exported to ONNX through **ONNX Runtime** (CPU provider; vendored `.so`
  installed beside the executable like YMConnect). No ultralytics / torch in the image.
- Subscribes `/camera/color/image_raw`, caches the latest frame (owned copy)
- Action `/vision/detect` (`Detect`) â goal `model_id`, a leaf key of the bind-mounted
  `models.yaml` (`runtime:` cans-style models, `calibration:` the red-dot model). Runs on a
  worker thread; feedback `loading_model | preprocess | inference | postprocess`; result
  carries the `ObbDetectionArray` and `inference_ms`. Goal `image_source` (optional
  container path) runs the model on that file instead of the cached frame â no camera
  needed; the result is then stamped `now()`. Rejected when the id is unknown, no frame has
  arrived (and no `image_source`), the file does not exist, or a detection is already running.
- Also publishes `/vision/detections_px` (`ObbDetectionArray`) â class, confidence, pixel
  corners, pixel center, angle. **Header stamp = the color frame the model ran on.**
- Both head types give the same message: `obb` â rotated corners + angle, `bbox` â axis-aligned
  rectangle, `angle = 0` (the calibration marker does not need one).
- Letterbox, decode, rotated NMS and the pixel-distance duplicate filter are hand-written
  (`postprocess.cpp`) and unit-tested; a golden test compares against the retired
  ultralytics node's output. Thresholds are per model in `models.yaml`.
- Models lazy-load on first goal unless listed in `preload`. `train_yolo_obb.py` /
  `export_to_onnx.py` are **offline tools** in `vision_workspace/tools/`, not nodes.

### 3.3 `localization_node` (pkg `cobot_vision`) â pure geometry, no ML

- Owns the **`/vision/scan` action** (`Scan`): goal `model_id` (empty â `default_model_id`
  param), `image_source` (passed through to `/vision/detect`; "" = live frame) and
  `localize` (default true). Execute = call `/vision/detect` â pick the depth frame nearest
  the detections' stamp â transform â order â result `TargetArray` (+ `detection_count`),
  also mirrored on `/vision/targets`. Feedback `detecting | localizing`. Rejected when
  uncalibrated or no depth frames (only while `localize` is true), the detector is down, or
  a scan is already running. **`localize: false`** stops after the detect: `detection_count`
  is filled, `targets` stays empty, nothing is published on `/vision/targets`, and no
  calibration / depth / camera is needed â the hardware-free path `vision_test` uses.
  `image_source` + `localize: true` localizes against the nearest *live* depth frame
  (best effort, bench use only).
- Subscribes `/camera/depth/image_raw` (small ring buffer of recent depth frames)
- **Requires a fitted cameraârobot calibration** (`calibration.yaml`, see Â§10). Uncalibrated
  â detections are dropped with a loud log and `verify_target` errors. There is deliberately
  no homography fallback: an unmeasured transform must not move the robot.
- Per detection:
  - depth at the can center + fitted transform â real x/y/z (`calibration.py`) â one
    consistent 3D model replaces the legacy planar homography AND the hardcoded
    `LAYER_Z1..Z7` table
  - `real_rz` from OBB corners (`rotation.py`), corners mapped through the same transform
  - pick ordering (`pathing.py`) â a **robot-frame snake**: rows grouped along one robot
    axis (`pathing_row_axis`), serpentine along the other. No bin-corner coordinates, so
    the order survives the bin shifting
- Publishes `/vision/targets` (`TargetArray`) â consumed by `task_manager` (data for the
  pick cycle it requested) and `web_node` (display)
- Services (cheap depth reads, no ML â see Â§5):
  - `/vision/verify_target` â was this target taken? Re-samples the live depth at the
    pixel the target was detected at (`Target.pixel_x/y`) and compares with the depth
    seen there at scan time (`Target.depth_mm`); a taken can reads ~`can_height_mm`
    deeper. `success=false` (no / stale depth, no valid patch) is inconclusive, never
    "gone". Called by `task_manager` **after** the place â the arm covers the camera
    mid-pick
  - `/vision/get_layer_height` â median live depth **at the last scan's detection pixels**
    â current layer index. Sampling where the cans were seen (not a fixed image ROI) keeps
    the height check valid when the bin moves
- Calibration math services (`task_manager` drives the robot between calls â see Â§10):
  - `/vision/calibration_begin` Â· `/vision/calibration_sample` Â· `/vision/calibration_compute`

### 3.4a `cobot_control` (pkg `cobot_control`) â hardware primitives, C++, YMConnect

The only process on the YMConnect link. No decisions, no choreography, no task state.
The SDK wrapper (`motoman_robot.cpp`) has no ROS in it and serialises every SDK call on
one mutex; the node owns the timers and the move thread.

- Action `/cobot/move_to_pose` (`MoveToPose`) â one supervised move. Named motion
  profile from `cobot_setup.yaml` (joint / **linear**, speed, accel/decel, accuracy) +
  per-goal overrides; the pose is expressed in `goal.tool_number` (â1 = the **active
  tool**, Â§3.4c); IK reachability pre-check; feedback `progress_percent` /
  `steps_remaining` / `paused`. **Cancel is immediate** â `MotionStop(true)`, trajectory
  cleared, goal CANCELED.
- `/cobot/set_active_tool`, `/cobot/convert_pose`, `/cobot/reload_profile` and the latched
  `/cobot/profile` â the tooling layer, Â§3.4c.
- `/cobot/pause`, `/cobot/resume` (Trigger) â hold the arm in place (`MotionStop(false)`,
  resumed with `MotionStart()`, the SDK's documented pause) or, with no move in flight,
  make the next move wait. `pause_uses_hold` switches to the controller HOLD signal.
- `/cobot/motion_stop` â immediate controlled stop (not an e-stop).
- Services for every other primitive: `set_output_bit` / `read_io_bit` (by **name** from
  `cobot_setup.yaml` or raw address), `set_motion_profile`, `get_current_pose`,
  `get_active_alarms`, `clear_faults`, `clear_trajectory`, `servo_on/off`, `reconnect`,
  int / position variables. Every response: `success`, `status_code` (0 ok; 1 not
  connected; else the YMConnect code), `message`.
- **The 10 Hz poll** (`poll_period_s`) publishes `/cobot/state` (`CobotState`): `ReadState`
  (servo / hold / alarm / erroring / Teach-Play-Remote), one `ReadByte` per distinct IO group
  so every named sensor and output costs one round-trip, the current TCP pose (in the
  active tool file, `pose_tool_number`) plus `joints_deg`, the active tool
  (`active_tool_number` / `active_tool_name`, `profile_loaded`), active alarms while
  alarming, `poll_ms`. The move loop consumes this instead of polling the controller
  itself. The same timer is the reconnect loop, and a successful connect kicks off the
  one-shot controller-profile read.
- One motion at a time: a goal arriving while another executes is REJECTED, as is any goal
  while alarming, servos off or in Teach mode.
- Publishes `/cobot/status` (`CobotStatus`, latched) on transitions â hardware status only.
- Threading: `MultiThreadedExecutor`; poll timer (exclusive group), services + action
  (reentrant group), one `std::thread` per executing move (joined before the next).

### 3.4b `cobot_operation` (pkg `cobot_operation`) â pick choreography, Python

`rclpy` + `cobot_interfaces` only â never imports robot code. Owns what the legacy
`cobot.py` owned above the hardware line: the phase order, the per-tool cell values
(`cobot_setup.yaml` `tools`), the remembered positions (`runtime_config/trajectories.json`,
Â§3.4c), `last_placed`, the report. The ACTIVE tool is read from `/cobot/state`, never a
parameter here.

- Action `/cobot/pick_can` (`PickCan`) â pick + place ONE can by chaining `MoveToPose`,
  `set_output_bit` and `/cobot/state` sensor reads; feedback = phase; the result carries the
  full report (`jobs[]`, `sensor_asserts[]`, `phase`). **Cancel is immediate**: the in-flight
  `MoveToPose` is canceled (controller stop) and the choreography returns at its next
  boundary â the can may still be gripped.
- Pause: `cobot_control`'s `/cobot/pause` holds the arm; the choreography sees
  `CobotState.paused` and waits before its next primitive. One service pauses both layers.
- Execute callback is a coroutine awaiting the client futures (one reentrant group on a
  `MultiThreadedExecutor`) â no thread is parked, so goal / cancel stay serviceable.
- The choreography runs against a `Primitives` protocol; `test/test_choreography.py`
  drives it with a fake (phase order, abort points, place selection, gating).
- Every pose it sends carries an explicit tool file number: teach points keep the one
  they were stored with, vision-derived pick/approach poses get the active tool resolved
  at goal accept. A `PickCan` goal is REJECTED while the stored positions are not in the
  active tool's frame (see the conversion in Â§3.4c).
- `/cobot/convert_trajectories` (`ConvertTrajectories`) re-expresses every remembered
  position in a tool file through `/cobot/convert_pose` and persists â all-or-nothing;
  runs automatically when the active tool changes (`auto_convert_on_tool_change`).
  `/cobot/save_teach_point` overwrites one remembered position with the current pose +
  joints from `/cobot/state`.

### 3.4c Tooling â controller profile, active tool, remembered positions

The YRC1000 already holds properly measured **tool files** (TCP offset, weight, COG). The
stack uses them instead of duplicating TCP math:

- **Controller profile, read ONCE per connect, strictly read-only.** After every successful
  connect `cobot_control` spawns a one-shot thread that calls `ReadSystemInformation`,
  `ListFiles(CND)` and `SaveFromControllerToString("TOOL.CND")` / `("UFRAME.CND")` (the
  SDK has no tool-data API; the files are the only source), parses them
  (`controller_profile.cpp`, pure C++, gtests on fixtures in `test/data/`) and latches
  `ControllerProfile` on `/cobot/profile`: model, software version, `ToolFile[]`,
  `UserFrame[]` (display / groundwork only), per-section `errors[]` (e.g. RS7 â  2 â
  file reads refused outside REMOTE mode), the configured tool names and the active tool.
  Raw files are dumped to `runtime_config/controller_profile/`. Nothing is ever loaded
  back to the controller (no `LoadToController*` is wrapped). `profile_source: fixture`
  runs the same path on `profile_fixture_dir` without a robot.
- **Active tool** = the tool file every pose is expressed in. ONE node-level value in
  `cobot_control` (`none` = tool 0, the boot default): the polled pose, `get_current_pose`,
  `MoveToPose` goals with `tool_number â1`, the IK check, `write_position_var` and
  `convert_pose` defaults all derive from it. Selected on the dashboard via
  `/cobot/set_active_tool` (name from `cobot_setup.yaml tools` or a number; validated
  against `TOOL.CND`; refused mid-move), persisted in `runtime_config/tooling_state.yaml`
  and restored at boot â a crash/respawn never flips the TCP back to tool 0. Motion
  profiles no longer carry a `tool_number`.
- **Per-tool cell values** live in `cobot_setup.yaml tools.<name>`: `tool_number`,
  `marker_offset_mm` (red dot relative to THAT tool's TCP â single source, the old
  `localization_node.marker_offset_mm` param is gone), `pick_tip_z_mm`,
  `approach_tip_z_mm`, `descend_profile`, `compression_mm`. `tip_offset_mm` is allowed
  ONLY on `none`: with a real tool file the controller's TCP already is the tip, so a
  non-zero offset there is a load error (no duplicated offset logic).
- **Remembered positions carry their coordinate system.** `runtime_config/trajectories.json`
  (v2; the packaged `cobot_bringup/config/trajectories.json` is only the first-boot seed, a
  v1 file is migrated in place with a `.v1.bak`) stores per pose `x y z rx ry rz`,
  `frame: base`, `tool_number`, canonical `joints_deg` (null until the controller has
  converted the pose) and provenance. When the active tool changes, `cobot_operation`
  asks `/cobot/convert_pose` per point â `cart(tool A)` â IK (Delta, seeded with the current
  joints; skipped when `joints_deg` is stored) â FK with `toolNumber = tool B` on the
  controller â and rewrites the file only if every point converted.
- **Calibration stays one base-frame fit.** The tool enters only through each sample's ground
  truth (`commanded TCP in the active tool + that tool's marker_offset_mm`);
  `calibration_begin` snapshots the active tool, `calibration_sample` refuses if it changes
  mid-run, and `calibration.yaml` records `tool_number` / `tool_name` /
  `marker_offset_mm`. Active tool â  calibration tool is a dashboard **warning**
  (`SystemState.calibration_tool_mismatch`), not a blocker.
- Dashboard: the *Tooling* card shows the controller line, errors, the tool-file table
  (named slots only — blank TOOL.CND entries hidden; active row highlighted,
  configured cell profiles tagged), named user frames (collapsed), the
  active-tool selector (`none` + every configured tool that exists on the controller),
  the remembered-positions pill (from `cobot_operation` heartbeat metrics) with a manual
  *Convert saved positions* retry, the calibration-tool badge and the live TCP/joints.


### 3.5 `task_manager` (pkg `cobot_tasks`) â state machine + sequencing

See Â§11. Owns start/stop/calibrate, the pick-layer and calibration loops, and
`/task/status` â the single source of operational truth.

### 3.6 `web_node` (pkg `cobot_web`)

FastAPI app, templates, static files, `/api/config`, `/ws` â all preserved. The node is the
browserâgraph bridge:

- The legacy `COMMAND_TRANSLATOR` stringâenum table becomes a stringâ(service client) table
  of the same shape â operations route to `/task/*` (the task manager), display-only
  commands (`/vision/scan`, `/camera/save_frames`) go direct
- Subscribes both image topics; throttles to ~5â10 Hz, JPEG-encodes, base64 over the existing
  websocket (the old `update_image` handler, fed by subscription instead of queue)
- Subscribes `/vision/detections_px`, `/vision/targets`, `/task/status`,
  `/task/layer_report` and forwards to the browser â the UI finally gets operation feedback
  (legacy reports only went to stdout)
- Config editor becomes a parameter client (see Â§6)
- Integration note: uvicorn's asyncio loop and the rclpy executor coexist by spinning rclpy
  in a background thread

---

## 4. Interface package â `cobot_interfaces`

```
msg/ObbDetection.msg        # class_id, confidence, center px, corners px    (pixel space)
msg/ObbDetectionArray.msg   # header (stamp = source color frame) + image w/h + detections
msg/Target.msg              # real_x, real_y, real_z, real_rz + class_id, confidence (any object)
msg/TargetArray.msg         # header + model_id + targets
msg/CobotStatus.msg         # HARDWARE status: disconnected/idle/busy/error + detail
msg/TaskStatus.msg          # OPERATIONAL state: idle/scanning/picking/calibrating/
                            # stopping/error + detail + active_task + progress   (Â§11)
msg/LayerReport.msg         # successes, failures, overall_ok
srv/VerifyTarget.srv        # Target â success/message, bool present, measured/expected depth, delta_mm, measured_z
srv/GetLayerHeight.srv      # (samples last detection pixels) â float64 median_z, int8 layer_index
srv/CalibrationSample.srv   # commanded TCP (x, y, z) â success, sample_count  (Â§10)
action/PickCan.action       # goal Target â success/message; feedback phase (Â§11)
action/Detect.action        # goal model_id[, image_source] â ObbDetectionArray + inference_ms (detector_node)
action/Scan.action          # goal model_id[, image_source, localize] â TargetArray + detection_count (localization_node)
action/MoveToPose.action    # goal x,y,z,rx,ry,rz â success/message            (Â§11)
```

Everything else uses stock types: `sensor_msgs/Image`, `std_srvs/Trigger`. The pixel/robot
split in the messages mirrors the node split â **the cobot never sees pixels, YOLO never
sees millimeters**.

QoS contract: images best-effort keep-last(1); detections/targets/status/reports reliable.

---

## 5. Bin-picking operating cycle â tiered sensing

YOLO must not run per pick. The design uses three tiers of sensing cost; the depth stream
(already continuous, already free) supplies the missing middle tier:

| Tier | Sensor | Cost | Question |
|---|---|---|---|
| 0 | Gripper sensors (`existance_check`) | free | "Did I actually grab something?" |
| 1 | **Depth sampling** (localization_node srvs) | ~Âµs, no ML | "Is a can still at (x,y)? What height is the layer, sampled where the cans were seen?" |
| 2 | **Detector scan** (`/vision/scan` â `/vision/detect`) | expensive | "Where is everything and how is it rotated?" |

The task manager drives the cycle (pick execution = one `PickCan` goal per can):

```
  layer start âââ¶ call /vision/scan âââ¶ receive /vision/targets (N cans, ordered)
                        â
      âââââââââââââââââââ¼âââââââââââââââââââââââ
      â for each target in layer:              â
      â   1. pick  (gripper existance_check)   â
      â        ââ grab failed? âââ¶ RESCAN      â
      â   2. place (bin is UNOCCLUDED now â    â
      â      free window for depth checks)     â
      â   3. /vision/verify_target (depth)  ââââ¼âââ¶ depth at the target's scan pixel
      â        ââ still there? âââ¶ RESCAN      â    now ~can_height deeper? (post-place
      â          (planned; reported only today)â    ONLY â the arm hides the camera mid-pick)
      âââââââââââââââââââ¬âââââââââââââââââââââââ
                        â¼
  targets exhausted âââ¶ /vision/get_layer_height (depth at last detection pixels)
                        ââ dropped ~can_height? âââ¶ scan next layer
                        ââ leftovers at old height? âââ¶ rescan same layer
                        ââ at bin floor? âââ¶ BIN_COMPLETE
```

Properties:

- **YOLO runs once per layer in the happy path.** Rescans are event-driven (verification
  mismatch, failed grab), not scheduled.
- **Every successful pick is verified against live depth after the place** â the task
  manager asks `verify_target` whether the surface at the target's scan-time pixel is now
  ~one can deeper. The check is exact (same pixel, same depth reading as the scan, no
  calibration round-trip) and cannot run earlier because the arm occludes the camera
  during the pick. A depth stream that is missing or stale answers `success=false`
  (inconclusive), never "gone".
- **Layer completion is measured, not inferred** â the depth plane dropping by
  `can_height_mm` replaces the legacy 70 %-count threshold heuristic (`bin.py`).
- **Rescan latency hides inside the place motion** â the controller requests scans while the
  arm is away from the bin, which also solves occlusion: vision never needs to detect
  whether an elbow is in frame.
- Known wiring gap to close during implementation: `cobot.py` currently hardcodes pick z at
  300/401 mm and ignores `real_z` â measured depth should feed the pick height.

---

## 6. Configuration strategy

Legacy: one `Config` object loaded from `config.JSON`, passed by reference everywhere; the
web app edits it at runtime (write file + in-memory `update()`). Live updates only reached
modules that re-sliced per operation â vision picked them up, the cobot silently kept stale
trajectories.

`config.JSON` holds three kinds of data, mapped differently:

| Kind | Examples | ROS2 home |
|---|---|---|
| Tunable scalars | YOLO conf/iou/max_det, `can_height_mm`, `layer_threshold`, row tolerance, `robot_ip`, tool-head offsets | **Declared ROS parameters** per owning node |
| Structured data | `cobot_place_trajectories` (named waypoint lists) | remembered positions are a **JSON file** in the runtime-writable `runtime_config/` mount (v2: each pose carries `frame` + `tool_number` + `joints_deg`), seeded from `cobot_bringup/config/trajectories.json`; bin-corner matrices retired â pick order is a robot-frame snake, layer height samples the detection pixels |
| Runtime-writable state | active tool, controller profile dumps | `runtime_config/tooling_state.yaml`, `runtime_config/controller_profile/` (bind-mounted at `/ws/runtime_config`, gitignored, never baked) |
| Offline / dead | `TRAIN_MODEL`, `TRAINING_*` (offline tool); `camera` section (never read â endpoints were hardcoded) | Not ROS params. Training config goes with the offline tool; camera endpoints become real params for the first time |

One YAML in `cobot_bringup` replaces the global object:

```yaml
# cobot_bringup/config/cobot_params.yaml
camera_node:
  ros__parameters:
    color_endpoint: "tcp://host.docker.internal:5550"
    depth_endpoint: "tcp://host.docker.internal:5551"

detector_node:
  ros__parameters:
    models_yaml: "/ws/runtime_models/models.yaml"   # ids, paths, thresholds
    preload: ["cans"]
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
    marker_offset_mm: [0.0, 0.0, 0.0]   # red dot off the toolhead point (Â§10)

cobot_control:
  ros__parameters:
    robot_ip: "192.168.1.31"
    tool_offset_mm: [0.0, 0.0, 0.0]     # tool TIP off the toolhead point (Â§10)
    tool_head_compression_distance_mm: 50.0
    trajectories_file: "config/trajectories.json"

task_manager:
  ros__parameters:
    scan_timeout_sec: 30.0
    # calibration pose grid (Â§10/Â§11) â zeros = unconfigured, calibrate refuses
    calibration_x_range_mm: [0.0, 0.0]
    calibration_y_range_mm: [0.0, 0.0]
    calibration_z_levels_mm: [0.0]
```

### Web config editor

`web_node` becomes a **parameter client** instead of a shared-object mutator:

- `GET /api/config` â aggregate each node's built-in `list_parameters`/`get_parameters`
  services (free on every rclpy node)
- `POST /api/config` â diff, then `set_parameters` on the owning node. Each node registers
  an on-set-parameter callback that **applies live or explicitly rejects** (e.g. YOLO
  thresholds: live; `robot_ip`: reject/restart) â making update semantics explicit fixes
  the legacy stale-reference problem
- Persistence: web_node writes accepted changes back to `cobot_params.yaml` on the
  bind-mounted config volume

```
LEGACY:  browser â POST â write config.JSON â Config.update() â (maybe) picked up
ROS2:    browser â POST â web_node â set_parameters on owning node â callback applies/rejects
                                   ââ write cobot_params.yaml (persistence)
```

Bonus: `ros2 param set /localization_node pathing_row_tolerance_mm 60.0` works from any terminal,
typed and logged. Detector thresholds live per model in `models.yaml` (restart `detector_node`).

---

## 7. The YMConnect SDK inside colcon

`cobot_control` is an `ament_cmake` package that vendors the SDK
(`third_party/YMConnect/{YMConnect.h, libYMConnect.so}`, moved with history from
`motoman_cpp`) as an IMPORTED shared library. `libYMConnect.so` is installed next to
`cobot_control_node` (`lib/cobot_control/`) and resolved through an `$ORIGIN` RPATH â the
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

Ordering rule, most stable â least stable. **ultralytics / torch are no longer in the image
at all** â training and the `.pt â .onnx` export happen on the dev PC. The detector links the
official ONNX Runtime release tarball, pinned by `ARG ORT_VERSION`:

1. apt packages â never change
2. **onnxruntime tarball â one pinned download, own layer, always cached**
3. remaining pip deps â rarely change
4. `package.xml` manifests â `rosdep install` â change when a package's deps change
5. workspace source â `colcon build` + `colcon test cobot_detector` â changes constantly

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

# ---- 2. ONNX Runtime: pinned release tarball -> /opt/onnxruntime ----
ARG ORT_VERSION=1.19.2
RUN curl -fsSL .../onnxruntime-linux-x64-${ORT_VERSION}.tgz | tar -xz -C /opt/onnxruntime --strip-components=1

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
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DONNXRUNTIME_ROOT=/opt/onnxruntime
RUN . install/setup.sh && colcon test --packages-select cobot_detector && colcon test-result --verbose

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

# ---- 3. remaining pip deps ----
RUN pip3 install --no-cache-dir \
    opencv-python-headless fastapi "uvicorn[standard]" pyzmq jinja2

# ---- install tree only: no compilers, no source ----
COPY --from=builder /ws/install /ws/install
COPY docker/entrypoint.sh /entrypoint.sh   # sources /opt/ros + /ws/install, exec "$@"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONUNBUFFERED=1

EXPOSE 8000
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "cobot_bringup", "robot.launch.py"]
```

Notes:

- ROS Humble pins Python **3.10** (legacy image was 3.11) â all current deps are compatible.
- Only the `install/` tree crosses to runtime: launch files, message definitions, and the
  compiled `.so` in one artifact. Same lean-image outcome as the legacy two-stage build.
- **Do not bake model weights.** The legacy image baked `cans_obb_best.pt`, but the
  `vision_workspace` bind mount shadowed it at runtime anyway â the volume is the source of
  truth; `model_path` is a parameter.

### 8.3 Compose

One service running the bringup launch file â a drop-in evolution of the legacy compose:

```yaml
services:
  cobot-ros:
    build: { context: ., dockerfile: docker/Dockerfile }
    image: cobot-ros
    pull_policy: never
    restart: unless-stopped
    command: ["ros2", "launch", "cobot_bringup", "robot.launch.py"]
    ports: ["8000:8000"]                      # web only â 5550/5551 were never listeners
    volumes:
      - ./vision_workspace:/ws/vision_workspace
      - ./ros2_ws/src/cobot_bringup/config:/ws/config   # params editable without rebuild
    environment:
      RMW_IMPLEMENTATION: rmw_cyclonedds_cpp
      ROS_DOMAIN_ID: "0"
```

- **Single container, not per-node services.** Deployment host is Docker Desktop for Windows
  (`host.docker.internal`), where `network_mode: host` â the standard DDS-in-containers fix â
  is unavailable. Per-node crash isolation comes from `respawn=True` per node in the launch
  file instead. Split into multiple services only if nodes move to different machines
  (containerâcontainer DDS on a shared bridge network does work if ever needed).
- The ZMQ camera source stays an external host process; `camera_node` bridges it into the
  graph via `host.docker.internal`.
- Legacy `privileged: true`, `/dev/bus/usb`, and the AstraSDK runtime libs were leftovers
  from the retired `astra_cpp` USB-camera path â all droppable (camera is ZMQ, Motoman is TCP).

### 8.4 Dev loop

Legacy `pip install -e` â `colcon build --symlink-install` inside the container with
`ros2_ws/src` bind-mounted (devcontainer on the same image). Python edits are live without
an image rebuild.

---

## 9. Migration map

### Code that moves (mostly unchanged)

| Legacy | Destination |
|---|---|
| `camera/camera_streamer.py` | `cobot_camera` (endpoints become params) |
| `camera/frame_saver.py` | `cobot_camera` (optional save service, later) |
| `vision_system/yolo_obb_detector.py` | `cobot_detector` (C++, `postprocess.cpp` + `onnx_detector.cpp`) |
| `vision_system/rotation.py`, `pathing.py` | `cobot_vision` / localization_node |
| `vision_system/homography.py` | retired â replaced by the fitted 3D calibration (`calibration.py`, Â§10) |
| `vision_system/train_yolo_obb.py` | `vision_workspace/tools/train_yolo_obb.py` (offline tool, dev-PC .venv) |
| `cobot/cobot.py` choreography | `cobot_operation/choreography.py` (against a `Primitives` protocol) |
| `cpp/` `MotomanRobot.cpp` + YMConnect | `cobot_control` C++ node (`motoman_robot.cpp`, `third_party/YMConnect`) |
| `cpp/` pybind bindings | retired in `motoman_cpp/` (`COLCON_IGNORE`) |
| `web_app/` templates, static, FastAPI routes | `cobot_web` |

Required code changes found during analysis:

- `run_YOLO(image, â¦)` ignores its `image` argument and always reads the inbox folder â
  must actually pass the subscribed frame through.
- `cobot.py` hardcodes pick z (300/401 mm) â wire measured `real_z` in.
- the ROS graph no longer imports `motoman_cpp`; `cobot_operation` talks to `cobot_control` over actions/services/topics.

### Code that dies

| Legacy | Replaced by |
|---|---|
| `Py_network`, `P2P_command`, command enums, `COMMAND_REGISTRY` | the ROS graph |
| `main.py` | `robot.launch.py` |
| `web_app/web_server.py` | already-dead earlier draft of `webpage_controller.py` |
| camera `snapshot`/`get_stream_provider`, vision `convert_to_jpeg` | frame topics |
| `bin.py` layer heuristics (stale imports, `LAYER_Z` table) | measured depth (Â§5) + bin state in `task_manager` |
| `Config` shared object | per-node parameters + bringup YAML (Â§6) |

---

## 10. Cameraârobot calibration

Replaces the hand-entered 4-corner homography and the separate `depth_at_top_layer`
guess with **one fitted 3D transform**, measured by the robot itself: a red dot is
mounted near the tool head, the robot parks it at known positions, the camera reports
where it sees it, and localization solves for the mapping. Model: `red_dot_best.pt`.

### 10.1 The math (`cobot_vision/calibration.py` â pure, no ROS imports)

A pinhole camera composed with a rigid cameraârobot transform is **exactly linear in
the features `[uÂ·d, vÂ·d, d, 1]`** (back-projection is `X_cam = (uâcx)Â·d/fx`). One 3Ã4
matrix `M` maps pixel + depth straight to robot base mm:

```
[X, Y, Z]áµ = M Â· [uÂ·d,  vÂ·d,  d,  1]áµ        M = [ RÂ·B | t ]
```

`M` factors into the two physical parts: **intrinsics** `B` (a static 3Ã3 holding
`fx, fy, cx, cy`) and **extrinsics** `R, t` (the camera pose vs the cobot base â the
only part that drifts when the camera is bumped). Two fitters:

- **`fit()` â full monolithic.** Solves all 12 numbers at once, absorbing intrinsics; no
  CameraInfo needed. **Requires depth variation** (â¥30 mm span) or it is rejected â with
  one height the feature matrix is rank-3 and Z is unconstrained ("sample at multiple
  heights", enforced in math).
- **`fit_extrinsic()` â Kabsch, with known intrinsics.** Back-projects each sample to a
  camera-frame 3D point using the fixed intrinsics, then solves the rigid transform
  `{P_cam}â{P_robot}` in closed form (SVD). Only **6 unknowns**, so ~4 poses suffice and
  the depth-span rule **relaxes** (Z = d is measured, not fit). Returns a *pure rotation*,
  so it can't drift into unphysical scale/shear. This is the small startup fit.

Per-axis RMS residuals are the honest quality measure either way; a fit above
`calibration_max_rms_mm` is rejected and the old transform kept. The inverse (robot xyz â
expected pixel + depth) is closed-form; `/vision/verify_target` only falls back to it for
hand-built targets that carry no scan-time pixel.

### 10.1a Best of both worlds â static intrinsics, dynamic extrinsics

Intrinsics are bolted into the sensor; only the camera *pose* moves. So they are sourced
**once, statically** and the pose is re-fit **cheaply, often**:

- **Intrinsics (static):** the camera SDK already knows `fx, fy, cx, cy`. The external
  streamer writes them to `intrinsics_file`; `camera_node` relays them as latched
  `/camera/color/camera_info`; localization caches them and also persists them into
  `calibration.yaml` (`camera_intrinsics` block) so they survive a silent camera.
- **Extrinsics (dynamic):** every boot, `task_manager` auto-runs a small extrinsic grid
  (`fit_extrinsic`) to correct for camera drift â see Â§11. No intrinsics re-measurement.

`require_intrinsics` (default true) makes calibration the extrinsic-only fit and refuses
until intrinsics arrive; set it false to allow the full monolithic fit when the camera
can't report intrinsics.

> **Depth must be aligned to color.** localization samples depth at the *color* pixel of
> each detection/dot and back-projects with the *color* intrinsics. That is only valid
> when the streamer has **depthâcolor registration ON**; otherwise the depth reading is
> from the wrong physical ray. Enable alignment in the streamer and publish the color
> intrinsics.

### 10.2 Offsets, tool files and the commanded point

The controller commands one pose â the TCP of the **active tool file** (Â§3.4c). What hangs
off that point depends on the tool file:

| offset | where | owner | applied to |
|---|---|---|---|
| red calibration dot | `cobot_setup.yaml` `tools.<name>.marker_offset_mm` (per tool file) | localization (`tooling.py`, active tool from `/cobot/state`) | each calibration sample's ground truth: `commanded_TCP(active tool) + marker_offset_mm` |
| end of the tooling | `cobot_setup.yaml` `tools.none.tip_offset_mm` â **tool 0 only** | `cobot_operation/geometry.py` (`ToolOffset`) | vision-derived pick poses while `none` is active: commanded pose = can position â tip offset |

With a real tool file active the controller's TCP already *is* the tip: `tip_offset_mm`
must be zero there (load error otherwise) and pick / approach heights are commanded
directly â the offset logic lives in the controller's tool file, never twice.

The marker offset is applied by localization **before** fitting and is never absorbed
into the fit â the transform maps pixels to **robot base** coordinates and is
tool-independent. Because it is accounted for during the fit, `/vision/targets`
coordinates are true robot-base positions of the **can**. Both offsets are measured in mm
along robot base axes at the fixed pick/calibration orientation
(`calibration_orientation_deg`, default 180/0/0); the tip offset's XY part rotates with
the commanded `rz`, the marker offset is added unrotated (fixed orientation).

Scope: the tip offset touches vision-derived pick poses only. Remembered positions
(`trajectories.json`) are stored *with* their tool file and are re-expressed through
the controller kinematics when the active tool changes; the calibration grid is
commanded in the active tool (`MoveToPose.tool_number â1`) and the vision side adds
that tool's `marker_offset_mm` to the same commanded pose.

### 10.3 Ownership split â movements vs math

| | owner | does |
|---|---|---|
| `/task/calibrate` | `task_manager` | pose grid from params, sequences the run (holds the one-task lock), sends one `MoveToPose` goal per pose to `cobot_control`, reports progress on `/task/status` (`calibrating`), stop cancels the active move |
| `/cobot/move_to_pose` | `cobot_control` | executes each grid move (the only node touching the robot) |
| `/vision/calibration_begin` | `localization_node` | clears the sample buffer, snapshots the ACTIVE tool (refuses without a marker offset for it), warms the red-dot model |
| `/vision/calibration_sample` | `localization_node` | "marker parked at TCP (x,y,z), measure now": N `/vision/detect` (dot model) reads â median (u,v), median-window depth from its own buffer, add the active tool's marker offset (refuses if the tool changed mid-run), store the pair |
| `/vision/calibration_compute` | `localization_node` | mode-select (extrinsic if intrinsics known, else full) â validate RMS â write `calibration.yaml` â hot-swap the live transform |
| `/vision/detect` (`dot_model_id`) | `detector_node` | red-dot inference on the latest color frame (ONNX Runtime) â **skipped entirely in manual mode**, see Â§10.5 |
| `/camera/color/camera_info` | `camera_node` | relays the SDK intrinsics (from the streamer's JSON) that make the extrinsic fit possible |

Sequence: web **Calibrate** â `/task/calibrate` â `begin` â {`MoveToPose` â settle â
`sample`} Ã poses â `compute` â result in `/task/status` detail. Poses that fail
(unreachable, dot occluded, bad depth) are skipped; the fit is overdetermined.

The same sequence runs **automatically at startup** on a small grid
(`startup_calibration_*`) to re-fit extrinsics for camera drift â see Â§11. `begin`
refuses (no motion) until CameraInfo intrinsics have arrived, so a boot with no camera
never moves the arm for a fit that can't complete.

### 10.4 Persistence & the no-fallback rule

`calibration.yaml` is written into the bringup config dir, which docker-compose
bind-mounts â calibration survives container rebuilds. It is per-machine and
gitignored. On startup localization loads it if present; **if absent the node refuses
to localize** (detections dropped loudly, `verify_target` errors). No homography
fallback exists: an unmeasured transform must never move the robot.

Prerequisites before running a calibration (all parameters):

- `task_manager.calibration_{x,y}_range_mm`, `calibration_z_levels_mm` â safe, reachable,
  in-frame bounds; `[0,0]` means unconfigured and `/task/calibrate` refuses
- `cobot_setup.yaml` `tools.<name>.marker_offset_mm` for the tool file you calibrate with â
  measured with a ruler after mounting the dot, relative to THAT tool file's TCP (and
  `tools.none.tip_offset_mm` for the tooling when tool 0 is used, Â§10.2)
- a fresh `/cobot/state` (the active tool is read from it; `calibration_begin` refuses
  without one)
- `models.yaml` `calibration.red_dot` â `red_dot_best.onnx` in the runtime_models mount; `localization_node.dot_model_id` names it
- Camera intrinsics available: the streamer writes `intrinsics_file` (color intrinsics,
  depthâcolor alignment ON), relayed as `/camera/color/camera_info`. With
  `require_intrinsics:=true` (default) calibration is extrinsic-only and waits for this

---

### 10.5 TEMPORARY: manual centre select

`localization_node.manual_dot_selection: true` replaces the red-dot model with an
operator click, for use while the model is still being trained. **Only the source of
`(u, v)` changes** â depth sampling, `marker_offset_mm`, the fit, the RMS gate and
persistence are the same code either way, so a manual calibration is as valid as an
automatic one (as accurate as your clicking).

```
localization  --/vision/manual_dot_request (Bool, latched)-->  web_node --ws--> browser
browser --ws("VISION","click_center",{u,v})--> web_node --/vision/manual_dot_click--> localization
browser --ws("VISION","skip_center")-------->  web_node --/vision/manual_dot_skip (Empty)--> localization
```

- `calibration_begin` skips the `/vision/detect` availability check and the model warm-up
  (so it returns instantly instead of after the ~110 s lazy load).
- `calibration_sample` publishes `manual_dot_request=true`, clears any stale click, and
  blocks up to `manual_click_timeout_sec`. A stale click sent while the arm was still
  moving can never be consumed as the next pose's sample.
- On timeout the pose fails cleanly and is skipped, like any other bad sample.
- **Skip:** if the marker is out of frame at a pose, the operator clicks *"Marker not
  in frame â skip pose"* instead of the feed. That publishes `manual_dot_skip`, and
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

## 11. Task manager â state machine + sequencing (`cobot_tasks`)

Why it exists: in the first migration draft, `/vision/targets` being *published* was what
*triggered* picking (cobot_control subscribed and immediately ran the layer). A data topic
doubling as a command is fragile. The task manager makes "what should happen next" an
explicit, single-owner decision, and turns cobot_control into a dumb, reliable motion server.

**Ownership rule** â mirrors the calibration split (Â§10.3), one row per node:

| node | owns | never does |
|---|---|---|
| `task_manager` | state machine, task loops, gating, progress | geometry math, motoman/camera access |
| `cobot_control` | motion execution (actions) | decisions, vision calls, task state |
| `localization_node` | geometry + calibration math | robot commands |
| `detector_node` | ML inference (C++, ONNX Runtime) | geometry, robot commands |

Dependency rule enforced in review: `cobot_tasks` depends on `rclpy` + `cobot_interfaces`
ONLY. If a change makes it compute geometry or import robot/camera code, it has become a
god-node â reject it. (It handles only small structured data: targets, poses, states;
never a numpy array.)

**Workflows (modular tasks).** Every task the operator can start is a *workflow* module
under `cobot_tasks/cobot_tasks/workflows/`, registered in `workflows/__init__.py`
(`REGISTRY`, explicit list â order = dashboard order). A workflow subclasses
`workflows/base.py:Workflow`: it declares the ROS params it reads (`ParamSpec`, declared
once on the node under the same name so `cobot_params.yaml` keys are stable) and the
action/service clients it needs (`ClientSpec`; the node creates ONE client per key, shared
by all workflows), a `check_ready(ctx)` gate and a `run(ctx)` body. The body talks to ROS
only through `cobot_tasks/context.py:WorkflowContext` (`publish_status`, `run_action` with
native cancel-on-stop, `call`, `client_ready`, `stop_requested`, `get_param`, report/event
publishers) â **workflows never import rclpy**, which is what makes them unit-testable with
a fake context (`cobot_tasks/test/test_workflows.py`). Registered today: `pick_layer`,
`calibration`, `startup_calibration`, `vision_test` (hardware-free stack check: every image
under `vision_test_image_dir` goes through the real Scan action with `image_source` +
`localize: false`, one of `vision_test_model_ids` drawn at random per image, detections only
counted; `requires = CAP_NONE`, so it needs just the two vision nodes). Adding a task = one
new file + one registry line;
the node, web_node and dashboard need no change.

**Interface:**

- `/task/start_workflow` (`StartWorkflow{workflow}`) â the dashboard's path; `""` = the
  `default_workflow` param
- `/task/start` Â· `/task/calibrate` (`std_srvs/Trigger`) â aliases for `default_workflow` /
  `calibration`; `/task/stop` (`Trigger`) cancels the active goal
- `/task/system_state` (`SystemState`, latched) â the DERIVED capability: individual
  robot/vision facts (connected, servos, YRC1000 control mode, alarming, camera,
  calibrated, â¦) plus a bitmask `capability` â {none=0, see=1, move=2, operate=3=see|move}.
  Aggregated read-only from `/cobot/state` (10 Hz) + `/health/summary` (1 Hz) by
  `cobot_tasks/system_state.py` â the task manager never sets it. MOVE mirrors
  cobot_control's own goal gate (connected, no alarm, servos on, mode â  TEACH) minus
  `busy`; SEE = camera streaming + localization alive + calibration fitted. Each
  workflow declares `requires` (a capability bitmask) and is ready only when
  `(capability & requires) == requires` â a motion-only workflow needs just `move`,
  a vision-only one just `see`.
- `/task/workflows` (`WorkflowList`, latched) â registry + readiness (+ reason + each
  workflow's `requires`); republished when readiness changes and after each run;
  web_node forwards and replays it
- `/task/status` (`TaskStatus`, latched) â THE operational state; `active_task` = workflow id
- `/task/layer_report` (`LayerReport`), `/task/pick_event` (`PickEvent`)
- Clients come from the workflows' `ClientSpec`s: actions `/vision/scan` (`Scan`, result
  carries the ordered targets â `/vision/targets` is no longer consumed here),
  `/cobot/pick_can`, `/cobot/move_to_pose`; services `/vision/verify_target`,
  `/vision/calibration_*`
- Subscribes `/cobot/status` (hardware state, mirrored into `/task/status` when idle)

**State machine:**

```
IDLE ââstart_workflow(id)âââ¶ <workflow's states> ââdoneâââ¶ IDLE   (failure ââ¶ ERROR)
   pick_layer:   SCANNING ââtargetsâââ¶ PICKING ââlayer doneâââ¶ IDLE
   calibration*: CALIBRATING ââ compute ok ââ¶ IDLE   (fit rejected ââ¶ ERROR)
any workflow ââstopâââ¶ STOPPING ââ active goal canceled ââ¶ IDLE
```

One workflow at a time (`_operation_lock`, held for the whole run); anything but `stop` is
refused while one runs. Stop cancels the in-flight action goal **natively** â a pick
stops at the next safe phase boundary (never between grip and lift) instead of waiting
for the layer's next can, which is why the motion interface is actions, not services.

**Auto extrinsic calibration at startup (opt-in, default off).** Calibration is
operator-selected from the dashboard; nothing runs at boot unless
`auto_calibrate_on_startup` is explicitly set true. When it is, a
background thread waits until the `startup_calibration` workflow reports ready (up to
`startup_calibration_wait_sec`), then launches it through the same `_launch` path as the
dashboard. It runs the calibration loop on a small grid (`startup_calibration_grid_size` /
`startup_calibration_z_levels_mm`) to re-fit the camera *pose* for drift (Â§10.1a). It is
the `CALIBRATING` state at boot. Safety: it **skips without moving** if the grid is
unconfigured, the robot/services never come up, or (via `begin`) the camera intrinsics
haven't arrived; a failed fit keeps the previously saved transform. The full-grid
`calibration` workflow (`/task/calibrate`) remains for a deliberate, denser re-fit.

Bin-cycle state (layer index, `BIN_COMPLETE`, tier-1 rescan decisions â Â§5) belongs here
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
| `detector_node` | WARN color frame stale | `models_loaded`, `scan_in_flight`, `last_model_id`, `last_scan_ms`, `last_detection_count`, `scan_failures` |
| `localization_node` | ERROR not calibrated; WARN depth stale / no intrinsics | `calibrated`, `rms_*_mm`, `depth_buffer_age_s`, `manual_click_waiting` |
| `cobot_control` | ERROR not connected / alarming, WARN servos off | `connected`, `busy`, `paused`, `servo_on`, `alarming`, `poll_ms` |
| `cobot_operation` | ERROR primitives missing / robot disconnected | `primitives_ready`, `busy`, `paused`, `last_placed`, `active_tool` |
| `task_manager` | WARN robot unavailable / task error | `state`, `active_task`, `progress`, one `<client_key>_ready` per registered workflow client (`scan`, `pick`, `verify`, `move`, `cal_*`) |
| `web_node` | â | `ws_clients`, `stream_hz_actual` |

`cobot_control`'s 10 Hz poll mirrors a link change
into `/cobot/status` (DISCONNECTED / IDLE), which task_manager already
mirrors into `/task/status` â a mid-session link loss is no longer invisible.

**health_monitor node** subscribes `/health/heartbeat`, and publishes a
latched `HealthSummary` on `/health/summary` at 1 Hz: per expected node
(`expected_nodes` param) UNKNOWN if never seen, STALE if silent for
`stale_factor Ã period_s`, else the node's own level; unexpected publishers
are appended. Level transitions are logged once. `web_node` forwards the
summary to the browser as `{"type": "health"}` (age computed from the header
stamp because the topic is latched); the dashboard flags the monitor itself
as offline if no summary arrives for 3.5 s.

**Pick monitor.** `task_manager` publishes one `PickEvent` on
`/task/pick_event` per `PickCan` attempt with `outcome`
(picked / failed / skipped) and â the "second depth scan" â the result of
calling `/vision/verify_target` after a successful pick, i.e. after the place
(`verify_ran = service success`, `verified = success && !present`,
`verify_delta_mm`, `measured_z`, and `verify_message` when the service could
not measure). `Target.detection_index` (set by localization) links each event
back to the scan's `ObbDetectionArray` entry so the browser recolors the right
box on the color-feed overlay; the pick monitor card shows the depth delta
("Î +87 mm â"), "still present", or "verify inconclusive: â¦". The verdict is
reported only; it does not yet gate the layer report or trigger a rescan
(Â§5), and the UI's `USE_VERIFY_GATE` flag in `app.js` keeps the strict
coloring off until the check is trusted on the real cell.

**Cobot panel.** `web_node` digests `/cobot/state` into `{"type": "cobot_state"}`
at `web_node.cobot_state_hz` (default 5 Hz; replayed on connect): link, servo,
hold, alarm/error flags with the active `Alarm[]`, control mode, pause/busy
latches, the named sensors/outputs, the TCP pose, and every joint in degrees.
The joint bars are drawn against `web_node.joint_limits_deg` (display only â
set to the arm's soft limits); the card greys out when the sample is older
than 2 s.

**Automatic calibration twins.** Every manual-click calibration workflow has
an automatic twin (`auto_calibration`, `auto_startup_calibration`) that finds
the marker with the green-light detector (`auto_calibration_model_id`, default
`green_light_v2`; `auto_calibration_class_id` filters a multi-class model)
over a denser grid (`auto_*_grid_size` / `auto_*_z_levels_mm`). The
marker-finding mode is pinned per run by `CalibrationBegin.srv`
(`/vision/calibration_begin`: mode / model_id / class_id / samples_per_pose),
so the twins coexist without touching localization's `manual_dot_selection`
parameter. `task_manager.startup_calibration_workflow` and
`calibrate_alias_workflow` choose which twin the boot run and `/task/calibrate`
use. For verification, localization echoes every sample on
`/vision/calibration_marker` (`CalibrationMarker`: accepted / rejected, pixel,
depth, hits/reads, the detector's raw boxes); the dashboard draws accepted
markers as numbered crosshairs, the last rejection as a red X, and live
marker-model detections as dashed boxes on the color feed, and calibration
detects never reset the can overlay or pick monitor.

Dashboard theme + stream telemetry: `style.css` is a flat blue theme in two
variants â light (default: white surfaces, bold `#2361C4` accent text, 1px
outlines) and dark (`--primary-hi #4c8cf2` carries accent text). All type is a
system monospace stack (no webfonts â the cell may be offline). `index.html`
stamps `data-theme` on `<html>` before first paint (localStorage
`cobot.theme`, else `prefers-color-scheme`); the top-bar button toggles and
persists it, and canvases repaint because they sample the CSS tokens at draw
time. The streams are the hero: each frame carries a telemetry strip â
`live / stalled` and frame age measured in the browser, camera-side Hz /
drops from `camera_node`'s heartbeat as forwarded in the health summary, and
a 60 s Hz sparkline sampled once per summary (the video well stays dark in
both themes). Layout `split / color / depth` (segment or click a stream) is
remembered in `localStorage`; the manual marker click stops propagation so it
never toggles the layout.

Dependency note: `cobot_tasks` may depend on `cobot_health` â it is itself
rclpy + `cobot_interfaces` only and carries no geometry/robot/camera code, so
the Â§11 god-node rule is preserved.
