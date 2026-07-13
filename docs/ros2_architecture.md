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

Five nodes plus a bringup package. Design decisions locked in:

- **Cobot stays Python + pybind** — no C++ ROS node; `motoman_cpp` is imported exactly as today.
- **Vision splits into two nodes** — ML (YOLO) and geometry (localization) are separate.
- **Vision pushes targets directly to the cobot** (topic) — the cobot's high-level controller
  decides what to run based on input, mirroring the legacy `command_map`.
- **All frames flow as topics** — vision *and* the web UI subscribe to the same camera streams.

```
              ZMQ 5550/5551
 external ──▶ camera_node ─▶ /camera/color/image_raw ──┬─▶ yolo_node
                          └▶ /camera/depth/image_raw ──┼────────────────▶ localization_node
                                                       │                    ▲            │
                                                       └─▶ web_node         │            │
                                                                            │            │
 browser ══ws══▶ web_node ── /vision/scan (Trigger) ──▶ yolo_node           │            │
                    │                                     │                 │            │
                    │                                     ▼                 │            │
                    │◀───────────── /vision/detections_px (pixel space) ────┘            │
                    │◀───────────── /vision/targets (robot space) ◀──────────────────────┘
                    │                        │
                    │                        ▼
                    ├── start/stop/calibrate (Trigger) ──▶ cobot_node (high-level controller)
                    │◀───────── /cobot/status + /cobot/layer_report ──┘
```

### Workspace layout

```
ros2_ws/src/
├── cobot_interfaces/      # msgs/srvs only — depends on nothing local
├── cobot_camera/          # ament_python: camera_streamer.py + camera_node
├── cobot_vision/          # ament_python: yolo_node + localization_node (two executables)
├── motoman_cpp/           # ament_cmake: pybind11 module + libYMConnect (see §7)
├── cobot_control/         # ament_python: cobot.py + cobot_node (imports motoman_cpp)
├── cobot_web/             # ament_python: FastAPI app + web_node
└── cobot_bringup/         # launch files + config YAMLs
```

Dependency rule: every package depends only on `cobot_interfaces` + standard messages
(`sensor_msgs`, `std_srvs`). `cobot_control` additionally depends on `motoman_cpp`.
Nothing depends on `cobot_bringup`.

---

## 3. Node specifications

### 3.1 `camera_node` (pkg `cobot_camera`)

`CameraStream` (ZMQ SUB threads) moves in unchanged; its `on_color`/`on_depth` callbacks
become publisher calls.

- Publishes `/camera/color/image_raw` — `sensor_msgs/Image`, `rgb8`
- Publishes `/camera/depth/image_raw` — `sensor_msgs/Image`, `mono16` (depth is already int16 mm)
- QoS: best-effort, keep-last(1) — freshest frame wins, no backlog
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
- Takes: `yolo_obb_detector.py`, annotated-image saving. `train_yolo_obb.py` remains an
  **offline tool**, not a node.
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
  - pick ordering (`pathing.py` — lives here because ordering is done in real space)
- Publishes `/vision/targets` (`CanTargetArray`) — consumed by `cobot_node` (pick trigger)
  and `web_node` (display)
- Services (cheap depth reads, no ML — see §5):
  - `/vision/verify_target` — is a can still at (x, y), and at what measured z?
  - `/vision/get_layer_height` — median depth over the bin ROI → current layer index
- Calibration math services (`cobot_node` drives the robot between calls — see §10):
  - `/vision/calibration_begin` · `/vision/calibration_sample` · `/vision/calibration_compute`

### 3.4 `cobot_node` (pkg `cobot_control`) — single node, Python, pybind

`cobot.py` and the `motoman_cpp` extension are untouched. The node *is* the high-level
controller — direct descendant of the legacy `command_map`:

- Subscribes `/vision/targets` → dispatches `pick_layer(targets)`
- Services `/cobot/start`, `/cobot/stop`, `/cobot/calibrate` (`std_srvs/Trigger`)
- `/cobot/calibrate` runs the calibration **movement loop** (§10): step the marker through
  the configured pose grid, calling `/vision/calibration_sample` at each stop and
  `/vision/calibration_compute` at the end — movements here, math in localization
- Service clients: `/vision/scan`, `/vision/verify_target`, `/vision/get_layer_height`,
  `/vision/calibration_*` (the controller decides *when* to sense — see §5)
- Publishes `/cobot/status` (`CobotStatus`: idle / picking / stopping / calibrating / error)
  and `/cobot/layer_report` (`LayerReport`, replacing the printed `Layer_Report`)
- One-operation-at-a-time guard (busy flag) held for a whole layer or a whole calibration
  run; a target array arriving mid-operation is rejected
- **`stop` lives in its own callback group** so it is reachable while a pick runs — the one
  place ROS threading genuinely matters here
- Owns the **bin state machine** (layer index, `BIN_COMPLETE`) — pick-sequencing state is
  control state, not perception state. Vision stays stateless. (Legacy `bin.py` kept this in
  vision; it is stale code — it imports modules that no longer exist.)

### 3.5 `web_node` (pkg `cobot_web`)

FastAPI app, templates, static files, `/api/config`, `/ws` — all preserved. The node is the
browser⇄graph bridge:

- The legacy `COMMAND_TRANSLATOR` string→enum table becomes a string→(service client) table
  of the same shape
- Subscribes both image topics; throttles to ~5–10 Hz, JPEG-encodes, base64 over the existing
  websocket (the old `update_image` handler, fed by subscription instead of queue)
- Subscribes `/vision/detections_px`, `/vision/targets`, `/cobot/status`,
  `/cobot/layer_report` and forwards to the browser — the UI finally gets operation feedback
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
msg/CobotStatus.msg         # state enum + detail string
msg/LayerReport.msg         # successes, failures, overall_ok
srv/VerifyTarget.srv        # CanTarget → bool present, float64 measured_z
srv/GetLayerHeight.srv      # (bin ROI from params) → float64 median_z, int8 layer_index
srv/LocateDot.srv           # () → found, dot center px (u, v), confidence     (§10)
srv/CalibrationSample.srv   # commanded TCP (x, y, z) → success, sample_count  (§10)
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
| 1 | **Depth sampling** (localization_node srvs) | ~µs, no ML | "Is a can still at (x,y)? What height is the layer?" |
| 2 | **YOLO scan** (yolo_node) | expensive | "Where is everything and how is it rotated?" |

The cobot high-level controller drives the cycle:

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
  targets exhausted ──▶ /vision/get_layer_height (depth over bin ROI)
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
| Structured data | `cobot_place_trajectories` (named waypoint lists), bin corner matrices | Corners flatten to 8-float params; trajectories stay a **JSON file** referenced by a path param |
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
    model_path: "/ws/vision_workspace/models/cans_obb_best.pt"
    confidence: 0.50
    iou: 0.4
    max_detections: 20
    image_size: 640
    duplicate_center_distance_px: 50.0

localization_node:
  ros__parameters:
    # Spatial setup data lives HERE, never as code defaults (defaults are
    # zeros = unconfigured; dependent features refuse to run until set).
    bin_image_corners_px: [3744.0,1178.0, 4114.0,4524.0, 492.0,4408.0, 904.0,1241.0]
    bin_corners_mm: [448.22,-1600.0, -677.22,-1600.0, -677.22,-565.96, 448.22,-565.96]
    can_height_mm: 87.0
    pathing_row_tolerance_mm: 50.0
    marker_offset_mm: [0.0, 0.0, 0.0]   # measured after mounting the dot (§10)

cobot_node:
  ros__parameters:
    robot_ip: "192.168.1.31"
    tool_head_length_mm: 620.0
    tool_head_xy_offset: 0.0
    tool_head_compression_distance_mm: 50.0
    trajectories_file: "config/trajectories.json"
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

## 7. The pybind module inside colcon

The pybind wrapper structure survives intact as a small `ament_cmake` package
(`motoman_cpp`), reusing the existing `cpp/CMakeLists.txt` content. Two changes only:

1. **Install destination instead of source-tree output.** Replace
   `LIBRARY_OUTPUT_DIRECTORY … python/cobot_application/cobot` with
   `install(TARGETS motoman_cpp DESTINATION ${PYTHON_INSTALL_DIR})`, and install
   `libYMConnect.so` beside it. The `$ORIGIN` RPATH and the copy trick carry over
   unchanged; the destination is the colcon `install/` tree, which lands on `PYTHONPATH`
   when the workspace is sourced. No more build artifacts written into source.
2. **pybind11 from a package** — Humble ships `pybind11_vendor` (or apt `pybind11-dev`),
   allowing the vendored `cpp/third_party/pybind` submodule to be dropped. Keeping the
   vendored copy also works.

`cobot_control` declares `<depend>motoman_cpp</depend>`; colcon guarantees build order.
Import becomes top-level (`import motoman_cpp`) instead of
`from cobot_application.cobot import motoman_cpp` — a one-line change in `cobot.py`.

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
COPY ros2_ws/src/motoman_cpp/package.xml        src/motoman_cpp/package.xml
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
| `vision_system/train_yolo_obb.py` | offline tool (not a node) |
| `cobot/cobot.py`, `cobot_logger.py` | `cobot_control` (import line change only) |
| `cpp/` pybind sources + YMConnect | `motoman_cpp` ament_cmake package |
| `web_app/` templates, static, FastAPI routes | `cobot_web` |

Required code changes found during analysis:

- `run_YOLO(image, …)` ignores its `image` argument and always reads the inbox folder —
  must actually pass the subscribed frame through.
- `cobot.py` hardcodes pick z (300/401 mm) — wire measured `real_z` in.
- `import motoman_cpp` becomes top-level.

### Code that dies

| Legacy | Replaced by |
|---|---|
| `Py_network`, `P2P_command`, command enums, `COMMAND_REGISTRY` | the ROS graph |
| `main.py` | `robot.launch.py` |
| `web_app/web_server.py` | already-dead earlier draft of `webpage_controller.py` |
| camera `snapshot`/`get_stream_provider`, vision `convert_to_jpeg` | frame topics |
| `bin.py` layer heuristics (stale imports, `LAYER_Z` table) | measured depth (§5) + bin state in `cobot_node` |
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
[X, Y, Z]ᵀ = M · [u·d,  v·d,  d,  1]ᵀ
```

Least-squares over the samples absorbs the intrinsics — no CameraInfo needed. The fit
**requires depth variation** (≥30 mm span) or it is rejected: with one height the
feature matrix is rank-3 and Z is unconstrained. This is the "sample at multiple
heights" rule enforced in math. Per-axis RMS residuals are the honest quality measure;
a fit above `calibration_max_rms_mm` is rejected and the old transform kept.

The inverse (robot xyz → expected pixel + depth) is closed-form and powers
`/vision/verify_target`.

### 10.2 The marker offset

The dot is not at the TCP. Each sample's ground truth is `commanded_TCP +
marker_offset_mm`, applied by **localization** before fitting (never absorbed into the
fit — the transform must localize cans, not tool flanges). To keep frames unambiguous
every calibration pose uses one fixed orientation (`calibration_orientation_deg`,
default 180/0/0 = the pick orientation), and the offset is measured **along robot base
axes at that orientation**, to the dot surface the camera sees.

### 10.3 Ownership split — movements vs math

| | owner | does |
|---|---|---|
| `/cobot/calibrate` | `cobot_node` | pose grid from params, drives the robot, holds the one-operation lock for the whole run, reports progress on `/cobot/status` (`calibrating`), stop-button aborts between poses |
| `/vision/calibration_begin` | `localization_node` | clears the sample buffer, warms the red-dot model |
| `/vision/calibration_sample` | `localization_node` | "marker parked at TCP (x,y,z), measure now": N `locate_dot` reads → median (u,v), median-window depth from its own buffer, add marker offset, store the pair |
| `/vision/calibration_compute` | `localization_node` | fit → validate RMS → write `calibration.yaml` → hot-swap the live transform |
| `/vision/locate_dot` | `yolo_node` | red-dot inference on the latest color frame (only ultralytics importer) |

Sequence: web **Calibrate** → `/cobot/calibrate` → `begin` → {move → settle →
`sample`} × poses → `compute` → result in `/cobot/status` detail. Poses that fail
(unreachable, dot occluded, bad depth) are skipped; the fit is overdetermined.

### 10.4 Persistence & the no-fallback rule

`calibration.yaml` is written into the bringup config dir, which docker-compose
bind-mounts — calibration survives container rebuilds. It is per-machine and
gitignored. On startup localization loads it if present; **if absent the node refuses
to localize** (detections dropped loudly, `verify_target` errors). No homography
fallback exists: an unmeasured transform must never move the robot.

Prerequisites before running a calibration (all parameters):

- `cobot_node.calibration_{x,y}_range_mm`, `calibration_z_levels_mm` — safe, reachable,
  in-frame bounds; `[0,0]` means unconfigured and `/cobot/calibrate` refuses
- `localization_node.marker_offset_mm` — measured with a ruler after mounting the dot
- `yolo_node.dot_model_path` — `red_dot_best.pt` in the models mount
