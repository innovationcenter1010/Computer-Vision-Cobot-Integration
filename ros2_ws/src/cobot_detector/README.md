# cobot_detector — C++ perception on ONNX Runtime

The ML half of the vision stack. One node, one action, any number of models.

```
/camera/color/image_raw ──▶ detector_node ──▶ action /vision/detect (Detect)
                                          └─▶ topic  /vision/detections_px (mirror, for the UI)
```

Pixel space only. Geometry (depth, calibration, robot frame, ordering) is
`localization_node` in `cobot_vision`, which calls this action from its own
`/vision/scan` action.

## What ONNX Runtime is, and how this package uses it

**ONNX** is a framework-neutral file format for a trained network: the graph
of standard operators plus the weights. `vision_workspace/tools/export_to_onnx.py`
writes one from a trained ultralytics `.pt`. Unlike `.pt` (a Python pickle) an
`.onnx` file runs no code on load and does not care which torch/ultralytics
version produced it.

**ONNX Runtime (ORT)** is the C++ inference engine for that format. Given a
file it builds an optimised execution plan (operator fusion, constant folding,
memory planning) and runs it on an execution provider — CPU here; CUDA /
TensorRT / OpenVINO are a `SessionOptions` change if a GPU ever appears. It is
a single shared library (`libonnxruntime.so`) with a header-only C++ API, which
is why it can be vendored exactly like YMConnect in `cobot_control`: CMake
imports the `.so` from `ONNXRUNTIME_ROOT`, installs it beside the executable,
and `$ORIGIN` rpath finds it at run time.

Per model (`onnx_detector.cpp`):

1. **Load once** — `Ort::Session` from the path in `models.yaml`. The input
   geometry `[1,3,H,W]` and the output tensor shape are read from the graph
   and the **head layout** is classified (overridable with `layout:` in
   `models.yaml`):

   | layout | output | decode |
   |---|---|---|
   | `raw` | `[1, 4+nc(+1), N]` | threshold + corners + polygon-IoU NMS in C++ |
   | `end2end` (YOLO26) | `[1, max_det, 6\|7]` = `cx,cy,w,h,conf,cls(,angle)` | NMS is inside the graph; threshold + unmap only |
   | `seg` (YOLOE) | `[1, 4+nc+mask, N]` + proto output | boxes only; mask rows and proto ignored; NMS in C++ |

   Everything is validated against the head type the yaml claims and against
   the exporter's `manifest.yaml` when present (task, class count, mask
   channels), so a mislabelled model fails at load with a clear message.
   Seg models *require* the manifest — the class count can't be inferred from
   the shape alone.
2. **Per detection** — letterbox the RGB frame to `H×W` (`letterbox.cpp`, same
   arithmetic as ultralytics), wrap the float buffer in an `Ort::Value` (no
   copy), `session.Run()`, decode per the table above ((`cx,cy,w,h,θ`) → four
   corners in ultralytics `xyxyxyxy` order, undo the letterbox), then the
   centre-distance duplicate filter the old Python node applied (when
   `remove_duplicates` is on), then cap at `max_detections`. When dedup is on,
   the NMS/decode stage itself samples up to `2 × max_detections` first, so a
   duplicate inside that sample can't silently leave the final result short
   of `max_detections`.

Nothing here imports ultralytics or torch. They are training-only and live in
the dev-PC `.venv` (`vision_workspace/`).

## models.yaml

Bind-mounted at `/ws/runtime_models/models.yaml`
(template: [config/models.example.yaml](config/models.example.yaml)).

```yaml
runtime:                      # models the application runs every cycle
  cans:
    path: cans_obb_best/cans_obb_best.onnx      # relative to this file
    type: obb                                   # obb | bbox
    confidence: 0.50
    iou: 0.40
    max_detections: 20
    remove_duplicates: true
    duplicate_center_distance_px: 50.0
calibration:                  # models only the calibration routine needs
  red_dot:
    path: red_dot_best/red_dot_best.onnx
    type: bbox
    confidence: 0.50
```

The leaf key is the `model_id` a goal names. Ids must be unique across both
sections. Both head types produce the same `ObbDetection`; a `bbox` model
fills an axis-aligned rectangle with `angle = 0`.

## Action `/vision/detect`

| | |
|---|---|
| goal | `string model_id`, `string image_source` (optional container path: run on that file instead of the cached frame — no camera needed) |
| result | `success`, `message`, `ObbDetectionArray detections` (stamp = colour frame used, `model_id` echoed), `inference_ms` |
| feedback | `stage`: `loading_model` · `preprocess` · `inference` · `postprocess` |
| rejected when | unknown id · no colour frame yet (without `image_source`) · `image_source` not found · a detection already running |

Models are lazy-loaded on first goal (seconds) unless listed in the `preload`
parameter.

## Parameters

| name | default | |
|---|---|---|
| `models_yaml` | `/ws/runtime_models/models.yaml` | registry path |
| `preload` | `[]` (`["cans"]` in cobot_params.yaml) | ids loaded at startup |
| `intra_op_threads` | `0` | ORT CPU threads, 0 = all cores |
| `heartbeat_period_s` | `2.0` | |
| `min_confidence` | `-1.0` | overrides every model's `confidence` from `models.yaml` when `>= 0`; `-1` = use each model's own value |
| `max_detections` | `-1` | overrides every model's `max_detections` from `models.yaml` when `>= 1`; `-1` = use each model's own value |

`min_confidence` / `max_detections` are ordinary (mutable, non-read-only) ROS
parameters, re-read from the node on every `/vision/detect` goal — tune them
live with `ros2 param set /detector_node min_confidence 0.6` (no restart, no
`models.yaml` edit). Per-model defaults in `models.yaml` still apply whenever
the node param is left at its `-1` sentinel.

## Build

```
# inside the ROS container (docker/Dockerfile unpacks ORT to /opt/onnxruntime)
colcon build --packages-up-to cobot_detector --cmake-args -DONNXRUNTIME_ROOT=/opt/onnxruntime
colcon test  --packages-select cobot_detector && colcon test-result --verbose
```

Tests: `test_letterbox`, `test_postprocess`, `test_model_registry` are
self-contained. `test_golden` runs the real `.onnx` files on frames under
`test/data/golden/` and compares with what the retired ultralytics node
produced (`vision_workspace/tools/dump_golden.py`); it skips itself when the
models or golden files are absent.

## Try it

```
ros2 action send_goal --feedback /vision/detect cobot_interfaces/action/Detect "{model_id: cans}"
ros2 action send_goal --feedback /vision/detect cobot_interfaces/action/Detect "{model_id: red_dot}"
ros2 action send_goal /vision/detect cobot_interfaces/action/Detect "{model_id: cans, image_source: /ws/vision_workspace/datasets/cans_obb/images/train/IMG_001.jpeg}"
ros2 topic echo /vision/detections_px --once
ros2 topic echo /health/heartbeat | grep -A12 detector_node
```
