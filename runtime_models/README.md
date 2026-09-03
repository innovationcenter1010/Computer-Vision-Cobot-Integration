# runtime_models/ — what the robot actually runs

The hand-off from the dev PC to the container: `docker-compose.yaml`
bind-mounts this directory at `/ws/runtime_models`, and `detector_node`
(pkg `cobot_detector`) loads `models.yaml` from here. Swapping a model never
needs an image rebuild — restart `detector_node`.

Rules:
- **Only `.onnx` + its `manifest.yaml`** live here, one directory per model,
  plus the `models.yaml` registry at the root.
- No `.pt` — trained weights stay in `../vision_workspace/weights/`
  (gitignored) and are exported here with
  `vision_workspace/tools/export_to_onnx.py`; YOLOE prompt models with
  `tools/export_yoloe.py`.
- No training happens in the docker image. Ever.

Layout:

```
runtime_models/
├── models.yaml          # id → path + type + thresholds (what a Detect goal names)
└── <name>/
    ├── <name>.onnx      # opset 17, static batch 1, static input geometry
    └── manifest.yaml    # sha256, input W/H, output tensors, class list, decode defaults
```

`manifest.yaml` is cross-checked by `detector_node` at load (head type, class
count) so a mislabelled model fails at startup with a clear message instead of
mid-pick.
