# models/ — ONNX exports ONLY

This is the hand-off from the dev PC to the container: `docker-compose.yaml`
bind-mounts `vision_workspace/` at `/ws/vision_workspace`, so anything here is
visible at `/ws/vision_workspace/models/<name>/` with no image rebuild.

Rules:
- **Only `.onnx` + its `manifest.yaml`** live here, one directory per model.
- No `.pt` — trained weights stay in `../weights/` (gitignored) and are
  exported here with `tools/export_to_onnx.py`; YOLOE prompt models are
  exported here with `tools/export_yoloe.py`.
- No training happens in the docker image. Ever.

Layout:

```
models/
└── <name>/
    ├── <name>.onnx      # opset 17, static batch 1, static input geometry
    └── manifest.yaml    # sha256, input W/H, output tensors, class list, decode defaults
```

Currently empty — nothing has been exported yet. Which model a node loads is
still decided by `ros2_ws/src/cobot_bringup/config/cobot_params.yaml`.
