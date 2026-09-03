# vision_workspace/ — datasets, training, and ONNX export (dev PC)

This directory is the whole model lifecycle for the cobot's vision stack, and
it is bind-mounted read/write into the container at `/ws/vision_workspace`
(see `docker-compose.yaml`).

**Rules**

- **No training in the docker image. Ever.** The image only runs inference.
  Training and export happen here, on the dev PC, inside `.venv/`.
- **`models/` holds ONNX only.** It is the hand-off to the container: one
  directory per model with `<name>.onnx` + `manifest.yaml`.
- **`.pt` files never go in `models/`.** Trained weights live in `weights/`
  (gitignored), pretrained starting points in `weights/base/`.
- The camera streams **640x480 (4:3)**, so every export defaults to
  `--imgsz 480 640` (H W) — the frame maps onto the net 1:1 with no letterbox
  padding. Do not copy the Unitree G1's 16:9 `544 960` shape.

## Layout

```
vision_workspace/
├── README.md                 # this file
├── requirements.txt          # ultralytics + onnx + onnxslim + pyyaml (dev PC only)
├── .venv/                    # dev-PC Python env (gitignored) — see Setup
├── tools/                    # every script runs from vision_workspace/ inside .venv
│   ├── _paths.py             # workspace-relative paths shared by the tools
│   ├── train_yolo_obb.py     # datasets/cans_obb   -> weights/cans_obb_best.pt
│   ├── train_green_light.py  # datasets/green_light -> weights/green_light_best.pt
│   ├── export_to_onnx.py     # weights/<x>.pt      -> models/<x>/<x>.onnx + manifest.yaml
│   ├── export_yoloe.py       # open-vocab YOLOE prompts -> models/yoloe_cobot_vN/ (mirror of the G1 tool)
│   ├── classes_cobot.yaml    # the YOLOE prompt list (order = class index)
│   └── manifest.example.yaml # what a YOLOE manifest looks like before anything is exported
├── datasets/
│   ├── cans_obb/             # YOLO-OBB, single class "top can" (images/train, images/val)
│   └── green_light/          # Roboflow export, polygon labels, single class "green_light"
├── weights/                  # trained .pt (gitignored)
│   ├── base/                 # yolo26n-obb.pt, yolo26m-obb.pt, ... (pretrained starting points)
│   └── red_dot_best.pt       # calibration marker detector (dot model)
├── training_runs/            # ultralytics run output: metrics, plots, weights/best.pt (gitignored)
├── (exports land in ../runtime_models/ — ONNX + models.yaml, see its README)
├── operation/                # runtime I/O with the container: camera_intrinsics.json, captures/, results/, inbox/
├── calibration/images/       # frames saved for camera→robot calibration
└── tests/                    # standalone hardware/streaming smoke tests (not pytest)
```

## Setup (once, dev PC)

Python ≥ 3.10. No GPU needed — the CPU torch wheel is enough for export and
for these small datasets (a CUDA torch makes training faster; install it first
from pytorch.org and then run the `pip install -r` on top).

```powershell
cd vision_workspace
python -m venv .venv
.venv\Scripts\python -m pip install --upgrade pip
.venv\Scripts\python -m pip install -r requirements.txt
```

> **Windows note — "filename or extension is too long" while installing torch.**
> Windows still defaults to a 260-character path limit and torch's wheel has
> a very deep license tree. Either enable long paths once (admin PowerShell:
> `New-ItemProperty -Path HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem -Name LongPathsEnabled -Value 1 -PropertyType DWORD -Force`,
> then reboot), or install through a short drive alias without admin:
>
> ```powershell
> subst V: C:\Users\<you>\Downloads\Computer-Vision-Cobot-Integration\vision_workspace
> cd V:\
> python -m venv .venv
> .venv\Scripts\python -m pip install -r requirements.txt
> ```
>
> The resulting `.venv` is the same folder on disk and works from the long
> path afterwards; only `pip install` needs the short one (`subst` is per
> login session — re-run it before installing more packages).

Every command below is run **from `vision_workspace/`** with the venv's
python (`.venv\Scripts\python` on Windows, `.venv/bin/python` elsewhere).

## 1. Train on the existing datasets

```powershell
.venv\Scripts\python tools\train_yolo_obb.py                 # 50 epochs, yolo26n-obb, cpu/auto
.venv\Scripts\python tools\train_yolo_obb.py --epochs 100 --device 0 --batch 4
.venv\Scripts\python tools\train_green_light.py              # detect task, yolo26n
.venv\Scripts\python tools\train_green_light.py --task segment
```

- Base weights are looked up in `weights/base/` first; anything not there is
  downloaded by ultralytics on first use.
- Run output goes to `training_runs/<run>/` (metrics, curves, `weights/best.pt`);
  the best weights are copied to `weights/<run>_best.pt`.
- `--device auto` picks CUDA 0 if torch can see a GPU, else CPU.
- The green-light dataset is 18 train / 1 val — its val metrics are a smoke
  test, not accuracy. Collect more frames with the UI's **Save frames**
  button (`/camera/save_frames` → `operation/captures/`), label them, and
  retrain.

## 2. Export trained weights to ONNX (`models/`)

```powershell
.venv\Scripts\python tools\export_to_onnx.py cans_obb_best               # -> models/cans_obb_best/
.venv\Scripts\python tools\export_to_onnx.py green_light_best
.venv\Scripts\python tools\export_to_onnx.py red_dot_best --name red_dot_v1
.venv\Scripts\python tools\export_to_onnx.py cans_obb_best --imgsz 640 640   # square, as trained
```

Writes `models/<name>/<name>.onnx` (opset 17, static batch 1, static
`1x3x480x640`) and a `manifest.yaml` with the sha256, input geometry, output
tensor shapes, task (`obb`/`detect`/`segment`), and class list.

## 3. Export a YOLOE prompt model (`models/yoloe_cobot_vN/`)

YOLOE is open-vocabulary: instead of training, you choose a list of text
prompts and they get baked into the detection head at export time. This is a
straight mirror of `Unitree-G1-Development/tools/models/export_yoloe.py`
(confirmed working there) with the camera shape and paths changed.

Pick the prompts either in `tools/classes_cobot.yaml` (order = class index)
or inline:

```powershell
.venv\Scripts\python tools\export_yoloe.py                                        # classes_cobot.yaml -> models/yoloe_cobot_v1/
.venv\Scripts\python tools\export_yoloe.py --prompts "aluminium can" "green light" "red dot"
.venv\Scripts\python tools\export_yoloe.py --name yoloe_cobot_v2 --checkpoint yoloe-11m-seg.pt
```

First run downloads `yoloe-11s-seg.pt` (~28 MB) and the MobileCLIP text
encoder `mobileclip_blt.ts` (~600 MB, used once to embed the prompts, then
discarded — it never ships). Both land in the cwd and are gitignored.
`tools/manifest.example.yaml` shows the manifest it writes. Bump `--name`
whenever the prompt set changes — the sha256 changes with it.

**Status:** `detector_node` (C++, `ros2_ws/src/cobot_detector`) reads
`/ws/runtime_models/models.yaml` (repo-root `runtime_models/`, bind-mounted)
and runs the `.onnx` files it points at through ONNX Runtime; `manifest.yaml`
beside each export is cross-checked at load (head type, class count). Export
with —

    python tools/export_to_onnx.py cans_obb_best     # -> runtime_models/cans_obb_best/
    python tools/export_to_onnx.py red_dot_best      # -> runtime_models/red_dot_best/

— and `runtime_models/models.yaml` names them. `.pt` weights never enter the
container.

## What the container sees

`docker-compose.yaml` mounts this directory at `/ws/vision_workspace`, so:

| host                                   | container                                   |
|----------------------------------------|---------------------------------------------|
| `runtime_models/<n>/` + `models.yaml`  | `/ws/runtime_models/` (what detector_node loads) |
| `vision_workspace/operation/`          | `/ws/vision_workspace/operation/` (runtime I/O) |

No image rebuild is ever needed to swap a model. `.venv/`, `weights/`,
`training_runs/` and downloaded checkpoints are gitignored; `models/` ONNX +
manifests are small enough to commit if you want them versioned.
