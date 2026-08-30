#!/usr/bin/env python3
"""Offline trainer for the green-light model (NOT a ROS node, never in docker).

Transferred from ros2_ws/src/cobot_vision/cobot_vision/train_green_light.py.

Trains a YOLO26 model on datasets/green_light and copies the best weights to
weights/green_light_best.pt. Export to models/ with export_to_onnx.py.

The roboflow export labels the green light with 8-point POLYGONS (segmentation
format). Two ways to train on that:

  * detect  (default) — ultralytics converts the polygons to bounding boxes;
    yields a light detector whose .boxes give a center. Base model yolo26n.pt.
  * segment (--task segment) — trains on the polygons as masks. Base model
    yolo26n-seg.pt. Heavier; use only if you actually need the mask.

Run from vision_workspace/ inside .venv:

    python tools/train_green_light.py                 # detect, defaults
    python tools/train_green_light.py --epochs 150
    python tools/train_green_light.py --task segment
    python tools/train_green_light.py --data /path/to/data.yaml

NOTE: the dataset is tiny (18 train / 1 val). One validation image makes the
val metrics close to meaningless — treat them as a smoke test, not accuracy,
and collect more frames (/camera/save_frames) before trusting the numbers.
"""
import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO

from _paths import DATASETS_DIR, TRAINING_RUNS_DIR, WEIGHTS_DIR, pick_device, resolve_base_model

DATA_YAML = DATASETS_DIR / "green_light" / "data.yaml"
RUN_NAME = "green_light"
FINAL_MODEL_PATH = WEIGHTS_DIR / "green_light_best.pt"
DEFAULT_MODELS = {"detect": "yolo26n.pt", "segment": "yolo26n-seg.pt"}


def train(args) -> Path:
    data_yaml = Path(args.data).expanduser().resolve() if args.data else DATA_YAML
    if not data_yaml.exists():
        raise FileNotFoundError(f"data.yaml not found: {data_yaml}")

    base_model = resolve_base_model(args.model or DEFAULT_MODELS[args.task])
    device = pick_device(args.device)

    print("Training the green-light model", flush=True)
    print(f"  task       : {args.task}", flush=True)
    print(f"  base model : {base_model}", flush=True)
    print(f"  data       : {data_yaml}", flush=True)
    print(f"  epochs     : {args.epochs}", flush=True)
    print(f"  image size : {args.imgsz}", flush=True)
    print(f"  batch      : {args.batch}", flush=True)
    print(f"  device     : {device}", flush=True)
    print(f"  run dir    : {TRAINING_RUNS_DIR / RUN_NAME}", flush=True)
    print(f"  output     : {FINAL_MODEL_PATH}", flush=True)

    model = YOLO(base_model)
    model.train(
        data=str(data_yaml),
        task=args.task,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        project=str(TRAINING_RUNS_DIR),
        name=RUN_NAME,
        exist_ok=True,
    )

    best = TRAINING_RUNS_DIR / RUN_NAME / "weights" / "best.pt"
    if not best.exists():
        raise FileNotFoundError(f"training finished but best.pt is missing: {best}")

    FINAL_MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best, FINAL_MODEL_PATH)
    print(f"Done. Best weights copied to: {FINAL_MODEL_PATH}", flush=True)
    print("Next: python tools/export_to_onnx.py green_light_best", flush=True)
    return FINAL_MODEL_PATH


def main():
    parser = argparse.ArgumentParser(description="Train the green-light YOLO26 model.")
    parser.add_argument("--task", choices=["detect", "segment"], default="detect",
                        help="detect (boxes, default) or segment (masks). Polygon labels train either.")
    parser.add_argument("--model", default=None,
                        help="base weights (default: yolo26n.pt for detect, yolo26n-seg.pt for segment)")
    parser.add_argument("--data", default=None,
                        help="path to data.yaml (default: datasets/green_light/data.yaml)")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="auto",
                        help="'auto' (GPU if available else CPU), 'cpu', or a CUDA index like '0'")
    train(parser.parse_args())


if __name__ == "__main__":
    main()
