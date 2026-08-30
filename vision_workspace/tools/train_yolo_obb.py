#!/usr/bin/env python3
"""Offline trainer for the can OBB model (NOT a ROS node, never in docker).

Transferred from ros2_ws/src/cobot_vision/cobot_vision/train_yolo_obb.py; the
hardcoded /app paths became vision_workspace-relative.

Trains a YOLO26-OBB model on datasets/cans_obb and copies the best weights to
weights/cans_obb_best.pt. The .pt is a training artefact only — export it to
models/ with export_to_onnx.py before the container can use it.

Run from vision_workspace/ inside .venv:

    python tools/train_yolo_obb.py                       # defaults below
    python tools/train_yolo_obb.py --epochs 100 --device 0
    python tools/train_yolo_obb.py --model yolo26m-obb.pt --batch 4
"""
import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO

from _paths import DATASETS_DIR, TRAINING_RUNS_DIR, WEIGHTS_DIR, pick_device, resolve_base_model

DATA_YAML = DATASETS_DIR / "cans_obb" / "can_obb.yaml"
RUN_NAME = "cans_obb"
FINAL_MODEL_PATH = WEIGHTS_DIR / "cans_obb_best.pt"


def train_yolo_obb(base_model: str, epochs: int, imgsz: int, batch: int,
                   device: str, force: bool = True) -> Path:
    """If force=False and FINAL_MODEL_PATH exists, training is skipped."""
    if FINAL_MODEL_PATH.exists() and not force:
        print(f"Model already exists, skipping training: {FINAL_MODEL_PATH}", flush=True)
        return FINAL_MODEL_PATH
    if not DATA_YAML.exists():
        raise FileNotFoundError(f"Dataset YAML not found: {DATA_YAML}")

    base_model = resolve_base_model(base_model)
    device = pick_device(device)

    print("Starting YOLO OBB training...", flush=True)
    print(f"  base model : {base_model}", flush=True)
    print(f"  data       : {DATA_YAML}", flush=True)
    print(f"  epochs     : {epochs}", flush=True)
    print(f"  image size : {imgsz}", flush=True)
    print(f"  batch      : {batch}", flush=True)
    print(f"  device     : {device}", flush=True)
    print(f"  run dir    : {TRAINING_RUNS_DIR / RUN_NAME}", flush=True)
    print(f"  output     : {FINAL_MODEL_PATH}", flush=True)

    model = YOLO(base_model)
    model.train(
        data=str(DATA_YAML),
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device=device,
        project=str(TRAINING_RUNS_DIR),
        name=RUN_NAME,
        exist_ok=True,
    )

    best_model = TRAINING_RUNS_DIR / RUN_NAME / "weights" / "best.pt"
    if not best_model.exists():
        raise FileNotFoundError(f"Training finished, but best.pt was not found: {best_model}")

    FINAL_MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best_model, FINAL_MODEL_PATH)
    print("Training complete.", flush=True)
    print(f"Best weights copied to: {FINAL_MODEL_PATH}", flush=True)
    print("Next: python tools/export_to_onnx.py cans_obb_best", flush=True)
    return FINAL_MODEL_PATH


def main():
    ap = argparse.ArgumentParser(description="Train the can OBB YOLO26 model.")
    ap.add_argument("--model", default="yolo26n-obb.pt",
                    help="base weights (looked up in weights/base/ first)")
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=1)
    ap.add_argument("--device", default="auto",
                    help="'auto' (GPU if available else CPU), 'cpu', or a CUDA index like '0'")
    ap.add_argument("--skip-if-exists", action="store_true",
                    help="do nothing if weights/cans_obb_best.pt already exists")
    a = ap.parse_args()
    train_yolo_obb(a.model, a.epochs, a.imgsz, a.batch, a.device, force=not a.skip_if_exists)


if __name__ == "__main__":
    main()
