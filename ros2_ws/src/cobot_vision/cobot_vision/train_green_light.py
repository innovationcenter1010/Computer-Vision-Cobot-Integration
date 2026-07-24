"""Offline trainer for the green-light model (NOT a ROS node).

Trains a YOLO26 model on vision_workspace/datasets/green_light and copies the
best weights to vision_workspace/models/green_light_best.pt, matching the
naming of cans_obb_best.pt / red_dot_best.pt.

The roboflow export labels the green light with 8-point POLYGONS (segmentation
format, "YOLO26 format"). Two ways to train on that:

  * detect  (default) — ultralytics converts the polygons to bounding boxes;
    yields a light detector whose .boxes give a center, exactly what the
    locate_dot-style consumers want. Base model yolo26n.pt.
  * segment (--task segment) — trains on the polygons as masks. Base model
    yolo26n-seg.pt. Heavier; use only if you actually need the mask.

Run it (host or inside the container):

    python3 -m cobot_vision.train_green_light                 # detect, defaults
    python3 -m cobot_vision.train_green_light --epochs 150
    python3 -m cobot_vision.train_green_light --task segment
    python3 -m cobot_vision.train_green_light --data /path/to/data.yaml

NOTE: the dataset is tiny (18 train / 1 val). One validation image makes the
val metrics close to meaningless — treat them as a smoke test, not accuracy,
and collect more frames (/camera/save_frames) before trusting the numbers.
"""

import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO


# Dataset yaml candidates, in priority order — makes the script run unchanged
# on the host (repo-relative) or in the container (/ws, legacy /app).
_REL = "vision_workspace/datasets/green_light/data.yaml"


def _repo_candidate():
    """Repo-relative data.yaml, or None when run from a shallow path (e.g.
    live-mounted at /ws/train_green_light.py in the container)."""
    parents = Path(__file__).resolve().parents
    return parents[4] / _REL if len(parents) > 4 else None


DATA_CANDIDATES = [
    c for c in (
        _repo_candidate(),
        Path("/ws/" + _REL),
        Path("/app/" + _REL),
    ) if c is not None
]

RUN_NAME = "green_light"
# Base model per task; --model overrides.
DEFAULT_MODELS = {"detect": "yolo26n.pt", "segment": "yolo26n-seg.pt"}


def _resolve_data(cli_data: str | None) -> Path:
    if cli_data:
        path = Path(cli_data).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"--data not found: {path}")
        return path
    for candidate in DATA_CANDIDATES:
        if candidate.exists():
            return candidate
    tried = "\n  ".join(str(c) for c in DATA_CANDIDATES)
    raise FileNotFoundError(
        "green_light data.yaml not found. Tried:\n  " + tried + "\nPass --data <path>."
    )


def _pick_device(requested: str) -> str:
    if requested != "auto":
        return requested
    try:
        import torch  # ultralytics pulls this in
        return "0" if torch.cuda.is_available() else "cpu"
    except Exception:  # noqa: BLE001 — torch missing/broken → just use CPU
        return "cpu"


def train(args) -> Path:
    data_yaml = _resolve_data(args.data)
    # vision_workspace/models sits two levels up from the dataset dir.
    vision_workspace = data_yaml.parents[2]
    runs_dir = vision_workspace / "models" / "training_runs"
    final_model = vision_workspace / "models" / f"{RUN_NAME}_best.pt"

    base_model = args.model or DEFAULT_MODELS[args.task]
    device = _pick_device(args.device)

    print("Training the green-light model", flush=True)
    print(f"  task       : {args.task}", flush=True)
    print(f"  base model : {base_model}", flush=True)
    print(f"  data       : {data_yaml}", flush=True)
    print(f"  epochs     : {args.epochs}", flush=True)
    print(f"  image size : {args.imgsz}", flush=True)
    print(f"  batch      : {args.batch}", flush=True)
    print(f"  device     : {device}", flush=True)
    print(f"  run dir    : {runs_dir / RUN_NAME}", flush=True)
    print(f"  output     : {final_model}", flush=True)

    model = YOLO(base_model)
    model.train(
        data=str(data_yaml),
        task=args.task,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        project=str(runs_dir),
        name=RUN_NAME,
        exist_ok=True,
    )

    best = runs_dir / RUN_NAME / "weights" / "best.pt"
    if not best.exists():
        raise FileNotFoundError(f"training finished but best.pt is missing: {best}")

    final_model.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best, final_model)
    print(f"Done. Best weights copied to: {final_model}", flush=True)
    return final_model


def main():
    parser = argparse.ArgumentParser(description="Train the green-light YOLO26 model.")
    parser.add_argument(
        "--task", choices=["detect", "segment"], default="detect",
        help="detect (boxes, default) or segment (masks). Polygon labels train either.",
    )
    parser.add_argument(
        "--model", default=None,
        help="base weights (default: yolo26n.pt for detect, yolo26n-seg.pt for segment)",
    )
    parser.add_argument("--data", default=None, help="path to data.yaml (auto-resolved if omitted)")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument(
        "--device", default="auto",
        help="'auto' (GPU if available else CPU), 'cpu', or a CUDA index like '0'",
    )
    train(parser.parse_args())


if __name__ == "__main__":
    main()
