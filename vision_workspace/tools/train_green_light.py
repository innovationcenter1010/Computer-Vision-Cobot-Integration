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

  * --linear-probe — YOLOE prompt-embedding fine-tune instead of YOLO26. Loads
    yoloe-11s-seg.pt (re-headed as a detect model, cached in weights/base/),
    fuses the text embedding of "green_light" into the class branch, FREEZES
    backbone, neck, box branch and the first two class-branch convs, and trains
    only the final 1x1 conv per scale (3 x Conv2d(128->1) = 387 parameters for
    yoloe-11s). The vision tower cannot overfit 18 frames because it never
    moves; the probe only relocates the class vector from "where the phrase
    'green light' points in CLIP space" to "where this LED actually is".
    Official recipe: AdamW, lr0 1e-3, wd 0.025, 80 epochs (Ultralytics YOLOE
    docs, "Linear probing"). --probe-depth 2|3 unfreezes more of the class
    branch if 387 parameters turn out to be too few. Output weights/green_light_v2.pt.

    python tools/train_green_light.py --linear-probe
    python tools/train_green_light.py --linear-probe --probe-depth 2 --epochs 120
    python tools/export_to_onnx.py green_light_v2          # -> runtime_models/green_light_v2/

NOTE: the dataset is tiny (18 train / 1 val). One validation image makes the
val metrics close to meaningless — treat them as a smoke test, not accuracy,
and collect more frames (/camera/save_frames) before trusting the numbers.
"""
import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO

from _paths import (BASE_WEIGHTS_DIR, DATASETS_DIR, TRAINING_RUNS_DIR, WEIGHTS_DIR, pick_device,
                     resolve_base_model)

DATA_YAML = DATASETS_DIR / "green_light" / "data.yaml"
RUN_NAME = "green_light"
DEFAULT_OUT_NAME = "green_light_best"
DEFAULT_EPOCHS = 100
DEFAULT_MODELS = {"detect": "yolo26n.pt", "segment": "yolo26n-seg.pt"}

# --linear-probe (YOLOE). Ultralytics only ships YOLOE weights with a seg head.
PROBE_BASE_MODEL = "yoloe-11s-seg.pt"
PROBE_RUN_NAME = "green_light_probe"
PROBE_OUT_NAME = "green_light_v2"
PROBE_EPOCHS = 80


def train(args) -> Path:
    data_yaml = Path(args.data).expanduser().resolve() if args.data else DATA_YAML
    if not data_yaml.exists():
        raise FileNotFoundError(f"data.yaml not found: {data_yaml}")

    base_model = resolve_base_model(args.model or DEFAULT_MODELS[args.task])
    device = pick_device(args.device)
    epochs = args.epochs or DEFAULT_EPOCHS
    final_path = WEIGHTS_DIR / f"{args.out_name or DEFAULT_OUT_NAME}.pt"

    print("Training the green-light model", flush=True)
    print(f"  task       : {args.task}", flush=True)
    print(f"  base model : {base_model}", flush=True)
    print(f"  data       : {data_yaml}", flush=True)
    print(f"  epochs     : {epochs}", flush=True)
    print(f"  image size : {args.imgsz}", flush=True)
    print(f"  batch      : {args.batch}", flush=True)
    print(f"  device     : {device}", flush=True)
    print(f"  run dir    : {TRAINING_RUNS_DIR / RUN_NAME}", flush=True)
    print(f"  output     : {final_path}", flush=True)

    model = YOLO(base_model)
    model.train(
        data=str(data_yaml),
        task=args.task,
        epochs=epochs,
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

    final_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best, final_path)
    print(f"Done. Best weights copied to: {final_path}", flush=True)
    print(f"Next: python tools/export_to_onnx.py {final_path.stem}", flush=True)
    return final_path


def yoloe_detect_checkpoint(seg_weights: str) -> Path:
    """Re-head a YOLOE *-seg checkpoint as a plain detect model, cached in weights/base/.

    Ultralytics only publishes YOLOE weights with a segmentation head, but the
    green_light labels are boxes (Roboflow polygons collapse to boxes in detect
    mode and one frame is box-only), so the detect variant is trained: same
    backbone, neck, box branch and class branch, minus the mask branch. Every
    matching tensor transfers (524/524 for yoloe-11s).
    """
    import torch
    from ultralytics import YOLOE
    from ultralytics.nn.tasks import YOLOEModel

    stem = Path(seg_weights).stem.replace("-seg", "")          # yoloe-11s-seg -> yoloe-11s
    out = BASE_WEIGHTS_DIR / f"{stem}-det.pt"
    if out.exists():
        return out
    seg = YOLOE(seg_weights).model                              # auto-downloads on first use
    det = YOLOEModel(f"{stem}.yaml", nc=80, verbose=False)      # yoloe-11.yaml, scale 's'
    det.load(seg)
    out.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"model": det.half(), "train_args": {}, "date": None}, out)
    print(f"  cached detect-head checkpoint: {out}", flush=True)
    return out


def probe_freeze_list(model, depth: int) -> list[str]:
    """Freeze everything except the tail of the head's class branch (cv3).

    depth 1: only the final 1x1 conv per scale — the fused text embedding.
             Official linear-probing recipe; 387 parameters for yoloe-11s.
    depth 2: also the second conv block of the class branch.
    depth 3: the whole class branch. Backbone, neck and box branch stay
             frozen at every depth.
    Entries are matched by ultralytics as the prefix "model.<entry>." on
    parameter names, so "23.cv3.0.1" freezes model.23.cv3.0.1.* only.
    """
    head = len(model.model.model) - 1
    freeze = [str(i) for i in range(head)]                      # backbone + neck
    for name, _ in model.model.model[-1].named_children():      # cv2 (box), dfl, ... but not cv3
        if "cv3" not in name:
            freeze.append(f"{head}.{name}")
    frozen_blocks = 3 - depth                                   # cv3[scale] = (block0, block1, final conv)
    freeze += [f"{head}.cv3.{scale}.{block}" for scale in range(3) for block in range(frozen_blocks)]
    return freeze


def train_linear_probe(args) -> Path:
    from ultralytics import YOLOE
    from ultralytics.models.yolo.yoloe import YOLOEPETrainer

    data_yaml = Path(args.data).expanduser().resolve() if args.data else DATA_YAML
    if not data_yaml.exists():
        raise FileNotFoundError(f"data.yaml not found: {data_yaml}")
    if args.task != "detect":
        raise SystemExit("--linear-probe trains the detect head; drop --task segment")

    device = pick_device(args.device)
    epochs = args.epochs or PROBE_EPOCHS
    final_path = WEIGHTS_DIR / f"{args.out_name or PROBE_OUT_NAME}.pt"
    run_name = PROBE_RUN_NAME if final_path.stem == PROBE_OUT_NAME else f"{PROBE_RUN_NAME}_{final_path.stem}"
    base = yoloe_detect_checkpoint(resolve_base_model(args.model or PROBE_BASE_MODEL))

    model = YOLOE(str(base))
    freeze = probe_freeze_list(model, args.probe_depth)

    print("Linear-probing YOLOE for the green-light model", flush=True)
    print(f"  base model : {base}", flush=True)
    print(f"  data       : {data_yaml}", flush=True)
    print(f"  epochs     : {epochs}", flush=True)
    print(f"  image size : {args.imgsz}", flush=True)
    print(f"  batch      : {args.batch}", flush=True)
    print(f"  device     : {device}", flush=True)
    print(f"  probe depth: {args.probe_depth} ({len(freeze)} frozen entries)", flush=True)
    print(f"  lr0        : {args.lr0}", flush=True)
    print(f"  run dir    : {TRAINING_RUNS_DIR / run_name}", flush=True)
    print(f"  output     : {final_path}", flush=True)

    # YOLOEPETrainer: computes the text embedding of the dataset class names,
    # fuses it into cv3's final conv (init from language, not random) and
    # trains with the ordinary box/cls/dfl losses. Hyper-parameters are the
    # official YOLOE linear-probing recipe.
    model.train(
        data=str(data_yaml),
        task="detect",
        trainer=YOLOEPETrainer,
        epochs=epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        freeze=freeze,
        optimizer="AdamW",
        lr0=args.lr0,
        warmup_bias_lr=0.0,
        weight_decay=0.025,
        momentum=0.9,
        close_mosaic=min(10, epochs),
        project=str(TRAINING_RUNS_DIR),
        name=run_name,
        exist_ok=True,
    )
    trainable = [(k, tuple(v.shape)) for k, v in model.trainer.model.named_parameters() if v.requires_grad]
    n_params = sum(v.numel() for _, v in model.trainer.model.named_parameters() if v.requires_grad)
    print(f"  trained {n_params} parameters in {len(trainable)} tensors:", flush=True)
    for k, shape in trainable:
        print(f"    {k} {shape}", flush=True)

    # The valid split currently has no labelled instance, so fitness is 0 in
    # every epoch and best.pt never moves past epoch 1 — take the final weights.
    last = TRAINING_RUNS_DIR / run_name / "weights" / "last.pt"
    if not last.exists():
        raise FileNotFoundError(f"training finished but last.pt is missing: {last}")
    final_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(last, final_path)
    print(f"Done. Final weights copied to: {final_path}", flush=True)
    print(f"Next: python tools/export_to_onnx.py {final_path.stem}", flush=True)
    return final_path


def main():
    parser = argparse.ArgumentParser(description="Train the green-light YOLO26 model.")
    parser.add_argument("--task", choices=["detect", "segment"], default="detect",
                        help="detect (boxes, default) or segment (masks). Polygon labels train either.")
    parser.add_argument("--model", default=None,
                        help="base weights (default: yolo26n.pt for detect, yolo26n-seg.pt for segment)")
    parser.add_argument("--data", default=None,
                        help="path to data.yaml (default: datasets/green_light/data.yaml)")
    parser.add_argument("--epochs", type=int, default=None,
                        help=f"default {DEFAULT_EPOCHS} (YOLO26) / {PROBE_EPOCHS} (--linear-probe)")
    parser.add_argument("--out-name", default=None,
                        help=f"weights/<name>.pt (default {DEFAULT_OUT_NAME}, or {PROBE_OUT_NAME} with --linear-probe)")
    parser.add_argument("--linear-probe", action="store_true",
                        help=f"YOLOE prompt-embedding fine-tune of {PROBE_BASE_MODEL}: vision tower frozen, "
                             "only the class branch tail trains (see module docstring)")
    parser.add_argument("--probe-depth", type=int, choices=[1, 2, 3], default=1,
                        help="--linear-probe only: 1 = final conv per scale (official recipe), "
                             "2 = + previous conv block, 3 = whole class branch")
    parser.add_argument("--lr0", type=float, default=1e-3,
                        help="--linear-probe only: initial learning rate (official recipe 1e-3)")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="auto",
                        help="'auto' (GPU if available else CPU), 'cpu', or a CUDA index like '0'")
    args = parser.parse_args()
    (train_linear_probe if args.linear_probe else train)(args)


if __name__ == "__main__":
    main()
