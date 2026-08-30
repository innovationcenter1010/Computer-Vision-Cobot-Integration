#!/usr/bin/env python3
"""Export a trained .pt from weights/ to portable ONNX under models/.

models/ holds ONNX only — it is the bind-mounted hand-off to the container.
.pt files are training artefacts and stay in weights/ (gitignored).

Each export gets its own directory with the same manifest layout the YOLOE
exporter writes (sha256, input geometry, output tensor shapes, class names),
so a future consumer node can validate any model the same way:

    models/<name>/<name>.onnx
    models/<name>/manifest.yaml

Run from vision_workspace/ inside .venv:

    python tools/export_to_onnx.py cans_obb_best                 # weights/cans_obb_best.pt -> models/cans_obb_best/
    python tools/export_to_onnx.py green_light_best --imgsz 480 640
    python tools/export_to_onnx.py red_dot_best --name red_dot_v1

Default input is 480x640 (H W) to match the 640x480 cobot camera 1:1; both
must be multiples of 32. The existing models were trained square (imgsz 640);
exporting a fully-convolutional net rectangular is fine, but validate on real
frames before trusting it.
"""
import argparse
import hashlib
import shutil
import sys
from pathlib import Path

import yaml

from _paths import MODELS_DIR, WEIGHTS_DIR


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def tensor_shapes(onnx_path: Path):
    import onnx

    model = onnx.load(str(onnx_path))
    onnx.checker.check_model(model)

    def describe(values):
        return [{"name": v.name,
                 "shape": [d.dim_value for d in v.type.tensor_type.shape.dim]}
                for v in values]

    return describe(model.graph.input), describe(model.graph.output)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("weights", help="name of a .pt in weights/ (with or without .pt), or a path")
    ap.add_argument("--name", default=None, help="output name (default: the weights stem)")
    ap.add_argument("--imgsz", type=int, nargs="+", default=[480, 640], metavar=("H", "W"),
                    help="input size 'H W' or one value for square (default 480 640)")
    ap.add_argument("--opset", type=int, default=17)
    ap.add_argument("--out", type=Path, default=MODELS_DIR)
    ap.add_argument("--conf-threshold", type=float, default=0.25)
    ap.add_argument("--nms-iou", type=float, default=0.7)
    args = ap.parse_args()

    if len(args.imgsz) == 1:
        args.imgsz = [args.imgsz[0], args.imgsz[0]]
    if len(args.imgsz) != 2 or any(v <= 0 or v % 32 for v in args.imgsz):
        sys.exit(f"error: --imgsz {args.imgsz} must be one or two positive multiples of 32")

    pt = Path(args.weights)
    if not pt.exists():
        stem = args.weights if args.weights.endswith(".pt") else args.weights + ".pt"
        pt = WEIGHTS_DIR / stem
    if not pt.exists():
        sys.exit(f"error: weights not found: {args.weights} (looked in {WEIGHTS_DIR})")
    name = args.name or pt.stem

    from ultralytics import YOLO  # deferred: heavy import

    model = YOLO(str(pt))
    task = model.task
    names = [model.names[i] for i in sorted(model.names)]
    print(f"[export] {pt} task={task} classes={names}")

    exported = Path(model.export(format="onnx", opset=args.opset, imgsz=args.imgsz,
                                 batch=1, dynamic=False, simplify=True, device="cpu"))

    out_dir = args.out / name
    out_dir.mkdir(parents=True, exist_ok=True)
    onnx_path = out_dir / f"{name}.onnx"
    shutil.move(str(exported), onnx_path)

    inputs, outputs = tensor_shapes(onnx_path)
    in_shape = inputs[0]["shape"]
    if len(inputs) != 1 or len(in_shape) != 4 or in_shape[1] != 3:
        sys.exit(f"error: unexpected input tensors {inputs}, want one [1, 3, H, W]")
    det = outputs[0]

    manifest = {
        "schema_version": 1,
        "name": name,
        "type": "detector",
        "task": task,  # obb | detect | segment
        "source_weights": pt.name,
        "onnx": onnx_path.name,
        "onnx_sha256": sha256_of(onnx_path),
        "input": {"name": inputs[0]["name"], "width": in_shape[3], "height": in_shape[2],
                  "layout": "nchw", "normalize": "div255"},
        "output": {"name": det["name"], "layout": f"yolo_{task}", "shape": det["shape"]},
        "unused_outputs": [o["name"] for o in outputs[1:]],
        "classes": names,
        "letterbox": {"pad_value": 114},
        "defaults": {"conf_threshold": args.conf_threshold, "nms_iou": args.nms_iou},
    }
    manifest_path = out_dir / "manifest.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False, default_flow_style=False),
                             encoding="utf-8")
    print(f"[export] wrote {onnx_path} ({onnx_path.stat().st_size / 1e6:.1f} MB)")
    print(f"[export] wrote {manifest_path}")
    print(f"[export] input  {inputs[0]['name']} {in_shape}")
    for o in outputs:
        print(f"[export] output {o['name']} {o['shape']}")
    print("[export] done — visible in the container at /ws/vision_workspace/models/"
          f"{name}/ (bind mount, no rebuild)")


if __name__ == "__main__":
    main()
