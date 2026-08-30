#!/usr/bin/env python3
"""Export a fixed-vocabulary YOLOE detector to portable ONNX + manifest.

Mirrored from Unitree-G1-Development/tools/models/export_yoloe.py (confirmed
working there); only the paths, the default input shape and the docs differ.

Dev-PC tool ONLY: never runs inside the cobot container, never in CI, never
inside colcon. Run it from vision_workspace/.venv (see ../README.md). The output lands in
vision_workspace/models/<name>/ next to the other exported ONNX models and
is NOT wired into any node yet.

Flow:
  1. Load an open-vocab YOLOE checkpoint (auto-downloaded on first run).
  2. Bake the prompt list from --classes (or --prompts) into the head via text
     embeddings (set_classes + get_text_pe) — the text encoder is discarded at
     export.
  3. Export ONNX (opset 17), static batch 1, static imgsz.
  4. Write vision_workspace/models/<name>/<name>.onnx + manifest.yaml (sha256,
     real tensor names/shapes read back from the ONNX, class list, decode
     parameters).

The input size is RECTANGULAR and matches the camera's aspect ratio on purpose.
The cobot camera streams 640x480 (4:3, see cobot_params.yaml camera_intrinsics),
so the default net is 480x640 HxW (both stride-32 aligned): the frame maps 1:1
with no letterbox padding. The G1 uses 544x960 for its 16:9 head camera — do
not copy that shape here.

Usage (from vision_workspace/, inside .venv):
  python tools/export_yoloe.py                          # classes_cobot.yaml -> models/yoloe_cobot_v1/
  python tools/export_yoloe.py --prompts "red can" "green light" "gripper"
  python tools/export_yoloe.py --imgsz 640 640          # square, if ever needed
  python tools/export_yoloe.py --name yoloe_cobot_v2 --checkpoint yoloe-11m-seg.pt
"""
import argparse
import hashlib
import shutil
import sys
from pathlib import Path

import yaml

from _paths import MODELS_DIR  # vision_workspace/models: ONNX only

HERE = Path(__file__).resolve().parent

# The YOLOE seg head emits [1, 4 + nc + MASK_COEFF, anchors]; a consumer
# decodes boxes only and skips the mask-coefficient block and proto output.
BOX_CHANNELS = 4
MASK_COEFF_CHANNELS = 32


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_classes(path: Path) -> list[str]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    classes = (data or {}).get("classes")
    if not isinstance(classes, list) or not classes:
        sys.exit(f"error: {path} must contain a non-empty 'classes:' list")
    if not all(isinstance(c, str) and c.strip() for c in classes):
        sys.exit(f"error: every entry in {path} must be a non-empty string")
    if len(set(classes)) != len(classes):
        sys.exit(f"error: duplicate class names in {path}")
    return [c.strip() for c in classes]


def tensor_shapes(onnx_path: Path) -> tuple[list[dict], list[dict]]:
    import onnx

    model = onnx.load(str(onnx_path))
    onnx.checker.check_model(model)

    def describe(values):
        out = []
        for v in values:
            dims = [d.dim_value for d in v.type.tensor_type.shape.dim]
            out.append({"name": v.name, "shape": dims})
        return out

    return describe(model.graph.input), describe(model.graph.output)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--classes", type=Path, default=HERE / "classes_cobot.yaml",
                    help="YAML with the fixed prompt vocabulary (order = class index)")
    ap.add_argument("--prompts", nargs="+", metavar="PROMPT",
                    help="select the prompts on the command line instead of "
                         "--classes (order = class index); e.g. "
                         "--prompts 'red can' 'green light'")
    ap.add_argument("--checkpoint", default="yoloe-11s-seg.pt",
                    help="YOLOE checkpoint (auto-downloaded by ultralytics)")
    ap.add_argument("--name", default="yoloe_cobot_v1",
                    help="model name -> vision_workspace/models/<name>/<name>.onnx")
    ap.add_argument("--imgsz", type=int, nargs="+", default=[480, 640],
                    metavar=("H", "W"),
                    help="input size as 'H W' (or one value for square). "
                         "Default 480 640 matches the 640x480 cobot camera "
                         "exactly (no letterbox padding). Both must be "
                         "multiples of 32.")
    ap.add_argument("--out", type=Path, default=MODELS_DIR,
                    help="output root (default: vision_workspace/models)")
    ap.add_argument("--conf-threshold", type=float, default=0.35)
    ap.add_argument("--nms-iou", type=float, default=0.5)
    args = ap.parse_args()

    if len(args.imgsz) == 1:
        args.imgsz = [args.imgsz[0], args.imgsz[0]]
    if len(args.imgsz) != 2:
        sys.exit(f"error: --imgsz takes one or two values, got {args.imgsz}")
    if any(v <= 0 or v % 32 for v in args.imgsz):
        sys.exit(f"error: --imgsz {args.imgsz} — both must be positive "
                 "multiples of 32 (the network's stride). Ultralytics would "
                 "silently round instead, and the manifest would then "
                 "disagree with the engine.")

    if args.prompts:
        classes = [c.strip() for c in args.prompts]
        if not all(classes):
            sys.exit("error: every --prompts entry must be a non-empty string")
        if len(set(classes)) != len(classes):
            sys.exit("error: duplicate prompts in --prompts")
    else:
        classes = load_classes(args.classes)
    print(f"[export] {len(classes)} prompts: {classes}")

    from ultralytics import YOLOE  # deferred: heavy import, torch download

    model = YOLOE(args.checkpoint)
    print("[export] baking class vocabulary into the head (text encoder runs once, then is discarded)")
    model.set_classes(classes, model.get_text_pe(classes))

    print("[export] exporting ONNX (opset 17, static batch 1, imgsz "
          f"{args.imgsz[0]}x{args.imgsz[1]} HxW)")
    exported = Path(model.export(
        format="onnx",
        opset=17,
        imgsz=args.imgsz,
        batch=1,
        dynamic=False,
        simplify=True,
        device="cpu",
    ))

    out_dir = args.out / args.name
    out_dir.mkdir(parents=True, exist_ok=True)
    onnx_path = out_dir / f"{args.name}.onnx"
    shutil.move(str(exported), onnx_path)

    inputs, outputs = tensor_shapes(onnx_path)
    if len(inputs) != 1:
        sys.exit(f"error: expected exactly one input tensor, got {inputs}")
    # Read the input geometry back out of the GRAPH rather than trusting the
    # argument: ultralytics silently adjusts imgsz to the model's stride, and a
    # manifest that disagreed with the engine would fail at first boot on the
    # robot instead of here.
    in_shape = inputs[0]["shape"]
    if len(in_shape) != 4 or in_shape[1] != 3:
        sys.exit(f"error: unexpected input shape {in_shape}, want [1, 3, H, W]")
    in_h, in_w = in_shape[2], in_shape[3]
    if [in_h, in_w] != args.imgsz:
        print(f"[export] NOTE: requested imgsz {args.imgsz}, graph says "
              f"[{in_h}, {in_w}] — recording the graph's.")
    det_out = outputs[0]
    expected_ch = BOX_CHANNELS + len(classes) + MASK_COEFF_CHANNELS
    if det_out["shape"][1] != expected_ch:
        sys.exit(f"error: detection head has {det_out['shape'][1]} channels, "
                 f"expected {BOX_CHANNELS}+{len(classes)}+{MASK_COEFF_CHANNELS}"
                 f"={expected_ch} — head layout drifted, update the consumer's "
                 "decoder and this check together")

    manifest = {
        "schema_version": 1,
        "name": args.name,
        "type": "detector",
        "onnx": onnx_path.name,
        "onnx_sha256": sha256_of(onnx_path),
        "input": {
            "name": inputs[0]["name"],
            "width": in_w,
            "height": in_h,
            "layout": "nchw",
            "normalize": "div255",
        },
        "output": {
            "name": det_out["name"],
            "layout": "yolo_seg",  # [1, 4+nc+32, anchors]; boxes decoded, masks skipped
            "box_channels": BOX_CHANNELS,
            "mask_coeff_channels": MASK_COEFF_CHANNELS,
            "anchors": det_out["shape"][2],
        },
        "unused_outputs": [o["name"] for o in outputs[1:]],  # mask protos
        "classes": classes,
        "letterbox": {"pad_value": 114},
        "defaults": {
            "conf_threshold": args.conf_threshold,
            "nms_iou": args.nms_iou,
        },
    }
    manifest_path = out_dir / "manifest.yaml"
    manifest_path.write_text(
        yaml.safe_dump(manifest, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )

    size_mb = onnx_path.stat().st_size / 1e6
    print(f"[export] wrote {onnx_path} ({size_mb:.1f} MB)")
    print(f"[export] wrote {manifest_path}")
    print(f"[export] input  {inputs[0]['name']} {inputs[0]['shape']}")
    for o in outputs:
        print(f"[export] output {o['name']} {o['shape']}")
    print("[export] done — model stored under vision_workspace/models/ "
          "(bind-mounted into the container at /ws/vision_workspace/models); "
          "not wired into any node yet")


if __name__ == "__main__":
    main()
