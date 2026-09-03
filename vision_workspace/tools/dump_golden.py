#!/usr/bin/env python3
"""Produce golden detections for cobot_detector's test_golden.cpp.

Runs the SAME ultralytics predict() the retired yolo_node used (so the C++
ONNX path is measured against the behaviour the robot was picking with) and
writes one text file per frame per model:

    <out>/<frame>.png
    <out>/<frame>.<model_id>.txt      # class conf cx cy x1 y1 x2 y2 x3 y3 x4 y4

Run from vision_workspace/ inside .venv (needs ultralytics — training env only):

    python tools/dump_golden.py operation/captures/*.png \
        --model cans=weights/cans_obb_best.pt:obb:0.5:0.4:20 \
        --model red_dot=weights/red_dot_best.pt:bbox:0.5:0.4:5 \
        --out ../ros2_ws/src/cobot_detector/test/data/golden

The thresholds after the path must match models.yaml for the comparison to
be meaningful. imgsz must match the exported ONNX input (default 480 640).
"""
import argparse
import math
import shutil
import sys
from pathlib import Path


def parse_model(arg: str):
    name, rest = arg.split("=", 1)
    parts = rest.split(":")
    if len(parts) != 5:
        sys.exit(f"--model {arg}: want id=path:obb|bbox:conf:iou:max_det")
    path, kind, conf, iou, max_det = parts
    return name, Path(path), kind, float(conf), float(iou), int(max_det)


def remove_duplicates(rows, min_dist):
    rows = sorted(rows, key=lambda r: r[1], reverse=True)
    kept = []
    for r in rows:
        if all(math.hypot(r[2] - k[2], r[3] - k[3]) >= min_dist for k in kept):
            kept.append(r)
    return kept


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames", nargs="+", help="image files")
    ap.add_argument("--model", action="append", required=True,
                    help="id=path.pt:obb|bbox:conf:iou:max_det (repeatable)")
    ap.add_argument("--imgsz", type=int, nargs="+", default=[480, 640])
    ap.add_argument("--dup-distance", type=float, default=50.0,
                    help="duplicate_center_distance_px applied to obb models (0 = off)")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    from ultralytics import YOLO  # heavy import, training env only
    import cv2

    args.out.mkdir(parents=True, exist_ok=True)
    models = [parse_model(m) for m in args.model]
    loaded = {mid: YOLO(str(p)) for mid, p, *_ in models}

    for frame in args.frames:
        src = Path(frame)
        bgr = cv2.imread(str(src))
        if bgr is None:
            print(f"skip unreadable {src}")
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)  # the node fed rgb8 frames
        shutil.copy(src, args.out / (src.stem + ".png")) if src.suffix == ".png" \
            else cv2.imwrite(str(args.out / (src.stem + ".png")), bgr)

        for mid, _, kind, conf, iou, max_det in models:
            res = loaded[mid].predict(source=rgb, imgsz=args.imgsz, conf=conf, iou=iou,
                                      max_det=max_det, verbose=False)[0]
            rows = []
            if kind == "obb" and res.obb is not None and len(res.obb):
                corners = res.obb.xyxyxyxy.cpu().numpy()
                for box, c, s in zip(corners, res.obb.cls.cpu().numpy(),
                                     res.obb.conf.cpu().numpy()):
                    cx, cy = float(box[:, 0].mean()), float(box[:, 1].mean())
                    rows.append([int(c), float(s), cx, cy] + [float(v) for pt in box for v in pt])
                if args.dup_distance > 0:
                    rows = remove_duplicates(rows, args.dup_distance)
            elif kind == "bbox" and res.boxes is not None and len(res.boxes):
                for (x1, y1, x2, y2), c, s in zip(res.boxes.xyxy.cpu().numpy(),
                                                  res.boxes.cls.cpu().numpy(),
                                                  res.boxes.conf.cpu().numpy()):
                    rows.append([int(c), float(s), (x1 + x2) / 2, (y1 + y2) / 2,
                                 x1, y1, x2, y1, x2, y2, x1, y2])
            out = args.out / f"{src.stem}.{mid}.txt"
            out.write_text("# class conf cx cy x1 y1 x2 y2 x3 y3 x4 y4\n" +
                           "".join(" ".join(f"{v:.4f}" if isinstance(v, float) else str(v)
                                            for v in r) + "\n" for r in rows))
            print(f"{src.name} {mid}: {len(rows)} detections -> {out.name}")


if __name__ == "__main__":
    main()
