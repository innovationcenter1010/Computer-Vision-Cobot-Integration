from pathlib import Path
import cv2
from ultralytics import YOLO

from cobot.app.vision_system import config
from cobot.app.vision_system.homography import HomographyTranslator
from cobot.app.vision_system.train_yolo_obb import train_yolo_obb
from cobot.app.vision_system.yolo_obb_detector import (
    YoloOBBDetector,
    remove_duplicate_detections,
)

"""
Select which model version will be run
"""

MODEL_PATH = Path("/app/vision_workspace/models/cans_obb_best.pt")

INBOX_DIR = Path("/app/vision_workspace/operation/inbox")
RESULTS_DIR = Path("/app/vision_workspace/operation/results")

KEEP_LATEST_OPERATION_IMAGES = 10

IMAGE_EXTENSIONS = {
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp",
    ".tif",
    ".tiff",
    ".webp",
}


def get_image_files(folder: Path) -> list[Path]:
    if not folder.exists():
        return []

    return sorted(
        [
            p for p in folder.iterdir()
            if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS
        ],
        key=lambda p: p.stat().st_mtime,
    )


def cleanup_keep_latest(folder: Path, keep_latest: int):
    images = get_image_files(folder)

    if len(images) <= keep_latest:
        return

    images_to_delete = images[: len(images) - keep_latest]

    for image in images_to_delete:
        print(f"Deleting old operation image: {image}", flush=True)
        image.unlink(missing_ok=True)

def save_annotated_image(
        image_path: Path,
        model_path: Path,
        output_path: Path,
        conf: float,
        iou: float,
        max_det: int,
):
    
    model = YOLO(str(model_path))

    results = model.predict(
        source=str(image_path),
        imgsz=640,
        conf=conf,
        iou=iou,
        max_det=max_det,
        save=True,
        verbose=False,
    )

    if not results:
        print(f"No visual results created for: {image_path.name}", flush=True)
        return

    annotated_image = results[0].plot()
    cv2.imwrite(str(output_path), annotated_image)

    print(f"Saved image: {output_path}", flush=True)


def scan_operation_folder(
    retrain_if_missing: bool = False,
    force_retrain: bool = False,
    conf: float = 0.85,
    iou: float = 0.4,
    max_det: int = 30,
    remove_duplicates: bool = True,
) -> list:

    INBOX_DIR.mkdir(parents=True, exist_ok=True)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    if force_retrain:
        print("Force retrain requested.", flush=True)
        train_yolo_obb(force=True)

    elif retrain_if_missing and not MODEL_PATH.exists():
        print("Model missing. Training model first.", flush=True)
        train_yolo_obb(force=False)

    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"Model not found: {MODEL_PATH}. "
            f"Train the model first or call scan_operation_folder(retrain_if_missing=True)."
        )

    detector = YoloOBBDetector(
        model_path=MODEL_PATH,
        conf=conf,
        imgsz=640,
        iou=iou,
        max_det=max_det,
    )

    translator = HomographyTranslator(
        config.BIN_IMAGE_CORNERS,
        config.BIN_REAL_WORLD_CORNERS,
    )

    images = get_image_files(INBOX_DIR)

    if not images:
        print(f"No operation images found in: {INBOX_DIR}", flush=True)
        return []

    all_detections = []

    for image_path in images:
        print(f"Scanning: {image_path.name}", flush=True)

        detections = detector.detect(
            image_path=image_path,
            save_visual=False,
        )

        if remove_duplicates:
            detections = remove_duplicate_detections(
                detections,
                min_center_distance_px=50,
            )

        for det in detections:
            real_x, real_y = translator.translate_point(det.center_x, det.center_y)
            det.real_x = float(real_x)
            det.real_y = float(real_y)

        output_json = RESULTS_DIR / f"{image_path.stem}.json"
        detector.save_json(detections, output_json)

        output_image = RESULTS_DIR / f"{image_path.stem}_detections.jpg"
        save_annotated_image(
            image_path=image_path,
            model_path=MODEL_PATH,
            output_path=output_image,
            conf=conf,
            iou=iou,
            max_det=max_det,
        )

        print(f"Detections: {len(detections)}", flush=True)
        print(f"Saved JSON: {output_json}", flush=True)

        all_detections.extend(detections)

    cleanup_keep_latest(INBOX_DIR, KEEP_LATEST_OPERATION_IMAGES)

    return all_detections


def run_YOLO():
    scan_operation_folder(
        retrain_if_missing=True,
        force_retrain=False,
        conf=0.85,
        iou=0.4,
        max_det=30,
        remove_duplicates=True,
    )