from pathlib import Path
import time

from cobot.app.vision_system.yolo_obb_detector import YoloOBBDetector


MODEL_PATH = Path("/app/vision_workspace/models/cans_obb_best.pt")

INBOX_DIR = Path("/app/vision_workspace/operation/inbox")
RESULTS_DIR = Path("/app/vision_workspace/operation/results")
DEBUG_IMAGES_DIR = Path("/app/vision_workspace/operation/debug_images")

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
    """
    Keeps only the newest N images in the folder.
    Deletes older images.
    """
    images = get_image_files(folder)

    if len(images) <= keep_latest:
        return

    images_to_delete = images[: len(images) - keep_latest]

    for image in images_to_delete:
        print(f"Deleting old operation image: {image}")
        image.unlink(missing_ok=True)


def process_operation_folder():
    detector = YoloOBBDetector(
        model_path=MODEL_PATH,
        conf=0.05,
        imgsz=640,
    )

    INBOX_DIR.mkdir(parents=True, exist_ok=True)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    DEBUG_IMAGES_DIR.mkdir(parents=True, exist_ok=True)

    images = get_image_files(INBOX_DIR)

    if not images:
        print(f"No operation images found in: {INBOX_DIR}")
        return

    for image_path in images:
        print(f"Scanning: {image_path.name}")

        detections = detector.detect(image_path)

        result_json_path = RESULTS_DIR / f"{image_path.stem}.json"
        detector.save_detections_json(detections, result_json_path)

        print(f"Detections: {len(detections)}")
        print(f"Saved result JSON: {result_json_path}")

    cleanup_keep_latest(INBOX_DIR, KEEP_LATEST_OPERATION_IMAGES)


if __name__ == "__main__":
    process_operation_folder()