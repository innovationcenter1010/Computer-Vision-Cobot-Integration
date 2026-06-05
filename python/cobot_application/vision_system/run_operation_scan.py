from pathlib import Path
import cv2

from cobot_application.shared_data import Config
from cobot_application.vision_system import config as vision_config
from cobot_application.vision_system.homography import HomographyTranslator
from cobot_application.vision_system.train_yolo_obb import train_yolo_obb
from cobot_application.vision_system.yolo_obb_detector import (
    YoloOBBDetector,
    OBBDetection,
    remove_duplicate_detections,
)


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
    """
    Returns all image files in a folder, sorted oldest to newest.
    """

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
    Keeps only the newest operation images in the inbox folder.
    Older images are deleted.
    """

    images = get_image_files(folder)

    if len(images) <= keep_latest:
        return

    images_to_delete = images[: len(images) - keep_latest]

    for image in images_to_delete:
        print(f"Deleting old operation image: {image}", flush=True)
        image.unlink(missing_ok=True)


def save_annotated_image(
    detector: YoloOBBDetector,
    image_path: Path,
    output_path: Path,
):
    """
    Saves an annotated image showing YOLO detections.

    This uses the same loaded detector/model instead of loading YOLO again.
    """

    output_path.parent.mkdir(parents=True, exist_ok=True)

    results = detector.model.predict(
        source=str(image_path),
        imgsz=detector.imgsz,
        conf=detector.conf,
        iou=detector.iou,
        max_det=detector.max_det,
        save=False,
        verbose=False,
    )

    if not results:
        print(f"No visual results created for: {image_path.name}", flush=True)
        return

    annotated_image = results[0].plot()
    cv2.imwrite(str(output_path), annotated_image)

    print(f"Saved image: {output_path}", flush=True)


def train_if_requested(cobot_configs: dict, model_path: Path):
    """
    Handles optional YOLO training based on config.JSON.

    Training happens only if:
    - should_retrain_yolo_model is true, or
    - force_retrain_yolo_model is true, or
    - train_yolo_if_model_missing is true and the model file is missing.
    """

    should_retrain = cobot_configs.get("should_retrain_yolo_model", False)
    force_retrain = cobot_configs.get("force_retrain_yolo_model", False)
    train_if_missing = cobot_configs.get("train_yolo_if_model_missing", True)

    model_missing = not model_path.exists()

    if should_retrain or force_retrain:
        print("YOLO retraining requested from config.JSON.", flush=True)

        train_yolo_obb(
            force=True,
            epochs=cobot_configs.get("training_epochs", 50),
            imgsz=cobot_configs.get("training_image_size", 640),
            batch=cobot_configs.get("training_batch_size", 1),
            device=cobot_configs.get("training_device", "cpu"),
        )

        return

    if train_if_missing and model_missing:
        print("YOLO model is missing. Training model first.", flush=True)

        train_yolo_obb(
            force=True,
            epochs=cobot_configs.get("training_epochs", 50),
            imgsz=cobot_configs.get("training_image_size", 640),
            batch=cobot_configs.get("training_batch_size", 1),
            device=cobot_configs.get("training_device", "cpu"),
        )

        return

    print("YOLO training skipped. Existing model will be used.", flush=True)


def scan_operation_folder(cobot_configs: dict) -> list[OBBDetection]:
    """
    Scans the operation inbox folder using YOLO OBB.

    This function:
    1. Reads paths/settings from config.JSON via cobot_configs.
    2. Optionally trains if requested or if model is missing.
    3. Runs YOLO OBB on every image in the inbox.
    4. Converts image center points to real-world X/Y with homography.
    5. Saves JSON and optional annotated images.
    6. Returns a list of OBBDetection objects.
    """

    model_path = Path(
        cobot_configs.get(
            "yolo_model_path",
            "/app/vision_workspace/models/cans_obb_best.pt",
        )
    )

    inbox_dir = Path(
        cobot_configs.get(
            "operation_inbox_dir",
            "/app/vision_workspace/operation/inbox",
        )
    )

    results_dir = Path(
        cobot_configs.get(
            "operation_results_dir",
            "/app/vision_workspace/operation/results",
        )
    )

    keep_latest_operation_images = cobot_configs.get(
        "keep_latest_operation_images",
        10,
    )

    conf = cobot_configs.get("yolo_confidence", 0.85)
    iou = cobot_configs.get("yolo_iou", 0.4)
    max_det = cobot_configs.get("yolo_max_detections", 30)
    imgsz = cobot_configs.get("yolo_image_size", 640)

    remove_duplicates = cobot_configs.get(
        "remove_duplicate_detections",
        True,
    )

    duplicate_center_distance_px = cobot_configs.get(
        "duplicate_center_distance_px",
        50,
    )

    save_images = cobot_configs.get(
        "save_annotated_images",
        True,
    )

    annotated_suffix = cobot_configs.get(
        "annotated_image_suffix",
        "_detections",
    )

    inbox_dir.mkdir(parents=True, exist_ok=True)
    results_dir.mkdir(parents=True, exist_ok=True)

    train_if_requested(cobot_configs, model_path)

    if not model_path.exists():
        raise FileNotFoundError(
            f"Model not found: {model_path}. "
            f"Train the model first or set train_yolo_if_model_missing=true."
        )

    detector = YoloOBBDetector(
        model_path=model_path,
        conf=conf,
        imgsz=imgsz,
        iou=iou,
        max_det=max_det,
    )

    translator = HomographyTranslator(
        vision_config.BIN_IMAGE_CORNERS,
        vision_config.BIN_REAL_WORLD_CORNERS,
    )

    images = get_image_files(inbox_dir)

    if not images:
        print(f"No operation images found in: {inbox_dir}", flush=True)
        return []

    all_detections: list[OBBDetection] = []

    for image_path in images:
        print(f"Scanning: {image_path.name}", flush=True)

        detections = detector.detect(
            image_path=image_path,
            save_visual=False,
        )

        if remove_duplicates:
            detections = remove_duplicate_detections(
                detections,
                min_center_distance_px=duplicate_center_distance_px,
            )

        for det in detections:
            real_x, real_y = translator.translate_point(
                det.center_x,
                det.center_y,
            )

            det.real_x = float(real_x)
            det.real_y = float(real_y)

        output_json = results_dir / f"{image_path.stem}.json"
        detector.save_json(detections, output_json)

        if save_images:
            output_image = results_dir / f"{image_path.stem}{annotated_suffix}.jpg"

            save_annotated_image(
                detector=detector,
                image_path=image_path,
                output_path=output_image,
            )

        print(f"Detections: {len(detections)}", flush=True)
        print(f"Saved JSON: {output_json}", flush=True)

        all_detections.extend(detections)

    cleanup_keep_latest(
        inbox_dir,
        keep_latest_operation_images,
    )

    return all_detections


def run_YOLO(cobot_configs: dict) -> list[OBBDetection]:
    """
    Main callable YOLO function used by Vision_Controller.

    Vision_Controller should call:

        detections = run_YOLO(self.configurations.as_dict()["cobot_configs"])

    Returns:
        list of OBBDetection objects.
    """

    detections = scan_operation_folder(cobot_configs)

    print(f"Total YOLO detections returned: {len(detections)}", flush=True)

    return detections


def main():
    """
    Allows this file to be run directly for testing:

        python3 -m cobot_application.vision_system.run_operation_scan
    """

    configurations = Config()
    cobot_configs = configurations.as_dict()["cobot_configs"]

    detections = run_YOLO(cobot_configs)

    print(f"Direct run complete. Detections: {len(detections)}", flush=True)


if __name__ == "__main__":
    main()