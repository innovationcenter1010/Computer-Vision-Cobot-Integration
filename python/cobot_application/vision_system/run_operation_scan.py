from pathlib import Path

import cv2
import numpy as np

from python.cobot_application.shared_data import Config
from cobot_application.vision_system.homography import HomographyTranslator
from cobot_application.vision_system.rotation import calculate_real_rz
from cobot_application.vision_system.train_yolo_obb import train_yolo_obb
from cobot_application.vision_system.pathing import(
    order_obb_detections_by_real_position,
    print_obb_order,
)
from cobot_application.vision_system.yolo_obb_detector import (
    YoloOBBDetector,
    OBBDetection,
    remove_duplicate_detections,
)


# =========================================================
# Hardcoded project paths
# =========================================================

MODEL_PATH = Path("/app/vision_workspace/models/cans_obb_best.pt")
INBOX_DIR = Path("/app/vision_workspace/operation/inbox")
RESULTS_DIR = Path("/app/vision_workspace/operation/results")


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
            path
            for path in folder.iterdir()
            if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        ],
        key=lambda path: path.stat().st_mtime,
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
        print(f"No annotated image created for: {image_path.name}", flush=True)
        return

    annotated_image = results[0].plot()
    cv2.imwrite(str(output_path), annotated_image)

    print(f"Saved annotated image: {output_path}", flush=True)


def train_if_requested(vision_configs: dict):
    """
    Uses one config switch only:

        TRAIN_MODEL = true   -> train/retrain model
        TRAIN_MODEL = false  -> skip training and use existing model

    If TRAIN_MODEL is false and MODEL_PATH does not exist, an error is raised.
    """

    train_model = vision_configs["TRAIN_MODEL"]

    if train_model:
        print("TRAIN_MODEL is true. Training YOLO model.", flush=True)

        train_yolo_obb(
            vision_configs=vision_configs,
            force=True,
        )

        return

    print("TRAIN_MODEL is false. Skipping YOLO training.", flush=True)

    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"TRAIN_MODEL is false, but YOLO model was not found: {MODEL_PATH}. "
            f"Set TRAIN_MODEL to true in config.JSON to train a model first."
        )


def scan_operation_folder(
    image: None | np.ndarray,
    vision_configs: dict,
) -> list[OBBDetection]:
    """
    Scans operation images using YOLO OBB.

    This function:
      1. Optionally trains the model based on TRAIN_MODEL.
      2. Loads the saved YOLO model.
      3. Scans images from INBOX_DIR.
      4. Converts detected image centers to real-world X/Y using homography.
      5. Saves JSON and annotated images.
      6. Returns a list of OBBDetection objects.
    """

    conf = vision_configs["YOLO_CONFIDENCE"]
    iou = vision_configs["YOLO_IOU"]
    max_det = vision_configs["YOLO_MAX_DETECTIONS"]
    imgsz = vision_configs["YOLO_IMAGE_SIZE"]

    remove_duplicates = vision_configs["REMOVE_DUPLICATE_DETECTIONS"]
    duplicate_center_distance_px = vision_configs["DUPLICATE_CENTER_DISTANCE_PX"]

    save_annotated_images = vision_configs["SAVE_ANNOTATED_IMAGES"]
    annotated_image_suffix = vision_configs["ANNOTATED_IMAGE_SUFFIX"]

    keep_latest_operation_images = vision_configs["KEEP_LATEST_OPERATION_IMAGES"]

    bin_configs = vision_configs["bin_configs"]

    INBOX_DIR.mkdir(parents=True, exist_ok=True)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    train_if_requested(vision_configs)

    detector = YoloOBBDetector(
        model_path=MODEL_PATH,
        conf=conf,
        imgsz=imgsz,
        iou=iou,
        max_det=max_det,
    )

    translator = HomographyTranslator(
        bin_configs["image_corners"],
        bin_configs["real_world_corners_mm"],
    )

    if image is None: # case image is none, asume image in inbox path

        images = get_image_files(INBOX_DIR)

        if not images:
            print(f"No operation images found in: {INBOX_DIR}", flush=True)
            return []
        
    else:# case image is a numpy array
        images = [image]

    all_detections: list[OBBDetection] = []

    for image_path in images:
        print(f"Scanning: {image_path.name}", flush=True)

        detections = detector.detect(
            image=image_path,
            save_visual=False,
        )

        if remove_duplicates:
            detections = remove_duplicate_detections(
                detections,
                min_center_distance_px=duplicate_center_distance_px,
            )

        for detection in detections:
            real_x, real_y = translator.translate_point(
                detection.center_x,
                detection.center_y,
            )

            detection.real_x = float(real_x)
            detection.real_y = float(real_y)

            pixel_TL = detection.pixel_corners[0]
            pixel_BL = detection.pixel_corners[3]

            real_TL_x, real_TL_y = translator.translate_point(
                pixel_TL[0],
                pixel_TL[1],
            )

            real_BL_x, real_BL_y = translator.translate_point(
                pixel_BL[0],
                pixel_BL[1],
            )

            detection.real_TL = [
                float(real_TL_x),
                float(real_TL_y),
            ]

            detection.real_BL = [
                float(real_BL_x),
                float(real_BL_y),
            ]

            detection.real_rz = float(
                calculate_real_rz(
                    real_TL=detection.real_TL,
                    real_BL=detection.real_BL,
                    #real_world_corners_mm=bin_configs["real_world_corners_mm"],
                )
            )

        detections = order_obb_detections_by_real_position(
             detections=detections,
             real_world_corners_mm=bin_configs["real_world_corners_mm"],
             row_tolerance_mm=bin_configs["pathing_row_tolerance_mm"],
        )

        print_obb_order(
            detections=detections,
            real_world_corners_mm=bin_configs["real_world_corners_mm"],
        )

        output_json = RESULTS_DIR / f"{image_path.stem}.json"
        detector.save_json(detections, output_json)

        if save_annotated_images:
            output_image = RESULTS_DIR / f"{image_path.stem}{annotated_image_suffix}.jpg"

            save_annotated_image(
                detector=detector,
                image_path=image_path,
                output_path=output_image,
            )

        print(f"Detections: {len(detections)}", flush=True)
        print(f"Saved JSON: {output_json}", flush=True)

        all_detections.extend(detections)

    cleanup_keep_latest(
        folder=INBOX_DIR,
        keep_latest=keep_latest_operation_images,
    )

    return all_detections


def run_YOLO(
    image: np.ndarray | None,
    vision_configs: dict,
    ) -> list[OBBDetection]:
    """
    Main YOLO function called by Vision_Controller.

    Vision_Controller should call:

        detections = run_YOLO(self.configurations.as_dict()["vision_system"])

    Returns:
        list of OBBDetection can objects.
    """

    detections = scan_operation_folder(None,vision_configs)

    print(f"Total YOLO detections returned: {len(detections)}", flush=True)

    return detections


def main():
    """
    Allows this file to be run directly for testing:

        python3 -m cobot_application.vision_system.run_operation_scan
    """

    configurations = Config()
    vision_configs = configurations.as_dict()["vision_system"]

    detections = run_YOLO(vision_configs)

    print(f"Direct run complete. Detections: {len(detections)}", flush=True)


if __name__ == "__main__":
    main()