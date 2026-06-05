from cobot.app.vision_system.run_operation_scan import scan_operation_folder
from cobot.app.vision_system.train_yolo_obb import train_yolo_obb


# =========================================================
# User settings
# =========================================================

SHOULD_RETRAIN_MODEL = False

TRAINING_EPOCHS = 50
TRAINING_IMAGE_SIZE = 640
TRAINING_BATCH_SIZE = 1
TRAINING_DEVICE = "cpu"

YOLO_CONFIDENCE = 0.85
YOLO_IOU = 0.4
YOLO_MAX_DETECTIONS = 30


def run_yolo():
    """
    Runs the YOLO OBB vision pipeline.

    Training is available from this main program, but it only happens
    when SHOULD_RETRAIN_MODEL is set to True.
    """

    if SHOULD_RETRAIN_MODEL:
        print("Retraining YOLO model because SHOULD_RETRAIN_MODEL is True.")

        train_yolo_obb(
            force=True,
            epochs=TRAINING_EPOCHS,
            imgsz=TRAINING_IMAGE_SIZE,
            batch=TRAINING_BATCH_SIZE,
            device=TRAINING_DEVICE,
        )
    else:
        print("Skipping YOLO training because SHOULD_RETRAIN_MODEL is False.", flush=True)

    detections = scan_operation_folder(
        retrain_if_missing=True,
        force_retrain=False,
        conf=YOLO_CONFIDENCE,
        iou=YOLO_IOU,
        max_det=YOLO_MAX_DETECTIONS,
        remove_duplicates=True,
    )

    print(f"Total detections: {len(detections)}", flush=True)

    for det in detections:
        print(
            f"Can: image=({det.center_x:.2f}, {det.center_y:.2f}) "
            f"real=({det.real_x:.2f}, {det.real_y:.2f}) "
            f"confidence={det.confidence:.3f}"
        )

    return detections


def main():
    """
    Main entry point for the cobot application.
    """

    run_yolo()


if __name__ == "__main__":
    main()


    