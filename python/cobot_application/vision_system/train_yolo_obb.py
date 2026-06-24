from pathlib import Path
import shutil

from ultralytics import YOLO

from python.cobot_application.shared_data import Config


# =========================================================
# Hardcoded project paths
# =========================================================

DATA_YAML = Path("/app/vision_workspace/datasets/cans_obb/can_obb.yaml")
RUNS_DIR = Path("/app/vision_workspace/models/training_runs")
RUN_NAME = "cans_obb"
FINAL_MODEL_PATH = Path("/app/vision_workspace/models/cans_obb_best.pt")


def train_yolo_obb(
    vision_configs: dict,
    force: bool = False,
) -> Path:
    """
    Trains the YOLO OBB model.

    Training settings come from config.JSON:
      - YOLO_BASE_MODEL
      - TRAINING_EPOCHS
      - TRAINING_IMAGE_SIZE
      - TRAINING_BATCH_SIZE
      - TRAINING_DEVICE

    Project paths are hardcoded in this file.

    If force=False and FINAL_MODEL_PATH already exists, training is skipped.
    If force=True, training runs and overwrites FINAL_MODEL_PATH.
    """

    if FINAL_MODEL_PATH.exists() and not force:
        print(f"Model already exists, skipping training: {FINAL_MODEL_PATH}", flush=True)
        return FINAL_MODEL_PATH

    if not DATA_YAML.exists():
        raise FileNotFoundError(f"Dataset YAML not found: {DATA_YAML}")

    base_model = vision_configs["YOLO_BASE_MODEL"]
    epochs = vision_configs["TRAINING_EPOCHS"]
    imgsz = vision_configs["TRAINING_IMAGE_SIZE"]
    batch = vision_configs["TRAINING_BATCH_SIZE"]
    device = vision_configs["TRAINING_DEVICE"]

    print("Starting YOLO OBB training...", flush=True)
    print(f"Base model: {base_model}", flush=True)
    print(f"Dataset YAML: {DATA_YAML}", flush=True)
    print(f"Epochs: {epochs}", flush=True)
    print(f"Image size: {imgsz}", flush=True)
    print(f"Batch: {batch}", flush=True)
    print(f"Device: {device}", flush=True)
    print(f"Training run directory: {RUNS_DIR / RUN_NAME}", flush=True)
    print(f"Final model path: {FINAL_MODEL_PATH}", flush=True)

    model = YOLO(base_model)

    model.train(
        data=str(DATA_YAML),
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device=device,
        project=str(RUNS_DIR),
        name=RUN_NAME,
        exist_ok=True,
    )

    best_model = RUNS_DIR / RUN_NAME / "weights" / "best.pt"

    if not best_model.exists():
        raise FileNotFoundError(
            f"Training finished, but best.pt was not found: {best_model}"
        )

    FINAL_MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best_model, FINAL_MODEL_PATH)

    print("Training complete.", flush=True)
    print(f"Best model copied to: {FINAL_MODEL_PATH}", flush=True)

    return FINAL_MODEL_PATH


def main():
    """
    Allows this file to be run directly for testing/training:

        python3 -m cobot_application.vision_system.train_yolo_obb
    """

    configurations = Config()
    vision_configs = configurations.as_dict()["vision_system"]

    train_yolo_obb(
        vision_configs=vision_configs,
        force=True,
    )


if __name__ == "__main__":
    main()