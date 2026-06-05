from pathlib import Path
import shutil
from ultralytics import YOLO


DATA_YAML = Path("/app/vision_workspace/datasets/cans_obb/can_obb.yaml")
RUNS_DIR = Path("/app/vision_workspace/models/training_runs")
RUN_NAME = "cans_obb"
FINAL_MODEL_PATH = Path("/app/vision_workspace/models/cans_obb_best.pt")


def train_yolo_obb(
    force: bool = False,
    epochs: int = 50,
    imgsz: int = 640,
    batch: int = 1,
    device: str = "cpu",
) -> Path:
    """
    Trains the YOLO OBB model only when needed.

    If force=False and the final model already exists, training is skipped.
    If force=True, training runs again and overwrites cans_obb_best.pt.
    """

    if FINAL_MODEL_PATH.exists() and not force:
        print(f"Model already exists, skipping training: {FINAL_MODEL_PATH}")
        return FINAL_MODEL_PATH

    if not DATA_YAML.exists():
        raise FileNotFoundError(f"Dataset YAML not found: {DATA_YAML}")

    print("Starting YOLO OBB training...")
    print(f"Dataset: {DATA_YAML}")
    print(f"Epochs: {epochs}")
    print(f"Image size: {imgsz}")
    print(f"Batch: {batch}")
    print(f"Device: {device}")

    model = YOLO("yolo26n-obb.pt")

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
        raise FileNotFoundError(f"Training finished, but best.pt was not found: {best_model}")

    FINAL_MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best_model, FINAL_MODEL_PATH)

    print("Training complete.")
    print(f"Best model copied to: {FINAL_MODEL_PATH}")

    return FINAL_MODEL_PATH


def run_YOLO():
    """
    Allows this file to be run directly.
    """

    train_yolo_obb(
        force=True,
        epochs=50,
        imgsz=640,
        batch=1,
        device="cpu",
    )


