from pathlib import Path
import shutil

from ultralytics import YOLO


DATA_YAML = Path("/app/vision_workspace/datasets/cans_obb/can_obb.yaml")
RUNS_DIR = Path("/app/vision_workspace/models/training_runs")
RUN_NAME = "cans_obb"
FINAL_MODEL_PATH = Path("/app/vision_workspace/models/cans_obb_best.pt")


def main():
    if not DATA_YAML.exists():
        raise FileNotFoundError(f"Dataset YAML not found: {DATA_YAML}")

    model = YOLO("yolo26n-obb.pt")

    results = model.train(
        data=str(DATA_YAML),
        epochs=50,
        imgsz=640,
        batch=1,
        device="cpu",
        project=str(RUNS_DIR),
        name=RUN_NAME,
        exist_ok=True,
    )

    best_model = RUNS_DIR / RUN_NAME / "weights" / "best.pt"

    if not best_model.exists():
        raise FileNotFoundError(f"Training finished, but best.pt was not found: {best_model}")

    FINAL_MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best_model, FINAL_MODEL_PATH)

    print(f"Training complete.")
    print(f"Best model copied to: {FINAL_MODEL_PATH}")


if __name__ == "__main__":
    main()
    