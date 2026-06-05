from pathlib import Path
import shutil

from ultralytics import YOLO

from cobot_application.shared_data import Config


def train_yolo_obb(
    cobot_configs: dict,
    force: bool | None = None,
) -> Path:
    """
    Trains the YOLO OBB model using values from config.JSON.

    Training is skipped if:
        - the final model already exists
        - and force is False

    Training runs if:
        - force is True
        - or the final model does not exist

    Returns:
        Path to the final saved model.
    """

    data_yaml = Path(cobot_configs["yolo_dataset_yaml"])
    runs_dir = Path(cobot_configs["yolo_training_runs_dir"])
    run_name = cobot_configs["yolo_training_run_name"]
    final_model_path = Path(cobot_configs["yolo_model_path"])

    base_model = cobot_configs["yolo_base_model"]

    epochs = cobot_configs["training_epochs"]
    imgsz = cobot_configs["training_image_size"]
    batch = cobot_configs["training_batch_size"]
    device = cobot_configs["training_device"]

    if force is None:
        force = cobot_configs["force_retrain_yolo_model"]

    if final_model_path.exists() and not force:
        print(
            f"Model already exists, skipping training: {final_model_path}",
            flush=True,
        )
        return final_model_path

    if not data_yaml.exists():
        raise FileNotFoundError(f"Dataset YAML not found: {data_yaml}")

    print("Starting YOLO OBB training...", flush=True)
    print(f"Base model: {base_model}", flush=True)
    print(f"Dataset: {data_yaml}", flush=True)
    print(f"Epochs: {epochs}", flush=True)
    print(f"Image size: {imgsz}", flush=True)
    print(f"Batch: {batch}", flush=True)
    print(f"Device: {device}", flush=True)
    print(f"Training runs directory: {runs_dir}", flush=True)
    print(f"Training run name: {run_name}", flush=True)
    print(f"Final model path: {final_model_path}", flush=True)

    model = YOLO(base_model)

    model.train(
        data=str(data_yaml),
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device=device,
        project=str(runs_dir),
        name=run_name,
        exist_ok=True,
    )

    best_model = runs_dir / run_name / "weights" / "best.pt"

    if not best_model.exists():
        raise FileNotFoundError(
            f"Training finished, but best.pt was not found: {best_model}"
        )

    final_model_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best_model, final_model_path)

    print("Training complete.", flush=True)
    print(f"Best model copied to: {final_model_path}", flush=True)

    return final_model_path


def main():
    """
    Allows this file to be run directly:

        python3 -m cobot_application.vision_system.train_yolo_obb
    """

    configurations = Config()
    cobot_configs = configurations.as_dict()["cobot_configs"]

    train_yolo_obb(
        cobot_configs=cobot_configs,
        force=True,
    )


if __name__ == "__main__":
    main()
