"""Workspace-relative paths shared by the vision_workspace tools.

Everything resolves from this file's location, so the tools run unchanged from
any cwd. Nothing here is imported by the ROS nodes.
"""
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
VISION_WS = TOOLS_DIR.parent                     # vision_workspace/
DATASETS_DIR = VISION_WS / "datasets"            # labelled training data
WEIGHTS_DIR = VISION_WS / "weights"              # trained .pt (gitignored)
BASE_WEIGHTS_DIR = WEIGHTS_DIR / "base"          # pretrained yolo26*.pt starting points
TRAINING_RUNS_DIR = VISION_WS / "training_runs"  # ultralytics run output (gitignored)
MODELS_DIR = VISION_WS / "models"                # ONNX exports ONLY — what the container consumes


def pick_device(requested: str = "auto") -> str:
    """'auto' -> CUDA 0 if available else cpu; anything else is passed through."""
    if requested != "auto":
        return requested
    try:
        import torch  # ultralytics pulls this in
        return "0" if torch.cuda.is_available() else "cpu"
    except Exception:  # noqa: BLE001 — torch missing/broken -> just use CPU
        return "cpu"


def resolve_base_model(name: str) -> str:
    """Prefer a checkpoint already in weights/base/; otherwise hand the bare
    name to ultralytics, which downloads it into the cwd on first use."""
    local = BASE_WEIGHTS_DIR / name
    return str(local) if local.exists() else name
