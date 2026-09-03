"""vision_test — run a random detector over every training image on disk.

Hardware-free check of task_manager → /vision/scan → /vision/detect → ONNX
Runtime. Each image goes through the REAL Scan action with
image_source=<file> and localize=false, so no camera, depth, calibration or
robot is needed — only detector_node and localization_node have to be up.
Nothing is done with the detections beyond counting them.

No rclpy here: everything goes through the WorkflowContext.
"""

from __future__ import annotations

import random
import time
from collections import Counter
from pathlib import Path

from cobot_interfaces.action import Scan
from cobot_interfaces.msg import TaskStatus

from cobot_tasks.system_state import CAP_NONE
from cobot_tasks.workflows.base import ClientSpec, ParamSpec, Workflow

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")
SKIP_PARTS = ("labels",)   # dataset label dirs (txt only, but be explicit)


def find_images(root) -> list[Path]:
    """Every image under root, recursively, sorted for a reproducible sweep."""
    root = Path(root)
    if not root.is_dir():
        return []
    return sorted(
        p for p in root.rglob("*")
        if p.is_file()
        and p.suffix.lower() in IMAGE_SUFFIXES
        and not any(part in SKIP_PARTS for part in p.parts)
    )


class VisionTestWorkflow(Workflow):
    id = "vision_test"
    label = "Vision test sweep"
    description = (
        "Run a randomly chosen detector over every training image on disk "
        "through the real Scan/Detect actions (detect only, no localization). "
        "Hardware-free stack check; detections are counted, not used."
    )
    # No camera / calibration / robot involved: the base check_ready then only
    # gates on the /vision/scan server being up.
    requires = CAP_NONE
    params = (
        ParamSpec("vision_test_image_dir", "/ws/vision_workspace/datasets",
                  "searched recursively for .jpg/.jpeg/.png; label dirs are skipped"),
        ParamSpec("vision_test_model_ids", ["cans", "green_light"],
                  "models.yaml ids drawn at random, one per image"),
        ParamSpec("vision_test_seed", 0, "seed for the model draw; 0 = nondeterministic"),
        ParamSpec("vision_test_scan_timeout_sec", 90.0,
                  "budget per image; the first goal per model lazy-loads its .onnx"),
    )
    clients = (ClientSpec("scan", "action", Scan, "/vision/scan"),)

    def run(self, ctx) -> None:
        image_dir = str(ctx.get_param("vision_test_image_dir"))
        model_ids = [str(m) for m in ctx.get_param("vision_test_model_ids") if str(m)]
        seed = int(ctx.get_param("vision_test_seed") or 0)
        timeout = float(ctx.get_param("vision_test_scan_timeout_sec"))

        if not model_ids:
            ctx.publish_status(TaskStatus.STATE_ERROR, "vision_test_model_ids is empty")
            return
        images = find_images(image_dir)
        if not images:
            ctx.publish_status(TaskStatus.STATE_ERROR, f"no images under {image_dir}")
            return

        rng = random.Random(seed) if seed else random.Random()
        total = len(images)
        attempts: Counter[str] = Counter()
        detections = 0
        failures = 0
        started = time.monotonic()
        ctx.publish_status(
            TaskStatus.STATE_SCANNING,
            f"sweeping {total} images with {', '.join(model_ids)}",
            0,
            total,
        )

        for index, image in enumerate(images, start=1):
            if ctx.stop_requested():
                ctx.publish_status(TaskStatus.STATE_IDLE, f"stopped at image {index - 1}/{total}")
                return

            model_id = rng.choice(model_ids)
            attempts[model_id] += 1
            goal = Scan.Goal(model_id=model_id, image_source=str(image), localize=False)
            try:
                result = ctx.run_action("scan", goal, timeout)
                problem = None if result.success else result.message
            except TimeoutError as e:
                problem = str(e)

            if ctx.stop_requested():
                ctx.publish_status(TaskStatus.STATE_IDLE, f"stopped at image {index}/{total}")
                return

            if problem is not None:
                # One bad image/model must not end the sweep — that is the
                # signal this workflow exists to surface.
                failures += 1
                ctx.log.warn(f"vision_test {index}/{total} {image.name} ({model_id}): {problem}")
                detail = f"{image.name} → {model_id}: FAILED ({problem})"
            else:
                count = int(result.detection_count)
                detections += count
                detail = f"{image.name} → {model_id}: {count} detections"
            ctx.publish_status(TaskStatus.STATE_SCANNING, detail, index, total)

        elapsed = time.monotonic() - started
        per_model = ", ".join(f"{m}×{n}" for m, n in sorted(attempts.items()))
        summary = (
            f"sweep done: {total} images, {detections} detections, "
            f"{failures} failed, {elapsed:.0f}s ({per_model})"
        )
        state = TaskStatus.STATE_ERROR if failures == total else TaskStatus.STATE_IDLE
        ctx.publish_status(state, summary)
