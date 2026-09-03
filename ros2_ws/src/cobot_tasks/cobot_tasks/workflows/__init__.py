"""Explicit workflow registry for task_manager.

Adding a task: write workflows/<name>.py with a Workflow subclass, then add
one instance to REGISTRY below. Order here is the dashboard order.
"""

from cobot_tasks.workflows.base import ClientSpec, ParamSpec, Workflow
from cobot_tasks.workflows.calibration import (
    AutoCalibrationWorkflow,
    AutoStartupCalibrationWorkflow,
    CalibrationWorkflow,
    StartupCalibrationWorkflow,
)
from cobot_tasks.workflows.pick_layer import PickLayerWorkflow
from cobot_tasks.workflows.vision_test import VisionTestWorkflow

REGISTRY: tuple[Workflow, ...] = (
    PickLayerWorkflow(),
    CalibrationWorkflow(),
    AutoCalibrationWorkflow(),
    StartupCalibrationWorkflow(),
    AutoStartupCalibrationWorkflow(),
    VisionTestWorkflow(),
)

WORKFLOWS: dict[str, Workflow] = {w.id: w for w in REGISTRY}

__all__ = ["ClientSpec", "ParamSpec", "Workflow", "REGISTRY", "WORKFLOWS"]


def validate_registry() -> None:
    """Fail fast at import/startup on the two mistakes that are otherwise
    silent: duplicate ids and one client key bound to two different servers."""
    ids = [w.id for w in REGISTRY]
    if len(ids) != len(set(ids)) or "" in ids:
        raise RuntimeError(f"workflow ids must be unique and non-empty: {ids}")
    seen: dict[str, tuple] = {}
    for wf in REGISTRY:
        for spec in wf.clients:
            sig = (spec.kind, spec.type, spec.name)
            if seen.setdefault(spec.key, sig) != sig:
                raise RuntimeError(
                    f"client key '{spec.key}' bound to {seen[spec.key]} and {sig} "
                    f"(workflow {wf.id})"
                )


validate_registry()
