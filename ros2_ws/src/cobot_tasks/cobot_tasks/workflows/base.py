"""Workflow contract for task_manager.

A workflow is ONE task the operator can start from the dashboard (pick a
layer, calibrate the camera, ...). It declares the ROS parameters it reads
and the action/service clients it needs, and does all its sequencing through
a WorkflowContext (cobot_tasks/context.py) — never through rclpy directly.
That keeps every workflow unit-testable with a fake context and keeps the
§11 dependency rule (rclpy + cobot_interfaces only) enforceable by grep.

Register a new workflow in workflows/__init__.py (explicit list).
"""

from __future__ import annotations

from dataclasses import dataclass

from cobot_tasks.system_state import CAP_OPERATE, CAPABILITY_NAMES


@dataclass(frozen=True)
class ParamSpec:
    """A ROS parameter the workflow reads via ctx.get_param(name). Declared on
    the node under exactly this name (no prefix) so cobot_params.yaml keys
    stay stable. Workflows sharing a name must agree on the default."""

    name: str
    default: object
    description: str = ""


@dataclass(frozen=True)
class ClientSpec:
    """An action server or service the workflow talks to via ctx.run_action(key)
    / ctx.call(key). The node creates ONE client per key across all
    workflows; two specs with the same key must be identical."""

    key: str
    kind: str          # "action" | "service"
    type: type         # e.g. PickCan, Trigger
    name: str          # e.g. "/cobot/pick_can"
    required: bool = True   # gate in check_ready; False = optional (degrade)


class Workflow:
    id: str = ""              # StartWorkflow.workflow / TaskStatus.active_task
    label: str = ""           # dashboard name
    description: str = ""
    params: tuple[ParamSpec, ...] = ()
    clients: tuple[ClientSpec, ...] = ()
    # Prerequisite capability bitmask (system_state.py): CAP_SEE for a
    # vision-only workflow, CAP_MOVE for motion-only, CAP_OPERATE for both.
    # Default is the strictest gate; set it explicitly on every workflow.
    requires: int = CAP_OPERATE

    def check_ready(self, ctx) -> tuple[bool, str]:
        """Gate evaluated before the one-task lock is taken (and for the
        ready flag on /task/workflows). Default: the derived capability
        satisfies `requires`, then every required client is up. Subclasses
        extend with their own preconditions."""
        current, blockers = ctx.capability_state(self.requires)
        if (current & self.requires) != self.requires:
            why = "; ".join(blockers) or "capability unavailable"
            return False, f"requires {CAPABILITY_NAMES[self.requires]}: {why}"
        for spec in self.clients:
            if spec.required and not ctx.client_ready(spec.key):
                return False, f"{spec.name} not available"
        return True, ""

    def run(self, ctx) -> None:
        """Body, executed on the node's worker thread while it holds the
        one-task lock. Poll ctx.stop_requested() between steps; in-flight
        action goals are canceled natively by ctx.run_action. Finish by
        publishing TaskStatus IDLE or ERROR through ctx.publish_status —
        the host publishes ERROR if this raises."""
        raise NotImplementedError

    def __repr__(self) -> str:
        return f"<Workflow {self.id}>"
