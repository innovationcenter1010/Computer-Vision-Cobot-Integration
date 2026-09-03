"""WorkflowContext — the only surface a workflow sees of task_manager.

Wraps the node's blocking call helpers, status publisher and stop flag so a
workflow never touches rclpy. Runs on the node's worker thread; action and
service futures are resolved by executor threads and bridged with events.
While an action goal is in flight a stop request cancels it natively, so
stop interrupts a pick mid-choreography instead of waiting for the next can.

Tests replace this with a plain fake (see test/test_workflows.py).
"""

from __future__ import annotations

import threading
import time

from cobot_interfaces.msg import TaskStatus

from cobot_tasks.system_state import CAP_MOVE, CAP_SEE

GOAL_ACK_TIMEOUT_SEC = 10.0     # server accepting/rejecting the goal

TERMINAL_STATES = (TaskStatus.STATE_IDLE, TaskStatus.STATE_ERROR, TaskStatus.STATE_DISCONNECTED)


class WorkflowContext:
    def __init__(self, node, workflow):
        self._node = node
        self._workflow = workflow
        self.log = node.get_logger()

    # ---- state / config -------------------------------------------------

    def publish_status(self, state: int, detail: str = "", current: int = 0, total: int = 0):
        active = "" if state in TERMINAL_STATES else self._workflow.id
        self._node._publish_status(state, detail, active, current, total)

    def get_param(self, name: str):
        return self._node.get_parameter(name).value

    def capability_state(self, required: int) -> tuple[int, list[str]]:
        """Current derived capability bitmask plus the blockers for whichever
        required bits are missing. Read-only view of the node's snapshot
        (see system_state.py) — a workflow can only observe it, never set it."""
        snap = self._node._system_snapshot
        blockers: list[str] = []
        if (required & CAP_SEE) and not (snap.capability & CAP_SEE):
            blockers += list(snap.see_blockers)
        if (required & CAP_MOVE) and not (snap.capability & CAP_MOVE):
            blockers += list(snap.move_blockers)
        return snap.capability, blockers

    def stop_requested(self) -> bool:
        return self._node._stop_requested.is_set()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def now_msg(self):
        return self._node.get_clock().now().to_msg()

    # ---- outputs --------------------------------------------------------

    def publish_layer_report(self, report) -> None:
        report.header.stamp = self.now_msg()
        self._node._report_pub.publish(report)

    def publish_pick_event(self, event) -> None:
        event.header.stamp = self.now_msg()
        self._node._pick_event_pub.publish(event)

    # ---- clients --------------------------------------------------------

    def _spec(self, key: str):
        try:
            return self._node._client_specs[key]
        except KeyError:
            raise KeyError(f"workflow {self._workflow.id} uses undeclared client '{key}'") from None

    def client_ready(self, key: str) -> bool:
        spec = self._spec(key)
        client = self._node._wf_clients[key]
        return client.server_is_ready() if spec.kind == "action" else client.service_is_ready()

    def call(self, key: str, request, timeout: float):
        """Blocking service call from the worker thread."""
        spec = self._spec(key)
        client = self._node._wf_clients[key]
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout):
            future.cancel()
            raise TimeoutError(f"{spec.name} timed out after {timeout:.0f}s")
        return future.result()

    def run_action(self, key: str, goal, timeout: float):
        """Send one goal and block for its result. While waiting, a stop
        request cancels the goal natively (the server stops at its next safe
        boundary). Returns the action result, or a synthesized failure
        result if the goal was rejected."""
        spec = self._spec(key)
        client = self._node._wf_clients[key]

        goal_future = client.send_goal_async(goal)
        acked = threading.Event()
        goal_future.add_done_callback(lambda _: acked.set())
        if not acked.wait(timeout=GOAL_ACK_TIMEOUT_SEC):
            raise TimeoutError(f"{spec.name} goal was never acknowledged")

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            result = spec.type.Result()
            result.success = False
            result.message = f"goal rejected by {spec.name} server (busy, not ready or disconnected)"
            return result

        result_future = goal_handle.get_result_async()
        done = threading.Event()
        result_future.add_done_callback(lambda _: done.set())

        cancel_sent = False
        deadline = time.monotonic() + timeout
        while not done.wait(timeout=0.2):
            if self.stop_requested() and not cancel_sent:
                self.log.info(f"stop → canceling {spec.name} goal")
                goal_handle.cancel_goal_async()
                cancel_sent = True
            if time.monotonic() > deadline:
                if not cancel_sent:
                    goal_handle.cancel_goal_async()
                raise TimeoutError(f"{spec.name} timed out after {timeout:.0f}s")

        return result_future.result().result
