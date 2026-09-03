"""task_manager_node — owns WHAT the system is doing.

The decision/sequencing layer between the web UI and the executors. It hosts
a registry of WORKFLOWS (cobot_tasks/workflows/), runs one at a time on a
worker thread, and publishes the single source of operational truth.
cobot_operation / cobot_control execute motion; the vision nodes do the
math; this node only decides, gates and reports.

  in:  /task/start_workflow (StartWorkflow)  run one workflow by id
       /task/start | /task/calibrate (Trigger)  aliases: default_workflow /
                                                "calibration"
       /task/stop (Trigger)                    cancels the active goal natively
       /cobot/status (CobotStatus)  — hardware/connection state, mirrored into
                                      /task/status when no workflow runs
       /cobot/state (CobotState, 10 Hz) + /health/summary (HealthSummary)
                                    — raw inputs for the DERIVED capability
                                      (system_state.py): none/see/move/operate.
                                      Read-only aggregation — never set here.
  out: /task/status (TaskStatus, latched) — THE operational state;
                                            active_task = workflow id
       /task/system_state (SystemState, latched) — individual robot/vision
                                      facts + the capability bitmask
       /task/workflows (WorkflowList, latched) — registry + readiness (each
                                      workflow's `requires` capability gates
                                      it), for the dashboard's selector
       /task/layer_report (LayerReport), /task/pick_event (PickEvent)
  calls: whatever the registered workflows declare (ClientSpec) — one
       action/service client per key, created here, shared by all workflows.

Adding a task: write workflows/<name>.py, add it to workflows/__init__.py
REGISTRY. Nothing in this file changes.

Dependency rule (docs/ros2_architecture.md §11): rclpy + cobot_interfaces
ONLY (plus cobot_health). Workflows must not even import rclpy — they talk
to ROS through WorkflowContext (context.py) so they stay unit-testable.

Threading model: one workflow at a time (_operation_lock, held for the whole
run). Trigger services live in their own callback group so /task/stop is
reachable WHILE a workflow runs (requires the MultiThreadedExecutor in
main()). Futures are resolved by executor threads and bridged to the worker
inside WorkflowContext; while a goal is in flight the worker polls
_stop_requested and cancels the goal natively.
"""

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_srvs.srv import Trigger

from cobot_interfaces.msg import (
    CobotState,
    CobotStatus,
    HealthSummary,
    Heartbeat,
    LayerReport,
    NodeHealth,
    PickEvent,
    SystemState,
    TaskStatus,
    WorkflowInfo,
    WorkflowList,
)
from cobot_interfaces.srv import StartWorkflow

from cobot_health.heartbeat import SUMMARY_TOPIC, HeartbeatPublisher

from cobot_tasks.context import WorkflowContext
from cobot_tasks.system_state import CAPABILITY_NAMES, RobotSample, VisionFacts, derive
from cobot_tasks.workflows import REGISTRY, WORKFLOWS

LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

SYSTEM_REFRESH_SEC = 1.0        # capability + workflow-readiness re-derivation period

TASK_STATE_NAMES = {
    TaskStatus.STATE_DISCONNECTED: "disconnected",
    TaskStatus.STATE_IDLE: "idle",
    TaskStatus.STATE_SCANNING: "scanning",
    TaskStatus.STATE_PICKING: "picking",
    TaskStatus.STATE_CALIBRATING: "calibrating",
    TaskStatus.STATE_STOPPING: "stopping",
    TaskStatus.STATE_ERROR: "error",
}
COBOT_STATE_NAMES = {
    CobotStatus.STATE_DISCONNECTED: "disconnected",
    CobotStatus.STATE_IDLE: "idle",
    CobotStatus.STATE_BUSY: "busy",
    CobotStatus.STATE_ERROR: "error",
}
HEALTH_LEVEL_NAMES = {
    NodeHealth.LEVEL_OK: "ok",
    NodeHealth.LEVEL_WARN: "warn",
    NodeHealth.LEVEL_ERROR: "error",
    NodeHealth.LEVEL_STALE: "stale",
    NodeHealth.LEVEL_UNKNOWN: "unknown",
}
RUNNING_STATES = (
    TaskStatus.STATE_SCANNING,
    TaskStatus.STATE_PICKING,
    TaskStatus.STATE_CALIBRATING,
    TaskStatus.STATE_STOPPING,
)


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager")

        # ---- node-level params --------------------------------------
        self.declare_parameter("default_workflow", "pick_layer")
        # OFF by default: calibration is operator-selected from the
        # dashboard, nothing runs at boot. Opting in re-enables the old
        # behavior (the startup_calibration workflow launches once the robot
        # and vision services are up) — THE ARM MOVES AT LAUNCH.
        self.declare_parameter("auto_calibrate_on_startup", False)
        # How long to wait for the robot + services before giving up the boot run.
        self.declare_parameter("startup_calibration_wait_sec", 120.0)
        # Which registered calibration workflow runs at boot / behind the
        # /task/calibrate alias: the manual ones or their automatic twins.
        self.declare_parameter("startup_calibration_workflow", "startup_calibration")
        self.declare_parameter("calibrate_alias_workflow", "calibration")

        # ---- workflow params: each name declared once ---------------
        declared: set[str] = set()
        for wf in REGISTRY:
            for spec in wf.params:
                if spec.name not in declared:
                    self.declare_parameter(spec.name, spec.default)
                    declared.add(spec.name)

        default = str(self.get_parameter("default_workflow").value)
        if default not in WORKFLOWS:
            fallback = REGISTRY[0].id
            self.get_logger().error(
                f"default_workflow '{default}' unknown (known: {sorted(WORKFLOWS)}) — using '{fallback}'"
            )
            default = fallback
        self._default_workflow = default

        # ---- publishers ---------------------------------------------
        self._status_pub = self.create_publisher(TaskStatus, "/task/status", LATCHED_QOS)
        self._system_pub = self.create_publisher(SystemState, "/task/system_state", LATCHED_QOS)
        self._workflows_pub = self.create_publisher(WorkflowList, "/task/workflows", LATCHED_QOS)
        self._report_pub = self.create_publisher(LayerReport, "/task/layer_report", 10)
        self._pick_event_pub = self.create_publisher(PickEvent, "/task/pick_event", 10)

        # Last published TaskStatus fields, for the heartbeat (one tuple = atomic).
        self._last_status: tuple[int, str, str, int, int] = (
            TaskStatus.STATE_DISCONNECTED, "", "", 0, 0
        )

        self._operation_lock = threading.Lock()  # one workflow at a time
        self._operation_thread: threading.Thread | None = None
        self._stop_requested = threading.Event()

        # Hardware state mirrored from cobot_control (latched). None = never heard.
        self._robot_state: int | None = None

        # Raw inputs for the derived capability (system_state.derive): newest
        # /cobot/state and /health/summary, each with a monotonic receive
        # stamp. Single-tuple assignment — atomic under the GIL, no lock.
        self._cobot_state: tuple | None = None
        self._health: tuple | None = None
        # Derived read-only snapshot; capability NONE (with "no fresh ..."
        # blockers) until both inputs arrive, so every workflow reads
        # not-ready before the graph is up.
        self._system_snapshot = derive(None, VisionFacts.empty())

        # ---- shared client pool from the registry's ClientSpecs ------
        self._client_specs = {}
        self._wf_clients = {}
        for wf in REGISTRY:
            for spec in wf.clients:
                if spec.key in self._wf_clients:
                    continue  # validate_registry() guarantees identical specs
                self._client_specs[spec.key] = spec
                if spec.kind == "action":
                    self._wf_clients[spec.key] = ActionClient(self, spec.type, spec.name)
                else:
                    self._wf_clients[spec.key] = self.create_client(spec.type, spec.name)

        # ---- services: own group so stop interleaves with a running task
        service_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            StartWorkflow, "/task/start_workflow", self._on_start_workflow,
            callback_group=service_group,
        )
        self.create_service(Trigger, "/task/start", self._on_start, callback_group=service_group)
        self.create_service(Trigger, "/task/stop", self._on_stop, callback_group=service_group)
        self.create_service(
            Trigger, "/task/calibrate", self._on_calibrate, callback_group=service_group
        )

        self.create_subscription(
            CobotStatus, "/cobot/status", self._on_cobot_status, LATCHED_QOS
        )
        self.create_subscription(CobotState, "/cobot/state", self._on_cobot_state, 1)
        self.create_subscription(HealthSummary, SUMMARY_TOPIC, self._on_health, LATCHED_QOS)

        self._publish_status(
            TaskStatus.STATE_DISCONNECTED, "waiting for /cobot/status from cobot_control"
        )

        # Latched snapshot + workflow list: publish now, then one 1 s timer
        # re-derives both, republishing each only when its content changes.
        self._publish_system_state()
        self._last_ready: tuple | None = None
        self._publish_workflows(force=True)
        self.create_timer(SYSTEM_REFRESH_SEC, self._refresh_system_state)

        if self.get_parameter("auto_calibrate_on_startup").value:
            threading.Thread(target=self._run_startup_calibration, daemon=True).start()

        self._hb = HeartbeatPublisher(self, self._health_report)
        self.get_logger().info(
            f"workflows: {[w.id for w in REGISTRY]} (default '{self._default_workflow}')"
        )

    # ------------------------------------------------------------------
    # Status helpers
    # ------------------------------------------------------------------

    def _publish_status(
        self,
        state: int,
        detail: str = "",
        active_task: str = "",
        current: int = 0,
        total: int = 0,
    ):
        msg = TaskStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.detail = detail
        msg.active_task = active_task
        msg.progress_current = int(current)
        msg.progress_total = int(total)
        self._last_status = (state, detail, active_task, int(current), int(total))
        self._status_pub.publish(msg)

    def _readiness(self) -> list[tuple[str, bool, str]]:
        out = []
        for wf in REGISTRY:
            ok, why = wf.check_ready(WorkflowContext(self, wf))
            out.append((wf.id, ok, why))
        return out

    def _publish_workflows(self, force: bool = False):
        readiness = self._readiness()
        # `why` is part of the key: with capability blockers in the reason
        # text, the reason changes while ok stays false — the dashboard must
        # see the new text (bounded by the 1 s refresh timer).
        key = tuple(readiness)
        if not force and key == self._last_ready:
            return
        self._last_ready = key

        msg = WorkflowList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.default_workflow = self._default_workflow
        for wf, (_, ok, why) in zip(REGISTRY, readiness):
            info = WorkflowInfo()
            info.id = wf.id
            info.label = wf.label
            info.description = wf.description
            info.ready = bool(ok)
            info.not_ready_reason = "" if ok else why
            info.requires = int(wf.requires)
            info.requires_name = CAPABILITY_NAMES.get(wf.requires, "?")
            msg.workflows.append(info)
        self._workflows_pub.publish(msg)

    # ------------------------------------------------------------------
    # Derived capability — read-only aggregation of other nodes' state
    # ------------------------------------------------------------------

    def _on_cobot_state(self, msg: CobotState):
        self._cobot_state = (msg, time.monotonic())

    def _on_health(self, msg: HealthSummary):
        self._health = (msg, time.monotonic())

    def _robot_sample(self) -> RobotSample | None:
        held = self._cobot_state
        if held is None:
            return None
        msg, received = held
        return RobotSample(
            connected=bool(msg.connected),
            servo_on=bool(msg.servo_on),
            alarming=bool(msg.alarming),
            erroring=bool(msg.erroring),
            in_hold=bool(msg.in_hold),
            control_mode=int(msg.control_mode),
            paused=bool(msg.paused),
            busy=bool(msg.busy),
            age_s=time.monotonic() - received,
            active_tool_number=int(msg.active_tool_number),
            active_tool_name=str(msg.active_tool_name),
        )

    def _vision_facts(self) -> VisionFacts:
        held = self._health
        if held is None:
            return VisionFacts.empty()
        msg, received = held
        rows = {n.node_name: n for n in msg.nodes}

        def level(name: str) -> str | None:
            row = rows.get(name)
            return None if row is None else HEALTH_LEVEL_NAMES.get(row.level, "unknown")

        loc = rows.get("localization_node")
        metrics = {kv.key: kv.value for kv in loc.metrics} if loc is not None else {}
        calibrated = None
        if "calibrated" in metrics:
            calibrated = metrics["calibrated"] == "true"
        try:
            depth_age = float(metrics["depth_buffer_age_s"])
        except (KeyError, ValueError):
            depth_age = None
        try:
            cal_tool = int(float(metrics["calibration_tool_number"]))
        except (KeyError, ValueError, TypeError):
            cal_tool = None
        return VisionFacts(
            camera_level=level("camera_node"),
            localization_level=level("localization_node"),
            detector_level=level("detector_node"),
            calibrated=calibrated,
            depth_age_s=depth_age,
            age_s=time.monotonic() - received,
            calibration_tool_number=cal_tool,
        )

    def _refresh_system_state(self):
        snapshot = derive(self._robot_sample(), self._vision_facts())
        if snapshot.content() != self._system_snapshot.content():
            self._system_snapshot = snapshot
            self._publish_system_state()
        else:
            self._system_snapshot = snapshot  # keep ages current for check_ready
        self._publish_workflows()

    def _publish_system_state(self):
        snap = self._system_snapshot
        msg = SystemState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.capability = int(snap.capability)
        msg.capability_name = snap.capability_name
        msg.robot_state_fresh = snap.robot_state_fresh
        msg.connected = snap.connected
        msg.servo_on = snap.servo_on
        msg.alarming = snap.alarming
        msg.erroring = snap.erroring
        msg.in_hold = snap.in_hold
        msg.paused = snap.paused
        msg.busy = snap.busy
        msg.control_mode = snap.control_mode
        msg.robot_state_age_s = float(snap.robot_state_age_s)
        msg.health_fresh = snap.health_fresh
        msg.camera_ok = snap.camera_ok
        msg.calibrated = snap.calibrated
        msg.depth_fresh = snap.depth_fresh
        msg.detector_ok = snap.detector_ok
        msg.health_age_s = float(snap.health_age_s)
        msg.move_blockers = list(snap.move_blockers)
        msg.see_blockers = list(snap.see_blockers)
        msg.active_tool_number = int(snap.active_tool_number)
        msg.active_tool_name = snap.active_tool_name
        msg.calibration_tool_number = int(snap.calibration_tool_number)
        msg.calibration_tool_mismatch = bool(snap.calibration_tool_mismatch)
        self._system_pub.publish(msg)

    def _health_report(self):
        state, detail, active_task, current, total = self._last_status
        robot = self._robot_state
        state_name = TASK_STATE_NAMES.get(state, "unknown")
        level = Heartbeat.LEVEL_OK
        if robot is None or robot in (CobotStatus.STATE_DISCONNECTED, CobotStatus.STATE_ERROR):
            level = Heartbeat.LEVEL_WARN
        if state == TaskStatus.STATE_ERROR:
            level = Heartbeat.LEVEL_WARN
        summary = state_name
        if active_task:
            summary += f" · {active_task}"
            if total:
                summary += f" {current}/{total}"
        elif detail:
            summary += f" · {detail}"
        metrics = {
            "state": state_name,
            "active_task": active_task or None,
            "progress": f"{current}/{total}" if total else None,
            "robot_state": None if robot is None else COBOT_STATE_NAMES.get(robot, "unknown"),
            "operation_running": self._operation_lock.locked(),
            "stop_requested": self._stop_requested.is_set(),
            "capability": self._system_snapshot.capability_name,
            "move_blockers": len(self._system_snapshot.move_blockers) or None,
            "see_blockers": len(self._system_snapshot.see_blockers) or None,
        }
        for key, spec in self._client_specs.items():
            client = self._wf_clients[key]
            ready = client.server_is_ready() if spec.kind == "action" else client.service_is_ready()
            metrics[f"{key}_ready"] = ready
        return level, summary, metrics

    def _robot_available(self) -> tuple[bool, str]:
        if self._robot_state is None:
            return False, "no /cobot/status yet — is cobot_control running?"
        if self._robot_state == CobotStatus.STATE_DISCONNECTED:
            return False, "robot not connected"
        if self._robot_state == CobotStatus.STATE_ERROR:
            return False, "robot in error state"
        return True, ""

    def _on_cobot_status(self, msg: CobotStatus):
        previous = self._robot_state
        self._robot_state = msg.state
        # Mirror connection changes into /task/status, but never fight a
        # running workflow for the topic.
        if msg.state == previous or self._operation_lock.locked():
            return
        if msg.state == CobotStatus.STATE_DISCONNECTED:
            self._publish_status(TaskStatus.STATE_DISCONNECTED, msg.detail or "robot disconnected")
        elif previous in (None, CobotStatus.STATE_DISCONNECTED):
            self._publish_status(TaskStatus.STATE_IDLE, "robot connected")

    # ------------------------------------------------------------------
    # The one launch path: gate → lock → worker thread
    # ------------------------------------------------------------------

    def _launch(self, workflow_id: str, source: str) -> tuple[bool, str]:
        wf = WORKFLOWS.get(workflow_id)
        if wf is None:
            return False, f"unknown workflow '{workflow_id}'; known: {[w.id for w in REGISTRY]}"

        ctx = WorkflowContext(self, wf)
        ok, why = wf.check_ready(ctx)
        if not ok:
            return False, why

        if not self._operation_lock.acquire(blocking=False):
            return False, "a task is already running"

        self._stop_requested.clear()
        self.get_logger().info(f"starting workflow '{wf.id}' ({source})")
        self._operation_thread = threading.Thread(
            target=self._run_workflow, args=(wf, ctx), daemon=True
        )
        self._operation_thread.start()
        return True, f"{wf.label} started — progress on /task/status"

    def _run_workflow(self, wf, ctx):
        """Owns self._operation_lock (acquired by _launch); releases on exit."""
        try:
            wf.run(ctx)
            if self._last_status[0] in RUNNING_STATES:
                # Workflow returned without a terminal status — never leave
                # the UI showing a task that is no longer running.
                self._publish_status(TaskStatus.STATE_IDLE, f"{wf.id} finished")
        except Exception as e:  # noqa: BLE001 — a failed workflow must not kill the node
            self.get_logger().error(f"workflow '{wf.id}' crashed: {e!r}")
            self._publish_status(TaskStatus.STATE_ERROR, f"{wf.id} crashed: {e}")
        finally:
            self._operation_lock.release()
            self._publish_workflows(force=True)

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _on_start_workflow(self, request, response):
        workflow_id = request.workflow or self._default_workflow
        response.success, response.message = self._launch(workflow_id, "web")
        return response

    def _on_start(self, request, response):
        response.success, response.message = self._launch(self._default_workflow, "start alias")
        return response

    def _on_calibrate(self, request, response):
        wf_id = str(self.get_parameter("calibrate_alias_workflow").value) or "calibration"
        response.success, response.message = self._launch(wf_id, "calibrate alias")
        return response

    def _on_stop(self, request, response):
        if not self._operation_lock.locked():
            response.success = True
            response.message = "nothing running"
            return response

        self._stop_requested.set()
        self._publish_status(
            TaskStatus.STATE_STOPPING,
            "stop requested — canceling active goal",
            self._last_status[2],
        )
        response.success = True
        response.message = "stop requested (cancels the active motion goal)"
        return response

    # ------------------------------------------------------------------
    # Startup calibration: wait for readiness, then launch the workflow
    # ------------------------------------------------------------------

    def _run_startup_calibration(self):
        """Auto extrinsic re-calibration at boot. Skips (no motion) if the
        grid is unconfigured or nothing comes up in time; a failed fit keeps
        the previously saved transform."""
        wf_id = str(self.get_parameter("startup_calibration_workflow").value) or "startup_calibration"
        wf = WORKFLOWS.get(wf_id)
        if wf is None or not hasattr(wf, "grid_configured"):
            self.get_logger().warn(
                f"startup calibration skipped — '{wf_id}' is not a registered calibration workflow"
            )
            return
        ctx = WorkflowContext(self, wf)

        if not wf.grid_configured(ctx):
            self.get_logger().info("startup calibration skipped — calibration grid not configured")
            return

        deadline = time.monotonic() + float(
            self.get_parameter("startup_calibration_wait_sec").value
        )
        while time.monotonic() < deadline:
            ok, _ = wf.check_ready(ctx)
            if ok:
                break
            time.sleep(1.0)
        else:
            self.get_logger().warn("startup calibration skipped — robot/services not ready in time")
            return

        ok, message = self._launch(wf.id, "startup")
        (self.get_logger().info if ok else self.get_logger().warn)(f"startup calibration: {message}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
