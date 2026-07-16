"""task_manager_node — owns WHAT the system is doing.

The decision/sequencing layer between the web UI and the executors. It runs
the state machine (idle / scanning / picking / calibrating), sends one action
goal or service call at a time, and publishes the single source of
operational truth. cobot_node executes motion; the vision nodes do the math;
this node only decides, gates and reports.

  in:  /task/start | /task/stop | /task/calibrate   (std_srvs/Trigger)
       /vision/targets (CanTargetArray) — DATA, not a trigger: consumed only
                                          while a scan this node requested is
                                          in flight; ignored otherwise
       /cobot/status (CobotStatus)      — hardware/connection state, mirrored
                                          into /task/status when no task runs
  out: /task/status (TaskStatus, latched) — THE operational state
       /task/layer_report (LayerReport)
  calls:
       /cobot/pick_can      (PickCan action)     one can per goal
       /cobot/move_to_pose  (MoveToPose action)  calibration grid moves
       /vision/scan, /vision/calibration_begin|sample|compute  (services)

Dependency rule (docs/ros2_architecture.md §11): rclpy + cobot_interfaces
ONLY. If this node ever computes geometry or imports anything robot- or
camera-facing, it has become a god-node — reject the change.

Threading model: tasks run in one background worker thread, one at a time
(_operation_lock, held for the whole task). Trigger services live in their
own callback group so /task/stop is reachable WHILE a task runs (requires the
MultiThreadedExecutor in main()). Action/service futures are resolved by
executor threads and bridged to the worker with events; while a goal is in
flight the worker polls _stop_requested and cancels the goal natively, so
stop interrupts a pick mid-choreography instead of waiting for the next can.
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

from cobot_interfaces.action import MoveToPose, PickCan
from cobot_interfaces.msg import CanTargetArray, CobotStatus, LayerReport, TaskStatus
from cobot_interfaces.srv import CalibrationSample


LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

GOAL_ACK_TIMEOUT_SEC = 10.0     # server accepting/rejecting the goal
PICK_TIMEOUT_SEC = 300.0        # one full pick+place choreography
MOVE_TIMEOUT_SEC = 120.0        # one calibration grid move
# calibration_begin lazy-loads the red-dot model on the vision side.
CAL_BEGIN_TIMEOUT_SEC = 120.0
CAL_SAMPLE_TIMEOUT_SEC = 30.0
CAL_COMPUTE_TIMEOUT_SEC = 60.0


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager")

        # How long after /vision/scan to wait for the targets message.
        self.declare_parameter("scan_timeout_sec", 30.0)

        # Calibration pose grid — SET SAFE, REACHABLE BOUNDS BEFORE RUNNING.
        # Every pose must keep the marker inside the camera frame and the arm
        # clear of the bin. [0,0] ranges mean "not configured": calibrate refuses.
        self.declare_parameter("calibration_x_range_mm", [0.0, 0.0])
        self.declare_parameter("calibration_y_range_mm", [0.0, 0.0])
        self.declare_parameter("calibration_z_levels_mm", [0.0])
        self.declare_parameter("calibration_grid_size", [3, 2])  # nx, ny per level
        self.declare_parameter("calibration_orientation_deg", [180.0, 0.0, 0.0])
        self.declare_parameter("calibration_settle_sec", 0.5)

        self._status_pub = self.create_publisher(TaskStatus, "/task/status", LATCHED_QOS)
        self._report_pub = self.create_publisher(LayerReport, "/task/layer_report", 10)

        self._operation_lock = threading.Lock()  # one task at a time
        self._operation_thread: threading.Thread | None = None
        self._stop_requested = threading.Event()

        # Hardware state mirrored from cobot_node (latched). None = never heard.
        self._robot_state: int | None = None

        # Targets handoff: _on_targets fills the slot only while a scan this
        # node requested is in flight (data, not a trigger).
        self._targets_lock = threading.Lock()
        self._awaiting_targets = False
        self._targets_slot: list | None = None
        self._targets_event = threading.Event()

        # Services in their own group: stop must interleave with a running task.
        service_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, "/task/start", self._on_start, callback_group=service_group)
        self.create_service(Trigger, "/task/stop", self._on_stop, callback_group=service_group)
        self.create_service(
            Trigger, "/task/calibrate", self._on_calibrate, callback_group=service_group
        )

        self.create_subscription(CanTargetArray, "/vision/targets", self._on_targets, 10)
        self.create_subscription(
            CobotStatus, "/cobot/status", self._on_cobot_status, LATCHED_QOS
        )

        self._pick_client = ActionClient(self, PickCan, "/cobot/pick_can")
        self._move_client = ActionClient(self, MoveToPose, "/cobot/move_to_pose")

        self._scan_client = self.create_client(Trigger, "/vision/scan")
        self._cal_begin_client = self.create_client(Trigger, "/vision/calibration_begin")
        self._cal_sample_client = self.create_client(
            CalibrationSample, "/vision/calibration_sample"
        )
        self._cal_compute_client = self.create_client(Trigger, "/vision/calibration_compute")

        self._publish_status(
            TaskStatus.STATE_DISCONNECTED, "waiting for /cobot/status from cobot_node"
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
        self._status_pub.publish(msg)

    def _robot_available(self) -> tuple[bool, str]:
        if self._robot_state is None:
            return False, "no /cobot/status yet — is cobot_node running?"
        if self._robot_state == CobotStatus.STATE_DISCONNECTED:
            return False, "robot not connected"
        if self._robot_state == CobotStatus.STATE_ERROR:
            return False, "robot in error state"
        return True, ""

    def _on_cobot_status(self, msg: CobotStatus):
        previous = self._robot_state
        self._robot_state = msg.state
        # Mirror connection changes into /task/status, but never fight a
        # running task for the topic.
        if msg.state == previous or self._operation_lock.locked():
            return
        if msg.state == CobotStatus.STATE_DISCONNECTED:
            self._publish_status(TaskStatus.STATE_DISCONNECTED, msg.detail or "robot disconnected")
        elif previous in (None, CobotStatus.STATE_DISCONNECTED):
            self._publish_status(TaskStatus.STATE_IDLE, "robot connected")

    # ------------------------------------------------------------------
    # Blocking call helpers (worker thread; executor threads resolve futures)
    # ------------------------------------------------------------------

    def _call(self, client, request, timeout: float):
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout):
            future.cancel()
            raise TimeoutError(f"{client.srv_name} timed out after {timeout:.0f}s")
        return future.result()

    def _run_action(self, client, action_type, name: str, goal, timeout: float):
        """Send one goal and block for its result. While waiting, a stop
        request cancels the goal natively (the server stops at its next safe
        phase boundary). Returns the action result (success/message) or a
        synthesized failure if the goal was rejected."""
        goal_future = client.send_goal_async(goal)
        acked = threading.Event()
        goal_future.add_done_callback(lambda _: acked.set())
        if not acked.wait(timeout=GOAL_ACK_TIMEOUT_SEC):
            raise TimeoutError(f"{name} goal was never acknowledged")

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            result = action_type.Result()
            result.success = False
            result.message = "goal rejected by cobot_node (busy or disconnected)"
            return result

        result_future = goal_handle.get_result_async()
        done = threading.Event()
        result_future.add_done_callback(lambda _: done.set())

        cancel_sent = False
        deadline = time.monotonic() + timeout
        while not done.wait(timeout=0.2):
            if self._stop_requested.is_set() and not cancel_sent:
                self.get_logger().info(f"stop → canceling {name} goal")
                goal_handle.cancel_goal_async()
                cancel_sent = True
            if time.monotonic() > deadline:
                if not cancel_sent:
                    goal_handle.cancel_goal_async()
                raise TimeoutError(f"{name} timed out after {timeout:.0f}s")

        return result_future.result().result

    # ------------------------------------------------------------------
    # Trigger services — gate, then hand off to the worker thread
    # ------------------------------------------------------------------

    def _on_start(self, request, response):
        ok, why = self._robot_available()
        if not ok:
            response.success = False
            response.message = why
            return response

        if not self._scan_client.service_is_ready():
            response.success = False
            response.message = "/vision/scan not available"
            return response

        if not self._pick_client.server_is_ready():
            response.success = False
            response.message = "/cobot/pick_can action server not available"
            return response

        if not self._operation_lock.acquire(blocking=False):
            response.success = False
            response.message = "a task is already running"
            return response

        self._stop_requested.clear()
        self._operation_thread = threading.Thread(target=self._run_pick_cycle, daemon=True)
        self._operation_thread.start()

        response.success = True
        response.message = "pick cycle started — progress on /task/status"
        return response

    def _on_stop(self, request, response):
        if not self._operation_lock.locked():
            response.success = True
            response.message = "nothing running"
            return response

        self._stop_requested.set()
        self._publish_status(TaskStatus.STATE_STOPPING, "stop requested — canceling active goal")
        response.success = True
        response.message = "stop requested (cancels the active motion goal)"
        return response

    def _on_calibrate(self, request, response):
        ok, why = self._robot_available()
        if not ok:
            response.success = False
            response.message = why
            return response

        if list(self.get_parameter("calibration_x_range_mm").value) == [0.0, 0.0] or \
           list(self.get_parameter("calibration_y_range_mm").value) == [0.0, 0.0] or \
           list(self.get_parameter("calibration_z_levels_mm").value) == [0.0]:
            response.success = False
            response.message = (
                "calibration grid not configured — set calibration_x_range_mm / "
                "calibration_y_range_mm / calibration_z_levels_mm to safe bounds"
            )
            return response

        if not self._move_client.server_is_ready():
            response.success = False
            response.message = "/cobot/move_to_pose action server not available"
            return response

        for client, name in (
            (self._cal_begin_client, "/vision/calibration_begin"),
            (self._cal_sample_client, "/vision/calibration_sample"),
            (self._cal_compute_client, "/vision/calibration_compute"),
        ):
            if not client.service_is_ready():
                response.success = False
                response.message = f"{name} not available"
                return response

        if not self._operation_lock.acquire(blocking=False):
            response.success = False
            response.message = "a task is already running"
            return response

        self._stop_requested.clear()
        self._operation_thread = threading.Thread(target=self._run_calibration, daemon=True)
        self._operation_thread.start()

        response.success = True
        response.message = "calibration started — progress on /task/status"
        return response

    # ------------------------------------------------------------------
    # Targets handoff (topic is data, not a trigger)
    # ------------------------------------------------------------------

    def _on_targets(self, msg: CanTargetArray):
        with self._targets_lock:
            if not self._awaiting_targets:
                self.get_logger().info(
                    f"ignoring {len(msg.targets)} targets — no scan in flight"
                )
                return
            self._targets_slot = list(msg.targets)
            self._awaiting_targets = False
        self._targets_event.set()

    def _await_targets(self, timeout: float) -> list | None:
        """Wait for the targets produced by the scan we just requested.
        Returns None on stop or timeout."""
        deadline = time.monotonic() + timeout
        while not self._targets_event.wait(timeout=0.2):
            if self._stop_requested.is_set() or time.monotonic() > deadline:
                with self._targets_lock:
                    self._awaiting_targets = False
                return None
        with self._targets_lock:
            return self._targets_slot

    # ------------------------------------------------------------------
    # Task: scan → pick layer
    # ------------------------------------------------------------------

    def _run_pick_cycle(self):
        """Owns self._operation_lock (acquired by _on_start); releases on exit."""
        try:
            self._publish_status(TaskStatus.STATE_SCANNING, "requesting scan", "pick_layer")

            with self._targets_lock:
                self._awaiting_targets = True
                self._targets_slot = None
                self._targets_event.clear()

            scan = self._call(self._scan_client, Trigger.Request(), timeout=10.0)
            if not scan.success:
                with self._targets_lock:
                    self._awaiting_targets = False
                self._publish_status(TaskStatus.STATE_ERROR, f"scan refused: {scan.message}")
                return

            timeout = float(self.get_parameter("scan_timeout_sec").value)
            targets = self._await_targets(timeout)

            if self._stop_requested.is_set():
                self._publish_status(TaskStatus.STATE_IDLE, "stopped while waiting for scan")
                return
            if targets is None:
                self._publish_status(
                    TaskStatus.STATE_ERROR, f"no targets within {timeout:.0f}s of scan"
                )
                return
            if not targets:
                self._publish_status(TaskStatus.STATE_IDLE, "scan found no cans")
                return

            report = LayerReport()
            report.overall_ok = True

            for index, can in enumerate(targets):
                if self._stop_requested.is_set():
                    self.get_logger().warn(f"stop — abandoning layer at can {index + 1}")
                    report.overall_ok = False
                    break

                self._publish_status(
                    TaskStatus.STATE_PICKING,
                    f"picking can at ({can.real_x:.1f}, {can.real_y:.1f}, rz={can.real_rz:.1f})",
                    "pick_layer",
                    index + 1,
                    len(targets),
                )

                goal = PickCan.Goal(target=can)
                result = self._run_action(
                    self._pick_client, PickCan, "/cobot/pick_can", goal, PICK_TIMEOUT_SEC
                )

                if result.success:
                    report.successes += 1
                else:
                    report.failures += 1
                    report.overall_ok = False
                    self.get_logger().warn(f"pick {index + 1} failed: {result.message}")

            report.header.stamp = self.get_clock().now().to_msg()
            self._report_pub.publish(report)
            self._publish_status(
                TaskStatus.STATE_IDLE,
                f"layer done: {report.successes} ok / {report.failures} failed",
            )

        except Exception as e:  # noqa: BLE001 — a failed task must not kill the node
            self.get_logger().error(f"pick cycle crashed: {e}")
            self._publish_status(TaskStatus.STATE_ERROR, str(e))

        finally:
            with self._targets_lock:
                self._awaiting_targets = False
            self._operation_lock.release()

    # ------------------------------------------------------------------
    # Task: calibration (movement loop here, math in localization_node)
    # ------------------------------------------------------------------

    def _calibration_poses(self) -> list[tuple[float, float, float]]:
        x_min, x_max = self.get_parameter("calibration_x_range_mm").value
        y_min, y_max = self.get_parameter("calibration_y_range_mm").value
        z_levels = list(self.get_parameter("calibration_z_levels_mm").value)
        nx, ny = (int(v) for v in self.get_parameter("calibration_grid_size").value)

        # linspace by hand to avoid importing numpy just for this
        def spread(lo: float, hi: float, n: int) -> list[float]:
            if n <= 1:
                return [(lo + hi) / 2.0]
            step = (hi - lo) / (n - 1)
            return [lo + i * step for i in range(n)]

        return [
            (x, y, z)
            for z in z_levels
            for y in spread(y_min, y_max, ny)
            for x in spread(x_min, x_max, nx)
        ]

    def _run_calibration(self):
        """Owns self._operation_lock (acquired by _on_calibrate); releases on exit."""
        try:
            poses = self._calibration_poses()
            rx, ry, rz = self.get_parameter("calibration_orientation_deg").value
            settle = float(self.get_parameter("calibration_settle_sec").value)

            self._publish_status(
                TaskStatus.STATE_CALIBRATING,
                f"starting: {len(poses)} poses",
                "calibration",
                0,
                len(poses),
            )

            # Vision clears its sample buffer and warms up the red-dot model.
            begin = self._call(self._cal_begin_client, Trigger.Request(), CAL_BEGIN_TIMEOUT_SEC)
            if not begin.success:
                self._publish_status(
                    TaskStatus.STATE_ERROR, f"calibration begin refused: {begin.message}"
                )
                return

            collected = 0
            for index, (x, y, z) in enumerate(poses):
                if self._stop_requested.is_set():
                    self._publish_status(
                        TaskStatus.STATE_IDLE,
                        f"calibration aborted by stop at pose {index + 1}/{len(poses)}",
                    )
                    return

                label = f"pose {index + 1}/{len(poses)} ({x:.0f}, {y:.0f}, {z:.0f})"
                self._publish_status(
                    TaskStatus.STATE_CALIBRATING,
                    f"{label} — {collected} samples so far",
                    "calibration",
                    index + 1,
                    len(poses),
                )

                goal = MoveToPose.Goal(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)
                move = self._run_action(
                    self._move_client, MoveToPose, "/cobot/move_to_pose", goal, MOVE_TIMEOUT_SEC
                )
                if not move.success:
                    self.get_logger().warn(f"{label}: move failed — skipped ({move.message})")
                    continue

                time.sleep(settle)  # let vibration die before vision measures

                sample = self._call(
                    self._cal_sample_client,
                    CalibrationSample.Request(x=x, y=y, z=z),
                    CAL_SAMPLE_TIMEOUT_SEC,
                )
                if sample.success:
                    collected = sample.sample_count
                else:
                    self.get_logger().warn(f"{label}: no sample ({sample.message})")

            self._publish_status(
                TaskStatus.STATE_CALIBRATING,
                f"grid done: {collected} samples — computing transform",
                "calibration",
                len(poses),
                len(poses),
            )

            compute = self._call(self._cal_compute_client, Trigger.Request(), CAL_COMPUTE_TIMEOUT_SEC)
            if compute.success:
                self._publish_status(
                    TaskStatus.STATE_IDLE, f"calibration complete: {compute.message}"
                )
            else:
                self._publish_status(
                    TaskStatus.STATE_ERROR, f"calibration failed: {compute.message}"
                )

        except Exception as e:  # noqa: BLE001 — a failed task must not kill the node
            self.get_logger().error(f"calibration crashed: {e}")
            self._publish_status(TaskStatus.STATE_ERROR, f"calibration crashed: {e}")

        finally:
            self._operation_lock.release()


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
