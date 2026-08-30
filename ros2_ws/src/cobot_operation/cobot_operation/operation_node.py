"""cobot_operation — pick choreography server. Python, no pybind.

  serves: /cobot/pick_can (PickCan action)  pick + place ONE can by chaining
          cobot_control primitives (MoveToPose, set_output_bit, /cobot/state)
  in:     /cobot/status (CobotStatus, latched)  goal gating
          /cobot/state  (CobotState, 10 Hz)     sensors / pause latch
  out:    /health/heartbeat

The task manager (cobot_tasks) is the only intended caller and sends one goal
at a time; the lock here makes any other caller safe.

Cancel: IMMEDIATE — the in-flight MoveToPose is canceled (cobot_control does
MotionStop) and the choreography returns at its next boundary.
Pause: cobot_control's /cobot/pause holds the arm; the choreography sees
CobotState.paused and waits before issuing its next primitive. One service
pauses both layers.

Execution model: the execute callback is a coroutine; it awaits action /
service futures via RosPrimitives, so no executor thread is ever parked and
goal / cancel requests stay serviceable mid-pick. Everything shares ONE
reentrant callback group on a MultiThreadedExecutor.
"""

from __future__ import annotations

import json
import threading
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from cobot_health.heartbeat import HeartbeatPublisher
from cobot_interfaces.action import PickCan
from cobot_interfaces.msg import CobotStatus, Heartbeat, JobResult as JobResultMsg
from cobot_interfaces.msg import SensorAssert as SensorAssertMsg

from cobot_operation.choreography import PickChoreography, PickConfig
from cobot_operation.primitives import RosPrimitives
from cobot_operation.report import OperationReport
from cobot_operation.setup_config import load_tools, sensor_names

LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def report_to_msgs(report: OperationReport):
    jobs = [
        JobResultMsg(
            job=j.job, ok=bool(j.ok), status_code=int(j.status_code),
            message=str(j.message), payload=str(j.payload),
        )
        for j in report.jobs
    ]
    asserts = [
        SensorAssertMsg(
            name=a.name, expected=bool(a.expected), actual=bool(a.actual), passed=bool(a.passed)
        )
        for a in report.sensor_asserts
    ]
    return jobs, asserts


class CobotOperationNode(Node):
    def __init__(self):
        super().__init__("cobot_operation")

        self.declare_parameter("setup_file", "")
        self.declare_parameter("active_tool", "")
        self.declare_parameter("trajectories_file", "")
        self.declare_parameter("move_timeout_s", 120.0)
        self.declare_parameter("service_timeout_s", 5.0)
        self.declare_parameter("sensor_assert_timeout_s", 1.0)

        setup_file = self.get_parameter("setup_file").value
        trajectories_file = self.get_parameter("trajectories_file").value
        if not setup_file:
            raise RuntimeError("setup_file parameter is required (cobot_bringup/config/cobot_setup.yaml)")
        if not trajectories_file:
            raise RuntimeError("trajectories_file parameter is required (cobot_bringup/config/trajectories.json)")

        tools, active = load_tools(setup_file)
        override = self.get_parameter("active_tool").value
        if override:
            if override not in tools:
                raise RuntimeError(f"active_tool '{override}' not in cobot_setup.yaml tools {sorted(tools)}")
            active = override
        self._tool = tools[active]
        trajectories = json.loads(Path(trajectories_file).read_text(encoding="utf-8"))

        self._group = ReentrantCallbackGroup()
        self._prims = RosPrimitives(
            self,
            self._group,
            move_timeout_s=float(self.get_parameter("move_timeout_s").value),
            service_timeout_s=float(self.get_parameter("service_timeout_s").value),
        )
        cfg = PickConfig(
            tool=self._tool,
            trajectories=trajectories,
            sensor_assert_timeout_s=float(self.get_parameter("sensor_assert_timeout_s").value),
            known_sensors=sensor_names(setup_file),
        )
        cfg.validate()
        self._choreo = PickChoreography(self._prims, cfg)
        self.get_logger().info(
            f"tool '{self._tool.name}' (tool file {self._tool.tool_number}, tip offset "
            f"{self._tool.tip_offset}, pick z {self._tool.pick_tip_z_mm}, "
            f"approach z {self._tool.approach_tip_z_mm})"
        )

        self._robot_state = None
        self.create_subscription(
            CobotStatus, "/cobot/status", self._on_cobot_status, LATCHED_QOS,
            callback_group=self._group,
        )

        # One pick at a time: acquired in the goal callback (reject if held),
        # released by the execute callback's finally.
        self._busy = threading.Lock()
        self._ready_logged = False
        self.create_timer(1.0, self._readiness_tick, callback_group=self._group)

        self._server = ActionServer(
            self,
            PickCan,
            "/cobot/pick_can",
            execute_callback=self._execute_pick,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=self._group,
        )

        self._hb = HeartbeatPublisher(self, self._health_report)

    # ------------------------------------------------------------------

    def _on_cobot_status(self, msg: CobotStatus):
        self._robot_state = msg.state

    def _readiness_tick(self):
        if self._prims.ready():
            if not self._ready_logged:
                self.get_logger().info("cobot_control primitives ready")
                self._ready_logged = True
        elif self._ready_logged:
            self.get_logger().warn(f"cobot_control primitives lost: {self._prims.missing()}")
            self._ready_logged = False

    def _health_report(self):
        ready = self._prims.ready()
        busy = self._busy.locked()
        if not ready:
            level, detail = Heartbeat.LEVEL_ERROR, f"waiting for {', '.join(self._prims.missing())}"
        elif self._robot_state in (None, CobotStatus.STATE_DISCONNECTED):
            level, detail = Heartbeat.LEVEL_ERROR, "robot disconnected"
        else:
            level = Heartbeat.LEVEL_OK
            detail = f"picking · {self._choreo.phase}" if busy else "idle"
        return level, detail, {
            "primitives_ready": ready,
            "busy": busy,
            "paused": self._prims.is_paused(),
            "last_placed": self._choreo.last_placed.name,
            "active_tool": self._tool.name,
        }

    # ------------------------------------------------------------------

    def _on_goal_request(self, goal_request):
        if not self._prims.ready():
            self.get_logger().warn(f"goal rejected: waiting for {self._prims.missing()}")
            return GoalResponse.REJECT
        if self._robot_state != CobotStatus.STATE_IDLE:
            self.get_logger().warn(f"goal rejected: robot status is {self._robot_state} (need IDLE)")
            return GoalResponse.REJECT
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn("goal rejected: a pick is already executing")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle):
        # Immediate: stop the arm now; the choreography returns at its next
        # boundary once the canceled move result comes back.
        if self._prims.cancel_inflight():
            self.get_logger().warn("cancel: in-flight move canceled (controller MotionStop)")
        return CancelResponse.ACCEPT

    async def _execute_pick(self, goal_handle):
        can = goal_handle.request.target
        feedback = PickCan.Feedback()
        result = PickCan.Result()
        try:
            self.get_logger().info(
                f"pick_can ({can.real_x:.1f}, {can.real_y:.1f}, rz={can.real_rz:.1f})"
            )

            def on_phase(name: str):
                feedback.phase = name
                goal_handle.publish_feedback(feedback)

            report = await self._choreo.pick_and_place(
                can,
                should_abort=lambda: goal_handle.is_cancel_requested,
                on_phase=on_phase,
            )
            result.phase = feedback.phase
            result.jobs, result.sensor_asserts = report_to_msgs(report)

            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = (
                    f"canceled during '{feedback.phase}' — can may still be gripped, "
                    f"arm may be mid-segment"
                )
                self.get_logger().warn(result.message)
                goal_handle.canceled()
            elif report.overall_ok:
                result.success = True
                result.message = "pick and place complete"
                goal_handle.succeed()
            else:
                result.success = False
                result.message = f"failed during '{feedback.phase}': {report.first_failure()}"
                self.get_logger().warn(f"pick failed:\n{report}")
                goal_handle.abort()
            return result

        except Exception as e:  # noqa: BLE001 — a failed pick must not kill the node
            self.get_logger().error(f"pick crashed: {e!r}")
            result.success = False
            result.message = str(e)
            result.phase = feedback.phase
            goal_handle.abort()
            return result

        finally:
            self._busy.release()


def main(args=None):
    rclpy.init(args=args)
    node = CobotOperationNode()
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
