"""cobot_node — motion execution server. Single node, Python, pybind underneath.

The robot arm as a service: no decisions, no vision, no task state. The task
manager (cobot_tasks) is the only intended caller; it owns the pick-layer and
calibration loops and sends this node one goal at a time.

  serves: /cobot/pick_can     (PickCan action)     pick + place ONE can
          /cobot/move_to_pose (MoveToPose action)  one supervised cartesian move
  out:    /cobot/status (CobotStatus, latched) — HARDWARE/connection status
          only (disconnected / idle / busy / error). Operational task state
          lives on /task/status, published by task_manager.

One motion at a time: a goal is REJECTED while another goal is executing
(the task manager serializes anyway; the lock makes any other caller safe).
Cancel policy — see the .action files: PickCan stops at the next safe phase
boundary (never between grip and lift); MoveToPose runs its single move to
completion and then reports canceled.

Threading model: execute callbacks run on MultiThreadedExecutor threads
(reentrant group) and drive the async Cobot choreography with asyncio.run();
goal and cancel requests stay serviceable while a motion runs, which is what
makes native cancellation work.
"""

import asyncio
import json
import threading
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from cobot_interfaces.action import MoveToPose, PickCan
from cobot_interfaces.msg import CobotStatus

from cobot_control.cobot import Cobot, Cobot_Operation_Status


LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CobotNode(Node):
    def __init__(self):
        super().__init__("cobot_node")

        self.declare_parameter("robot_ip", "192.168.1.31")
        # Tool TIP relative to the official toolhead point, [x, y, z] mm along
        # robot base axes at rz=0. Counterpart of localization_node's
        # marker_offset_mm (the red dot off that same point). Zeros = the tip
        # is the toolhead point, i.e. previous behavior.
        self.declare_parameter("tool_offset_mm", [0.0, 0.0, 0.0])
        self.declare_parameter("tool_head_compression_distance_mm", 50.0)
        self.declare_parameter("trajectories_file", "")

        trajectories_file = self.get_parameter("trajectories_file").value
        if not trajectories_file:
            raise RuntimeError(
                "trajectories_file parameter is required "
                "(see cobot_bringup/config/trajectories.json)"
            )
        trajectories = json.loads(Path(trajectories_file).read_text(encoding="utf-8"))

        # Same shape the legacy Cobot consumed from config.JSON.
        cobot_configs = {
            "cobot_ip": self.get_parameter("robot_ip").value,
            "tool_offset_mm": list(self.get_parameter("tool_offset_mm").value),
            "tool_head_compression_distance_mm": self.get_parameter(
                "tool_head_compression_distance_mm"
            ).value,
            "cobot_place_trajectories": trajectories,
        }

        self._status_pub = self.create_publisher(CobotStatus, "/cobot/status", LATCHED_QOS)

        # One motion at a time: acquired in the goal callback (reject if
        # held), released by the execute callback's finally.
        self._busy = threading.Lock()

        self.get_logger().info(f"Connecting to Motoman at {cobot_configs['cobot_ip']} ...")
        try:
            # Connects, then clears any trajectory left queued by a previous
            # session so the first motion_start() can't execute stale points.
            self._cobot = Cobot(cobot_configs)
            self._publish_status(CobotStatus.STATE_IDLE, "connected, trajectory cleared")
        except Exception as e:  # noqa: BLE001 — node must come up even if robot is off
            self._cobot = None
            self.get_logger().error(f"Cobot connection failed: {e}")
            self._publish_status(CobotStatus.STATE_DISCONNECTED, str(e))

        # Reentrant group: goal/cancel requests must be serviceable while an
        # execute callback blocks on the choreography.
        action_group = ReentrantCallbackGroup()
        self._pick_server = ActionServer(
            self,
            PickCan,
            "/cobot/pick_can",
            execute_callback=self._execute_pick,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=action_group,
        )
        self._move_server = ActionServer(
            self,
            MoveToPose,
            "/cobot/move_to_pose",
            execute_callback=self._execute_move,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=action_group,
        )

    # ------------------------------------------------------------------
    # Status helper
    # ------------------------------------------------------------------

    def _publish_status(self, state: int, detail: str = ""):
        msg = CobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.detail = detail
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Goal gating (shared by both actions)
    # ------------------------------------------------------------------

    def _on_goal_request(self, goal_request):
        if self._cobot is None:
            self.get_logger().warn("goal rejected: robot not connected")
            return GoalResponse.REJECT
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn("goal rejected: a motion goal is already executing")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle):
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # /cobot/pick_can
    # ------------------------------------------------------------------

    def _execute_pick(self, goal_handle):
        can = goal_handle.request.target
        feedback = PickCan.Feedback()
        result = PickCan.Result()
        try:
            self._publish_status(
                CobotStatus.STATE_BUSY,
                f"pick_can ({can.real_x:.1f}, {can.real_y:.1f}, rz={can.real_rz:.1f})",
            )

            def on_phase(name: str):
                feedback.phase = name
                goal_handle.publish_feedback(feedback)

            report = asyncio.run(
                self._cobot.pick_and_place(
                    can,
                    should_abort=lambda: goal_handle.is_cancel_requested,
                    on_phase=on_phase,
                )
            )

            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = (
                    f"canceled during '{feedback.phase}' — can may still be gripped"
                )
                goal_handle.canceled()
            elif report.overall_status == Cobot_Operation_Status.job_success:
                result.success = True
                result.message = "pick and place complete"
                goal_handle.succeed()
            else:
                result.success = False
                result.message = f"failed during '{feedback.phase}' — see cobot_node log"
                self.get_logger().warn(f"pick failed:\n{report}")
                goal_handle.abort()
            return result

        except Exception as e:  # noqa: BLE001 — a failed pick must not kill the node
            self.get_logger().error(f"pick crashed: {e}")
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result

        finally:
            self._publish_status(CobotStatus.STATE_IDLE, "ready")
            self._busy.release()

    # ------------------------------------------------------------------
    # /cobot/move_to_pose
    # ------------------------------------------------------------------

    def _execute_move(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()
        try:
            self._publish_status(
                CobotStatus.STATE_BUSY,
                f"move_to_pose ({goal.x:.1f}, {goal.y:.1f}, {goal.z:.1f})",
            )

            report = asyncio.run(
                self._cobot.move_to(goal.x, goal.y, goal.z, goal.rx, goal.ry, goal.rz)
            )
            reached = report.overall_status == Cobot_Operation_Status.job_success

            if goal_handle.is_cancel_requested:
                # A single move runs to completion — no mid-motion stop in the
                # pybind layer. The cancel is acknowledged once it settles.
                result.success = False
                result.message = (
                    "canceled (move had already run to completion)"
                    if reached
                    else "canceled (move failed)"
                )
                goal_handle.canceled()
            elif reached:
                result.success = True
                result.message = "reached pose"
                goal_handle.succeed()
            else:
                result.success = False
                result.message = "move failed — see cobot_node log"
                self.get_logger().warn(f"move failed:\n{report}")
                goal_handle.abort()
            return result

        except Exception as e:  # noqa: BLE001 — a failed move must not kill the node
            self.get_logger().error(f"move crashed: {e}")
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result

        finally:
            self._publish_status(CobotStatus.STATE_IDLE, "ready")
            self._busy.release()


def main(args=None):
    rclpy.init(args=args)
    node = CobotNode()
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
