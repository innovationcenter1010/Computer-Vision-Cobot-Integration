"""cobot_operation — pick choreography server. Python, no pybind.

  serves: /cobot/pick_can (PickCan action)  pick + place ONE can by chaining
          cobot_control primitives (MoveToPose, set_output_bit, /cobot/state)
          /cobot/hide (Hide action)  one MoveToPose to the remembered "hide"
          teach point — parks the arm out of the camera's view. The ONLY
          other caller-facing motion action besides pick_can: task_manager
          must never talk to cobot_control directly for a remembered
          position, so pick_layer sends this instead of /cobot/move_to_pose.
          Shares the same busy lock / active-tool gate as a pick.
          /cobot/convert_trajectories (ConvertTrajectories)  re-express EVERY
          remembered position in a tool file via /cobot/convert_pose
          (controller kinematics) and persist — all-or-nothing
          /cobot/save_teach_point (SaveTeachPoint)  overwrite one remembered
          position with the current pose + joints
  in:     /cobot/status (CobotStatus, latched)  goal gating
          /cobot/state  (CobotState, 10 Hz)     sensors / pause latch / ACTIVE TOOL
  out:    /health/heartbeat

Tooling: cobot_setup.yaml `tools` holds the per-tool cell values (loaded once);
the ACTIVE tool is whatever cobot_control reports on /cobot/state — never a
parameter here. Remembered positions (runtime_config/trajectories.json) each
carry the tool file they are expressed in; a pick is refused until they all
match the active tool, and (auto_convert_on_tool_change) they are converted
through the controller's kinematics as soon as the active tool changes.

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

import threading
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from cobot_health.heartbeat import HeartbeatPublisher
from cobot_interfaces.action import Hide, PickCan
from cobot_interfaces.msg import CobotStatus, Heartbeat, JobResult as JobResultMsg
from cobot_interfaces.msg import SensorAssert as SensorAssertMsg
from cobot_interfaces.srv import ConvertTrajectories, SaveTeachPoint, SetTeachPoint

from cobot_operation.choreography import PickChoreography, PickConfig
from cobot_operation.geometry import Pose6, ToolProfile
from cobot_operation.primitives import RosPrimitives
from cobot_operation.report import OperationReport
from cobot_operation.setup_config import load_tools, sensor_names
from cobot_operation.trajectory_store import TrajectoryStore

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
        self.declare_parameter("trajectories_file", "")
        self.declare_parameter("trajectories_default_file", "")
        self.declare_parameter("move_timeout_s", 120.0)
        self.declare_parameter("service_timeout_s", 5.0)
        self.declare_parameter("sensor_assert_timeout_s", 1.0)
        self.declare_parameter("sensor_assert_authoritative", False)
        self.declare_parameter("auto_convert_on_tool_change", True)

        setup_file = self.get_parameter("setup_file").value
        trajectories_file = self.get_parameter("trajectories_file").value
        default_file = self.get_parameter("trajectories_default_file").value
        if not setup_file:
            raise RuntimeError("setup_file parameter is required (cobot_bringup/config/cobot_setup.yaml)")
        if not trajectories_file:
            raise RuntimeError("trajectories_file parameter is required (runtime_config/trajectories.json)")

        # ---- per-tool cell values (once) ----
        self._tools = load_tools(setup_file)
        for note in self._tools.notes:
            self.get_logger().warn(note)
        self.get_logger().info(
            "tools: " + ", ".join(
                f"{t.name}=tool file {t.tool_number}" for t in self._tools.by_name.values()
            )
        )

        # ---- remembered positions (runtime-writable, seeded, migrated) ----
        raw_before = Path(trajectories_file).read_text(encoding="utf-8") if Path(trajectories_file).exists() else None
        self._store = TrajectoryStore.seed(
            trajectories_file, default_file or trajectories_file, log=self.get_logger().info
        )
        if self._store.migrated:
            if raw_before is not None:
                bak = self._store.backup_v1(raw_before)
                self.get_logger().warn(f"migrated v1 trajectories.json (backup {bak})")
            self._store.save()
        self._store.validate()
        self.get_logger().info(f"trajectories: {self._store.summary()}")

        self._group = ReentrantCallbackGroup()
        self._prims = RosPrimitives(
            self,
            self._group,
            move_timeout_s=float(self.get_parameter("move_timeout_s").value),
            service_timeout_s=float(self.get_parameter("service_timeout_s").value),
        )
        cfg = PickConfig(
            tool=self._tools.none(),
            trajectories=self._store.poses(),
            sensor_assert_timeout_s=float(self.get_parameter("sensor_assert_timeout_s").value),
            sensor_assert_authoritative=bool(self.get_parameter("sensor_assert_authoritative").value),
            known_sensors=sensor_names(setup_file),
        )
        cfg.validate()
        self._choreo = PickChoreography(self._prims, cfg)

        self._robot_state = None
        self.create_subscription(
            CobotStatus, "/cobot/status", self._on_cobot_status, LATCHED_QOS,
            callback_group=self._group,
        )

        # One pick OR one conversion at a time: acquired in the goal callback
        # (reject if held) / at conversion start, released in a finally.
        self._busy = threading.Lock()
        self._pick_tool: Optional[ToolProfile] = None
        self._ready_logged = False
        # Auto-conversion bookkeeping: the last active tool number we reacted
        # to, so one change triggers one attempt (retry = next change or the
        # explicit service).
        self._auto_convert = bool(self.get_parameter("auto_convert_on_tool_change").value)
        self._last_seen_tool: Optional[int] = None
        self._auto_attempted_for: Optional[int] = None
        self._last_convert_message = ""
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
        self._hide_server = ActionServer(
            self,
            Hide,
            "/cobot/hide",
            execute_callback=self._execute_hide,
            goal_callback=self._on_hide_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=self._group,
        )
        self.create_service(
            ConvertTrajectories, "/cobot/convert_trajectories", self._on_convert_trajectories,
            callback_group=self._group,
        )
        self.create_service(
            SaveTeachPoint, "/cobot/save_teach_point", self._on_save_teach_point,
            callback_group=self._group,
        )
        self.create_service(
            SetTeachPoint, "/cobot/set_teach_point", self._on_set_teach_point,
            callback_group=self._group,
        )

        self._hb = HeartbeatPublisher(self, self._health_report)

    # ------------------------------------------------------------------

    def _on_cobot_status(self, msg: CobotStatus):
        self._robot_state = msg.state

    def _resolve_active_tool(self) -> Tuple[Optional[ToolProfile], str]:
        """The cobot_setup.yaml profile for the tool cobot_control says is active."""
        active = self._prims.active_tool()
        if active is None:
            return None, "no /cobot/state yet"
        number, name = active
        profile = self._tools.by_number(number)
        if profile is None:
            return None, (
                f"active tool '{name}' is tool file {number} but cobot_setup.yaml tools has no "
                f"entry with that tool_number ({sorted(self._tools.numbers())})"
            )
        return profile, ""

    def _trajectories_ready(self) -> Tuple[bool, str]:
        active = self._prims.active_tool()
        if active is None:
            return False, "no /cobot/state yet"
        number = active[0]
        if self._store.is_in_tool(number):
            return True, ""
        return False, (
            f"teach points are in tool file(s) {sorted(self._store.tool_numbers())}, active tool is "
            f"{number} — run /cobot/convert_trajectories"
        )

    def _readiness_tick(self):
        if self._prims.ready():
            if not self._ready_logged:
                self.get_logger().info("cobot_control primitives ready")
                self._ready_logged = True
        elif self._ready_logged:
            self.get_logger().warn(f"cobot_control primitives lost: {self._prims.missing()}")
            self._ready_logged = False

        # Active-tool watch → automatic conversion of the remembered positions.
        active = self._prims.active_tool()
        if active is None:
            return
        number = active[0]
        if number != self._last_seen_tool:
            if self._last_seen_tool is not None:
                self.get_logger().warn(
                    f"active tool changed: {self._last_seen_tool} -> {number} ({active[1]})"
                )
            self._last_seen_tool = number
        if not self._auto_convert or self._store.is_in_tool(number):
            return
        if self._auto_attempted_for == number:
            return  # one attempt per change; retry via the service
        if not self._prims.convert_ready() or self._busy.locked():
            return
        self._auto_attempted_for = number
        self.get_logger().warn(
            f"remembered positions are in tool file(s) {sorted(self._store.tool_numbers())}; "
            f"converting them to the active tool {number} via the controller kinematics"
        )
        # Fire the async conversion; it takes _busy itself.
        self.executor.create_task(self._convert_all(number))

    def _health_report(self):
        ready = self._prims.ready()
        busy = self._busy.locked()
        active = self._prims.active_tool()
        profile, why_tool = self._resolve_active_tool()
        traj_ready, why_traj = self._trajectories_ready()
        if not ready:
            level, detail = Heartbeat.LEVEL_ERROR, f"waiting for {', '.join(self._prims.missing())}"
        elif self._robot_state in (None, CobotStatus.STATE_DISCONNECTED):
            level, detail = Heartbeat.LEVEL_ERROR, "robot disconnected"
        elif profile is None:
            level, detail = Heartbeat.LEVEL_WARN, why_tool
        elif not traj_ready:
            level, detail = Heartbeat.LEVEL_WARN, why_traj
        else:
            level = Heartbeat.LEVEL_OK
            detail = f"picking · {self._choreo.phase}" if busy else "idle"
        summary = self._store.summary()
        return level, detail, {
            "primitives_ready": ready,
            "busy": busy,
            "paused": self._prims.is_paused(),
            "last_placed": self._choreo.last_placed.name,
            "active_tool_number": active[0] if active else -1,
            "active_tool_name": active[1] if active else "",
            "active_tool_profile": profile.name if profile else "",
            "trajectories_tool_numbers": ",".join(str(n) for n in summary["tool_numbers"]),
            "trajectories_ready": traj_ready,
            "trajectories_count": summary["count"],
            "trajectories_with_joints": summary["with_joints"],
            "convert_ready": self._prims.convert_ready(),
            "last_convert": self._last_convert_message,
        }

    # ------------------------------------------------------------------ pick

    def _on_goal_request(self, goal_request):
        if not self._prims.ready():
            self.get_logger().warn(f"goal rejected: waiting for {self._prims.missing()}")
            return GoalResponse.REJECT
        # Reject only states that genuinely block a pick. Deliberately NOT
        # "require IDLE": /cobot/status is latched and our cached copy lags a
        # few ms behind cobot_control's IDLE publish, so when task_manager
        # fires the next goal right after a pick result, a stale BUSY here
        # used to burn the whole layer (16 goals rejected in ~70 ms). Our own
        # _busy lock serialises picks, and cobot_control rejects a MoveToPose
        # while a foreign motion (e.g. calibration) executes — that surfaces
        # as a clean failed first move, not an unsafe overlap.
        if self._robot_state in (None, CobotStatus.STATE_DISCONNECTED, CobotStatus.STATE_ERROR):
            self.get_logger().warn(
                f"goal rejected: robot status is {self._robot_state} (disconnected/error)"
            )
            return GoalResponse.REJECT
        tool, why = self._resolve_active_tool()
        if tool is None:
            self.get_logger().warn(f"goal rejected: {why}")
            return GoalResponse.REJECT
        traj_ready, why = self._trajectories_ready()
        if not traj_ready:
            self.get_logger().warn(f"goal rejected: {why}")
            return GoalResponse.REJECT
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn("goal rejected: a pick or a conversion is already executing")
            return GoalResponse.REJECT
        self._pick_tool = tool
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
        tool = self._pick_tool or self._tools.none()
        try:
            self.get_logger().info(
                f"pick_can ({can.real_x:.1f}, {can.real_y:.1f}, rz={can.real_rz:.1f}) "
                f"with tool '{tool.name}' (tool file {tool.tool_number})"
            )

            def on_phase(name: str):
                feedback.phase = name
                goal_handle.publish_feedback(feedback)

            report = await self._choreo.pick_and_place(
                can,
                should_abort=lambda: goal_handle.is_cancel_requested,
                on_phase=on_phase,
                tool=tool,
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
            self._pick_tool = None
            self._busy.release()

    # ------------------------------------------------------------------ hide

    def _on_hide_goal_request(self, goal_request):
        if not self._prims.ready():
            self.get_logger().warn(f"hide goal rejected: waiting for {self._prims.missing()}")
            return GoalResponse.REJECT
        if self._robot_state in (None, CobotStatus.STATE_DISCONNECTED, CobotStatus.STATE_ERROR):
            self.get_logger().warn(
                f"hide goal rejected: robot status is {self._robot_state} (disconnected/error)"
            )
            return GoalResponse.REJECT
        if "hide" not in self._store.points:
            self.get_logger().warn(
                "hide goal rejected: no 'hide' teach point configured "
                "(add one via /cobot/set_teach_point)"
            )
            return GoalResponse.REJECT
        traj_ready, why = self._trajectories_ready()
        if not traj_ready:
            self.get_logger().warn(f"hide goal rejected: {why}")
            return GoalResponse.REJECT
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn("hide goal rejected: a pick or a conversion is already executing")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _execute_hide(self, goal_handle):
        result = Hide.Result()
        try:
            pose = self._store.poses()["hide"][0]
            self.get_logger().info(f"hide: moving to {pose}")
            job = await self._prims.move_to_pose(pose)

            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = f"canceled: {job.message}"
                self.get_logger().warn(result.message)
                goal_handle.canceled()
            elif job.ok:
                result.success = True
                result.message = "parked at hide position"
                goal_handle.succeed()
            else:
                result.success = False
                result.message = f"hide move failed: {job.message}"
                self.get_logger().warn(result.message)
                goal_handle.abort()
            return result

        except Exception as e:  # noqa: BLE001 — a failed hide must not kill the node
            self.get_logger().error(f"hide crashed: {e!r}")
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result

        finally:
            self._busy.release()

    # ------------------------------------------------------------------ remembered positions

    async def _convert_all(self, to_tool: int) -> Tuple[bool, str, int]:
        """Convert every stored point to `to_tool` via /cobot/convert_pose.
        All-or-nothing; takes the shared busy lock."""
        if not self._busy.acquire(blocking=False):
            return False, "a pick is executing — convert when idle", 0
        try:
            if not self._prims.convert_ready():
                return False, "/cobot/convert_pose not available", 0
            pending = self._store.pending(to_tool)
            if not pending:
                msg = f"remembered positions already in tool file {to_tool}"
                self._last_convert_message = msg
                return True, msg, 0
            results = {}
            for key, index, tp in pending:
                res = await self._prims.convert_pose(tp.pose, to_tool, tp.joints_deg)
                results[(key, index)] = res
                if not res.ok:
                    self.get_logger().error(f"convert {key}[{index}] {tp.pose}: {res.message}")
                    break  # all-or-nothing: no point converting the rest
            report = self._store.apply_conversion(to_tool, results)
            if report.ok:
                self._choreo.cfg.trajectories = self._store.poses()
                self._choreo.cfg.validate()
                self.get_logger().warn(report.summary() + f" → {self._store.path}")
            else:
                self.get_logger().error(report.summary())
            self._last_convert_message = report.summary()
            return report.ok, report.summary(), report.converted
        except Exception as e:  # noqa: BLE001
            msg = f"conversion crashed: {e!r}"
            self.get_logger().error(msg)
            self._last_convert_message = msg
            return False, msg, 0
        finally:
            self._busy.release()

    async def _on_convert_trajectories(self, request, response):
        to_tool = int(request.to_tool)
        if to_tool < 0:
            active = self._prims.active_tool()
            if active is None:
                response.success = False
                response.message = "no /cobot/state yet — cannot tell which tool is active"
                response.to_tool = -1
                return response
            to_tool = active[0]
        response.to_tool = to_tool
        ok, msg, n = await self._convert_all(to_tool)
        response.success = ok
        response.message = msg
        response.converted = n
        return response

    async def _on_save_teach_point(self, request, response):
        current = self._prims.current_pose_with_joints()
        active = self._prims.active_tool()
        if current is None or active is None:
            response.success = False
            response.message = "robot not connected / no /cobot/state"
            return response
        pose, joints = current
        if pose.tool_number != active[0]:
            response.success = False
            response.message = (
                f"pose on /cobot/state is in tool file {pose.tool_number} but the active tool is "
                f"{active[0]} — wait for the next poll"
            )
            return response
        if not self._store.is_in_tool(active[0]):
            response.success = False
            response.message = (
                f"other teach points are in tool file(s) {sorted(self._store.tool_numbers())}; "
                f"convert them to {active[0]} first"
            )
            return response
        if not self._busy.acquire(blocking=False):
            response.success = False
            response.message = "a pick or a conversion is executing"
            return response
        try:
            tp = self._store.set_point(request.key, int(request.index), pose, joints or None)
            self._choreo.cfg.trajectories = self._store.poses()
            response.success = True
            response.message = f"{request.key}[{request.index}] := {tp.pose}"
            response.tool_number = tp.tool_number
            response.x, response.y, response.z = tp.pose.x, tp.pose.y, tp.pose.z
            response.rx, response.ry, response.rz = tp.pose.rx, tp.pose.ry, tp.pose.rz
            self.get_logger().warn(f"teach point saved: {response.message}")
        except (KeyError, IndexError, ValueError) as e:
            response.success = False
            response.message = str(e)
        finally:
            self._busy.release()
        return response

    async def _on_set_teach_point(self, request, response):
        """Developer dashboard: store a hand-edited pose. joints_deg is
        cleared — the values were typed, not read from the controller."""
        tool = int(request.tool_number)
        if tool < 0:
            active = self._prims.active_tool()
            if active is None:
                response.success = False
                response.message = (
                    "no /cobot/state yet — pass an explicit tool_number to edit offline"
                )
                response.tool_number = -1
                return response
            tool = active[0]
        response.tool_number = tool
        if not self._store.is_in_tool(tool):
            response.success = False
            response.message = (
                f"other teach points are in tool file(s) {sorted(self._store.tool_numbers())}; "
                f"convert them to {tool} first"
            )
            return response
        if not self._busy.acquire(blocking=False):
            response.success = False
            response.message = "a pick or a conversion is executing"
            return response
        try:
            pose = Pose6(
                float(request.x), float(request.y), float(request.z),
                float(request.rx), float(request.ry), float(request.rz),
                tool_number=tool,
            )
            tp = self._store.set_point(request.key, int(request.index), pose, None)
            self._choreo.cfg.trajectories = self._store.poses()
            response.success = True
            response.message = f"{request.key}[{request.index}] := {tp.pose} (edited, joints cleared)"
            self.get_logger().warn(f"teach point edited: {response.message}")
        except (KeyError, IndexError, ValueError) as e:
            response.success = False
            response.message = str(e)
        finally:
            self._busy.release()
        return response


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
