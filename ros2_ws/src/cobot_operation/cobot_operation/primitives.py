"""RosPrimitives — the choreography's view of cobot_control.

Every method is a coroutine that awaits rclpy futures directly (the node runs
its execute callback as an `async def` on a MultiThreadedExecutor with ONE
reentrant callback group). Nothing here blocks a thread and nothing raises
into the choreography: transport problems (server gone, goal rejected,
timeout) become JobResult(ok=False, status_code=-1).

Reads come from the /cobot/state poll (cobot_control publishes every named
sensor / output at 10 Hz), not from services.
"""

from __future__ import annotations

from typing import Callable, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.task import Future

from cobot_interfaces.action import MoveToPose
from cobot_interfaces.msg import CobotState
from cobot_interfaces.srv import ConvertPose, SetOutputBit

from cobot_operation.geometry import Pose6
from cobot_operation.report import JobResult
from cobot_operation.trajectory_store import ConvertResult

GOAL_ACK_TIMEOUT_S = 5.0


class RosPrimitives:
    def __init__(
        self,
        node: Node,
        group: CallbackGroup,
        *,
        move_timeout_s: float = 120.0,
        service_timeout_s: float = 5.0,
        gripper_name: str = "gripper",
        state_timeout_s: float = 1.0,
    ):
        self._node = node
        self._group = group
        self._move_timeout = float(move_timeout_s)
        self._service_timeout = float(service_timeout_s)
        self._gripper_name = gripper_name
        self._state_timeout = float(state_timeout_s)

        self._move = ActionClient(node, MoveToPose, "/cobot/move_to_pose", callback_group=group)
        self._set_output = node.create_client(
            SetOutputBit, "/cobot/set_output_bit", callback_group=group
        )
        # Controller kinematics for re-expressing remembered poses in another
        # tool file. Deliberately NOT part of ready(): a pick never needs it.
        self._convert = node.create_client(
            ConvertPose, "/cobot/convert_pose", callback_group=group
        )
        node.create_subscription(
            CobotState, "/cobot/state", self._on_state, 10, callback_group=group
        )

        self._state: Optional[CobotState] = None
        self._state_waiters: List[Future] = []
        self._inflight = None  # ClientGoalHandle of the running MoveToPose

    # ------------------------------------------------------------------
    # readiness / state
    # ------------------------------------------------------------------

    def ready(self) -> bool:
        return (
            self._move.server_is_ready()
            and self._set_output.service_is_ready()
            and self._state is not None
        )

    def missing(self) -> List[str]:
        out = []
        if not self._move.server_is_ready():
            out.append("/cobot/move_to_pose")
        if not self._set_output.service_is_ready():
            out.append("/cobot/set_output_bit")
        if self._state is None:
            out.append("/cobot/state")
        return out

    @property
    def state(self) -> Optional[CobotState]:
        return self._state

    def _on_state(self, msg: CobotState) -> None:
        self._state = msg
        waiters, self._state_waiters = self._state_waiters, []
        for fut in waiters:
            if not fut.done():
                fut.set_result(msg)

    def sensors(self) -> dict:
        s = self._state
        if s is None:
            return {}
        return dict(zip(s.sensor_names, s.sensor_values))

    def outputs(self) -> dict:
        s = self._state
        if s is None:
            return {}
        return dict(zip(s.output_names, s.output_values))

    def is_paused(self) -> bool:
        return bool(self._state is not None and self._state.paused)

    def convert_ready(self) -> bool:
        return self._convert.service_is_ready()

    def active_tool(self) -> Optional[Tuple[int, str]]:
        """(active_tool_number, active_tool_name) from the last /cobot/state."""
        s = self._state
        if s is None:
            return None
        return int(s.active_tool_number), str(s.active_tool_name)

    def current_pose_with_joints(self) -> Optional[Tuple[Pose6, list]]:
        """Current TCP pose (tagged with pose_tool_number) + joints, or None
        when the robot is not connected / no state yet."""
        s = self._state
        if s is None or not s.connected:
            return None
        pose = Pose6(s.x, s.y, s.z, s.rx, s.ry, s.rz, tool_number=int(s.pose_tool_number))
        return pose, [float(v) for v in s.joints_deg]

    # ------------------------------------------------------------------
    # awaiting helpers (no asyncio loop here — rclpy futures + timers)
    # ------------------------------------------------------------------

    async def _await(self, fut: Future, timeout_s: float):
        """Await `fut` for at most timeout_s. Returns fut.result() or None."""
        done = Future()

        def on_done(_):
            if not done.done():
                done.set_result(True)

        fut.add_done_callback(on_done)
        timer = self._node.create_timer(
            max(timeout_s, 0.01),
            lambda: (not done.done()) and done.set_result(False),
            callback_group=self._group,
        )
        try:
            ok = await done
        finally:
            timer.cancel()
            self._node.destroy_timer(timer)
        if not ok or not fut.done():
            return None
        try:
            return fut.result()
        except Exception:  # noqa: BLE001 — surfaced as a transport failure by the caller
            return None

    async def next_state(self, timeout_s: float) -> Optional[CobotState]:
        fut = Future()
        self._state_waiters.append(fut)
        return await self._await(fut, timeout_s)

    async def sleep(self, seconds: float) -> None:
        await self._await(Future(), seconds)

    async def wait_while_paused(self) -> None:
        while self.is_paused():
            await self.next_state(0.5)

    # ------------------------------------------------------------------
    # primitives
    # ------------------------------------------------------------------

    async def move_to_pose(self, pose: Pose6, profile: str = "") -> JobResult:
        goal = MoveToPose.Goal()
        goal.x, goal.y, goal.z = float(pose.x), float(pose.y), float(pose.z)
        goal.rx, goal.ry, goal.rz = float(pose.rx), float(pose.ry), float(pose.rz)
        goal.motion_profile = profile or ""
        goal.tool_number = int(pose.tool_number)  # -1 = active tool (cobot_control resolves)
        payload = f"{pose}" + (f" [{profile}]" if profile else "")

        if not self._move.server_is_ready():
            return JobResult.transport("move", "/cobot/move_to_pose not available", payload)

        handle = await self._await(self._move.send_goal_async(goal), GOAL_ACK_TIMEOUT_S)
        if handle is None:
            return JobResult.transport("move", "move goal not acknowledged", payload)
        if not handle.accepted:
            return JobResult.transport(
                "move", "move goal rejected by cobot_control (busy / not ready)", payload
            )

        self._inflight = handle
        try:
            wrapped = await self._await(handle.get_result_async(), self._move_timeout)
        finally:
            self._inflight = None

        if wrapped is None:
            handle.cancel_goal_async()  # best effort; the node stops the arm
            return JobResult.transport(
                "move", f"move timed out after {self._move_timeout:.0f} s", payload
            )
        res = wrapped.result
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            return JobResult("move", False, 0, f"canceled: {res.message}", payload)
        return JobResult(
            "move", bool(res.success), 0 if res.success else int(res.status_code),
            res.message, payload,
        )

    async def convert_pose(self, pose: Pose6, to_tool: int, joints_deg=None) -> ConvertResult:
        """Ask cobot_control to re-express `pose` (in pose.tool_number) in
        tool file `to_tool` through the controller kinematics. Transport
        problems become ConvertResult(ok=False)."""
        if not self._convert.service_is_ready():
            return ConvertResult(False, message="/cobot/convert_pose not available")
        req = ConvertPose.Request()
        req.x, req.y, req.z = float(pose.x), float(pose.y), float(pose.z)
        req.rx, req.ry, req.rz = float(pose.rx), float(pose.ry), float(pose.rz)
        req.from_tool = int(pose.tool_number)
        req.to_tool = int(to_tool)
        req.joints_deg = [float(v) for v in joints_deg] if joints_deg else []
        res = await self._await(self._convert.call_async(req), self._service_timeout)
        if res is None:
            return ConvertResult(False, message="convert_pose timed out")
        if not res.success:
            return ConvertResult(False, message=f"[{res.status_code}] {res.message}")
        out = Pose6(res.x, res.y, res.z, res.rx, res.ry, res.rz, tool_number=int(res.to_tool))
        joints = tuple(float(v) for v in res.joints_deg) if len(res.joints_deg) == 6 else None
        return ConvertResult(True, pose=out, joints_deg=joints, message=res.message)

    def cancel_inflight(self) -> bool:
        """Fire-and-forget cancel of the running move (called from the
        PickCan cancel callback so the arm stops immediately)."""
        handle = self._inflight
        if handle is None:
            return False
        handle.cancel_goal_async()
        return True

    async def set_gripper(self, engaged: bool) -> JobResult:
        job = "engage_gripper" if engaged else "disengage_gripper"
        payload = f"{self._gripper_name}={engaged}"
        if not self._set_output.service_is_ready():
            return JobResult.transport(job, "/cobot/set_output_bit not available", payload)
        req = SetOutputBit.Request()
        req.name = self._gripper_name
        req.value = bool(engaged)
        res = await self._await(self._set_output.call_async(req), self._service_timeout)
        if res is None:
            return JobResult.transport(job, "set_output_bit timed out", payload)
        if not res.success:
            return JobResult(job, False, int(res.status_code), res.message, payload)
        # Let the next poll sample land so following reads are fresh, and
        # note the controller's own read-back of the output.
        msg = res.message
        state = await self.next_state(self._state_timeout)
        if state is not None:
            readback = dict(zip(state.output_names, state.output_values)).get(self._gripper_name)
            if readback is not None and readback != engaged:
                msg += f" (controller read-back still {readback})"
        return JobResult(job, True, 0, msg, payload)

    async def read_sensor(self, name: str) -> Tuple[JobResult, bool]:
        state = self._state
        if state is None:
            return JobResult.transport("sensor_read", "no /cobot/state yet", name), False
        if not state.connected:
            return JobResult("sensor_read", False, 1, "robot not connected", name), False
        values = dict(zip(state.sensor_names, state.sensor_values))
        if name not in values:
            return (
                JobResult("sensor_read", False, 2, f"unknown sensor '{name}' (cobot_setup.yaml)", name),
                False,
            )
        value = bool(values[name])
        return JobResult("sensor_read", True, 0, f"{name}={value}", name), value

    async def assert_sensor(
        self, name: str, expected: bool, timeout_s: float
    ) -> Tuple[JobResult, bool]:
        """Wait up to timeout_s for a /cobot/state sample with sensor == expected.
        Replaces the legacy 5 x 0.2 s re-read loop."""
        deadline = self._node.get_clock().now().nanoseconds / 1e9 + max(timeout_s, 0.0)
        job, value = await self.read_sensor(name)
        while job.ok and value != expected:
            remaining = deadline - self._node.get_clock().now().nanoseconds / 1e9
            if remaining <= 0.0:
                break
            if await self.next_state(remaining) is None:
                break
            job, value = await self.read_sensor(name)
        if job.ok:
            job = JobResult("sensor_assert", True, 0, f"{name}={value} (expected {expected})", name)
        return job, value


PhaseCallback = Callable[[str], None]
