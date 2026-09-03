"""Pick-and-place choreography — the ONLY place the phase order lives.

Ported from the legacy Cobot.pick_and_place (motoman_cpp/retired_python/
cobot.py) against a `Primitives` protocol, so it runs unchanged under the
ROS clients (primitives.RosPrimitives) and under a fake in tests.

Parity with the legacy sequence:
  check -> approach(above, pick) -> grip -> lift(above, check)
  -> assert existance -> read divot / right / left -> place (2 teach points)
  -> gripper off -> assert existance
Deliberate differences:
  * cancel is IMMEDIATE (the node cancels the in-flight move); the
    choreography returns at the next primitive boundary wherever that is.
  * a failed move/gripper/sensor-read job halts the whole choreography at
    the next boundary too (legacy only gated the pick descent; every step
    is now load-bearing — see `aborted()`).
  * pause (cobot_control /cobot/pause) is honoured before every primitive.
  * descents use the tool profile's `descend_profile` (linear by default).
  * the legacy PositionOffset / offset-recovery path is dropped — it was a
    no-op (goto sent the raw pose) and dead code respectively.
  * "no free place slot" is an explicit failure (legacy: UnboundLocalError).
  * every pose is sent with an EXPLICIT tool file number: teach points keep
    the one they were stored with (trajectories.json v2), vision-derived
    pick / approach poses get the active tool's number resolved at goal
    accept — a tool switch can never leak into a running pick.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Protocol, Tuple

from cobot_operation.geometry import PlacePosition, Pose6, ToolProfile
from cobot_operation.report import JobResult, OperationReport

PHASES = ("check", "approach", "grip", "lift", "place")

SENSOR_EXISTANCE = "existance_check"
SENSOR_DIVOT = "divot_check"
SENSOR_RIGHT = "right_complete"
SENSOR_LEFT = "left_complete"
REQUIRED_SENSORS = (SENSOR_EXISTANCE, SENSOR_DIVOT, SENSOR_RIGHT, SENSOR_LEFT)

PICK_ORIENTATION = (180.0, 0.0)  # rx, ry


class Primitives(Protocol):
    async def move_to_pose(self, pose: Pose6, profile: str = "") -> JobResult: ...
    async def set_gripper(self, engaged: bool) -> JobResult: ...
    async def read_sensor(self, name: str) -> Tuple[JobResult, bool]: ...
    async def assert_sensor(
        self, name: str, expected: bool, timeout_s: float
    ) -> Tuple[JobResult, bool]: ...
    async def wait_while_paused(self) -> None: ...


TEACH_COUNTS = {"check": 1, "right": 2, "right_flip": 2, "left": 2, "left_flip": 2}


@dataclass
class PickConfig:
    tool: ToolProfile  # default tool profile (the node passes the ACTIVE one per pick)
    trajectories: dict  # key -> list[Pose6], each Pose6 carrying its tool_number
    sensor_assert_timeout_s: float = 1.0
    required_sensors: Tuple[str, ...] = REQUIRED_SENSORS
    known_sensors: List[str] = field(default_factory=list)  # from cobot_setup.yaml, for validation
    # A raw IO sensor bit is not ground truth for a pick's success: it can be
    # misaligned, bounce, or read stale during a fast cycle. False by default
    # so a mismatched assert_sensor (e.g. existance_check after lift/place)
    # is recorded on the report/UI but never fails the PickCan result on its
    # own — the workflow's post-pick vision depth check (pick_layer) is the
    # actual confirmation. Set true (cobot_params.yaml, bind-mounted, no
    # rebuild) to restore the old gate if a cell needs it.
    sensor_assert_authoritative: bool = False

    def validate(self) -> None:
        for key, want in TEACH_COUNTS.items():
            if key not in self.trajectories:
                raise ValueError(f"trajectories.json is missing '{key}'")
            poses = self.trajectories[key]
            if len(poses) != want:
                raise ValueError(f"trajectories.json '{key}' must hold exactly {want} pose(s)")
            for index, pose in enumerate(poses):
                if not isinstance(pose, Pose6):
                    raise ValueError(f"trajectories.json '{key}'[{index}] must be a Pose6")
        if self.known_sensors:
            missing = [s for s in self.required_sensors if s not in self.known_sensors]
            if missing:
                raise ValueError(
                    f"cobot_setup.yaml io.sensors is missing {missing} "
                    f"(needed by the pick choreography)"
                )


class PickChoreography:
    def __init__(self, prims: Primitives, cfg: PickConfig):
        self.prims = prims
        self.cfg = cfg
        self.last_placed: PlacePosition = PlacePosition.left
        self.phase: str = ""

    # ------------------------------------------------------------------

    def _teach(self, key: str, index: int = 0) -> Pose6:
        """Stored pose, tool_number included (never re-expressed here)."""
        return self.cfg.trajectories[key][index]

    def _select_place(
        self, divot: bool, right_full: bool, left_full: bool
    ) -> Tuple[Optional[List[Pose6]], Optional[PlacePosition]]:
        """Legacy rule, verbatim: alternate sides; skip a full side; the
        divot reading picks the normal vs flipped teach pair."""
        if self.last_placed == PlacePosition.left and not right_full:
            key = "right" if divot else "right_flip"
            return [self._teach(key, 0), self._teach(key, 1)], PlacePosition.right
        if self.last_placed == PlacePosition.right and not left_full:
            key = "left_flip" if divot else "left"
            return [self._teach(key, 0), self._teach(key, 1)], PlacePosition.left
        return None, None

    # ------------------------------------------------------------------

    async def pick_and_place(
        self,
        can,
        should_abort: Optional[Callable[[], bool]] = None,
        on_phase: Optional[Callable[[str], None]] = None,
        tool: Optional[ToolProfile] = None,
    ) -> OperationReport:
        """`can` is anything with real_x / real_y / real_rz (Target).
        `tool` = the ACTIVE tool's profile (defaults to cfg.tool)."""
        report = OperationReport("PICK AND PLACE REPORT")
        tool = tool or self.cfg.tool
        prims = self.prims

        def phase(name: str) -> None:
            self.phase = name
            if on_phase is not None:
                on_phase(name)

        def aborted() -> bool:
            """True on an external cancel OR any primitive failure recorded
            so far (a failed move/gripper/sensor job — see report.add).
            Every primitive call below is followed by `if aborted(): return
            report`, so ANY step failing halts the whole choreography at
            that point instead of running the rest of the pick blind."""
            if should_abort is not None and should_abort():
                report.mark_failed(f"aborted during '{self.phase}'")
                return True
            return not report.overall_ok

        async def move(pose: Pose6, profile: str = "") -> JobResult:
            await prims.wait_while_paused()
            return report.add(await prims.move_to_pose(pose, profile))

        async def gripper(engaged: bool) -> JobResult:
            await prims.wait_while_paused()
            return report.add(await prims.set_gripper(engaged))

        async def assert_sensor(name: str, expected: bool) -> bool:
            job, actual = await prims.assert_sensor(name, expected, self.cfg.sensor_assert_timeout_s)
            report.add(job)
            passed = report.add_sensor_assert(name, expected, actual)
            if not passed and self.cfg.sensor_assert_authoritative:
                report.mark_failed(f"sensor {name}: expected {expected}, got {actual}")
            return passed

        async def read_sensor(name: str) -> bool:
            job, value = await prims.read_sensor(name)
            report.add(job)
            return bool(value)

        # can.real_x/real_y is where the CAN is in robot base coords. The
        # tool tip must land there. For tool file 0 the Python tip offset
        # converts tip -> commanded pose; for a real tool file the offset is
        # zero (setup_config enforces it) and the controller's TCP is the tip.
        # Both poses are tagged with the tool file they are expressed in.
        # (compression_mm is not applied yet — as before.)
        rz = float(can.real_rz)
        rx, ry = PICK_ORIENTATION
        px, py, pz = tool.tip_offset.command_for(can.real_x, can.real_y, tool.pick_tip_z_mm, rz)
        ax, ay, az = tool.tip_offset.command_for(can.real_x, can.real_y, tool.approach_tip_z_mm, rz)
        pick_pose = Pose6(px, py, pz, rx, ry, rz, tool_number=tool.tool_number)
        above_pose = Pose6(ax, ay, az, rx, ry, rz, tool_number=tool.tool_number)
        check_pose = self._teach("check")
        descend = tool.descend_profile

        phase("check")
        await move(check_pose)
        if aborted():
            return report

        phase("approach")
        await move(above_pose)
        if aborted():
            return report
        await move(pick_pose, descend)
        if aborted():
            return report

        phase("grip")
        await gripper(True)
        if aborted():
            return report

        phase("lift")
        await move(above_pose)
        if aborted():
            return report
        await move(check_pose)
        if aborted():
            return report

        await assert_sensor(SENSOR_EXISTANCE, True)
        divot = await read_sensor(SENSOR_DIVOT)
        right_full = await read_sensor(SENSOR_RIGHT)
        left_full = await read_sensor(SENSOR_LEFT)
        if aborted():
            return report

        phase("place")
        trajectory, just_placed = self._select_place(divot, right_full, left_full)
        if trajectory is None:
            # Gripper stays ON, arm parked at check — same observable outcome
            # as the legacy crash, but reported.
            report.mark_failed(
                f"no free place slot (last_placed={self.last_placed.name}, "
                f"right_full={right_full}, left_full={left_full})"
            )
            return report
        self.last_placed = just_placed  # legacy: updated before the moves run

        for index, pose in enumerate(trajectory):
            await move(pose, descend if index == len(trajectory) - 1 else "")
            if aborted():
                return report

        await gripper(False)
        if aborted():
            return report

        await assert_sensor(SENSOR_EXISTANCE, True)  # legacy asserts True here too
        return report
