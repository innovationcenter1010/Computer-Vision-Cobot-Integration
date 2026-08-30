"""Choreography tests against a fake Primitives — no rclpy, no robot."""

import asyncio
from types import SimpleNamespace

import pytest

from cobot_operation.choreography import (
    SENSOR_DIVOT,
    SENSOR_EXISTANCE,
    SENSOR_LEFT,
    SENSOR_RIGHT,
    PickChoreography,
    PickConfig,
)
from cobot_operation.geometry import PlacePosition, Pose6, ToolOffset, ToolProfile
from cobot_operation.report import JobResult

TRAJECTORIES = {
    "check": [0, -925, 750, 180, 0, -90],
    "right": [[500, -900, 700, 180, 0, 0], [500, -900, 400, 180, 0, 0]],
    "right_flip": [[510, -900, 700, 180, 0, 0], [510, -900, 400, 180, 0, 0]],
    "left": [[-500, -900, 700, 180, 0, 0], [-500, -900, 400, 180, 0, 0]],
    "left_flip": [[-510, -900, 700, 180, 0, 0], [-510, -900, 400, 180, 0, 0]],
}

CAN = SimpleNamespace(real_x=100.0, real_y=-800.0, real_rz=30.0)


class FakePrimitives:
    def __init__(self, sensors=None, move_ok=None):
        self.calls = []
        self.sensors = sensors or {
            SENSOR_EXISTANCE: True, SENSOR_DIVOT: True, SENSOR_RIGHT: False, SENSOR_LEFT: False,
        }
        self.move_ok = list(move_ok or [])  # per-move outcomes; default True
        self.gripper = None
        self.abort_after_calls = None  # set by tests to flip an abort flag mid-run
        self.abort_flag = False

    def _tick(self):
        if self.abort_after_calls is not None and len(self.calls) >= self.abort_after_calls:
            self.abort_flag = True

    async def move_to_pose(self, pose: Pose6, profile: str = "") -> JobResult:
        self.calls.append(("move", pose, profile))
        ok = self.move_ok.pop(0) if self.move_ok else True
        self._tick()
        return JobResult("move", ok, 0 if ok else 99, "ok" if ok else "controller said no", str(pose))

    async def set_gripper(self, engaged: bool) -> JobResult:
        self.calls.append(("gripper", engaged))
        self.gripper = engaged
        self._tick()
        return JobResult("engage_gripper" if engaged else "disengage_gripper", True, 0, "ok", "")

    async def read_sensor(self, name: str):
        self.calls.append(("read", name))
        value = self.sensors[name]
        return JobResult("sensor_read", True, 0, f"{name}={value}", name), value

    async def assert_sensor(self, name: str, expected: bool, timeout_s: float):
        self.calls.append(("assert", name, expected))
        value = self.sensors[name]
        return JobResult("sensor_assert", True, 0, f"{name}={value}", name), value

    async def wait_while_paused(self) -> None:
        return None


def make(prims, tip_z=0.0, descend="descend"):
    tool = ToolProfile(
        name="t", tip_offset=ToolOffset(0.0, 0.0, tip_z), pick_tip_z_mm=300.0,
        approach_tip_z_mm=401.0, descend_profile=descend,
    )
    cfg = PickConfig(tool=tool, trajectories=TRAJECTORIES)
    cfg.validate()
    return PickChoreography(prims, cfg)


def run(choreo, prims, phases=None):
    return asyncio.run(
        choreo.pick_and_place(
            CAN, should_abort=lambda: prims.abort_flag,
            on_phase=(phases.append if phases is not None else None),
        )
    )


def moves(prims):
    return [c for c in prims.calls if c[0] == "move"]


# ----------------------------------------------------------------------


def test_happy_path_phase_order_and_call_sequence():
    prims = FakePrimitives()
    choreo = make(prims)
    phases = []
    report = run(choreo, prims, phases)

    assert report.overall_ok, str(report)
    assert phases == ["check", "approach", "grip", "lift", "place"]

    kinds = [c[0] for c in prims.calls]
    assert kinds == [
        "move",            # check
        "move", "move",    # approach: above, pick
        "gripper",         # grip on
        "move", "move",    # lift: above, check
        "assert",          # existance
        "read", "read", "read",  # divot, right, left
        "move", "move",    # place pair
        "gripper",         # release
        "assert",          # existance
    ]
    assert prims.calls[3] == ("gripper", True)
    assert prims.calls[-2] == ("gripper", False)

    mv = moves(prims)
    check = Pose6.from_list(TRAJECTORIES["check"])
    assert mv[0][1] == check and mv[4][1] == check
    # approach: above then pick, at the tool's heights, rz from the can
    assert mv[1][1] == Pose6(100.0, -800.0, 401.0, 180.0, 0.0, 30.0)
    assert mv[2][1] == Pose6(100.0, -800.0, 300.0, 180.0, 0.0, 30.0)
    # descents use the descend profile, everything else the default
    assert [m[2] for m in mv] == ["", "", "descend", "", "", "", "descend"]
    # last_placed was left, right side free, divot -> "right"
    assert mv[5][1] == Pose6.from_list(TRAJECTORIES["right"][0])
    assert mv[6][1] == Pose6.from_list(TRAJECTORIES["right"][1])
    assert choreo.last_placed == PlacePosition.right


def test_tool_tip_offset_applies_to_vision_poses_only():
    prims = FakePrimitives()
    choreo = make(prims, tip_z=-600.0)
    run(choreo, prims)
    mv = moves(prims)
    assert mv[1][1].z == 1001.0 and mv[2][1].z == 900.0   # tip heights + 600
    assert mv[0][1] == Pose6.from_list(TRAJECTORIES["check"])  # teach point raw


@pytest.mark.parametrize(
    "last, divot, right_full, left_full, expected_key, expected_side",
    [
        (PlacePosition.left, True, False, False, "right", PlacePosition.right),
        (PlacePosition.left, False, False, False, "right_flip", PlacePosition.right),
        (PlacePosition.right, False, False, False, "left", PlacePosition.left),
        (PlacePosition.right, True, False, False, "left_flip", PlacePosition.left),
    ],
)
def test_place_selection_matrix(last, divot, right_full, left_full, expected_key, expected_side):
    prims = FakePrimitives(sensors={
        SENSOR_EXISTANCE: True, SENSOR_DIVOT: divot, SENSOR_RIGHT: right_full, SENSOR_LEFT: left_full,
    })
    choreo = make(prims)
    choreo.last_placed = last
    report = run(choreo, prims)
    assert report.overall_ok
    mv = moves(prims)
    assert mv[5][1] == Pose6.from_list(TRAJECTORIES[expected_key][0])
    assert choreo.last_placed == expected_side


def test_no_free_slot_fails_and_keeps_gripper_on():
    # last placed left, right side full -> no branch matches
    prims = FakePrimitives(sensors={
        SENSOR_EXISTANCE: True, SENSOR_DIVOT: True, SENSOR_RIGHT: True, SENSOR_LEFT: False,
    })
    choreo = make(prims)
    report = run(choreo, prims)
    assert not report.overall_ok
    assert "no free place slot" in report.first_failure()
    assert prims.gripper is True
    assert len(moves(prims)) == 5  # no place moves
    assert choreo.last_placed == PlacePosition.left


def test_failed_descent_gates_the_grip():
    prims = FakePrimitives(move_ok=[True, True, False])
    choreo = make(prims)
    phases = []
    report = run(choreo, prims, phases)
    assert not report.overall_ok
    assert report.first_failure() == "move: controller said no"
    assert any(f.startswith("approach descent failed") for f in report.failures)
    assert phases == ["check", "approach"]
    assert prims.gripper is None
    assert not any(c[0] == "gripper" for c in prims.calls)


def test_failed_lift_move_is_recorded_but_run_continues():
    # legacy parity: only the descent is gated
    prims = FakePrimitives(move_ok=[True, True, True, False])
    choreo = make(prims)
    report = run(choreo, prims)
    assert not report.overall_ok
    assert len(moves(prims)) == 7
    assert [j.ok for j in report.jobs if j.job == "move"] == [True, True, True, False, True, True, True]


@pytest.mark.parametrize(
    "after_calls, expected_phases, expect_gripper",
    [
        (1, ["check"], None),                  # abort after the check move
        (2, ["check", "approach"], None),      # abort after the approach move
        (4, ["check", "approach", "grip"], True),   # cancel right after grip: immediate now
        (6, ["check", "approach", "grip", "lift"], True),
    ],
)
def test_abort_stops_at_next_boundary(after_calls, expected_phases, expect_gripper):
    prims = FakePrimitives()
    prims.abort_after_calls = after_calls
    choreo = make(prims)
    phases = []
    report = run(choreo, prims, phases)
    assert not report.overall_ok
    assert report.first_failure().startswith("aborted during")
    assert phases == expected_phases
    assert prims.gripper is expect_gripper
    assert len(prims.calls) == after_calls  # nothing issued after the abort


def test_sensor_assert_failure_marks_report():
    prims = FakePrimitives(sensors={
        SENSOR_EXISTANCE: False, SENSOR_DIVOT: True, SENSOR_RIGHT: False, SENSOR_LEFT: False,
    })
    choreo = make(prims)
    report = run(choreo, prims)
    assert not report.overall_ok
    assert any(not a.passed for a in report.sensor_asserts)
    assert "existance_check" in report.first_failure()


def test_config_validation_reports_missing_pieces():
    tool = ToolProfile(name="t")
    with pytest.raises(ValueError, match="missing 'left_flip'"):
        PickConfig(tool=tool, trajectories={k: v for k, v in TRAJECTORIES.items() if k != "left_flip"}).validate()
    with pytest.raises(ValueError, match="io.sensors is missing"):
        PickConfig(tool=tool, trajectories=TRAJECTORIES, known_sensors=["existance_check"]).validate()
