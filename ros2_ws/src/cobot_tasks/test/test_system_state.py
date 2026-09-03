"""Derivation tests for the read-only capability snapshot — pure, no ROS.

Only the constants-parity test needs the built cobot_interfaces overlay;
everything else runs with plain pytest anywhere.
"""

import pytest

from cobot_tasks.system_state import (
    CAP_MOVE,
    CAP_NONE,
    CAP_OPERATE,
    CAP_SEE,
    RobotSample,
    VisionFacts,
    derive,
)


def robot(**overrides):
    base = dict(
        connected=True, servo_on=True, alarming=False, erroring=False,
        in_hold=False, control_mode=2, paused=False, busy=False, age_s=0.1,
    )
    base.update(overrides)
    return RobotSample(**base)


def vision(**overrides):
    base = dict(
        camera_level="ok", localization_level="ok", detector_level="ok",
        calibrated=True, depth_age_s=0.5, age_s=0.5,
    )
    base.update(overrides)
    return VisionFacts(**base)


def blockers_text(snap):
    return " | ".join(snap.move_blockers + snap.see_blockers)


# ----------------------------------------------------------------------
# capability assembly
# ----------------------------------------------------------------------

def test_both_healthy_is_operate():
    snap = derive(robot(), vision())
    assert snap.capability == CAP_OPERATE
    assert snap.capability_name == "operate"
    assert snap.move_blockers == () and snap.see_blockers == ()
    assert snap.robot_state_fresh and snap.health_fresh
    assert snap.control_mode == "remote"


def test_never_heard_anything_is_none():
    snap = derive(None, VisionFacts.empty())
    assert snap.capability == CAP_NONE
    assert "no fresh /cobot/state" in blockers_text(snap)
    assert "no fresh /health/summary" in blockers_text(snap)
    assert snap.robot_state_age_s == -1.0 and snap.health_age_s == -1.0


def test_vision_down_is_move_only():
    snap = derive(robot(), vision(camera_level="error"))
    assert snap.capability == CAP_MOVE
    assert any("camera stream down" in b for b in snap.see_blockers)
    assert not snap.camera_ok


def test_robot_disconnected_is_see_only():
    snap = derive(robot(connected=False, control_mode=0), vision())
    assert snap.capability == CAP_SEE
    assert snap.move_blockers == ("robot not connected",)
    # control_mode 0 is also the disconnected default — must NOT read as TEACH.
    assert "TEACH" not in blockers_text(snap)
    assert snap.control_mode == "-"


# ----------------------------------------------------------------------
# MOVE predicate
# ----------------------------------------------------------------------

def test_teach_mode_blocks_move():
    snap = derive(robot(control_mode=0), vision())
    assert snap.capability == CAP_SEE
    assert snap.move_blockers == ("pendant in TEACH mode",)
    assert snap.control_mode == "teach"


def test_play_mode_allows_move():
    snap = derive(robot(control_mode=1), vision())
    assert snap.capability == CAP_OPERATE
    assert snap.control_mode == "play"


def test_servos_off_blocks_move():
    snap = derive(robot(servo_on=False), vision())
    assert snap.move_blockers == ("servos off",)


def test_alarming_blocks_move():
    snap = derive(robot(alarming=True), vision())
    assert snap.move_blockers == ("controller alarm active",)
    assert snap.alarming


def test_multiple_move_blockers_all_reported():
    snap = derive(robot(alarming=True, servo_on=False, control_mode=0), vision())
    assert len(snap.move_blockers) == 3


def test_busy_does_not_block_move():
    # A running pick must not drop capability mid-workflow.
    snap = derive(robot(busy=True), vision())
    assert snap.capability == CAP_OPERATE
    assert snap.busy


@pytest.mark.parametrize("field", ["paused", "erroring", "in_hold"])
def test_soft_states_do_not_block_move(field):
    # cobot_control accepts goals in these states — capability must not be
    # stricter than the executor. They are still reported as facts.
    snap = derive(robot(**{field: True}), vision())
    assert snap.capability == CAP_OPERATE
    assert getattr(snap, field) is True


def test_stale_robot_sample_blocks_move_and_greys_facts():
    snap = derive(robot(age_s=2.5), vision())
    assert snap.capability == CAP_SEE
    assert not snap.robot_state_fresh
    assert not snap.connected and not snap.servo_on
    assert snap.control_mode == "-"
    assert "no fresh /cobot/state" in snap.move_blockers[0]


# ----------------------------------------------------------------------
# SEE predicate
# ----------------------------------------------------------------------

def test_stale_health_summary_blocks_see():
    snap = derive(robot(), vision(age_s=10.0))
    assert snap.capability == CAP_MOVE
    assert not snap.health_fresh
    assert "no fresh /health/summary" in snap.see_blockers[0]


def test_localization_stale_blocks_see_despite_calibrated_metric():
    # NodeHealth keeps old metrics through STALE — a dead localization must
    # not keep advertising calibrated=true.
    snap = derive(robot(), vision(localization_level="stale", calibrated=True))
    assert snap.capability == CAP_MOVE
    assert any("localization_node stale" in b for b in snap.see_blockers)


def test_not_calibrated_blocks_see():
    snap = derive(robot(), vision(calibrated=False))
    assert snap.capability == CAP_MOVE
    assert any("calibration missing" in b for b in snap.see_blockers)
    assert not snap.calibrated


def test_missing_health_rows_block_see():
    snap = derive(robot(), vision(camera_level=None, localization_level=None))
    assert snap.capability == CAP_MOVE
    assert any("missing" in b for b in snap.see_blockers)


def test_localization_warn_still_sees():
    # WARN = stale depth / no intrinsics: transient, the Scan action fails
    # cleanly on its own — do not flap readiness over it.
    snap = derive(robot(), vision(localization_level="warn", depth_age_s=5.0))
    assert snap.capability == CAP_OPERATE
    assert not snap.depth_fresh  # still visible on the dashboard


def test_camera_warn_still_sees():
    snap = derive(robot(), vision(camera_level="warn"))
    assert snap.capability == CAP_OPERATE


def test_depth_and_detector_are_informational():
    snap = derive(robot(), vision(depth_age_s=None, detector_level="stale"))
    assert snap.capability == CAP_OPERATE
    assert not snap.depth_fresh and not snap.detector_ok


# ----------------------------------------------------------------------
# publish-on-change key
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
# tooling facts (warning, never a blocker)
# ----------------------------------------------------------------------

def test_calibration_tool_mismatch_is_warning_not_blocker():
    snap = derive(robot(active_tool_number=1, active_tool_name="vacuum_head"),
                  vision(calibration_tool_number=0))
    assert snap.capability == CAP_OPERATE
    assert snap.calibration_tool_mismatch
    assert snap.active_tool_number == 1 and snap.active_tool_name == "vacuum_head"
    assert snap.calibration_tool_number == 0
    assert blockers_text(snap) == ""


def test_matching_or_unknown_calibration_tool_is_not_a_mismatch():
    assert not derive(robot(active_tool_number=1), vision(calibration_tool_number=1)).calibration_tool_mismatch
    unknown = derive(robot(active_tool_number=1), vision())  # pre-tooling calibration.yaml
    assert unknown.calibration_tool_number == -1 and not unknown.calibration_tool_mismatch


def test_stale_robot_greys_tooling_facts():
    snap = derive(robot(active_tool_number=1, age_s=5.0), vision(calibration_tool_number=0))
    assert snap.active_tool_number == -1 and snap.active_tool_name == ""
    assert not snap.calibration_tool_mismatch


def test_stale_health_hides_calibration_tool():
    snap = derive(robot(active_tool_number=1), vision(calibration_tool_number=0, age_s=30.0))
    assert snap.calibration_tool_number == -1 and not snap.calibration_tool_mismatch


def test_content_ignores_raw_ages():
    a = derive(robot(age_s=0.1), vision(age_s=0.2))
    b = derive(robot(age_s=0.6), vision(age_s=1.2))
    assert a != b
    assert a.content() == b.content()
    c = derive(robot(servo_on=False), vision())
    assert a.content() != c.content()


# ----------------------------------------------------------------------
# parity with the generated message
# ----------------------------------------------------------------------

def test_constants_match_msg():
    msg = pytest.importorskip("cobot_interfaces.msg")
    state = msg.SystemState
    assert (state.CAP_NONE, state.CAP_SEE, state.CAP_MOVE, state.CAP_OPERATE) == (
        CAP_NONE, CAP_SEE, CAP_MOVE, CAP_OPERATE,
    )
    assert CAP_OPERATE == CAP_SEE | CAP_MOVE
