"""Derived system capability — the pure math behind /task/system_state.

task_manager continuously aggregates what other nodes PUBLISH — /cobot/state
(10 Hz) and /health/summary (1 Hz latched) — into one read-only snapshot:
individual facts (connected, servos, YRC1000 control mode, alarming, camera,
calibration, ...) plus an overall capability. The task planner never sets or
changes any of it.

Capability is a BITMASK with exactly four values:

    CAP_NONE    = 0
    CAP_SEE     = 1     vision usable (camera + calibration), robot not needed
    CAP_MOVE    = 2     robot motion usable, vision not needed
    CAP_OPERATE = 3     SEE | MOVE

so a workflow prerequisite is one expression: (capability & requires) == requires.

MOVE mirrors cobot_control's own goal gate (connected, no alarm, servos on,
control mode not TEACH) MINUS busy — capability describes what the system CAN
do, and a pick in progress must not drop it. erroring / in_hold / paused are
reported as facts but are NOT blockers: cobot_control accepts goals in those
states, and the capability must never be stricter than the executor.

SEE requires a fresh health summary, camera_node ok/warn, localization_node
ok/warn (stale/unknown block — NodeHealth keeps old metrics through STALE, so
a dead localization would otherwise advertise calibrated=true forever) and
the fitted calibration. localization WARN (stale depth / no intrinsics) does
not block: transient and self-healing; the Scan action fails cleanly itself.

Deliberately stdlib-only (no rclpy, no cobot_interfaces) so the tests in
test/test_system_state.py run anywhere; a parity test pins the CAP_* values
to SystemState.msg.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass

# Mirrored by cobot_interfaces/msg/SystemState.msg (parity-tested).
CAP_NONE = 0
CAP_SEE = 1
CAP_MOVE = 2
CAP_OPERATE = CAP_SEE | CAP_MOVE

CAPABILITY_NAMES = {
    CAP_NONE: "none",
    CAP_SEE: "see",
    CAP_MOVE: "move",
    CAP_OPERATE: "operate",
}

# Freshness thresholds.
ROBOT_STALE_SEC = 1.0    # /cobot/state is 10 Hz — 1 s means 10 missed polls
HEALTH_STALE_SEC = 3.0   # /health/summary is 1 Hz; matches monitor stale_factor
DEPTH_STALE_SEC = 2.0    # camera_node's own stall threshold (informational)

# CobotState.msg control_mode constants (0 is also the disconnected default,
# which is why connectivity is checked before mode).
CONTROL_MODE_TEACH = 0
CONTROL_MODE_NAMES = {0: "teach", 1: "play", 2: "remote"}

# NodeHealth levels that mean "the node is alive and its metrics are current".
LIVE_LEVELS = ("ok", "warn")


@dataclass(frozen=True)
class RobotSample:
    """The newest /cobot/state, plus how old it is."""

    connected: bool
    servo_on: bool
    alarming: bool
    erroring: bool
    in_hold: bool
    control_mode: int
    paused: bool
    busy: bool
    age_s: float
    active_tool_number: int = -1   # CobotState.active_tool_number (-1 = unknown)
    active_tool_name: str = ""


@dataclass(frozen=True)
class VisionFacts:
    """Vision-side rows of the newest /health/summary. Levels are the
    NodeHealth names ("ok"/"warn"/"error"/"stale"/"unknown"); None = the row
    was absent. age_s None = never heard a summary."""

    camera_level: str | None
    localization_level: str | None
    detector_level: str | None
    calibrated: bool | None
    depth_age_s: float | None
    age_s: float | None
    # Tool file the saved calibration was captured with (localization metric
    # calibration_tool_number); None = unknown / pre-tooling file.
    calibration_tool_number: int | None = None

    @classmethod
    def empty(cls) -> "VisionFacts":
        return cls(None, None, None, None, None, None)


@dataclass(frozen=True)
class SystemSnapshot:
    capability: int
    capability_name: str
    # robot facts (all False when not fresh — grey them, don't trust them)
    robot_state_fresh: bool
    connected: bool
    servo_on: bool
    alarming: bool
    erroring: bool
    in_hold: bool
    paused: bool
    busy: bool
    control_mode: str        # "teach"/"play"/"remote"; "-" when unknown
    robot_state_age_s: float  # -1 = never heard
    # vision facts (all False when not fresh)
    health_fresh: bool
    camera_ok: bool
    calibrated: bool
    depth_fresh: bool
    detector_ok: bool
    health_age_s: float       # -1 = never heard
    move_blockers: tuple[str, ...]
    see_blockers: tuple[str, ...]
    # tooling facts — a mismatch is a WARNING (the fit is base-frame and
    # tool-independent), never a blocker
    active_tool_number: int = -1
    active_tool_name: str = ""
    calibration_tool_number: int = -1
    calibration_tool_mismatch: bool = False

    def content(self) -> "SystemSnapshot":
        """Copy with the raw ages neutralized — equality on THIS decides
        whether a republish is needed (the ages tick on every refresh; the
        freshness booleans already capture what matters)."""
        return dataclasses.replace(self, robot_state_age_s=-1.0, health_age_s=-1.0)


def derive(robot: RobotSample | None, vision: VisionFacts) -> SystemSnapshot:
    # ---- MOVE side --------------------------------------------------
    move_blockers: list[str] = []
    fresh = robot is not None and robot.age_s <= ROBOT_STALE_SEC
    if not fresh:
        move_blockers.append("no fresh /cobot/state — is cobot_control running?")
    elif not robot.connected:
        move_blockers.append("robot not connected")
    else:
        if robot.alarming:
            move_blockers.append("controller alarm active")
        if not robot.servo_on:
            move_blockers.append("servos off")
        if robot.control_mode == CONTROL_MODE_TEACH:
            move_blockers.append("pendant in TEACH mode")

    if fresh:
        connected = robot.connected
        servo_on = robot.servo_on
        alarming = robot.alarming
        erroring = robot.erroring
        in_hold = robot.in_hold
        paused = robot.paused
        busy = robot.busy
        control_mode = (
            CONTROL_MODE_NAMES.get(robot.control_mode, "?") if robot.connected else "-"
        )
    else:
        connected = servo_on = alarming = erroring = in_hold = paused = busy = False
        control_mode = "-"

    # ---- SEE side ---------------------------------------------------
    see_blockers: list[str] = []
    health_fresh = vision.age_s is not None and vision.age_s <= HEALTH_STALE_SEC
    if not health_fresh:
        see_blockers.append("no fresh /health/summary — is health_monitor running?")
        camera_ok = calibrated = depth_fresh = detector_ok = False
    else:
        camera_ok = vision.camera_level in LIVE_LEVELS
        if not camera_ok:
            see_blockers.append(
                f"camera stream down (camera_node {vision.camera_level or 'missing'})"
            )
        if vision.localization_level not in LIVE_LEVELS:
            see_blockers.append(
                f"localization_node {vision.localization_level or 'missing'}"
            )
        calibrated = vision.calibrated is True
        if not calibrated:
            see_blockers.append("camera→robot calibration missing")
        depth_fresh = vision.depth_age_s is not None and vision.depth_age_s <= DEPTH_STALE_SEC
        detector_ok = vision.detector_level in LIVE_LEVELS

    capability = (CAP_SEE if not see_blockers else 0) | (
        CAP_MOVE if not move_blockers else 0
    )

    # ---- tooling facts ---------------------------------------------
    active_tool_number = robot.active_tool_number if fresh else -1
    active_tool_name = robot.active_tool_name if fresh else ""
    calibration_tool_number = (
        -1 if not health_fresh or vision.calibration_tool_number is None
        else int(vision.calibration_tool_number)
    )
    calibration_tool_mismatch = (
        fresh and robot.connected and active_tool_number >= 0
        and calibration_tool_number >= 0
        and active_tool_number != calibration_tool_number
    )

    return SystemSnapshot(
        capability=capability,
        capability_name=CAPABILITY_NAMES[capability],
        robot_state_fresh=fresh,
        connected=connected,
        servo_on=servo_on,
        alarming=alarming,
        erroring=erroring,
        in_hold=in_hold,
        paused=paused,
        busy=busy,
        control_mode=control_mode,
        robot_state_age_s=-1.0 if robot is None else float(robot.age_s),
        health_fresh=health_fresh,
        camera_ok=camera_ok,
        calibrated=calibrated,
        depth_fresh=depth_fresh,
        detector_ok=detector_ok,
        health_age_s=-1.0 if vision.age_s is None else float(vision.age_s),
        move_blockers=tuple(move_blockers),
        see_blockers=tuple(see_blockers),
        active_tool_number=int(active_tool_number),
        active_tool_name=str(active_tool_name),
        calibration_tool_number=int(calibration_tool_number),
        calibration_tool_mismatch=bool(calibration_tool_mismatch),
    )
