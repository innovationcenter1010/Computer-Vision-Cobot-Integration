"""Workflow tests against a fake WorkflowContext — no rclpy, no robot.

Needs the built cobot_interfaces overlay sourced (the workflows build real
goal / report messages), same as cobot_operation's tests."""

from types import SimpleNamespace

import pytest

pytest.importorskip("cobot_interfaces")

from cobot_interfaces.msg import PickEvent, TaskStatus  # noqa: E402

from cobot_tasks.system_state import CAP_MOVE, CAP_OPERATE, CAP_SEE  # noqa: E402
from cobot_tasks.workflows import REGISTRY, WORKFLOWS, Workflow, validate_registry  # noqa: E402
from cobot_tasks.workflows.calibration import (  # noqa: E402
    AutoCalibrationWorkflow,
    AutoStartupCalibrationWorkflow,
    CalibrationWorkflow,
    StartupCalibrationWorkflow,
    spread,
)
from cobot_tasks.workflows.pick_layer import PickLayerWorkflow  # noqa: E402


def target(x, y, idx):
    from cobot_interfaces.msg import Target
    return Target(real_x=float(x), real_y=float(y), real_z=0.0, real_rz=0.0, detection_index=idx)


def scan_result(targets, success=True, message="ok"):
    return SimpleNamespace(success=success, message=message,
                           targets=SimpleNamespace(targets=list(targets)))


def ok(message="ok", **extra):
    return SimpleNamespace(success=True, message=message, **extra)


def fail(message="no"):
    return SimpleNamespace(success=False, message=message)


def verify_gone(delta_mm=87.0, measured_z=1900.0):
    return ok(present=False, delta_mm=delta_mm, measured_z=measured_z)


def verify_present(delta_mm=3.0, measured_z=1800.0):
    return ok(present=True, delta_mm=delta_mm, measured_z=measured_z)


def verify_inconclusive(message="no depth frame"):
    return SimpleNamespace(success=False, message=message, present=False,
                           delta_mm=0.0, measured_z=-1.0)


class FakeContext:
    def __init__(self, params=None, ready=None, actions=None, services=None):
        self.params = dict(params or {})
        self.ready = dict(ready or {})
        self.actions = {k: list(v) for k, v in (actions or {}).items()}
        self.services = {k: list(v) for k, v in (services or {}).items()}
        self.calls = []
        self.statuses = []
        self.reports = []
        self.events = []
        self.capability = CAP_OPERATE
        self.blockers = {CAP_SEE: [], CAP_MOVE: []}
        self.stop_after_calls = None
        self._stop = False
        self.warnings = []
        self.log = SimpleNamespace(
            info=lambda m: None, warn=self.warnings.append, error=self.warnings.append
        )

    def _tick(self):
        if self.stop_after_calls is not None and len(self.calls) >= self.stop_after_calls:
            self._stop = True

    def _pop(self, table, key):
        queue = table.get(key)
        if not queue:
            raise AssertionError(f"no scripted result for '{key}'")
        item = queue.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def run_action(self, key, goal, timeout):
        self.calls.append((key, goal))
        self._tick()
        return self._pop(self.actions, key)

    def call(self, key, request, timeout):
        self.calls.append((key, request))
        self._tick()
        return self._pop(self.services, key)

    def client_ready(self, key):
        return self.ready.get(key, True)

    def stop_requested(self):
        return self._stop

    def publish_status(self, state, detail="", current=0, total=0):
        self.statuses.append((state, detail, current, total))

    def publish_layer_report(self, report):
        self.reports.append(report)

    def publish_pick_event(self, event):
        self.events.append(event)

    def get_param(self, name):
        return self.params[name]

    def sleep(self, seconds):
        pass

    def capability_state(self, required):
        missing = []
        if required & CAP_SEE and not (self.capability & CAP_SEE):
            missing += self.blockers[CAP_SEE]
        if required & CAP_MOVE and not (self.capability & CAP_MOVE):
            missing += self.blockers[CAP_MOVE]
        return self.capability, missing

    @property
    def final_state(self):
        return self.statuses[-1][0]

    def call_keys(self):
        return [c[0] for c in self.calls]


# ======================================================================
# pick_layer
# ======================================================================

PICK_PARAMS = {"scan_timeout_sec": 30.0, "scan_model_id": ""}


def pick_ctx(targets, picks, verifies=None, hide=None, **kw):
    return FakeContext(
        PICK_PARAMS,
        actions={"hide": [hide or ok()], "scan": [scan_result(targets)], "pick": picks},
        services={"verify": verifies or []},
        **kw,
    )


def test_pick_layer_happy_path():
    cans = [target(1, 2, 0), target(3, 4, 1)]
    ctx = pick_ctx(cans, [ok(), ok()], [verify_gone()] * 2)
    PickLayerWorkflow().run(ctx)

    assert ctx.call_keys() == ["hide", "scan", "pick", "verify", "pick", "verify"]
    assert ctx.calls[2][1].target.real_x == 1.0
    assert ctx.calls[4][1].target.real_x == 3.0
    assert len(ctx.reports) == 1
    report = ctx.reports[0]
    assert (report.successes, report.failures, report.overall_ok) == (2, 0, True)
    assert ctx.final_state == TaskStatus.STATE_IDLE
    assert [s[0] for s in ctx.statuses][:3] == [
        TaskStatus.STATE_SCANNING, TaskStatus.STATE_SCANNING, TaskStatus.STATE_PICKING,
    ]
    assert ctx.statuses[2][2:] == (1, 2) and ctx.statuses[3][2:] == (2, 2)
    assert [e.outcome for e in ctx.events] == [PickEvent.OUTCOME_PICKED] * 2
    assert all(e.verify_ran and e.verified for e in ctx.events)
    assert all(e.verify_delta_mm == 87.0 and e.verify_message == "" for e in ctx.events)
    assert ctx.events[0].detection_index == 0 and ctx.events[1].detection_index == 1


def test_pick_layer_hide_failed():
    ctx = pick_ctx([], [], hide=fail("no 'hide' teach point configured"))
    PickLayerWorkflow().run(ctx)
    assert ctx.call_keys() == ["hide"]
    assert ctx.final_state == TaskStatus.STATE_ERROR
    assert "no 'hide' teach point configured" in ctx.statuses[-1][1]
    assert ctx.reports == []


def test_pick_layer_stop_during_hide():
    ctx = pick_ctx([target(1, 2, 0)], [ok()])
    ctx.stop_after_calls = 1  # stop flag set as the hide move returns
    PickLayerWorkflow().run(ctx)
    assert ctx.call_keys() == ["hide"]
    assert ctx.final_state == TaskStatus.STATE_IDLE
    assert ctx.reports == []


def test_pick_layer_scan_rejected():
    ctx = FakeContext(PICK_PARAMS, actions={"hide": [ok()], "scan": [fail("not calibrated")]})
    PickLayerWorkflow().run(ctx)
    assert ctx.call_keys() == ["hide", "scan"]
    assert ctx.final_state == TaskStatus.STATE_ERROR
    assert "not calibrated" in ctx.statuses[-1][1]
    assert ctx.reports == []


def test_pick_layer_no_targets():
    ctx = pick_ctx([], [])
    PickLayerWorkflow().run(ctx)
    assert ctx.call_keys() == ["hide", "scan"]
    assert ctx.final_state == TaskStatus.STATE_IDLE
    assert "no cans" in ctx.statuses[-1][1]
    assert ctx.reports == []


def test_pick_layer_stop_during_scan():
    ctx = pick_ctx([target(1, 2, 0)], [ok()])
    ctx.stop_after_calls = 2  # stop flag set as the scan returns
    PickLayerWorkflow().run(ctx)
    assert ctx.call_keys() == ["hide", "scan"]
    assert ctx.final_state == TaskStatus.STATE_IDLE
    assert ctx.reports == []


def test_pick_layer_stop_mid_layer():
    cans = [target(1, 2, 0), target(3, 4, 1), target(5, 6, 2)]
    ctx = pick_ctx(cans, [ok(), ok(), ok()], [verify_gone()] * 3)
    ctx.stop_after_calls = 4  # after hide, scan, pick 1, verify 1
    PickLayerWorkflow().run(ctx)
    assert ctx.call_keys() == ["hide", "scan", "pick", "verify"]
    report = ctx.reports[0]
    assert (report.successes, report.failures, report.overall_ok) == (1, 0, False)
    assert ctx.final_state == TaskStatus.STATE_IDLE
    assert len(ctx.events) == 1


def test_pick_layer_failed_pick_outcomes():
    cans = [target(1, 2, 0), target(3, 4, 1)]
    ctx = pick_ctx(cans, [fail("gripper"), fail("canceled")])
    ctx.stop_after_calls = 4  # stop lands after the second pick call
    PickLayerWorkflow().run(ctx)
    assert [e.outcome for e in ctx.events] == [PickEvent.OUTCOME_FAILED, PickEvent.OUTCOME_SKIPPED]
    report = ctx.reports[0]
    assert (report.successes, report.failures, report.overall_ok) == (0, 2, False)
    assert ctx.call_keys() == ["hide", "scan", "pick", "pick"]  # no verify on failure


def test_pick_layer_verify_unavailable():
    ctx = pick_ctx([target(1, 2, 0)], [ok()], ready={"verify": False})
    PickLayerWorkflow().run(ctx)
    assert ctx.call_keys() == ["hide", "scan", "pick"]
    event = ctx.events[0]
    assert event.outcome == PickEvent.OUTCOME_PICKED
    assert not event.verify_ran and not event.verified
    assert event.verify_message == "verify_target unavailable"
    assert "verify_target unavailable" in event.message


def test_pick_layer_verify_still_present():
    ctx = pick_ctx([target(1, 2, 0)], [ok()], [verify_present(delta_mm=3.0, measured_z=1800.0)])
    PickLayerWorkflow().run(ctx)
    event = ctx.events[0]
    assert event.verify_ran and not event.verified
    assert event.measured_z == 1800.0
    assert event.verify_delta_mm == 3.0
    assert "WARNING" in event.message
    assert ctx.reports[0].successes == 1


def test_pick_layer_verify_inconclusive_is_not_verified():
    # A service answer with success=False (no depth / stale / no patch) must
    # never be read as "can gone".
    ctx = pick_ctx([target(1, 2, 0)], [ok()], [verify_inconclusive("depth stale (4.2 s)")])
    PickLayerWorkflow().run(ctx)
    event = ctx.events[0]
    assert event.outcome == PickEvent.OUTCOME_PICKED
    assert not event.verify_ran and not event.verified
    assert event.measured_z == -1.0 and event.verify_delta_mm == 0.0
    assert event.verify_message == "depth stale (4.2 s)"
    assert "verify inconclusive: depth stale" in event.message
    assert ctx.reports[0].successes == 1
    assert any("could not measure" in w for w in ctx.warnings)


def test_pick_layer_verify_timeout_does_not_break_loop():
    cans = [target(1, 2, 0), target(3, 4, 1)]
    ctx = pick_ctx(cans, [ok(), ok()], [TimeoutError("slow"), verify_gone()])
    PickLayerWorkflow().run(ctx)
    assert ctx.call_keys() == ["hide", "scan", "pick", "verify", "pick", "verify"]
    assert "verify timed out" in ctx.events[0].message
    assert ctx.events[0].verify_message.startswith("timed out")
    assert ctx.events[1].verify_ran
    assert ctx.reports[0].successes == 2


def test_pick_layer_action_exception_propagates():
    ctx = FakeContext(PICK_PARAMS, actions={"hide": [ok()], "scan": [TimeoutError("never acked")]})
    with pytest.raises(TimeoutError):
        PickLayerWorkflow().run(ctx)
    assert ctx.reports == []


# ======================================================================
# calibration
# ======================================================================

CAL_PARAMS = {
    "calibration_x_range_mm": [-350.0, 50.0],
    "calibration_y_range_mm": [-1300.0, -900.0],
    "calibration_z_levels_mm": [500.0, 600.0],
    "calibration_grid_size": [3, 2],
    "calibration_orientation_deg": [180.0, 0.0, 0.0],
    "calibration_settle_sec": 0.0,
    "calibration_sample_timeout_sec": 5.0,
    "startup_calibration_grid_size": [2, 2],
    "startup_calibration_z_levels_mm": [500.0],
    "auto_calibration_model_id": "green_light_v2",
    "auto_calibration_class_id": -1,
    "auto_calibration_samples_per_pose": 5,
    "auto_calibration_grid_size": [4, 3],
    "auto_calibration_z_levels_mm": [500.0, 600.0],
    "auto_startup_calibration_grid_size": [3, 3],
    "auto_startup_calibration_z_levels_mm": [500.0],
}


def cal_ctx(n_poses, move=None, samples=None, begin=None, compute=None, **kw):
    return FakeContext(
        CAL_PARAMS,
        actions={"move": move if move is not None else [ok()] * n_poses},
        services={
            "cal_begin": [begin or ok()],
            "cal_sample": samples if samples is not None
            else [ok(sample_count=i + 1) for i in range(n_poses)],
            "cal_compute": [compute or ok("rms 1.2 mm")],
        },
        **kw,
    )


def test_spread():
    assert spread(0.0, 10.0, 1) == [5.0]
    assert spread(0.0, 10.0, 3) == [0.0, 5.0, 10.0]


def test_calibration_poses_grid():
    ctx = FakeContext(CAL_PARAMS)
    poses = CalibrationWorkflow().poses(ctx)
    assert len(poses) == 3 * 2 * 2
    assert poses[0] == (-350.0, -1300.0, 500.0)
    assert poses[-1] == (50.0, -900.0, 600.0)


def test_startup_variant_uses_its_own_grid_params():
    ctx = FakeContext(CAL_PARAMS)
    wf = StartupCalibrationWorkflow()
    assert wf.id == "startup_calibration"
    assert len(wf.poses(ctx)) == 2 * 2 * 1
    assert {p.name for p in wf.params} >= {
        "startup_calibration_grid_size", "startup_calibration_z_levels_mm", "calibration_x_range_mm",
    }


@pytest.mark.parametrize(
    "override",
    [
        {"calibration_x_range_mm": [0.0, 0.0]},
        {"calibration_y_range_mm": [0.0, 0.0]},
        {"calibration_z_levels_mm": [0.0]},
    ],
)
def test_calibration_check_ready_grid_unconfigured(override):
    ctx = FakeContext({**CAL_PARAMS, **override})
    ctx.capability = 0
    ctx.blockers[CAP_MOVE] = ["should not be consulted"]
    ready, why = CalibrationWorkflow().check_ready(ctx)
    assert not ready
    assert "calibration grid not configured" in why


def test_calibration_check_ready_missing_client():
    ctx = FakeContext(CAL_PARAMS, ready={"move": False})
    ready, why = CalibrationWorkflow().check_ready(ctx)
    assert (ready, why) == (False, "/cobot/move_to_pose not available")


def test_calibration_check_ready_capability_missing():
    ctx = FakeContext(CAL_PARAMS)
    ctx.capability = CAP_SEE
    ctx.blockers[CAP_MOVE] = ["servos off"]
    assert CalibrationWorkflow().check_ready(ctx) == (
        False, "requires operate: servos off"
    )


def test_calibration_check_ready_ok():
    assert CalibrationWorkflow().check_ready(FakeContext(CAL_PARAMS)) == (True, "")


def test_calibration_happy_path():
    ctx = cal_ctx(12)
    CalibrationWorkflow().run(ctx)
    keys = ctx.call_keys()
    assert keys[0] == "cal_begin" and keys[-1] == "cal_compute"
    assert keys.count("move") == 12 and keys.count("cal_sample") == 12
    assert keys[1:-1] == ["move", "cal_sample"] * 12
    goal = ctx.calls[1][1]
    assert (goal.x, goal.y, goal.z, goal.rx) == (-350.0, -1300.0, 500.0, 180.0)
    assert ctx.final_state == TaskStatus.STATE_IDLE
    assert "rms 1.2 mm" in ctx.statuses[-1][1]
    assert ctx.statuses[0][2:] == (0, 12)
    assert ctx.statuses[1][2:] == (1, 12)


def test_calibration_move_failure_skips_sample():
    moves = [ok()] * 12
    moves[2] = fail("unreachable")
    ctx = cal_ctx(12, move=moves, samples=[ok(sample_count=1)] * 11)
    CalibrationWorkflow().run(ctx)
    keys = ctx.call_keys()
    assert keys.count("move") == 12 and keys.count("cal_sample") == 11
    assert keys[-1] == "cal_compute"
    assert any("skipped" in w for w in ctx.warnings)
    assert ctx.final_state == TaskStatus.STATE_IDLE


def test_calibration_begin_refused():
    ctx = cal_ctx(12, begin=fail("no intrinsics"))
    CalibrationWorkflow().run(ctx)
    assert ctx.call_keys() == ["cal_begin"]
    assert ctx.final_state == TaskStatus.STATE_ERROR


def test_calibration_compute_refused():
    ctx = cal_ctx(12, compute=fail("rms too high"))
    CalibrationWorkflow().run(ctx)
    assert ctx.final_state == TaskStatus.STATE_ERROR
    assert "rms too high" in ctx.statuses[-1][1]


def test_calibration_stop_after_first_move():
    ctx = cal_ctx(12)
    ctx.stop_after_calls = 2  # begin, move #1
    CalibrationWorkflow().run(ctx)
    assert ctx.call_keys() == ["cal_begin", "move", "cal_sample"]
    assert ctx.final_state == TaskStatus.STATE_IDLE
    assert "aborted by stop at pose 2/12" in ctx.statuses[-1][1]


# ======================================================================
# capability prerequisites
# ======================================================================

def test_workflows_require_operate():
    from cobot_tasks.workflows.pick_layer import PickLayerWorkflow as P
    assert P().requires == CAP_OPERATE
    assert CalibrationWorkflow().requires == CAP_OPERATE
    assert StartupCalibrationWorkflow().requires == CAP_OPERATE


def test_check_ready_names_only_missing_side():
    ctx = FakeContext(PICK_PARAMS)
    ctx.capability = CAP_SEE  # vision fine, motion missing
    ctx.blockers[CAP_MOVE] = ["servos off", "pendant in TEACH mode"]
    ctx.blockers[CAP_SEE] = ["should not appear"]
    ready, why = PickLayerWorkflow().check_ready(ctx)
    assert not ready
    assert why == "requires operate: servos off; pendant in TEACH mode"


def test_move_only_workflow_passes_without_vision():
    class MoveOnly(Workflow):
        id = "move_only"
        label = "Move only"
        description = "no vision needed"
        requires = CAP_MOVE

    ctx = FakeContext({})
    ctx.capability = CAP_MOVE  # camera fully down
    ctx.blockers[CAP_SEE] = ["camera stream down"]
    assert MoveOnly().check_ready(ctx) == (True, "")


# ======================================================================
# registry
# ======================================================================

def test_registry_consistent():
    validate_registry()
    assert list(WORKFLOWS) == [w.id for w in REGISTRY]
    assert "pick_layer" in WORKFLOWS and "calibration" in WORKFLOWS
    for wf in REGISTRY:
        assert wf.label and wf.description


# ======================================================================
# vision_test
# ======================================================================

import random  # noqa: E402

from cobot_tasks.system_state import CAP_NONE  # noqa: E402
from cobot_tasks.workflows.vision_test import VisionTestWorkflow, find_images  # noqa: E402

VT_MODELS = ["cans", "green_light"]


def vt_params(root, seed=7, models=None):
    return {
        "vision_test_image_dir": str(root),
        "vision_test_model_ids": list(models or VT_MODELS),
        "vision_test_seed": seed,
        "vision_test_scan_timeout_sec": 90.0,
    }


def make_tree(root):
    """Both real dataset layouts (ultralytics + roboflow) plus decoys that must
    be skipped. Returns the images in sweep order."""
    images = [
        "cans_obb/images/train/IMG_001.jpeg",
        "cans_obb/images/train/IMG_002.jpg",
        "cans_obb/images/val/IMG_003.JPG",
        "green_light/train/images/a.png",
        "green_light/valid/images/b.png",
    ]
    decoys = [
        "cans_obb/labels/train/IMG_001.txt",
        "cans_obb/labels/train.cache",
        "cans_obb/can_obb.yaml",
        "green_light/train/labels/a.txt",
        "green_light/README.roboflow.txt",
    ]
    for rel in images + decoys:
        path = root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"x")
    return find_images(root)


def detect_only(count):
    return SimpleNamespace(success=True, message=f"{count} detection(s)",
                           detection_count=count, targets=SimpleNamespace(targets=[]))


def test_vision_test_find_images_skips_labels_and_sorts(tmp_path):
    images = make_tree(tmp_path)
    assert [p.name for p in images] == ["IMG_001.jpeg", "IMG_002.jpg", "IMG_003.JPG", "a.png", "b.png"]
    assert all("labels" not in p.parts for p in images)
    assert find_images(tmp_path / "missing") == []


def test_vision_test_seeded_sweep(tmp_path):
    images = make_tree(tmp_path)
    ctx = FakeContext(vt_params(tmp_path, seed=7),
                      actions={"scan": [detect_only(k) for k in (3, 0, 2, 1, 5)]})
    VisionTestWorkflow().run(ctx)

    assert ctx.call_keys() == ["scan"] * 5
    replay = random.Random(7)
    for (_, goal), image in zip(ctx.calls, images):
        assert goal.model_id == replay.choice(VT_MODELS)
        assert goal.image_source == str(image)
        assert goal.localize is False
    assert ctx.final_state == TaskStatus.STATE_IDLE
    summary = ctx.statuses[-1][1]
    assert "5 images" in summary and "11 detections" in summary and "0 failed" in summary
    progress = [s for s in ctx.statuses if s[0] == TaskStatus.STATE_SCANNING]
    assert progress[0][2:] == (0, 5) and progress[-1][2:] == (5, 5)
    assert ctx.warnings == []


def test_vision_test_unseeded_still_draws_from_the_list(tmp_path):
    make_tree(tmp_path)
    ctx = FakeContext(vt_params(tmp_path, seed=0), actions={"scan": [detect_only(0)] * 5})
    VisionTestWorkflow().run(ctx)
    assert all(goal.model_id in VT_MODELS for _, goal in ctx.calls)
    assert ctx.final_state == TaskStatus.STATE_IDLE


def test_vision_test_failures_do_not_abort_sweep(tmp_path):
    make_tree(tmp_path)
    ctx = FakeContext(vt_params(tmp_path), actions={"scan": [
        detect_only(1), fail("goal rejected"), TimeoutError("slow"), detect_only(2), detect_only(0),
    ]})
    VisionTestWorkflow().run(ctx)
    assert ctx.call_keys() == ["scan"] * 5
    assert ctx.final_state == TaskStatus.STATE_IDLE
    summary = ctx.statuses[-1][1]
    assert "2 failed" in summary and "3 detections" in summary
    assert len(ctx.warnings) == 2 and "slow" in ctx.warnings[1]
    failed_status = [s for s in ctx.statuses if "FAILED" in s[1]]
    assert len(failed_status) == 2 and failed_status[0][2:] == (2, 5)


def test_vision_test_every_image_failing_is_an_error(tmp_path):
    make_tree(tmp_path)
    ctx = FakeContext(vt_params(tmp_path), actions={"scan": [fail("no detector")] * 5})
    VisionTestWorkflow().run(ctx)
    assert ctx.call_keys() == ["scan"] * 5
    assert ctx.final_state == TaskStatus.STATE_ERROR
    assert "5 failed" in ctx.statuses[-1][1]


def test_vision_test_stop_mid_sweep(tmp_path):
    make_tree(tmp_path)
    ctx = FakeContext(vt_params(tmp_path), actions={"scan": [detect_only(1)] * 5})
    ctx.stop_after_calls = 2
    VisionTestWorkflow().run(ctx)
    assert ctx.call_keys() == ["scan", "scan"]
    assert ctx.final_state == TaskStatus.STATE_IDLE
    assert "stopped" in ctx.statuses[-1][1]


def test_vision_test_no_images_or_models(tmp_path):
    ctx = FakeContext(vt_params(tmp_path))
    VisionTestWorkflow().run(ctx)
    assert ctx.calls == [] and ctx.final_state == TaskStatus.STATE_ERROR
    assert "no images" in ctx.statuses[-1][1]

    make_tree(tmp_path)
    ctx = FakeContext(vt_params(tmp_path, models=[""]))
    VisionTestWorkflow().run(ctx)
    assert ctx.calls == [] and ctx.final_state == TaskStatus.STATE_ERROR


def test_vision_test_registered_and_ungated():
    wf = WORKFLOWS["vision_test"]
    assert isinstance(wf, VisionTestWorkflow) and wf.requires == CAP_NONE
    assert REGISTRY[-1] is wf  # dashboard order: after the real tasks
    validate_registry()

    ctx = FakeContext(vt_params("/nowhere"))
    ctx.capability = CAP_NONE
    ctx.blockers = {CAP_SEE: ["camera stream down"], CAP_MOVE: ["servos off"]}
    assert wf.check_ready(ctx) == (True, "")   # blockers are irrelevant to it
    ctx.ready = {"scan": False}
    ready, why = wf.check_ready(ctx)
    assert not ready and "/vision/scan" in why


# ======================================================================
# automatic calibration twins
# ======================================================================

def test_manual_calibration_begin_pins_manual_mode():
    from cobot_interfaces.srv import CalibrationBegin
    ctx = cal_ctx(12)
    CalibrationWorkflow().run(ctx)
    key, req = ctx.calls[0]
    assert key == "cal_begin"
    assert isinstance(req, CalibrationBegin.Request)
    assert req.mode == CalibrationBegin.Request.MODE_MANUAL
    assert req.model_id == "" and req.class_id == -1 and req.samples_per_pose == 0


def test_auto_calibration_is_denser_and_automatic():
    from cobot_interfaces.srv import CalibrationBegin
    wf = AutoCalibrationWorkflow()
    manual = CalibrationWorkflow()
    ctx = cal_ctx(24)
    assert len(wf.poses(ctx)) == 24 > len(manual.poses(ctx)) == 12
    wf.run(ctx)
    key, req = ctx.calls[0]
    assert key == "cal_begin"
    assert req.mode == CalibrationBegin.Request.MODE_AUTOMATIC
    assert req.model_id == "green_light_v2"
    assert req.class_id == -1
    assert req.samples_per_pose == 5
    keys = ctx.call_keys() if hasattr(ctx, "call_keys") else [k for k, _ in ctx.calls]
    assert keys.count("move") == 24 and keys.count("cal_sample") == 24
    assert ctx.statuses[-1][0] == TaskStatus.STATE_IDLE


def test_auto_startup_twin_uses_its_own_grid():
    wf = AutoStartupCalibrationWorkflow()
    ctx = cal_ctx(9)
    assert wf.id == "auto_startup_calibration" and wf.automatic
    assert len(wf.poses(ctx)) == 9
    names = {p.name for p in wf.params}
    assert {"auto_startup_calibration_grid_size", "auto_startup_calibration_z_levels_mm",
            "auto_calibration_model_id"} <= names


def test_auto_twins_registered_next_to_their_manual_twins():
    ids = [w.id for w in REGISTRY]
    assert ids.index("auto_calibration") == ids.index("calibration") + 1
    assert ids.index("auto_startup_calibration") == ids.index("startup_calibration") + 1
    for wf_id in ("auto_calibration", "auto_startup_calibration"):
        assert WORKFLOWS[wf_id].requires == CAP_OPERATE
