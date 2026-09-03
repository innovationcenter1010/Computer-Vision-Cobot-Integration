"""TrajectoryStore: v1 migration, v2 round trip, seeding, all-or-nothing
conversion, atomic save — no ROS, no robot."""

import json

import pytest

from cobot_operation.geometry import Pose6
from cobot_operation.trajectory_store import (
    ConvertResult,
    TeachPoint,
    TrajectoryStore,
)

V1 = {
    "check": [0, -925, 750, 180, 0, -90],
    "right": [[500, -900, 700, 180, 0, 0], [500, -900, 400, 180, 0, 0]],
    "right_flip": [[510, -900, 700, 180, 0, 0], [510, -900, 400, 180, 0, 0]],
    "left": [[-500, -900, 700, 180, 0, 0], [-500, -900, 400, 180, 0, 0]],
    "left_flip": [[-510, -900, 700, 180, 0, 0], [-510, -900, 400, 180, 0, 0]],
    "await": [[930, -140, 200, 180, 0, 0], [1, 2, 3, 4, 5, 6]],
    "note": "kept as-is",
}


def shift_converter(dz=-600.0, fail_key=None):
    """Fake /cobot/convert_pose: same pose shifted in z, joints invented."""

    def convert(pose, to_tool, joints):
        return ConvertResult(
            True, pose=Pose6(pose.x, pose.y, pose.z + dz, pose.rx, pose.ry, pose.rz, tool_number=to_tool),
            joints_deg=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), message="ok",
        )

    return convert


def run_conversion(store, to_tool, convert):
    results = {}
    for key, index, tp in store.pending(to_tool):
        results[(key, index)] = convert(tp.pose, to_tool, tp.joints_deg)
    return store.apply_conversion(to_tool, results)


# ----------------------------------------------------------------------


def test_v1_migration_shape():
    store = TrajectoryStore.from_v1(V1)
    assert store.migrated
    assert set(store.points) == {"check", "right", "right_flip", "left", "left_flip", "await"}
    assert len(store.points["check"]) == 1  # flat 6-list -> 1-element list
    assert store.points["check"][0].pose == Pose6(0, -925, 750, 180, 0, -90, tool_number=0)
    assert store.tool_numbers() == {0}
    assert all(tp.joints_deg is None for _, _, tp in store.all_points())
    assert store.extra == {"note": "kept as-is"}
    store.validate()  # required keys/counts are satisfied


def test_load_detects_v1_and_round_trips_v2(tmp_path):
    p = tmp_path / "trajectories.json"
    p.write_text(json.dumps(V1), encoding="utf-8")
    store = TrajectoryStore.load(p)
    assert store.migrated
    store.save()
    assert not (tmp_path / "trajectories.json.tmp").exists()
    raw = json.loads(p.read_text(encoding="utf-8"))
    assert raw["version"] == 2 and raw["updated_at"]
    assert raw["points"]["check"][0]["frame"] == "base"
    assert raw["points"]["check"][0]["tool_number"] == 0
    assert raw["note"] == "kept as-is"

    again = TrajectoryStore.load(p)
    assert not again.migrated
    assert again.poses() == store.poses()
    assert again.updated_at == store.updated_at


def test_seed_copies_default_only_when_missing(tmp_path):
    default = tmp_path / "default.json"
    default.write_text(json.dumps(V1), encoding="utf-8")
    runtime = tmp_path / "rt" / "trajectories.json"
    logged = []
    store = TrajectoryStore.seed(runtime, default, log=logged.append)
    assert runtime.exists() and logged and "seeded" in logged[0]
    assert store.migrated  # the default was v1 in this test
    store.save()
    # second boot: existing file wins, default not consulted
    default.write_text("garbage", encoding="utf-8")
    again = TrajectoryStore.seed(runtime, default)
    assert not again.migrated and again.count() == store.count()


def test_seed_without_default_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        TrajectoryStore.seed(tmp_path / "missing.json", tmp_path / "nope.json")


def test_conversion_all_or_nothing_success(tmp_path):
    p = tmp_path / "trajectories.json"
    p.write_text(json.dumps(V1), encoding="utf-8")
    store = TrajectoryStore.load(p)
    store.save()
    before = p.read_text(encoding="utf-8")

    report = run_conversion(store, 1, shift_converter(-600.0))
    assert report.ok and report.converted == store.count() and report.skipped == 0
    assert store.tool_numbers() == {1}
    assert store.is_in_tool(1) and not store.is_in_tool(0)
    check = store.points["check"][0]
    assert check.pose.z == 150.0 and check.pose.tool_number == 1
    assert check.joints_deg == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert check.converted_from["tool_number"] == 0 and check.converted_from["had_joints"] is False
    assert p.read_text(encoding="utf-8") != before  # persisted
    assert json.loads(p.read_text(encoding="utf-8"))["points"]["check"][0]["joints_deg"] == [1, 2, 3, 4, 5, 6]

    # idempotent: nothing pending in tool 1
    again = run_conversion(store, 1, shift_converter(-600.0))
    assert again.ok and again.converted == 0 and again.skipped == store.count()

    # back to 0 uses the stored joints
    seen = []

    def convert(pose, to_tool, joints):
        seen.append(joints)
        return shift_converter(600.0)(pose, to_tool, joints)

    back = run_conversion(store, 0, convert)
    assert back.ok and all(j == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0) for j in seen)
    assert store.points["check"][0].pose.z == 750.0


def test_conversion_failure_leaves_memory_and_disk_untouched(tmp_path):
    p = tmp_path / "trajectories.json"
    p.write_text(json.dumps(V1), encoding="utf-8")
    store = TrajectoryStore.load(p)
    store.save()
    before_disk = p.read_text(encoding="utf-8")
    before_mem = store.poses()

    results = {}
    for i, (key, index, tp) in enumerate(store.pending(2)):
        if key == "left":
            results[(key, index)] = ConvertResult(False, message="IK failed")
        else:
            results[(key, index)] = shift_converter()(tp.pose, 2, tp.joints_deg)
    report = store.apply_conversion(2, results)
    assert not report.ok and report.converted == 0
    assert any("left[" in f and "IK failed" in f for f in report.failures)
    assert store.poses() == before_mem
    assert p.read_text(encoding="utf-8") == before_disk
    assert store.tool_numbers() == {0}

    # a missing result is a failure too
    partial = {k: v for k, v in results.items() if k[0] != "check"}
    assert not store.apply_conversion(2, partial).ok


def test_set_point_and_summary(tmp_path):
    p = tmp_path / "trajectories.json"
    p.write_text(json.dumps(V1), encoding="utf-8")
    store = TrajectoryStore.load(p)
    store.save()
    tp = store.set_point("check", 0, Pose6(1, 2, 3, 180, 0, 0, tool_number=0), (0, 10, 20, 0, -90, 0))
    assert tp.taught_at and tp.joints_deg == (0.0, 10.0, 20.0, 0.0, -90.0, 0.0)
    assert store.summary()["with_joints"] == 1
    assert store.summary()["count"] == 11
    assert json.loads(p.read_text(encoding="utf-8"))["points"]["check"][0]["taught_at"] == tp.taught_at
    with pytest.raises(KeyError):
        store.set_point("nope", 0, Pose6(1, 2, 3, 180, 0, 0, tool_number=0))
    with pytest.raises(IndexError):
        store.set_point("check", 3, Pose6(1, 2, 3, 180, 0, 0, tool_number=0))
    with pytest.raises(ValueError):
        store.set_point("check", 0, Pose6(1, 2, 3, 180, 0, 0))  # no tool number


@pytest.mark.parametrize(
    "obj, match",
    [
        ({"x": 1, "y": 2, "z": 3, "rx": 0, "ry": 0}, "missing"),
        ({"x": 1, "y": 2, "z": 3, "rx": 0, "ry": 0, "rz": 0, "frame": "user1"}, "frame"),
        ({"x": 1, "y": 2, "z": 3, "rx": 0, "ry": 0, "rz": 0, "tool_number": -1}, "tool_number"),
        ({"x": 1, "y": 2, "z": 3, "rx": 0, "ry": 0, "rz": 0, "joints_deg": [1, 2]}, "joints_deg"),
        ([1, 2, 3, 4, 5, 6], "pose object"),
    ],
)
def test_point_validation(obj, match):
    with pytest.raises(ValueError, match=match):
        TeachPoint.from_json(obj, "t")


def test_validate_reports_missing_and_counts():
    store = TrajectoryStore.from_v1({k: v for k, v in V1.items() if k != "left_flip"})
    with pytest.raises(ValueError, match="missing 'left_flip'"):
        store.validate()
    bad = dict(V1)
    bad["right"] = [[1, 2, 3, 4, 5, 6]]
    with pytest.raises(ValueError, match="'right' must hold exactly 2"):
        TrajectoryStore.from_v1(bad).validate()


def test_unsupported_version_rejected(tmp_path):
    p = tmp_path / "t.json"
    p.write_text(json.dumps({"version": 9, "points": {}}), encoding="utf-8")
    with pytest.raises(ValueError, match="version"):
        TrajectoryStore.load(p)
