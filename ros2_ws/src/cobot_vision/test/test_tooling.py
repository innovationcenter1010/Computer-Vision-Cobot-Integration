"""Per-tool marker offsets (cobot_setup.yaml `tools`) and the calibration
file's tool stamp — pure, no ROS."""

import numpy as np
import pytest

from cobot_vision.calibration import CameraRobotCalibration
from cobot_vision.tooling import load_marker_offsets, select_marker_offset

SETUP = """
tools:
  none:
    tool_number: 0
    tip_offset_mm: [0.0, 0.0, -600.0]
    marker_offset_mm: [0.0, 0.0, 300.0]
  vacuum_head:
    tool_number: 1
    marker_offset_mm: [1.5, -2.0, -300.0]
  bare:
    tool_number: 2
"""


def test_load_and_select(tmp_path):
    p = tmp_path / "cobot_setup.yaml"
    p.write_text(SETUP, encoding="utf-8")
    table = load_marker_offsets(str(p))
    assert sorted(table) == [0, 1, 2]
    assert select_marker_offset(table, 0).offset_mm == (0.0, 0.0, 300.0)
    vh = select_marker_offset(table, 1)
    assert vh.name == "vacuum_head" and vh.offset_mm == (1.5, -2.0, -300.0) and not vh.is_zero
    assert select_marker_offset(table, 2).is_zero  # missing offset -> zeros, flagged by the node
    assert select_marker_offset(table, 9) is None


def test_duplicate_numbers_and_bad_offsets(tmp_path):
    p = tmp_path / "cobot_setup.yaml"
    p.write_text("tools:\n  a: {tool_number: 1}\n  b: {tool_number: 1}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="also used"):
        load_marker_offsets(str(p))
    p.write_text("tools:\n  a: {tool_number: 1, marker_offset_mm: [1, 2]}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="must be \\[x, y, z\\]"):
        load_marker_offsets(str(p))
    with pytest.raises(FileNotFoundError):
        load_marker_offsets(str(tmp_path / "nope.yaml"))


def test_calibration_file_round_trips_tool_stamp(tmp_path):
    cal = CameraRobotCalibration(
        np.eye(3, 4), (0.1, 0.2, 0.3), 6, created="2026-08-30T00:00:00+00:00",
        tool_number=1, tool_name="vacuum_head", marker_offset_mm=(0.0, 0.0, -300.0),
    )
    p = tmp_path / "calibration.yaml"
    cal.save(p)
    back = CameraRobotCalibration.load(p)
    assert back.tool_number == 1 and back.tool_name == "vacuum_head"
    assert back.marker_offset_mm == (0.0, 0.0, -300.0)
    assert np.allclose(back.matrix, cal.matrix)


def test_calibration_file_without_tool_stamp_still_loads(tmp_path):
    cal = CameraRobotCalibration(np.eye(3, 4), (0.1, 0.2, 0.3), 6)
    p = tmp_path / "calibration.yaml"
    cal.save(p)
    text = p.read_text(encoding="utf-8")
    assert "tool_number" not in text and "marker_offset_mm" not in text
    back = CameraRobotCalibration.load(p)
    assert back.tool_number is None and back.tool_name == "" and back.marker_offset_mm is None
