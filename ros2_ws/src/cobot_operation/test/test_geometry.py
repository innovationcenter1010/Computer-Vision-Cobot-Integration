import math

from cobot_operation.geometry import Pose6, ToolOffset


def test_zero_offset_is_identity():
    off = ToolOffset()
    assert off.is_empty()
    assert off.command_for(100.0, -200.0, 300.0, 45.0) == (100.0, -200.0, 300.0)


def test_z_offset_subtracts_regardless_of_rz():
    off = ToolOffset(0.0, 0.0, -600.0)
    for rz in (0.0, 90.0, 180.0, -37.5):
        x, y, z = off.command_for(10.0, 20.0, 300.0, rz)
        assert (x, y) == (10.0, 20.0)
        assert z == 900.0


def test_xy_offset_rotates_with_rz():
    off = ToolOffset(10.0, 0.0, 0.0)
    x, y, _ = off.command_for(0.0, 0.0, 0.0, 0.0)
    assert math.isclose(x, -10.0) and math.isclose(y, 0.0, abs_tol=1e-9)
    x, y, _ = off.command_for(0.0, 0.0, 0.0, 90.0)
    assert math.isclose(x, 0.0, abs_tol=1e-9) and math.isclose(y, -10.0)
    x, y, _ = off.command_for(0.0, 0.0, 0.0, 180.0)
    assert math.isclose(x, 10.0) and math.isclose(y, 0.0, abs_tol=1e-9)


def test_pose_tool_number_defaults_to_active():
    p = Pose6(1, 2, 3, 180, 0, 0)
    assert p.tool_number == -1 and "T" not in str(p)
    q = p.with_tool(2)
    assert q.tool_number == 2 and str(q).endswith(" T2") and q.xyzrpy() == p.xyzrpy()
    assert Pose6.from_list([1, 2, 3, 4, 5, 6], tool_number=0).tool_number == 0


def test_pose_from_list_validates_length():
    p = Pose6.from_list([1, 2, 3, 180, 0, -90])
    assert p == Pose6(1.0, 2.0, 3.0, 180.0, 0.0, -90.0)
    try:
        Pose6.from_list([1, 2, 3])
    except ValueError as e:
        assert "expected 6" in str(e)
    else:
        raise AssertionError("short list accepted")
