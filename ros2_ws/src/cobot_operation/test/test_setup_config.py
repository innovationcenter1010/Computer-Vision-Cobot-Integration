"""cobot_setup.yaml `tools` table rules."""

import pytest

from cobot_operation.setup_config import load_tools

GOOD = """
tools:
  none:
    tool_number: 0
    tip_offset_mm: [0.0, 0.0, -600.0]
    marker_offset_mm: [0.0, 0.0, 300.0]
    pick_tip_z_mm: 300.0
    approach_tip_z_mm: 401.0
    descend_profile: descend
  vacuum_head:
    tool_number: 1
    marker_offset_mm: [0.0, 0.0, -300.0]
    pick_tip_z_mm: 310.0
"""


def write(tmp_path, text):
    p = tmp_path / "cobot_setup.yaml"
    p.write_text(text, encoding="utf-8")
    return str(p)


def test_happy_path(tmp_path):
    table = load_tools(write(tmp_path, GOOD))
    assert table.names() == ["none", "vacuum_head"]
    assert table.numbers() == {0, 1}
    none = table.none()
    assert none.tool_number == 0 and none.tip_offset.z == -600.0 and none.uses_python_tip_offset
    vh = table.by_number(1)
    assert vh.name == "vacuum_head" and vh.tip_offset.is_empty() and not vh.uses_python_tip_offset
    assert vh.marker_offset_mm == (0.0, 0.0, -300.0) and vh.pick_tip_z_mm == 310.0
    assert table.by_number(7) is None
    assert table.notes == []


def test_tip_offset_forbidden_on_real_tool_file(tmp_path):
    text = GOOD.replace("    marker_offset_mm: [0.0, 0.0, -300.0]\n",
                        "    marker_offset_mm: [0.0, 0.0, -300.0]\n    tip_offset_mm: [0.0, 0.0, -600.0]\n")
    with pytest.raises(ValueError, match="must be omitted/zero for tool file 1"):
        load_tools(write(tmp_path, text))


def test_none_is_mandatory_and_tool_zero(tmp_path):
    with pytest.raises(ValueError, match="must contain `none`"):
        load_tools(write(tmp_path, "tools:\n  g: {tool_number: 2}\n"))
    with pytest.raises(ValueError, match="must be 0"):
        load_tools(write(tmp_path, "tools:\n  none: {tool_number: 2}\n"))
    with pytest.raises(ValueError, match="reserved for `none`"):
        load_tools(write(tmp_path, "tools:\n  none: {tool_number: 0}\n  g: {tool_number: 0}\n"))


def test_duplicate_and_out_of_range_numbers(tmp_path):
    with pytest.raises(ValueError, match="already used"):
        load_tools(write(tmp_path, "tools:\n  none: {tool_number: 0}\n  a: {tool_number: 3}\n  b: {tool_number: 3}\n"))
    with pytest.raises(ValueError, match="0-63"):
        load_tools(write(tmp_path, "tools:\n  none: {tool_number: 0}\n  a: {tool_number: 64}\n"))


def test_notes_for_zero_marker_and_stale_active_tool(tmp_path):
    table = load_tools(write(tmp_path, "tools:\n  none: {tool_number: 0}\nactive_tool: none\n"))
    assert any("marker_offset_mm is all zero" in n for n in table.notes)
    assert any("active_tool" in n for n in table.notes)


def test_missing_tools_section(tmp_path):
    with pytest.raises(ValueError, match="non-empty mapping"):
        load_tools(write(tmp_path, "io: {}\n"))
