"""cobot_setup.yaml — the `tools` table (per-tool cell values) and `io.sensors`.

cobot_control reads the same file for io + motion_profiles + the tool
names (C++, yaml-cpp). The ACTIVE tool is not in this file any more: it is
runtime state owned by cobot_control (/cobot/set_active_tool, persisted in
runtime_config/tooling_state.yaml) and read from /cobot/state.

Rules enforced here (a violation is a boot failure with the key named):
  * `tools` is a non-empty mapping and contains `none` with tool_number 0
  * every other entry has tool_number 1-63; numbers are unique
  * tip_offset_mm may be non-zero ONLY on tool 0 — with a real tool file the
    controller's TCP already is the tip, so applying an offset again would
    double it (that is the duplicated-TCP-math hazard this design removes)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from cobot_operation.geometry import ToolOffset, ToolProfile

NONE_TOOL = "none"


def _triple(values, where: str) -> tuple:
    if values is None:
        return (0.0, 0.0, 0.0)
    if len(values) != 3:
        raise ValueError(f"{where}: expected [x, y, z], got {values!r}")
    return tuple(float(v) for v in values)


def load_setup(path: str) -> dict:
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"setup file not found: {path}")
    data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: top level must be a mapping")
    return data


@dataclass
class ToolTable:
    by_name: dict = field(default_factory=dict)  # name -> ToolProfile
    notes: list = field(default_factory=list)    # non-fatal remarks to log

    def by_number(self, tool_number: int) -> ToolProfile | None:
        for t in self.by_name.values():
            if t.tool_number == int(tool_number):
                return t
        return None

    def none(self) -> ToolProfile:
        return self.by_name[NONE_TOOL]

    def names(self) -> list:
        return list(self.by_name)

    def numbers(self) -> set:
        return {t.tool_number for t in self.by_name.values()}

    def __len__(self) -> int:
        return len(self.by_name)


def load_tools(path: str) -> ToolTable:
    data = load_setup(path)
    tools_raw = data.get("tools") or {}
    if not isinstance(tools_raw, dict) or not tools_raw:
        raise ValueError(f"{path}: `tools` must be a non-empty mapping (at least `none`)")

    table = ToolTable()
    for name, cfg in tools_raw.items():
        cfg = cfg or {}
        where = f"{path}: tools.{name}"
        if not isinstance(cfg, dict):
            raise ValueError(f"{where} must be a mapping")
        number = int(cfg.get("tool_number", 0))
        if number < 0 or number > 63:
            raise ValueError(f"{where}.tool_number must be 0-63, got {number}")
        if str(name) == NONE_TOOL and number != 0:
            raise ValueError(f"{where}.tool_number must be 0 ('none' is the tool-0 default)")
        if str(name) != NONE_TOOL and number == 0:
            raise ValueError(
                f"{where}.tool_number 0 is reserved for `none`; give the tool file its real number"
            )
        tip = _triple(cfg.get("tip_offset_mm"), f"{where}.tip_offset_mm")
        if number != 0 and any(v != 0.0 for v in tip):
            raise ValueError(
                f"{where}.tip_offset_mm must be omitted/zero for tool file {number}: the "
                "controller's tool file already places the TCP at the tip (no duplicated offset)"
            )
        marker = _triple(cfg.get("marker_offset_mm"), f"{where}.marker_offset_mm")
        if all(v == 0.0 for v in marker):
            table.notes.append(
                f"tools.{name}.marker_offset_mm is all zero — measure it before calibrating "
                f"with tool file {number}"
            )
        profile = ToolProfile(
            name=str(name),
            tool_number=number,
            tip_offset=ToolOffset(*tip),
            marker_offset_mm=marker,
            compression_mm=float(cfg.get("compression_mm", 0.0)),
            pick_tip_z_mm=float(cfg.get("pick_tip_z_mm", 300.0)),
            approach_tip_z_mm=float(cfg.get("approach_tip_z_mm", 401.0)),
            descend_profile=str(cfg.get("descend_profile") or ""),
        )
        if table.by_number(number) is not None:
            raise ValueError(
                f"{where}.tool_number {number} is already used by "
                f"tools.{table.by_number(number).name}"
            )
        table.by_name[str(name)] = profile

    if NONE_TOOL not in table.by_name:
        raise ValueError(f"{path}: tools must contain `none` (tool_number 0)")
    if "active_tool" in data:
        table.notes.append(
            "`active_tool` in cobot_setup.yaml is ignored: the active tool is selected on the "
            "dashboard (/cobot/set_active_tool) and read from /cobot/state"
        )
    return table


def sensor_names(path: str) -> list[str]:
    data = load_setup(path)
    io = data.get("io") or {}
    sensors = io.get("sensors") or {}
    return [str(k) for k in sensors]
