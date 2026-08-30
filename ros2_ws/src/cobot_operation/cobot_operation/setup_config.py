"""cobot_setup.yaml — the `tools` / `active_tool` / `io.sensors` sections.

cobot_control reads the same file for io + motion_profiles (C++, yaml-cpp).
"""

from __future__ import annotations

from pathlib import Path

import yaml

from cobot_operation.geometry import ToolOffset, ToolProfile


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


def load_tools(path: str) -> tuple[dict[str, ToolProfile], str]:
    """Return ({name: ToolProfile}, active_tool_name)."""
    data = load_setup(path)
    tools_raw = data.get("tools") or {}
    if not isinstance(tools_raw, dict) or not tools_raw:
        raise ValueError(f"{path}: `tools` must be a non-empty mapping")

    tools: dict[str, ToolProfile] = {}
    for name, cfg in tools_raw.items():
        cfg = cfg or {}
        where = f"{path}: tools.{name}"
        if not isinstance(cfg, dict):
            raise ValueError(f"{where} must be a mapping")
        tip = _triple(cfg.get("tip_offset_mm"), f"{where}.tip_offset_mm")
        tools[name] = ToolProfile(
            name=str(name),
            tool_number=int(cfg.get("tool_number", 0)),
            tip_offset=ToolOffset(*tip),
            marker_offset_mm=_triple(cfg.get("marker_offset_mm"), f"{where}.marker_offset_mm"),
            compression_mm=float(cfg.get("compression_mm", 0.0)),
            pick_tip_z_mm=float(cfg.get("pick_tip_z_mm", 300.0)),
            approach_tip_z_mm=float(cfg.get("approach_tip_z_mm", 401.0)),
            descend_profile=str(cfg.get("descend_profile") or ""),
        )

    active = str(data.get("active_tool") or next(iter(tools)))
    if active not in tools:
        raise ValueError(f"{path}: active_tool '{active}' is not in tools {sorted(tools)}")
    return tools, active


def sensor_names(path: str) -> list[str]:
    data = load_setup(path)
    io = data.get("io") or {}
    sensors = io.get("sensors") or {}
    return [str(k) for k in sensors]
