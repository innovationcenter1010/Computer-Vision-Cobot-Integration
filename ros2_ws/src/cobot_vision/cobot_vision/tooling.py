"""Per-tool marker offsets from cobot_setup.yaml `tools` — the vision side's
only interest in the tool table.

The camera→robot fit maps pixels to ROBOT BASE coordinates and is
tool-independent; the tool enters only through the ground truth of each
calibration sample: commanded TCP (expressed in the ACTIVE tool file) +
that tool's marker offset. So localization needs exactly one number per
tool file: where the red dot sits relative to that tool's TCP.

Deliberately does NOT import cobot_operation (vision must not depend on
robot code); it reads the same YAML with the same key names.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class MarkerOffset:
    name: str
    tool_number: int
    offset_mm: tuple  # (x, y, z) along robot base axes at the calibration orientation

    @property
    def is_zero(self) -> bool:
        return all(v == 0.0 for v in self.offset_mm)


def load_marker_offsets(setup_file: str) -> dict:
    """{tool_number: MarkerOffset} from cobot_setup.yaml `tools`.
    Raises FileNotFoundError / ValueError with the key named."""
    p = Path(setup_file)
    if not p.is_file():
        raise FileNotFoundError(f"setup file not found: {setup_file}")
    data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    tools = data.get("tools") or {}
    if not isinstance(tools, dict):
        raise ValueError(f"{setup_file}: `tools` must be a mapping")
    table: dict = {}
    for name, cfg in tools.items():
        cfg = cfg or {}
        if not isinstance(cfg, dict):
            raise ValueError(f"{setup_file}: tools.{name} must be a mapping")
        number = int(cfg.get("tool_number", 0))
        raw = cfg.get("marker_offset_mm")
        if raw is None:
            offset = (0.0, 0.0, 0.0)
        elif len(raw) != 3:
            raise ValueError(f"{setup_file}: tools.{name}.marker_offset_mm must be [x, y, z]")
        else:
            offset = tuple(float(v) for v in raw)
        if number in table:
            raise ValueError(
                f"{setup_file}: tools.{name}.tool_number {number} is also used by tools.{table[number].name}"
            )
        table[number] = MarkerOffset(name=str(name), tool_number=number, offset_mm=offset)
    return table


def select_marker_offset(table: dict, tool_number: int) -> MarkerOffset | None:
    return table.get(int(tool_number))
