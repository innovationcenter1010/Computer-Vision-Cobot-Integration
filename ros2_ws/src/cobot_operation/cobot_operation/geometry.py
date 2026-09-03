"""Pure geometry / value types for the choreography. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import Enum, auto

ACTIVE_TOOL = -1  # Pose6.tool_number sentinel: "whatever tool file is active" (MoveToPose -1)


@dataclass(frozen=True)
class Pose6:
    """Commanded TCP pose, robot base frame, mm / degrees, expressed in the
    YRC1000 tool file `tool_number` (-1 = the active tool, the MoveToPose
    default). Remembered positions always carry an explicit tool number —
    the pose only means something together with the TCP it was taught in."""

    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float
    tool_number: int = ACTIVE_TOOL

    def __str__(self) -> str:
        s = (
            f"({self.x:.1f}, {self.y:.1f}, {self.z:.1f}, "
            f"{self.rx:.1f}, {self.ry:.1f}, {self.rz:.1f})"
        )
        return s + (f" T{self.tool_number}" if self.tool_number >= 0 else "")

    def with_tool(self, tool_number: int) -> "Pose6":
        return replace(self, tool_number=int(tool_number))

    def xyzrpy(self) -> tuple:
        return (self.x, self.y, self.z, self.rx, self.ry, self.rz)

    @staticmethod
    def from_list(values, where: str = "pose", tool_number: int = ACTIVE_TOOL) -> "Pose6":
        if len(values) != 6:
            raise ValueError(f"{where}: expected 6 numbers, got {len(values)}")
        return Pose6(*(float(v) for v in values), tool_number=int(tool_number))


@dataclass
class ToolOffset:
    """Tool tip relative to the commanded TCP — used ONLY for tool file 0
    ("none"), where the controller knows nothing about the tooling. With a
    real tool file active the controller's TCP already IS the tip and this
    offset must be zero (setup_config enforces it), so the TCP math is never
    duplicated.

    Two things hang off the tool-0 reference point and neither sits on it:
    the end of the tooling (this, applied here) and the red calibration dot
    (tools.<name>.marker_offset_mm, applied on the vision side before
    fitting). Same origin, different features — measure the two separately.

    Measured along ROBOT BASE axes with the tool at rz = 0 and the pick
    orientation (rx=180, ry=0). The XY part rotates with the commanded rz;
    Z does not, since rx/ry are fixed.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def is_empty(self) -> bool:
        return self.x == 0.0 and self.y == 0.0 and self.z == 0.0

    def command_for(self, x: float, y: float, z: float, rz: float):
        """(x, y, z) where the TIP must land → the TCP pose to command."""
        theta = math.radians(rz)
        dx = self.x * math.cos(theta) - self.y * math.sin(theta)
        dy = self.x * math.sin(theta) + self.y * math.cos(theta)
        return x - dx, y - dy, z - self.z


@dataclass
class ToolProfile:
    """One `tools.<name>` entry of cobot_setup.yaml — the per-tool CELL values."""

    name: str
    tool_number: int = 0
    tip_offset: ToolOffset = field(default_factory=ToolOffset)  # tool 0 only
    marker_offset_mm: tuple = (0.0, 0.0, 0.0)  # red dot relative to THIS tool's TCP
    compression_mm: float = 0.0  # declared, not yet applied (as before)
    # Pick heights, as TOOL TIP height in robot base mm (tip_offset converts
    # them to a commanded pose for tool 0; for a real tool file the TCP is
    # the tip, so they are commanded directly).
    pick_tip_z_mm: float = 300.0
    approach_tip_z_mm: float = 401.0
    descend_profile: str = ""  # cobot_control motion profile for descents; "" = default

    @property
    def uses_python_tip_offset(self) -> bool:
        return self.tool_number == 0


class PlacePosition(Enum):
    right = auto()
    left = auto()
