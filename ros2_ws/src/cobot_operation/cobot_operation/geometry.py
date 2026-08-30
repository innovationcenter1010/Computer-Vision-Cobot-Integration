"""Pure geometry / value types for the choreography. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto


@dataclass(frozen=True)
class Pose6:
    """Commanded toolhead pose, robot base frame, mm / degrees."""

    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float

    def __str__(self) -> str:
        return (
            f"({self.x:.1f}, {self.y:.1f}, {self.z:.1f}, "
            f"{self.rx:.1f}, {self.ry:.1f}, {self.rz:.1f})"
        )

    @staticmethod
    def from_list(values, where: str = "pose") -> "Pose6":
        if len(values) != 6:
            raise ValueError(f"{where}: expected 6 numbers, got {len(values)}")
        return Pose6(*(float(v) for v in values))


@dataclass
class ToolOffset:
    """Tool tip relative to the OFFICIAL toolhead point — the pose the
    controller actually commands.

    Two things hang off that same reference point and neither sits on it:
    the end of the tooling (this, applied here) and the red calibration dot
    (localization_node.marker_offset_mm, applied on the vision side before
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
        """(x, y, z) where the TIP must land → the toolhead pose to command."""
        theta = math.radians(rz)
        dx = self.x * math.cos(theta) - self.y * math.sin(theta)
        dy = self.x * math.sin(theta) + self.y * math.cos(theta)
        return x - dx, y - dy, z - self.z


@dataclass
class ToolProfile:
    """One `tools.<name>` entry of cobot_setup.yaml."""

    name: str
    tool_number: int = 0
    tip_offset: ToolOffset = field(default_factory=ToolOffset)
    marker_offset_mm: tuple = (0.0, 0.0, 0.0)
    compression_mm: float = 0.0  # declared, not yet applied (as before)
    # Pick heights, as TOOL TIP height in robot base mm (tip_offset converts
    # them to a commanded toolhead pose). With a zero tip offset these are
    # commanded-toolhead heights — the pre-offset behaviour.
    pick_tip_z_mm: float = 300.0
    approach_tip_z_mm: float = 401.0
    descend_profile: str = ""  # cobot_control motion profile for descents; "" = default


class PlacePosition(Enum):
    right = auto()
    left = auto()
