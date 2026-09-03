"""Remembered positions (runtime_config/trajectories.json, schema v2). Pure
Python — json / os / pathlib only, no ROS — so the whole lifecycle is
pytest-able: seed, v1 migration, validation, all-or-nothing conversion,
atomic save.

Every pose carries its coordinate system:
    x y z rx ry rz   commanded TCP pose, robot BASE frame, mm / deg
    frame            "base" (the only frame the stack commands today)
    tool_number      YRC1000 tool file the pose is expressed in
    joints_deg       S L U R B T — canonical, tool-independent form once the
                     controller has converted the pose (null until then)
    taught_at / converted_from   provenance

Conversion between tool files is NEVER computed here: the node asks
cobot_control's /cobot/convert_pose (the controller's own kinematics) per
point and hands the results to `apply_conversion`, which rewrites the file
only if EVERY point converted.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional

from cobot_operation.geometry import Pose6

SCHEMA_VERSION = 2
FRAME_BASE = "base"
REQUIRED_KEYS = ("check", "right", "right_flip", "left", "left_flip")
REQUIRED_COUNTS = {"check": 1, "right": 2, "right_flip": 2, "left": 2, "left_flip": 2}
POSE_FIELDS = ("x", "y", "z", "rx", "ry", "rz")


def _now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


@dataclass
class TeachPoint:
    pose: Pose6  # tool_number >= 0 always
    joints_deg: Optional[tuple] = None
    taught_at: Optional[str] = None
    converted_from: Optional[dict] = None

    @property
    def tool_number(self) -> int:
        return self.pose.tool_number

    def to_json(self) -> dict:
        return {
            "x": self.pose.x, "y": self.pose.y, "z": self.pose.z,
            "rx": self.pose.rx, "ry": self.pose.ry, "rz": self.pose.rz,
            "frame": FRAME_BASE,
            "tool_number": int(self.pose.tool_number),
            "joints_deg": list(self.joints_deg) if self.joints_deg is not None else None,
            "taught_at": self.taught_at,
            "converted_from": self.converted_from,
        }

    @staticmethod
    def from_json(obj, where: str) -> "TeachPoint":
        if not isinstance(obj, dict):
            raise ValueError(f"{where}: expected a pose object, got {type(obj).__name__}")
        missing = [k for k in POSE_FIELDS if k not in obj]
        if missing:
            raise ValueError(f"{where}: missing {missing}")
        try:
            values = [float(obj[k]) for k in POSE_FIELDS]
        except (TypeError, ValueError) as e:
            raise ValueError(f"{where}: pose values must be numbers ({e})") from e
        frame = obj.get("frame", FRAME_BASE)
        if frame != FRAME_BASE:
            raise ValueError(f"{where}: frame '{frame}' unsupported (only '{FRAME_BASE}')")
        tool = int(obj.get("tool_number", 0))
        if tool < 0 or tool > 63:
            raise ValueError(f"{where}: tool_number must be 0-63, got {tool}")
        joints = obj.get("joints_deg")
        if joints is not None:
            if len(joints) != 6:
                raise ValueError(f"{where}: joints_deg must have 6 values")
            joints = tuple(float(v) for v in joints)
        return TeachPoint(
            pose=Pose6(*values, tool_number=tool),
            joints_deg=joints,
            taught_at=obj.get("taught_at"),
            converted_from=obj.get("converted_from"),
        )


@dataclass
class ConvertResult:
    """One /cobot/convert_pose outcome, as the node reports it back."""

    ok: bool
    pose: Optional[Pose6] = None  # expressed in to_tool
    joints_deg: Optional[tuple] = None
    message: str = ""


@dataclass
class ConvertReport:
    to_tool: int
    converted: int = 0
    skipped: int = 0  # already in to_tool
    failures: list = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.failures

    def summary(self) -> str:
        if self.ok:
            return f"{self.converted} pose(s) converted to tool file {self.to_tool}, {self.skipped} already there"
        return f"conversion to tool file {self.to_tool} FAILED, file untouched: {self.failures[0]}"


class TrajectoryStore:
    def __init__(self, path, points: dict, extra: Optional[dict] = None, migrated: bool = False,
                 updated_at: Optional[str] = None):
        self.path = Path(path) if path is not None else None
        self.points: dict = points  # key -> list[TeachPoint]
        self.extra: dict = dict(extra or {})  # unknown top-level keys, preserved
        self.migrated = migrated
        self.updated_at = updated_at

    # ------------------------------------------------------------------ load

    @classmethod
    def load(cls, path) -> "TrajectoryStore":
        p = Path(path)
        raw = json.loads(p.read_text(encoding="utf-8"))
        if not isinstance(raw, dict):
            raise ValueError(f"{p}: top level must be an object")
        if "version" not in raw:
            store = cls.from_v1(raw, p)
            return store
        version = int(raw.get("version", 0))
        if version != SCHEMA_VERSION:
            raise ValueError(f"{p}: unsupported trajectories schema version {version}")
        points_raw = raw.get("points")
        if not isinstance(points_raw, dict):
            raise ValueError(f"{p}: `points` must be an object")
        points = {}
        for key, lst in points_raw.items():
            if not isinstance(lst, list):
                raise ValueError(f"{p}: points.{key} must be a list of poses")
            points[key] = [TeachPoint.from_json(obj, f"{p}: points.{key}[{i}]") for i, obj in enumerate(lst)]
        extra = {k: v for k, v in raw.items() if k not in ("version", "updated_at", "points")}
        return cls(p, points, extra, migrated=False, updated_at=raw.get("updated_at"))

    @classmethod
    def from_v1(cls, data: dict, path=None) -> "TrajectoryStore":
        """Flat v1 file: key -> [x,y,z,rx,ry,rz] or key -> [[...], [...]].
        Every pose was a raw commanded pose in tool file 0."""
        points = {}
        extra = {}
        where = str(path) if path is not None else "trajectories"
        for key, value in data.items():
            if isinstance(value, list) and value and all(isinstance(v, (int, float)) for v in value):
                value = [value]
            if isinstance(value, list) and value and all(isinstance(v, list) for v in value):
                points[key] = [
                    TeachPoint(pose=Pose6.from_list(v, f"{where}: {key}[{i}]", tool_number=0))
                    for i, v in enumerate(value)
                ]
            else:
                extra[key] = value
        return cls(path, points, extra, migrated=True)

    @classmethod
    def seed(cls, path, default_path, log: Optional[Callable[[str], None]] = None) -> "TrajectoryStore":
        """Ensure the runtime file exists (copied from the packaged default),
        then load it."""
        p = Path(path)
        if not p.exists():
            src = Path(default_path)
            if not src.is_file():
                raise FileNotFoundError(
                    f"trajectories file {p} missing and default {src} not found"
                )
            p.parent.mkdir(parents=True, exist_ok=True)
            _atomic_write(p, src.read_text(encoding="utf-8"))
            if log:
                log(f"seeded {p} from {src}")
        return cls.load(p)

    # ------------------------------------------------------------------ queries

    def validate(self) -> None:
        for key in REQUIRED_KEYS:
            if key not in self.points:
                raise ValueError(f"trajectories.json is missing '{key}'")
            want = REQUIRED_COUNTS[key]
            if len(self.points[key]) != want:
                raise ValueError(
                    f"trajectories.json '{key}' must hold exactly {want} pose(s), "
                    f"has {len(self.points[key])}"
                )

    def poses(self) -> dict:
        """key -> list[Pose6] (each carrying its tool_number) for PickConfig."""
        return {k: [tp.pose for tp in v] for k, v in self.points.items()}

    def all_points(self):
        for key, lst in self.points.items():
            for index, tp in enumerate(lst):
                yield key, index, tp

    def tool_numbers(self) -> set:
        return {tp.tool_number for _, _, tp in self.all_points()}

    def is_in_tool(self, tool_number: int) -> bool:
        return self.tool_numbers() == {int(tool_number)} or not self.points

    def count(self) -> int:
        return sum(len(v) for v in self.points.values())

    def summary(self) -> dict:
        pts = list(self.all_points())
        return {
            "path": str(self.path) if self.path else "",
            "count": len(pts),
            "tool_numbers": sorted(self.tool_numbers()),
            "with_joints": sum(1 for _, _, tp in pts if tp.joints_deg is not None),
            "updated_at": self.updated_at,
        }

    # ------------------------------------------------------------------ mutation

    def to_json(self) -> dict:
        out = dict(self.extra)
        out["version"] = SCHEMA_VERSION
        out["updated_at"] = self.updated_at
        out["points"] = {k: [tp.to_json() for tp in v] for k, v in self.points.items()}
        return out

    def save(self) -> None:
        if self.path is None:
            raise ValueError("store has no path")
        self.updated_at = _now_iso()
        _atomic_write(self.path, json.dumps(self.to_json(), indent=2) + "\n")

    def backup_v1(self, raw_text: str) -> Path:
        bak = self.path.with_suffix(self.path.suffix + ".v1.bak")
        if not bak.exists():
            _atomic_write(bak, raw_text)
        return bak

    def pending(self, to_tool: int) -> list:
        """[(key, index, TeachPoint)] not yet expressed in to_tool."""
        return [(k, i, tp) for k, i, tp in self.all_points() if tp.tool_number != int(to_tool)]

    def apply_conversion(self, to_tool: int, results: dict) -> ConvertReport:
        """results: {(key, index): ConvertResult} for every `pending` point.
        ALL-OR-NOTHING: any missing/failed result leaves memory and disk
        untouched; otherwise every pending point is rewritten and saved."""
        to_tool = int(to_tool)
        report = ConvertReport(to_tool=to_tool)
        pending = self.pending(to_tool)
        report.skipped = self.count() - len(pending)
        new_points = {k: list(v) for k, v in self.points.items()}
        now = _now_iso()
        for key, index, tp in pending:
            res = results.get((key, index))
            if res is None:
                report.failures.append(f"{key}[{index}]: no conversion result")
                continue
            if not res.ok or res.pose is None:
                report.failures.append(f"{key}[{index}]: {res.message or 'conversion failed'}")
                continue
            new_points[key][index] = TeachPoint(
                pose=res.pose.with_tool(to_tool),
                joints_deg=tuple(res.joints_deg) if res.joints_deg else tp.joints_deg,
                taught_at=tp.taught_at,
                converted_from={
                    "tool_number": tp.tool_number,
                    "at": now,
                    "had_joints": tp.joints_deg is not None,
                    "pose": list(tp.pose.xyzrpy()),
                },
            )
            report.converted += 1
        if not report.ok:
            report.converted = 0
            return report
        if pending:
            self.points = new_points
            self.save()
        return report

    def set_point(self, key: str, index: int, pose: Pose6, joints_deg=None,
                  taught_at: Optional[str] = None) -> TeachPoint:
        if key not in self.points:
            raise KeyError(f"unknown teach point '{key}' (have {sorted(self.points)})")
        if index < 0 or index >= len(self.points[key]):
            raise IndexError(f"{key}[{index}] out of range (0-{len(self.points[key]) - 1})")
        if pose.tool_number < 0:
            raise ValueError("a remembered pose needs an explicit tool_number")
        tp = TeachPoint(
            pose=pose,
            joints_deg=tuple(joints_deg) if joints_deg else None,
            taught_at=taught_at or _now_iso(),
            converted_from=None,
        )
        self.points[key][index] = tp
        self.save()
        return tp


def _atomic_write(path: Path, text: str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, path)
