"""Camera→robot calibration model. Pure math — zero ROS imports.

The model
---------
A pinhole camera composed with a rigid camera→robot transform is exactly
linear in the features [u·d, v·d, d, 1]  (back-projection is
X_cam = (u - cx)·d / fx, linear in u·d and d; a rigid transform keeps it
linear). So one 3x4 matrix M maps a pixel + its depth reading straight into
robot base coordinates:

    [X, Y, Z]^T = M @ [u*d, v*d, d, 1]^T        (mm everywhere)

Fitting M by least squares over red-dot samples absorbs the camera
intrinsics — we never need a CameraInfo. Lens distortion is the ignored
residual; the per-axis RMS reported by fit() is the honest measure of it.

The features degenerate when every sample shares one depth (column d becomes
a multiple of column 1) — fit() rejects that with a clear message, which is
exactly the "must sample multiple heights" rule enforced in math.
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml


MIN_SAMPLES = 6          # 4 solves it; below 6 there is no meaningful residual
MIN_DEPTH_SPAN_MM = 30.0  # samples flatter than this cannot constrain Z


class CalibrationError(Exception):
    """Raised when a fit is impossible or a saved file is unusable."""


class CameraRobotCalibration:
    """A fitted (or loaded) camera→robot transform."""

    def __init__(self, matrix: np.ndarray, rms_mm: tuple[float, float, float],
                 sample_count: int, created: str = ""):
        matrix = np.asarray(matrix, dtype=np.float64)
        if matrix.shape != (3, 4):
            raise CalibrationError(f"matrix must be 3x4, got {matrix.shape}")
        self.matrix = matrix
        self.rms_mm = tuple(float(v) for v in rms_mm)
        self.sample_count = int(sample_count)
        self.created = created

    # ------------------------------------------------------------------
    # Fitting
    # ------------------------------------------------------------------

    @staticmethod
    def _features(samples_uvd: np.ndarray) -> np.ndarray:
        u, v, d = samples_uvd[:, 0], samples_uvd[:, 1], samples_uvd[:, 2]
        return np.column_stack([u * d, v * d, d, np.ones_like(d)])

    @classmethod
    def fit(cls, samples_uvd, samples_xyz) -> "CameraRobotCalibration":
        """Least-squares fit from N matched samples.

        samples_uvd: N x 3 — (pixel u, pixel v, depth mm) of the marker
        samples_xyz: N x 3 — true marker position in robot base frame, mm
                     (commanded TCP + marker offset — offset already applied)
        """
        uvd = np.asarray(samples_uvd, dtype=np.float64).reshape(-1, 3)
        xyz = np.asarray(samples_xyz, dtype=np.float64).reshape(-1, 3)

        if len(uvd) != len(xyz):
            raise CalibrationError(
                f"sample count mismatch: {len(uvd)} camera vs {len(xyz)} robot"
            )
        if len(uvd) < MIN_SAMPLES:
            raise CalibrationError(
                f"need at least {MIN_SAMPLES} samples, got {len(uvd)}"
            )

        depth_span = float(uvd[:, 2].max() - uvd[:, 2].min())
        if depth_span < MIN_DEPTH_SPAN_MM:
            raise CalibrationError(
                f"depth readings span only {depth_span:.1f} mm "
                f"(need > {MIN_DEPTH_SPAN_MM:.0f}) — sample at multiple heights"
            )

        features = cls._features(uvd)

        # Rank check on column-normalized features: u*d columns are ~1e6 while
        # the bias column is 1, so normalize before judging degeneracy.
        scaled = features / np.abs(features).max(axis=0)
        if np.linalg.matrix_rank(scaled, tol=1e-6) < 4:
            raise CalibrationError(
                "degenerate sample geometry (collinear grid?) — "
                "spread samples across x, y and z"
            )

        # M^T (4x3), one lstsq for all three output axes.
        solution, *_ = np.linalg.lstsq(features, xyz, rcond=None)
        matrix = solution.T

        predicted = features @ solution
        rms = tuple(np.sqrt(np.mean((predicted - xyz) ** 2, axis=0)))

        return cls(
            matrix=matrix,
            rms_mm=rms,
            sample_count=len(uvd),
            created=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        )

    # ------------------------------------------------------------------
    # Transforms
    # ------------------------------------------------------------------

    def transform(self, u: float, v: float, d: float) -> tuple[float, float, float]:
        """(pixel u, pixel v, depth mm) → robot base (x, y, z) mm."""
        p = self.matrix @ np.array([u * d, v * d, d, 1.0])
        return float(p[0]), float(p[1]), float(p[2])

    def inverse(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Robot base (x, y, z) mm → the (u, v, depth mm) that would see it.

        M = [A | t]:  P = A·[u·d, v·d, d] + t  →  [u·d, v·d, d] = A⁻¹(P − t),
        then d is the third component and u, v divide out.
        """
        a = self.matrix[:, :3]
        t = self.matrix[:, 3]
        ud, vd, d = np.linalg.solve(a, np.array([x, y, z]) - t)
        if abs(d) < 1e-6:
            raise CalibrationError(f"point ({x}, {y}, {z}) maps to zero depth")
        return float(ud / d), float(vd / d), float(d)

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save(self, path: str | Path) -> None:
        payload = {
            "camera_robot_calibration": {
                "model": "xyz = M @ [u*d, v*d, d, 1]  (mm)",
                "matrix": [[float(v) for v in row] for row in self.matrix],
                "rms_mm": {
                    "x": self.rms_mm[0],
                    "y": self.rms_mm[1],
                    "z": self.rms_mm[2],
                },
                "sample_count": self.sample_count,
                "created": self.created,
            }
        }
        Path(path).write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "CameraRobotCalibration":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        try:
            data = raw["camera_robot_calibration"]
            return cls(
                matrix=np.array(data["matrix"], dtype=np.float64),
                rms_mm=(
                    data["rms_mm"]["x"],
                    data["rms_mm"]["y"],
                    data["rms_mm"]["z"],
                ),
                sample_count=data["sample_count"],
                created=data.get("created", ""),
            )
        except (KeyError, TypeError) as e:
            raise CalibrationError(f"unreadable calibration file {path}: {e}") from e
