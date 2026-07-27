"""Camera→robot calibration model. Pure math — zero ROS imports.

The model
---------
A pinhole camera composed with a rigid camera→robot transform is exactly
linear in the features [u·d, v·d, d, 1]  (back-projection is
X_cam = (u - cx)·d / fx, linear in u·d and d; a rigid transform keeps it
linear). So one 3x4 matrix M maps a pixel + its depth reading straight into
robot base coordinates:

    [X, Y, Z]^T = M @ [u*d, v*d, d, 1]^T        (mm everywhere)

M factors into the two physical parts:

    M = [ R·B | t ]        B = [[1/fx, 0, -cx/fx],
                                [0, 1/fy, -cy/fy],   (intrinsics — STATIC)
                                [0,  0,     1   ]]
                           R, t = camera pose vs robot base (extrinsics — MOVE)

Two ways to obtain M:

- fit() — the full monolithic least squares. Fits all 12 numbers at once,
  absorbing the intrinsics; needs a wide depth span or Z is unconstrained
  (the d column becomes a multiple of the 1 column). No CameraInfo required.
- fit_extrinsic() — when the intrinsics are already known (camera SDK /
  CameraInfo). Back-projects each sample to a camera-frame 3D point with the
  fixed intrinsics, then solves the rigid transform {P_cam}→{P_robot} in
  closed form (Kabsch/Umeyama). Only 6 unknowns, so a handful of poses
  suffice and the depth-span rule relaxes (Z_cam = d is measured, not fit).
  This is the "small calibration at startup" that corrects for camera drift
  without re-measuring the intrinsics.

Lens distortion is the ignored residual either way; the per-axis RMS reported
by both fitters is the honest measure of it.
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml


MIN_SAMPLES = 6              # 4 solves the full fit; below 6 no meaningful residual
MIN_DEPTH_SPAN_MM = 30.0     # samples flatter than this cannot constrain Z (full fit)
MIN_EXTRINSIC_SAMPLES = 4    # a rigid transform needs 3 non-collinear points; 4 for margin


class CalibrationError(Exception):
    """Raised when a fit is impossible or a saved file is unusable."""


class CameraRobotCalibration:
    """A fitted (or loaded) camera→robot transform."""

    def __init__(self, matrix: np.ndarray, rms_mm: tuple[float, float, float],
                 sample_count: int, created: str = "", source: str = "full",
                 intrinsics: dict | None = None):
        matrix = np.asarray(matrix, dtype=np.float64)
        if matrix.shape != (3, 4):
            raise CalibrationError(f"matrix must be 3x4, got {matrix.shape}")
        self.matrix = matrix
        self.rms_mm = tuple(float(v) for v in rms_mm)
        self.sample_count = int(sample_count)
        self.created = created
        # "full" (monolithic fit) or "extrinsic" (Kabsch with known intrinsics).
        self.source = source
        # The intrinsics used/absorbed, if known: {fx, fy, cx, cy}. Persisted
        # so the next startup can run an extrinsic-only fit without them again.
        self.intrinsics = dict(intrinsics) if intrinsics else None

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
            source="full",
        )

    @staticmethod
    def _backproject(uvd: np.ndarray, fx, fy, cx, cy) -> np.ndarray:
        """(u, v, depth) rows → camera-frame 3D points (mm), given intrinsics."""
        u, v, d = uvd[:, 0], uvd[:, 1], uvd[:, 2]
        return np.column_stack([(u - cx) / fx * d, (v - cy) / fy * d, d])

    @classmethod
    def fit_extrinsic(cls, samples_uvd, samples_xyz, intrinsics) -> "CameraRobotCalibration":
        """Rigid (Kabsch) fit with KNOWN intrinsics — the small startup fit.

        Back-projects each sample to a camera-frame 3D point using the fixed
        intrinsics, then solves the rigid transform {P_cam} → {P_robot} in
        closed form. Reconstructs the same [R·B | t] matrix the runtime uses,
        so transform()/inverse() are unchanged.

        intrinsics: dict with fx, fy, cx, cy.
        """
        for key in ("fx", "fy", "cx", "cy"):
            if key not in intrinsics:
                raise CalibrationError(f"intrinsics missing '{key}'")
        fx, fy = float(intrinsics["fx"]), float(intrinsics["fy"])
        cx, cy = float(intrinsics["cx"]), float(intrinsics["cy"])
        if fx == 0.0 or fy == 0.0:
            raise CalibrationError("intrinsics fx/fy must be non-zero")

        uvd = np.asarray(samples_uvd, dtype=np.float64).reshape(-1, 3)
        xyz = np.asarray(samples_xyz, dtype=np.float64).reshape(-1, 3)
        if len(uvd) != len(xyz):
            raise CalibrationError(
                f"sample count mismatch: {len(uvd)} camera vs {len(xyz)} robot"
            )
        if len(uvd) < MIN_EXTRINSIC_SAMPLES:
            raise CalibrationError(
                f"need at least {MIN_EXTRINSIC_SAMPLES} samples, got {len(uvd)}"
            )

        p_cam = cls._backproject(uvd, fx, fy, cx, cy)

        # Points must not be collinear or the rotation axis is undetermined.
        centered_cam = p_cam - p_cam.mean(axis=0)
        if np.linalg.matrix_rank(centered_cam, tol=1e-6) < 2:
            raise CalibrationError(
                "degenerate sample geometry (marker points near-collinear) — "
                "spread samples across x, y and z"
            )

        # Kabsch/Umeyama rigid alignment P_cam -> P_robot (rotation only).
        cam_centroid = p_cam.mean(axis=0)
        rob_centroid = xyz.mean(axis=0)
        h = (p_cam - cam_centroid).T @ (xyz - rob_centroid)
        u_svd, _, vt = np.linalg.svd(h)
        # Reflection guard: force a proper rotation (det +1).
        sign = np.sign(np.linalg.det(vt.T @ u_svd.T))
        rotation = vt.T @ np.diag([1.0, 1.0, sign]) @ u_svd.T
        translation = rob_centroid - rotation @ cam_centroid

        # Rebuild M = [R·B | t] so the runtime path is identical to fit().
        b = np.array([
            [1.0 / fx, 0.0, -cx / fx],
            [0.0, 1.0 / fy, -cy / fy],
            [0.0, 0.0, 1.0],
        ])
        matrix = np.column_stack([rotation @ b, translation])

        predicted = (rotation @ p_cam.T).T + translation
        rms = tuple(np.sqrt(np.mean((predicted - xyz) ** 2, axis=0)))

        return cls(
            matrix=matrix,
            rms_mm=rms,
            sample_count=len(uvd),
            created=datetime.now(timezone.utc).isoformat(timespec="seconds"),
            source="extrinsic",
            intrinsics={"fx": fx, "fy": fy, "cx": cx, "cy": cy},
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
        calibration = {
            "model": "xyz = M @ [u*d, v*d, d, 1]  (mm)",
            "matrix": [[float(v) for v in row] for row in self.matrix],
            "rms_mm": {
                "x": self.rms_mm[0],
                "y": self.rms_mm[1],
                "z": self.rms_mm[2],
            },
            "sample_count": self.sample_count,
            "source": self.source,
            "created": self.created,
        }
        payload = {"camera_robot_calibration": calibration}
        # Static intrinsics block — kept beside the transform so the next
        # startup's extrinsic fit can reuse them if the camera is silent.
        if self.intrinsics:
            payload["camera_intrinsics"] = {
                "fx": float(self.intrinsics["fx"]),
                "fy": float(self.intrinsics["fy"]),
                "cx": float(self.intrinsics["cx"]),
                "cy": float(self.intrinsics["cy"]),
            }
        Path(path).write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "CameraRobotCalibration":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        try:
            data = raw["camera_robot_calibration"]
            intrinsics = raw.get("camera_intrinsics")
            return cls(
                matrix=np.array(data["matrix"], dtype=np.float64),
                rms_mm=(
                    data["rms_mm"]["x"],
                    data["rms_mm"]["y"],
                    data["rms_mm"]["z"],
                ),
                sample_count=data["sample_count"],
                created=data.get("created", ""),
                source=data.get("source", "full"),
                intrinsics=intrinsics,
            )
        except (KeyError, TypeError) as e:
            raise CalibrationError(f"unreadable calibration file {path}: {e}") from e
