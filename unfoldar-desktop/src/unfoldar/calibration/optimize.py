from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from ..models import CameraProfile, ShotRecord
from .camera import project_points


@dataclass(frozen=True, slots=True)
class CalibrationCorrespondence:
    point_scan: tuple[float, float, float]
    pixel: tuple[float, float]
    shot: ShotRecord


@dataclass(slots=True)
class CalibrationResult:
    profile: CameraProfile
    initial_rms_pixels: float
    rms_pixels: float
    success: bool
    message: str
    evaluations: int


def optimize_profile(
    initial: CameraProfile,
    correspondences: list[CalibrationCorrespondence],
    groups: set[str] | None = None,
) -> CalibrationResult:
    groups = groups or {"boresight", "translation"}
    if len(correspondences) < 6:
        raise ValueError("At least six 3D/2D correspondences are required")
    if "boresight" in groups and "angle_zeros" in groups:
        raise ValueError(
            "Do not optimize boresight and pan/tilt zero offsets together; "
            "they are strongly correlated. Calibrate boresight/translation first."
        )
    if groups & {"boresight", "translation", "angle_zeros"}:
        distinct_poses = {(item.shot.cmd_az, item.shot.cmd_tilt) for item in correspondences}
        if len(distinct_poses) < 3:
            raise ValueError(
                "Extrinsic calibration needs correspondences from at least three different photographs"
            )
    profile = CameraProfile.from_dict(initial.to_dict())
    fields: list[tuple[str, int]] = []
    values: list[float] = []
    lower: list[float] = []
    upper: list[float] = []

    def add(name: str, current: list[float], lo: float | list[float], hi: float | list[float]) -> None:
        lows = [lo] * len(current) if isinstance(lo, (int, float)) else lo
        highs = [hi] * len(current) if isinstance(hi, (int, float)) else hi
        for i, value in enumerate(current):
            fields.append((name, i)); values.append(float(value)); lower.append(float(lows[i])); upper.append(float(highs[i]))

    if "intrinsics" in groups:
        add("intrinsics", [profile.fx, profile.fy, profile.cx, profile.cy], [20, 20, -profile.image_width, -profile.image_height], [5000, 5000, 2 * profile.image_width, 2 * profile.image_height])
    if "distortion" in groups:
        add("distortion", profile.distortion, -5.0, 5.0)
    if "boresight" in groups:
        current = profile.boresight_ypr_deg
        add("boresight", current, [value - 15.0 for value in current], [value + 15.0 for value in current])
    if "translation" in groups:
        current = profile.camera_center_xyz_m
        add("translation", current, [value - 0.30 for value in current], [value + 0.30 for value in current])
    if "angle_zeros" in groups:
        current = [profile.pan_zero_deg, profile.tilt_zero_deg]
        add("zeros", current, [value - 10.0 for value in current], [value + 10.0 for value in current])
    if not fields:
        raise ValueError("No calibration parameter groups selected")

    points = np.asarray([item.point_scan for item in correspondences], dtype=np.float64)
    pixels = np.asarray([item.pixel for item in correspondences], dtype=np.float64)

    def apply(vector: np.ndarray) -> None:
        buckets: dict[str, list[float]] = {}
        for (name, index), value in zip(fields, vector, strict=True):
            bucket = buckets.setdefault(name, [])
            while len(bucket) <= index:
                bucket.append(0.0)
            bucket[index] = float(value)
        if "intrinsics" in buckets:
            profile.fx, profile.fy, profile.cx, profile.cy = buckets["intrinsics"]
        if "distortion" in buckets: profile.distortion = buckets["distortion"]
        if "boresight" in buckets: profile.boresight_ypr_deg = buckets["boresight"]
        if "translation" in buckets: profile.camera_center_xyz_m = buckets["translation"]
        if "zeros" in buckets: profile.pan_zero_deg, profile.tilt_zero_deg = buckets["zeros"]

    def residual(vector: np.ndarray) -> np.ndarray:
        apply(vector)
        out = np.empty((len(points), 2), dtype=np.float64)
        for i, item in enumerate(correspondences):
            projection = project_points(points[i : i + 1], profile, item.shot)
            out[i] = projection.uv[0] - pixels[i] if projection.valid[0] else (1000.0, 1000.0)
        return out.ravel()

    initial_errors = residual(np.asarray(values, dtype=np.float64)).reshape(-1, 2)
    initial_rms = float(np.sqrt(np.mean(np.sum(initial_errors * initial_errors, axis=1))))
    solved = least_squares(
        residual,
        values,
        bounds=(lower, upper),
        loss="huber",
        f_scale=3.0,
        x_scale="jac",
        max_nfev=500,
    )
    apply(solved.x)
    errors = residual(solved.x).reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum(errors * errors, axis=1))))
    return CalibrationResult(profile, initial_rms, rms, bool(solved.success), str(solved.message), int(solved.nfev))
