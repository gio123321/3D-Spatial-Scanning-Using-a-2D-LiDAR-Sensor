from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from ..models import CameraProfile, LensModel, ShotRecord


@dataclass(slots=True)
class Projection:
    uv: np.ndarray
    depth: np.ndarray
    valid: np.ndarray
    camera_points: np.ndarray


def _axis_angle(axis: list[float] | np.ndarray, angle_deg: float) -> np.ndarray:
    vector = np.asarray(axis, dtype=np.float64)
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        raise ValueError("Rotation axis cannot be zero")
    return Rotation.from_rotvec(vector / norm * np.deg2rad(angle_deg)).as_matrix()


def validate_rotation_matrix(matrix: np.ndarray, *, name: str) -> None:
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f"{name} must be 3x3")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6) or np.linalg.det(matrix) < 0.999:
        raise ValueError(f"{name} must be a right-handed orthonormal matrix")


def map_lidar_points(points: np.ndarray, profile: CameraProfile) -> np.ndarray:
    axis_map = np.asarray(profile.axis_map, dtype=np.float64)
    validate_rotation_matrix(axis_map, name="axis_map")
    points = np.asarray(points, dtype=np.float64)
    return (points @ axis_map.T) * float(profile.unit_scale_to_m)


def camera_pose(profile: CameraProfile, shot: ShotRecord) -> np.ndarray:
    """Return camera-to-scan transform for a shot using commanded angles."""
    basis = np.asarray(profile.camera_basis, dtype=np.float64)
    validate_rotation_matrix(basis, name="camera_basis")
    pan = _axis_angle(profile.pan_axis, profile.pan_sign * (shot.cmd_az + profile.pan_zero_deg))
    tilt = _axis_angle(profile.tilt_axis, profile.tilt_sign * (shot.cmd_tilt + profile.tilt_zero_deg))
    yaw, pitch, roll = profile.boresight_ypr_deg
    boresight = Rotation.from_euler("zyx", [yaw, pitch, roll], degrees=True).as_matrix()
    stage_rotation = pan @ tilt if profile.rotation_order == "pan_tilt" else tilt @ pan
    if profile.rotation_order not in {"pan_tilt", "tilt_pan"}:
        raise ValueError("rotation_order must be 'pan_tilt' or 'tilt_pan'")
    rotation = stage_rotation @ basis @ boresight
    center = pan @ np.asarray(profile.camera_center_xyz_m, dtype=np.float64)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = center
    return transform


def project_camera_points(camera_points: np.ndarray, profile: CameraProfile) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(camera_points, dtype=np.float64)
    if profile.lens_model == LensModel.FISHEYE:
        distortion = np.asarray(profile.distortion[:4], dtype=np.float64).reshape(4, 1)
        uv, _ = cv2.fisheye.projectPoints(
            points.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), profile.intrinsic_matrix, distortion
        )
    else:
        distortion = np.asarray(profile.distortion, dtype=np.float64)
        uv, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), profile.intrinsic_matrix, distortion)
    uv = uv.reshape(-1, 2)
    finite = np.isfinite(uv).all(axis=1)
    return uv, finite


def project_points(points_scan: np.ndarray, profile: CameraProfile, shot: ShotRecord) -> Projection:
    transform = camera_pose(profile, shot)
    rotation, center = transform[:3, :3], transform[:3, 3]
    camera_points = (np.asarray(points_scan, dtype=np.float64) - center) @ rotation
    depth = camera_points[:, 2]
    front = depth > 1e-5
    uv = np.full((len(camera_points), 2), np.nan, dtype=np.float64)
    finite = np.zeros(len(camera_points), dtype=bool)
    if np.any(front):
        projected, projected_finite = project_camera_points(camera_points[front], profile)
        uv[front] = projected
        finite[front] = projected_finite
    valid = (
        front
        & finite
        & (uv[:, 0] >= 0)
        & (uv[:, 0] < profile.image_width)
        & (uv[:, 1] >= 0)
        & (uv[:, 1] < profile.image_height)
    )
    return Projection(uv=uv, depth=depth, valid=valid, camera_points=camera_points)
