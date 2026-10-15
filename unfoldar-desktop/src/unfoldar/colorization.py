from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
import open3d as o3d

from .calibration.camera import map_lidar_points, project_points
from .calibration.profile import rescale_profile
from .io.scan import image_pose_pairs, load_cloud, write_colored_cloud
from .models import CameraProfile, ScanRecord
from .photometric import apply_photo_gain, estimate_photo_gains


Progress = Callable[[float, str], None]
Cancelled = Callable[[], bool]


@dataclass(slots=True)
class ColorizationSettings:
    depth_absolute_tolerance_m: float = 0.035
    depth_relative_tolerance: float = 0.008
    zbuffer_splat_radius_px: int = 1
    normal_radius_m: float = 0.08
    normal_max_neighbors: int = 40
    blend_top_k: int = 2
    min_score: float = 0.01
    normalize_photos: bool = True
    normalization_strength: float = 0.65


@dataclass(slots=True)
class ColorizationResult:
    output_path: Path
    report_path: Path
    colored_count: int
    point_count: int
    coverage: float
    image_contributions: dict[int, int]


@dataclass(slots=True)
class CalibrationOverlayFrame:
    """Rendered calibration view plus the exact LiDAR samples drawn in it."""

    image_rgb: np.ndarray
    overlay_rgb: np.ndarray
    points_scan: np.ndarray
    projected_uv: np.ndarray
    visible_point_ids: np.ndarray


def point_zbuffer(
    uv: np.ndarray,
    depth: np.ndarray,
    valid: np.ndarray,
    width: int,
    height: int,
    splat_radius: int = 1,
) -> np.ndarray:
    zbuffer = np.full(width * height, np.inf, dtype=np.float32)
    ids = np.flatnonzero(valid)
    if not len(ids):
        return zbuffer.reshape(height, width)
    x = np.rint(uv[ids, 0]).astype(np.int32)
    y = np.rint(uv[ids, 1]).astype(np.int32)
    z = depth[ids].astype(np.float32)
    for dy in range(-splat_radius, splat_radius + 1):
        yy = y + dy
        for dx in range(-splat_radius, splat_radius + 1):
            xx = x + dx
            inside = (xx >= 0) & (xx < width) & (yy >= 0) & (yy < height)
            np.minimum.at(zbuffer, yy[inside] * width + xx[inside], z[inside])
    return zbuffer.reshape(height, width)


def visible_points(
    uv: np.ndarray,
    depth: np.ndarray,
    valid: np.ndarray,
    zbuffer: np.ndarray,
    absolute_tolerance_m: float,
    relative_tolerance: float,
) -> np.ndarray:
    visible = np.zeros(len(depth), dtype=bool)
    ids = np.flatnonzero(valid)
    if not len(ids):
        return visible
    h, w = zbuffer.shape
    x = np.clip(np.rint(uv[ids, 0]).astype(np.int32), 0, w - 1)
    y = np.clip(np.rint(uv[ids, 1]).astype(np.int32), 0, h - 1)
    nearest = zbuffer[y, x]
    tolerance = np.maximum(absolute_tolerance_m, relative_tolerance * depth[ids])
    visible[ids] = depth[ids] <= nearest + tolerance
    return visible


def estimate_scan_normals(points: np.ndarray, radius: float, max_neighbors: int) -> np.ndarray:
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_neighbors))
    cloud.orient_normals_towards_camera_location(np.zeros(3))
    return np.asarray(cloud.normals).copy()


def _sample_image(image_rgb: np.ndarray, uv: np.ndarray, ids: np.ndarray) -> np.ndarray:
    h, w = image_rgb.shape[:2]
    x = np.clip(np.rint(uv[ids, 0]).astype(np.int32), 0, w - 1)
    y = np.clip(np.rint(uv[ids, 1]).astype(np.int32), 0, h - 1)
    return image_rgb[y, x]


def _view_scores(
    points: np.ndarray,
    normals: np.ndarray,
    uv: np.ndarray,
    depth: np.ndarray,
    ids: np.ndarray,
    camera_center: np.ndarray,
    image_rgb: np.ndarray,
) -> np.ndarray:
    h, w = image_rgb.shape[:2]
    nx = (uv[ids, 0] - (w - 1) / 2) / (w / 2)
    ny = (uv[ids, 1] - (h - 1) / 2) / (h / 2)
    center = np.clip(1.0 - 0.55 * np.sqrt(nx * nx + ny * ny), 0.05, 1.0) ** 2
    rays = camera_center - points[ids]
    ranges = np.linalg.norm(rays, axis=1)
    rays /= np.maximum(ranges[:, None], 1e-9)
    incidence = np.clip(np.abs(np.sum(normals[ids] * rays, axis=1)), 0.15, 1.0)
    colors = _sample_image(image_rgb, uv, ids).astype(np.float32)
    luma = colors @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
    exposure = np.clip(1.0 - np.abs(luma - 127.5) / 170.0, 0.2, 1.0)
    saturation = 1.0 - np.mean((colors < 4) | (colors > 251), axis=1) * 0.7
    gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
    sharp = np.abs(cv2.Laplacian(gray, cv2.CV_32F))
    local_sharp = _sample_image(sharp[..., None], uv, ids).reshape(-1)
    sharp_scale = max(float(np.percentile(local_sharp, 90)), 1.0)
    sharpness = 0.6 + 0.4 * np.clip(local_sharp / sharp_scale, 0.0, 1.0)
    resolution = 1.0 / np.sqrt(np.maximum(depth[ids], 0.1))
    return center * incidence * exposure * saturation * sharpness * resolution


def colorize_scan(
    scan: ScanRecord,
    profile: CameraProfile,
    output_path: str | Path,
    settings: ColorizationSettings | None = None,
    progress: Progress | None = None,
    cancelled: Cancelled | None = None,
    source_path: str | Path | None = None,
    azimuth_scale: float = 1.0,
) -> ColorizationResult:
    from .publication import staged_call, protect_raw_output
    protect_raw_output(output_path,[scan])
    output_path = Path(output_path)
    return staged_call(output_path.parent, lambda staging: _colorize_scan(
        scan, profile, staging / output_path.name, settings, progress, cancelled,
        source_path, azimuth_scale), cancelled or (lambda: False))


def _colorize_scan(scan, profile, output_path, settings, progress, cancelled, source_path, azimuth_scale):
    settings = settings or ColorizationSettings()
    progress = progress or (lambda _fraction, _message: None)
    cancelled = cancelled or (lambda: False)
    source = load_cloud(source_path or scan.cloud_path, require_raw=source_path is None)
    points = map_lidar_points(source.xyz, profile)
    usable = image_pose_pairs(scan, profile.image_pose_offset_frames, azimuth_scale)
    # Decode once for gain estimation and projection.
    decoded_images: dict[int, np.ndarray] = {}
    for image_shot,_pose_shot in usable:
        image_bgr = cv2.imread(str(scan.image_paths[image_shot.index]), cv2.IMREAD_COLOR)
        if image_bgr is None:
            raise ValueError(f"Cannot decode {scan.image_paths[image_shot.index]}")
        decoded_images[image_shot.index] = image_bgr
    if settings.normalize_photos:
        progress(0.01,"Measuring photograph exposure and color balance")
        gain_values=estimate_photo_gains(
            images=[decoded_images[image_shot.index] for image_shot,_pose_shot in usable],
            strength=settings.normalization_strength,
        )
    else:
        gain_values=[np.ones(3,dtype=np.float32) for _item in usable]
    photo_gains={image_shot.index:gain for (image_shot,_pose_shot),gain in zip(usable,gain_values,strict=True)}
    progress(0.03, "Estimating point normals")
    normals = estimate_scan_normals(points, settings.normal_radius_m, settings.normal_max_neighbors)
    k = max(1, min(int(settings.blend_top_k), 3))
    best_scores = np.full((len(points), k), -np.inf, dtype=np.float32)
    best_colors = np.zeros((len(points), k, 3), dtype=np.uint8)
    best_shots = np.full((len(points), k), -1, dtype=np.int32)
    for position, (image_shot, pose_shot) in enumerate(usable):
        if cancelled():
            raise InterruptedError("Colorization cancelled")
        image_bgr = decoded_images[image_shot.index]
        effective_profile=rescale_profile(profile,image_bgr.shape[1],image_bgr.shape[0])
        image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        if settings.normalize_photos:
            image = apply_photo_gain(image, photo_gains[image_shot.index])
        projection = project_points(points, effective_profile, pose_shot)
        zbuffer = point_zbuffer(
            projection.uv,
            projection.depth,
            projection.valid,
            effective_profile.image_width,
            effective_profile.image_height,
            settings.zbuffer_splat_radius_px,
        )
        visible = visible_points(
            projection.uv,
            projection.depth,
            projection.valid,
            zbuffer,
            settings.depth_absolute_tolerance_m,
            settings.depth_relative_tolerance,
        )
        ids = np.flatnonzero(visible)
        if len(ids):
            from .calibration.camera import camera_pose

            center = camera_pose(effective_profile, pose_shot)[:3, 3]
            scores = _view_scores(points, normals, projection.uv, projection.depth, ids, center, image)
            weakest = np.argmin(best_scores[ids], axis=1)
            replace = scores > best_scores[ids, weakest]
            selected = ids[replace]
            slots = weakest[replace]
            best_scores[selected, slots] = scores[replace]
            best_colors[selected, slots] = _sample_image(image, projection.uv, selected)
            best_shots[selected, slots] = image_shot.index
        progress(0.05 + 0.9 * (position + 1) / max(len(usable), 1), f"Projected image {image_shot.index:03d} at pose {pose_shot.index:03d}")
    valid_scores = best_scores > settings.min_score
    weights = np.where(valid_scores, np.maximum(best_scores, 0.0), 0.0)
    denom = weights.sum(axis=1)
    colored = denom > 0
    rgb = np.zeros((len(points), 3), dtype=np.uint8)
    if np.any(colored):
        blended = np.sum(best_colors.astype(np.float32) * weights[..., None], axis=1)
        rgb[colored] = np.clip(blended[colored] / denom[colored, None], 0, 255).astype(np.uint8)
    output = write_colored_cloud(source, output_path, rgb, colored.astype(np.uint8), normals)
    contributions: dict[int, int] = {}
    winner = np.argmax(best_scores, axis=1)
    winning_shots = best_shots[np.arange(len(points)), winner]
    for shot_id in winning_shots[colored]:
        contributions[int(shot_id)] = contributions.get(int(shot_id), 0) + 1
    report_path = output.with_suffix(".report.json")
    report_path.write_text(
        json.dumps(
            {
                "scan_id": scan.scan_id,
                "geometry_source": "lidar_points_only",
                "image_only_regions_ignored": True,
                "pose_source": "cmd_az/cmd_tilt only",
                "azimuth_scale": float(azimuth_scale),
                "camera_profile": {
                    "name": profile.name,
                    "lens_model": profile.lens_model.value,
                    "fx": profile.fx,
                    "fy": profile.fy,
                    "cx": profile.cx,
                    "cy": profile.cy,
                    "distortion": profile.distortion,
                },
                "point_count": len(points),
                "colored_count": int(colored.sum()),
                "coverage": float(colored.mean()),
                "image_contributions": contributions,
                "photo_normalization_gains_rgb": {str(key):[float(value) for value in gain] for key,gain in photo_gains.items()},
                "image_pose_offset_frames": scan.session.image_pose_offset_frames if scan.session.image_pose_offset_frames is not None else profile.image_pose_offset_frames,
                "skipped_images_without_pose": len(scan.image_paths) - len(usable),
                "settings": asdict(settings),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    progress(1.0, f"Colorized {colored.mean():.1%} of points")
    return ColorizationResult(output, report_path, int(colored.sum()), len(points), float(colored.mean()), contributions)


def calibration_overlay_frame(
    scan: ScanRecord,
    profile: CameraProfile,
    shot_index: int,
    *,
    opacity: float = 0.75,
    point_radius: int = 1,
    max_points: int = 50_000,
    source_path: str | Path | None = None,
    azimuth_scale: float = 1.0,
) -> CalibrationOverlayFrame:
    source = load_cloud(source_path or scan.cloud_path, require_raw=source_path is None)
    points = map_lidar_points(source.xyz, profile)
    pairs = {image.index: pose for image, pose in image_pose_pairs(scan, profile.image_pose_offset_frames, azimuth_scale)}
    if shot_index not in pairs:
        raise ValueError(f"Image {shot_index:03d} has no trustworthy associated pose")
    shot = pairs[shot_index]
    image_bgr = cv2.imread(str(scan.image_paths[shot_index]), cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise ValueError(f"Cannot decode shot {shot_index}")
    image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    effective_profile=rescale_profile(profile,image.shape[1],image.shape[0])
    projection = project_points(points, effective_profile, shot)
    zbuffer = point_zbuffer(projection.uv, projection.depth, projection.valid, effective_profile.image_width, effective_profile.image_height)
    visible = visible_points(projection.uv, projection.depth, projection.valid, zbuffer, 0.035, 0.008)
    ids = np.flatnonzero(visible)
    if len(ids) > max_points:
        ids = ids[np.linspace(0, len(ids) - 1, max_points, dtype=np.int64)]
    overlay = image.copy()
    if len(ids):
        depth = projection.depth[ids]
        lo, hi = np.percentile(depth, [2, 98])
        normalized = np.clip((depth - lo) / max(hi - lo, 1e-6), 0, 1)
        # An explicit column avoids OpenCV treating N colors as one image row.
        color_values = (normalized * 255).astype(np.uint8).reshape(-1, 1)
        colors = cv2.applyColorMap(color_values, cv2.COLORMAP_TURBO).reshape(-1, 3)[:, ::-1]
        for idx, color in zip(ids, colors, strict=True):
            x, y = np.rint(projection.uv[idx]).astype(int)
            cv2.circle(overlay, (x, y), point_radius, tuple(int(v) for v in color), -1, cv2.LINE_AA)
    blended = cv2.addWeighted(overlay, float(opacity), image, 1.0 - float(opacity), 0)
    return CalibrationOverlayFrame(
        image_rgb=image,
        overlay_rgb=blended,
        points_scan=points[ids].copy(),
        projected_uv=projection.uv[ids].copy(),
        visible_point_ids=ids.copy(),
    )


def calibration_overlay(
    scan: ScanRecord,
    profile: CameraProfile,
    shot_index: int,
    *,
    opacity: float = 0.75,
    point_radius: int = 1,
    max_points: int = 50_000,
    source_path: str | Path | None = None,
    azimuth_scale: float = 1.0,
) -> np.ndarray:
    """Return the RGB overlay without point-selection metadata."""

    return calibration_overlay_frame(
        scan,
        profile,
        shot_index,
        opacity=opacity,
        point_radius=point_radius,
        max_points=max_points,
        source_path=source_path,
        azimuth_scale=azimuth_scale,
    ).overlay_rgb
