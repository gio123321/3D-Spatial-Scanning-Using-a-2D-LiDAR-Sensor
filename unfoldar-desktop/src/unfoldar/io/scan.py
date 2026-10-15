from __future__ import annotations

import json
import re
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable

import numpy as np
from plyfile import PlyData, PlyElement

from ..models import ScanRecord, SessionRecord, ShotRecord


REQUIRED_VERTEX_PROPERTIES = {"x", "y", "z", "intensity", "theta_deg"}


class ScanValidationError(ValueError):
    pass


@dataclass(slots=True)
class PlyCloud:
    vertex: np.ndarray
    comments: list[str]

    @property
    def property_names(self) -> tuple[str, ...]:
        return tuple(self.vertex.dtype.names or ())

    @property
    def xyz(self) -> np.ndarray:
        return np.column_stack((self.vertex["x"], self.vertex["y"], self.vertex["z"])).astype(
            np.float64, copy=False
        )


def _slug(value: str) -> str:
    value = re.sub(r"[^a-zA-Z0-9._-]+", "-", value.strip()).strip("-.")
    return value or "scan"


def load_cloud(path: str | Path, *, require_raw: bool = True) -> PlyCloud:
    path = Path(path)
    try:
        ply = PlyData.read(str(path))
        vertex = ply["vertex"].data.copy()
    except Exception as exc:
        raise ScanValidationError(f"Cannot read PLY {path}: {exc}") from exc
    names = set(vertex.dtype.names or ())
    required = REQUIRED_VERTEX_PROPERTIES if require_raw else {"x", "y", "z"}
    missing = sorted(required - names)
    if missing:
        suffix = "; this looks like a derived/colorized cloud, not a raw scan" if require_raw else ""
        raise ScanValidationError(f"PLY is missing vertex properties {missing}{suffix}")
    xyz = np.column_stack((vertex["x"], vertex["y"], vertex["z"]))
    if len(vertex) == 0 or not np.isfinite(xyz).all():
        raise ScanValidationError("PLY contains no points or non-finite XYZ values")
    return PlyCloud(vertex=vertex, comments=list(ply.comments))


def load_session(path: str | Path) -> SessionRecord:
    path = Path(path)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ScanValidationError(f"Cannot read session JSON {path}: {exc}") from exc
    if payload.get("version") != 2:
        raise ScanValidationError(f"Unsupported session version {payload.get('version')!r}; expected 2")
    camera = payload.get("camera") or {}
    if not isinstance(camera.get("width"), int) or not isinstance(camera.get("height"), int):
        raise ScanValidationError("session.camera.width/height must be integers")
    raw_shots = payload.get("shots")
    if not isinstance(raw_shots, list):
        raise ScanValidationError("session.shots must be a list (empty for geometry-only scans)")
    shots: list[ShotRecord] = []
    seen: set[int] = set()
    for item in raw_shots:
        try:
            shot = ShotRecord(
                index=int(item["index"]),
                file=str(item["file"]),
                cmd_az=float(item["cmd_az"]),
                cmd_tilt=float(item["cmd_tilt"]),
                usable=bool(item.get("usable", True)),
                note=str(item.get("note", "")),
                meas_az=float(item["meas_az"]) if item.get("meas_az") is not None else None,
                meas_tilt=float(item["meas_tilt"]) if item.get("meas_tilt") is not None else None,
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ScanValidationError(f"Invalid shot record: {item!r}") from exc
        if shot.index in seen:
            raise ScanValidationError(f"Duplicate shot index {shot.index}")
        if Path(shot.file).name != shot.file:
            raise ScanValidationError(f"Shot path must be a filename: {shot.file!r}")
        seen.add(shot.index)
        shots.append(shot)
    return SessionRecord(
        version=2,
        mode=str(payload.get("mode", "")),
        created=str(payload.get("created", "")),
        name=str(payload.get("name", path.parent.name)),
        camera_width=camera["width"],
        camera_height=camera["height"],
        shots=sorted(shots, key=lambda shot: shot.index),
        transfer=dict(payload.get("transfer") or {}),
        image_pose_offset_frames=(
            int(camera["pose_index_offset"])
            if camera.get("pose_index_offset") is not None
            else None
        ),
        camera_pose_timing=str(camera.get("pose_timing", "")),
        capture=dict(payload.get("capture") or {}),
        firmware_version=str(payload.get("firmware_version", "")),
    )


def image_pose_pairs(
    scan: ScanRecord,
    profile_offset: int,
    azimuth_scale: float = 1.0,
) -> list[tuple[ShotRecord, ShotRecord]]:
    """Pair each stored JPEG record with the pose at which it was exposed.

    Session timing takes precedence over the camera profile's fallback offset.
    """
    offset = effective_pose_offset(scan, profile_offset)
    poses = {shot.index: shot for shot in scan.session.shots}
    pairs: list[tuple[ShotRecord, ShotRecord]] = []
    for image_shot in scan.session.shots:
        pose_shot = poses.get(image_shot.index + offset)
        if image_shot.usable and image_shot.index in scan.image_paths and pose_shot is not None:
            if azimuth_scale != 1.0:
                pose_shot = replace(pose_shot, cmd_az=pose_shot.cmd_az * float(azimuth_scale))
            pairs.append((image_shot, pose_shot))
    return pairs


def effective_pose_offset(scan: ScanRecord, profile_offset: int) -> int:
    """Return the session's image-to-pose offset, falling back to the profile."""
    offset=scan.session.image_pose_offset_frames
    return int(profile_offset) if offset is None else int(offset)


def import_scan_folder(folder: str | Path) -> ScanRecord:
    folder = Path(folder).expanduser().resolve()
    if not folder.is_dir():
        raise ScanValidationError(f"Not a directory: {folder}")
    cloud_path = folder / "cloud.ply"
    session_path = folder / "session.json"
    if not cloud_path.is_file() or not session_path.is_file():
        raise ScanValidationError("Scan folder must contain cloud.ply and session.json")
    cloud = load_cloud(cloud_path, require_raw=True)
    session = load_session(session_path)
    image_paths: dict[int, Path] = {}
    warnings: list[str] = []
    for shot in session.shots:
        image_path = folder / shot.file
        if shot.usable and not image_path.is_file():
            raise ScanValidationError(f"Usable shot {shot.index} is missing {shot.file}")
        if shot.usable and image_path.is_file():
            image_paths[shot.index] = image_path
    if not any(shot.usable and shot.index in image_paths for shot in session.shots):
        warnings.append("Geometry-only scan: colorization and texturing require usable photographs")
    transfer_received = session.transfer.get("received")
    if transfer_received is not None and int(transfer_received) != len(image_paths):
        warnings.append(f"session transfer.received={transfer_received}, found {len(image_paths)} JPEGs")
    return ScanRecord(
        scan_id=_slug(folder.name),
        source_folder=folder,
        session=session,
        cloud_path=cloud_path,
        image_paths=image_paths,
        point_count=len(cloud.vertex),
        vertex_properties=cloud.property_names,
        validation_warnings=warnings,
    )


def discover_scans(root: str | Path) -> tuple[list[ScanRecord], dict[Path, str]]:
    root = Path(root).expanduser().resolve()
    candidates: Iterable[Path]
    if (root / "cloud.ply").is_file():
        candidates = [root]
    else:
        candidates = sorted((p for p in root.iterdir() if p.is_dir()), key=lambda p: p.name.casefold())
    accepted: list[ScanRecord] = []
    rejected: dict[Path, str] = {}
    for candidate in candidates:
        try:
            accepted.append(import_scan_folder(candidate))
        except ScanValidationError as exc:
            rejected[candidate] = str(exc)
    counts: dict[str, int] = {}
    for scan in accepted:
        base = scan.scan_id
        counts[base] = counts.get(base, 0) + 1
        if counts[base] > 1:
            scan.scan_id = f"{base}-{counts[base]}"
    return accepted, rejected


def write_colored_cloud(
    source: PlyCloud,
    path: str | Path,
    rgb: np.ndarray,
    colored: np.ndarray,
    normals: np.ndarray | None = None,
) -> Path:
    path = Path(path)
    rgb = np.asarray(rgb, dtype=np.uint8)
    colored = np.asarray(colored, dtype=np.uint8)
    if rgb.shape != (len(source.vertex), 3) or colored.shape != (len(source.vertex),):
        raise ValueError("RGB/mask shape does not match source cloud")
    skip_names = {"red", "green", "blue", "colored"}
    if normals is not None:
        normals = np.asarray(normals, dtype=np.float32)
        if normals.shape != (len(source.vertex), 3):
            raise ValueError("Normals shape does not match source cloud")
        skip_names |= {"nx", "ny", "nz"}
    keep_names = [name for name in source.property_names if name not in skip_names]
    dtype = [(name, source.vertex.dtype.fields[name][0]) for name in keep_names]
    dtype += [("red", "u1"), ("green", "u1"), ("blue", "u1"), ("colored", "u1")]
    if normals is not None:
        dtype += [("nx", "f4"), ("ny", "f4"), ("nz", "f4")]
    output = np.empty(len(source.vertex), dtype=dtype)
    for name in keep_names:
        output[name] = source.vertex[name]
    output["red"], output["green"], output["blue"] = rgb.T
    output["colored"] = colored
    if normals is not None:
        output["nx"], output["ny"], output["nz"] = normals.T
    path.parent.mkdir(parents=True, exist_ok=True)
    PlyData([PlyElement.describe(output, "vertex")], text=False, comments=source.comments).write(str(path))
    return path


def write_leveled_cloud(
    source: PlyCloud,
    path: str | Path,
    corrected_xyz: np.ndarray,
    *,
    half_pass: np.ndarray | None = None,
) -> Path:
    """Write corrected XYZ coordinates while preserving all other vertex properties."""
    path = Path(path)
    corrected_xyz = np.asarray(corrected_xyz, dtype=np.float64)
    if corrected_xyz.shape != (len(source.vertex), 3):
        raise ValueError("Corrected XYZ shape does not match source cloud")
    skip_names = {"x", "y", "z"}
    keep_names = [name for name in source.property_names if name not in skip_names]
    xyz_dtype = source.vertex.dtype.fields["x"][0]
    dtype = [("x", xyz_dtype), ("y", xyz_dtype), ("z", xyz_dtype)]
    dtype += [(name, source.vertex.dtype.fields[name][0]) for name in keep_names]
    if half_pass is not None and "half_pass" not in keep_names:
        half_pass = np.asarray(half_pass, dtype=np.uint8)
        if half_pass.shape != (len(source.vertex),):
            raise ValueError("Half-pass labels do not match source cloud")
        dtype.append(("half_pass", "u1"))
    output = np.empty(len(source.vertex), dtype=dtype)
    output["x"], output["y"], output["z"] = corrected_xyz.T
    for name in keep_names:
        output[name] = source.vertex[name]
    if half_pass is not None:
        output["half_pass"] = half_pass
    path.parent.mkdir(parents=True, exist_ok=True)
    PlyData([PlyElement.describe(output, "vertex")], text=False, comments=source.comments).write(str(path))
    return path
