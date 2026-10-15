from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import StrEnum
from pathlib import Path
from typing import Any

import numpy as np


class LensModel(StrEnum):
    FISHEYE = "opencv_fisheye"
    BROWN = "opencv_brown"


class StageStatus(StrEnum):
    MISSING = "missing"
    READY = "ready"
    RUNNING = "running"
    COMPLETE = "complete"
    STALE = "stale"
    FAILED = "failed"


@dataclass(slots=True)
class ShotRecord:
    index: int
    file: str
    cmd_az: float
    cmd_tilt: float
    usable: bool = True
    note: str = ""
    meas_az: float | None = None
    meas_tilt: float | None = None


@dataclass(slots=True)
class SessionRecord:
    version: int
    mode: str
    created: str
    name: str
    camera_width: int
    camera_height: int
    shots: list[ShotRecord]
    transfer: dict[str, Any] = field(default_factory=dict)
    # Use CameraProfile timing when the session omits an offset.
    image_pose_offset_frames: int | None = None
    camera_pose_timing: str = ""
    capture: dict[str, Any] = field(default_factory=dict)
    firmware_version: str = ""


@dataclass(slots=True)
class ScanRecord:
    scan_id: str
    source_folder: Path
    session: SessionRecord
    cloud_path: Path
    image_paths: dict[int, Path]
    point_count: int
    vertex_properties: tuple[str, ...]
    transform: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float64))
    colored_cloud_path: Path | None = None
    validation_warnings: list[str] = field(default_factory=list)


@dataclass(slots=True)
class CameraProfile:
    """Reusable hardware calibration.

    Stored XYZ is first mapped through ``axis_map``. Camera vectors use the
    OpenCV convention (+x right, +y down, +z forward). ``camera_basis`` maps
    those vectors into the zero-pan/zero-tilt scan frame.
    """

    schema_version: int = 3
    name: str = "OV2640 wide-angle rig"
    image_width: int = 640
    image_height: int = 480
    lens_model: LensModel = LensModel.FISHEYE
    # Defaults from the rig calibration optimizer.
    fx: float = 438.0
    fy: float = 438.0
    cx: float = 308.0
    cy: float = 308.0
    distortion: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    axis_map: list[list[float]] = field(
        default_factory=lambda: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    )
    camera_basis: list[list[float]] = field(
        # Columns are camera right, down and forward expressed in the PLY
        # scan frame. The mobile converter's empirical LiDAR mounting map
        # makes the camera look along PLY +X at cmd_az=cmd_tilt=0.
        default_factory=lambda: [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    )
    pan_axis: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    tilt_axis: list[float] = field(default_factory=lambda: [0.0, 1.0, 0.0])
    pan_sign: float = -1.0
    # Positive tilt raises the upright camera. At zero pan its physical
    # right-axis is PLY -Y, hence the negative effective rotation axis.
    tilt_sign: float = -1.0
    rotation_order: str = "pan_tilt"
    pan_zero_deg: float = 0.0
    tilt_zero_deg: float = 0.0
    boresight_ypr_deg: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    camera_center_xyz_m: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    unit_scale_to_m: float = 1.0
    # Fallback offset for sessions without image-to-pose timing metadata.
    image_pose_offset_frames: int = 0

    @property
    def intrinsic_matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["lens_model"] = self.lens_model.value
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CameraProfile":
        values = dict(data)
        # Convert the schema-1 default basis while preserving custom mappings.
        source_schema = int(values.get("schema_version", 1))
        if source_schema == 1:
            old_basis = [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
            if np.allclose(values.get("camera_basis", old_basis), old_basis):
                values["camera_basis"] = [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
            values["schema_version"] = 3
        elif source_schema == 2:
            shipped_basis = [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
            if np.allclose(values.get("camera_basis", shipped_basis), shipped_basis) and float(values.get("tilt_sign", 1.0)) == 1.0:
                values["tilt_sign"] = -1.0
            values["schema_version"] = 3
        values["lens_model"] = LensModel(values.get("lens_model", LensModel.FISHEYE))
        return cls(**values)


@dataclass(slots=True)
class RegistrationEdge:
    source_id: str
    target_id: str
    transformation: np.ndarray
    information: np.ndarray
    method: str
    fitness: float
    inlier_rmse: float
    confidence: float
    accepted: bool
    uncertain: bool = True
    manual: bool = False


@dataclass(slots=True)
class ProjectState:
    schema_version: int = 2
    name: str = "UnfoLDAR Project"
    project_file: Path | None = None
    scan_paths: list[Path] = field(default_factory=list)
    scan_ids: dict[str, str] = field(default_factory=dict)
    migration_warnings: list[str] = field(default_factory=list)
    reference_scan_id: str | None = None
    calibration_profile_path: Path | None = None
    transforms: dict[str, np.ndarray] = field(default_factory=dict)
    edges: list[RegistrationEdge] = field(default_factory=list)
    stage_status: dict[str, StageStatus] = field(
        default_factory=lambda: {
            key: StageStatus.MISSING
            for key in ("import", "leveling", "calibration", "colorization", "registration", "fusion", "mesh", "texture")
        }
    )
    settings: dict[str, Any] = field(default_factory=dict)
    artifacts: dict[str, str] = field(default_factory=dict)
    artifact_fingerprints: dict[str, str] = field(default_factory=dict)
