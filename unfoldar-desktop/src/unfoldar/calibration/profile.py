from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path

from ..models import CameraProfile
from .camera import validate_rotation_matrix


def validate_profile(profile: CameraProfile) -> None:
    if profile.schema_version != 3:
        raise ValueError(f"Unsupported camera profile schema {profile.schema_version}")
    if profile.image_width <= 0 or profile.image_height <= 0 or profile.fx <= 0 or profile.fy <= 0:
        raise ValueError("Image dimensions and focal lengths must be positive")
    validate_rotation_matrix(profile.axis_map, name="axis_map")
    validate_rotation_matrix(profile.camera_basis, name="camera_basis")
    if profile.rotation_order not in {"pan_tilt", "tilt_pan"}:
        raise ValueError("rotation_order must be pan_tilt or tilt_pan")
    expected = 4 if profile.lens_model.value == "opencv_fisheye" else (4, 5, 8, 12, 14)
    if isinstance(expected, int) and len(profile.distortion) != expected:
        raise ValueError("Fisheye distortion requires exactly four coefficients")
    if isinstance(expected, tuple) and len(profile.distortion) not in expected:
        raise ValueError("Brown distortion requires 4, 5, 8, 12, or 14 coefficients")


def save_profile(profile: CameraProfile, path: str | Path) -> Path:
    validate_profile(profile)
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(profile.to_dict(), indent=2, sort_keys=True) + "\n"
    handle, temp_name = tempfile.mkstemp(prefix=destination.name, suffix=".tmp", dir=destination.parent)
    try:
        with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, destination)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)
    return destination


def load_profile(path: str | Path) -> CameraProfile:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    profile = CameraProfile.from_dict(data)
    validate_profile(profile)
    return profile


def rescale_profile(profile: CameraProfile, width: int, height: int) -> CameraProfile:
    if (width,height)==(profile.image_width,profile.image_height):
        return CameraProfile.from_dict(profile.to_dict())
    old_ratio=profile.image_width/profile.image_height; new_ratio=width/height
    if abs(old_ratio-new_ratio)>1e-6:
        raise ValueError(f"Cannot rescale calibration from {profile.image_width}x{profile.image_height} to different-aspect {width}x{height}")
    scaled=CameraProfile.from_dict(profile.to_dict()); sx=width/profile.image_width; sy=height/profile.image_height
    scaled.fx*=sx; scaled.fy*=sy; scaled.cx=(scaled.cx+.5)*sx-.5; scaled.cy=(scaled.cy+.5)*sy-.5
    scaled.image_width=width; scaled.image_height=height
    return scaled
