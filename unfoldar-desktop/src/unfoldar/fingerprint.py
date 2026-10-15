from __future__ import annotations

import hashlib
import json
from typing import Any

import numpy as np

from .models import CameraProfile, ScanRecord


def compute_stage_fingerprint(
    scans: list[ScanRecord], stage: str, profile: CameraProfile,
    settings: dict[str, Any], transforms: dict[str, np.ndarray],
) -> str:
    """Hash stage inputs for staleness detection, including missing-file markers."""
    dependencies = {
        "leveling": ("leveling",),
        "calibration": ("leveling",),
        "colorization": ("leveling", "colorization"),
        "registration_pairs": ("leveling", "colorization", "registration"),
        "registration": ("leveling", "colorization", "registration"),
        "fusion": ("leveling", "colorization", "registration", "fusion", "room_mode"),
        "mesh": ("leveling", "colorization", "registration", "fusion", "room_mode", "mesh"),
        "texture": ("leveling", "colorization", "registration", "fusion", "room_mode", "mesh", "texture"),
    }.get(stage, (stage,))
    photos = stage != "leveling"
    sources=[]
    for scan in scans:
        files=[scan.cloud_path,scan.source_folder/"session.json"]
        if photos: files.extend(scan.image_paths.values())
        entries=[]
        for path in files:
            try:
                digest = hashlib.sha256()
                with path.open("rb") as stream:
                    for block in iter(lambda: stream.read(1024 * 1024), b""): digest.update(block)
                entries.append((path.name, digest.hexdigest()))
            except OSError:
                entries.append((path.name, "missing"))
        sources.append({"scan_id":scan.scan_id,"files":entries})
    payload={"version": 2, "stage":stage,"sources":sources,
             "profile":profile.to_dict() if photos else None,
             "settings":{key: settings.get(key) for key in dependencies},
             "transforms":{key:value.tolist() for key,value in transforms.items()}
             if stage in {"fusion", "mesh", "texture", "registration"} else {}}
    return hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode()).hexdigest()
