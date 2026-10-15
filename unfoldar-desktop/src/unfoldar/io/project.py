from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any
from uuid import uuid4

import numpy as np

from ..models import ProjectState, RegistrationEdge, StageStatus


def _path(value: str | None, workspace: Path) -> Path | None:
    if not value: return None
    path = Path(value)
    return (path if path.is_absolute() else workspace / path).resolve()


def _stored_path(value: str | Path | None, workspace: Path | None) -> str | None:
    if not value: return None
    path = Path(value).resolve()
    if workspace:
        try: return path.relative_to(workspace).as_posix()
        except ValueError: pass
    return str(path)


def project_to_dict(project: ProjectState) -> dict[str, Any]:
    workspace = project.project_file.parent.resolve() if project.project_file else None
    for path in project.scan_paths:
        if str(path.resolve()) not in project.scan_ids:
            from .scan import _slug
            legacy=_slug(path.name)
            identity=legacy if legacy in project.transforms and legacy not in project.scan_ids.values() else "scan-"+uuid4().hex[:16]
            project.scan_ids[str(path.resolve())]=identity
    return {
        "schema_version": 2,
        "name": project.name,
        "scan_paths": [_stored_path(p, workspace) for p in project.scan_paths],
        "scans": [{"id": project.scan_ids[str(p.resolve())], "path": _stored_path(p, workspace)} for p in project.scan_paths],
        "reference_scan_id": project.reference_scan_id,
        "calibration_profile_path": _stored_path(project.calibration_profile_path, workspace),
        "transforms": {key: value.tolist() for key, value in project.transforms.items()},
        "edges": [
            {
                "source_id": edge.source_id,
                "target_id": edge.target_id,
                "transformation": edge.transformation.tolist(),
                "information": edge.information.tolist(),
                "method": edge.method,
                "fitness": edge.fitness,
                "inlier_rmse": edge.inlier_rmse,
                "confidence": edge.confidence,
                "accepted": edge.accepted,
                "uncertain": edge.uncertain,
                "manual": edge.manual,
            }
            for edge in project.edges
        ],
        "stage_status": {key: value.value for key, value in project.stage_status.items()},
        "settings": project.settings,
        "artifacts": {key: _stored_path(value, workspace) for key, value in project.artifacts.items()},
        "artifact_fingerprints": project.artifact_fingerprints,
    }


def save_project(project: ProjectState, path: str | Path | None = None) -> Path:
    destination = Path(path or project.project_file or "project.unfoldar.json").expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    project.project_file = destination
    payload = json.dumps(project_to_dict(project), indent=2, sort_keys=True)
    handle, temp_name = tempfile.mkstemp(prefix=destination.name, suffix=".tmp", dir=destination.parent)
    try:
        with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(payload)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, destination)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)
    project.project_file = destination
    return destination


def load_project(path: str | Path) -> ProjectState:
    source = Path(path).expanduser().resolve()
    data = json.loads(source.read_text(encoding="utf-8"))
    if data.get("schema_version") not in (1, 2):
        raise ValueError(f"Unsupported project schema {data.get('schema_version')!r}")
    workspace = source.parent
    scan_paths = [_path(p, workspace) for p in data.get("scan_paths", [])]
    scan_ids = {}
    warnings = []
    if data.get("schema_version") == 2:
        scans = data.get("scans", [])
        ids = [item["id"] for item in scans]
        if len(ids) != len(set(ids)):
            raise ValueError("Project contains duplicate scan identities")
        scan_paths = [_path(item["path"], workspace) for item in scans]
        scan_ids = {str(path): item["id"] for path, item in zip(scan_paths, scans, strict=True)}
    else:
        from .scan import _slug
        for path in scan_paths:
            identity = _slug(path.name)
            if identity in scan_ids.values():
                warnings.append(f"Ambiguous legacy identity {identity}: {path}; registration must be repeated")
                identity += "-" + uuid4().hex[:8]
            scan_ids[str(path)] = identity
    edges = [
        RegistrationEdge(
            source_id=item["source_id"],
            target_id=item["target_id"],
            transformation=np.asarray(item["transformation"], dtype=np.float64),
            information=np.asarray(item["information"], dtype=np.float64),
            method=item["method"],
            fitness=float(item["fitness"]),
            inlier_rmse=float(item["inlier_rmse"]),
            confidence=float(item["confidence"]),
            accepted=bool(item["accepted"]),
            uncertain=bool(item.get("uncertain", True)),
            manual=bool(item.get("manual", False)),
        )
        for item in data.get("edges", [])
    ]
    project = ProjectState(
        schema_version=2,
        name=data.get("name", "UnfoLDAR Project"),
        project_file=source,
        scan_paths=scan_paths,
        scan_ids=scan_ids,
        migration_warnings=warnings,
        reference_scan_id=data.get("reference_scan_id"),
        calibration_profile_path=_path(data.get("calibration_profile_path"), workspace),
        transforms={key: np.asarray(value, dtype=np.float64) for key, value in data.get("transforms", {}).items()},
        edges=edges,
        stage_status={key: StageStatus(value) for key, value in data.get("stage_status", {}).items()},
        settings=dict(data.get("settings", {})),
        artifacts={key: str(_path(value, workspace)) for key, value in data.get("artifacts", {}).items()},
        artifact_fingerprints=dict(data.get("artifact_fingerprints", {})),
    )
    if warnings:
        project.transforms.clear()
        project.edges.clear()
        for stage in ("registration", "fusion", "mesh", "texture"):
            project.stage_status[stage] = StageStatus.STALE
    return project
