"""Stage complete output sets, with rollback if publication fails."""
from contextlib import contextmanager
from dataclasses import fields, is_dataclass, replace
from pathlib import Path
import os
import shutil
import tempfile


def relocate_result(value, source: Path, destination: Path):
    if isinstance(value, Path):
        try: return destination / value.relative_to(source)
        except ValueError: return value
    if is_dataclass(value):
        return replace(value, **{field.name: relocate_result(getattr(value, field.name), source, destination) for field in fields(value)})
    if isinstance(value, list): return [relocate_result(item, source, destination) for item in value]
    if isinstance(value, tuple): return tuple(relocate_result(item, source, destination) for item in value)
    return value


def protect_raw_output(destination,scans):
    destination=Path(destination).resolve()
    for scan in scans:
        cloud=Path(scan.cloud_path).resolve()
        raw={cloud,cloud.parent/"session.json",*[Path(path).resolve() for path in getattr(scan,"image_paths",{}).values()]}
        if destination in raw:raise ValueError("Processing cannot overwrite raw scan files")


@contextmanager
def staged_output(destination: Path, cancelled=lambda: False):
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".unfoldar-stage-", dir=destination.parent))
    backup = staging / ".rollback"
    installed = []
    moved = []
    try:
        yield staging
        if cancelled(): raise InterruptedError("Cancelled before publishing outputs")
        files = [path for path in staging.rglob("*") if path.is_file()]
        for path in files:
            relative = path.relative_to(staging)
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            if target.exists():
                previous = backup / relative
                previous.parent.mkdir(parents=True, exist_ok=True)
                os.replace(target, previous)
                moved.append((previous, target))
            os.replace(path, target)
            installed.append(target)
    except BaseException:
        for target in reversed(installed): target.unlink(missing_ok=True)
        for previous, target in reversed(moved): os.replace(previous, target)
        raise
    finally:
        shutil.rmtree(staging, ignore_errors=True)


def staged_call(destination, operation, cancelled=lambda: False):
    destination = Path(destination)
    with staged_output(destination, cancelled) as staging:
        result = operation(staging)
    return relocate_result(result, staging, destination)


def export_outputs(source, destination, raw_folders, *, progress, cancelled):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    forbidden = [source, *[Path(path).resolve() for path in raw_folders]]
    if any(destination == path or path in destination.parents or destination in path.parents for path in forbidden):
        raise ValueError("Choose an export location outside raw captures and project outputs")
    # A new child avoids overwriting any unrelated data in the selected folder.
    target = destination / "UnfoLDAR-results"
    if target.exists(): raise FileExistsError(f"Choose another folder: {target} already exists")
    files = [path for path in source.rglob("*") if path.is_file() and not any(part.startswith(".unfoldar-stage-") for part in path.parts)]
    with staged_output(target, cancelled) as staging:
        for index, path in enumerate(files):
            if cancelled(): raise InterruptedError("Export cancelled")
            output = staging / path.relative_to(source)
            output.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, output)
            progress((index + 1) / max(len(files), 1), f"Exporting {path.name}")
    return target


def export_mesh_asset(source,destination,raw_folders,*,progress,cancelled):
    source,destination=Path(source).resolve(),Path(destination).resolve()
    if any(destination==Path(folder).resolve() or Path(folder).resolve() in destination.parents for folder in raw_folders):
        raise ValueError("Mesh export cannot overwrite raw capture files")
    if destination==source:raise ValueError("Choose a new mesh export path")
    files={destination.name:source}
    if source.suffix.lower()==".obj":
        for line in source.read_text(encoding="utf-8").splitlines():
            if not line.startswith("mtllib "):continue
            relative=Path(line[7:].strip()); material=(source.parent/relative).resolve()
            if not material.is_relative_to(source.parent):raise ValueError("Material path escapes the complete mesh set")
            files[relative.as_posix()]=material
            for item in material.read_text(encoding="utf-8").splitlines():
                if item.startswith("map_Kd "):
                    texture_relative=relative.parent/Path(item[7:].strip())
                    texture=(source.parent/texture_relative).resolve()
                    if not texture.is_relative_to(source.parent):raise ValueError("Texture path escapes the complete mesh set")
                    files[texture_relative.as_posix()]=texture
    with staged_output(destination.parent,cancelled) as staging:
        for index,(relative,path) in enumerate(files.items()):
            if cancelled():raise InterruptedError("Mesh export cancelled")
            target=destination.parent/relative
            if target==path:raise ValueError("Export cannot overwrite source assets")
            if relative!=destination.name and target.exists() and target.read_bytes()!=path.read_bytes():
                raise FileExistsError(f"Existing sidecar would be overwritten: {target}; choose an empty export folder")
            output=staging/relative;output.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,output)
            progress((index+1)/len(files),f"Exporting {path.name}")
    return destination
