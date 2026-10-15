from __future__ import annotations

import json
import os
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

import numpy as np
import open3d as o3d
from plyfile import PlyData, PlyElement

from .calibration.camera import map_lidar_points
from .colorization import estimate_scan_normals
from .io.scan import load_cloud
from .models import CameraProfile, ScanRecord
from .surface_fusion import collapse_supported_layers, cross_scan_surface_metrics, refine_alignment, source_balanced_voxels, surface_reliability


@dataclass(slots=True)
class FusionSettings:
    voxel_size_m: float = 0.01
    statistical_neighbors: int = 24
    statistical_std_ratio: float = 2.5
    radius_m: float = 0.04
    radius_min_neighbors: int = 2
    apply_statistical_filter: bool = True
    apply_radius_filter: bool = False
    # Architectural mode projects supported samples onto fitted planes.
    # This regularizes planar surfaces but can also flatten genuine detail.
    flatten_planes: bool = True
    plane_distance_threshold_m: float = 0.02
    plane_min_fraction: float = 0.01
    plane_max_count: int = 6
    # Optional local normal-direction smoothing for the non-flattening branch.
    denoise_points: bool = True
    denoise_neighborhood_m: float = 0.03
    denoise_strength: float = 0.6
    cleanup_mode: str = "balanced"
    refine_alignment: bool = True
    balanced_layer_separation_m: float = 0.03
    architectural_layer_separation_m: float = 0.08
    balanced_normal_angle_deg: float = 10.0
    architectural_normal_angle_deg: float = 15.0
    range_weight_scale_m: float = 3.0
    minimum_range_weight: float = 0.2


@dataclass(slots=True)
class FusionResult:
    output_path: Path
    input_points: int
    output_points: int
    report_path: Path
    transforms: dict[str, np.ndarray] | None = None
    provenance_path: Path | None = None
    before_preview_path: Path | None = None


def _denoise_points(points: np.ndarray, normals: np.ndarray, neighborhood_m: float, strength: float) -> np.ndarray:
    """Move points toward local centroids only along their estimated normals.

    Centroids average unshifted and half-cell-shifted grids. Strength is clamped
    to [0, 1]. Tangential coordinates are not directly averaged, but estimated
    normals and neighbourhoods can still change real detail or curved surfaces.
    """
    if neighborhood_m <= 0 or strength <= 0 or len(points) < 4:
        return points

    def local_centroids(offset: np.ndarray) -> np.ndarray:
        shifted = points + offset
        keys = np.floor(shifted / neighborhood_m).astype(np.int64)
        _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        groups = len(counts)
        sums = np.column_stack([np.bincount(inverse, points[:, axis], minlength=groups) for axis in range(3)])
        return (sums / counts[:, None])[inverse]

    # Average two half-cell-offset grids to reduce centroid discontinuities
    # at voxel boundaries.
    half_cell = np.full(3, neighborhood_m / 2.0)
    local_centroid = 0.5 * (local_centroids(np.zeros(3)) + local_centroids(half_cell))
    perpendicular = np.sum((points - local_centroid) * normals, axis=1, keepdims=True) * normals
    return points - min(max(strength, 0.0), 1.0) * perpendicular


def _flatten_planes(
    points: np.ndarray, normals: np.ndarray, distance_threshold: float, min_points: int, max_planes: int,
) -> tuple[np.ndarray, np.ndarray, int]:
    """RANSAC-fit dominant planes and project their inliers onto those planes.

    The caller restricts architectural flattening to multi-scan-supported samples.
    Plane assumptions can remove genuine non-planarity; normals retain their side.
    """
    if max_planes <= 0 or len(points) < min_points:
        return points, normals, 0
    flattened_points = points.copy(); flattened_normals = normals.copy()
    working = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    remaining = np.arange(len(points))
    planes_found = 0
    for _ in range(max_planes):
        if len(remaining) < min_points:
            break
        model, inlier_local = working.segment_plane(distance_threshold, ransac_n=3, num_iterations=1000)
        if len(inlier_local) < min_points:
            break
        normal = np.asarray(model[:3]); scale = np.linalg.norm(normal)
        if scale < 1e-9:
            break
        normal = normal / scale; offset = model[3] / scale
        global_idx = remaining[inlier_local]
        pts = flattened_points[global_idx]
        signed_distance = pts @ normal + offset
        flattened_points[global_idx] = pts - signed_distance[:, None] * normal
        # Keep the original normal orientation when replacing it with a plane normal.
        sign = np.sign((flattened_normals[global_idx] * normal).sum(axis=1)); sign[sign == 0] = 1.0
        flattened_normals[global_idx] = normal[None, :] * sign[:, None]
        planes_found += 1
        keep_mask = np.ones(len(remaining), dtype=bool); keep_mask[inlier_local] = False
        remaining = remaining[keep_mask]
        working = working.select_by_index(inlier_local, invert=True)
    return flattened_points, flattened_normals, planes_found


def _atomic_ply(path: Path,vertex: np.ndarray,comments: list[str]) -> None:
    path.parent.mkdir(parents=True,exist_ok=True)
    handle,temp_name=tempfile.mkstemp(prefix=path.stem+".",suffix=".tmp.ply",dir=path.parent); os.close(handle)
    try:
        PlyData([PlyElement.describe(vertex,"vertex")],text=False,comments=comments).write(temp_name)
        os.replace(temp_name,path)
    finally:
        if os.path.exists(temp_name):os.unlink(temp_name)


def _atomic_json(path: Path,payload: dict) -> None:
    path.parent.mkdir(parents=True,exist_ok=True)
    handle,temp_name=tempfile.mkstemp(prefix=path.stem+".",suffix=".tmp",dir=path.parent)
    try:
        with os.fdopen(handle,"w",encoding="utf-8",newline="\n") as stream:
            json.dump(payload,stream,indent=2);stream.write("\n");stream.flush();os.fsync(stream.fileno())
        os.replace(temp_name,path)
    finally:
        if os.path.exists(temp_name):os.unlink(temp_name)


def _write_before_preview(path: Path,points: np.ndarray,source_ids: np.ndarray) -> None:
    keys=np.floor(points/.02).astype(np.int64); _,idx=np.unique(keys,axis=0,return_index=True); idx=np.sort(idx)
    palette=np.array([[53,208,186],[255,92,119],[255,202,58],[89,126,247],[177,108,234],[255,145,77],[73,190,94],[237,99,190],[120,144,156]],np.uint8)
    vertex=np.empty(len(idx),dtype=[("x","f4"),("y","f4"),("z","f4"),("red","u1"),("green","u1"),("blue","u1"),("source_scan","u2")])
    vertex["x"],vertex["y"],vertex["z"]=points[idx].T; vertex["source_scan"]=source_ids[idx]
    vertex["red"],vertex["green"],vertex["blue"]=palette[source_ids[idx]%len(palette)].T
    _atomic_ply(path,vertex,["Registered inputs before surface fusion; color identifies source scan"])


def _kept_provenance(offsets: np.ndarray,point_ids: np.ndarray,source_ids: np.ndarray,keep: np.ndarray):
    output_offsets=[0]; output_points=[]; output_sources=[]
    for group in keep:
        start,stop=int(offsets[group]),int(offsets[group+1]); output_points.append(point_ids[start:stop]);output_sources.append(source_ids[start:stop]);output_offsets.append(output_offsets[-1]+stop-start)
    return np.asarray(output_offsets,np.int64),np.concatenate(output_points) if output_points else np.empty(0,np.int64),np.concatenate(output_sources) if output_sources else np.empty(0,np.int32)


def fuse_scans(
    scans: list[ScanRecord], transforms: dict[str,np.ndarray], profile: CameraProfile,
    output_path: str|Path, colored_paths: dict[str,Path]|None=None,
    settings: FusionSettings|None=None, leveled_paths: dict[str,Path]|None=None,
    progress: Callable[[float,str],None]|None=None,
    cancelled: Callable[[],bool]|None=None,
    reference_scan_id: str|None=None,
) -> FusionResult:
    from .publication import staged_call, protect_raw_output
    protect_raw_output(output_path,scans)
    output_path=Path(output_path)
    return staged_call(output_path.parent,lambda staging:_fuse_scans(
        scans,transforms,profile,staging/output_path.name,colored_paths,settings,
        leveled_paths,progress,cancelled,reference_scan_id),cancelled or (lambda:False))


def _fuse_scans(scans,transforms,profile,output_path,colored_paths,settings,leveled_paths,progress,cancelled,reference_scan_id):
    settings=settings or FusionSettings(); colored_paths=colored_paths or {}; leveled_paths=leveled_paths or {}
    progress=progress or (lambda _v,_m:None); cancelled=cancelled or (lambda:False)
    if settings.cleanup_mode not in {"balanced","architectural"}:raise ValueError("cleanup_mode must be balanced or architectural")
    local_points={}; scan_payload={}
    progress(.02,"Loading per-scan geometry")
    for source_index,scan in enumerate(scans):
        if cancelled():raise InterruptedError("Fusion cancelled")
        source_path=colored_paths.get(scan.scan_id) or leveled_paths.get(scan.scan_id) or scan.cloud_path
        raw=load_cloud(source_path,require_raw=source_path==scan.cloud_path); points=map_lidar_points(raw.xyz,profile); names=set(raw.property_names)
        if {"nx","ny","nz"}.issubset(names):normals=np.column_stack((raw.vertex["nx"],raw.vertex["ny"],raw.vertex["nz"])).astype(np.float64)
        else:normals=estimate_scan_normals(points,.08,40)
        colors=np.column_stack((raw.vertex["red"],raw.vertex["green"],raw.vertex["blue"])).astype(np.float64) if {"red","green","blue"}.issubset(names) else np.zeros((len(points),3))
        valid=raw.vertex["colored"].astype(bool) if "colored" in names else np.zeros(len(points),bool)
        intensity=raw.vertex["intensity"].astype(np.float64) if "intensity" in names else np.zeros(len(points))
        local_points[scan.scan_id]=points; scan_payload[scan.scan_id]=(normals,colors,valid,intensity,source_index)
    initial={scan.scan_id:np.asarray(transforms.get(scan.scan_id,np.eye(4) if len(scans)==1 else None),dtype=np.float64) for scan in scans}
    if any(value.shape!= (4,4) for value in initial.values()):raise ValueError("Missing optimized transform for one or more scans")
    reference=reference_scan_id or scans[0].scan_id
    alignment=refine_alignment(local_points,initial,reference,settings.refine_alignment,
        lambda value,message:progress(.05+.25*value,message),cancelled)
    poses=alignment.transforms
    all_points=[];all_normals=[];all_colors=[];all_valid=[];all_intensity=[];all_sources=[];all_point_ids=[];all_ranges=[];reliable=[]
    progress(.32,"Preparing source-aware surface samples")
    for scan in scans:
        normals,colors,valid,intensity,source_index=scan_payload[scan.scan_id]; pose=poses[scan.scan_id]
        points=_transform_points(local_points[scan.scan_id],pose); world_normals=normals@pose[:3,:3].T
        all_points.append(points);all_normals.append(world_normals);all_colors.append(colors);all_valid.append(valid);all_intensity.append(intensity)
        all_sources.append(np.full(len(points),source_index,np.int32));all_point_ids.append(np.arange(len(points),dtype=np.int64))
        all_ranges.append(np.linalg.norm(local_points[scan.scan_id],axis=1))
        reliable.append(surface_reliability(points,world_normals))
    points=np.concatenate(all_points);normals=np.concatenate(all_normals);colors=np.concatenate(all_colors);valid=np.concatenate(all_valid)
    intensity=np.concatenate(all_intensity);source_ids=np.concatenate(all_sources);point_ids=np.concatenate(all_point_ids)
    ranges=np.concatenate(all_ranges);reliable_mask=np.concatenate(reliable);input_count=len(points)
    output=Path(output_path);preview=output.with_name("registered_before_fusion.ply")
    architectural=settings.cleanup_mode=="architectural"
    separation=settings.architectural_layer_separation_m if architectural else settings.balanced_layer_separation_m
    angle=settings.architectural_normal_angle_deg if architectural else settings.balanced_normal_angle_deg
    progress(.48,f"Collapsing supported surface layers ({settings.cleanup_mode})")
    layer_before=cross_scan_surface_metrics(points,normals,source_ids,max(.10,separation*1.25),angle)
    points,cleanup=collapse_supported_layers(points,normals,source_ids,reliable_mask,separation,angle,architectural,cancelled,
        ranges,settings.range_weight_scale_m,settings.minimum_range_weight,
        lambda value,message:progress(.48+.22*value,message))
    layer_after=cross_scan_surface_metrics(points,normals,source_ids,max(.10,separation*1.25),angle)
    cleanup["cross_scan_before"]=layer_before;cleanup["cross_scan_after"]=layer_after
    progress(.72,"Consolidating source-balanced observations")
    points,normals,colors,valid,intensity,observations,scan_support,offsets,prov_points,prov_sources=source_balanced_voxels(
        points,normals,colors,valid,intensity,source_ids,point_ids,settings.voxel_size_m,reliable_mask,ranges,
        settings.range_weight_scale_m,settings.minimum_range_weight)
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points));keep=np.arange(len(points))
    if settings.apply_statistical_filter and len(points)>settings.statistical_neighbors:
        _,idx=cloud.remove_statistical_outlier(settings.statistical_neighbors,settings.statistical_std_ratio);keep=keep[np.asarray(idx)]
    if settings.apply_radius_filter and len(keep):
        filtered=cloud.select_by_index(keep.tolist());_,idx=filtered.remove_radius_outlier(settings.radius_min_neighbors,settings.radius_m);keep=keep[np.asarray(idx)]
    points=points[keep];normals=normals[keep];colors=colors[keep];valid=valid[keep];intensity=intensity[keep];observations=observations[keep];scan_support=scan_support[keep]
    planes_found=0
    if architectural and settings.flatten_planes:
        # Project only samples already supported by multiple scans.
        supported=scan_support>=2
        min_points=max(500,int(np.sum(supported)*settings.plane_min_fraction))
        flat,flat_normals,planes_found=_flatten_planes(points[supported],normals[supported],
            settings.plane_distance_threshold_m,min_points,settings.plane_max_count)
        points[supported]=flat;normals[supported]=flat_normals
    elif settings.denoise_points:
        points=_denoise_points(points,normals,settings.denoise_neighborhood_m,settings.denoise_strength)
    provenance_offsets,provenance_points,provenance_sources=_kept_provenance(offsets,prov_points,prov_sources,keep)
    vertex=np.empty(len(points),dtype=[("x","f4"),("y","f4"),("z","f4"),("nx","f4"),("ny","f4"),("nz","f4"),("intensity","u1"),("red","u1"),("green","u1"),("blue","u1"),("colored","u1"),("observations","u4"),("supporting_scans","u2")])
    vertex["x"],vertex["y"],vertex["z"]=points.T;vertex["nx"],vertex["ny"],vertex["nz"]=normals.T
    vertex["intensity"]=np.clip(intensity,0,255).astype(np.uint8);vertex["red"],vertex["green"],vertex["blue"]=np.clip(colors,0,255).astype(np.uint8).T
    vertex["colored"]=valid.astype(np.uint8);vertex["observations"]=observations;vertex["supporting_scans"]=scan_support
    provenance=output.with_suffix(".provenance.npz");report=output.with_suffix(".report.json")
    payload={"model_version":"source-aware-fusion-v1","input_points":input_count,"output_points":len(points),
             "cleanup_mode":settings.cleanup_mode,"cleanup":cleanup,"planes_flattened":planes_found,
             "alignment":{"accepted":alignment.accepted,"reason":alignment.reason,"edges_used":alignment.edges_used,
             "before":alignment.before,"after":alignment.after,"max_translation_m":alignment.max_translation_m,
             "max_rotation_deg":alignment.max_rotation_deg,
             "substantial_unresolved_misalignment":bool((alignment.before.get("median_mm") or 0)>20 or (alignment.before.get("p90_mm") or 0)>60)},
             "retained_fraction":len(points)/max(input_count,1),"removed_points":input_count-len(points),
             "multi_scan_output_points":int(np.sum(scan_support>=2)),
             "single_scan_details_retained":int(np.sum(scan_support==1)),
             "transforms":{key:value.tolist() for key,value in poses.items()},
             "scanner_positions_m":{key:value[:3,3].tolist() for key,value in poses.items()},"settings":asdict(settings)}
    if cancelled():raise InterruptedError("Fusion cancelled")
    progress(.92,"Staging fused cloud and provenance")
    output.parent.mkdir(parents=True,exist_ok=True)
    staged=[]
    try:
        for destination,suffix in ((output,".ply"),(preview,".ply"),(provenance,".npz"),(report,".json")):
            handle,name=tempfile.mkstemp(prefix=destination.stem+".stage.",suffix=suffix,dir=output.parent)
            os.close(handle);os.unlink(name);staged.append((Path(name),destination))
        staged_output,staged_preview,staged_provenance,staged_report=(item[0] for item in staged)
        _atomic_ply(staged_output,vertex,["Generated by UnfoLDAR Desktop"])
        _write_before_preview(staged_preview,np.concatenate(all_points),source_ids)
        with staged_provenance.open("wb") as stream:
            np.savez_compressed(stream,offsets=provenance_offsets,scan_index=provenance_sources,
                                point_index=provenance_points,scan_ids=np.asarray([scan.scan_id for scan in scans]))
            stream.flush();os.fsync(stream.fileno())
        _atomic_json(staged_report,payload)
        if cancelled():raise InterruptedError("Fusion cancelled")
        # Publish the primary cloud after all companion outputs exist.
        for staged_path,destination in staged[1:]:os.replace(staged_path,destination)
        os.replace(staged[0][0],output)
    finally:
        for staged_path,_destination in staged:
            if staged_path.exists():staged_path.unlink()
    progress(1.,"Surface-aware fusion complete")
    return FusionResult(output,input_count,len(points),report,poses,provenance,preview)


def _transform_points(points: np.ndarray,pose: np.ndarray) -> np.ndarray:
    return points@pose[:3,:3].T+pose[:3,3]
