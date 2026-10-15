from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

from .io.scan import PlyCloud, load_cloud, write_leveled_cloud
from .models import ScanRecord

Progress = Callable[[float, str], None]
Cancelled = Callable[[], bool]
MODEL_VERSION = "opposite-pass-physical-v1"
PAN_DIRECTION = -1.0


@dataclass(slots=True)
class LevelingSettings:
    enabled: bool = True
    optimize_theta_affine: bool = True
    target_points: int = 8000
    validation_fraction: float = 0.2
    normal_knn: int = 16
    max_neighbor_radius_m: float = 0.30
    max_correspondence_distance_m: float = 0.20
    final_correspondence_distance_m: float = 0.12
    min_normal_dot: float = 0.75
    max_planarity_ratio: float = 0.30
    huber_delta_m: float = 0.015
    max_outer_iterations: int = 5
    max_function_evals: int = 120
    min_points_required: int = 1000
    min_correspondences: int = 150
    min_overlap: float = 0.05
    min_median_improvement: float = 0.10
    max_accepted_tilt_deg: float = 10.0
    max_sensor_offset_m: float = 0.20
    theta_scale_min: float = 0.90
    theta_scale_max: float = 1.10
    max_condition_number: float = 1e8

    @property
    def knn(self) -> int:
        return self.normal_knn


@dataclass(slots=True)
class LevelingResult:
    scan_id: str
    output_path: Path | None
    report_path: Path | None
    applied: bool
    skipped_reason: str | None
    nx: float = 0.0
    ny: float = 0.0
    theta0_deg: float = 0.0
    tilt_deg: float = 0.0
    tilt_azimuth_deg: float = 0.0
    cost_before_mm: float = 0.0
    cost_after_mm: float = 0.0
    converged: bool = False
    points_used: int = 0
    near_far_gap_mm: float | None = None
    lever_arm_flag: str = ""
    theta_scale: float = 1.0
    sensor_offset_x_m: float = 0.0
    sensor_offset_y_m: float = 0.0
    validation_median_before_mm: float | None = None
    validation_median_after_mm: float | None = None
    validation_p90_before_mm: float | None = None
    validation_p90_after_mm: float | None = None
    overlap_before: float = 0.0
    overlap_after: float = 0.0
    condition_number: float | None = None


def _rotate_z(points: np.ndarray, angles_rad: np.ndarray) -> np.ndarray:
    c, s = np.cos(angles_rad), np.sin(angles_rad)
    return np.column_stack((points[:, 0] * c - points[:, 1] * s,
                            points[:, 0] * s + points[:, 1] * c,
                            points[:, 2]))


def _rotate_axis_angle(points: np.ndarray, axis: np.ndarray, angles_rad: np.ndarray) -> np.ndarray:
    k = np.asarray(axis, dtype=np.float64); k /= np.linalg.norm(k)
    c, s = np.cos(angles_rad)[:, None], np.sin(angles_rad)[:, None]
    return c * points + s * np.cross(k, points) + (1.0 - c) * (points @ k)[:, None] * k


def apply_axis_correction(points: np.ndarray, thetas_deg: np.ndarray, nx: float, ny: float,
                          theta0_deg: float = 0.0, theta_scale: float = 1.0,
                          sensor_offset_x_m: float = 0.0,
                          sensor_offset_y_m: float = 0.0) -> np.ndarray:
    """Undo capture's Rz(-theta), then apply tilt, lever arm and angle scale."""
    points = np.asarray(points, dtype=np.float64)
    theta = np.radians(np.asarray(thetas_deg, dtype=np.float64))
    sensor = _rotate_z(points, theta)
    sensor += np.array((sensor_offset_x_m, sensor_offset_y_m, 0.0))
    nz = np.sqrt(max(1.0 - nx * nx - ny * ny, 1e-12))
    corrected_angle = PAN_DIRECTION * float(theta_scale) * theta + np.radians(theta0_deg)
    return _rotate_axis_angle(sensor, np.array((nx, ny, nz)), corrected_angle)


def _sigma_min(points: np.ndarray, k: int) -> np.ndarray:
    _, ids = cKDTree(points).query(points, k=min(k + 1, len(points)))
    neighbors = points[ids]; centered = neighbors - neighbors.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered) / neighbors.shape[1]
    return np.sqrt(np.maximum(np.linalg.eigvalsh(covariance)[:, 0], 0.0))


def _balanced_indices(points: np.ndarray, theta: np.ndarray, target: int) -> np.ndarray:
    if len(points) <= target: return np.arange(len(points), dtype=np.int64)
    cells = np.floor(points / 0.08).astype(np.int64)
    bins = np.floor(np.mod(theta, 360.0) / 5.0).astype(np.int64)
    keys = np.column_stack((bins, cells))
    order = np.lexsort((np.arange(len(points)), keys[:, 3], keys[:, 2], keys[:, 1], keys[:, 0]))
    sorted_keys = keys[order]
    unique = order[np.r_[True, np.any(sorted_keys[1:] != sorted_keys[:-1], axis=1)]]
    if len(unique) >= target: return unique[np.linspace(0, len(unique) - 1, target, dtype=int)]
    remaining = np.setdiff1d(np.arange(len(points)), unique, assume_unique=False)
    extra = remaining[np.linspace(0, len(remaining) - 1, target - len(unique), dtype=int)]
    return np.r_[unique, extra]


def _validation_mask(points: np.ndarray, theta: np.ndarray, fraction: float) -> np.ndarray:
    cells = np.floor(points / 0.12).astype(np.int64)
    bins = np.floor(np.mod(theta, 360.0) / 10.0).astype(np.int64)
    hashed = (cells[:, 0] * 73856093) ^ (cells[:, 1] * 19349663) ^ (cells[:, 2] * 83492791) ^ bins
    return np.mod(hashed, max(2, int(round(1.0 / max(fraction, 1e-3))))) == 0


def _features(points: np.ndarray, settings: LevelingSettings) -> tuple[np.ndarray, np.ndarray]:
    k = min(settings.normal_knn + 1, len(points))
    distances, ids = cKDTree(points).query(points, k=k)
    neighbors = points[ids]; centered = neighbors - neighbors.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered) / k
    values, vectors = np.linalg.eigh(covariance)
    normals = vectors[:, :, 0]
    ratio = values[:, 0] / np.maximum(values[:, 1], 1e-12)
    reliable = (ratio <= settings.max_planarity_ratio) & (distances[:, -1] <= settings.max_neighbor_radius_m)
    return normals, reliable


def _pairs(points: np.ndarray, half: np.ndarray, eligible: np.ndarray,
           settings: LevelingSettings, max_distance: float):
    candidates=int(np.sum(eligible)); normals, planar = _features(points, settings); eligible = eligible & planar
    sources=[]; destinations=[]; output_normals=[]; kinds=[]
    for source_half in (False, True):
        source=np.flatnonzero(eligible & (half == source_half)); target=np.flatnonzero(eligible & (half != source_half))
        if not len(source) or not len(target): continue
        distance, nearest = cKDTree(points[target]).query(points[source], k=1); destination=target[nearest]
        compatible=np.abs(np.sum(normals[source]*normals[destination],axis=1)) >= settings.min_normal_dot
        keep=(distance <= max_distance) & compatible
        sources.append(source[keep]); destinations.append(destination[keep]); output_normals.append(normals[destination[keep]])
        kinds.append(np.abs(normals[destination[keep],2]))
    if not sources:
        empty=np.empty(0,dtype=np.int64); return empty,empty,np.empty((0,3)),np.empty(0),candidates
    return np.concatenate(sources),np.concatenate(destinations),np.concatenate(output_normals),np.concatenate(kinds),candidates


def _transform_params(points: np.ndarray, theta: np.ndarray, params: np.ndarray) -> np.ndarray:
    return apply_axis_correction(points,theta,params[0],params[1],0.0,params[4],params[2],params[3])


def _fixed_residuals(params, raw, theta, source, destination, normals):
    corrected=_transform_params(raw,theta,params)
    return np.sum((corrected[source]-corrected[destination])*normals,axis=1)


def _metrics(corrected,half,eligible,settings):
    src,dst,normals,normal_z,candidates=_pairs(corrected,half,eligible,settings,settings.final_correspondence_distance_m)
    values=np.abs(np.sum((corrected[src]-corrected[dst])*normals,axis=1)) if len(src) else np.empty(0)
    def stats(mask):
        selected=values[mask]
        return {"count":int(len(selected)),"median_mm":float(np.median(selected)*1000) if len(selected) else None,
                "p90_mm":float(np.percentile(selected,90)*1000) if len(selected) else None}
    walls=stats(normal_z<0.5); horizontal=stats(normal_z>0.8)
    usable_categories=[(name,item) for name,item in (("walls",walls),("horizontal_surfaces",horizontal)) if item["count"]>=20]
    worst_name,worst=max(usable_categories,key=lambda item:item[1]["median_mm"]) if usable_categories else ("all",stats(np.ones(len(values),dtype=bool)))
    return {**stats(np.ones(len(values),dtype=bool)),"overlap":float(len(values)/max(candidates,1)),
            "walls":walls,"horizontal_surfaces":horizontal,"score_surface":worst_name,
            "score_median_mm":worst["median_mm"],"score_p90_mm":worst["p90_mm"]}


def fit_spin_axis(points: np.ndarray, thetas_deg: np.ndarray, settings: LevelingSettings) -> dict:
    points=np.asarray(points,dtype=np.float64); theta=np.asarray(thetas_deg,dtype=np.float64)
    selected=_balanced_indices(points,theta,settings.target_points); raw=points[selected]; angles=theta[selected]
    half=np.mod(angles,360.0)>=180.0; validation=_validation_mask(raw,angles,settings.validation_fraction); training=~validation
    if min(np.sum(training&half),np.sum(training&~half),np.sum(validation&half),np.sum(validation&~half))<40:
        return {"converged":False,"message":"insufficient balanced coverage in both half-turns","points_used":len(raw)}
    initial=np.array((0.,0.,0.,0.,1.)); params=initial.copy(); bound=np.sin(np.radians(settings.max_accepted_tilt_deg))
    scale_bounds=(settings.theta_scale_min,settings.theta_scale_max) if settings.optimize_theta_affine else (1.-1e-12,1.+1e-12)
    lower=np.array((-bound,-bound,-settings.max_sensor_offset_m,-settings.max_sensor_offset_m,scale_bounds[0]))
    upper=np.array((bound,bound,settings.max_sensor_offset_m,settings.max_sensor_offset_m,scale_bounds[1]))
    result=None; correspondence_count=0
    for iteration in range(settings.max_outer_iterations):
        corrected=_transform_params(raw,angles,params)
        distance=settings.max_correspondence_distance_m+(settings.final_correspondence_distance_m-settings.max_correspondence_distance_m)*iteration/max(settings.max_outer_iterations-1,1)
        source,destination,normals,_kinds,_candidates=_pairs(corrected,half,training,settings,distance)
        correspondence_count=len(source)
        if correspondence_count<settings.min_correspondences: break
        result=least_squares(_fixed_residuals,params,args=(raw,angles,source,destination,normals),bounds=(lower,upper),
                             loss="huber",f_scale=settings.huber_delta_m,method="trf",max_nfev=settings.max_function_evals,
                             ftol=1e-7,xtol=1e-7,gtol=1e-7)
        change=np.linalg.norm(result.x-params); params=result.x
        if change<1e-7: break
    before=_metrics(_transform_params(raw,angles,initial),half,validation,settings)
    after=_metrics(_transform_params(raw,angles,params),half,validation,settings)
    condition=None; full_rank=False
    if result is not None and result.jac.size:
        singular=np.linalg.svd(result.jac,compute_uv=False); full_rank=len(singular)==5 and singular[-1]>1e-10
        condition=float(singular[0]/singular[-1]) if full_rank else None
    # Validate the surface class that was most doubled before correction. A
    # horizontal lever arm is tangential to a floor, so pooling a large floor
    # with a smaller wall would hide a real wall improvement in the median.
    score_surface=before["score_surface"]
    after_score=after.get(score_surface,after) if score_surface != "all" else after
    bx=before["score_median_mm"]; ax=after_score["median_mm"]
    before_p90=before["score_p90_mm"]; after_p90=after_score["p90_mm"]
    improvement=(bx-ax)/bx if bx is not None and ax is not None and bx>1e-9 else 0.0
    accepted=(result is not None and bool(result.success) and correspondence_count>=settings.min_correspondences
              and after["count"]>=settings.min_correspondences and before["overlap"]>=settings.min_overlap
              and after["overlap"]+1e-12>=before["overlap"] and improvement>=settings.min_median_improvement
              and after_p90 is not None and before_p90 is not None and after_p90<=before_p90
              and full_rank and condition is not None and condition<=settings.max_condition_number)
    reasons=[]
    if result is None or not result.success: reasons.append("optimizer did not converge")
    if correspondence_count<settings.min_correspondences or after["count"]<settings.min_correspondences: reasons.append("too few cross-pass correspondences")
    if before["overlap"]<settings.min_overlap: reasons.append("insufficient overlap")
    if after["overlap"]+1e-12<before["overlap"]: reasons.append("held-out overlap decreased")
    if improvement<settings.min_median_improvement: reasons.append(f"held-out median improved only {improvement*100:.1f}%")
    if after_p90 is None or before_p90 is None or after_p90>before_p90: reasons.append("held-out p90 worsened")
    if not full_rank or condition is None or condition>settings.max_condition_number: reasons.append("fit is poorly constrained")
    nx,ny,dx,dy,scale=map(float,params); nz=np.sqrt(max(1.-nx*nx-ny*ny,1e-12))
    return {"model_version":MODEL_VERSION,"nx":nx,"ny":ny,"sensor_offset_x_m":dx,"sensor_offset_y_m":dy,
            "theta0_deg":0.0,"theta_scale":scale,"tilt_deg":float(np.degrees(np.arccos(np.clip(nz,-1.,1.)))),
            "tilt_azimuth_deg":float(np.degrees(np.arctan2(ny,nx))%360.),"cost_before_mm":bx or 0.,"cost_after_mm":ax or 0.,
            "converged":accepted,"points_used":len(raw),"correspondences":correspondence_count,"condition_number":condition,
            "validation_before":before,"validation_after":after,
            "cross_sections":{"walls":{"before":before["walls"],"after":after["walls"]},
                              "horizontal_surfaces":{"before":before["horizontal_surfaces"],"after":after["horizontal_surfaces"]}},
            "message":"accepted" if accepted else "; ".join(dict.fromkeys(reasons))}


def _remove_stale_outputs(output_path: Path) -> None:
    for path in (output_path,output_path.with_suffix(".report.json")):
        try: path.unlink()
        except FileNotFoundError: pass


def level_scan(scan: ScanRecord,output_path: str|Path,settings: LevelingSettings|None=None,
               progress: Progress|None=None,cancelled: Cancelled|None=None) -> LevelingResult:
    from .publication import staged_call, protect_raw_output
    protect_raw_output(output_path,[scan])
    output_path=Path(output_path)
    result=staged_call(output_path.parent,lambda staging:_level_scan(scan,staging/output_path.name,
        settings,progress,cancelled),cancelled or (lambda:False))
    # A completed rejected fit must not leave its predecessor active. Failure
    # and cancellation never reach this point and preserve the complete result.
    if not result.applied:output_path.unlink(missing_ok=True)
    return result


def _level_scan(scan,output_path,settings,progress,cancelled):
    settings=settings or LevelingSettings(); progress=progress or (lambda _f,_m:None); cancelled=cancelled or (lambda:False)
    output_path=Path(output_path); _remove_stale_outputs(output_path)
    if not settings.enabled: return LevelingResult(scan.scan_id,None,None,False,"leveling disabled")
    source: PlyCloud=load_cloud(scan.cloud_path,require_raw=True); points=source.xyz; theta=np.asarray(source.vertex["theta_deg"],dtype=np.float64)
    if len(points)<settings.min_points_required:
        return LevelingResult(scan.scan_id,None,None,False,f"only {len(points)} points, need at least {settings.min_points_required}")
    if cancelled(): raise InterruptedError("Leveling cancelled")
    progress(.1,f"Fitting opposite half-turns for {scan.scan_id}"); fit=fit_spin_axis(points,theta,settings)
    report_path=output_path.with_suffix(".report.json"); report_path.parent.mkdir(parents=True,exist_ok=True)
    report_path.write_text(json.dumps({"scan_id":scan.scan_id,**fit,"settings":asdict(settings)},indent=2)+"\n",encoding="utf-8")
    if not fit.get("converged",False):
        before=fit.get("validation_before",{}); after=fit.get("validation_after",{})
        return LevelingResult(scan.scan_id,None,report_path,False,fit.get("message","fit rejected"),
            nx=fit.get("nx",0.),ny=fit.get("ny",0.),theta_scale=fit.get("theta_scale",1.),
            sensor_offset_x_m=fit.get("sensor_offset_x_m",0.),sensor_offset_y_m=fit.get("sensor_offset_y_m",0.),
            tilt_deg=fit.get("tilt_deg",0.),tilt_azimuth_deg=fit.get("tilt_azimuth_deg",0.),
            cost_before_mm=fit.get("cost_before_mm",0.),cost_after_mm=fit.get("cost_after_mm",0.),
            points_used=fit.get("points_used",0),condition_number=fit.get("condition_number"),
            validation_median_before_mm=before.get("median_mm"),validation_median_after_mm=after.get("median_mm"),
            validation_p90_before_mm=before.get("p90_mm"),validation_p90_after_mm=after.get("p90_mm"),
            overlap_before=before.get("overlap",0.),overlap_after=after.get("overlap",0.))
    if cancelled(): report_path.unlink(missing_ok=True); raise InterruptedError("Leveling cancelled")
    corrected=apply_axis_correction(points,theta,fit["nx"],fit["ny"],0.,fit["theta_scale"],fit["sensor_offset_x_m"],fit["sensor_offset_y_m"])
    progress(.75,f"Writing {len(points):,} corrected points")
    write_leveled_cloud(source,output_path,corrected,half_pass=(np.mod(theta,360.)>=180.).astype(np.uint8))
    before,after=fit["validation_before"],fit["validation_after"]
    progress(1.,f"Corrected {scan.scan_id}: {fit['cost_before_mm']:.1f} -> {fit['cost_after_mm']:.1f} mm")
    return LevelingResult(scan.scan_id,output_path,report_path,True,None,nx=fit["nx"],ny=fit["ny"],theta_scale=fit["theta_scale"],
        sensor_offset_x_m=fit["sensor_offset_x_m"],sensor_offset_y_m=fit["sensor_offset_y_m"],tilt_deg=fit["tilt_deg"],
        tilt_azimuth_deg=fit["tilt_azimuth_deg"],cost_before_mm=fit["cost_before_mm"],cost_after_mm=fit["cost_after_mm"],
        converged=True,points_used=fit["points_used"],condition_number=fit["condition_number"],
        validation_median_before_mm=before["median_mm"],validation_median_after_mm=after["median_mm"],
        validation_p90_before_mm=before["p90_mm"],validation_p90_after_mm=after["p90_mm"],overlap_before=before["overlap"],overlap_after=after["overlap"])


def level_scans(scans:list[ScanRecord],settings:LevelingSettings|None=None,progress:Progress|None=None,
                cancelled:Cancelled|None=None,output_dir:str|Path|None=None)->list[LevelingResult]:
    settings=settings or LevelingSettings(); progress=progress or (lambda _f,_m:None); cancelled=cancelled or (lambda:False); results=[]
    for index,scan in enumerate(scans):
        if cancelled(): raise InterruptedError("Leveling cancelled")
        progress(index/max(len(scans),1),f"Correcting {scan.scan_id}")
        path=(Path(output_dir) if output_dir else scan.cloud_path.parent)/scan.scan_id/"leveled_cloud.ply"
        try: result=level_scan(scan,path,settings,None,cancelled)
        except InterruptedError: raise
        except Exception as error: result=LevelingResult(scan.scan_id,None,None,False,f"failed: {error}")
        results.append(result)
    progress(1.,"Doubled-surface correction complete"); return results
