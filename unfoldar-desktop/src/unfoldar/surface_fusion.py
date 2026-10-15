from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from typing import Callable

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from .models import RegistrationEdge
from .registration.core import optimize_registration_graph


@dataclass(slots=True)
class AlignmentResult:
    transforms: dict[str,np.ndarray]
    accepted: bool
    before: dict
    after: dict
    edges_used: int
    max_translation_m: float
    max_rotation_deg: float
    reason: str


def _transform(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    return points@pose[:3,:3].T+pose[:3,3]


def _normal_transform(normals: np.ndarray, pose: np.ndarray) -> np.ndarray:
    return normals@pose[:3,:3].T


def _training_cloud(points: np.ndarray, voxel: float, validation: bool) -> o3d.geometry.PointCloud:
    keys=np.floor(points/max(voxel,.005)).astype(np.int64)
    holdout=np.mod((keys[:,0]*73856093)^(keys[:,1]*19349663)^(keys[:,2]*83492791),5)==0
    selected=points[holdout if validation else ~holdout]
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(selected)).voxel_down_sample(voxel)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*3,max_nn=40))
    return cloud


def _pair_metrics(clouds: dict[str,np.ndarray],poses: dict[str,np.ndarray],distance: float=.12) -> dict:
    residuals=[]; matched=0; candidates=0
    validation={key:_training_cloud(value,.025,True) for key,value in clouds.items()}
    for a,b in combinations(clouds,2):
        a_cloud=validation[a]; b_cloud=validation[b]
        pa=_transform(np.asarray(a_cloud.points),poses[a]); pb=_transform(np.asarray(b_cloud.points),poses[b])
        if len(pa)<20 or len(pb)<20:continue
        na=_normal_transform(np.asarray(a_cloud.normals),poses[a]); nb=_normal_transform(np.asarray(b_cloud.normals),poses[b])
        for source,target,target_normals in ((pa,pb,nb),(pb,pa,na)):
            dist,idx=cKDTree(target).query(source,k=1); candidates+=len(source); keep=dist<=distance; matched+=int(keep.sum())
            if np.any(keep):residuals.extend(np.abs(np.sum((source[keep]-target[idx[keep]])*target_normals[idx[keep]],axis=1)).tolist())
    values=np.asarray(residuals)
    return {"count":int(len(values)),"median_mm":float(np.median(values)*1000) if len(values) else None,
            "p90_mm":float(np.percentile(values,90)*1000) if len(values) else None,
            "overlap":float(matched/max(candidates,1))}


def refine_alignment(
    clouds: dict[str,np.ndarray],initial: dict[str,np.ndarray],reference_id: str,
    enabled: bool=True,progress: Callable[[float,str],None]|None=None,
    cancelled: Callable[[],bool]|None=None,
) -> AlignmentResult:
    progress=progress or (lambda _v,_m:None); cancelled=cancelled or (lambda:False)
    baseline={key:np.asarray(value,dtype=np.float64).copy() for key,value in initial.items()}
    before=_pair_metrics(clouds,baseline)
    if not enabled or len(clouds)<2:
        return AlignmentResult(baseline,False,before,before,0,0.,0.,"disabled or single scan")
    # Keep 1 cm source detail for the 4/2/1 cm ICP levels.
    training={key:_training_cloud(points,.01,False) for key,points in clouds.items()}
    edges=[]; pairs=list(combinations(clouds,2))
    loss=o3d.pipelines.registration.TukeyLoss(k=.03)
    for index,(source_id,target_id) in enumerate(pairs):
        if cancelled():raise InterruptedError("Fusion alignment cancelled")
        progress(index/max(len(pairs),1),f"Refining {source_id} against {target_id}")
        source=training[source_id]; target=training[target_id]
        transform=np.linalg.inv(baseline[target_id])@baseline[source_id]; result=None
        for voxel in (.04,.02,.01):
            src=source.voxel_down_sample(voxel); dst=target.voxel_down_sample(voxel)
            src.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*3,max_nn=50))
            dst.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*3,max_nn=50))
            try:
                result=o3d.pipelines.registration.registration_icp(
                    src,dst,voxel*2,transform,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=35),
                )
            except RuntimeError:
                result=None;break
            transform=np.asarray(result.transformation)
        if result is None or result.fitness<.12:continue
        information=o3d.pipelines.registration.get_information_matrix_from_point_clouds(src,dst,.02,transform)
        edges.append(RegistrationEdge(source_id,target_id,transform,information,"fusion_fine_icp",float(result.fitness),float(result.inlier_rmse),float(result.fitness/max(result.inlier_rmse,.001)),True,True,False))
    try:
        candidate=optimize_registration_graph(list(clouds),edges,reference_id,.02)
    except ValueError as error:
        return AlignmentResult(baseline,False,before,before,len(edges),0.,0.,str(error))
    after=_pair_metrics(clouds,candidate)
    translations=[]; rotations=[]
    for key in clouds:
        delta=np.linalg.inv(baseline[key])@candidate[key]
        translations.append(float(np.linalg.norm(delta[:3,3])))
        rotations.append(float(np.degrees(np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1)))))
    bm,am=before["median_mm"],after["median_mm"]
    improvement=(bm-am)/bm if bm and am is not None else 0.
    accepted=(after["count"]>=200 and improvement>=.10 and after["p90_mm"] is not None
              and before["p90_mm"] is not None and after["p90_mm"]<=before["p90_mm"]
              and after["overlap"]>=before["overlap"] and max(translations,default=0)<=.10
              and max(rotations,default=0)<=3.)
    reason="accepted" if accepted else f"held-out validation rejected refinement (median improvement {improvement*100:.1f}%)"
    return AlignmentResult(candidate if accepted else baseline,accepted,before,after,len(edges),max(translations,default=0.),max(rotations,default=0.),reason)


def surface_reliability(points: np.ndarray,normals: np.ndarray) -> np.ndarray:
    """Continuous local plane confidence estimated inside one source scan."""
    if len(points)<8:return np.zeros(len(points),dtype=np.float64)
    distance,ids=cKDTree(points).query(points,k=min(10,len(points)))
    agreement=np.median(np.abs(np.einsum("nki,ni->nk",normals[ids],normals)),axis=1)
    angular=np.clip((agreement-.90)/.10,0.,1.)
    support=np.clip((.20-distance[:,-1])/.15,0.,1.)
    return angular*support


def cross_scan_surface_metrics(points: np.ndarray,normals: np.ndarray,source_ids: np.ndarray,
                               maximum_distance_m: float=.10,normal_angle_deg: float=20.) -> dict:
    """Actual symmetric cross-scan point-to-plane error on distributed samples."""
    residuals=[];matched=0;candidates=0;cosine=np.cos(np.radians(normal_angle_deg))
    scans=np.unique(source_ids);trees={int(scan):cKDTree(points[source_ids==scan]) for scan in scans}
    scan_points={int(scan):points[source_ids==scan] for scan in scans}
    scan_normals={int(scan):normals[source_ids==scan] for scan in scans}
    for source in scans:
        source=int(source);indices=np.flatnonzero(source_ids==source)
        if len(indices)>12000:indices=indices[np.linspace(0,len(indices)-1,12000,dtype=np.int64)]
        sample_points=points[indices];sample_normals=normals[indices]
        for target in scans:
            target=int(target)
            if target==source:continue
            distance,index=trees[target].query(sample_points,k=1,distance_upper_bound=maximum_distance_m)
            valid=index<len(scan_points[target]);candidates+=len(indices)
            if not np.any(valid):continue
            target_normal=scan_normals[target][index[valid]];compatible=np.einsum("ni,ni->n",sample_normals[valid],target_normal)>=cosine
            valid_rows=np.flatnonzero(valid)[compatible];target_index=index[valid][compatible];matched+=len(valid_rows)
            if len(valid_rows):
                delta=sample_points[valid_rows]-scan_points[target][target_index]
                residuals.extend(np.abs(np.einsum("ni,ni->n",delta,target_normal[compatible])).tolist())
    values=np.asarray(residuals)
    return {"count":int(len(values)),"overlap":float(matched/max(candidates,1)),
            "median_mm":float(np.median(values)*1000) if len(values) else None,
            "p90_mm":float(np.percentile(values,90)*1000) if len(values) else None}


def collapse_supported_layers(
    points: np.ndarray,normals: np.ndarray,source_ids: np.ndarray,reliable: np.ndarray,
    separation_m: float,normal_angle_deg: float,architectural: bool,
    cancelled: Callable[[],bool]|None=None,ranges: np.ndarray|None=None,
    range_scale_m: float=3.,minimum_range_weight: float=.2,
    progress: Callable[[float,str],None]|None=None,
) -> tuple[np.ndarray,dict]:
    """Move multi-scan observations onto a shared local surface without chaining."""
    cancelled=cancelled or (lambda:False);progress=progress or (lambda _v,_m:None); output=points.copy(); tree=cKDTree(points)
    ranges=np.ones(len(points)) if ranges is None else np.asarray(ranges,dtype=np.float64)
    range_weight=np.maximum(minimum_range_weight,1./(1.+np.square(ranges/max(range_scale_m,1e-6))))
    cosine=np.cos(np.radians(normal_angle_deg)); tangent_limit=.055 if architectural else .03
    moved=0; groups=0; rejected_span=0; chunk=30000
    for start in range(0,len(points),chunk):
        if cancelled():raise InterruptedError("Surface fusion cancelled")
        progress(start/max(len(points),1),"Matching cross-scan surface neighborhoods")
        stop=min(start+chunk,len(points)); block_points=points[start:stop];block_normals=normals[start:stop]
        _distance,ids=tree.query(block_points,k=min(24,len(points)),distance_upper_bound=np.hypot(separation_m,tangent_limit))
        valid_ids=ids<len(points);safe_ids=np.where(valid_ids,ids,0)
        delta=points[safe_ids]-block_points[:,None,:]
        perpendicular=np.einsum("nki,ni->nk",delta,block_normals)
        tangent2=np.maximum(0.,np.einsum("nki,nki->nk",delta,delta)-perpendicular*perpendicular)
        normal_dot=np.einsum("nki,ni->nk",normals[safe_ids],block_normals)
        row_sources=source_ids[start:stop]
        # Same-sign scanner-oriented normals keep opposite sides of thin surfaces apart.
        eligible=(valid_ids&(reliable[start:stop,None]>=.55)&(reliable[safe_ids]>=.55)
                  &(source_ids[safe_ids]!=row_sources[:,None])&(np.abs(perpendicular)<=separation_m)
                  &(tangent2<=tangent_limit*tangent_limit)&(normal_dot>=cosine))
        scan_count=int(source_ids.max())+1;scan_offsets=np.full((stop-start,scan_count),np.nan);scan_weights=np.zeros_like(scan_offsets)

        def row_median(values: np.ndarray,mask: np.ndarray) -> tuple[np.ndarray,np.ndarray]:
            count=mask.sum(axis=1);ordered=np.sort(np.where(mask,values,np.inf),axis=1)
            lo=np.maximum((count-1)//2,0);hi=count//2;rows=np.arange(len(count))
            median=(ordered[rows,lo]+ordered[rows,hi])*.5
            return median,count>0

        for scan in range(scan_count):
            own=row_sources==scan;scan_offsets[own,scan]=0.;scan_weights[own,scan]=range_weight[start:stop][own]
            scan_mask=eligible&(source_ids[safe_ids]==scan)
            median_offset,has=row_median(perpendicular,scan_mask)
            median_weight,_=row_median(range_weight[safe_ids],scan_mask)
            scan_offsets[has,scan]=median_offset[has];scan_weights[has,scan]=median_weight[has]
        support=np.sum(np.isfinite(scan_offsets),axis=1)>=2
        finite=np.isfinite(scan_offsets)
        span=np.max(np.where(finite,scan_offsets,-np.inf),axis=1)-np.min(np.where(finite,scan_offsets,np.inf),axis=1)
        rejected=support&(span>separation_m);rejected_span+=int(rejected.sum())
        accepted=support&~rejected
        numerator=np.nansum(scan_offsets*scan_weights,axis=1);denominator=scan_weights.sum(axis=1)
        correction=np.divide(numerator,denominator,out=np.zeros_like(numerator),where=denominator>0)
        correction=np.clip(correction,-separation_m,separation_m)
        block=output[start:stop];block[accepted]+=correction[accepted,None]*block_normals[accepted]
        groups+=int(accepted.sum());moved+=int(np.sum(accepted&(np.abs(correction)>1e-9)))
    displacement=np.linalg.norm(output-points,axis=1); nonzero=displacement[displacement>1e-9]
    progress(1.,"Cross-scan surface matching complete")
    return output,{"points_moved":moved,"supported_groups":groups,"groups_rejected_for_span":rejected_span,
                   "maximum_layer_separation_m":separation_m,"normal_angle_deg":normal_angle_deg,
                   "median_corrected_displacement_mm":float(np.median(nonzero)*1000) if len(nonzero) else 0.,
                   "p90_corrected_displacement_mm":float(np.percentile(nonzero,90)*1000) if len(nonzero) else 0.,
                   "maximum_corrected_displacement_mm":float(nonzero.max()*1000) if len(nonzero) else 0.}


def source_balanced_voxels(points,normals,colors,valid,intensity,source_ids,point_ids,voxel,reliability=None,
                           ranges=None,range_scale_m=3.,minimum_range_weight=.2):
    if voxel<=0:
        return points,normals,colors,valid,intensity,np.ones(len(points),np.uint32),np.ones(len(points),np.uint16),np.arange(len(points)+1),point_ids,source_ids
    keys=np.floor(points/voxel).astype(np.int64); _,inverse=np.unique(keys,axis=0,return_inverse=True); groups=int(inverse.max())+1
    pair=np.column_stack((inverse,source_ids)); unique_pairs,pair_inverse,pair_count=np.unique(pair,axis=0,return_inverse=True,return_counts=True)
    confidence=np.ones(len(points)) if reliability is None else np.clip(np.asarray(reliability,dtype=np.float64),.1,1.)
    ranges=np.ones(len(points)) if ranges is None else np.asarray(ranges,dtype=np.float64)
    range_weight=np.maximum(minimum_range_weight,1./(1.+np.square(ranges/max(range_scale_m,1e-6))))
    # Normalize source density, then weight by scanner distance.
    pair_confidence=np.bincount(pair_inverse,confidence,minlength=len(unique_pairs))
    pair_range=np.bincount(pair_inverse,confidence*range_weight,minlength=len(unique_pairs))/np.maximum(pair_confidence,1e-12)
    weights=confidence/np.maximum(pair_confidence[pair_inverse],1e-12)*pair_range[pair_inverse]
    def mean(values,extra=None):
        values2=values if values.ndim==2 else values[:,None]; weight=weights if extra is None else weights*extra
        out=np.column_stack([np.bincount(inverse,values2[:,i]*weight,minlength=groups) for i in range(values2.shape[1])]); denom=np.bincount(inverse,weight,minlength=groups)
        out/=np.maximum(denom[:,None],1e-12); return out if values.ndim==2 else out[:,0]
    fused_normals=mean(normals); fused_normals/=np.maximum(np.linalg.norm(fused_normals,axis=1,keepdims=True),1e-12)
    fused_valid=np.bincount(inverse,valid.astype(float),minlength=groups)>0
    fused_colors=mean(colors,valid.astype(float)); fused_colors[~fused_valid]=0
    observations=np.bincount(inverse,minlength=groups).astype(np.uint32)
    scan_support=np.bincount(unique_pairs[:,0],minlength=groups).astype(np.uint16)
    order=np.argsort(inverse,kind="stable"); counts=np.bincount(inverse,minlength=groups); offsets=np.r_[0,np.cumsum(counts)]
    return mean(points),fused_normals,fused_colors,fused_valid,mean(intensity),observations,scan_support,offsets,point_ids[order],source_ids[order]
