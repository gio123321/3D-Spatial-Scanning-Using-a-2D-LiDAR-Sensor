from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from pathlib import Path
from typing import Callable

import numpy as np
import open3d as o3d

from ..calibration.camera import map_lidar_points
from ..io.scan import load_cloud
from ..models import CameraProfile, RegistrationEdge, ScanRecord


@dataclass(slots=True)
class RegistrationSettings:
    voxel_size_m: float = 0.08
    normal_radius_factor: float = 2.5
    feature_radius_factor: float = 5.0
    ransac_distance_factor: float = 1.5
    ransac_iterations: int = 100_000
    ransac_confidence: float = 0.999
    refine_scale_factors: tuple[float, ...] = (2.0, 1.0, 0.5)
    refine_iterations: tuple[int, ...] = (60, 40, 25)
    refinement: str = "point_to_plane"
    use_colored_icp: bool = True
    min_fitness: float = 0.18
    max_inlier_rmse_factor: float = 1.5
    random_seed: int = 7


def build_scan_cloud(
    scan: ScanRecord, profile: CameraProfile,
    colored_path: str | Path | None = None, leveled_path: str | Path | None = None,
) -> o3d.geometry.PointCloud:
    source_path = colored_path or leveled_path or scan.cloud_path
    raw = load_cloud(source_path, require_raw=colored_path is None and leveled_path is None)
    points = map_lidar_points(raw.xyz, profile)
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    names = set(raw.property_names)
    if {"red", "green", "blue"}.issubset(names):
        colors = np.column_stack((raw.vertex["red"], raw.vertex["green"], raw.vertex["blue"])).astype(np.float64) / 255.0
        cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud


def _proxy(cloud: o3d.geometry.PointCloud, settings: RegistrationSettings) -> tuple[o3d.geometry.PointCloud, o3d.pipelines.registration.Feature]:
    voxel = settings.voxel_size_m
    down = cloud.voxel_down_sample(voxel)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * settings.normal_radius_factor, max_nn=40)
    )
    features = o3d.pipelines.registration.compute_fpfh_feature(
        down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * settings.feature_radius_factor, max_nn=100)
    )
    return down, features


def register_pair(
    source_id: str,
    target_id: str,
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    settings: RegistrationSettings | None = None,
) -> RegistrationEdge:
    settings = settings or RegistrationSettings()
    o3d.utility.random.seed(settings.random_seed)
    source_down, source_fpfh = _proxy(source, settings)
    target_down, target_fpfh = _proxy(target, settings)
    distance = settings.voxel_size_m * settings.ransac_distance_factor
    coarse = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        True,
        distance,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3,
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance),
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(
            settings.ransac_iterations, settings.ransac_confidence
        ),
    )
    transform = coarse.transformation
    result = coarse
    method = settings.refinement
    # Track the cloud pair and distance used by the final refinement stage.
    info_source, info_target, info_distance = source_down, target_down, distance
    for scale, iterations in zip(settings.refine_scale_factors, settings.refine_iterations, strict=True):
        voxel = settings.voxel_size_m * scale
        src = source.voxel_down_sample(voxel)
        dst = target.voxel_down_sample(voxel)
        radius = voxel * settings.normal_radius_factor
        src.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=50))
        dst.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=50))
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iterations)
        icp_distance = voxel * 1.5
        try:
            if settings.refinement == "generalized_icp":
                step = o3d.pipelines.registration.registration_generalized_icp(
                    src, dst, icp_distance, transform,
                    o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(), criteria,
                )
            else:
                step = o3d.pipelines.registration.registration_icp(
                    src, dst, icp_distance, transform,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(), criteria,
                )
        except RuntimeError:
            # Keep the coarser result when this scale has no correspondences.
            break
        result = step
        transform = result.transformation
        info_source, info_target, info_distance = src, dst, icp_distance
    source_color_coverage = float(np.mean(np.any(np.asarray(source.colors) > 0, axis=1))) if source.has_colors() else 0.0
    target_color_coverage = float(np.mean(np.any(np.asarray(target.colors) > 0, axis=1))) if target.has_colors() else 0.0
    if settings.use_colored_icp and result.fitness > 0.0 and min(source_color_coverage, target_color_coverage) >= 0.2:
        voxel = settings.voxel_size_m * 0.5
        src = source.voxel_down_sample(voxel); dst = target.voxel_down_sample(voxel)
        src.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.5, max_nn=50))
        dst.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.5, max_nn=50))
        icp_distance = voxel * 1.5
        try:
            colored = o3d.pipelines.registration.registration_colored_icp(
                src, dst, icp_distance, transform,
                o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
            )
        except RuntimeError:
            colored = None
        if colored is not None and colored.fitness > 0.0:
            result = colored
            transform = result.transformation
            method += "+colored"
            info_source, info_target, info_distance = src, dst, icp_distance
    # Use the cloud pair and distance from the last successful refinement.
    information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        info_source, info_target, info_distance, transform
    )
    source_diag = np.linalg.norm(np.asarray(source.get_max_bound()) - np.asarray(source.get_min_bound()))
    target_diag = np.linalg.norm(np.asarray(target.get_max_bound()) - np.asarray(target.get_min_bound()))
    plausible_translation = np.linalg.norm(transform[:3, 3]) <= 2.0 * max(source_diag, target_diag)
    accepted = (
        result.fitness >= settings.min_fitness
        and result.inlier_rmse <= settings.voxel_size_m * settings.max_inlier_rmse_factor
        and plausible_translation
    )
    confidence = float(result.fitness / max(result.inlier_rmse + settings.voxel_size_m, 1e-9))
    return RegistrationEdge(
        source_id, target_id, np.asarray(transform), np.asarray(information), method,
        float(result.fitness), float(result.inlier_rmse), confidence, bool(accepted), True, False,
    )


def register_scans(
    scans: list[ScanRecord],
    profile: CameraProfile,
    colored_paths: dict[str, Path] | None = None,
    settings: RegistrationSettings | None = None,
    pair_mode: str = "auto",
    progress: Callable[[float, str], None] | None = None,
    cancelled: Callable[[], bool] | None = None,
    leveled_paths: dict[str, Path] | None = None,
) -> list[RegistrationEdge]:
    settings = settings or RegistrationSettings()
    progress = progress or (lambda _fraction, _message: None)
    cancelled = cancelled or (lambda: False)
    colored_paths = colored_paths or {}
    leveled_paths = leveled_paths or {}
    clouds = {
        scan.scan_id: build_scan_cloud(scan, profile, colored_paths.get(scan.scan_id), leveled_paths.get(scan.scan_id))
        for scan in scans
    }
    if pair_mode == "adjacent" or (pair_mode == "auto" and len(scans) > 10):
        pairs = list(zip(scans[:-1], scans[1:]))
        # Include a closing edge between the sequence endpoints for loop correction.
        if len(scans) > 2:
            pairs.append((scans[-1], scans[0]))
    else:
        pairs = list(combinations(scans, 2))
    edges: list[RegistrationEdge] = []
    for index, (source, target) in enumerate(pairs):
        if cancelled():
            raise InterruptedError("Registration cancelled")
        progress(index / max(len(pairs), 1), f"Registering {source.scan_id} -> {target.scan_id}")
        try:
            edge = register_pair(source.scan_id, target.scan_id, clouds[source.scan_id], clouds[target.scan_id], settings)
        except InterruptedError:
            raise
        except Exception as error:  # noqa: BLE001 - one bad pair must not abort the whole batch
            edge = RegistrationEdge(
                source.scan_id, target.scan_id, np.eye(4), np.eye(6), f"failed: {error}",
                0.0, float("inf"), 0.0, False, True, False,
            )
        edges.append(edge)
    progress(1.0, "Pairwise registration complete")
    return edges


def _initial_world_poses(
    scan_ids: list[str], edges: list[RegistrationEdge], reference_id: str
) -> tuple[dict[str, np.ndarray], set[int]]:
    accepted = sorted((edge for edge in edges if edge.accepted), key=lambda edge: edge.confidence, reverse=True)
    parent = {scan_id: scan_id for scan_id in scan_ids}

    def root(node: str) -> str:
        while parent[node] != node:
            parent[node] = parent[parent[node]]; node = parent[node]
        return node

    tree: list[RegistrationEdge] = []
    for edge in accepted:
        a, b = root(edge.source_id), root(edge.target_id)
        if a != b:
            parent[a] = b; tree.append(edge)
    if len(tree) != len(scan_ids) - 1:
        groups: dict[str, list[str]] = {}
        for scan_id in scan_ids:
            groups.setdefault(root(scan_id), []).append(scan_id)
        raise ValueError(f"Registration graph is disconnected: {list(groups.values())}")
    adjacency: dict[str, list[tuple[str, np.ndarray]]] = {scan_id: [] for scan_id in scan_ids}
    for edge in tree:
        adjacency[edge.source_id].append((edge.target_id, edge.transformation))
        adjacency[edge.target_id].append((edge.source_id, np.linalg.inv(edge.transformation)))
    poses = {reference_id: np.eye(4)}
    queue = [reference_id]
    while queue:
        current = queue.pop(0)
        for neighbor, neighbor_from_current in adjacency[current]:
            if neighbor not in poses:
                poses[neighbor] = poses[current] @ np.linalg.inv(neighbor_from_current)
                queue.append(neighbor)
    return poses, {id(edge) for edge in tree}


@dataclass(slots=True)
class ScanResidual:
    scan_id: str
    max_edge_error_m: float
    mean_edge_error_m: float
    edge_count: int
    worst_partner: str | None


def compute_registration_residuals(
    scan_ids: list[str], edges: list[RegistrationEdge], poses: dict[str, np.ndarray],
) -> dict[str, ScanResidual]:
    """Measure translation disagreement between accepted edges and optimized poses.
    Edges map source to target; implied transforms are inv(poses[target]) @ poses[source].
    """
    per_scan: dict[str, list[tuple[float, str]]] = {scan_id: [] for scan_id in scan_ids}
    for edge in edges:
        if not edge.accepted:
            continue
        source_pose = poses.get(edge.source_id); target_pose = poses.get(edge.target_id)
        if source_pose is None or target_pose is None:
            continue
        implied = np.linalg.inv(target_pose) @ source_pose
        error_matrix = np.linalg.inv(edge.transformation) @ implied
        error_m = float(np.linalg.norm(error_matrix[:3, 3]))
        per_scan[edge.source_id].append((error_m, edge.target_id))
        per_scan[edge.target_id].append((error_m, edge.source_id))
    residuals: dict[str, ScanResidual] = {}
    for scan_id in scan_ids:
        entries = per_scan[scan_id]
        if not entries:
            residuals[scan_id] = ScanResidual(scan_id, 0.0, 0.0, 0, None); continue
        errors = [error for error, _ in entries]; worst = max(entries, key=lambda item: item[0])
        residuals[scan_id] = ScanResidual(scan_id, max(errors), float(np.mean(errors)), len(entries), worst[1])
    return residuals


def optimize_registration_graph(
    scan_ids: list[str], edges: list[RegistrationEdge], reference_id: str, voxel_size_m: float
) -> dict[str, np.ndarray]:
    if reference_id not in scan_ids:
        raise ValueError("Reference scan is not present")
    initial, tree_edges = _initial_world_poses(scan_ids, edges, reference_id)
    index = {scan_id: i for i, scan_id in enumerate(scan_ids)}
    graph = o3d.pipelines.registration.PoseGraph()
    for scan_id in scan_ids:
        graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(initial[scan_id]))
    for edge in edges:
        if edge.accepted:
            graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(
                    index[edge.source_id], index[edge.target_id], edge.transformation,
                    edge.information, uncertain=edge.uncertain and not edge.manual and id(edge) not in tree_edges,
                    confidence=max(0.0, min(1.0, edge.confidence)),
                )
            )
    o3d.pipelines.registration.global_optimization(
        graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=voxel_size_m * 1.5,
            edge_prune_threshold=0.25,
            reference_node=index[reference_id],
        ),
    )
    return {scan_id: np.asarray(graph.nodes[i].pose) for scan_id, i in index.items()}
