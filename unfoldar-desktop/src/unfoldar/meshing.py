from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from collections.abc import Callable

import numpy as np
import open3d as o3d


@dataclass(slots=True)
class MeshingSettings:
    method: str = "poisson"
    poisson_depth: int = 10
    poisson_density_quantile: float = 0.02
    # Use linear iso-vertex interpolation for planar surfaces.
    poisson_linear_fit: bool = True
    normal_radius_m: float = 0.06
    normal_max_neighbors: int = 50
    orient_consistency_neighbors: int = 30
    support_distance_m: float = 0.06
    bounds_margin_m: float = 0.05
    min_component_triangles: int = 200
    min_component_fraction: float = 0.0005
    smooth_iterations: int = 0
    # Keep the full reconstruction separately from the mesh used for viewing/export.
    simplify_target_triangles: int = 300_000
    # Squared before passing to Open3D maximum_error (quadric cost).
    # This is not a measured maximum geometric-distance guarantee.
    simplify_max_deviation_m: float = 0.01
    bpa_radius_multipliers: tuple[float, ...] = (1.5, 3.0, 6.0)


@dataclass(slots=True)
class MeshingResult:
    ply_path: Path
    obj_path: Path | None
    detail_ply_path: Path | None
    vertices: int
    triangles: int
    report_path: Path


def _point_spacing(cloud: o3d.geometry.PointCloud) -> float:
    distances = np.asarray(cloud.compute_nearest_neighbor_distance())
    finite = distances[np.isfinite(distances) & (distances > 0)]
    return float(np.median(finite)) if len(finite) else 0.01


def _remove_unsupported(mesh: o3d.geometry.TriangleMesh, cloud: o3d.geometry.PointCloud, threshold: float) -> None:
    if threshold <= 0 or not len(mesh.vertices):
        return
    vertices = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(mesh.vertices)))
    distances = np.asarray(vertices.compute_point_cloud_distance(cloud))
    mesh.remove_vertices_by_mask(distances > threshold)


def _remove_small_components(mesh: o3d.geometry.TriangleMesh, minimum: int, fraction: float) -> None:
    if not len(mesh.triangles):
        return
    labels, counts, _ = mesh.cluster_connected_triangles()
    labels = np.asarray(labels); counts = np.asarray(counts)
    limit = max(minimum, int(len(mesh.triangles) * fraction))
    mesh.remove_triangles_by_mask(counts[labels] < limit)
    mesh.remove_unreferenced_vertices()


def generate_mesh(
    merged_cloud_path: str | Path,
    output_ply: str | Path,
    settings: MeshingSettings | None = None,
    output_obj: str | Path | None = None,
    progress: Callable[[float, str], None] | None = None,
    cancelled: Callable[[], bool] | None = None,
) -> MeshingResult:
    from .publication import staged_call
    destination = Path(output_ply)
    if output_obj and Path(output_obj).parent.resolve() != destination.parent.resolve():
        raise ValueError("PLY and OBJ output must share a directory for safe publication")
    return staged_call(destination.parent, lambda staging: _generate_mesh(
        merged_cloud_path, staging / destination.name, settings,
        staging / Path(output_obj).name if output_obj else None, progress, cancelled),
        cancelled or (lambda: False))


def _generate_mesh(merged_cloud_path, output_ply, settings, output_obj, progress, cancelled):
    settings = settings or MeshingSettings()
    progress = progress or (lambda _value, _message: None)
    cancelled = cancelled or (lambda: False)
    if not merged_cloud_path:
        raise ValueError("Generate/fuse a merged point cloud before meshing")
    progress(0.02, "Loading merged point cloud")
    cloud = o3d.io.read_point_cloud(str(merged_cloud_path))
    if cloud.is_empty():
        raise ValueError("Merged point cloud is empty")
    if not cloud.has_normals():
        progress(0.08, "Estimating mesh normals")
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(settings.normal_radius_m, settings.normal_max_neighbors))
    if cancelled():
        raise InterruptedError("Meshing cancelled")
    progress(0.14, "Orienting normals")
    try:
        cloud.orient_normals_consistent_tangent_plane(settings.orient_consistency_neighbors)
    except RuntimeError:
        pass
    spacing = _point_spacing(cloud)
    if cancelled():
        raise InterruptedError("Meshing cancelled")
    if settings.method == "ball_pivoting":
        progress(0.24, "Ball-pivoting reconstruction")
        radii = [spacing * factor for factor in settings.bpa_radius_multipliers]
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(cloud, o3d.utility.DoubleVector(radii))
        densities = None
    elif settings.method == "poisson":
        progress(0.24, f"Poisson reconstruction (depth {settings.poisson_depth})")
        mesh, densities_raw = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            cloud, depth=settings.poisson_depth, linear_fit=settings.poisson_linear_fit
        )
        if cancelled():
            raise InterruptedError("Meshing cancelled")
        progress(0.58, "Removing low-density Poisson artifacts")
        densities = np.asarray(densities_raw)
        if len(densities) and settings.poisson_density_quantile > 0:
            cutoff = np.quantile(densities, settings.poisson_density_quantile)
            mesh.remove_vertices_by_mask(densities < cutoff)
    else:
        raise ValueError("Meshing method must be 'poisson' or 'ball_pivoting'")
    if not len(mesh.triangles):
        raise ValueError("Surface reconstruction produced no triangles; try Ball Pivoting or a lower Poisson depth")
    progress(0.66, "Cropping and removing unsupported surface")
    bounds = cloud.get_axis_aligned_bounding_box()
    bounds = o3d.geometry.AxisAlignedBoundingBox(
        bounds.min_bound - settings.bounds_margin_m,
        bounds.max_bound + settings.bounds_margin_m,
    )
    mesh = mesh.crop(bounds)
    _remove_unsupported(mesh, cloud, max(settings.support_distance_m, spacing * 5.0))
    mesh.remove_degenerate_triangles(); mesh.remove_duplicated_triangles(); mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges(); mesh.remove_unreferenced_vertices()
    _remove_small_components(mesh, settings.min_component_triangles, settings.min_component_fraction)
    if not len(mesh.triangles):
        raise ValueError(
            "Mesh cleanup removed every triangle; increase support distance or reduce the minimum component size"
        )
    if settings.smooth_iterations > 0:
        progress(0.73, "Applying careful Taubin smoothing")
        mesh = mesh.filter_smooth_taubin(number_of_iterations=settings.smooth_iterations)
    unsimplified_triangles = len(mesh.triangles)
    output_ply = Path(output_ply); output_ply.parent.mkdir(parents=True, exist_ok=True)
    detail_path: Path | None = None
    if 0 < settings.simplify_target_triangles < len(mesh.triangles):
        if cancelled():
            raise InterruptedError("Meshing cancelled")
        detail_path = output_ply.with_name(f"{output_ply.stem}_full{output_ply.suffix}")
        progress(0.78, f"Saving full-detail mesh ({len(mesh.triangles):,} triangles)")
        mesh.compute_vertex_normals()
        if not o3d.io.write_triangle_mesh(str(detail_path), mesh, write_ascii=False):
            raise OSError(f"Failed to write {detail_path}")
        progress(0.84, f"Simplifying working mesh (up to {settings.simplify_target_triangles:,} triangles, {settings.simplify_max_deviation_m*1000:.1f} mm tolerance)")
        # Open3D may stop above the requested triangle target when its
        # quadric error bound prevents further collapses.
        max_error = settings.simplify_max_deviation_m ** 2 if settings.simplify_max_deviation_m > 0 else float("inf")
        mesh = mesh.simplify_quadric_decimation(settings.simplify_target_triangles, maximum_error=max_error, boundary_weight=2.0)
        # Remove degenerate triangles left by edge collapses.
        mesh.remove_degenerate_triangles(); mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    progress(0.92, "Writing mesh exports")
    if not o3d.io.write_triangle_mesh(str(output_ply), mesh, write_ascii=False):
        raise OSError(f"Failed to write {output_ply}")
    obj_path = Path(output_obj) if output_obj else None
    if obj_path:
        obj_path.parent.mkdir(parents=True, exist_ok=True)
        if not o3d.io.write_triangle_mesh(str(obj_path), mesh, write_triangle_uvs=False, write_vertex_colors=False):
            raise OSError(f"Failed to write {obj_path}")
    report = output_ply.with_suffix(".report.json")
    report.write_text(json.dumps({
        "method": settings.method, "point_spacing_m": spacing,
        "vertices": len(mesh.vertices), "triangles": len(mesh.triangles),
        "triangles_before_simplification": unsimplified_triangles,
        "full_detail_mesh": str(detail_path) if detail_path else None,
        "settings": asdict(settings),
    }, indent=2) + "\n", encoding="utf-8")
    progress(1.0, f"Mesh ready: {len(mesh.triangles):,} triangles")
    return MeshingResult(output_ply, obj_path, detail_path, len(mesh.vertices), len(mesh.triangles), report)
