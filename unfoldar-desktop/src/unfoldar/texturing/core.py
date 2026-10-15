from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from collections.abc import Callable

import cv2
import numpy as np
import open3d as o3d
import xatlas

from ..calibration.camera import camera_pose, project_camera_points
from ..calibration.profile import rescale_profile
from ..models import CameraProfile, ScanRecord, ShotRecord
from ..io.scan import image_pose_pairs
from ..photometric import apply_photo_gain, estimate_photo_gains


@dataclass(slots=True)
class TextureSettings:
    atlas_size: int = 4096
    blend_top_k: int = 3
    min_view_cosine: float = 0.08
    visibility_tolerance_m: float = 0.02
    atlas_gutter_px: int = 6
    min_camera_score: float = 0.001
    # Bound triangles for texturing; retain the full reconstruction separately.
    max_input_triangles: int = 75_000
    # Room-shell normals may face away from the scanners. First-hit ray tests
    # still reject views through walls.
    two_sided_room_surfaces: bool = True
    normalize_photos: bool = True
    normalization_strength: float = 0.65


@dataclass(slots=True)
class TextureCamera:
    camera_id: str
    scan_id: str
    shot: ShotRecord
    image_path: Path
    world_from_camera: np.ndarray
    image_rgb: np.ndarray
    sharpness: float
    profile: CameraProfile
    normalization_gain_rgb: np.ndarray


@dataclass(slots=True)
class TexturingResult:
    obj_path: Path
    mtl_path: Path
    texture_paths: list[Path]
    coverage: float
    camera_contributions: dict[str, int]
    unused_cameras: list[str]
    report_path: Path


def build_texture_cameras(
    scans: list[ScanRecord],
    transforms: dict[str, np.ndarray],
    profile: CameraProfile,
    *,
    normalize_photos: bool = True,
    normalization_strength: float = 0.65,
    azimuth_scales: dict[str, float] | None = None,
) -> list[TextureCamera]:
    cameras: list[TextureCamera] = []
    entries = []
    for scan in scans:
        transform_value = transforms.get(scan.scan_id)
        if transform_value is None and len(scans) == 1:
            transform_value = np.eye(4, dtype=np.float64)
        if transform_value is None:
            raise ValueError(f"Missing optimized transform for {scan.scan_id}")
        world_from_scan = np.asarray(transform_value, dtype=np.float64)
        azimuth_scale = (azimuth_scales or {}).get(scan.scan_id, 1.0)
        for image_shot, pose_shot in image_pose_pairs(scan, profile.image_pose_offset_frames, azimuth_scale):
            entries.append((scan,image_shot,pose_shot,world_from_scan))
    # Decode once for gain estimation and projection.
    decoded = [cv2.imread(str(scan.image_paths[image_shot.index]), cv2.IMREAD_COLOR) for scan, image_shot, _, _ in entries]
    gains = (
        estimate_photo_gains(images=decoded, strength=normalization_strength)
        if normalize_photos
        else [np.ones(3, dtype=np.float32) for _ in decoded]
    )
    for (scan, image_shot, pose_shot, world_from_scan), gain, bgr in zip(entries, gains, decoded, strict=True):
        if bgr is None:
            continue
        try:
            effective_profile = rescale_profile(profile, bgr.shape[1], bgr.shape[0])
        except ValueError:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if normalize_photos:
            rgb = apply_photo_gain(rgb, gain)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        sharpness = float(cv2.Laplacian(gray, cv2.CV_32F).var())
        cameras.append(TextureCamera(
            f"{scan.scan_id}/shot_{image_shot.index:03d}",
            scan.scan_id,
            pose_shot,
            scan.image_paths[image_shot.index],
            world_from_scan @ camera_pose(effective_profile, pose_shot),
            rgb,
            sharpness,
            effective_profile,
            gain,
        ))
    return cameras


def _project_world(points: np.ndarray, camera: TextureCamera, profile: CameraProfile) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    profile=camera.profile
    transform = camera.world_from_camera
    camera_points = (points - transform[:3, 3]) @ transform[:3, :3]
    uv = np.full((len(points), 2), np.nan)
    valid = camera_points[:, 2] > 1e-5
    if np.any(valid):
        projected, finite = project_camera_points(camera_points[valid], profile)
        ids = np.flatnonzero(valid)
        uv[ids] = projected
        valid[ids] &= finite
    valid &= (uv[:, 0] >= 0) & (uv[:, 0] < profile.image_width - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < profile.image_height - 1)
    return uv, camera_points[:, 2], valid


def _visible_centroids(
    scene: o3d.t.geometry.RaycastingScene, centroids: np.ndarray, camera: TextureCamera,
    valid: np.ndarray, tolerance: float,
) -> np.ndarray:
    visible = np.zeros(len(centroids), dtype=bool)
    ids = np.flatnonzero(valid)
    center = camera.world_from_camera[:3, 3]
    for start in range(0, len(ids), 100_000):
        chunk = ids[start : start + 100_000]
        vectors = centroids[chunk] - center
        distances = np.linalg.norm(vectors, axis=1)
        directions = vectors / np.maximum(distances[:, None], 1e-12)
        origins = np.broadcast_to(center, directions.shape)
        rays = o3d.core.Tensor(np.column_stack((origins, directions)).astype(np.float32))
        hits = scene.cast_rays(rays)
        hit_distance = hits["t_hit"].numpy()
        visible[chunk] = np.isfinite(hit_distance) & (np.abs(hit_distance - distances) <= tolerance)
    return visible


def _bilinear(image: np.ndarray, uv: np.ndarray) -> np.ndarray:
    maps_x = uv[:, 0].astype(np.float32).reshape(-1, 1)
    maps_y = uv[:, 1].astype(np.float32).reshape(-1, 1)
    return cv2.remap(image, maps_x, maps_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT).reshape(-1, 3)


def _camera_candidates(
    vertices: np.ndarray, faces: np.ndarray, normals: np.ndarray, cameras: list[TextureCamera],
    profile: CameraProfile, scene: o3d.t.geometry.RaycastingScene, settings: TextureSettings,
    progress: Callable[[float, str], None] | None = None,
    cancelled: Callable[[], bool] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    progress = progress or (lambda _value, _message: None)
    cancelled = cancelled or (lambda: False)
    centroids = vertices[faces].mean(axis=1)
    face_normals = normals[faces].mean(axis=1)
    face_normals /= np.maximum(np.linalg.norm(face_normals, axis=1, keepdims=True), 1e-12)
    k = max(1, min(settings.blend_top_k, 3))
    top_scores = np.full((len(faces), k), -np.inf, dtype=np.float32)
    top_cameras = np.full((len(faces), k), -1, dtype=np.int32)
    sharp_values = np.array([cam.sharpness for cam in cameras])
    sharp_median = max(float(np.median(sharp_values)), 1e-6) if len(cameras) else 1.0
    for camera_index, camera in enumerate(cameras):
        if cancelled():
            raise InterruptedError("Texturing cancelled")
        uv, depth, valid = _project_world(centroids, camera, profile)
        vectors = camera.world_from_camera[:3, 3] - centroids
        distance = np.linalg.norm(vectors, axis=1)
        direction = vectors / np.maximum(distance[:, None], 1e-12)
        signed_cosine = np.sum(face_normals * direction, axis=1)
        cosine = np.abs(signed_cosine) if settings.two_sided_room_surfaces else signed_cosine
        valid &= cosine >= settings.min_view_cosine
        valid &= _visible_centroids(scene, centroids, camera, valid, settings.visibility_tolerance_m)
        camera_profile=camera.profile
        nx = (uv[:, 0] - camera_profile.cx) / max(camera_profile.image_width / 2, 1)
        ny = (uv[:, 1] - camera_profile.cy) / max(camera_profile.image_height / 2, 1)
        center_score = np.clip(1.0 - 0.6 * np.sqrt(nx * nx + ny * ny), 0.0, 1.0) ** 2
        sharp_score = np.clip(np.sqrt(camera.sharpness / sharp_median), 0.5, 1.5)
        scores = np.where(valid, cosine * center_score * sharp_score / np.maximum(distance, 0.1), -np.inf)
        weakest = np.argmin(top_scores, axis=1)
        rows = np.arange(len(faces)); replace = scores > top_scores[rows, weakest]
        top_scores[rows[replace], weakest[replace]] = scores[replace]
        top_cameras[rows[replace], weakest[replace]] = camera_index
        progress(
            (camera_index + 1) / max(len(cameras), 1),
            f"Visibility camera {camera_index + 1}/{len(cameras)}",
        )
    order = np.argsort(top_scores, axis=1)[:, ::-1]
    return np.take_along_axis(top_cameras, order, axis=1), np.take_along_axis(top_scores, order, axis=1)


def _exposure_gains(
    centroids: np.ndarray, candidate_ids: np.ndarray, candidate_scores: np.ndarray,
    cameras: list[TextureCamera], profile: CameraProfile,
) -> np.ndarray:
    equations=[]; rhs=[]
    for face_ids in np.array_split(np.arange(len(centroids)), max(1, len(centroids)//20_000 + 1)):
        for row in face_ids[::10]:
            a, b = candidate_ids[row, :2] if candidate_ids.shape[1] >= 2 else (-1, -1)
            if a < 0 or b < 0 or candidate_scores[row, 1] <= 0: continue
            colors=[]
            for camera_id in (a,b):
                uv,_,valid=_project_world(centroids[row:row+1],cameras[camera_id],profile)
                if not valid[0]: break
                sampled = _bilinear(cameras[camera_id].image_rgb, uv)[0].astype(float)
                if np.any(sampled <= 8.0) or np.any(sampled >= 247.0):
                    break
                color = sampled
                colors.append(color)
            if len(colors)==2:
                eq=np.zeros(len(cameras)); eq[a]=1; eq[b]=-1
                equations.append(eq); rhs.append(np.log(colors[1]/colors[0]))
                if len(equations)>=5000: break
        if len(equations)>=5000: break
    if not equations:return np.ones((len(cameras),3),dtype=np.float32)
    # A weak neutral prior for every camera prevents disconnected overlap
    # groups from drifting to arbitrary exposure/color corrections.
    regularization = 0.08 * np.eye(len(cameras), dtype=np.float64)
    matrix = np.vstack([*equations, regularization])
    target = np.vstack([*rhs, np.zeros((len(cameras), 3), dtype=np.float64)])
    log_gain=np.linalg.lstsq(matrix,target,rcond=None)[0]
    return np.clip(np.exp(log_gain),0.65,1.55).astype(np.float32)


def _barycentric_pixels(triangle_uv: np.ndarray, size: int) -> tuple[np.ndarray, np.ndarray]:
    pixels = triangle_uv * (size - 1)
    xmin=max(0,int(np.floor(pixels[:,0].min()))); xmax=min(size-1,int(np.ceil(pixels[:,0].max())))
    ymin=max(0,int(np.floor(pixels[:,1].min()))); ymax=min(size-1,int(np.ceil(pixels[:,1].max())))
    if xmax<xmin or ymax<ymin: return np.empty((0,2),int),np.empty((0,3))
    yy,xx=np.mgrid[ymin:ymax+1,xmin:xmax+1]; samples=np.column_stack((xx.ravel()+0.5,yy.ravel()+0.5))
    a,b,c=pixels; v0=b-a; v1=c-a; v2=samples-a
    denom=v0[0]*v1[1]-v1[0]*v0[1]
    if abs(denom)<1e-9: return np.empty((0,2),int),np.empty((0,3))
    w1=(v2[:,0]*v1[1]-v1[0]*v2[:,1])/denom; w2=(v0[0]*v2[:,1]-v2[:,0]*v0[1])/denom; w0=1-w1-w2
    bary=np.column_stack((w0,w1,w2)); inside=np.all(bary>=-1e-6,axis=1)
    return np.column_stack((xx.ravel()[inside],yy.ravel()[inside])).astype(int),bary[inside]


def _write_obj(
    path: Path, vertices: np.ndarray, normals: np.ndarray, uvs: np.ndarray,
    faces: np.ndarray, face_pages: np.ndarray, mtl_name: str,
) -> None:
    # Write joined blocks to limit per-line I/O overhead.
    lines=[f"mtllib {mtl_name}\n"]
    lines.append("".join(f"v {v[0]:.9g} {v[1]:.9g} {v[2]:.9g}\n" for v in vertices))
    lines.append("".join(f"vt {uv[0]:.9g} {1.0-uv[1]:.9g}\n" for uv in uvs))
    lines.append("".join(f"vn {n[0]:.9g} {n[1]:.9g} {n[2]:.9g}\n" for n in normals))
    face_lines=[]; active_page=-1
    for f,page in zip(faces+1,face_pages,strict=True):
        if int(page)!=active_page:
            active_page=int(page); face_lines.append(f"usemtl material_{active_page:03d}\n")
        face_lines.append(f"f {f[0]}/{f[0]}/{f[0]} {f[1]}/{f[1]}/{f[1]} {f[2]}/{f[2]}/{f[2]}\n")
    lines.append("".join(face_lines))
    path.write_text("".join(lines),encoding="utf-8",newline="\n")


def texture_mesh(
    mesh_path: str | Path, scans: list[ScanRecord], transforms: dict[str,np.ndarray], profile: CameraProfile,
    output_dir: str | Path, settings: TextureSettings | None = None,
    progress: Callable[[float, str], None] | None = None,
    cancelled: Callable[[], bool] | None = None,
    azimuth_scales: dict[str, float] | None = None,
) -> TexturingResult:
    from ..publication import staged_call
    return staged_call(Path(output_dir), lambda staging: _texture_mesh(
        mesh_path, scans, transforms, profile, staging, settings, progress,
        cancelled, azimuth_scales), cancelled or (lambda: False))


def _texture_mesh(mesh_path, scans, transforms, profile, output_dir, settings, progress, cancelled, azimuth_scales):
    settings=settings or TextureSettings()
    progress=progress or (lambda _value,_message:None)
    cancelled=cancelled or (lambda:False)
    output_dir=Path(output_dir); texture_dir=output_dir/"textures"
    texture_dir.mkdir(parents=True,exist_ok=True)
    if not mesh_path:
        raise ValueError("Generate a mesh before texturing")
    progress(.01,"Loading mesh for texturing")
    mesh=o3d.io.read_triangle_mesh(str(mesh_path)); mesh.compute_vertex_normals()
    source_triangles=len(mesh.triangles)
    if not source_triangles: raise ValueError("Mesh has no triangles")
    if 0 < settings.max_input_triangles < source_triangles:
        progress(.04,f"Preparing {settings.max_input_triangles:,}-triangle texture mesh from {source_triangles:,} triangles")
        mesh=mesh.simplify_quadric_decimation(settings.max_input_triangles,boundary_weight=2.0)
        mesh.remove_degenerate_triangles(); mesh.remove_unreferenced_vertices(); mesh.compute_vertex_normals()
    if cancelled(): raise InterruptedError("Texturing cancelled")
    vertices=np.asarray(mesh.vertices); faces=np.asarray(mesh.triangles,dtype=np.int32); normals=np.asarray(mesh.vertex_normals)
    progress(.08,f"Unwrapping {len(faces):,} triangles")
    atlas=xatlas.Atlas(); atlas.add_mesh(vertices,faces)
    pack=xatlas.PackOptions(); pack.resolution=int(settings.atlas_size); pack.padding=int(settings.atlas_gutter_px)
    atlas.generate(pack_options=pack)
    if cancelled(): raise InterruptedError("Texturing cancelled")
    vmapping,new_faces,uvs=atlas[0]
    vmapping=np.asarray(vmapping,dtype=np.int64); new_faces=np.asarray(new_faces,dtype=np.int32); uvs=np.asarray(uvs,dtype=np.float64)
    vertex_pages,_chart_ids=atlas.get_mesh_vertex_assignment(0); vertex_pages=np.asarray(vertex_pages,dtype=np.int32)
    face_pages=vertex_pages[new_faces[:,0]]
    atlas_vertices=vertices[vmapping]; atlas_normals=normals[vmapping]
    tensor_mesh=o3d.t.geometry.TriangleMesh.from_legacy(mesh); scene=o3d.t.geometry.RaycastingScene(); scene.add_triangles(tensor_mesh)
    progress(.16,"Loading registered photographs")
    cameras=build_texture_cameras(
        scans,transforms,profile,
        normalize_photos=settings.normalize_photos,
        normalization_strength=settings.normalization_strength,
        azimuth_scales=azimuth_scales,
    )
    if not cameras: raise ValueError("No usable texture cameras")
    # Candidates refer to original triangles; xatlas preserves triangle order.
    candidate_ids,candidate_scores=_camera_candidates(
        vertices,faces,normals,cameras,profile,scene,settings,
        lambda value,message:progress(.20+.42*value,message),cancelled,
    )
    if cancelled(): raise InterruptedError("Texturing cancelled")
    progress(.64,"Balancing photograph exposure")
    centroids=vertices[faces].mean(axis=1); gains=_exposure_gains(centroids,candidate_ids,candidate_scores,cameras,profile)
    size=int(settings.atlas_size); page_count=max(1,int(atlas.atlas_count))
    # Non-overlapping atlas charts allow uint8 textures and masks.
    textures=[np.zeros((size,size,3),dtype=np.uint8) for _ in range(page_count)]
    colored_masks=[np.zeros((size,size),dtype=np.uint8) for _ in range(page_count)]
    surface_masks=[np.zeros((size,size),dtype=np.uint8) for _ in range(page_count)]
    contributions={cam.camera_id:0 for cam in cameras}
    for triangle_index, face in enumerate(new_faces):
        if triangle_index % 256 == 0:
            if cancelled(): raise InterruptedError("Texturing cancelled")
            progress(.67+.27*triangle_index/max(len(new_faces),1),f"Painting triangle {triangle_index:,}/{len(new_faces):,}")
        page=int(face_pages[triangle_index]); texture=textures[page]; colored_mask=colored_masks[page]; surface_mask=surface_masks[page]
        pixels,bary=_barycentric_pixels(uvs[face],size)
        if not len(pixels): continue
        surface_mask[pixels[:,1],pixels[:,0]]=1
        world=bary @ atlas_vertices[face]
        accum=np.zeros((len(pixels),3),dtype=np.float32); weights=np.zeros(len(pixels),dtype=np.float32)
        for camera_id,base_score in zip(candidate_ids[triangle_index],candidate_scores[triangle_index],strict=True):
            if camera_id<0 or base_score<settings.min_camera_score: continue
            uv,_,valid=_project_world(world,cameras[camera_id],profile)
            if not np.any(valid): continue
            sampled=_bilinear(cameras[camera_id].image_rgb,uv[valid]).astype(np.float32)*gains[camera_id][None,:]
            camera_profile=cameras[camera_id].profile
            nx=(uv[valid,0]-camera_profile.cx)/(camera_profile.image_width/2); ny=(uv[valid,1]-camera_profile.cy)/(camera_profile.image_height/2)
            feather=np.clip(1-np.sqrt(nx*nx+ny*ny),0.05,1.0); w=base_score*feather
            accum[valid]+=sampled*w[:,None]; weights[valid]+=w
            contributions[cameras[camera_id].camera_id]+=int(np.count_nonzero(valid))
        valid=weights>0
        x,y=pixels[valid].T
        texture[y,x]=np.clip(accum[valid]/weights[valid,None],0,255).astype(np.uint8)
        colored_mask[y,x]=1
    texture_paths=[]
    progress(.95,"Filling atlas gutters and writing textures")
    for page,(texture,colored_mask) in enumerate(zip(textures,colored_masks,strict=True)):
        mask=colored_mask.astype(bool)
        for _ in range(max(0,settings.atlas_gutter_px)):
            expanded=cv2.dilate(mask.astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)
            new=expanded&~mask
            if not np.any(new): break
            dilated=cv2.dilate(texture,np.ones((3,3),np.uint8)); texture[new]=dilated[new]; mask=expanded
        texture_path=texture_dir/f"texture_{page:03d}.png"
        if not cv2.imwrite(str(texture_path),cv2.cvtColor(texture,cv2.COLOR_RGB2BGR)):
            raise OSError(f"Failed to write {texture_path}")
        texture_paths.append(texture_path)
    obj_path=output_dir/"textured_mesh.obj"; mtl_path=output_dir/"textured_mesh.mtl"
    _write_obj(obj_path,atlas_vertices,atlas_normals,uvs,new_faces,face_pages,mtl_path.name)
    mtl_path.write_text("".join(f"newmtl material_{page:03d}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\nillum 1\nmap_Kd textures/texture_{page:03d}.png\n\n" for page in range(page_count)),encoding="utf-8")
    used={key:value for key,value in contributions.items() if value>0}; unused=[key for key,value in contributions.items() if value==0]
    covered=sum(np.count_nonzero(colored.astype(bool)&surface.astype(bool)) for colored,surface in zip(colored_masks,surface_masks,strict=True))
    total=sum(np.count_nonzero(surface) for surface in surface_masks)
    coverage=float(covered/max(total,1))
    report=output_dir/"texturing.report.json"; report.write_text(json.dumps({"coverage":coverage,"geometry_source":"lidar_mesh_only","image_only_regions_ignored":True,"pose_source":"cmd_az/cmd_tilt only","source_triangles":source_triangles,"textured_triangles":len(faces),"atlas_pages":page_count,"camera_contributions":used,"unused_cameras":{camera_id:"no visible, front-facing atlas samples passed scoring" for camera_id in unused},"photo_normalization_gains_rgb":{camera.camera_id:[float(value) for value in camera.normalization_gain_rgb] for camera in cameras},"overlap_correction_gains_rgb":{camera.camera_id:[float(value) for value in gains[index]] for index,camera in enumerate(cameras)},"settings":asdict(settings)},indent=2)+"\n",encoding="utf-8")
    progress(1.0,f"Textured mesh ready: {coverage:.1%} atlas coverage")
    return TexturingResult(obj_path,mtl_path,texture_paths,coverage,used,unused,report)
