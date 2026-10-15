from __future__ import annotations

from pathlib import Path

import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PySide6.QtCore import Signal
from PySide6.QtWidgets import QFrame, QVBoxLayout
from vtkmodules.vtkRenderingCore import vtkPointPicker

from ..io.scan import load_cloud


def _normal_shaded_rgb(data: pv.PolyData, arrays: set[str], base_rgb: tuple[int, int, int]) -> np.ndarray:
    """Shade fallback point colors by their normals; use flat color when normals are absent."""
    base = np.asarray(base_rgb, dtype=np.float64)
    if not {"nx", "ny", "nz"}.issubset(arrays):
        return np.tile(base, (data.n_points, 1)).astype(np.uint8)
    normals = np.column_stack((data["nx"], data["ny"], data["nz"])).astype(np.float64)
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / np.clip(length, 1e-9, None)
    light_dir = np.array([0.45, -0.45, 0.77]); light_dir /= np.linalg.norm(light_dir)
    diffuse = np.clip(normals @ light_dir, 0.0, 1.0)
    # Keep back-facing surfaces visible with ambient light.
    shade = 0.55 + 0.45 * diffuse
    return np.clip(base[None, :] * shade[:, None], 0, 255).astype(np.uint8)


def read_point_cloud(path: str | Path) -> pv.PolyData:
    """Load PLY scalar vertex properties into a PyVista dataset using plyfile."""

    cloud=load_cloud(path,require_raw=False)
    data=pv.PolyData(cloud.xyz)
    for property_name in cloud.property_names:
        if property_name in {"x","y","z"}:continue
        values=np.asarray(cloud.vertex[property_name])
        if values.ndim==1:data.point_data[property_name]=values
    return data


class SceneView(QFrame):
    # Distance in metres, emitted when a second point completes a measurement.
    measured = Signal(float)
    measurement_state_changed = Signal(str)

    _MEASURE_ACTORS = (
        "measure:a", "measure:a-label", "measure:b", "measure:b-label",
        "measure:line", "measure:distance-label",
    )

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0)
        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter.interactor)
        self.plotter.set_background("#151a21", top="#3a4654")
        self.plotter.add_axes()
        self.actors: dict[str, object] = {}
        self.datasets: dict[str, pv.DataSet] = {}
        self.mesh_names: set[str] = set()
        self._wireframe = False
        self._mesh_lighting = True
        self._camera_initialized = False
        self._cloud_cache: dict[Path, tuple[float, pv.PolyData]] = {}
        self._measuring = False
        self._measure_points: list[np.ndarray] = []
        self._measure_sources: list[str] = []
        self._measure_press_position: tuple[int, int] | None = None
        self._measure_picker = vtkPointPicker()
        self._measure_picker.SetTolerance(0.025)
        self._configure_lighting()
        # Keep observers installed; handlers act only in measurement mode.
        self.plotter.iren.add_observer("LeftButtonPressEvent", self._on_left_press)
        self.plotter.iren.add_observer("LeftButtonReleaseEvent", self._on_left_release)

    def _configure_lighting(self) -> None:
        """Install a neutral studio rig suitable for room-scale geometry."""

        self.plotter.renderer.remove_all_lights()
        lights=(
            pv.Light(position=(1.0,-1.0,2.0),focal_point=(0.0,0.0,0.0),color="#fff4df",intensity=0.90,light_type="scene light",shadow_attenuation=0.35),
            pv.Light(position=(-1.0,-0.2,0.8),focal_point=(0.0,0.0,0.0),color="#d7e7ff",intensity=0.42,light_type="scene light",shadow_attenuation=0.55),
            pv.Light(position=(0.2,1.0,1.4),focal_point=(0.0,0.0,0.0),color="#ffffff",intensity=0.30,light_type="scene light",shadow_attenuation=0.65),
        )
        for light in lights:self.plotter.add_light(light)
        try:self.plotter.enable_anti_aliasing("fxaa")
        except RuntimeError:pass
        # Enable mesh effects when a mesh is added; skip them for point-only scenes.
        self._set_shadow_effects(False)

    def _set_shadow_effects(self,enabled: bool) -> None:
        try:
            if enabled:
                self.plotter.enable_shadows()
                self.plotter.enable_ssao(radius=0.25,bias=0.01,kernel_size=128,blur=True)
            else:
                self.plotter.disable_shadows()
                self.plotter.disable_ssao()
        except (RuntimeError,AttributeError):
            # Some remote OpenGL implementations lack shadow/SSAO framebuffer support.
            pass

    def clear(self) -> None:
        self._clear_measurement(emit_state=False)
        self.plotter.clear(); self.plotter.add_axes()
        self.actors.clear(); self.datasets.clear(); self.mesh_names.clear()
        self._camera_initialized = False
        self._configure_lighting()
        self.measurement_state_changed.emit("first" if self._measuring else "inactive")

    def remove(self, name: str) -> None:
        if name in self._measure_sources:
            self._clear_measurement()
        actor = self.actors.pop(name, None)
        if actor is not None:
            self.plotter.remove_actor(actor)
        self.datasets.pop(name, None)
        self.mesh_names.discard(name)

    def _maybe_reset_camera(self) -> None:
        """Frame the first content added after clear().

        Later layer changes preserve the view; fit_view() resets it explicitly.
        """
        if not self._camera_initialized:
            self.plotter.reset_camera(); self._camera_initialized = True

    def fit_view(self) -> None:
        self.plotter.reset_camera()

    def _load_cached(self, path: str | Path) -> pv.PolyData:
        path = Path(path)
        mtime = path.stat().st_mtime
        cached = self._cloud_cache.get(path)
        if cached is not None and cached[0] == mtime:
            return cached[1].copy()
        data = read_point_cloud(path)
        self._cloud_cache[path] = (mtime, data)
        return data.copy()

    def add_cloud(
        self,
        path: str | Path,
        name: str,
        color: str | tuple | None = None,
        transform: np.ndarray | None = None,
        scalar: str | None = None,
        cmap: str = "turbo",
    ) -> None:
        self.remove(name)
        data = self._load_cached(path)
        if transform is not None:
            data = data.transform(np.asarray(transform), inplace=False)
        # Render points as spheres so uniform clouds retain depth cues.
        kwargs = {"point_size": 3, "render_points_as_spheres": True}
        arrays = set(data.point_data.keys())
        if color is not None:
            kwargs["color"] = color
        elif scalar is not None and scalar in arrays:
            kwargs.update(scalars=scalar, cmap=cmap, show_scalar_bar=False)
        elif "RGB" in arrays:
            kwargs.update(scalars="RGB", rgb=True)
        elif {"red", "green", "blue"}.issubset(arrays):
            rgb = np.column_stack((data["red"], data["green"], data["blue"])).astype(np.uint8)
            if "colored" in arrays:
                uncolored = np.asarray(data["colored"]) == 0
                if uncolored.any():
                    rgb[uncolored] = _normal_shaded_rgb(data, arrays, (216, 221, 229))[uncolored]
            data.point_data["RGB"] = rgb; kwargs.update(scalars="RGB", rgb=True)
        elif "intensity" in arrays:
            kwargs.update(scalars="intensity", cmap="gray")
        else:
            data.point_data["RGB"] = _normal_shaded_rgb(data, arrays, (216, 221, 229))
            kwargs.update(scalars="RGB", rgb=True)
        self.datasets[name] = data
        self.actors[name] = self.plotter.add_mesh(data, name=name, **kwargs)
        self._maybe_reset_camera()

    def add_mesh(self, path: str | Path, name: str, texture_path: str | Path | None = None) -> None:
        self.remove(name)
        data = pv.read(str(path))
        texture = pv.read_texture(str(texture_path)) if texture_path else None
        self.datasets[name] = data
        was_empty = not self.mesh_names
        # Use the interpolation mode that preserves base-color textures in VTK.
        self.actors[name] = self.plotter.add_mesh(
            data, name=name, texture=texture, color=None if texture else "#c9ced6",
            show_edges=self._wireframe, smooth_shading=True, split_sharp_edges=True,
            lighting=self._mesh_lighting, ambient=0.18, diffuse=0.82,
            specular=0.08 if texture else 0.16, specular_power=18.0,
            pbr=texture is None, metallic=0.0, roughness=0.72,
        )
        self.mesh_names.add(name)
        if was_empty:self._set_shadow_effects(self._mesh_lighting)
        self._maybe_reset_camera()

    def set_mesh_lighting(self,enabled: bool) -> None:
        self._mesh_lighting=enabled
        self._set_shadow_effects(enabled)
        for name in self.mesh_names:
            actor=self.actors.get(name)
            if actor is not None:actor.GetProperty().SetLighting(enabled)
        self.plotter.render()

    def set_visible(self, name: str, visible: bool) -> None:
        actor = self.actors.get(name)
        if actor is not None:
            if not visible and name in self._measure_sources:
                self._clear_measurement()
            actor.SetVisibility(visible)
            self.plotter.render()

    def set_wireframe(self, enabled: bool) -> None:
        self._wireframe = enabled
        for name, actor in self.actors.items():
            if name.startswith("measure:"):
                continue
            prop = actor.GetProperty()
            prop.SetRepresentationToWireframe() if enabled else prop.SetRepresentationToSurface()
        self.plotter.render()

    def show_normals(self, name: str, enabled: bool) -> None:
        normal_name = f"{name}:normals"
        self.remove(normal_name)
        if not enabled or name not in self.datasets:
            return
        data = self.datasets[name]
        step = max(1, data.n_points // 5000)
        sample = data.extract_points(np.arange(0, data.n_points, step), adjacent_cells=False)
        if "Normals" not in sample.point_data:
            sample = sample.compute_normals(point_normals=True, cell_normals=False)
        arrows = sample.glyph(orient="Normals", scale=False, factor=max(data.length * 0.01, 0.01))
        self.datasets[normal_name] = arrows
        self.actors[normal_name] = self.plotter.add_mesh(arrows, color="#ffc857", pickable=False)

    def set_measuring(self, enabled: bool) -> None:
        """Toggle two-point distance picking, snapping to existing geometry.
        A third click clears the previous pair.
        """
        self._measuring = enabled
        self._clear_measurement()

    def _clear_measurement(self, emit_state: bool = True) -> None:
        # Remove directly rather than through remove(), since remove() also
        # watches source layers and would make this operation recursive.
        for name in self._MEASURE_ACTORS:
            actor = self.actors.pop(name, None)
            if actor is not None:
                self.plotter.remove_actor(actor, render=False)
            self.datasets.pop(name, None)
        self._measure_points.clear()
        self._measure_sources.clear()
        if emit_state:
            self.measurement_state_changed.emit("first" if self._measuring else "inactive")

    def _on_left_press(self, *_args) -> None:
        if self._measuring:
            self._measure_press_position = tuple(self.plotter.iren.get_event_position())

    def _on_left_release(self, *_args) -> None:
        if not self._measuring or self._measure_press_position is None:
            return
        release_position = tuple(self.plotter.iren.get_event_position())
        press_position = self._measure_press_position
        self._measure_press_position = None
        # A camera orbit starts with the same button. Only a stationary click
        # is a measurement selection.
        if np.linalg.norm(np.subtract(release_position, press_position)) > 4.0:
            return
        x, y = release_position
        if not self._measure_picker.Pick(x, y, 0.0, self.plotter.renderer):
            return
        picked_actor = self._measure_picker.GetActor()
        point_id = self._measure_picker.GetPointId()
        source = self._pickable_layer_name(picked_actor)
        if source is None or point_id < 0:
            return
        dataset = self._measure_picker.GetDataSet()
        if dataset is None or point_id >= dataset.GetNumberOfPoints():
            return
        self._accept_measurement_point(dataset.GetPoint(point_id), source)

    def _pickable_layer_name(self, picked_actor) -> str | None:
        for name, actor in self.actors.items():
            if name.startswith("measure:") or name.endswith(":normals"):
                continue
            if actor is picked_actor and actor.GetVisibility():
                return name
        return None

    def _accept_measurement_point(self, point, source: str) -> None:
        point = np.asarray(point, dtype=np.float64)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            return
        if len(self._measure_points) >= 2:
            self._clear_measurement()
        self._measure_points.append(point)
        self._measure_sources.append(source)
        label = "measure:a" if len(self._measure_points) == 1 else "measure:b"
        self.actors[label] = self.plotter.add_points(
            self._measure_points[-1].reshape(1, 3), color="#ffca3a", point_size=16, render_points_as_spheres=True,
            pickable=False, reset_camera=False,
        )
        self.actors[f"{label}-label"] = self.plotter.add_point_labels(
            self._measure_points[-1].reshape(1, 3), ["A" if len(self._measure_points) == 1 else "B"],
            name=f"{label}-label", show_points=False, text_color="#ffca3a", shape_color="#20252d",
            font_size=14, pickable=False, always_visible=True, reset_camera=False, render=False,
        )
        if len(self._measure_points) == 2:
            a, b = self._measure_points
            distance_m = float(np.linalg.norm(b - a))
            line = pv.Line(a, b)
            self.datasets["measure:line"] = line
            self.actors["measure:line"] = self.plotter.add_mesh(
                line, color="#ffca3a", line_width=3, pickable=False, reset_camera=False,
            )
            midpoint = ((a + b) / 2.0).reshape(1, 3)
            distance_text = f"{distance_m:.3f} m ({distance_m * 100.0:.1f} cm)"
            self.actors["measure:distance-label"] = self.plotter.add_point_labels(
                midpoint, [distance_text], name="measure:distance-label", show_points=False,
                text_color="white", shape_color="#20252d", font_size=13, pickable=False,
                always_visible=True, reset_camera=False, render=False,
            )
            self.measurement_state_changed.emit("complete")
            self.measured.emit(distance_m)
        else:
            self.measurement_state_changed.emit("second")
        self.plotter.render()
