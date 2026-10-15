from __future__ import annotations

import json
import shutil
import traceback
from uuid import uuid4
from dataclasses import asdict
from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QIcon, QKeySequence, QPixmap, QShortcut
from PySide6.QtWidgets import (
    QDialog, QFileDialog, QFrame, QGroupBox, QHBoxLayout,
    QHeaderView, QInputDialog, QLabel, QLineEdit, QMainWindow, QMessageBox, QProgressBar,
    QPushButton, QScrollArea, QSplitter,
    QTextEdit, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget,
)

from ..calibration.profile import load_profile, save_profile
from ..colorization import ColorizationSettings, colorize_scan
from ..fingerprint import compute_stage_fingerprint
from ..fusion import FusionSettings, fuse_scans
from ..io.project import load_project, save_project
from ..io.scan import discover_scans, import_scan_folder
from ..jobs import JobQueue
from ..publication import export_outputs, export_mesh_asset
from ..leveling import MODEL_VERSION, LevelingSettings, level_scans
from ..meshing import MeshingSettings, generate_mesh
from ..models import CameraProfile, ProjectState, StageStatus
from ..registration import (
    RegistrationSettings, optimize_registration_graph, register_scans,
)
from ..texturing import TextureSettings, texture_mesh
from .calibration_dialog import CalibrationDialog
from .scene import SceneView

_RESOURCES_DIR = Path(__file__).resolve().parent.parent / "resources"
_APP_ICON_PATH = _RESOURCES_DIR / "app_icon.svg"
_LOGO_PATH = _RESOURCES_DIR / "freeuni_logo.png"

def _default_profile() -> CameraProfile:
    path=_RESOURCES_DIR/"camera_profile.json"
    return load_profile(path) if path.exists() else CameraProfile()


def _default_processing_settings() -> dict:
    return {
        "leveling": asdict(LevelingSettings()),
        "colorization": asdict(ColorizationSettings()),
        "registration": {**asdict(RegistrationSettings()), "pair_mode": "auto"},
        "room_mode": False,
        "fusion": asdict(FusionSettings()),
        "mesh": asdict(MeshingSettings(smooth_iterations=6)),
        "texture": asdict(TextureSettings(atlas_size=2048, blend_top_k=2)),
    }

_STYLESHEET = """
QGroupBox { font-weight: 600; border: 1px solid palette(mid); border-radius: 6px; margin-top: 16px; padding-top: 8px; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QPushButton { padding: 5px 12px; border: 1px solid palette(mid); border-radius: 4px; background-color: palette(button); color: palette(button-text); }
QPushButton:hover { background-color: palette(light); }
QPushButton:pressed { background-color: palette(mid); }
QPushButton:checked { background-color: #3b82f6; color: white; border-color: #2563eb; }
QPushButton:disabled { color: palette(mid); border-color: palette(midlight); }
QPushButton#toolbtn { padding: 3px 8px; font-size: 11px; }
QLabel#hint { font-style: italic; color: palette(dark); }
QLabel#badge { font-size: 11px; min-width: 78px; }
QLineEdit { padding: 4px 6px; border-radius: 4px; }
QTreeWidget { border: 1px solid palette(mid); border-radius: 4px; }
QTreeWidget::item { padding: 2px 0; }
QSplitter::handle { background: palette(mid); }
QSplitter::handle:horizontal { width: 3px; }
QProgressBar { border: 1px solid palette(mid); border-radius: 4px; min-height: 20px; text-align: center; background: palette(base); }
QProgressBar::chunk { background-color: #3b82f6; border-radius: 3px; }
QFrame#logoFrame { border: 1px solid palette(mid); border-radius: 6px; background: palette(base); }
"""

# Processing stage represented by each step badge.
_STEP_STAGE_MAP = {
    "import": "import", "leveling": "leveling", "calibration": "calibration", "colorization": "colorization",
    "optimize": "registration", "fusion": "fusion", "mesh": "mesh", "texture": "texture",
}
_STAGE_BADGES = {
    StageStatus.MISSING: ("Not started", "#8b93a1"), StageStatus.READY: ("Ready", "#3b82f6"),
    StageStatus.RUNNING: ("Running…", "#f59e0b"), StageStatus.COMPLETE: ("Done", "#1f9d55"),
    StageStatus.STALE: ("Needs redo", "#f59e0b"), StageStatus.FAILED: ("Failed", "#dc2626"),
}
# Reject colorization below this coverage fraction.
_LOW_COVERAGE_THRESHOLD = 0.05


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__(); self.setWindowTitle("UnfoLDAR Desktop"); self.resize(1280,720)
        if _APP_ICON_PATH.exists(): self.setWindowIcon(QIcon(str(_APP_ICON_PATH)))
        self.project=ProjectState(); self.scans=[]; self.profile=_default_profile(); self.job_queue=JobQueue(); self._result_handler=None
        self._tree_syncing=False; self._hidden_scans:set[str]=set(); self._scan_layer_preferences:dict[str,set[str]]={}
        self._combined_hidden=False; self._combined_layer_preferences:set[str]=set()
        self._closing=False
        self._active_stage=None
        self._use_default_settings()
        self._build_ui(); self._connect_jobs(); self._refresh_tree(); self._update_actions()

    @property
    def workspace(self) -> Path | None:
        return self.project.project_file.parent if self.project.project_file else None

    def _build_ui(self) -> None:
        for text,callback,shortcut,attr in [
            ("New project…",self.new_project,"Ctrl+N","new_project_action"),
            ("Open project…",self.open_project,"Ctrl+O","open_project_action"),
            ("Save",self.save,"Ctrl+S",None),
            ("Import scan folder…",self.import_scans,"Ctrl+I","import_action"),
            ("Export results…",self.export_results,None,"export_action")]:
            action=QAction(text,self); action.triggered.connect(callback)
            if shortcut: action.setShortcut(shortcut)
            if attr: setattr(self,attr,action)
            self.addAction(action)
        self.setStyleSheet(_STYLESHEET)
        self.statusBar().showMessage("Ready")
        central=QWidget(); root=QVBoxLayout(central); root.setContentsMargins(6,6,6,6); root.setSpacing(6); self.setCentralWidget(central)
        split=QSplitter(Qt.Orientation.Horizontal); split.setChildrenCollapsible(False); root.addWidget(split,1)
        split.addWidget(self._build_layer_panel())
        self.scene=SceneView(); split.addWidget(self.scene)
        self._measure_escape=QShortcut(QKeySequence(Qt.Key.Key_Escape),self)
        self._measure_escape.activated.connect(self._cancel_measurement)
        self.pipeline_panel=self._build_pipeline_panel(); split.addWidget(self.pipeline_panel)
        split.setSizes([320,860,360]); split.setStretchFactor(0,0); split.setStretchFactor(1,1); split.setStretchFactor(2,0)
        bottom=QHBoxLayout(); root.addLayout(bottom)
        self.progress=QProgressBar(); self.progress.setRange(0,1000); self.progress.setFormat("%p%"); bottom.addWidget(self.progress,1)
        self.cancel=QPushButton("Cancel"); self.cancel.setToolTip("Native calculations finish their current step before cancellation takes effect"); self.cancel.clicked.connect(self._cancel_job); self.cancel.setEnabled(False); bottom.addWidget(self.cancel)

    def _build_layer_panel(self) -> QWidget:
        panel=QWidget(); layout=QVBoxLayout(panel); layout.setContentsMargins(4,4,4,4); layout.setSpacing(6)
        if _LOGO_PATH.exists():
            logo_frame=QFrame(); logo_frame.setObjectName("logoFrame")
            logo_frame.setFrameShape(QFrame.Shape.StyledPanel); logo_frame.setFrameShadow(QFrame.Shadow.Raised)
            logo_layout=QVBoxLayout(logo_frame); logo_layout.setContentsMargins(8,8,8,8)
            logo_label=QLabel(); pixmap=QPixmap(str(_LOGO_PATH))
            if not pixmap.isNull():
                logo_label.setPixmap(pixmap.scaledToWidth(160,Qt.TransformationMode.SmoothTransformation))
            logo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            logo_layout.addWidget(logo_label)
            layout.addWidget(logo_frame)
        title=QLabel("Scene layers"); title.setStyleSheet("font-weight:600; font-size:13px;"); layout.addWidget(title)
        self.layer_filter=QLineEdit(); self.layer_filter.setPlaceholderText("Filter scans by name…"); self.layer_filter.setClearButtonEnabled(True)
        self.layer_filter.textChanged.connect(self._filter_tree); layout.addWidget(self.layer_filter)
        self.layer_filter.setVisible(False)
        toolbar=QHBoxLayout(); toolbar.setSpacing(4)
        self._layer_toolbar_buttons=[]
        for text,callback,tooltip in [
            ("Expand all",self.tree_expand_all,"Expand every group in the tree"),
            ("Collapse all",self.tree_collapse_all,"Collapse every group in the tree"),
            ("Show all",lambda:self._set_all_visibility(True),"Show every scan and the combined result"),
            ("Hide all",lambda:self._set_all_visibility(False),"Hide every scan and the combined result"),
            ("Fit view",lambda:self.scene.fit_view(),"Re-frame the camera on everything currently visible")]:
            button=QPushButton(text); button.setObjectName("toolbtn"); button.setToolTip(tooltip); button.clicked.connect(callback); toolbar.addWidget(button)
            self._layer_toolbar_buttons.append(button)
        layout.addLayout(toolbar)
        for button in self._layer_toolbar_buttons: button.setVisible(False)
        self.tree=QTreeWidget(); self.tree.setHeaderLabels(["Layer","Visibility"]); self.tree.setAlternatingRowColors(True)
        self.tree.setMinimumWidth(210); self.tree.setUniformRowHeights(True); self.tree.setIndentation(14)
        self.tree.header().setStretchLastSection(False)
        self.tree.header().setSectionResizeMode(0,QHeaderView.ResizeMode.Stretch)
        self.tree.header().setSectionResizeMode(1,QHeaderView.ResizeMode.ResizeToContents)
        self.tree.itemDoubleClicked.connect(self._display_item); self.tree.itemChanged.connect(self._tree_item_changed)
        layout.addWidget(self.tree,1)
        return panel

    def tree_expand_all(self) -> None: self.tree.expandAll()
    def tree_collapse_all(self) -> None: self.tree.collapseAll()

    def _set_all_visibility(self, visible: bool) -> None:
        """Apply all layer visibility changes, then rebuild the tree once.

        Synchronous itemChanged handlers can destroy tree items during iteration.
        """
        for scan in self.scans:
            self._set_scan_group_visible(scan.scan_id,visible)
        if len(self.scans)>1:
            self._set_combined_group_visible(visible)
        self._refresh_tree()

    def _filter_tree(self, text: str) -> None:
        needle=text.strip().lower()
        for i in range(self.tree.topLevelItemCount()):
            top=self.tree.topLevelItem(i)
            if top.text(0)!="Source scans":continue
            for j in range(top.childCount()):
                item=top.child(j); item.setHidden(bool(needle) and needle not in item.text(0).lower())


    def _build_pipeline_panel(self) -> QWidget:
        page=QWidget(); layout=QVBoxLayout(page); layout.setContentsMargins(8,8,8,8); layout.setSpacing(8)
        steps_box=QGroupBox("Steps"); steps_layout=QVBoxLayout(steps_box); steps_layout.setSpacing(4)
        self.buttons={}; self.step_status_labels={}
        for key,label,callback in [
            ("import","1. Import scans",self.import_scans),("leveling","2. Level scans",self.level),
            ("calibration","3. Calibrate camera",self.calibrate),
            ("colorization","4. Colorize scans",self.colorize),("registration","5. Register pairs",self.register),
            ("optimize","6. Optimize pose graph",self.optimize),
            ("fusion","7. Fuse / clean",self.fuse),("mesh","8. Generate mesh",self.mesh),
            ("export_mesh","Export mesh…",self.export_mesh),("texture","9. Texture mesh",self.texture),
            ("export","10. Export all results…",self.export_results)]:
            row=QHBoxLayout(); button=QPushButton(label); button.clicked.connect(callback); row.addWidget(button,1); self.buttons[key]=button
            if key in _STEP_STAGE_MAP:
                badge=QLabel(); badge.setObjectName("badge"); badge.setAlignment(Qt.AlignmentFlag.AlignCenter); row.addWidget(badge); self.step_status_labels[key]=badge
            steps_layout.addLayout(row)
        layout.addWidget(steps_box)
        measure_box=QGroupBox("Measurement"); measure_layout=QVBoxLayout(measure_box)
        self.measure_mode=QPushButton("Measure distance"); self.measure_mode.setCheckable(True)
        self.measure_mode.setToolTip("Select two scan points to measure their straight-line 3D distance. Press Escape to exit.")
        self.measure_label=QLabel("Measurement off"); self.measure_label.setObjectName("hint"); self.measure_label.setWordWrap(True)
        measure_layout.addWidget(self.measure_mode); measure_layout.addWidget(self.measure_label)
        self.measure_mode.toggled.connect(self._toggle_measuring)
        self.scene.measured.connect(self._on_measured)
        self.scene.measurement_state_changed.connect(self._on_measurement_state_changed)
        layout.addWidget(measure_box)
        layout.addStretch()
        scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(page)
        return scroll


    def _connect_jobs(self) -> None:
        self.job_queue.progress.connect(lambda value,message:(self.progress.setValue(int(value*1000)),self._show_status(message)))
        self.job_queue.busy_changed.connect(self._busy_changed)
        self.job_queue.error.connect(self._job_error)
        self.job_queue.result.connect(self._job_result)

    def _show_status(self, message: str) -> None:
        self.statusBar().showMessage(message, 8000)
    def _busy_changed(self,busy: bool) -> None:
        self.cancel.setEnabled(busy); self.pipeline_panel.setEnabled(not busy); self._update_actions()
        if not busy and self._closing: QTimer.singleShot(0,self.close)
    def _cancel_job(self) -> None:
        self.job_queue.cancel()
        self.statusBar().showMessage("Cancellation requested — waiting for the current native calculation to finish")
        self.cancel.setEnabled(False)
    def closeEvent(self,event) -> None:
        if self.job_queue.current is not None:
            self._closing=True; self._cancel_job(); event.ignore(); return
        self.save(); event.accept()
    def _job_error(self,message: str) -> None:
        self._result_handler=None; self._show_status("Operation cancelled" if message=="Operation cancelled" else "Processing failed")
        if self._active_stage: self.project.stage_status[self._active_stage]=StageStatus.FAILED
        self._active_stage=None; self.save()
        if not self._closing and message!="Operation cancelled": QMessageBox.critical(self,"Processing failed",message[-4000:])
    def _job_result(self,result) -> None:
        handler=self._result_handler; self._result_handler=None
        if handler is None:return
        try:
            if handler: handler(result)
        except Exception:
            self._job_error(traceback.format_exc()); return
        self.progress.setValue(1000); self.save(); self._refresh_tree(); self._update_actions()
        self._active_stage=None
    def _run(self,operation,handler,stage=None) -> None:
        if self.job_queue.current is not None:
            QMessageBox.information(self,"Busy","Wait for the current operation to finish."); return
        self._check_fingerprints()
        self._active_stage=stage
        if stage:self.project.stage_status[stage]=StageStatus.RUNNING
        self._result_handler=handler
        if not self.job_queue.start(operation): self._result_handler=None

    def _ensure_project(self) -> bool:
        if self.workspace: return True
        self.new_project(); return self.workspace is not None

    def new_project(self) -> None:
        folder=QFileDialog.getExistingDirectory(self,"Choose an empty project workspace")
        if not folder: return
        path=Path(folder)/"project.unfoldar.json"
        if path.exists() and QMessageBox.question(self,"Replace project?",f"{path} exists. Replace its project manifest?")!=QMessageBox.StandardButton.Yes: return
        self._cancel_measurement()
        self.project=ProjectState(name=Path(folder).name,project_file=path); self.scans=[]; self.profile=_default_profile()
        self._use_default_settings()
        self._hidden_scans.clear(); self._scan_layer_preferences.clear(); self._combined_hidden=False; self._combined_layer_preferences.clear()
        (Path(folder)/"outputs").mkdir(exist_ok=True); (Path(folder)/"cache").mkdir(exist_ok=True)
        self.project.calibration_profile_path=Path(folder)/"camera_profile.json"; save_profile(self.profile,self.project.calibration_profile_path); self.save(); self.scene.clear(); self._refresh_tree(); self._update_actions()

    def open_project(self) -> None:
        path,_=QFileDialog.getOpenFileName(self,"Open project","","UnfoLDAR project (*.unfoldar.json);;JSON (*.json)")
        if not path:return
        try:
            self._cancel_measurement()
            loaded=load_project(path)
            self.profile=CameraProfile()
            if loaded.calibration_profile_path and loaded.calibration_profile_path.exists(): self.profile=load_profile(loaded.calibration_profile_path)
            elif loaded.calibration_profile_path:
                loaded.stage_status["calibration"]=StageStatus.MISSING
                self._show_status(f"Missing camera profile: {loaded.calibration_profile_path}; calibration reset")
            self.project=loaded; self.scans=[]
            self._use_default_settings()
            for warning in loaded.migration_warnings: self._show_status(warning)
            self.scene.clear()
            self._hidden_scans.clear(); self._scan_layer_preferences.clear(); self._combined_hidden=False; self._combined_layer_preferences.clear()
            for scan_path in self.project.scan_paths:
                try:
                    scan=import_scan_folder(scan_path)
                    scan.scan_id=self.project.scan_ids[str(scan_path.resolve())]
                    self.scans.append(scan)
                except Exception as exc:self._show_status(f"Skipped {scan_path}: {exc}")
            if len(self.scans)==1:self.project.transforms[self.scans[0].scan_id]=np.eye(4)
            self._check_fingerprints()
            if self.scans:
                first=self.scans[0]
                for scan in self.scans:self._scan_layer_preferences[scan.scan_id]={f"{scan.scan_id}:white"}
                self._hidden_scans.update(scan.scan_id for scan in self.scans[1:])
                self.scene.add_cloud(first.cloud_path,f"{first.scan_id}:white",color="#d8dde5",transform=self._display_transform(first.scan_id))
                self._restore_stitched_result()
            self._refresh_tree(); self._update_actions()
        except Exception as exc: QMessageBox.critical(self,"Cannot open project",str(exc))

    def save(self) -> None:
        if self.project.project_file:
            try: save_project(self.project)
            except Exception as exc: QMessageBox.critical(self,"Cannot save project",str(exc))

    def import_scans(self) -> None:
        if not self._ensure_project(): return
        folder=QFileDialog.getExistingDirectory(self,"Choose a scan folder or a folder containing scans")
        if not folder:return
        accepted,rejected=discover_scans(folder); known={scan.source_folder for scan in self.scans}
        added=[scan for scan in accepted if scan.source_folder not in known]
        for scan in added:
            scan.scan_id="scan-"+uuid4().hex[:16]
            self.project.scan_ids[str(scan.source_folder.resolve())]=scan.scan_id
        self.scans.extend(added)
        self.project.scan_paths=[scan.source_folder for scan in self.scans]
        if self.scans and not self.project.reference_scan_id:self.project.reference_scan_id=self.scans[0].scan_id
        if len(self.scans)==1:self.project.transforms[self.scans[0].scan_id]=np.eye(4)
        self.project.stage_status["import"]=StageStatus.COMPLETE if self.scans else StageStatus.FAILED
        if added:
            for stage in ("leveling","colorization","registration","fusion","mesh","texture"):
                if self.project.stage_status.get(stage)!=StageStatus.MISSING:self.project.stage_status[stage]=StageStatus.STALE
                self.project.artifact_fingerprints.pop(stage,None)
        for path,reason in rejected.items():self._show_status(f"Skipped {path.name}: {reason}")
        self._show_status(f"Imported {len(accepted)} raw scan(s); skipped {len(rejected)} invalid/non-raw folder(s).")
        self.save(); self._refresh_tree(); self._update_actions()
        if added:
            for extra in added:self._scan_layer_preferences.setdefault(extra.scan_id,{f"{extra.scan_id}:white"})
            scan=added[0]; self._hidden_scans.update(extra.scan_id for extra in added[1:])
            self.scene.add_cloud(scan.cloud_path,f"{scan.scan_id}:white",color="#d8dde5",transform=self._display_transform(scan.scan_id))
            self._refresh_tree()

    def level(self) -> None:
        if not self.workspace:return
        scans=list(self.scans); base=self.workspace/"outputs"/"scans"
        settings=LevelingSettings(**_default_processing_settings()["leveling"])
        self.project.settings["leveling"]=asdict(settings)
        def operation(progress,cancelled):return level_scans(scans,settings,progress,cancelled,output_dir=base)
        def done(results):
            summary=[]
            for result in results:
                summary.append({
                    "scan_id":result.scan_id,"applied":result.applied,"skipped_reason":result.skipped_reason,
                    "nx":result.nx,"ny":result.ny,"theta0_deg":result.theta0_deg,"theta_scale":result.theta_scale,
                    "tilt_deg":result.tilt_deg,"tilt_azimuth_deg":result.tilt_azimuth_deg,
                    "sensor_offset_x_m":result.sensor_offset_x_m,"sensor_offset_y_m":result.sensor_offset_y_m,
                    "cost_before_mm":result.cost_before_mm,
                    "cost_after_mm":result.cost_after_mm,"validation_p90_before_mm":result.validation_p90_before_mm,
                    "validation_p90_after_mm":result.validation_p90_after_mm,"overlap_before":result.overlap_before,
                    "overlap_after":result.overlap_after,"condition_number":result.condition_number,
                })
            report=self.workspace/"outputs"/"leveling.report.json"
            report.write_text(json.dumps({"model_version":MODEL_VERSION,"scans":summary},indent=2)+"\n",encoding="utf-8")
            self.project.artifacts["leveling_report"]=str(report)
            self.project.stage_status["leveling"]=StageStatus.COMPLETE
            # Geometry and azimuth scale changes invalidate camera calibration and later stages.
            self._stale_after("leveling")
            self.project.artifact_fingerprints["leveling"]=self._stage_fingerprint("leveling")
            self.save(); self._refresh_tree(); self._update_actions()
        self._run(operation,done,"leveling")

    def calibrate(self) -> None:
        if not self.scans:return
        try:
            photo_scans=[scan for scan in self.scans if scan.image_paths]
            if not photo_scans:return
            source=photo_scans[0]
            if len(photo_scans)>1:
                scan_ids=[scan.scan_id for scan in photo_scans]
                default=self.project.reference_scan_id if self.project.reference_scan_id in scan_ids else scan_ids[0]
                scan_id,ok=QInputDialog.getItem(self,"Choose source scan","Use test images from scan:",scan_ids,scan_ids.index(default),False)
                if not ok:return
                source=next(scan for scan in self.scans if scan.scan_id==scan_id)
            pose_correction=self._leveling_pose_corrections().get(source.scan_id,{})
            dialog=CalibrationDialog(
                source,self.profile,self,
                leveled_path=self._leveled_paths().get(source.scan_id),
                azimuth_scale=float(pose_correction.get("theta_scale",1.0)),
                pan_axis=pose_correction.get("pan_axis"),
            )
            if dialog.exec()==QDialog.DialogCode.Accepted:
                self.profile=dialog.profile
                if self.project.calibration_profile_path:save_profile(self.profile,self.project.calibration_profile_path)
                report=self.workspace/"outputs"/"calibration.report.json"; report.parent.mkdir(parents=True,exist_ok=True)
                report.write_text(json.dumps({"profile":self.profile.to_dict(),"reference_scan":source.scan_id,"pose_source":"cmd_az/cmd_tilt with accepted leveling axis/scale","leveling_pose_correction":pose_correction,"lens_metadata_75mm_used":False},indent=2)+"\n",encoding="utf-8")
                self.project.artifacts["calibration_report"]=str(report)
                self.project.stage_status["calibration"]=StageStatus.COMPLETE; self._stale_after("calibration"); self.project.artifact_fingerprints["calibration"]=self._stage_fingerprint("calibration"); self.save(); self._refresh_tree(); self._update_actions()
        except Exception as exc:
            QMessageBox.critical(self,"Cannot calibrate",str(exc))

    def _colored_paths(self) -> dict[str,Path]:
        if not self.workspace:return {}
        if self.project.stage_status.get("colorization") != StageStatus.COMPLETE:return {}
        return {scan.scan_id:self.workspace/"outputs"/"scans"/scan.scan_id/"colored_cloud.ply" for scan in self.scans if (self.workspace/"outputs"/"scans"/scan.scan_id/"colored_cloud.ply").exists()}

    def _leveled_paths(self) -> dict[str,Path]:
        if not self.workspace:return {}
        if self.project.stage_status.get("leveling") != StageStatus.COMPLETE:return {}
        active=None
        report=self.project.artifacts.get("leveling_report")
        if report and Path(report).is_file():
            try:active={item["scan_id"] for item in json.loads(Path(report).read_text())["scans"] if item.get("applied")}
            except (ValueError,KeyError,TypeError):return {}
        return {scan.scan_id:self.workspace/"outputs"/"scans"/scan.scan_id/"leveled_cloud.ply" for scan in self.scans if (active is None or scan.scan_id in active) and (self.workspace/"outputs"/"scans"/scan.scan_id/"leveled_cloud.ply").exists()}

    def _leveling_pose_corrections(self) -> dict[str,dict]:
        corrections={}
        for scan_id,path in self._leveled_paths().items():
            try:
                report=json.loads(path.with_suffix(".report.json").read_text(encoding="utf-8"))
                if report.get("converged"):
                    nx=float(report.get("nx",0.0)); ny=float(report.get("ny",0.0))
                    nz=float(np.sqrt(max(1.0-nx*nx-ny*ny,1e-12)))
                    axis_map=np.asarray(self.profile.axis_map,dtype=np.float64)
                    pan_axis=axis_map@np.array((nx,ny,nz),dtype=np.float64)
                    pan_axis/=np.linalg.norm(pan_axis)
                    corrections[scan_id]={"theta_scale":float(report.get("theta_scale",1.0)),"pan_axis":pan_axis.tolist()}
            except (OSError,ValueError,TypeError):continue
        return corrections

    def _leveling_theta_scales(self) -> dict[str,float]:
        return {scan_id:float(values["theta_scale"]) for scan_id,values in self._leveling_pose_corrections().items()}

    def colorize(self) -> None:
        if not self.workspace:return
        scans=[scan for scan in self.scans if scan.image_paths]; profile=self.profile; base=self.workspace/"outputs"/"scans"
        if not scans:return
        settings=ColorizationSettings(**_default_processing_settings()["colorization"])
        self.project.settings["colorization"]=asdict(settings)
        leveled=self._leveled_paths(); azimuth_scales=self._leveling_theta_scales()
        def operation(progress,cancelled):
            results=[]
            for i,scan in enumerate(scans):
                result=colorize_scan(scan,profile,base/scan.scan_id/"colored_cloud.ply",settings,lambda f,m:progress((i+f)/len(scans),m),cancelled,source_path=leveled.get(scan.scan_id),azimuth_scale=azimuth_scales.get(scan.scan_id,1.0)); results.append(result)
            return results
        def done(results):
            low_coverage=[]
            for scan,result in zip(scans,results,strict=True):
                scan.colored_cloud_path=result.output_path
                if result.coverage < _LOW_COVERAGE_THRESHOLD:low_coverage.append(f"{scan.scan_id} ({result.coverage:.1%})")
                preferences=self._scan_layer_preferences.setdefault(scan.scan_id,set())
                for previous in tuple(preferences):self._retire_layer(previous)
                preferences.clear(); preferences.add(f"{scan.scan_id}:colored")
                if scan.scan_id not in self._hidden_scans:
                    self.scene.add_cloud(result.output_path,f"{scan.scan_id}:colored",transform=self._display_transform(scan.scan_id))
            self.project.stage_status["colorization"]=StageStatus.FAILED if low_coverage else StageStatus.COMPLETE
            self._stale_after("colorization"); self.project.artifact_fingerprints["colorization"]=self._stage_fingerprint("colorization")
            if low_coverage:
                message=f"Colorization produced almost no color for: {', '.join(low_coverage)} — check camera poses/photos for these scans."
                self._show_status(message); QMessageBox.warning(self,"Low colorization coverage",message)
        self._run(operation,done,"colorization")

    def register(self) -> None:
        values=_default_processing_settings()["registration"]; pair_mode=values.pop("pair_mode")
        settings=RegistrationSettings(**values)
        self.project.settings["registration"]={**asdict(settings),"pair_mode":pair_mode}
        def operation(progress,cancelled):return register_scans(self.scans,self.profile,self._colored_paths(),settings,pair_mode,progress,cancelled,leveled_paths=self._leveled_paths())
        def done(edges):
            self.project.edges=edges; self.project.stage_status["registration"]=StageStatus.READY
            report=self.workspace/"outputs"/"registration.report.json"; report.parent.mkdir(parents=True,exist_ok=True)
            report.write_text(json.dumps({"edges":[{"source":e.source_id,"target":e.target_id,"method":e.method,"fitness":e.fitness,"inlier_rmse":e.inlier_rmse,"confidence":e.confidence,"accepted":e.accepted,"manual":e.manual} for e in edges]},indent=2)+"\n",encoding="utf-8")
            self.project.artifacts["registration_report"]=str(report); self.project.artifact_fingerprints["registration_pairs"]=self._stage_fingerprint("registration_pairs")
            self._show_status(f"Accepted {sum(e.accepted for e in edges)}/{len(edges)} registration edges")
        self._run(operation,done,"registration")

    def optimize(self) -> None:
        ids=[s.scan_id for s in self.scans]; reference=self.project.reference_scan_id or ids[0]; voxel=RegistrationSettings().voxel_size_m
        edges=list(self.project.edges)
        def operation(progress,cancelled):
            progress(.05,"Optimizing pose graph"); poses=optimize_registration_graph(ids,edges,reference,voxel); progress(1.,"Pose graph optimized"); return poses
        def done(poses):
            self.project.transforms=poses; self.project.stage_status["registration"]=StageStatus.COMPLETE
            self._stale_after("registration"); self.project.artifact_fingerprints["registration"]=self._stage_fingerprint("registration")
            self._show_registered()
        self._run(operation,done,"registration")


    def fuse(self) -> None:
        output=self.workspace/"outputs"/"merged_cloud.ply"
        transforms=dict(self.project.transforms)
        if len(self.scans)==1:
            transforms[self.scans[0].scan_id]=np.eye(4)
            self.project.transforms=dict(transforms)
        settings=FusionSettings(**_default_processing_settings()["fusion"])
        self.project.settings["room_mode"]=False
        self.project.settings["fusion"]=asdict(settings)
        def operation(progress,cancelled):
            return fuse_scans(self.scans,transforms,self.profile,output,self._colored_paths(),settings,
                              leveled_paths=self._leveled_paths(),progress=progress,cancelled=cancelled,
                              reference_scan_id=self.project.reference_scan_id)
        def done(result):
            self.project.transforms=dict(result.transforms or transforms)
            self.project.artifacts["merged_cloud"]=str(result.output_path)
            self.project.artifacts["fusion_report"]=str(result.report_path)
            if result.provenance_path:self.project.artifacts["fusion_provenance"]=str(result.provenance_path)
            if result.before_preview_path:self.project.artifacts["fusion_before_preview"]=str(result.before_preview_path)
            self.project.stage_status["fusion"]=StageStatus.COMPLETE; self._stale_after("fusion"); self.project.artifact_fingerprints["fusion"]=self._stage_fingerprint("fusion")
            self.project.artifact_fingerprints["registration"]=self._stage_fingerprint("registration")
            if len(self.scans)>1:
                for actor_name in tuple(self.scene.actors):
                    if actor_name.startswith("combined:"):self.scene.remove(actor_name)
                self._combined_layer_preferences={"combined:merged","combined:fusion-before"}
                if result.before_preview_path:self.scene.add_cloud(result.before_preview_path,"combined:fusion-before")
                self.scene.add_cloud(result.output_path,"combined:merged")
                self._hide_individual_scan_layers()
        self._run(operation,done,"fusion")

    def mesh(self) -> None:
        merged=self.project.artifacts.get("merged_cloud"); output=self.workspace/"outputs"/"mesh.ply"
        settings=MeshingSettings(**_default_processing_settings()["mesh"])
        self.project.settings["mesh"]=asdict(settings)
        def operation(progress,cancelled):return generate_mesh(merged,output,settings,self.workspace/"outputs"/"mesh.obj",progress,cancelled)
        def done(result):
            self.project.artifacts["mesh"]=str(result.ply_path)
            if result.obj_path:self.project.artifacts["mesh_obj"]=str(result.obj_path)
            if result.detail_ply_path:self.project.artifacts["mesh_full"]=str(result.detail_ply_path)
            self.project.stage_status["mesh"]=StageStatus.COMPLETE; self._stale_after("mesh"); self.project.artifact_fingerprints["mesh"]=self._stage_fingerprint("mesh")
            name=f"{self.scans[0].scan_id}:mesh" if len(self.scans)==1 else "combined:mesh"
            preferences=self._scan_layer_preferences.setdefault(self.scans[0].scan_id,set()) if len(self.scans)==1 else self._combined_layer_preferences
            for previous in tuple(preferences):self._retire_layer(previous)
            preferences.clear(); preferences.add(name)
            self.scene.add_mesh(result.ply_path,name)
            if len(self.scans)>1:self._hide_individual_scan_layers()
        self._run(operation,done,"mesh")

    def export_mesh(self) -> None:
        source_text=self.project.artifacts.get("mesh_obj") or self.project.artifacts.get("mesh")
        if not source_text:return
        source=Path(source_text)
        path,_=QFileDialog.getSaveFileName(self,"Export mesh",source.name,"Mesh (*.obj *.ply)")
        if not path:return
        target=Path(path)
        if target.suffix.lower() not in {".obj",".ply"}:target=target.with_suffix(source.suffix)
        selected=self.project.artifacts.get("mesh_obj") if target.suffix.lower()==".obj" else self.project.artifacts.get("mesh")
        if not selected:return
        def operation(progress,cancelled):
            return export_mesh_asset(selected,target,[scan.source_folder for scan in self.scans],progress=progress,cancelled=cancelled)
        self._run(operation,lambda result:self._show_status(f"Exported complete mesh assets to {result}"))

    def texture(self) -> None:
        mesh_path=self.project.artifacts.get("mesh"); output=self.workspace/"outputs"
        transforms=dict(self.project.transforms)
        if len(self.scans)==1:
            transforms[self.scans[0].scan_id]=np.eye(4)
            self.project.transforms=dict(transforms)
        settings=TextureSettings(**_default_processing_settings()["texture"])
        self.project.settings["texture"]=asdict(settings)
        azimuth_scales=self._leveling_theta_scales()
        def operation(progress,cancelled):return texture_mesh(mesh_path,self.scans,transforms,self.profile,output,settings,progress,cancelled,azimuth_scales=azimuth_scales)
        def done(result):
            self.project.artifacts.update(textured_obj=str(result.obj_path),textured_mtl=str(result.mtl_path),texture=str(result.texture_paths[0]),texture_report=str(result.report_path)); self.project.stage_status["texture"]=StageStatus.COMPLETE; self.project.artifact_fingerprints["texture"]=self._stage_fingerprint("texture")
            name=f"{self.scans[0].scan_id}:textured" if len(self.scans)==1 else "combined:textured"
            preferences=self._scan_layer_preferences.setdefault(self.scans[0].scan_id,set()) if len(self.scans)==1 else self._combined_layer_preferences
            for previous in tuple(preferences):self._retire_layer(previous)
            preferences.clear(); preferences.add(name)
            self.scene.add_mesh(result.obj_path,name,result.texture_paths[0])
            if len(self.scans)>1:self._hide_individual_scan_layers()
        self._run(operation,done,"texture")

    def export_results(self) -> None:
        if not self.project.artifacts:return
        folder=QFileDialog.getExistingDirectory(self,"Export results to")
        if not folder:return
        target=Path(folder).resolve()
        source=(self.workspace/"outputs").resolve()
        if target==source or source in target.parents:
            QMessageBox.warning(self,"Invalid export location","Choose a folder outside the project outputs directory."); return
        def operation(progress,cancelled):
            return export_outputs(source,target,[scan.source_folder for scan in self.scans],progress=progress,cancelled=cancelled)
        self._run(operation,lambda result:self._show_status(f"Exported complete result set to {result}"))

    def _stale_after(self,stage: str) -> None:
        order=["leveling","calibration","colorization","registration","fusion","mesh","texture"]
        for item in order[order.index(stage)+1:]:
            if self.project.stage_status.get(item)!=StageStatus.MISSING:self.project.stage_status[item]=StageStatus.STALE
            self.project.artifact_fingerprints.pop(item,None)

    def _stage_fingerprint(self, stage: str) -> str:
        return compute_stage_fingerprint(self.scans,stage,self.profile,self.project.settings,self.project.transforms)

    def _check_fingerprints(self) -> None:
        for stage,previous in list(self.project.artifact_fingerprints.items()):
            if previous!=self._stage_fingerprint(stage):
                key="registration" if stage=="registration_pairs" else stage
                self.project.stage_status[key]=StageStatus.STALE
                self._show_status(f"{key}: inputs changed; repeat this stage")
        for key,value in self.project.artifacts.items():
            if not Path(value).exists():
                self._show_status(f"Missing result: {key}: {value}")
                stage=next((stage for stage in ("leveling","calibration","colorization","registration","texture","mesh") if key.startswith(stage)),"fusion")
                if key.startswith("textured"):stage="texture"
                self.project.stage_status[stage]=StageStatus.STALE;self._stale_after(stage)


    def _use_default_settings(self) -> None:
        defaults=_default_processing_settings()
        for key,value in defaults.items():
            previous=self.project.settings.get(key)
            if previous is None or json.dumps(previous,sort_keys=True)==json.dumps(value,sort_keys=True):continue
            stage="fusion" if key=="room_mode" else key
            if self.project.stage_status.get(stage)!=StageStatus.MISSING:
                self.project.stage_status[stage]=StageStatus.STALE
            self.project.artifact_fingerprints.pop(stage,None)
            if stage=="registration":self.project.artifact_fingerprints.pop("registration_pairs",None)
            self._stale_after(stage)
        self.project.settings.pop("ui",None)
        self.project.settings.update(defaults)

    def _layer_checked(self, name: str, preferences: set[str], enabled: bool) -> bool:
        """Return effective layer visibility, retaining its preference while hidden."""
        if enabled:
            if name=="combined:registered":
                actors=[actor for actor_name,actor in self.scene.actors.items() if actor_name.startswith("combined:registered:")]
                if actors:return any(bool(actor.GetVisibility()) for actor in actors)
            else:
                actor=self.scene.actors.get(name)
                if actor is not None:return bool(actor.GetVisibility())
        return name in preferences

    def _scan_layers(self, scan) -> list[tuple[str,str,str,str,dict]]:
        """Return source-scan layer definitions for tree display and visibility updates."""
        layers=[("White dots",str(scan.cloud_path),f"{scan.scan_id}:white","cloud",{"color":"#d8dde5"}),
                ("Reflectance dots",str(scan.cloud_path),f"{scan.scan_id}:reflectance","cloud",{"scalar":"intensity","cmap":"turbo"})]
        leveled=self._leveled_paths().get(scan.scan_id)
        if leveled:
            layers.append(("Corrected dots",str(leveled),f"{scan.scan_id}:leveled","cloud",{"color":"#7fd8a6"}))
            layers.append(("Corrected half passes",str(leveled),f"{scan.scan_id}:half-passes","cloud",{"scalar":"half_pass","cmap":["#36a2eb","#ff6384"]}))
        colored=self._colored_paths().get(scan.scan_id)
        if colored:layers.append(("Color dots",str(colored),f"{scan.scan_id}:colored","cloud",{}))
        elif {"red","green","blue"}.issubset(scan.vertex_properties):layers.append(("Color dots",str(scan.cloud_path),f"{scan.scan_id}:colored","cloud",{}))
        if len(self.scans)==1 and self.project.artifacts.get("mesh"):
            layers.append(("Mesh",self.project.artifacts["mesh"],f"{scan.scan_id}:mesh","mesh",{}))
        if len(self.scans)==1 and self.project.artifacts.get("textured_obj"):
            layers.append(("Textured mesh",self.project.artifacts["textured_obj"],f"{scan.scan_id}:textured","mesh",{"texture_path":self.project.artifacts.get("texture")}))
        return layers

    def _combined_layers(self) -> list[tuple[str,str,str,str,dict]]:
        """Return the available combined-result layer definitions."""
        layers=[]
        if len(self.project.transforms)==len(self.scans):layers.append(("Aligned scans (before fusion)","","combined:registered","registered_layer",{}))
        if self.project.artifacts.get("fusion_before_preview"):layers.append(("Before surface fusion (source colors)",self.project.artifacts["fusion_before_preview"],"combined:fusion-before","cloud",{}))
        if self.project.artifacts.get("merged_cloud"):layers.append(("Stitched point cloud",self.project.artifacts["merged_cloud"],"combined:merged","cloud",{}))
        if self.project.artifacts.get("mesh"):layers.append(("Mesh",self.project.artifacts["mesh"],"combined:mesh","mesh",{}))
        if self.project.artifacts.get("textured_obj"):layers.append(("Textured mesh",self.project.artifacts["textured_obj"],"combined:textured","mesh",{"texture_path":self.project.artifacts.get("texture")}))
        return layers

    def _restore_stitched_result(self) -> None:
        """Restore the saved stitched cloud from project artifacts or the workspace.

        Keep combined-layer selection separate from each source scan's visibility.
        """
        if len(self.scans)<2:return
        configured=self.project.artifacts.get("merged_cloud")
        configured_path=Path(configured) if configured else None
        workspace_copy=self.workspace/"outputs"/"merged_cloud.ply" if self.workspace else None
        use_workspace_copy=(
            workspace_copy is not None and workspace_copy.is_file() and
            (configured_path is None or not configured_path.is_file() or configured_path.resolve()!=workspace_copy.resolve())
        )
        if use_workspace_copy:
            # Recover older manifests from the workspace and persist the resolved path.
            self.project.artifacts["merged_cloud"]=str(workspace_copy)
            self.save()
        layers={name:(path,kind,options) for _label,path,name,kind,options in self._combined_layers()}
        selected=None
        merged=layers.get("combined:merged")
        if merged is not None and Path(merged[0]).is_file():
            selected="combined:merged"
        elif "combined:registered" in layers:
            selected="combined:registered"
        else:
            for fallback in ("combined:textured","combined:mesh"):
                layer=layers.get(fallback)
                if layer is not None and Path(layer[0]).is_file():
                    selected=fallback;break
        if selected is None:return
        self._combined_hidden=False
        self._combined_layer_preferences={selected}
        self._hide_individual_scan_layers()
        path,kind,options=layers[selected]
        if kind=="registered_layer":self._show_registered()
        elif kind=="cloud":self.scene.add_cloud(path,selected,**options)
        else:self.scene.add_mesh(path,selected,options.get("texture_path"))

    def _set_scan_group_visible(self, scan_id: str, checked: bool) -> None:
        """Apply source-scan visibility without changing live tree items."""
        if checked:self._hidden_scans.discard(scan_id)
        else:self._hidden_scans.add(scan_id)
        preferences=self._scan_layer_preferences.setdefault(scan_id,set())
        if checked and not preferences:preferences.add(f"{scan_id}:white")
        scan=next((s for s in self.scans if s.scan_id==scan_id),None)
        if scan is not None:
            for _label,path,name,kind,options in self._scan_layers(scan):
                if name not in preferences:continue
                if checked:
                    if name not in self.scene.actors:
                        if kind=="cloud":self.scene.add_cloud(path,name,transform=self._display_transform(name),**options)
                        else:self.scene.add_mesh(path,name,options.get("texture_path"))
                elif name in self.scene.actors:self._retire_layer(name)
        if checked and "combined:registered" in self._combined_layer_preferences:self._show_registered()

    def _set_combined_group_visible(self, checked: bool) -> None:
        """Apply combined-result visibility without changing live tree items."""
        self._combined_hidden=not checked
        if not checked:
            for actor_name in tuple(self.scene.actors):
                if actor_name.startswith("combined:"):self.scene.remove(actor_name)
        else:
            for _label,path,name,kind,options in self._combined_layers():
                if name not in self._combined_layer_preferences:continue
                if kind=="registered_layer":self._show_registered()
                elif name not in self.scene.actors:
                    if kind=="cloud":self.scene.add_cloud(path,name,**options)
                    elif kind=="mesh":self.scene.add_mesh(path,name,options.get("texture_path"))

    def _refresh_tree_deferred(self) -> None:
        """Rebuild after Qt finishes processing the current itemChanged signal.

        Clearing the tree inside that signal destroys the item Qt is still using.
        A zero-delay timer defers destruction until control returns to the event loop.
        """
        QTimer.singleShot(0,self._refresh_tree)

    def _refresh_tree(self) -> None:
        self._tree_syncing=True
        try:
            self.tree.clear(); scans_node=QTreeWidgetItem(["Source scans",str(len(self.scans))]); self.tree.addTopLevelItem(scans_node)
            for scan in self.scans:
                preferences=self._scan_layer_preferences.setdefault(scan.scan_id,set())
                enabled=scan.scan_id not in self._hidden_scans
                item=QTreeWidgetItem([scan.scan_id,(f"shown · {scan.point_count:,} points" if enabled else f"hidden · {scan.point_count:,} points")]); item.setData(0,Qt.ItemDataRole.UserRole,("scan_group",scan.scan_id)); scans_node.addChild(item)
                item.setToolTip(0,"Master visibility for this source scan. Child layer choices are remembered while hidden.")
                item.setFlags(item.flags()|Qt.ItemFlag.ItemIsUserCheckable); item.setCheckState(0,Qt.CheckState.Checked if enabled else Qt.CheckState.Unchecked)
                for label,path,name,kind,options in self._scan_layers(scan):
                    checked=self._layer_checked(name,preferences,enabled)
                    child=QTreeWidgetItem([label,"shown" if enabled and checked else "hidden"]); child.setData(0,Qt.ItemDataRole.UserRole,(kind,path,name,options)); item.addChild(child)
                    flags=child.flags()|Qt.ItemFlag.ItemIsUserCheckable
                    if not enabled:flags&=~Qt.ItemFlag.ItemIsEnabled
                    child.setFlags(flags); child.setCheckState(0,Qt.CheckState.Checked if checked else Qt.CheckState.Unchecked)
                item.setExpanded(True)
            scans_node.setExpanded(True)

            if len(self.scans)>1:
                combined_ready=bool(self._combined_layers())
                combined=QTreeWidgetItem(["Stitched result (combined scans)",("hidden" if self._combined_hidden else "shown") if combined_ready else "not aligned"])
                combined.setData(0,Qt.ItemDataRole.UserRole,("combined_group","combined")); combined.setFlags(combined.flags()|Qt.ItemFlag.ItemIsUserCheckable)
                combined.setToolTip(0,"Saved stitched point cloud, aligned scans, mesh, and textured result.")
                combined.setCheckState(0,Qt.CheckState.Unchecked if self._combined_hidden else Qt.CheckState.Checked); self.tree.insertTopLevelItem(0,combined)
                for label,path,name,kind,options in self._combined_layers():
                    checked=self._layer_checked(name,self._combined_layer_preferences,not self._combined_hidden)
                    child=QTreeWidgetItem([label,"shown" if not self._combined_hidden and checked else "hidden"]); child.setData(0,Qt.ItemDataRole.UserRole,(kind,path,name,options)); combined.addChild(child)
                    flags=child.flags()|Qt.ItemFlag.ItemIsUserCheckable
                    if self._combined_hidden:flags&=~Qt.ItemFlag.ItemIsEnabled
                    child.setFlags(flags); child.setCheckState(0,Qt.CheckState.Checked if checked else Qt.CheckState.Unchecked)
                combined.setExpanded(True)

            reports=[(key,path) for key,path in self.project.artifacts.items() if Path(path).suffix.lower()==".json"]
            if reports:
                reports_node=QTreeWidgetItem(["Reports",str(len(reports))]); self.tree.addTopLevelItem(reports_node)
                for key,path in reports:
                    child=QTreeWidgetItem([key,"ready" if Path(path).exists() else "missing"]); child.setData(0,Qt.ItemDataRole.UserRole,("report",path,key,{})); reports_node.addChild(child)
        finally:
            self._tree_syncing=False

    def _hide_individual_scan_layers(self) -> None:
        """Hide individual scan layers when displaying a combined result."""
        for scan in self.scans:
            preferences=self._scan_layer_preferences.get(scan.scan_id,set())
            for previous in tuple(preferences):self._retire_layer(previous)
            preferences.clear()

    def _retire_layer(self, name: str) -> None:
        """Release an actor and its geometry; registered layers also own per-scan actors."""
        if name == "combined:registered":
            for actor_name in tuple(self.scene.actors):
                if actor_name.startswith("combined:registered:"):self.scene.remove(actor_name)
        elif name in self.scene.actors:
            self.scene.remove(name)

    def _display_item(self,item,column) -> None:
        data=item.data(0,Qt.ItemDataRole.UserRole)
        if not data:return
        kind=data[0]
        try:
            if kind=="scan_group" or kind=="combined_group":
                item.setCheckState(0,Qt.CheckState.Unchecked if item.checkState(0)==Qt.CheckState.Checked else Qt.CheckState.Checked); return
            path,name=data[1:3]; options=data[3] if len(data)>3 else {}
            if kind in {"cloud","mesh","registered_layer"} and item.flags()&Qt.ItemFlag.ItemIsUserCheckable:
                item.setCheckState(0,Qt.CheckState.Unchecked if item.checkState(0)==Qt.CheckState.Checked else Qt.CheckState.Checked); return
            if kind=="report":
                dialog=QDialog(self); dialog.setWindowTitle(name); layout=QVBoxLayout(dialog); view=QTextEdit(); view.setReadOnly(True); view.setPlainText(Path(path).read_text(encoding="utf-8")); layout.addWidget(view); dialog.resize(800,600); dialog.exec(); return
        except Exception as exc:QMessageBox.critical(self,"Cannot display item",str(exc))

    def _tree_item_changed(self, item: QTreeWidgetItem, _column: int) -> None:
        if self._tree_syncing or _column!=0:return
        data=item.data(0,Qt.ItemDataRole.UserRole)
        if not data:return
        kind=data[0]; checked=item.checkState(0)==Qt.CheckState.Checked
        if kind=="scan_group":
            self._set_scan_group_visible(data[1],checked)
            self._refresh_tree_deferred(); return
        if kind=="combined_group":
            self._set_combined_group_visible(checked)
            self._refresh_tree_deferred(); return
        if kind=="registered_layer":
            if checked:
                self._uncheck_other_point_layers(item,"combined:registered",self._combined_layer_preferences)
                self._combined_layer_preferences.add("combined:registered")
                if not self._combined_hidden:self._show_registered()
            else:
                self._combined_layer_preferences.discard("combined:registered")
                self._retire_layer("combined:registered")
            item.setText(1,"shown" if checked and not self._combined_hidden else "hidden")
            return
        if kind not in {"cloud","mesh"}:return
        _,path,name=data[:3]; options=data[3] if len(data)>3 else {}
        if name.startswith("combined:"):
            preferences=self._combined_layer_preferences; master_visible=not self._combined_hidden
        else:
            scan_id=name.split(":",1)[0]; preferences=self._scan_layer_preferences.setdefault(scan_id,set()); master_visible=scan_id not in self._hidden_scans
        if checked:
            if self._is_point_layer(name):self._uncheck_other_point_layers(item,name,preferences)
            preferences.add(name)
        else:preferences.discard(name)
        visible=checked and master_visible
        if visible:
            if name in self.scene.actors:self.scene.set_visible(name,True)
            else:
                try:
                    if kind=="cloud":self.scene.add_cloud(path,name,transform=self._display_transform(name),**options)
                    else:self.scene.add_mesh(path,name,options.get("texture_path"))
                except Exception as exc:
                    QMessageBox.critical(self,"Cannot display item",str(exc))
        # Retire unchecked actors to release their geometry from memory.
        elif name in self.scene.actors:self._retire_layer(name)
        item.setText(1,"shown" if visible else "hidden")

    @staticmethod
    def _is_point_layer(name: str) -> bool:
        return name=="combined:registered" or name=="combined:merged" or name.endswith((":white",":reflectance",":colored"))

    def _uncheck_other_point_layers(self,item: QTreeWidgetItem,name: str,preferences: set[str]) -> None:
        parent=item.parent()
        if parent is None:return
        self._tree_syncing=True
        try:
            for child_index in range(parent.childCount()):
                sibling=parent.child(child_index); sibling_data=sibling.data(0,Qt.ItemDataRole.UserRole)
                if not sibling_data or len(sibling_data)<3:continue
                sibling_name=sibling_data[2]
                if sibling_name==name or not self._is_point_layer(sibling_name):continue
                preferences.discard(sibling_name); sibling.setCheckState(0,Qt.CheckState.Unchecked); sibling.setText(1,"hidden")
                self._retire_layer(sibling_name)
        finally:
            self._tree_syncing=False

    def _show_registered(self) -> None:
        """Show aligned scans independently of source-layer visibility."""
        colors=["#ff595e","#ffca3a","#8ac926","#1982c4","#6a4c93","#f9844a","#43aa8b"]
        self._combined_layer_preferences.add("combined:registered")
        for i,scan in enumerate(self.scans):
            name=f"combined:registered:{scan.scan_id}"
            self.scene.add_cloud(scan.cloud_path,name,colors[i%len(colors)],self._display_transform(name))

    def _display_transform(self, name: str) -> np.ndarray:
        scan_id=name.rsplit(":",1)[-1] if name.startswith("combined:registered:") else name.split(":",1)[0]
        if scan_id not in {scan.scan_id for scan in self.scans}:return np.eye(4)
        axis=np.eye(4); axis[:3,:3]=np.asarray(self.profile.axis_map)*self.profile.unit_scale_to_m
        return np.asarray(self.project.transforms.get(scan_id,np.eye(4)))@axis


    def _toggle_measuring(self,enabled: bool) -> None:
        self.scene.set_measuring(enabled)

    def _cancel_measurement(self) -> None:
        if self.measure_mode.isChecked():
            self.measure_mode.setChecked(False)
        else:
            self.scene.set_measuring(False)

    def _on_measurement_state_changed(self,state: str) -> None:
        messages={
            "inactive":"Measurement off",
            "first":"Select first point.",
            "second":"Select second point.",
        }
        if state in messages:self.measure_label.setText(messages[state])

    def _on_measured(self,distance_m: float) -> None:
        cm=distance_m*100.0
        self.measure_label.setText(f"Distance: {distance_m:.3f} m ({cm:.1f} cm)")
        self._show_status(f"Measured distance: {cm:.1f} cm ({distance_m:.4f} m)")


    def _update_actions(self) -> None:
        busy=self.job_queue.current is not None; has=bool(self.scans); calibrated=self.project.stage_status.get("calibration")==StageStatus.COMPLETE
        single=len(self.scans)==1
        photos=any(scan.image_paths for scan in self.scans)
        self.buttons["registration"].setText("Registration (requires 2+ scans)" if single else "5. Register pairs")
        self.buttons["optimize"].setText("Pose graph (requires 2+ scans)" if single else "6. Optimize pose graph")
        self.buttons["fusion"].setText("5. Prepare / clean scan" if single else "7. Fuse / clean")
        self.buttons["mesh"].setText("6. Generate mesh" if single else "8. Generate mesh")
        self.buttons["texture"].setText("7. Texture mesh" if single else "9. Texture mesh")
        self.buttons["export"].setText("8. Export all results…" if single else "10. Export all results…")
        self.buttons["leveling"].setEnabled(has and not busy)
        self.buttons["calibration"].setEnabled(photos and not busy); self.buttons["colorization"].setEnabled(photos and calibrated and not busy)
        for key in ("calibration","colorization","texture"):
            self.buttons[key].setToolTip("Requires usable photographs; geometry stages remain available" if not photos else "")
        self.buttons["registration"].setEnabled(len(self.scans)>=2 and not busy); self.buttons["optimize"].setEnabled(bool(self.project.edges) and not busy)
        registered=(
            self.project.stage_status.get("registration")==StageStatus.COMPLETE
            and len(self.project.transforms)==len(self.scans)
            and has
        )
        geometry_ready=single or registered
        self.buttons["fusion"].setEnabled(geometry_ready and not busy)
        mesh_ready=self.project.stage_status.get("fusion")==StageStatus.COMPLETE and bool(self.project.artifacts.get("merged_cloud"))
        self.buttons["mesh"].setEnabled(mesh_ready and not busy)
        self.buttons["export_mesh"].setEnabled(bool(self.project.artifacts.get("mesh")) and not busy)
        texture_ready=self.project.stage_status.get("mesh")==StageStatus.COMPLETE and bool(self.project.artifacts.get("mesh"))
        self.buttons["texture"].setEnabled(photos and texture_ready and geometry_ready and calibrated and not busy)
        self.buttons["export"].setEnabled(bool(self.project.artifacts) and not busy)
        for action in (self.import_action,self.new_project_action,self.open_project_action):action.setEnabled(not busy)
        self.export_action.setEnabled(bool(self.project.artifacts) and not busy)
        self._refresh_step_badges()
        self.statusBar().showMessage("Working…" if busy else (f"{len(self.scans)} scan(s) loaded" if has else "No project scans loaded — import a scan folder to begin"))

    def _refresh_step_badges(self) -> None:
        for key,stage in _STEP_STAGE_MAP.items():
            badge=self.step_status_labels.get(key)
            if not badge:continue
            status=self.project.stage_status.get(stage,StageStatus.MISSING)
            text,color=_STAGE_BADGES.get(status,("",""))
            badge.setText(text); badge.setStyleSheet(f"color:{color}; font-weight:600;")


