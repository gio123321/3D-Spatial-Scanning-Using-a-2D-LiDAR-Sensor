from __future__ import annotations

from itertools import permutations, product
from pathlib import Path

import numpy as np
import cv2
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QFileDialog, QFormLayout,
    QCheckBox, QHBoxLayout, QLabel, QMessageBox, QPushButton, QScrollArea, QSpinBox,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from ..calibration.optimize import CalibrationCorrespondence, optimize_profile
from ..calibration.profile import load_profile, save_profile
from ..colorization import CalibrationOverlayFrame, calibration_overlay_frame
from ..models import CameraProfile, LensModel, ScanRecord
from ..io.scan import effective_pose_offset, image_pose_pairs


class ClickableImageLabel(QLabel):
    pixel_clicked=Signal(float,float)

    def mousePressEvent(self,event) -> None:
        pixmap=self.pixmap()
        if pixmap:
            x=event.position().x()-(self.width()-pixmap.width())/2; y=event.position().y()-(self.height()-pixmap.height())/2
            if 0<=x<pixmap.width() and 0<=y<pixmap.height():self.pixel_clicked.emit(x,y)
        super().mousePressEvent(event)


class CalibrationDialog(QDialog):
    def __init__(self, scan: ScanRecord, profile: CameraProfile, parent=None,
                 leveled_path: Path | None = None, azimuth_scale: float = 1.0,
                 pan_axis: list[float] | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Reusable camera calibration")
        self.resize(1200, 760)
        self.scan = scan
        self._leveled_path = leveled_path
        self._azimuth_scale = float(azimuth_scale)
        self.profile = CameraProfile.from_dict(profile.to_dict())
        self._leveled_pan_axis: list[float] | None = None
        self._leveled_pan_sign = float(self.profile.pan_sign)
        if pan_axis is not None:
            axis=np.asarray(pan_axis,dtype=np.float64)
            if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis)<1e-9:
                raise ValueError("Leveling supplied an invalid camera pan axis")
            self._leveled_pan_axis=(axis/np.linalg.norm(axis)).tolist()
            self.profile.pan_axis=list(self._leveled_pan_axis)
        self._overlay_frame: CalibrationOverlayFrame | None = None
        self._pick_mode: str | None = None
        self._source_marker: tuple[int, int, float, float] | None = None
        root = QHBoxLayout(self)
        left = QVBoxLayout(); root.addLayout(left, 3)
        self.image_label = ClickableImageLabel(alignment=Qt.AlignmentFlag.AlignCenter); self.image_label.pixel_clicked.connect(self._handle_overlay_click)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(self.image_label); left.addWidget(scroll)
        row = QHBoxLayout(); left.addLayout(row)
        self.shot_combo = QComboBox()
        effective_offset=effective_pose_offset(scan,profile.image_pose_offset_frames)
        for image_shot,pose_shot in image_pose_pairs(scan,profile.image_pose_offset_frames,self._azimuth_scale):
            self.shot_combo.addItem(
                f"Image {image_shot.index:03d} → pose {pose_shot.index:03d}: az={pose_shot.cmd_az:.2f}, tilt={pose_shot.cmd_tilt:.2f}",
                image_shot.index,
            )
        row.addWidget(self.shot_combo)
        self.opacity = QDoubleSpinBox(); self.opacity.setRange(0.05,1); self.opacity.setSingleStep(.05); self.opacity.setValue(.75); row.addWidget(self.opacity)
        self.point_size = QSpinBox(); self.point_size.setRange(1,6); self.point_size.setValue(1); row.addWidget(self.point_size)
        self.show_points = QCheckBox("Projected points"); self.show_points.setChecked(True); self.show_points.toggled.connect(self.refresh); row.addWidget(self.show_points)
        update = QPushButton("Update overlay"); update.clicked.connect(self.refresh); row.addWidget(update)
        controls = QWidget(); form = QFormLayout(controls); control_scroll=QScrollArea(); control_scroll.setWidgetResizable(True); control_scroll.setWidget(controls); root.addWidget(control_scroll, 2)
        self.lens = QComboBox(); self.lens.addItems([LensModel.FISHEYE.value,LensModel.BROWN.value]); self.lens.setCurrentText(self.profile.lens_model.value); form.addRow("Lens model",self.lens)
        self.rotations=self._rotation_options()
        self.axis_map=QComboBox(); self.axis_map.addItems(self.rotations); self._select_matrix(self.axis_map,self.profile.axis_map); form.addRow("PLY → scan axes",self.axis_map)
        self.camera_basis=QComboBox(); self.camera_basis.addItems(self.rotations); self._select_matrix(self.camera_basis,self.profile.camera_basis); form.addRow("Camera → scan basis",self.camera_basis)
        self.pan_axis=QComboBox(); self.pan_axis.addItems(["+X","-X","+Y","-Y","+Z","-Z"]); self.pan_axis.setCurrentText(self._axis_text(self.profile.pan_axis,self.profile.pan_sign)); form.addRow("Pan axis/sign",self.pan_axis)
        if self._leveled_pan_axis is not None:
            self.pan_axis.setEnabled(False)
            self.pan_axis.setToolTip("Set by the accepted doubled-surface correction for this rig")
        self.tilt_axis=QComboBox(); self.tilt_axis.addItems(["+X","-X","+Y","-Y","+Z","-Z"]); self.tilt_axis.setCurrentText(self._axis_text(self.profile.tilt_axis,self.profile.tilt_sign)); form.addRow("Tilt axis/sign",self.tilt_axis)
        self.rotation_order=QComboBox(); self.rotation_order.addItems(["pan_tilt","tilt_pan"]); self.rotation_order.setCurrentText(self.profile.rotation_order); form.addRow("Rotation order",self.rotation_order)
        self.pose_offset=QSpinBox(); self.pose_offset.setRange(-3,3); self.pose_offset.setValue(effective_offset)
        self.pose_offset.setEnabled(scan.session.image_pose_offset_frames is None)
        form.addRow("Image → pose index offset",self.pose_offset)
        self.fields: dict[str,QDoubleSpinBox] = {}
        values={"fx":self.profile.fx,"fy":self.profile.fy,"cx":self.profile.cx,"cy":self.profile.cy,
                "pan_zero_deg":self.profile.pan_zero_deg,"tilt_zero_deg":self.profile.tilt_zero_deg,
                "yaw":self.profile.boresight_ypr_deg[0],"pitch":self.profile.boresight_ypr_deg[1],"roll":self.profile.boresight_ypr_deg[2],
                "tx":self.profile.camera_center_xyz_m[0],"ty":self.profile.camera_center_xyz_m[1],"tz":self.profile.camera_center_xyz_m[2]}
        values.update({f"d{i}": value for i,value in enumerate((self.profile.distortion+[0]*8)[:8])})
        hide_remaining=False
        for name,value in values.items():
            box=QDoubleSpinBox(); box.setDecimals(6); box.setRange(-5000,5000); box.setValue(value)
            box.setSingleStep(.001 if name.startswith(("d","t")) else .1); self.fields[name]=box
            labels={"d0":"k1","d1":"k2","d2":"k3 / p1","d3":"k4 / p2","d4":"k3 (Brown)","d5":"k4 (Brown)","d6":"k5 (Brown)","d7":"k6 (Brown)"}
            form.addRow(labels.get(name,name),box)
            if hide_remaining:form.setRowVisible(box,False)
            if name=="cy":hide_remaining=True
        reset_fov=QPushButton("Reset focal length from 120° FOV"); reset_fov.clicked.connect(self._reset_fov); form.addRow(reset_fov); form.setRowVisible(reset_fov,False)
        reset_pose=QPushButton("Reset supplied rig pose axes"); reset_pose.clicked.connect(self._reset_rig_pose); form.addRow(reset_pose); form.setRowVisible(reset_pose,False)
        corr_label=QLabel("LiDAR point → matching photo pixel")
        self.corr_table=QTableWidget(0,6); self.corr_table.setHorizontalHeaderLabels(["X","Y","Z","u","v","shot"]); form.addRow(corr_label); form.addRow(self.corr_table)
        form.setRowVisible(corr_label,False); form.setRowVisible(self.corr_table,False)
        pick_buttons=QHBoxLayout()
        pick_lidar=QPushButton("1. Pick projected LiDAR point"); pick_lidar.clicked.connect(self._begin_lidar_pick)
        pick_image=QPushButton("2. Pick matching photo point"); pick_image.clicked.connect(self._begin_image_pick)
        pick_buttons.addWidget(pick_lidar); pick_buttons.addWidget(pick_image); form.addRow(pick_buttons); form.setRowVisible(pick_buttons,False)
        self.pick_status=QLabel("Use the two pick buttons. Collect 6–12 landmarks across at least 3 images and different depths.")
        self.pick_status.setWordWrap(True); form.addRow(self.pick_status); form.setRowVisible(self.pick_status,False)
        corr_buttons=QHBoxLayout(); add_corr=QPushButton("Add manually"); add_corr.clicked.connect(self._add_correspondence); remove_corr=QPushButton("Remove"); remove_corr.clicked.connect(self._remove_correspondence); optimize=QPushButton("Optimize"); optimize.clicked.connect(self._optimize)
        corr_buttons.addWidget(add_corr); corr_buttons.addWidget(remove_corr); corr_buttons.addWidget(optimize); form.addRow(corr_buttons); form.setRowVisible(corr_buttons,False)
        self.optimize_intrinsics=QCheckBox("Intrinsics"); self.optimize_distortion=QCheckBox("Distortion")
        self.optimize_extrinsics=QCheckBox("Boresight + XYZ translation"); self.optimize_extrinsics.setChecked(True)
        self.optimize_angle_zeros=QCheckBox("Pan/tilt zero offsets (separate pass)")
        optimizer_groups=QVBoxLayout(); optimizer_groups.addWidget(self.optimize_intrinsics); optimizer_groups.addWidget(self.optimize_distortion); optimizer_groups.addWidget(self.optimize_extrinsics); optimizer_groups.addWidget(self.optimize_angle_zeros); form.addRow(optimizer_groups); form.setRowVisible(optimizer_groups,False)
        load_button=QPushButton("Load reusable profile…"); load_button.clicked.connect(self.load_from); form.addRow(load_button)
        save_button=QPushButton("Save reusable profile…"); save_button.clicked.connect(self.save_as); form.addRow(save_button)
        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept); buttons.rejected.connect(self.reject); form.addRow(buttons)
        self.shot_combo.currentIndexChanged.connect(self.refresh)
        self.refresh()

    def _apply_fields(self) -> None:
        p=self.profile; p.lens_model=LensModel(self.lens.currentText())
        p.axis_map=self.rotations[self.axis_map.currentText()].tolist(); p.camera_basis=self.rotations[self.camera_basis.currentText()].tolist()
        if self._leveled_pan_axis is None:
            p.pan_axis,p.pan_sign=self._axis_value(self.pan_axis.currentText())
        else:
            p.pan_axis=list(self._leveled_pan_axis); p.pan_sign=self._leveled_pan_sign
        p.tilt_axis,p.tilt_sign=self._axis_value(self.tilt_axis.currentText())
        p.rotation_order=self.rotation_order.currentText()
        p.image_pose_offset_frames=self.pose_offset.value()
        p.fx=self.fields["fx"].value(); p.fy=self.fields["fy"].value(); p.cx=self.fields["cx"].value(); p.cy=self.fields["cy"].value()
        p.pan_zero_deg=self.fields["pan_zero_deg"].value(); p.tilt_zero_deg=self.fields["tilt_zero_deg"].value()
        p.boresight_ypr_deg=[self.fields[x].value() for x in ("yaw","pitch","roll")]
        p.camera_center_xyz_m=[self.fields[x].value() for x in ("tx","ty","tz")]
        count=4 if p.lens_model==LensModel.FISHEYE else 8; p.distortion=[self.fields[f"d{i}"].value() for i in range(count)]

    @staticmethod
    def _rotation_options() -> dict[str,np.ndarray]:
        result={}; axes="XYZ"
        for order in permutations(range(3)):
            for signs in product((-1,1),repeat=3):
                matrix=np.zeros((3,3))
                for row,(column,sign) in enumerate(zip(order,signs,strict=True)):matrix[row,column]=sign
                if np.linalg.det(matrix)>0:
                    label=",".join(("+" if signs[row]>0 else "-")+axes[column] for row,column in enumerate(order)); result[label]=matrix
        return result

    def _select_matrix(self,combo: QComboBox,value) -> None:
        for label,matrix in self.rotations.items():
            if np.allclose(matrix,value):combo.setCurrentText(label); return

    @staticmethod
    def _axis_text(axis,sign: float) -> str:
        vector=np.asarray(axis)*sign; index=int(np.argmax(np.abs(vector))); return ("+" if vector[index]>=0 else "-")+"XYZ"[index]

    @staticmethod
    def _axis_value(text: str) -> tuple[list[float],float]:
        axis=[0.,0.,0.]; axis["XYZ".index(text[1])]=1.; return axis,1. if text[0]=="+" else -1.

    def _reset_fov(self) -> None:
        import math
        focal=self.profile.image_width/(2*math.radians(60)) if self.lens.currentText()==LensModel.FISHEYE.value else self.profile.image_width/(2*math.tan(math.radians(60)))
        self.fields["fx"].setValue(focal); self.fields["fy"].setValue(focal); self.refresh()

    def _reset_rig_pose(self) -> None:
        """Restore the measured mobile-PLY convention for the supplied rig."""
        defaults=CameraProfile()
        self._select_matrix(self.axis_map,defaults.axis_map)
        self._select_matrix(self.camera_basis,defaults.camera_basis)
        self.pan_axis.setCurrentText(self._axis_text(defaults.pan_axis,defaults.pan_sign))
        self.tilt_axis.setCurrentText(self._axis_text(defaults.tilt_axis,defaults.tilt_sign))
        self.rotation_order.setCurrentText(defaults.rotation_order)
        if self.scan.session.image_pose_offset_frames is None:
            self.pose_offset.setValue(defaults.image_pose_offset_frames)
        self.fields["pan_zero_deg"].setValue(0.0)
        self.fields["tilt_zero_deg"].setValue(0.0)
        for name in ("yaw","pitch","roll"):
            self.fields[name].setValue(0.0)
        self.refresh()

    def _add_correspondence(self) -> None:
        row=self.corr_table.rowCount(); self.corr_table.insertRow(row)
        values=["0","0","0","","",str(self.shot_combo.currentData())]
        for column,value in enumerate(values):self.corr_table.setItem(row,column,QTableWidgetItem(value))
        self.corr_table.setCurrentCell(row,0)

    def _remove_correspondence(self) -> None:
        row=self.corr_table.currentRow()
        if row>=0:
            self.corr_table.removeRow(row)
            self._source_marker=None
            self.refresh()

    def _begin_lidar_pick(self) -> None:
        self.show_points.setChecked(True)
        self._pick_mode="lidar"
        self.pick_status.setText("Click a projected LiDAR dot on a sharp physical edge or corner.")

    def _begin_image_pick(self) -> None:
        row=self.corr_table.currentRow()
        if row < 0:
            self.pick_status.setText("Pick a projected LiDAR point first.")
            return
        self._pick_mode="image"
        self.show_points.setChecked(False)
        self.pick_status.setText("Projected dots are hidden. Click the same physical edge/corner in the photograph.")

    def _handle_overlay_click(self,u: float,v: float) -> None:
        if self._pick_mode == "lidar":
            frame=self._overlay_frame
            if frame is None or not len(frame.projected_uv):
                self.pick_status.setText("No visible projected LiDAR points are available in this image.")
                return
            distances=np.linalg.norm(frame.projected_uv-np.array([u,v]),axis=1)
            nearest=int(np.argmin(distances))
            if distances[nearest] > max(18.0,4.0*self.point_size.value()):
                self.pick_status.setText("Click closer to a visible projected dot (within about 18 pixels).")
                return
            row=self.corr_table.rowCount(); self.corr_table.insertRow(row)
            point=frame.points_scan[nearest]
            values=[f"{value:.8f}" for value in point]+["","",str(self.shot_combo.currentData())]
            for column,value in enumerate(values):self.corr_table.setItem(row,column,QTableWidgetItem(value))
            self.corr_table.setCurrentCell(row,3)
            source_uv=frame.projected_uv[nearest]
            self._source_marker=(row,int(self.shot_combo.currentData()),float(source_uv[0]),float(source_uv[1]))
            self._pick_mode="image"
            self.show_points.setChecked(False)
            self.pick_status.setText("Now click where that exact physical point belongs in the clean photograph.")
            self.refresh()
            return
        if self._pick_mode != "image":
            self.pick_status.setText("Choose step 1 or step 2 before clicking the image.")
            return
        row=self.corr_table.currentRow()
        if row<0:
            self.pick_status.setText("Pick a projected LiDAR point first.")
            return
        row_shot=self.corr_table.item(row,5)
        if row_shot is None or int(float(row_shot.text())) != int(self.shot_combo.currentData()):
            self.pick_status.setText("Return to the photograph where this LiDAR point was selected.")
            return
        self.corr_table.setItem(row,3,QTableWidgetItem(f"{u:.3f}")); self.corr_table.setItem(row,4,QTableWidgetItem(f"{v:.3f}")); self.corr_table.setItem(row,5,QTableWidgetItem(str(self.shot_combo.currentData())))
        self._pick_mode=None
        self.show_points.setChecked(True)
        self.pick_status.setText("Pair saved. Change image and repeat; use near and far edges in upward and downward views.")
        self.refresh()

    def _optimize(self) -> None:
        try:
            self._apply_fields()
            correspondences=[]
            poses={image.index:pose for image,pose in image_pose_pairs(self.scan,self.profile.image_pose_offset_frames,self._azimuth_scale)}
            for row in range(self.corr_table.rowCount()):
                cells=[self.corr_table.item(row,column) for column in range(6)]
                if any(cell is None or not cell.text().strip() for cell in cells):
                    raise ValueError(f"Correspondence row {row + 1} is incomplete")
                values=[float(cell.text()) for cell in cells if cell is not None]
                image_index=int(values[5])
                if image_index not in poses:raise ValueError(f"Image {image_index:03d} has no associated pose")
                correspondences.append(CalibrationCorrespondence(tuple(values[:3]),tuple(values[3:5]),poses[image_index]))
            groups=set()
            if self.optimize_intrinsics.isChecked():groups.add("intrinsics")
            if self.optimize_distortion.isChecked():groups.add("distortion")
            if self.optimize_extrinsics.isChecked():groups.update({"boresight","translation"})
            if self.optimize_angle_zeros.isChecked():groups.add("angle_zeros")
            result=optimize_profile(self.profile,correspondences,groups)
            if not result.success:
                raise ValueError(f"Solver did not converge; profile was not changed: {result.message}")
            if result.rms_pixels >= result.initial_rms_pixels:
                raise ValueError(
                    f"Fit did not improve ({result.initial_rms_pixels:.2f} → {result.rms_pixels:.2f} px); "
                    "profile was not changed. Check the selected point pairs."
                )
            self.profile=result.profile
            values={"fx":self.profile.fx,"fy":self.profile.fy,"cx":self.profile.cx,"cy":self.profile.cy,"pan_zero_deg":self.profile.pan_zero_deg,"tilt_zero_deg":self.profile.tilt_zero_deg,
                    "yaw":self.profile.boresight_ypr_deg[0],"pitch":self.profile.boresight_ypr_deg[1],"roll":self.profile.boresight_ypr_deg[2],"tx":self.profile.camera_center_xyz_m[0],"ty":self.profile.camera_center_xyz_m[1],"tz":self.profile.camera_center_xyz_m[2]}
            values.update({f"d{i}":value for i,value in enumerate((self.profile.distortion+[0]*8)[:8])})
            for key,value in values.items():self.fields[key].setValue(value)
            self.refresh(); QMessageBox.information(
                self,
                "Calibration optimized",
                f"RMS reprojection error: {result.initial_rms_pixels:.3f} → {result.rms_pixels:.3f} px\n{result.message}",
            )
        except Exception as exc:QMessageBox.critical(self,"Cannot optimize calibration",str(exc))

    def refresh(self) -> None:
        try:
            self._apply_fields(); shot=int(self.shot_combo.currentData())
            self._overlay_frame=calibration_overlay_frame(self.scan,self.profile,shot,opacity=self.opacity.value(),point_radius=self.point_size.value(),source_path=self._leveled_path,azimuth_scale=self._azimuth_scale)
            image=(self._overlay_frame.overlay_rgb if self.show_points.isChecked() else self._overlay_frame.image_rgb).copy()
            if self._source_marker is not None and self._source_marker[1] == shot:
                _,_,u,v=self._source_marker
                cv2.circle(image,(round(u),round(v)),8,(0,255,255),2,cv2.LINE_AA)
                cv2.drawMarker(image,(round(u),round(v)),(0,255,255),cv2.MARKER_CROSS,12,2,cv2.LINE_AA)
            for row in range(self.corr_table.rowCount()):
                shot_item=self.corr_table.item(row,5); u_item=self.corr_table.item(row,3); v_item=self.corr_table.item(row,4)
                if not shot_item or not u_item or not v_item or not u_item.text() or not v_item.text():continue
                if int(float(shot_item.text())) != shot:continue
                target=(round(float(u_item.text())),round(float(v_item.text())))
                cv2.drawMarker(image,target,(255,0,255),cv2.MARKER_TILTED_CROSS,12,2,cv2.LINE_AA)
            qimage=QImage(image.data,image.shape[1],image.shape[0],image.strides[0],QImage.Format.Format_RGB888).copy()
            self.image_label.setPixmap(QPixmap.fromImage(qimage))
        except Exception as exc:
            self.image_label.setText(str(exc))

    def save_as(self) -> None:
        self._apply_fields(); path,_=QFileDialog.getSaveFileName(self,"Save calibration profile","camera_profile.json","JSON (*.json)")
        if path:
            try: save_profile(self.profile,Path(path))
            except Exception as exc: QMessageBox.critical(self,"Cannot save profile",str(exc))

    def load_from(self) -> None:
        path,_=QFileDialog.getOpenFileName(self,"Load calibration profile","","JSON (*.json)")
        if not path:return
        try:
            self.profile=load_profile(path); p=self.profile
            self.lens.setCurrentText(p.lens_model.value); self._select_matrix(self.axis_map,p.axis_map); self._select_matrix(self.camera_basis,p.camera_basis)
            self.pan_axis.setCurrentText(self._axis_text(p.pan_axis,p.pan_sign)); self.tilt_axis.setCurrentText(self._axis_text(p.tilt_axis,p.tilt_sign)); self.rotation_order.setCurrentText(p.rotation_order)
            if self.scan.session.image_pose_offset_frames is None:self.pose_offset.setValue(p.image_pose_offset_frames)
            values={"fx":p.fx,"fy":p.fy,"cx":p.cx,"cy":p.cy,"pan_zero_deg":p.pan_zero_deg,"tilt_zero_deg":p.tilt_zero_deg,
                    "yaw":p.boresight_ypr_deg[0],"pitch":p.boresight_ypr_deg[1],"roll":p.boresight_ypr_deg[2],"tx":p.camera_center_xyz_m[0],"ty":p.camera_center_xyz_m[1],"tz":p.camera_center_xyz_m[2]}
            values.update({f"d{i}":value for i,value in enumerate((p.distortion+[0]*8)[:8])})
            for key,value in values.items():self.fields[key].setValue(value)
            self.refresh()
        except Exception as exc:QMessageBox.critical(self,"Cannot load profile",str(exc))

    def accept(self) -> None:
        self._apply_fields(); super().accept()
