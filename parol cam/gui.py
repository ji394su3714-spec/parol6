# gui.py
import numpy as np
from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QMessageBox, QFileDialog, QStackedWidget, QTabBar
from PySide6.QtCore import Qt, QVariantAnimation, QAbstractAnimation
from PySide6.QtGui import Qt, QCursor 
from vispy import scene

import styles
from widgets import PreparePanelWidget, PreviewPanelWidget, FloatingToolbarWidget
from picker import MeshPicker 
from config import Robot3DView
from tcp_manager import TCPManager 
from tcp_dialog import TCPManagerDialog 
from core_engine import CAMEngine         
from processor import PostProcessor

class ParolCamWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Parol CAM")
        self.resize(1250, 850)
            
        self.tcp_manager = TCPManager()
        self.cam_engine = CAMEngine()
        self.post_processor = PostProcessor(self.tcp_manager)
        
        self.current_joints = [0.0] * 6
        self.baked_trajectory = [] 
        self.current_script_data = None 
        
        self.robot_view = None
        self.mesh_visual = None
        self.path_visual = None          
        self.path_marker_visual = None
        self.path_visuals = []   
        self.tcp_axes = []

        self._setup_ui()
        self.zero_robot_joints()
        
        self.hover_visual = scene.visuals.Line(
            color='#00FFFF', width=3, antialias=True, method='gl', parent=self.robot_view.view.scene
        )
        self.hover_visual.visible = False

        self.mesh_picker = MeshPicker(self.cam_engine, self.robot_view, self.on_mesh_picked, self.on_mesh_hovered)

    def _setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        self.main_v_layout = QVBoxLayout(central_widget)
        self.main_v_layout.setContentsMargins(0, 0, 0, 0)
        self.main_v_layout.setSpacing(0)

        tab_layout = QHBoxLayout()
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.setSpacing(0)
        
        self.tab_bar = QTabBar()
        self.tab_bar.setShape(QTabBar.RoundedNorth)
        self.tab_bar.setStyleSheet(styles.STYLE_TAB_BAR)
        self.tab_bar.addTab("主頁")
        self.tab_bar.addTab("準備")
        self.tab_bar.addTab("預覽")
        self.tab_bar.addTab("設備")
        
        tab_layout.addWidget(self.tab_bar)
        tab_layout.addStretch() 
        self.main_v_layout.addLayout(tab_layout)

        workspace_layout = QHBoxLayout()
        workspace_layout.setContentsMargins(0, 0, 0, 0)
        workspace_layout.setSpacing(0)
        self.main_v_layout.addLayout(workspace_layout, stretch=1)

        self.left_panel = QStackedWidget()
        self.left_panel.setFixedWidth(360)  
        self.left_panel.setStyleSheet(styles.STYLE_LEFT_PANEL)
        
        self.page_main = QWidget()
        self.prepare_panel = PreparePanelWidget() 
        self.preview_panel = PreviewPanelWidget() 
        self.page_device = QWidget()
        
        self.left_panel.addWidget(self.page_main)
        self.left_panel.addWidget(self.prepare_panel)
        self.left_panel.addWidget(self.preview_panel)
        self.left_panel.addWidget(self.page_device)
        workspace_layout.addWidget(self.left_panel)

        self.right_panel = QWidget()
        right_layout = QVBoxLayout(self.right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        self.robot_view = Robot3DView() 
        self.toolbar = FloatingToolbarWidget(self.robot_view.view.canvas.native)
        
        overlay_layout = QHBoxLayout(self.robot_view.view.canvas.native)
        overlay_layout.setContentsMargins(0,0,0,0)
        overlay_layout.addWidget(self.toolbar, 0, Qt.AlignTop | Qt.AlignLeft)
        
        right_layout.addWidget(self.robot_view.view.canvas.native)
        workspace_layout.addWidget(self.right_panel, stretch=1)

        # ===============================================
        # 信號綁定區
        # ===============================================
        self.tab_bar.currentChanged.connect(self.on_tab_changed)
        
        self.prepare_panel.btn_tcp_config.clicked.connect(self.open_tcp_dialog)
        self.prepare_panel.btn_zero_joints.clicked.connect(self.zero_robot_joints)
        self.prepare_panel.cb_tool_select.currentIndexChanged.connect(self.change_active_tool)
        self.prepare_panel.btn_load_stl.clicked.connect(self.on_load_stl_clicked)
        self.prepare_panel.spin_safe_z.editingFinished.connect(self.update_3d_path)
        self.prepare_panel.btn_path_merge.clicked.connect(self.on_path_merge)
        
        for spin in self.prepare_panel.model_spins:
            spin.valueChanged.connect(self.update_model_transform)
            
        # 將原本的 offset 與新增的進階參數，統一綁定重算事件
        for spin in self.prepare_panel.offset_spins:
            spin.valueChanged.connect(self.recalculate_all_segments)
            
        advanced_spins = [
            self.prepare_panel.spin_chordal,
            self.prepare_panel.spin_max_step,
            self.prepare_panel.spin_lead_dist,
            self.prepare_panel.spin_lead_angle,
            self.prepare_panel.spin_overcut
        ]
        for spin in advanced_spins:
            spin.editingFinished.connect(self.recalculate_all_segments)
            
        self.prepare_panel.cb_align_mode.currentIndexChanged.connect(self.on_align_mode_changed)
        self.prepare_panel.btn_path_del.clicked.connect(self.on_path_delete)
        self.prepare_panel.btn_path_up.clicked.connect(self.on_path_up)
        self.prepare_panel.btn_path_down.clicked.connect(self.on_path_down)
        
        self.preview_panel.btn_bake_ik.clicked.connect(self.on_bake_ik_clicked)
        self.preview_panel.btn_play.clicked.connect(self.toggle_playback)
        self.preview_panel.slider.valueChanged.connect(self.on_slider_moved)
        self.preview_panel.btn_save_script.clicked.connect(self.on_save_script_clicked)
        
        self.toolbar.btn_magic.toggled.connect(self.on_magic_wand_toggled)
        self.toolbar.cb_mode.currentIndexChanged.connect(
            lambda idx: setattr(self.cam_engine, 'extraction_mode', 'planar' if idx == 0 else '3d_feature')
        )
        self.toolbar.cb_loop_mode.currentIndexChanged.connect(self.recalculate_all_segments)
        self.anim = QVariantAnimation()
        self.anim.valueChanged.connect(self.update_animation_frame)
        self.anim.stateChanged.connect(self.on_animation_state_changed)
        
        self.refresh_tool_list()
        self.tab_bar.setCurrentIndex(1)

    def on_tab_changed(self, index):
        self.left_panel.setCurrentIndex(index)
        if index == 1: self.toolbar.show()
        else: self.toolbar.hide()

    def get_adv_params(self):
        return {
            'lead_in_dist': self.prepare_panel.spin_lead_dist.value(),
            'lead_in_angle': self.prepare_panel.spin_lead_angle.value(),
            'overcut_dist': self.prepare_panel.spin_overcut.value(),
            'chordal_error': self.prepare_panel.spin_chordal.value(), 
            'max_step': self.prepare_panel.spin_max_step.value(),
            'loop_mode': self.toolbar.cb_loop_mode.currentIndex() # 0=自動, 1=封閉, 2=開放
        }

    # =========================================================
    # 連續路徑渲染邏輯
    # =========================================================
    def update_3d_path(self):
        safe_z = self.prepare_panel.spin_safe_z.value()
        tagged_path = self.cam_engine.get_linked_path(safe_z)
        self.cam_engine.current_tagged_path = tagged_path

        for v in self.path_visuals: v.parent = None
        self.path_visuals.clear()
        for ax in self.tcp_axes: ax.parent = None
        self.tcp_axes.clear()

        if not tagged_path: return

        prev_pt = None
        for seg in tagged_path:
            if seg['type'] in ['RETRACT', 'APPROACH', 'PTP_APPROACH', 'PLUNGE']:
                pt = seg['location'] / 1000.0
                if prev_pt is not None:
                    color = '#FF00FF' if seg['type'] == 'PLUNGE' else '#00FFFF'
                    line = scene.visuals.Line(pos=np.array([prev_pt, pt]), color=color, width=2, parent=self.robot_view.view.scene)
                    self.path_visuals.append(line)
                prev_pt = pt

            elif seg['type'] == 'CUT':
                locs = seg['locations'] / 1000.0
                mats = seg['matrices']
                
                line = scene.visuals.Line(pos=locs, color='#FFFF00', width=2, antialias=True, connect='strip', parent=self.robot_view.view.scene)
                markers = scene.visuals.Markers(pos=locs, size=5, face_color='#FFFF00', edge_width=0, parent=self.robot_view.view.scene)
                self.path_visuals.extend([line, markers])
                prev_pt = locs[-1]

                step = max(1, len(locs) // 15)
                for i in range(0, len(mats), step):
                    T_tcp = mats[i]
                    tcp_axis = scene.visuals.XYZAxis(parent=self.robot_view.view.scene)
                    transform = scene.transforms.MatrixTransform()
                    draw_matrix = np.eye(4)
                    draw_matrix[:3, :3] = T_tcp[:3, :3] * 0.03 
                    draw_matrix[:3, 3] = T_tcp[:3, 3] / 1000.0 
                    transform.matrix = draw_matrix.T 
                    tcp_axis.transform = transform
                    self.tcp_axes.append(tcp_axis)

    def on_align_mode_changed(self, index):
        self.cam_engine.tcp_align_mode = "minimum_twist" if index == 0 else "tangent"
        self.recalculate_all_segments()

    def recalculate_all_segments(self):
        if not self.cam_engine.segments: return 
        rx, ry, rz = [spin.value() for spin in self.prepare_panel.offset_spins]
        adv_params = self.get_adv_params()
        
        try:
            # 呼叫核心重建
            self.cam_engine.rebuild_all_paths(
                offset_rx=rx, offset_ry=ry, offset_rz=rz, **adv_params
            )
            self.prepare_panel.list_paths.clear()
            for seg in self.cam_engine.segments:
                self.prepare_panel.list_paths.addItem(seg['name'])
                
            if self.prepare_panel.list_paths.count() > 0:
                self.prepare_panel.list_paths.setCurrentRow(self.prepare_panel.list_paths.count() - 1)
            self.update_3d_path()
        except Exception as e: 
            print(f"重算失敗: {e}")

    def on_path_merge(self):
        selected = [item.row() for item in self.prepare_panel.list_paths.selectedIndexes()]
        if len(selected) < 2:
            QMessageBox.warning(self, "提示", "請選擇至少兩條路徑！")
            return
            
        rx, ry, rz = [spin.value() for spin in self.prepare_panel.offset_spins]
        adv_params = self.get_adv_params()
        
        try:
            # 嘗試合併，若未相接將會觸發 ValueError
            self.cam_engine.merge_segments(selected, offset_rx=rx, offset_ry=ry, offset_rz=rz, **adv_params)
            
            self.prepare_panel.list_paths.clear()
            for seg in self.cam_engine.segments:
                self.prepare_panel.list_paths.addItem(seg['name'])
            self.prepare_panel.list_paths.setCurrentRow(self.prepare_panel.list_paths.count() - 1)
            self.update_3d_path()
            
        except ValueError as e:
            # 攔截錯誤並彈出警告視窗
            QMessageBox.critical(self, "合併失敗", str(e))
        except Exception as e:
            QMessageBox.critical(self, "系統錯誤", str(e))

    def on_path_delete(self):
        row = self.prepare_panel.list_paths.currentRow()
        if row >= 0:
            self.cam_engine.delete_segment(row)
            self.prepare_panel.list_paths.takeItem(row)
            self.update_3d_path()

    def on_path_up(self):
        row = self.prepare_panel.list_paths.currentRow()
        if row > 0:
            self.cam_engine.move_segment_up(row)
            item = self.prepare_panel.list_paths.takeItem(row)
            self.prepare_panel.list_paths.insertItem(row - 1, item)
            self.prepare_panel.list_paths.setCurrentRow(row - 1)
            self.update_3d_path()

    def on_path_down(self):
        row = self.prepare_panel.list_paths.currentRow()
        if row >= 0 and row < self.prepare_panel.list_paths.count() - 1:
            self.cam_engine.move_segment_down(row)
            item = self.prepare_panel.list_paths.takeItem(row)
            self.prepare_panel.list_paths.insertItem(row + 1, item)
            self.prepare_panel.list_paths.setCurrentRow(row + 1)
            self.update_3d_path()

    def on_mesh_hovered(self, boundary_pts):
        if boundary_pts is not None:
            self.hover_visual.set_data(pos=boundary_pts / 1000.0)
            self.hover_visual.visible = True
        else:
            self.hover_visual.visible = False

    def on_mesh_picked(self, hit_face_idx, hit_pos):
        self.hover_visual.visible = False 
        rx, ry, rz = [spin.value() for spin in self.prepare_panel.offset_spins]
        adv_params = self.get_adv_params()
        
        try:
            name = self.cam_engine.add_segment(
                hit_face_idx, hit_pos, 
                offset_rx=rx, offset_ry=ry, offset_rz=rz,
                **adv_params
            )
            self.prepare_panel.list_paths.addItem(name)
            self.prepare_panel.list_paths.setCurrentRow(self.prepare_panel.list_paths.count() - 1)
            self.update_3d_path()
        except Exception as e:
            QMessageBox.critical(self, "選取錯誤", str(e))

    def on_magic_wand_toggled(self, checked):
        if self.cam_engine.workpiece_mesh is None:
            self.toolbar.btn_magic.blockSignals(True)
            self.toolbar.btn_magic.setChecked(False)
            self.toolbar.btn_magic.blockSignals(False)
            QMessageBox.warning(self, "警告", "請先載入 STL 檔案！")
            return

        if checked:
            self.mesh_picker.toggle_picking_mode(True)
            self.robot_view.canvas.native.setCursor(QCursor(Qt.CrossCursor))
        else:
            self.mesh_picker.toggle_picking_mode(False)
            self.robot_view.canvas.native.setCursor(QCursor(Qt.ArrowCursor))

    def on_bake_ik_clicked(self):
        if not hasattr(self.cam_engine, 'current_tagged_path') or not self.cam_engine.current_tagged_path:
            QMessageBox.warning(self, "警告", "請先建立特徵路徑！")
            return

        try:
            self.preview_panel.lbl_bake_status.setText("狀態：執行逆向運動學運算中...")
            self.repaint() 

            # 擷取 UI 上的過濾門檻與速度動態參數
            min_step_val = self.prepare_panel.spin_min_step.value()
            c_spd = self.preview_panel.spin_cut_speed.value()
            c_acc = self.preview_panel.spin_cut_accel.value()
            m_spd = self.preview_panel.spin_move_speed.value()
            m_acc = self.preview_panel.spin_move_accel.value()

            # 將參數往下傳給處理器
            start_joints, script_data, preview_joints = self.post_processor.bake_trajectory(
                self.cam_engine.current_tagged_path, 
                min_step_mm=min_step_val,
                cut_speed=c_spd,
                cut_accel=c_acc,
                move_speed=m_spd,
                move_accel=m_acc
            )
            
            self.current_script_data = script_data 
            self.baked_trajectory = preview_joints 
            
            total_points = len(self.baked_trajectory)
            self.preview_panel.slider.setEnabled(True)
            self.preview_panel.btn_play.setEnabled(True)
            self.preview_panel.slider.setMaximum(max(0, total_points - 1))
            self.preview_panel.slider.setValue(0)
            self.preview_panel.lbl_bake_status.setText(f"狀態：編譯成功！(共 {total_points} 點)")
            
            QMessageBox.information(self, "編譯成功", "腳本編譯完成！(資料已暫存於記憶體)\n請點擊「另存腳本」匯出。")
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.preview_panel.lbl_bake_status.setText("狀態：運算失敗！")
            QMessageBox.critical(self, "IK 運算失敗", str(e))

    def on_save_script_clicked(self):
        if not self.current_script_data:
            QMessageBox.warning(self, "警告", "尚未產生軌跡資料！請先點擊「計算 IK 並編譯腳本」。")
            return
            
        filename, _ = QFileDialog.getSaveFileName(self, "儲存 CAM 腳本", "parol_cam_export.json", "JSON Files (*.json)")
        if filename:
            try:
                import re
                import json
                raw_json = json.dumps(self.current_script_data, indent=4, ensure_ascii=False)
                compact_json = re.sub(
                    r'\[\s+([-0-9.eE]+(?:,\s*[-0-9.eE]+)*)\s+\]',
                    lambda m: '[' + re.sub(r'\s+', '', m.group(1)).replace(',', ', ') + ']',
                    raw_json
                )
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(compact_json)
                    
                QMessageBox.information(self, "成功", f"工業腳本已成功匯出至：\n{filename}")
            except Exception as e:
                QMessageBox.critical(self, "儲存失敗", f"寫入檔案時發生錯誤：\n{str(e)}")

    def toggle_playback(self):
        if not self.baked_trajectory: return

        if self.anim.state() == QAbstractAnimation.Running:
            self.anim.pause()
            self.preview_panel.btn_play.setText("▶️ 繼續")
        else:
            current_idx = self.preview_panel.slider.value()
            max_idx = self.preview_panel.slider.maximum()
            
            if current_idx >= max_idx:
                current_idx = 0
                self.preview_panel.slider.setValue(0)
                
            self.anim.setStartValue(float(current_idx))
            self.anim.setEndValue(float(max_idx))
            
            remaining_points = max_idx - current_idx
            self.anim.setDuration(max(100, remaining_points * 10)) 
            self.anim.start()
            self.preview_panel.btn_play.setText("⏸ 暫停")

    def on_animation_state_changed(self, state):
        if state == QAbstractAnimation.Stopped:
            self.preview_panel.btn_play.setText("▶️ 播放預覽")

    def update_animation_frame(self, frame_index):
        idx = int(round(frame_index))
        self.preview_panel.slider.blockSignals(True)
        self.preview_panel.slider.setValue(idx)
        self.preview_panel.lbl_frame.setText(f"{idx} / {len(self.baked_trajectory)-1}")
        self.preview_panel.slider.blockSignals(False)
        self.robot_view.update_joints(self.baked_trajectory[idx], tcp_mat=self.tcp_manager.get_active_matrix())

    def on_slider_moved(self, value):
        if not self.baked_trajectory: return
        self.preview_panel.lbl_frame.setText(f"{value} / {len(self.baked_trajectory)-1}")
        self.robot_view.update_joints(self.baked_trajectory[value], tcp_mat=self.tcp_manager.get_active_matrix())

    def refresh_tool_list(self):
        self.prepare_panel.cb_tool_select.blockSignals(True)
        self.prepare_panel.cb_tool_select.clear()
        for tool in self.tcp_manager.tools:
            self.prepare_panel.cb_tool_select.addItem(tool.get("name", "Unknown Tool"))
        self.prepare_panel.cb_tool_select.setCurrentIndex(self.tcp_manager.current_index)
        self.prepare_panel.cb_tool_select.blockSignals(False)

    def change_active_tool(self, index):
        if index >= 0:
            self.tcp_manager.set_current_index(index)
            self.robot_view.update_joints(self.current_joints, tcp_mat=self.tcp_manager.get_active_matrix())

    def open_tcp_dialog(self):
        dialog = TCPManagerDialog(self.tcp_manager, self)
        dialog.exec()
        self.robot_view.update_joints(self.current_joints, tcp_mat=self.tcp_manager.get_active_matrix())
        self.refresh_tool_list()

    def zero_robot_joints(self):
        self.current_joints = [0.0] * 6
        self.robot_view.update_joints(self.current_joints, tcp_mat=self.tcp_manager.get_active_matrix())

    def update_model_transform(self):
        if self.cam_engine.workpiece_mesh is None: return 
        x, y, z = [spin.value() for spin in self.prepare_panel.model_spins[:3]]
        rx, ry, rz = [spin.value() for spin in self.prepare_panel.model_spins[3:]]
        mesh = self.cam_engine.apply_model_transform(x, y, z, rx, ry, rz)
        
        if self.mesh_visual: self.mesh_visual.set_data(vertices=mesh.vertices / 1000.0, faces=mesh.faces)
        if self.path_visual: self.path_visual.parent = None; self.path_visual = None
        if self.path_marker_visual: self.path_marker_visual.parent = None; self.path_marker_visual = None
        for ax in self.tcp_axes: ax.parent = None
        self.tcp_axes.clear()

    def on_load_stl_clicked(self):
        filepath, _ = QFileDialog.getOpenFileName(self, "選擇 STL 檔案", "", "STL Files (*.stl);;All Files (*)")
        if not filepath: return 
        try:
            mesh = self.cam_engine.load_stl_mesh(filepath)
            
            # 載入成功後，立刻取得面數並更新 UI 顯示
            face_count = len(mesh.faces)
            self.prepare_panel.lbl_face_count.setText(f"網格面數: {face_count:,} 面")
            
            if self.mesh_visual: self.mesh_visual.parent = None
            self.mesh_visual = scene.visuals.Mesh(
                vertices=mesh.vertices / 1000.0, faces=mesh.faces, 
                color=(0.5, 0.5, 0.5, 0.8), shading='flat', parent=self.robot_view.view.scene
            )
            self.robot_view.view.camera.set_range(margin=0.2)
            self.update_model_transform()
        except Exception as e:
            QMessageBox.critical(self, "錯誤", str(e))