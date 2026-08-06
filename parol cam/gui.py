# gui.py
import sys
import os       
import re       
import json     
import numpy as np
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QMessageBox, 
                               QFileDialog, QStackedWidget, QTabBar, QMenu, QWidgetAction, QGridLayout,
                               QPushButton, QLabel)
from PySide6.QtCore import Qt, QVariantAnimation, QAbstractAnimation, QObject, Signal
from PySide6.QtGui import QCursor, QAction 
from vispy import scene

import styles
from widgets import PreparePanelWidget, PreviewPanelWidget, FloatingToolbarWidget, LogPanelWidget, ModelTransformWidget, FloatingPlaybackWidget
from picker import MeshPicker 
from config import Robot3DView
from tcp_manager import TCPManager 
from tcp_dialog import TCPManagerDialog 
from core_engine import CAMEngine         
from processor import PostProcessor

class EmittingStream(QObject):
    textWritten = Signal(str)
    def write(self, text):
        self.textWritten.emit(str(text))
    def flush(self):
        pass

class ParolCamWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Parol CAM")
        self.resize(1400, 700) 
        
        self.tcp_manager = TCPManager()
        self.cam_engine = CAMEngine()
        self.post_processor = PostProcessor(self.tcp_manager)
        
        self.current_joints = [0.0] * 6
        self.baked_trajectory = [] 
        self.current_script_data = None 
        
        self.current_project_stl = None
        self.is_script_outdated = False 
        
        self.robot_view = None
        self.mesh_visual = None
        self.path_visual = None          
        self.path_marker_visual = None
        self.path_visuals = []   
        self.tcp_axes = []
        self._setup_ui()
        
        self.sys_logger = EmittingStream()
        sys.stdout = self.sys_logger 
        sys.stderr = self.sys_logger 
        self.sys_logger.textWritten.connect(self.log_panel.append_log)
        #print("[System] 系統啟動成功，日誌模組已掛載。")
        
        self.zero_robot_joints()
        
        self.hover_visual = scene.visuals.Line(
            color='#00FFFF', width=3, antialias=True, method='gl', parent=self.robot_view.view.scene
        )
        self.hover_visual.visible = False

        # 這是新增的實體選取外框
        self.bbox_visual = scene.visuals.Line(
            connect='segments', color='#aaaaaa', width=2, antialias=True, method='gl', parent=self.robot_view.view.scene
        )
        self.bbox_visual.visible = False

        self.mesh_picker = MeshPicker(self.cam_engine, self.robot_view, self.on_mesh_picked, self.on_mesh_hovered)
        
        # 左鍵點擊事件綁定
        self.robot_view.view.canvas.events.mouse_press.connect(self.on_canvas_clicked)

    def _setup_ui(self):
        self.setStyleSheet(styles.STYLE_MAIN_WINDOW)
        
        central_widget = QWidget()
        central_widget.setObjectName("CentralWidget")
        self.setCentralWidget(central_widget)
        
        self.main_v_layout = QVBoxLayout(central_widget)
        self.main_v_layout.setContentsMargins(0, 0, 0, 0) 
        self.main_v_layout.setSpacing(0)

        self.top_bar_widget = QWidget()
        self.top_bar_widget.setObjectName("TopBar")
        self.top_bar_widget.setStyleSheet(styles.STYLE_TOP_BAR)
        top_layout = QHBoxLayout(self.top_bar_widget)
        top_layout.setContentsMargins(10, 0, 10, 0)
        top_layout.setSpacing(0)
        
        self.tab_bar = QTabBar()
        self.tab_bar.setShape(QTabBar.RoundedNorth)
        self.tab_bar.setStyleSheet(styles.STYLE_TAB_BAR)
        self.tab_bar.addTab("主頁")
        self.tab_bar.addTab("準備")
        self.tab_bar.addTab("預覽")
        self.tab_bar.addTab("設備")
        
        top_layout.addWidget(self.tab_bar)
        top_layout.addStretch() 
        
        self.btn_load_proj = QPushButton("載入專案")
        self.btn_save_proj = QPushButton("儲存專案")
        btn_style = """
            QPushButton { background-color: transparent; color: #ccc; border: 1px solid #444; border-radius: 4px; padding: 4px 12px; font-weight: bold; }
            QPushButton:hover { background-color: #333; color: white; border: 1px solid #666; }
            QPushButton:pressed { background-color: #007acc; border: 1px solid #007acc; }
        """
        self.btn_load_proj.setStyleSheet(btn_style)
        self.btn_save_proj.setStyleSheet(btn_style)
        top_layout.addWidget(self.btn_load_proj)
        top_layout.addWidget(self.btn_save_proj)
        
        self.main_v_layout.addWidget(self.top_bar_widget)

        workspace_wrapper = QWidget()
        workspace_layout = QHBoxLayout(workspace_wrapper)
        workspace_layout.setContentsMargins(4, 4, 4, 4) 
        workspace_layout.setSpacing(4) 
        self.main_v_layout.addWidget(workspace_wrapper, stretch=1)

        self.left_panel = QStackedWidget()
        self.left_panel.setObjectName("LeftPanel")
        self.left_panel.setFixedWidth(360)  
        self.left_panel.setStyleSheet(styles.STYLE_BLOCK)
        
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
        self.right_panel.setObjectName("RightPanel")
        self.right_panel.setStyleSheet(styles.STYLE_BLOCK)
        right_layout = QVBoxLayout(self.right_panel)
        right_layout.setContentsMargins(5, 5, 5, 5) 
        
        self.robot_view = Robot3DView() 
        self.toolbar = FloatingToolbarWidget(self.robot_view.view.canvas.native)
        self.model_transform_widget = ModelTransformWidget(self.robot_view.view.canvas.native)
        
        self.log_panel = LogPanelWidget()
        self.playback_bar = FloatingPlaybackWidget(self.robot_view.view.canvas.native)
        self.playback_bar.setVisible(False)
        
        # 核心修復：不使用 QWidget 容器，直接建立純粹的 QVBoxLayout
        left_ui_layout = QVBoxLayout()
        left_ui_layout.setContentsMargins(0, 0, 0, 0)
        left_ui_layout.setSpacing(10)
        
        left_ui_layout.addWidget(self.toolbar)
        left_ui_layout.addWidget(self.model_transform_widget)
        
        overlay_layout = QGridLayout(self.robot_view.view.canvas.native)
        overlay_layout.setContentsMargins(10, 10, 15, 15)
        
        # 將純 layout 直接加入 Grid，完美解決按鈕失效與穿透問題！
        overlay_layout.addLayout(left_ui_layout, 0, 0, Qt.AlignTop | Qt.AlignLeft)
        overlay_layout.addWidget(self.log_panel, 0, 0, Qt.AlignBottom | Qt.AlignRight)
        overlay_layout.addWidget(self.playback_bar, 0, 0, Qt.AlignBottom | Qt.AlignHCenter)
        
        right_layout.addWidget(self.robot_view.view.canvas.native)
        workspace_layout.addWidget(self.right_panel, stretch=1)

        self.status_bar = self.statusBar()
        self.status_bar.setStyleSheet(styles.STYLE_STATUS_BAR)
        
        self.status_label = QLabel("  🟢 System Ready (CAM Engine v2.0)  |  ")
        self.status_bar.addWidget(self.status_label)
        
        self.btn_toggle_log = QPushButton("系統日誌 (Log)")
        self.btn_toggle_log.setCheckable(True)
        self.btn_toggle_log.setCursor(Qt.PointingHandCursor)
        self.btn_toggle_log.setStyleSheet(styles.STYLE_STATUS_BTN)
        self.btn_toggle_log.toggled.connect(self.log_panel.toggle_log)
        self.status_bar.addPermanentWidget(self.btn_toggle_log)

        # ===============================================
        # 信號綁定區
        # ===============================================
        self.btn_save_proj.clicked.connect(self.on_save_project)
        self.btn_load_proj.clicked.connect(self.on_load_project)
        
        self.tab_bar.currentChanged.connect(self.on_tab_changed)
        
        self.toolbar.btn_tcp_config.clicked.connect(self.open_tcp_dialog)
        self.toolbar.btn_zero_joints.clicked.connect(self.zero_robot_joints)
        self.toolbar.cb_tool_select.currentIndexChanged.connect(self.change_active_tool)
        self.toolbar.btn_load_stl.clicked.connect(self.on_load_stl_clicked)
        
        self.prepare_panel.spin_safe_z.editingFinished.connect(self.update_3d_path)
        
        for spin in self.model_transform_widget.spins:
            spin.valueChanged.connect(self.update_model_transform)
            
        for spin in self.prepare_panel.offset_spins:
            spin.valueChanged.connect(self.on_params_changed)
            
        advanced_spins = [
            self.prepare_panel.spin_chordal,
            self.prepare_panel.spin_max_step,
            self.prepare_panel.spin_min_step,
            self.prepare_panel.spin_lead_dist,
            self.prepare_panel.spin_lead_angle,
            self.prepare_panel.spin_overcut
        ]
        for spin in advanced_spins:
            spin.editingFinished.connect(self.on_params_changed)
            
        # 將四個速度參數變動也綁定到髒標記，因為它們會影響腳本
        self.prepare_panel.spin_cut_speed.valueChanged.connect(self.mark_script_outdated)
        self.prepare_panel.spin_cut_accel.valueChanged.connect(self.mark_script_outdated)
        self.prepare_panel.spin_move_speed.valueChanged.connect(self.mark_script_outdated)
        self.prepare_panel.spin_move_accel.valueChanged.connect(self.mark_script_outdated)
            
        self.prepare_panel.cb_align_mode.currentIndexChanged.connect(self.on_params_changed)
        self.toolbar.cb_loop_mode.currentIndexChanged.connect(self.on_params_changed)
        
        self.prepare_panel.list_paths.itemSelectionChanged.connect(self.on_path_selection_changed)
        self.prepare_panel.btn_path_merge.clicked.connect(self.on_path_merge)
        self.prepare_panel.btn_path_del.clicked.connect(self.on_path_delete)
        self.prepare_panel.btn_path_up.clicked.connect(self.on_path_up)
        self.prepare_panel.btn_path_down.clicked.connect(self.on_path_down)
        
        self.preview_panel.btn_save_script.clicked.connect(self.on_save_script_clicked)
        
        self.playback_bar.btn_play.clicked.connect(self.toggle_playback)
        self.playback_bar.slider.valueChanged.connect(self.on_slider_moved)
        
        self.toolbar.btn_magic.toggled.connect(self.on_magic_wand_toggled)
        self.toolbar.cb_mode.currentIndexChanged.connect(
            lambda idx: setattr(self.cam_engine, 'extraction_mode', 'planar' if idx == 0 else '3d_feature')
        )
        self.anim = QVariantAnimation()
        self.anim.valueChanged.connect(self.update_animation_frame)
        self.anim.stateChanged.connect(self.on_animation_state_changed)
        
        self.refresh_tool_list()
        self.tab_bar.setCurrentIndex(1)

    # 新增：髒標記觸發器，改變 UI 警告文字
    def mark_script_outdated(self, *args):
        # 如果本來就是過時的，就不用重複洗頻印 Log
        if not self.is_script_outdated:
            self.is_script_outdated = True
            if self.current_script_data:
                print("\n⚠️ [Warning] 參數已修改，目前的軌跡腳本已過時！切換至「預覽」將自動重算。")

    def on_tab_changed(self, index):
        self.left_panel.setCurrentIndex(index)
        
        if index == 1: self.toolbar.show()
        else: self.toolbar.hide()
            
        if index == 2: 
            self.playback_bar.show()
            # 全自動化：切換到預覽時，如果參數有修改，系統直接自動重算！
            if self.is_script_outdated and hasattr(self.cam_engine, 'current_tagged_path') and self.cam_engine.current_tagged_path:
                print("[System] 偵測到參數變更，自動執行 IK 軌跡重算...")
                self.on_bake_ik_clicked()
        else: 
            self.playback_bar.hide()

    def update_3d_path(self):
        self.mark_script_outdated() # 任何路徑變更都會觸發過時
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
        self.on_params_changed()

    def get_current_ui_params(self):
        return {
            'offset_rx': self.prepare_panel.offset_spins[0].value(),
            'offset_ry': self.prepare_panel.offset_spins[1].value(),
            'offset_rz': self.prepare_panel.offset_spins[2].value(),
            'lead_in_dist': self.prepare_panel.spin_lead_dist.value(),
            'lead_in_angle': self.prepare_panel.spin_lead_angle.value(),
            'overcut_dist': self.prepare_panel.spin_overcut.value(),
            'chordal_error': self.prepare_panel.spin_chordal.value(),
            'max_step': self.prepare_panel.spin_max_step.value(),
            'loop_mode': self.toolbar.cb_loop_mode.currentIndex(),
            'align_mode': "minimum_twist" if self.prepare_panel.cb_align_mode.currentIndex() == 0 else "tangent"
        }

    def _block_param_signals(self, block: bool):
        for spin in self.prepare_panel.offset_spins: spin.blockSignals(block)
        for spin in [self.prepare_panel.spin_chordal, self.prepare_panel.spin_max_step, self.prepare_panel.spin_min_step, self.prepare_panel.spin_lead_dist, self.prepare_panel.spin_lead_angle, self.prepare_panel.spin_overcut]:
            spin.blockSignals(block)
        self.prepare_panel.cb_align_mode.blockSignals(block)
        self.toolbar.cb_loop_mode.blockSignals(block)

    def on_path_selection_changed(self):
        selected = [item.row() for item in self.prepare_panel.list_paths.selectedIndexes()]
        if not selected: return
        
        params = self.cam_engine.segments[selected[0]].get('params', {})
        
        self._block_param_signals(True)
        self.prepare_panel.offset_spins[0].setValue(params.get('offset_rx', 0.0))
        self.prepare_panel.offset_spins[1].setValue(params.get('offset_ry', 0.0))
        self.prepare_panel.offset_spins[2].setValue(params.get('offset_rz', 0.0))
        
        self.prepare_panel.spin_lead_dist.setValue(params.get('lead_in_dist', 2.0))
        self.prepare_panel.spin_lead_angle.setValue(params.get('lead_in_angle', 45.0))
        self.prepare_panel.spin_overcut.setValue(params.get('overcut_dist', 2.0))
        self.prepare_panel.spin_chordal.setValue(params.get('chordal_error', 0.05))
        self.prepare_panel.spin_max_step.setValue(params.get('max_step', 5.0))
        
        self.toolbar.cb_loop_mode.setCurrentIndex(params.get('loop_mode', 0))
        self.prepare_panel.cb_align_mode.setCurrentIndex(0 if params.get('align_mode', 'minimum_twist') == 'minimum_twist' else 1)
        self._block_param_signals(False)

    def on_params_changed(self):
        selected = [item.row() for item in self.prepare_panel.list_paths.selectedIndexes()]
        if not selected: return 
        
        params = self.get_current_ui_params()
        try:
            for row in selected:
                self.cam_engine.update_segment_params(row, **params)
            self.update_3d_path()
        except Exception as e:
            print(f"[Error] 參數更新失敗: {e}")

    def on_mesh_picked(self, hit_face_idx, hit_pos):
        self.hover_visual.visible = False 
        try:
            name = self.cam_engine.add_segment(hit_face_idx, hit_pos, **self.get_current_ui_params())
            self.prepare_panel.list_paths.addItem(name)
            self.prepare_panel.list_paths.setCurrentRow(self.prepare_panel.list_paths.count() - 1)
            self.update_3d_path()
        except Exception as e:
            QMessageBox.critical(self, "選取錯誤", str(e))

    def on_path_merge(self):
        selected = [item.row() for item in self.prepare_panel.list_paths.selectedIndexes()]
        if len(selected) < 2:
            QMessageBox.warning(self, "提示", "請按住 Ctrl (或 Shift) 選擇至少兩條路徑！")
            return
            
        try:
            self.cam_engine.merge_segments(selected, **self.get_current_ui_params())
            self.prepare_panel.list_paths.clear()
            for seg in self.cam_engine.segments:
                self.prepare_panel.list_paths.addItem(seg['name'])
            self.prepare_panel.list_paths.setCurrentRow(self.prepare_panel.list_paths.count() - 1)
            self.update_3d_path()
        except ValueError as e:
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
            print("[Warning] 請先建立特徵路徑！")
            return

        try:
            print("\n[System] 開始執行逆向運動學 (IK) 軌跡運算...")
            self.repaint() 

            min_step_val = self.prepare_panel.spin_min_step.value()
            c_spd = self.prepare_panel.spin_cut_speed.value()
            c_acc = self.prepare_panel.spin_cut_accel.value()
            m_spd = self.prepare_panel.spin_move_speed.value()
            m_acc = self.prepare_panel.spin_move_accel.value()

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
            
            self.playback_bar.slider.setEnabled(True)
            self.playback_bar.btn_play.setEnabled(True)
            self.playback_bar.slider.setMaximum(max(0, total_points - 1))
            self.playback_bar.slider.setValue(0)
            
            # 運算成功，解除髒標記，並把狀態直接印到 Log 區 (無打擾設計)
            self.is_script_outdated = False
            print(f"[Success] 腳本編譯成功！共產生 {total_points} 個微步點位。")
            
        except Exception as e:
            print(f"[Error] IK 運算失敗: {e}")
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
                    
                print(f"[System] 腳本成功匯出至: {filename}")
                QMessageBox.information(self, "成功", f"工業腳本已成功匯出至：\n{filename}")
            except Exception as e:
                print(f"[Error] 寫入檔案失敗: {e}")
                QMessageBox.critical(self, "儲存失敗", f"寫入檔案時發生錯誤：\n{str(e)}")

    def toggle_playback(self):
        if not self.baked_trajectory: return

        if self.anim.state() == QAbstractAnimation.Running:
            self.anim.pause()
            self.playback_bar.btn_play.setText("▶️ 繼續")
        else:
            current_idx = self.playback_bar.slider.value()
            max_idx = self.playback_bar.slider.maximum()
            
            if current_idx >= max_idx:
                current_idx = 0
                self.playback_bar.slider.setValue(0)
                
            self.anim.setStartValue(float(current_idx))
            self.anim.setEndValue(float(max_idx))
            
            remaining_points = max_idx - current_idx
            self.anim.setDuration(max(100, remaining_points * 10)) 
            self.anim.start()
            self.playback_bar.btn_play.setText("⏸ 暫停")

    def on_animation_state_changed(self, state):
        if state == QAbstractAnimation.Stopped:
            self.playback_bar.btn_play.setText("▶️ 播放")

    def update_animation_frame(self, frame_index):
        idx = int(round(frame_index))
        self.playback_bar.slider.blockSignals(True)
        self.playback_bar.slider.setValue(idx)
        self.playback_bar.lbl_frame.setText(f"{idx} / {len(self.baked_trajectory)-1}")
        self.playback_bar.slider.blockSignals(False)
        self.robot_view.update_joints(self.baked_trajectory[idx], tcp_mat=self.tcp_manager.get_active_matrix())

    def on_slider_moved(self, value):
        if not self.baked_trajectory: return
        self.playback_bar.lbl_frame.setText(f"{value} / {len(self.baked_trajectory)-1}")
        self.robot_view.update_joints(self.baked_trajectory[value], tcp_mat=self.tcp_manager.get_active_matrix())

    def refresh_tool_list(self):
        self.toolbar.cb_tool_select.blockSignals(True)
        self.toolbar.cb_tool_select.clear()
        for tool in self.tcp_manager.tools:
            self.toolbar.cb_tool_select.addItem(tool.get("name", "Unknown Tool"))
        self.toolbar.cb_tool_select.setCurrentIndex(self.tcp_manager.current_index)
        self.toolbar.cb_tool_select.blockSignals(False)

    def change_active_tool(self, index):
        if index >= 0:
            self.mark_script_outdated() # 切換刀具也會影響 IK，標記為過時
            self.tcp_manager.set_current_index(index)
            self.robot_view.update_joints(self.current_joints, tcp_mat=self.tcp_manager.get_active_matrix())
            print(f"[TCP] 已切換工具為: {self.tcp_manager.tools[index].get('name')}")

    def open_tcp_dialog(self):
        dialog = TCPManagerDialog(self.tcp_manager, self)
        dialog.exec()
        self.robot_view.update_joints(self.current_joints, tcp_mat=self.tcp_manager.get_active_matrix())
        self.refresh_tool_list()
        self.mark_script_outdated() # 在對話框中可能修改了 XYZ 偏差，標記為過時

    def zero_robot_joints(self):
        self.current_joints = [0.0] * 6
        self.robot_view.update_joints(self.current_joints, tcp_mat=self.tcp_manager.get_active_matrix())
        print("[Kinematics] 手臂各軸已強制歸零。")

    def on_canvas_clicked(self, event):
        if event.button != 1: return
        if self.toolbar.btn_magic.isChecked(): return
        
        # 如果在「預覽」分頁 (index 2)，直接忽略點擊事件，不觸發面板！
        if self.tab_bar.currentIndex() == 2: return
        
        visuals = self.robot_view.view.canvas.visuals_at(event.pos)
        
        if self.mesh_visual and (self.mesh_visual in visuals or self.bbox_visual in visuals):
            if not self.model_transform_widget.is_panel_visible:
                self.bbox_visual.visible = True
                self.model_transform_widget.set_panel_visible(True)
                print("[UI] 實體已選取，顯示姿態校正面板。")
        else:
            self.bbox_visual.visible = False
            self.model_transform_widget.set_panel_visible(False)
                
        self.robot_view.view.canvas.update()

    def update_model_transform(self):
        if self.cam_engine.workpiece_mesh is None: return 
        self.mark_script_outdated() 
        x, y, z = [spin.value() for spin in self.model_transform_widget.spins[:3]]
        rx, ry, rz = [spin.value() for spin in self.model_transform_widget.spins[3:]]
        mesh = self.cam_engine.apply_model_transform(x, y, z, rx, ry, rz)
        
        verts = mesh.vertices / 1000.0
        if self.mesh_visual: 
            self.mesh_visual.set_data(vertices=verts, faces=mesh.faces)
            
            # 即時計算並繪製 3D 灰階外框 (AABB Bounding Box)
            vmin = verts.min(axis=0)
            vmax = verts.max(axis=0)
            corners = np.array([
                [vmin[0], vmin[1], vmin[2]], [vmax[0], vmin[1], vmin[2]], 
                [vmax[0], vmax[1], vmin[2]], [vmin[0], vmax[1], vmin[2]],
                [vmin[0], vmin[1], vmax[2]], [vmax[0], vmin[1], vmax[2]], 
                [vmax[0], vmax[1], vmax[2]], [vmin[0], vmax[1], vmax[2]]
            ])
            lines = np.array([
                corners[0], corners[1], corners[1], corners[2], corners[2], corners[3], corners[3], corners[0],
                corners[4], corners[5], corners[5], corners[6], corners[6], corners[7], corners[7], corners[4],
                corners[0], corners[4], corners[1], corners[5], corners[2], corners[6], corners[3], corners[7]
            ])
            self.bbox_visual.set_data(pos=lines)
        
        if self.path_visual: self.path_visual.parent = None; self.path_visual = None
        if self.path_marker_visual: self.path_marker_visual.parent = None; self.path_marker_visual = None
        for ax in self.tcp_axes: ax.parent = None
        self.tcp_axes.clear()

    def on_load_stl_clicked(self):
        filepath, _ = QFileDialog.getOpenFileName(self, "選擇 STL 檔案", "", "STL Files (*.stl);;All Files (*)")
        if not filepath: return 
        self._load_stl_core(filepath)
        
        self.model_transform_widget.spins[0].blockSignals(True)
        self.model_transform_widget.spins[0].setValue(200.0)
        self.model_transform_widget.spins[0].blockSignals(False)
        self.update_model_transform()
        
        # 改用 set_panel_visible
        self.bbox_visual.visible = True
        self.model_transform_widget.set_panel_visible(True)
        
    def _load_stl_core(self, filepath):
        try:
            mesh = self.cam_engine.load_stl_mesh(filepath)
            self.current_project_stl = filepath 
            face_count = len(mesh.faces)
            print(f"[System] 載入 STL 成功！路徑: {filepath}")
            print(f"[System] 實體網格面數解析完成: {face_count:,} 面")
            
            if self.mesh_visual: self.mesh_visual.parent = None
            self.mesh_visual = scene.visuals.Mesh(
                vertices=mesh.vertices / 1000.0, faces=mesh.faces, 
                color=(0.5, 0.5, 0.5, 0.8), shading='flat', parent=self.robot_view.view.scene
            )
            self.mesh_visual.interactive = True 
            
            self.update_model_transform()
            self.robot_view.view.camera.set_range(margin=0.2)
        except Exception as e:
            print(f"[Error] 載入 STL 檔案失敗: {e}")
            QMessageBox.critical(self, "錯誤", str(e))

    def on_save_project(self):
        if self.cam_engine.workpiece_mesh is None:
            QMessageBox.warning(self, "警告", "尚未載入任何模型，無法儲存專案！")
            return
            
        if self.is_script_outdated and hasattr(self.cam_engine, 'current_tagged_path') and self.cam_engine.current_tagged_path:
            reply = QMessageBox.question(
                self, 
                "參數已修改", 
                "系統偵測到參數已變更，是否要先「自動計算 IK」驗證軌跡再儲存專案？\n(選 No 將直接儲存目前的參數設定)",
                QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel,
                QMessageBox.Yes
            )
            
            if reply == QMessageBox.Yes:
                self.on_bake_ik_clicked()
                # 檢查如果運算失敗，髒標記還是 True，就必須中斷儲存！
                if self.is_script_outdated:
                    return
            elif reply == QMessageBox.Cancel:
                return

        filepath, _ = QFileDialog.getSaveFileName(self, "儲存 CAM 專案", "untitled.pcam", "Parol CAM Project (*.pcam);;JSON Files (*.json)")
        if not filepath: return
        
        try:
            proj_data = {
                "version": "1.0",
                "stl_path": self.current_project_stl,
                "transform": {
                    "x": round(self.model_transform_widget.spins[0].value(), 3),
                    "y": round(self.model_transform_widget.spins[1].value(), 3),
                    "z": round(self.model_transform_widget.spins[2].value(), 3),
                    "rx": round(self.model_transform_widget.spins[3].value(), 3),
                    "ry": round(self.model_transform_widget.spins[4].value(), 3),
                    "rz": round(self.model_transform_widget.spins[5].value(), 3),
                },
                "global_settings": {
                    "safe_z": round(self.prepare_panel.spin_safe_z.value(), 3),
                    "cut_speed": self.prepare_panel.spin_cut_speed.value(),
                    "cut_accel": self.prepare_panel.spin_cut_accel.value(),
                    "move_speed": self.prepare_panel.spin_move_speed.value(),
                    "move_accel": self.prepare_panel.spin_move_accel.value()
                },
                "segments": []
            }
            
            for seg in self.cam_engine.segments:
                seg_data = {
                    'name': str(seg['name']),
                    'mode': str(seg['mode']),
                    'is_composite': bool(seg.get('is_composite', False)),
                    'face_idx': int(seg.get('face_idx', -1)),
                    'hit_pos': np.round(seg['hit_pos'], 4).tolist(),
                    'facet_normal': np.round(seg['facet_normal'], 6).tolist(),
                    'raw_locs': [np.round(p, 4).tolist() for p in seg['raw_locs']],
                    'raw_norms': [np.round(n, 6).tolist() for n in seg['raw_norms']],
                    'params': seg['params'] 
                }
                proj_data["segments"].append(seg_data)
                
            raw_json = json.dumps(proj_data, indent=4, ensure_ascii=False)
            compact_json = re.sub(
                r'\[\s+([-0-9.eE]+(?:,\s*[-0-9.eE]+)*)\s+\]',
                lambda m: '[' + re.sub(r'\s+', '', m.group(1)).replace(',', ', ') + ']',
                raw_json
            )
                
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(compact_json)
                
            print(f"[System] 專案儲存成功: {filepath}")
            QMessageBox.information(self, "成功", "專案儲存成功！")
        except Exception as e:
            print(f"[Error] 專案儲存失敗: {e}")
            QMessageBox.critical(self, "錯誤", f"專案儲存失敗:\n{str(e)}")

    def on_load_project(self):
        filepath, _ = QFileDialog.getOpenFileName(self, "載入 CAM 專案", "", "Parol CAM Project (*.pcam);;JSON Files (*.json)")
        if not filepath: return
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                proj_data = json.load(f)
                
            stl_path = proj_data.get("stl_path", "")
            if not os.path.exists(stl_path):
                QMessageBox.warning(self, "模型遺失", f"找不到原始模型檔案:\n{stl_path}\n請手動重新指定 STL 的位置。")
                stl_path, _ = QFileDialog.getOpenFileName(self, "重新指定 STL 檔案", "", "STL Files (*.stl)")
                if not stl_path: return
                
            self._load_stl_core(stl_path)
            
            tf = proj_data.get("transform", {})
            for spin in self.model_transform_widget.spins: spin.blockSignals(True)
            self.model_transform_widget.spins[0].setValue(tf.get("x", 0.0))
            self.model_transform_widget.spins[1].setValue(tf.get("y", 0.0))
            self.model_transform_widget.spins[2].setValue(tf.get("z", 0.0))
            self.model_transform_widget.spins[3].setValue(tf.get("rx", 0.0))
            self.model_transform_widget.spins[4].setValue(tf.get("ry", 0.0))
            self.model_transform_widget.spins[5].setValue(tf.get("rz", 0.0))
            for spin in self.model_transform_widget.spins: spin.blockSignals(False)
            self.update_model_transform()

            g_settings = proj_data.get("global_settings", {})
            self.prepare_panel.spin_safe_z.blockSignals(True)
            self.prepare_panel.spin_safe_z.setValue(g_settings.get("safe_z", 20.0))
            self.prepare_panel.spin_safe_z.blockSignals(False)
            
            self.prepare_panel.spin_cut_speed.setValue(g_settings.get("cut_speed", 30))
            self.prepare_panel.spin_cut_accel.setValue(g_settings.get("cut_accel", 100))
            self.prepare_panel.spin_move_speed.setValue(g_settings.get("move_speed", 100))
            self.prepare_panel.spin_move_accel.setValue(g_settings.get("move_accel", 100))
            
            self.cam_engine.segments.clear()
            self.prepare_panel.list_paths.clear()
            
            for seg_data in proj_data.get("segments", []):
                seg = {
                    'name': seg_data['name'],
                    'mode': seg_data['mode'],
                    'is_composite': seg_data['is_composite'],
                    'face_idx': seg_data['face_idx'],
                    'hit_pos': np.array(seg_data['hit_pos']),
                    'facet_normal': np.array(seg_data['facet_normal']),
                    'raw_locs': np.array(seg_data['raw_locs']),
                    'raw_norms': np.array(seg_data['raw_norms']),
                    'params': seg_data['params']
                }
                self.cam_engine.segments.append(seg)
                self.cam_engine._process_segment(seg) 
                self.prepare_panel.list_paths.addItem(seg['name'])
                
            self.update_3d_path()
            print(f"[System] 專案載入成功: {filepath}")
            QMessageBox.information(self, "成功", "專案與特徵路徑已完全恢復！")
            
        except Exception as e:
            print(f"[Error] 專案載入失敗: {e}")
            QMessageBox.critical(self, "錯誤", f"專案載入失敗:\n{str(e)}")