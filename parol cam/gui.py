# gui.py
import numpy as np
import qtawesome as qta 
from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QGroupBox, QLabel, 
                               QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                               QPushButton, QMessageBox, QFileDialog, 
                               QGridLayout, QSlider, QSplitter, QStackedWidget, QTabBar)
from PySide6.QtCore import Qt, QVariantAnimation, QAbstractAnimation, QSize
from vispy import scene

from PySide6.QtGui import Qt, QCursor 
from picker import MeshPicker 

from cam_settings import Robot3DView
from tcp_manager import TCPManager 
from tcp_dialog import TCPManagerDialog 
from core import CAMEngine
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
        
        self.robot_view = None
        self.mesh_visual = None
        self.path_visual = None
        self.tcp_axes = []

        # 👑 新增：紀錄最後一次萃取特徵的資訊，供姿態更新按鈕使用
        self.last_hit_face_idx = None
        self.last_hit_pos = None

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

        # ==========================================
        # 頂部：全局 Tab Bar
        # ==========================================
        tab_layout = QHBoxLayout()
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.setSpacing(0)
        
        self.tab_bar = QTabBar()
        self.tab_bar.setShape(QTabBar.RoundedNorth)
        self.tab_bar.setStyleSheet("""
            QTabBar::tab { height: 35px; width: 100px; font-weight: bold; font-size: 14px; background: #2b2b2b; color: #888; margin-right: 2px; border-top-left-radius: 4px; border-top-right-radius: 4px; }
            QTabBar::tab:selected { background-color: #444; color: white; border-bottom: 3px solid #b37700; }
            QTabBar::tab:hover { background-color: #3b3b3b; color: white; }
        """)
        self.tab_bar.addTab("主頁")
        self.tab_bar.addTab("準備")
        self.tab_bar.addTab("預覽")
        self.tab_bar.addTab("設備")
        
        tab_layout.addWidget(self.tab_bar)
        tab_layout.addStretch() 
        self.main_v_layout.addLayout(tab_layout)

        # ==========================================
        # 下方工作區：水平 Splitter
        # ==========================================
        self.splitter = QSplitter(Qt.Horizontal)
        self.main_v_layout.addWidget(self.splitter, stretch=1)

        # ------------------------------------------
        # Splitter 左側：動態切換的控制面板區
        # ------------------------------------------
        self.left_panel = QStackedWidget()
        self.left_panel.setMaximumWidth(350)  
        self.left_panel.setMinimumWidth(0)    
        
        self.left_panel.setStyleSheet("""
            QGroupBox {
                border: none;
                border-bottom: 1px solid #444444; 
                padding-top: 15px;
                padding-bottom: 15px;
                margin-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                color: #aaaaaa;
                font-weight: bold;
            }
        """)
        
        self.page_main = QWidget()
        self.page_prepare = QWidget()
        self.page_preview = QWidget()
        self.page_device = QWidget()
        
        self.left_panel.addWidget(self.page_main)
        self.left_panel.addWidget(self.page_prepare)
        self.left_panel.addWidget(self.page_preview)
        self.left_panel.addWidget(self.page_device)

        self.splitter.addWidget(self.left_panel)

        # ------------------------------------------
        # Splitter 右側：3D 視窗與頂部工具列 (Overlay)
        # ------------------------------------------
        self.right_panel = QWidget()
        
        right_layout = QGridLayout(self.right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        self.robot_view = Robot3DView() 
        right_layout.addWidget(self.robot_view, 0, 0)
        
        self.prepare_toolbar = QWidget()
        self.prepare_toolbar.setObjectName("FloatingToolbar")
        
        self.prepare_toolbar.setStyleSheet("""
            QWidget#FloatingToolbar {
                background-color: #2b2b2b; 
                border-bottom: 1px solid #111111;
            }
        """)
        toolbar_layout = QHBoxLayout(self.prepare_toolbar)
        toolbar_layout.setContentsMargins(8, 5, 8, 5) 
        toolbar_layout.setSpacing(5)
        
        self.btn_tb_magic = QPushButton(qta.icon('mdi.auto-fix', color='white'), "")
        self.btn_tb_magic.setCheckable(True) 
        self.btn_tb_magic.toggled.connect(self.on_magic_wand_toggled)
        self.btn_tb_magic.setToolTip("幾何萃取 (魔術棒)")
        
        self.btn_tb_move = QPushButton(qta.icon('mdi.cursor-move', color='white'), "")
        self.btn_tb_move.setToolTip("移動")
        
        self.btn_tb_scale = QPushButton(qta.icon('mdi.arrow-expand-all', color='white'), "")
        self.btn_tb_scale.setToolTip("縮放")
        
        self.btn_tb_mirror = QPushButton(qta.icon('mdi.flip-horizontal', color='white'), "")
        self.btn_tb_mirror.setToolTip("鏡射")
        
        self.btn_tb_add = QPushButton(qta.icon('mdi.shape-square-plus', color='white'), "")
        self.btn_tb_add.setToolTip("增加實體")
        
        tb_style = """
            QPushButton { 
                background-color: transparent; 
                border-radius: 4px; 
                border: none; 
            } 
            QPushButton:hover { background-color: #444444; }
            QPushButton:pressed { background-color: #b37700; }
            QPushButton:checked { background-color: #b37700; }
        """
        for btn in [self.btn_tb_magic, self.btn_tb_move, self.btn_tb_scale, self.btn_tb_mirror, self.btn_tb_add]:
            btn.setFixedSize(40, 40)       
            btn.setIconSize(QSize(22, 22)) 
            btn.setStyleSheet(tb_style)
            toolbar_layout.addWidget(btn)
            
        toolbar_layout.addStretch() 
        right_layout.addWidget(self.prepare_toolbar, 0, 0, Qt.AlignTop)
        
        self.splitter.addWidget(self.right_panel)
        self.splitter.setSizes([350, 1000])

        self.tab_bar.currentChanged.connect(self.on_tab_changed)

        self._setup_tab_prepare()
        self._setup_tab_preview()

        self.tab_bar.setCurrentIndex(1)
        self.prepare_toolbar.show() 

    def on_tab_changed(self, index):
        self.left_panel.setCurrentIndex(index)
        if index == 1: 
            self.prepare_toolbar.show()
        else:
            self.prepare_toolbar.hide()

    def _setup_tab_prepare(self):
        prepare_layout = QVBoxLayout(self.page_prepare) 
        prepare_layout.setContentsMargins(15, 10, 15, 10)
        
        sys_group = QGroupBox("系統與刀具 (TCP) 綁定")
        sys_layout = QVBoxLayout()
        self.btn_tcp_config = QPushButton("打開 TCP Manager")
        self.btn_tcp_config.clicked.connect(self.open_tcp_dialog)
        
        tool_layout = QHBoxLayout()
        tool_layout.addWidget(QLabel("綁定刀具:"))
        self.cb_tool_select = QComboBox()
        self.cb_tool_select.currentIndexChanged.connect(self.change_active_tool)
        self.refresh_tool_list()
        tool_layout.addWidget(self.cb_tool_select)
        
        self.btn_zero_joints = QPushButton("手臂歸零 (Home Pose)")
        self.btn_zero_joints.clicked.connect(self.zero_robot_joints)
        
        sys_layout.addWidget(self.btn_tcp_config)
        sys_layout.addLayout(tool_layout)
        sys_layout.addWidget(self.btn_zero_joints)
        sys_group.setLayout(sys_layout)

        # 👑 將 STL 載入按鈕搬遷至此處
        model_group = QGroupBox("模型位置與姿態校正")
        model_layout = QVBoxLayout()
        
        self.btn_load_stl = QPushButton("📂 載入 STL 檔案")
        self.btn_load_stl.setStyleSheet("background-color: #5c3a21; color: white; padding: 8px; border-radius: 4px;")
        self.btn_load_stl.clicked.connect(self.on_load_stl_clicked) 
        model_layout.addWidget(self.btn_load_stl)
        
        grid = QGridLayout()
        self.model_spins = []
        labels_model = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        for i in range(6):
            h_layout = QHBoxLayout()
            lbl = QLabel(labels_model[i])
            lbl.setFixedWidth(20) 
            h_layout.addWidget(lbl)
            spin = QDoubleSpinBox()
            spin.setRange(-2000.0 if i < 3 else -180.0, 2000.0 if i < 3 else 180.0)
            spin.setSingleStep(10.0 if i < 3 else 5.0)
            spin.valueChanged.connect(self.update_model_transform)
            self.model_spins.append(spin)
            h_layout.addWidget(spin)
            grid.addLayout(h_layout, i // 3, i % 3)
        model_layout.addLayout(grid)
        model_group.setLayout(model_layout)

        # 👑 修改佈局以容納更新按鈕
        offset_group = QGroupBox("表面加工姿態微調")
        offset_layout = QVBoxLayout() 
        
        spins_layout = QHBoxLayout()
        self.offset_spins = []
        labels_offset = ["Rx", "Ry", "Rz"]
        for i in range(3):
            layout_o = QVBoxLayout()
            layout_o.addWidget(QLabel(labels_offset[i]))
            spin = QDoubleSpinBox()
            spin.setRange(-180.0, 180.0)
            spin.setSingleStep(15.0)
            self.offset_spins.append(spin)
            layout_o.addWidget(spin)
            spins_layout.addLayout(layout_o)
        offset_layout.addLayout(spins_layout)
        
        # 👑 新增的刷新按鈕
        self.btn_update_offset = QPushButton("🔄 更新微調姿態")
        self.btn_update_offset.setStyleSheet("background-color: #444444; color: white; padding: 6px; border-radius: 4px;")
        self.btn_update_offset.clicked.connect(self.on_update_offset_clicked)
        offset_layout.addWidget(self.btn_update_offset)
        
        offset_group.setLayout(offset_layout)

        # 把第三個幾何群組移除，保持 UI 簡潔
        prepare_layout.addWidget(sys_group)
        prepare_layout.addWidget(model_group)
        prepare_layout.addWidget(offset_group)
        prepare_layout.addStretch()

    def _setup_tab_preview(self):
        preview_layout = QVBoxLayout(self.page_preview) 
        preview_layout.setContentsMargins(15, 10, 15, 10)

        bake_group = QGroupBox("軌跡運算")
        bake_layout = QVBoxLayout()
        self.btn_bake_ik = QPushButton("計算 IK 並烘焙腳本")
        self.btn_bake_ik.setStyleSheet("background-color: #b37700; color: white; padding: 15px; font-weight: bold; border-radius: 4px;")
        self.btn_bake_ik.clicked.connect(self.on_bake_ik_clicked)
        
        self.lbl_bake_status = QLabel("狀態：等待運算...")
        bake_layout.addWidget(self.btn_bake_ik)
        bake_layout.addWidget(self.lbl_bake_status)
        bake_group.setLayout(bake_layout)

        player_group = QGroupBox("軌跡動畫預覽")
        player_layout = QVBoxLayout()
        
        slider_layout = QHBoxLayout()
        self.lbl_frame = QLabel("0 / 0")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(0)
        self.slider.setEnabled(False) 
        self.slider.valueChanged.connect(self.on_slider_moved)
        slider_layout.addWidget(self.slider)
        slider_layout.addWidget(self.lbl_frame)
        
        btn_layout = QHBoxLayout()
        self.btn_play = QPushButton("▶️ 播放")
        self.btn_play.setEnabled(False)
        self.btn_play.clicked.connect(self.toggle_playback)
        btn_layout.addWidget(self.btn_play)
        
        player_layout.addLayout(slider_layout)
        player_layout.addLayout(btn_layout)
        player_group.setLayout(player_layout)

        preview_layout.addWidget(bake_group)
        preview_layout.addWidget(player_group)
        preview_layout.addStretch()

        self.anim = QVariantAnimation()
        self.anim.valueChanged.connect(self.update_animation_frame)
        self.anim.stateChanged.connect(self.on_animation_state_changed)

    def toggle_playback(self):
        if not self.baked_trajectory: return

        if self.anim.state() == QAbstractAnimation.Running:
            self.anim.pause()
            self.btn_play.setText("▶️ 繼續")
        else:
            current_idx = self.slider.value()
            max_idx = self.slider.maximum()
            
            if current_idx >= max_idx:
                current_idx = 0
                self.slider.setValue(0)
                
            self.anim.setStartValue(float(current_idx))
            self.anim.setEndValue(float(max_idx))
            
            remaining_points = max_idx - current_idx
            real_duration_ms = remaining_points * 10 
            
            self.anim.setDuration(max(100, real_duration_ms)) 
            self.anim.start()
            self.btn_play.setText("⏸ 暫停")

    def on_animation_state_changed(self, state):
        if state == QAbstractAnimation.Stopped:
            self.btn_play.setText("▶️ 播放")

    def update_animation_frame(self, frame_index):
        idx = int(round(frame_index))
        
        self.slider.blockSignals(True)
        self.slider.setValue(idx)
        self.lbl_frame.setText(f"{idx} / {len(self.baked_trajectory)-1}")
        self.slider.blockSignals(False)
        
        target_joints = self.baked_trajectory[idx]
        if self.robot_view:
            self.robot_view.update_joints(target_joints, tcp_mat=self.tcp_manager.get_active_matrix())

    def on_slider_moved(self, value):
        if not self.baked_trajectory: return
        self.lbl_frame.setText(f"{value} / {len(self.baked_trajectory)-1}")
        
        target_joints = self.baked_trajectory[value]
        if self.robot_view:
            self.robot_view.update_joints(target_joints, tcp_mat=self.tcp_manager.get_active_matrix())

    def on_bake_ik_clicked(self):
        if len(self.cam_engine.cam_tcp_matrices) == 0:
            QMessageBox.warning(self, "警告", "請先在「準備」頁籤進行幾何萃取！")
            return

        try:
            self.lbl_bake_status.setText("狀態：烘焙原始腳本中...")
            self.repaint() 

            start_joints = self.post_processor.bake_trajectory(self.cam_engine.cam_tcp_matrices)
            
            import json
            with open("parol_cam_export.json", "r", encoding="utf-8") as f:
                script_data = json.load(f)
                
            waypoints = []
            if isinstance(script_data, list):
                if len(script_data) > 0 and isinstance(script_data[0], dict) and 'type' in script_data[0]:
                    cam_block = next((b for b in script_data if b.get("type") == "CAM_PATH"), None)
                    if cam_block:
                        waypoints = cam_block.get("path_data", [])
                else:
                    waypoints = script_data
            elif isinstance(script_data, dict):
                if script_data.get("type") == "CAM_PATH":
                    waypoints = script_data.get("path_data", [])
                else:
                    waypoints = script_data.get("path_data", script_data.get("waypoints", []))

            if not waypoints:
                raise ValueError("無法在腳本中找到有效的路徑點 (CAM_PATH)！")

            formatted_waypoints = []
            active_tcp = self.tcp_manager.get_active_matrix()
            
            for wp in waypoints:
                if isinstance(wp, dict):
                    joints = wp.get('target_joints', wp.get('joints', None))
                    speed_factor = wp.get('speed_factor', 1.0)
                elif isinstance(wp, (list, tuple)):
                    joints = wp
                    speed_factor = 1.0
                else:
                    continue
                    
                if joints is not None:
                    formatted_waypoints.append({
                        'target_joints': joints,
                        'speed_factor': speed_factor,
                        'tcp_offset_mat': active_tcp
                    })

            if not formatted_waypoints:
                raise ValueError("解析點位失敗，請檢查 JSON 內部格式！")

            self.lbl_bake_status.setText("狀態：計算 5 次樣條與 SO(3) 運動學中...")
            self.repaint()

            from kinematics import TrajectoryMathEngine
            generator, total_time, msg, N = TrajectoryMathEngine.calculate_spline_trajectory(
                start_joints=formatted_waypoints[0]['target_joints'], 
                spline_waypoints=formatted_waypoints,
                interval=0.010 
            )

            self.baked_trajectory = list(generator)
            total_points = len(self.baked_trajectory)
            
            self.slider.setEnabled(True)
            self.btn_play.setEnabled(True)
            self.slider.setMaximum(total_points - 1)
            self.slider.setValue(0)
            self.lbl_bake_status.setText(f"狀態：{msg} (耗時 {total_time:.2f}s)")
            
            QMessageBox.information(self, "成功", f"高精度樣條烘焙完成！\n總預覽時間：{total_time:.2f} 秒\n(共生成 {total_points} 個 10ms 物理點位)")
            
        except Exception as e:
            import traceback
            traceback.print_exc() 
            self.lbl_bake_status.setText("狀態：樣條運算失敗！")
            QMessageBox.critical(self, "IK/樣條 運算失敗", str(e))

    def refresh_tool_list(self):
        self.cb_tool_select.blockSignals(True)
        self.cb_tool_select.clear()
        for tool in self.tcp_manager.tools:
            self.cb_tool_select.addItem(tool.get("name", "Unknown Tool"))
        self.cb_tool_select.setCurrentIndex(self.tcp_manager.current_index)
        self.cb_tool_select.blockSignals(False)

    def change_active_tool(self, index):
        if index >= 0:
            self.tcp_manager.set_current_index(index)
            if self.robot_view:
                self.robot_view.update_joints(self.current_joints, tcp_mat=self.tcp_manager.get_active_matrix())

    def open_tcp_dialog(self):
        dialog = TCPManagerDialog(self.tcp_manager, self)
        dialog.exec()
        if self.robot_view:
            self.robot_view.update_joints(self.current_joints, tcp_mat=self.tcp_manager.get_active_matrix())
        self.refresh_tool_list()

    def zero_robot_joints(self):
        self.current_joints = [0.0] * 6
        if self.robot_view:
            self.robot_view.update_joints(self.current_joints, tcp_mat=self.tcp_manager.get_active_matrix())

    def update_model_transform(self):
        if self.cam_engine.original_mesh is None: return 
        x, y, z = [self.model_spins[i].value() for i in range(3)]
        rx, ry, rz = [self.model_spins[i].value() for i in range(3, 6)]
        mesh = self.cam_engine.apply_model_transform(x, y, z, rx, ry, rz)
        if self.mesh_visual:
            self.mesh_visual.set_data(vertices=mesh.vertices / 1000.0, faces=mesh.faces)
        if self.path_visual: 
            self.path_visual.parent = None
            self.path_visual = None
        for ax in self.tcp_axes: ax.parent = None
        self.tcp_axes.clear()

    def on_load_stl_clicked(self):
        filepath, _ = QFileDialog.getOpenFileName(self, "選擇 STL 檔案", "", "STL Files (*.stl);;All Files (*)")
        if not filepath: return 
        try:
            mesh = self.cam_engine.load_stl_mesh(filepath)
            if self.mesh_visual: self.mesh_visual.parent = None
            self.mesh_visual = scene.visuals.Mesh(
                vertices=mesh.vertices / 1000.0, faces=mesh.faces, 
                color=(0.5, 0.5, 0.5, 0.8), shading='flat', parent=self.robot_view.view.scene
            )
            self.robot_view.view.camera.set_range(margin=0.2)
            self.update_model_transform()
        except Exception as e:
            QMessageBox.critical(self, "錯誤", str(e))

    def on_update_offset_clicked(self):
        """👑 觸發更新：直接拿最後一次選取的模型座標，套用最新參數進行重繪"""
        if self.last_hit_face_idx is None or self.last_hit_pos is None:
            QMessageBox.information(self, "提示", "請先使用「魔術棒」在 3D 畫面中萃取一次特徵！")
            return
            
        # 呼叫 picking 函式重新計算，它內部會自動抓取 offset_spins 最新輸入的值
        self.on_mesh_picked(self.last_hit_face_idx, self.last_hit_pos)

    def on_mesh_hovered(self, boundary_pts):
        if boundary_pts is not None:
            self.hover_visual.set_data(pos=boundary_pts / 1000.0)
            self.hover_visual.visible = True
        else:
            self.hover_visual.visible = False
            
    def on_mesh_picked(self, hit_face_idx, hit_pos):
        self.btn_tb_magic.setChecked(False)
        self.hover_visual.visible = False 
        
        # 👑 記錄此座標點，供「更新微調姿態」按鈕再次取用
        self.last_hit_face_idx = hit_face_idx
        self.last_hit_pos = hit_pos
        
        rx, ry, rz = [spin.value() for spin in self.offset_spins]

        try:
            locations, matrices = self.cam_engine.extract_contour_from_boundary(
                hit_face_idx, hit_pos, offset_rx=rx, offset_ry=ry, offset_rz=rz
            )

            if self.path_visual: self.path_visual.parent = None
            for axis in self.tcp_axes: axis.parent = None
            self.tcp_axes.clear()

            self.path_visual = scene.visuals.Markers(
                pos=locations / 1000.0, size=5, face_color='yellow', parent=self.robot_view.view.scene
            )

            for i, T_tcp in enumerate(matrices):
                if i % (max(1, len(locations) // 15)) == 0:  
                    tcp_axis = scene.visuals.XYZAxis(parent=self.robot_view.view.scene)
                    transform = scene.transforms.MatrixTransform()
                    draw_matrix = np.eye(4)
                    draw_matrix[:3, :3] = T_tcp[:3, :3] * 0.015 
                    draw_matrix[:3, 3] = T_tcp[:3, 3] / 1000.0 
                    transform.matrix = draw_matrix.T 
                    tcp_axis.transform = transform
                    self.tcp_axes.append(tcp_axis)

            print(f"[GUI] 成功繪製精準輪廓！共生成 {len(locations)} 個點。")

        except Exception as e:
            QMessageBox.critical(self, "錯誤", str(e))

    def on_magic_wand_toggled(self, checked):
        if self.cam_engine.workpiece_mesh is None:
            self.btn_tb_magic.blockSignals(True)
            self.btn_tb_magic.setChecked(False)
            self.btn_tb_magic.blockSignals(False)
            QMessageBox.warning(self, "警告", "請先載入 STL 檔案！")
            return

        if checked:
            self.mesh_picker.toggle_picking_mode(True)
            self.robot_view.canvas.native.setCursor(QCursor(Qt.CrossCursor))
        else:
            self.mesh_picker.toggle_picking_mode(False)
            self.robot_view.canvas.native.setCursor(QCursor(Qt.ArrowCursor))