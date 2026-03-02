import sys
import datetime
import numpy as np
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QSlider, QLabel, QPushButton, QGridLayout, QGroupBox, 
                             QDoubleSpinBox, QTextEdit, QFrame, 
                             QListWidget, QListWidgetItem, QSizePolicy, 
                             QMessageBox, QComboBox, QMenu, QAction, QApplication)
from PyQt5.QtCore import Qt, QSize, QTimer, QEvent
import qtawesome as qta

from ui import styles
from ui.widgets import CircularButton, MonitorFrame, WaypointRow
from ui.tcp_dialog import TCPSettingsDialog
from tcp_manager import TCPManager
from serial_manager import SerialManager

from simulation_standalone import RobotSimulation
from path_manager import PathManager
import config
import kinematics

class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(config.APP_TITLE)
        self.resize(1920, 1200)

        # 核心模組
        self.sim = RobotSimulation(self)
        self.tcp_manager = TCPManager()
        
        # 【新增】實例化 Serial Manager
        self.serial_manager = SerialManager()
        self.serial_manager.log_signal.connect(self.log)
        self.serial_manager.connection_state_signal.connect(self.on_serial_connection_changed)
        
        self.current_joints = [0.0] * 6
        self.prev_joints = None  
        
        # 硬體修正矩陣 (物理法蘭面 = Z - 23.6mm)
        self.T_hw_fix = np.eye(4)
        self.T_hw_fix[2, 3] = -0.0236 
        
        self.path_manager = PathManager(parent=self)
        self.path_manager.log_signal.connect(self.log)
        self.path_manager.joint_update_signal.connect(self.on_manager_update_joints)
        self.path_manager.list_update_signal.connect(self.refresh_waypoint_list)

        self.setStyleSheet(styles.get_global_style())

        self.setup_ui()
        QTimer.singleShot(100, self.reposition_overlays)
        
        self.log("System Initialized.")
        
        user_offset = self.tcp_manager.get_active_matrix()
        self.sim.update_simulation(self.current_joints, user_offset)
        self.update_monitor()
        
        # 顯示當前工具名稱
        tool_name = self.tcp_manager.get_active_tool_data()['name']
        self.log(f"Active Tool: {tool_name}")

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.root_layout = QVBoxLayout(central_widget)
        self.root_layout.setContentsMargins(0, 0, 0, 0)
        self.root_layout.setSpacing(0)
        self.setup_top_bar()
        content_widget = QWidget()
        self.content_layout = QHBoxLayout(content_widget)
        self.content_layout.setContentsMargins(15, 0, 15, 15)
        self.content_layout.setSpacing(5)
        self.setup_left_panel()
        self.setup_right_panel()
        self.root_layout.addWidget(content_widget)

    def setup_top_bar(self):
        top_bar = QFrame()
        top_bar.setFixedHeight(90)
        top_bar.setStyleSheet(styles.TOP_BAR_STYLE)
        layout = QHBoxLayout(top_bar)
        layout.setContentsMargins(20, 10, 20, 10)
        layout.setSpacing(15)
        
        title_lbl = QLabel("PAROL6 Controller")
        title_lbl.setStyleSheet(styles.TITLE_LABEL_STYLE)
        layout.addWidget(title_lbl)
        
        self._setup_run_buttons(layout)

        BTN_HEIGHT = 55
        btn_stop = QPushButton(" STOP")
        btn_stop.setFixedHeight(BTN_HEIGHT)
        btn_stop.setIcon(qta.icon('fa5s.pause-circle', color='white'))
        btn_stop.setStyleSheet(styles.BTN_STOP_STYLE)
        btn_stop.clicked.connect(self.emergency_stop)
        layout.addWidget(btn_stop)
        
        btn_home = QPushButton("HOME")
        btn_home.setFixedHeight(BTN_HEIGHT)
        btn_home.setIcon(qta.icon('fa5s.home', color='white'))
        btn_home.setToolTip("Move joints to 0")
        btn_home.setStyleSheet(styles.BTN_HOME_STYLE)
        btn_home.clicked.connect(self.home_robot)
        layout.addWidget(btn_home)

        # ★★★ HOMING 按鈕 ★★★
        btn_homing = QPushButton("HOMING")
        btn_homing.setFixedHeight(BTN_HEIGHT)
        btn_homing.setStyleSheet(styles.BTN_HOMING_STYLE)
        btn_homing.clicked.connect(self.confirm_homing)
        layout.addWidget(btn_homing)
        
        layout.addSpacing(20)

        # --- 【新增】硬體連線區域 (移至 Top Bar) ---
        
        # Port Label
        lbl_port = QLabel("Port:")
        lbl_port.setStyleSheet("color: #ecf0f1; font-weight: bold; font-size: 16px;")
        layout.addWidget(lbl_port)
        
        # Port ComboBox
        self.combo_ports = QComboBox()
        self.combo_ports.setMinimumWidth(120)
        self.combo_ports.setFixedHeight(40)
        self.combo_ports.setStyleSheet("""
            QComboBox { font-size: 14px; padding: 5px; border-radius: 4px; background: white; }
            QComboBox::drop-down { border: 0px; }
        """)
        self.refresh_ports() # 初始掃描
        layout.addWidget(self.combo_ports)
        
        # Refresh Button
        btn_refresh = QPushButton()
        btn_refresh.setIcon(qta.icon('fa5s.sync-alt', color='white'))
        btn_refresh.setFixedSize(40, 40)
        btn_refresh.setToolTip("Refresh Ports")
        btn_refresh.setStyleSheet("""
            QPushButton { background-color: #7f8c8d; border-radius: 4px; border: none; }
            QPushButton:hover { background-color: #95a5a6; }
            QPushButton:pressed { background-color: #5d6d7e; }
        """)
        btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(btn_refresh)
        
        # Connect Button
        self.btn_connect = QPushButton(" Connect")
        self.btn_connect.setIcon(qta.icon('fa5s.plug', color='white'))
        self.btn_connect.setFixedHeight(40)
        self.btn_connect.setStyleSheet("""
            QPushButton { 
                background-color: #27ae60; color: white; font-weight: bold; 
                padding: 0px 15px; border-radius: 4px; font-size: 16px; border: none;
            }
            QPushButton:hover { background-color: #2ecc71; }
        """)
        self.btn_connect.clicked.connect(self.toggle_connection)
        layout.addWidget(self.btn_connect)
        
        layout.addSpacing(20)
        # ----------------------------------------

        # TOOL 設定按鈕
        btn_tool = QPushButton(" TOOL")
        btn_tool.setFixedHeight(BTN_HEIGHT)
        btn_tool.setIcon(qta.icon('fa5s.tools', color='white'))
        btn_tool.setToolTip("Configure Tool Center Point (TCP)")
        btn_tool.setStyleSheet(styles.BTN_TOOL_STYLE)
        btn_tool.clicked.connect(self.open_tcp_settings)
        layout.addWidget(btn_tool)
        
        layout.addStretch()
        self.root_layout.addWidget(top_bar)

    # --- 【新增】 連線相關函式 ---
    def refresh_ports(self):
        self.combo_ports.clear()
        ports = self.serial_manager.list_ports()
        if ports:
            self.combo_ports.addItems(ports)
        else:
            self.combo_ports.addItem("No Ports")

    def toggle_connection(self):
        if self.serial_manager.is_connected:
            self.serial_manager.disconnect()
        else:
            port = self.combo_ports.currentText()
            if "No Ports" in port: return
            self.serial_manager.connect(port, 115200)

    def on_serial_connection_changed(self, connected):
        if connected:
            self.btn_connect.setText(" Disconnect")
            self.btn_connect.setIcon(qta.icon('fa5s.times', color='white'))
            self.btn_connect.setStyleSheet("""
                QPushButton { 
                    background-color: #c0392b; color: white; font-weight: bold; 
                    padding: 0px 15px; border-radius: 4px; font-size: 16px; border: none;
                }
                QPushButton:hover { background-color: #e74c3c; }
            """)
            self.combo_ports.setEnabled(False)
        else:
            self.btn_connect.setText(" Connect")
            self.btn_connect.setIcon(qta.icon('fa5s.plug', color='white'))
            self.btn_connect.setStyleSheet("""
                QPushButton { 
                    background-color: #27ae60; color: white; font-weight: bold; 
                    padding: 0px 15px; border-radius: 4px; font-size: 16px; border: none;
                }
                QPushButton:hover { background-color: #2ecc71; }
            """)
            self.combo_ports.setEnabled(True)
    # ---------------------------

    # === 緊急停止函式 ===
    def emergency_stop(self):
        # 1. 軟體端停止 (停止路徑規劃運算、計時器)
        self.path_manager.stop_path()
        
        # 2. 硬體端停止 (發送 <STOP> 指令)
        # 直接存取 serial_manager 內部的 ser 物件來發送最快
        if self.serial_manager.is_connected and hasattr(self.serial_manager, 'ser'):
            try:
                # 加上 \n 確保指令被送出
                self.serial_manager.ser.write(b"<STOP>\n")
                self.log(">> [EMERGENCY] <STOP> sent to Hardware!")
            except Exception as e:
                self.log(f"Error sending STOP: {e}")
        else:
            self.log(">> STOP pressed (Software only, Hardware not connected)")

    def _setup_run_buttons(self, parent_layout):
        run_container = QWidget()
        run_layout = QHBoxLayout(run_container)
        run_layout.setContentsMargins(0, 0, 0, 0)
        run_layout.setSpacing(2) 
        BTN_HEIGHT = 55
        btn_run_main = QPushButton(" RUN PATH")
        btn_run_main.setFixedHeight(BTN_HEIGHT)
        btn_run_main.setIcon(qta.icon('fa5s.play-circle', color='white'))
        btn_run_main.setStyleSheet(styles.BTN_RUN_MAIN_STYLE)
        btn_run_main.clicked.connect(lambda: self.trigger_run_path(loop=False))
        
        self.btn_run_menu = QPushButton()
        self.btn_run_menu.setFixedSize(30, BTN_HEIGHT)
        self.btn_run_menu.setIcon(qta.icon('fa5s.chevron-down', color='white'))
        self.btn_run_menu.setStyleSheet(styles.BTN_RUN_MENU_STYLE)
        
        self.run_menu = QMenu(self)
        self.run_menu.setStyleSheet(styles.MENU_STYLE)
        action_run_once = QAction(qta.icon('fa5s.play', color='white'), "Run Path", self)
        action_run_loop = QAction(qta.icon('fa5s.sync', color='white'), "Loop Run", self)
        action_run_once.triggered.connect(lambda: self.trigger_run_path(loop=False))
        action_run_loop.triggered.connect(lambda: self.trigger_run_path(loop=True))
        
        self.run_menu.addAction(action_run_once)
        self.run_menu.addAction(action_run_loop)
        self.btn_run_menu.clicked.connect(self.show_run_menu)
        run_layout.addWidget(btn_run_main)
        run_layout.addWidget(self.btn_run_menu)
        parent_layout.addWidget(run_container)

    def show_run_menu(self):
        pos = self.btn_run_menu.mapToGlobal(self.btn_run_menu.rect().bottomRight())
        self.run_menu.exec_(pos)

    def setup_left_panel(self):
        self.left_panel = QVBoxLayout()
        self.left_panel.setSpacing(20)
        self.setup_joint_control()
        self.setup_cartesian_control()
        self.setup_waypoints_manager()
        left_widget = QWidget()
        left_widget.setLayout(self.left_panel)
        left_widget.setFixedWidth(680)
        self.content_layout.addWidget(left_widget)

    def setup_joint_control(self):
        group = QGroupBox("Joint Control")
        layout = QVBoxLayout()
        layout.setSpacing(8)
        self.sliders = []
        self.spinboxes = []
        for i in range(6):
            row = QHBoxLayout()
            min_a, max_a = config.JOINT_LIMITS[i]
            lbl = QLabel(f"J{i+1}")
            lbl.setFixedWidth(40)
            lbl.setStyleSheet("font-weight: bold;")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(min_a * 100), int(max_a * 100))
            spin = QDoubleSpinBox()
            spin.setRange(min_a, max_a)
            spin.setDecimals(1)
            spin.setSingleStep(1.0)
            spin.setFixedWidth(100)
            slider.valueChanged.connect(lambda v, idx=i: self.on_slider_change(v, idx))
            spin.valueChanged.connect(lambda v, idx=i: self.on_spinbox_change(v, idx))

            # === 【新增】 綁定「動作結束」事件 (負責發送給硬體) ===
            slider.sliderReleased.connect(self.on_control_finished)
            spin.editingFinished.connect(self.on_control_finished)

            row.addWidget(lbl)
            row.addWidget(slider)
            row.addWidget(spin)
            layout.addLayout(row)
            self.sliders.append(slider)
            self.spinboxes.append(spin)
        group.setLayout(layout)
        self.left_panel.addWidget(group, 0)

    def on_slider_change(self, value, index):
        real_angle = value / 100.0
        self.spinboxes[index].blockSignals(True)
        self.spinboxes[index].setValue(real_angle)
        self.spinboxes[index].blockSignals(False)
        self.update_joint_data(index, real_angle)

    def on_spinbox_change(self, value, index):
        slider_val = int(value * 100)
        self.sliders[index].blockSignals(True)
        self.sliders[index].setValue(slider_val)
        self.sliders[index].blockSignals(False)
        self.update_joint_data(index, value)

    def update_joint_data(self, index, angle):
        self.current_joints[index] = angle
        # 【修改】使用 Manager
        user_offset = self.tcp_manager.get_active_matrix()
        self.sim.update_simulation(self.current_joints, user_offset)
        self.update_monitor()

    # === 當滑桿放開或輸入框按 Enter 時才呼叫 ===
    def on_control_finished(self):
        self.serial_manager.send_joints(self.current_joints)
        # 選擇性：可以在 Log 顯示一下，確認已發送
        # self.log(f"Manual Move Sent.")

    def setup_cartesian_control(self):
        group = QGroupBox("Cartesian Control")
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 15, 10, 10)
        layout.setSpacing(15)

        step_row = QHBoxLayout()
        step_row.addWidget(QLabel("Step:"))
        
        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(0.1, 100.0)
        self.step_spin.setValue(10.0)
        self.step_spin.setSuffix(" mm/ °")
        self.step_spin.setFixedWidth(180)
        step_row.addWidget(self.step_spin)
        
        btn_undo = QPushButton()
        btn_undo.setIcon(qta.icon('fa5s.undo', color='#1e1e1e'))
        btn_undo.setIconSize(QSize(22, 22))
        btn_undo.setFixedSize(40, 40)
        btn_undo.setStyleSheet(styles.BTN_UNDO_STYLE)
        btn_undo.clicked.connect(self.undo_last_jog)
        step_row.addWidget(btn_undo)
        step_row.addStretch()
        layout.addLayout(step_row)

        grid = QGridLayout()
        grid.setSpacing(10)
        grid.setColumnStretch(2, 1)

        self.mon_xyz = [QLabel("0.00") for _ in range(3)]
        self.mon_rpy = [QLabel("0.00") for _ in range(3)]

        rows_data = [
            ("X",  'x',  "X",    self.mon_xyz[0]),
            ("Y",  'y',  "Y",    self.mon_xyz[1]),
            ("Z",  'z',  "Z",    self.mon_xyz[2]),
            ("Rx", 'rx', "Roll", self.mon_rpy[0]),
            ("Ry", 'ry', "Pitch",self.mon_rpy[1]),
            ("Rz", 'rz', "Yaw",  self.mon_rpy[2]),
        ]

        for i, (name, axis, mon_lbl, mon_widget) in enumerate(rows_data):
            for col, sign_str, sign_val in [(0, "+", 1), (1, "-", -1)]:
                btn = QPushButton(f"{name}{sign_str}")
                btn.setStyleSheet(styles.BTN_JOG_STYLE)
                btn.clicked.connect(lambda ch, a=axis, s=sign_val: self.jog_tcp(a, s))
                grid.addWidget(btn, i, col)

            mon_frame = MonitorFrame(mon_lbl, mon_widget)
            grid.addWidget(mon_frame, i, 3)

        layout.addLayout(grid)
        group.setLayout(layout)
        self.left_panel.addWidget(group, 0)
    
    def open_tcp_settings(self):
        # 將 self.tcp_manager 傳給 Dialog
        dialog = TCPSettingsDialog(self.tcp_manager, self)
        if dialog.exec_():
            # Dialog 關閉後，資料已經在 Manager 裡更新了
            user_offset = self.tcp_manager.get_active_matrix()
            tool_name = self.tcp_manager.get_active_tool_data()['name']
            
            self.log(f"Active Tool Switched: {tool_name}")
            self.sim.update_simulation(self.current_joints, user_offset)
            self.update_monitor()

    def undo_last_jog(self):
        if self.prev_joints:
            self.on_manager_update_joints(self.prev_joints)
            self.log("Undo: Reverted.")
        else:
            self.log("Undo: No history.")
    
    def update_monitor(self):
        T_math_flange = kinematics.forward_kinematics(self.current_joints)
        user_offset = self.tcp_manager.get_active_matrix() # <--- 這裡
        T_real_tcp = T_math_flange @ self.T_hw_fix @ user_offset
        
        x, y, z = T_real_tcp[:3, 3] * 1000.0
        r = kinematics.R.from_matrix(T_real_tcp[:3, :3])
        rpy = r.as_euler('xyz', degrees=True)
        
        self.mon_xyz[0].setText(f"{x:.2f} mm")
        self.mon_xyz[1].setText(f"{y:.2f} mm")
        self.mon_xyz[2].setText(f"{z:.2f} mm")
        self.mon_rpy[0].setText(f"{rpy[0]:.2f}      °")
        self.mon_rpy[1].setText(f"{rpy[1]:.2f}      °")
        self.mon_rpy[2].setText(f"{rpy[2]:.2f}      °")

    def trigger_run_path(self, loop=False):
        user_offset = self.tcp_manager.get_active_matrix() # <--- 這裡
        T_total_offset = self.T_hw_fix @ user_offset
        self.path_manager.run_path(self.current_joints, loop=loop, tcp_offset=T_total_offset)

    def jog_tcp(self, axis, sign):
        if self.path_manager.worker and self.path_manager.worker.isRunning():
            self.log("Path running.")
            return

        self.prev_joints = list(self.current_joints)
        raw_step = self.step_spin.value()
        
        T_math_flange = kinematics.forward_kinematics(self.current_joints)
        user_offset = self.tcp_manager.get_active_matrix() # <--- 這裡
        T_total_offset = self.T_hw_fix @ user_offset
        T_tcp_curr = T_math_flange @ T_total_offset
        
        T_step = np.eye(4)
        
        if axis in ['x', 'y', 'z']:
            step = (raw_step * sign) / 1000.0
            idx = {'x': 0, 'y': 1, 'z': 2}[axis]
            T_step[idx, 3] = step 
            self.log(f"Jog Tool {axis.upper()} {step*1000:.1f} mm")
        else:
            step = raw_step * sign
            rad = np.deg2rad(step)
            vec = np.zeros(3)
            rot_idx = {'x':0, 'y':1, 'z':2}[axis[1]] 
            vec[rot_idx] = rad
            T_step[:3, :3] = kinematics.R.from_rotvec(vec).as_matrix()
            self.log(f"Jog Tool {axis.upper()} {step:.1f} deg")

        T_tcp_target = T_tcp_curr @ T_step
        T_flange_target = T_tcp_target @ np.linalg.inv(T_total_offset)

        new_joints, error_score = kinematics.inverse_kinematics(T_flange_target, self.current_joints)
        
        if new_joints is not None:
            # --- 精度二次驗證 (Double Check) ---
            T_check_flange = kinematics.forward_kinematics(new_joints)

            pos_diff = np.linalg.norm(T_check_flange[:3, 3] - T_flange_target[:3, 3]) * 1000.0 # 轉 mm

            if pos_diff > config.IK_POS_TOLERANCE:
                self.log(f"[Error] IK Inaccurate! Diff: {pos_diff:.3f}mm")
                return # 拒絕移動

            for i, angle in enumerate(new_joints):
                min_lim, max_lim = config.JOINT_LIMITS[i]
                if angle < (min_lim - 0.1) or angle > (max_lim + 0.1):
                    self.log(f"[Error] Limit Hit J{i+1}")
                    return

            self.on_manager_update_joints(list(new_joints))
        else:
            self.log("[Error] IK Failed")

    def setup_waypoints_manager(self):
        group = QGroupBox("Waypoints Manager")
        layout = QVBoxLayout()
        layout.setSpacing(12)
        
        self.waypoint_list = QListWidget()
        self.waypoint_list.itemClicked.connect(self.preview_waypoint)
        self.waypoint_list.itemDoubleClicked.connect(self.rename_waypoint_item)
        layout.addWidget(self.waypoint_list)
        
        settings_row = QHBoxLayout()
        settings_row.addWidget(QLabel("Type:"))
        self.type_combo = QComboBox()
        self.type_combo.addItems(["PTP", "LIN"])
        self.type_combo.setFixedWidth(85)
        settings_row.addWidget(self.type_combo)
        settings_row.addSpacing(15)
        
        settings_row.addWidget(QLabel("Delay:"))
        self.delay_spin = QDoubleSpinBox()
        self.delay_spin.setRange(0.0, 60.0)
        self.delay_spin.setDecimals(1)
        self.delay_spin.setSingleStep(0.5)
        self.delay_spin.setSuffix(" s")
        self.delay_spin.setFixedWidth(110)
        settings_row.addWidget(self.delay_spin)
        settings_row.addSpacing(15)

        settings_row.addWidget(QLabel("Speed:"))
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["50%", "75%", "100%"])
        self.speed_combo.setCurrentText("50%")
        self.speed_combo.setFixedWidth(100)
        self.speed_combo.currentTextChanged.connect(self.on_speed_change)
        settings_row.addWidget(self.speed_combo)
        
        settings_row.addStretch()
        layout.addLayout(settings_row)
        
        btn_row = QHBoxLayout()
        btn_style = "QPushButton { padding: 6px; font-size: 22px; }"
        
        actions = [
            ("Record", self.trigger_record),
            ("Del ALL", self.confirm_delete_all),
            ("Save", self.path_manager.save_to_file),
            ("Load", self.path_manager.load_from_file)
        ]
        
        for text, func in actions:
            b = QPushButton(text)
            b.setStyleSheet(btn_style)
            b.clicked.connect(func)
            btn_row.addWidget(b)

        layout.addLayout(btn_row)
        group.setLayout(layout)
        self.left_panel.addWidget(group, 1)

    def setup_right_panel(self):
        right_panel = QVBoxLayout()
        right_panel.setSpacing(20)
        view_group = QGroupBox("3D View")
        view_layout = QVBoxLayout()
        view_layout.setContentsMargins(0, 0, 0, 0)
        sim_widget = self.sim.get_widget()
        container = QWidget() 
        container_layout = QVBoxLayout(container)
        container_layout.setContentsMargins(15, 20, 15, 15)
        container_layout.addWidget(sim_widget)
        
        def make_btn(icon, tip, func=None, checkable=False, checked=False):
            btn = CircularButton(icon, tip, parent=container, checkable=checkable, checked=checked)
            btn.setFixedSize(55, 55) 
            if func: btn.clicked.connect(func)
            return btn

        self.btn_tcp      = make_btn('fa5s.location-arrow', "Toggle TCP", self.sim.toggle_tcp, True, True)
        self.btn_floor    = make_btn('fa5s.border-all', "Toggle Floor", self.sim.toggle_floor, True, True)
        self.btn_skel     = make_btn('fa5s.project-diagram', "Skeleton Mode", self.on_toggle_skeleton, True, False)
        self.btn_compress = make_btn('fa5s.compress', "Compress View (TBD)")

        def cam_action(func):
            return lambda: (func(), self.sim.fit_view())

        self.btn_fit       = make_btn('fa5s.cube', "Fit View", self.sim.fit_view)
        self.btn_cam_iso   = make_btn('fa5s.sync-alt', "ISO View", self.sim.reset_camera) 
        self.btn_cam_top   = make_btn('fa5s.arrow-down', "Top View", cam_action(self.sim.view_top))
        self.btn_cam_front = make_btn('fa5s.arrow-up', "Front View", cam_action(self.sim.view_front))
        self.btn_cam_side  = make_btn('fa5s.arrow-right', "Side View", cam_action(self.sim.view_side))
        
        btns = [
            self.btn_tcp, self.btn_floor, self.btn_skel, self.btn_compress, 
            self.btn_fit, self.btn_cam_iso, self.btn_cam_top, self.btn_cam_front, self.btn_cam_side
        ]
        for b in btns: b.raise_()
        
        view_layout.addWidget(container)
        view_group.setLayout(view_layout)
        right_panel.addWidget(view_group, 2)
        
        log_group = QGroupBox("System Log")
        log_layout = QVBoxLayout() 
        log_layout.setContentsMargins(10, 15, 10, 10)
        self.log_window = QTextEdit()
        self.log_window.setReadOnly(True)
        self.log_window.installEventFilter(self) 
        self.log_window.setStyleSheet(styles.LOG_WINDOW_STYLE)
        self.btn_clear_log = QPushButton(self.log_window)
        self.btn_clear_log.setCursor(Qt.PointingHandCursor)
        self.btn_clear_log.setIcon(qta.icon('fa5s.trash-alt', color='#7f8c8d'))
        self.btn_clear_log.setIconSize(QSize(22, 22))
        self.btn_clear_log.setFixedSize(32, 32)
        self.btn_clear_log.setToolTip("Clear Log")
        self.btn_clear_log.setStyleSheet(styles.BTN_CLEAR_STYLE)
        self.btn_clear_log.clicked.connect(self.clear_log)
        log_layout.addWidget(self.log_window)
        log_group.setLayout(log_layout)
        right_panel.addWidget(log_group, 1)
        right_widget = QWidget()
        right_widget.setLayout(right_panel)
        self.content_layout.addWidget(right_widget)
    
    def eventFilter(self, source, event):
        if source == self.log_window and event.type() == QEvent.Resize:
            self.reposition_clear_btn()
        return super().eventFilter(source, event)

    def reposition_clear_btn(self):
        if hasattr(self, 'btn_clear_log') and hasattr(self, 'log_window'):
            scroll_width = 12
            margin = 5
            x = self.log_window.width() - self.btn_clear_log.width() - scroll_width - margin
            y = margin
            self.btn_clear_log.move(x, y)

    def reposition_overlays(self):
        if not hasattr(self, 'btn_fit') or not self.btn_fit.parent():
            return
            
        parent = self.btn_fit.parent()
        w, h = parent.width(), parent.height()
        btn_size = 55 
        spacing = 15
        margin_bottom = 40
        margin_side = 40
        margin_left_offset = 40
        
        y_left = h - btn_size - margin_bottom
        self.btn_fit.move(margin_left_offset, y_left)
        y_left -= (btn_size + spacing)
        self.btn_cam_side.move(margin_left_offset, y_left)
        y_left -= (btn_size + spacing)
        self.btn_cam_front.move(margin_left_offset, y_left)
        y_left -= (btn_size + spacing)
        self.btn_cam_top.move(margin_left_offset, y_left)
        y_left -= (btn_size + spacing)
        self.btn_cam_iso.move(margin_left_offset, y_left)
        
        x_right = w - btn_size - margin_side
        y_pos = h - btn_size - margin_bottom
        self.btn_compress.move(x_right, y_pos) 
        y_pos -= (btn_size + spacing)
        self.btn_skel.move(x_right, y_pos)
        y_pos -= (btn_size + spacing)
        self.btn_floor.move(x_right, y_pos)
        y_pos -= (btn_size + spacing)
        self.btn_tcp.move(x_right, y_pos)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.reposition_overlays()

    def on_toggle_skeleton(self, checked):
        self.sim.toggle_skeleton(checked)
        self.on_manager_update_joints(self.current_joints)

    def preview_waypoint(self, item):
        if self.path_manager.worker and self.path_manager.worker.isRunning(): return
        row = self.waypoint_list.row(item)
        if row < 0 or row >= len(self.path_manager.waypoints): return
        target_data = self.path_manager.waypoints[row]
        self.log(f"Previewing: {target_data['name']}")
        self.on_manager_update_joints(target_data['joints'])

    def rename_waypoint_item(self, item):
        row = self.waypoint_list.row(item)
        if row < 0 or row >= len(self.path_manager.waypoints):
            return
        target_data = self.path_manager.waypoints[row]
        old_name = target_data['name']
        from PyQt5.QtWidgets import QInputDialog 
        text, ok = QInputDialog.getText(self, "Rename Point", 
                                      "New Name (Max 15 chars):", 
                                      text=old_name)
        if ok and text:
            new_name = text[:15].strip()
            if not new_name: return 
            self.path_manager.waypoints[row]['name'] = new_name
            self.refresh_waypoint_list()
            self.log(f"Renamed: {old_name} -> {new_name}")

    def trigger_record(self):
        self.path_manager.record_point(self.current_joints, self.delay_spin.value(), self.type_combo.currentText())

    def on_speed_change(self, text):
        try:
            val = int(text.replace("%", ""))
            self.path_manager.set_speed(val)
        except ValueError:
            pass

    def confirm_delete_all(self):
        reply = QMessageBox.question(self, 'Delete All', "Delete ALL waypoints?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.path_manager.delete_all_points()

    # === 【關鍵修改】確認並發送歸零指令 ===
    def confirm_homing(self):
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("WARNING")
        msg_box.setText("警告! 即將進行原點復歸 (Homing)。\n\n手臂進行物理校正移動。\n請確保手臂姿態安全。")
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.Cancel)
        msg_box.setDefaultButton(QMessageBox.Cancel)
        if msg_box.exec_() == QMessageBox.Yes:
            self.log("[HOMING] Sending Trigger (J2=999, J3=999)...")
            
            # 1. 發送歸零指令給 Arduino
            # 格式：[J1, J2, J3, J4, J5, J6]
            # 同時讓 J2 和 J3 歸零
            homing_cmd = [0.0, 999.0, 999.0, 0.0, 0.0, 0.0]
            self.serial_manager.send_joints(homing_cmd)
            
            # 2. 強制同步 GUI 數值 (配合 Arduino 的歸零設定)

            # --- 設定 J2 為 -50 度 ---
            self.current_joints[1] = -50.0
            self.sliders[1].blockSignals(True)
            self.sliders[1].setValue(int(-50.0 * 100))
            self.sliders[1].blockSignals(False)
            self.spinboxes[1].blockSignals(True)
            self.spinboxes[1].setValue(-50.0)
            self.spinboxes[1].blockSignals(False)

            # --- 【新增】設定 J3 為 70 度 ---
            self.current_joints[2] = 70.0
            self.sliders[2].blockSignals(True)
            self.sliders[2].setValue(int(70.0 * 100))
            self.sliders[2].blockSignals(False)
            self.spinboxes[2].blockSignals(True)
            self.spinboxes[2].setValue(70.0)
            self.spinboxes[2].blockSignals(False)

            # ==========================================

            # 3. 更新 3D 模擬畫面與數值顯示
            user_offset = self.tcp_manager.get_active_matrix()
            self.sim.update_simulation(self.current_joints, user_offset)
            self.update_monitor()
            
            self.log("GUI Synced: J2=-50, J3=70")

    def home_robot(self):
        self.log(">> Homing (Software)...")
        self.current_joints = [0.0] * 6
        self.on_manager_update_joints(self.current_joints)

    def clear_log(self):
        self.log_window.clear()

    def log(self, message):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_window.append(f"[{timestamp}] {message}")
        self.log_window.verticalScrollBar().setValue(self.log_window.verticalScrollBar().maximum())

    def on_manager_update_joints(self, joints):
        self.current_joints = joints
        for i, angle in enumerate(self.current_joints):
            self.sliders[i].blockSignals(True)
            self.sliders[i].setValue(int(angle * 100))
            self.sliders[i].blockSignals(False)
            self.spinboxes[i].blockSignals(True)
            self.spinboxes[i].setValue(angle)
            self.spinboxes[i].blockSignals(False)
        user_offset = self.tcp_manager.get_active_matrix()
        self.sim.update_simulation(self.current_joints, user_offset)
        self.update_monitor()
        # 發送給硬體
        self.serial_manager.send_joints(self.current_joints)

    def refresh_waypoint_list(self):
        self.waypoint_list.clear()
        for i, pt in enumerate(self.path_manager.waypoints):
            item = QListWidgetItem(self.waypoint_list)
            row_widget = WaypointRow(
                index=i, 
                data=pt, 
                on_toggle_cb=self.path_manager.toggle_point_active,
                on_delete_cb=self.path_manager.delete_point
            )
            item.setSizeHint(row_widget.sizeHint())
            self.waypoint_list.setItemWidget(item, row_widget)
        self.waypoint_list.scrollToBottom()