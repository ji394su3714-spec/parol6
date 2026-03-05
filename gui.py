# gui.py
import sys
import datetime
import numpy as np
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QGroupBox, 
                             QDoubleSpinBox, QFrame, QDialog,
                             QListWidget, QListWidgetItem, QMessageBox, 
                             QComboBox, QMenu, QAction)
from PyQt5.QtCore import Qt, QSize, QTimer
import qtawesome as qta

from ui import styles
from ui.widgets import CircularButton,  WaypointHeader, WaypointListWidget, WaypointRow, JointControlRow, LogWidget ,CartesianControlRow
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
        
        # Serial Manager
        self.serial_manager = SerialManager()
        self.serial_manager.log_signal.connect(self.log)
        self.serial_manager.connection_state_signal.connect(self.on_serial_connection_changed)
        
        self.current_joints = [0.0] * 6
        self.prev_joints = None  
        
        # 硬體修正矩陣
        self.T_hw_fix = np.eye(4)
        # 如果 config 裡沒有這個變數，提供預設值 -0.0236 作為保險
        self.T_hw_fix[2, 3] = getattr(config, 'HW_Z_OFFSET', -0.0236)
        
        self.path_manager = PathManager(parent=self)
        self.path_manager.log_signal.connect(self.log)
        self.path_manager.joint_update_signal.connect(self.on_manager_update_joints)
        self.path_manager.list_update_signal.connect(self.refresh_waypoint_list)

        self.setStyleSheet(styles.get_global_style())

        self.setup_ui()
        QTimer.singleShot(10, self.reposition_overlays)
        
        self.log("System Initialized.")
        
        user_offset = self.tcp_manager.get_active_matrix()
        self.sim.update_simulation(self.current_joints, user_offset)
        self.update_monitor()
        
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
        btn_home.setStyleSheet(styles.BTN_HOME_STYLE)
        btn_home.clicked.connect(self.home_robot)
        layout.addWidget(btn_home)

        #  HOMING 按鈕  
        btn_homing = QPushButton("HOMING")
        btn_homing.setFixedHeight(BTN_HEIGHT)
        btn_homing.setStyleSheet(styles.BTN_HOMING_STYLE)
        btn_homing.clicked.connect(self.confirm_homing)
        layout.addWidget(btn_homing)
        
        layout.addSpacing(20)

        # --- 硬體連線區域 ---
        lbl_port = QLabel("Port:")
        lbl_port.setStyleSheet("color: #ecf0f1; font-weight: bold; font-size: 16px;")
        layout.addWidget(lbl_port)
        
        self.combo_ports = QComboBox()
        self.combo_ports.setMinimumWidth(120)
        self.combo_ports.setFixedHeight(40)
        self.combo_ports.setStyleSheet(styles.COMBO_PORT_STYLE) 
        self.refresh_ports() 
        layout.addWidget(self.combo_ports)
        
        btn_refresh = QPushButton()
        btn_refresh.setIcon(qta.icon('fa5s.sync-alt', color='white'))
        btn_refresh.setFixedSize(40, 40)
        btn_refresh.setToolTip("Refresh Ports")
        btn_refresh.setStyleSheet(styles.BTN_REFRESH_STYLE) 
        btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(btn_refresh)
        
        self.btn_connect = QPushButton(" Connect")
        self.btn_connect.setIcon(qta.icon('fa5s.plug', color='white'))
        self.btn_connect.setFixedHeight(40)
        self.btn_connect.setStyleSheet(styles.BTN_CONNECT_STYLE) 
        self.btn_connect.clicked.connect(self.toggle_connection)
        layout.addWidget(self.btn_connect)
        
        layout.addSpacing(20)

        btn_tool = QPushButton(" TOOL")
        btn_tool.setFixedHeight(BTN_HEIGHT)
        btn_tool.setIcon(qta.icon('fa5s.tools', color='white'))
        btn_tool.setStyleSheet(styles.BTN_TOOL_STYLE)
        btn_tool.clicked.connect(self.open_tcp_settings)
        layout.addWidget(btn_tool)
        
        layout.addStretch()
        self.root_layout.addWidget(top_bar)

    def open_tcp_settings(self):
        dialog = TCPSettingsDialog(self.tcp_manager, self)
        if dialog.exec_():
            user_offset = self.tcp_manager.get_active_matrix()
            tool_name = self.tcp_manager.get_active_tool_data()['name']
            
            self.log(f"Active Tool Switched: {tool_name}")
            self.sim.update_simulation(self.current_joints, user_offset)
            self.update_monitor()

    def update_monitor(self):
        """計算當前 TCP 的絕對座標，並更新 UI 顯示"""
        T_math_flange = kinematics.forward_kinematics(self.current_joints)
        user_offset = self.tcp_manager.get_active_matrix() 
        T_real_tcp = T_math_flange @ self.T_hw_fix @ user_offset
        
        x, y, z = T_real_tcp[:3, 3] * 1000.0
        r = kinematics.R.from_matrix(T_real_tcp[:3, :3])
        rpy = r.as_euler('xyz', degrees=True)
        
        self.mon_xyz[0].setText(f"{x:.2f} mm")
        self.mon_xyz[1].setText(f"{y:.2f} mm")
        self.mon_xyz[2].setText(f"{z:.2f} mm")
        self.mon_rpy[0].setText(f"{rpy[0]:.2f} °")
        self.mon_rpy[1].setText(f"{rpy[1]:.2f} °")
        self.mon_rpy[2].setText(f"{rpy[2]:.2f} °")

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
            self.serial_manager.connect(port, 250000)

    def on_serial_connection_changed(self, connected):
        if connected:
            self.btn_connect.setText(" Disconnect")
            self.btn_connect.setIcon(qta.icon('fa5s.times', color='white'))
            self.btn_connect.setStyleSheet(styles.BTN_DISCONNECT_STYLE)
            self.combo_ports.setEnabled(False)
        else:
            self.btn_connect.setText(" Connect")
            self.btn_connect.setIcon(qta.icon('fa5s.plug', color='white'))
            self.btn_connect.setStyleSheet(styles.BTN_CONNECT_STYLE)
            self.combo_ports.setEnabled(True)

    def emergency_stop(self):
        self.path_manager.stop_path()
        if self.serial_manager.is_connected and hasattr(self.serial_manager, 'ser'):
            try:
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
        left_widget.setFixedWidth(700)
        self.content_layout.addWidget(left_widget)

    # ===  JointControlRow  ===
    def setup_joint_control(self):
        group = QGroupBox("Joint Control")
        layout = QVBoxLayout() 
        layout.setContentsMargins(15, 15, 15, 10) #(左, 上, 右, 下)
        layout.setSpacing(5)
        
        self.joint_rows = [] 
        
        for i in range(6):
            min_a, max_a = config.JOINT_LIMITS[i]
            row = JointControlRow(i, min_a, max_a)
            row.valueChanged.connect(lambda v, idx=i: self.update_joint_data(idx, v))
            row.editingFinished.connect(self.on_control_finished)
            
            layout.addWidget(row)
            self.joint_rows.append(row)
            
        group.setLayout(layout)
        self.left_panel.addWidget(group, 0)

    def update_joint_data(self, index, angle):
        """當滑桿或按鈕數值改變時，更新內部姿態與 3D 模擬畫面"""
        self.current_joints[index] = angle
        user_offset = self.tcp_manager.get_active_matrix()
        self.sim.update_simulation(self.current_joints, user_offset)
        self.update_monitor()

    def on_control_finished(self):
        """當動作結束(滑鼠放開或連發觸發)時，把陣列送給硬體"""
        # 【新增防護罩】：如果現在正在執行自動路徑動畫，絕對不允許發送手動指令！
        if getattr(self, 'is_animating', False):
            return 
            
        self.serial_manager.send_joints(self.current_joints)

    # --- CartesianControlRow ---
    def setup_cartesian_control(self):
        if not hasattr(self, 'current_step_size'):
            self.current_step_size = 10.0

        group = QGroupBox("Cartesian Control")
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 15, 10, 10)
        layout.setSpacing(15)
 
        top_row = QHBoxLayout() # 頂部控制列
        top_row.setSpacing(10)
        
        top_row.addWidget(QLabel("Frame:"))
        self.coord_combo = QComboBox()
        self.coord_combo.addItems(["World", "Tool"])
        self.coord_combo.setFixedWidth(110)
        top_row.addWidget(self.coord_combo)
        
        top_row.addSpacing(12)
        
        top_row.addWidget(QLabel("Jog:"))
        self.jog_combo = QComboBox()
        self.jog_combo.addItems(["Step", "Cont."])
        self.jog_combo.setCurrentText("Cont.")
        self.jog_combo.setFixedWidth(100)
        self.jog_combo.currentTextChanged.connect(self.on_jog_combo_changed)
        top_row.addWidget(self.jog_combo)
        
        top_row.addSpacing(12)
        
        top_row.addWidget(QLabel("Speed:"))
        self.jog_speed_combo = QComboBox()
        self.jog_speed_combo.addItems(["1%", "10%", "50%", "100%"])
        self.jog_speed_combo.setCurrentText("50%")
        self.jog_speed_combo.setFixedWidth(100)
        top_row.addWidget(self.jog_speed_combo)
        
        top_row.addStretch() 
        
        self.btn_jog_settings = QPushButton()
        self.btn_jog_settings.setIcon(qta.icon('fa5s.cog', color='#34495e'))
        self.btn_jog_settings.setFixedSize(36, 36)
        self.btn_jog_settings.setToolTip("Set Step Size")
        self.btn_jog_settings.setStyleSheet("QPushButton { border: none; background: transparent; } QPushButton:hover { background-color: #bdc3c7; border-radius: 4px; }")
        self.btn_jog_settings.clicked.connect(self.open_jog_settings)
        top_row.addWidget(self.btn_jog_settings)
        
        layout.addLayout(top_row)

        # 核心控制按鈕 
        axis_layout = QVBoxLayout()
        axis_layout.setSpacing(10)

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

        self.cartesian_rows = [] # 改為儲存 Row 物件

        for name, axis, mon_lbl, mon_widget in rows_data:
            row = CartesianControlRow(name, axis, mon_lbl, mon_widget)
            row.jogRequested.connect(self.jog_tcp)
            
            axis_layout.addWidget(row)
            self.cartesian_rows.append(row)

        layout.addLayout(axis_layout)
        group.setLayout(layout)
        self.left_panel.addWidget(group, 0)
        
        self.on_jog_combo_changed(self.jog_combo.currentText())

    def on_jog_combo_changed(self, text):
        is_cont = (text == "Cont.")
        
        self.jog_speed_combo.setEnabled(is_cont)      
        self.btn_jog_settings.setEnabled(not is_cont) 
        
        for row in self.cartesian_rows:
            row.set_auto_repeat(is_cont)

    def open_jog_settings(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Step Settings")
        dialog.setFixedSize(280, 130)
        layout = QVBoxLayout(dialog)

        row = QHBoxLayout()
        row.addWidget(QLabel("Step Size (mm/°):"))
        spin = QDoubleSpinBox()
        spin.setRange(0.01, 100.0)
        spin.setValue(self.current_step_size)
        spin.setDecimals(1)
        row.addWidget(spin)
        layout.addLayout(row)

        btn_ok = QPushButton("OK")
        btn_ok.setFixedHeight(35)
        btn_ok.setStyleSheet(styles.BTN_TOOL_STYLE) 
        btn_ok.clicked.connect(dialog.accept)
        layout.addWidget(btn_ok)

        if dialog.exec_() == QDialog.Accepted:
            self.current_step_size = spin.value()
            self.log(f"Step size updated to: {self.current_step_size} mm/°")

    def jog_tcp(self, axis, sign):
        if self.path_manager.worker and self.path_manager.worker.isRunning():
            self.log("Path running. Cannot jog.")
            return 

        is_cont = (self.jog_combo.currentText() == "Cont.")

        # 解析移動量與速度
        if not is_cont:
            raw_step = getattr(self, 'current_step_size', 10.0)
            self.prev_joints = list(self.current_joints) 
        else:
            speed_str = self.jog_speed_combo.currentText().replace("%", "")
            speed_pct = float(speed_str) / 100.0
            raw_step = 2.0 * speed_pct 
            if raw_step < 0.05: raw_step = 0.05 

        # 準備呼叫參數
        frame = self.coord_combo.currentText() 
        step_val = raw_step * sign
        user_offset = self.tcp_manager.get_active_matrix()
        T_total_offset = self.T_hw_fix @ user_offset
        
        new_joints, error_msg = kinematics.calculate_jog_joints(
            self.current_joints, axis, step_val, frame, T_total_offset
        )

        if new_joints is not None:
            self.on_manager_update_joints(new_joints)
        else:
            self.log(f"[Error] {error_msg}")

    def setup_waypoints_manager(self):
        group = QGroupBox("Waypoints Manager")
        layout = QVBoxLayout()
        layout.setContentsMargins(15, 20, 15, 15)
        layout.setSpacing(12)
        
        # 清單容器 (包含 Header 與 List)
        list_container = QVBoxLayout()
        list_container.setSpacing(0) # 設定為 0，讓標題跟清單緊緊黏在一起
        
        # 1. 加入標題列
        self.waypoint_header = WaypointHeader()
        list_container.addWidget(self.waypoint_header)
        
        # 2. 🌟 加入自訂的 ListWidget (這裡直接呼叫新類別，取代原先的 QListWidget)
        self.waypoint_list = WaypointListWidget() 
        self.waypoint_list.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.waypoint_list.itemClicked.connect(self.preview_waypoint)
        self.waypoint_list.itemDoubleClicked.connect(self.rename_waypoint_item)

        scrollbar = self.waypoint_list.verticalScrollBar()
        scrollbar.rangeChanged.connect(
            lambda min_val, max_val: self.waypoint_header.set_scrollbar_width(12 if max_val > min_val else 0)
        )
        
        list_container.addWidget(self.waypoint_list)
        layout.addLayout(list_container)

        # 3. 加入按鈕列
        btn_row = QHBoxLayout()
        btn_style = "QPushButton { padding: 6px; font-size: 24px; }"
        
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
        self.btn_skel     = make_btn('fa5s.project-diagram', "Skeleton Mode", self.on_toggle_skeleton, True, False)
        self.btn_refresh_traj = make_btn('fa5s.route', "Refresh Trajectory", self.refresh_waypoint_list)

        def cam_action(func):
            return lambda: (func(), self.sim.fit_view())

        self.btn_fit       = make_btn('fa5s.cube', "Fit View", self.sim.fit_view)
        self.btn_cam_iso   = make_btn('fa5s.sync-alt', "ISO View", self.sim.reset_camera) 
        self.btn_cam_top   = make_btn('fa5s.arrow-down', "Top View", cam_action(self.sim.view_top))
        self.btn_cam_front = make_btn('fa5s.arrow-up', "Front View", cam_action(self.sim.view_front))
        self.btn_cam_side  = make_btn('fa5s.arrow-right', "Side View", cam_action(self.sim.view_side))
        
        btns = [
            self.btn_tcp, self.btn_skel, self.btn_refresh_traj,  
            self.btn_fit, self.btn_cam_iso, self.btn_cam_top, self.btn_cam_front, self.btn_cam_side
        ]
        for b in btns: b.raise_()
        
        view_layout.addWidget(container)
        view_group.setLayout(view_layout)
        right_panel.addWidget(view_group, 2)
        
        log_group = QGroupBox("System Log")
        log_layout = QVBoxLayout() 
        log_layout.setContentsMargins(10, 15, 10, 10)
        
        self.log_widget = LogWidget()
        log_layout.addWidget(self.log_widget)
        
        log_group.setLayout(log_layout)
        right_panel.addWidget(log_group, 1)
        right_widget = QWidget()
        right_widget.setLayout(right_panel)
        self.content_layout.addWidget(right_widget)

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
        
        # 1. 將按鈕依照「由下往上」的順序打包成清單
        left_btns = [
            self.btn_fit, self.btn_cam_side, self.btn_cam_front, 
            self.btn_cam_top, self.btn_cam_iso
        ]
        right_btns = [
            self.btn_refresh_traj, self.btn_skel, self.btn_tcp
        ]
        
        y_left = h - btn_size - margin_bottom # 用迴圈自動排列左側按鈕
        for btn in left_btns:
            btn.move(margin_left_offset, y_left)
            y_left -= (btn_size + spacing)
            
        x_right = w - btn_size - margin_side  # 用迴圈自動排列右側按鈕
        y_right = h - btn_size - margin_bottom
        for btn in right_btns:
            btn.move(x_right, y_right)
            y_right -= (btn_size + spacing)

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
        self.on_manager_update_joints(list(target_data['joints']))

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
        default_delay = 0.0
        default_type = "PTP"
        
        self.path_manager.record_point(self.current_joints, default_delay, default_type)
        self.log(f"Recorded Point (Default: {default_type}, {default_delay}s)")
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

    def confirm_homing(self):
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("WARNING")
        msg_box.setText("警告! 即將進行原點復歸 (Homing)。\n\n手臂進行物理校正移動。\n請確保手臂姿態安全。")
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.Cancel)
        msg_box.setDefaultButton(QMessageBox.Cancel)
        if msg_box.exec_() == QMessageBox.Yes:
            self.log("[HOMING] Sending Trigger (J2=999, J3=999)...")
            
            homing_cmd = [999.0, 999.0, 999.0, 999.0, 999.0, 999.0]
            self.serial_manager.send_joints(homing_cmd)
            self.current_joints = [0.0] * 6
            for i in range(6):
                self.joint_rows[i].set_value(0.0) 

            user_offset = self.tcp_manager.get_active_matrix()
            self.sim.update_simulation(self.current_joints, user_offset)
            self.update_monitor()
            
            self.log("GUI Synced: All joints set to 0")

    def home_robot(self):
        self.log(">> Homing (Software)...")
        self.current_joints = [0.0] * 6
        self.on_manager_update_joints(self.current_joints)

    def clear_log(self):
        self.log_widget.clear_log() 

    def log(self, message):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_widget.append_log(f"[{timestamp}] {message}") 

    
    def on_manager_update_joints(self, joints):
        # 1. 打開防護罩：告訴系統「現在是動畫在跑，絕對不准發送 Serial 碎指令！」
        self.is_animating = True 
        
        self.current_joints = joints
        for i, angle in enumerate(self.current_joints):
            # 這行會轉動滑桿
            self.joint_rows[i].set_value(angle) 
            
        # 2. 畫面更新完畢，關閉防護罩
        self.is_animating = False
            
        user_offset = self.tcp_manager.get_active_matrix()
        self.sim.update_simulation(self.current_joints, user_offset)
        self.update_monitor() # 更新 XYZ/RPY 數值
        
        #【修改這裡】：如果現在正在跑自動路徑，UI 只負責更新畫面，不發送硬體指令！
        # 這樣就不會跟 PathManager 的指令打架了
        if self.path_manager.worker and self.path_manager.worker.isRunning():
            return 
            
        # 只有在手動 Jogging 或點擊預覽時，才由 UI 發送指令
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

        # 計算所有點的 XYZ 座標並畫出 3D 軌跡線
        trajectory_points = []
        user_offset = self.tcp_manager.get_active_matrix()
        T_total_offset = self.T_hw_fix @ user_offset
        
        # 1. 篩選出所有啟用的點
        active_wps = [pt for pt in self.path_manager.waypoints if pt.get('active', True)]
        
        # 2. 只有兩個點以上才能連成線
        if len(active_wps) >= 2:
            for i in range(len(active_wps) - 1):
                # 取得起點與終點的關節角度
                joints_start = np.array(active_wps[i]['joints'])
                joints_end = np.array(active_wps[i+1]['joints'])
                
                # 判斷目標點的運動模式 (假設您的點位資料有存 'type' 或 'mode')
                # 如果找不到，這裡預設給 'LIN' (依照您目前的需求)
                move_type = active_wps[i+1].get('type', 'LIN') 
                
                steps = 20
                if move_type == 'PTP':
                    # 【PTP 模式】關節空間插值 -> 產生弧線
                    for t in np.linspace(0, 1, steps):
                        interp_joints = joints_start + t * (joints_end - joints_start)
                        T_tcp = kinematics.forward_kinematics(interp_joints) @ T_total_offset
                        trajectory_points.append(T_tcp[:3, 3])
                else:
                    # 【LIN 模式】笛卡爾空間 (XYZ) 插值 -> 產生絕對直線
                    T_start = kinematics.forward_kinematics(joints_start) @ T_total_offset
                    T_end = kinematics.forward_kinematics(joints_end) @ T_total_offset
                    
                    xyz_start = T_start[:3, 3]
                    xyz_end = T_end[:3, 3]
                    
                    # 直接在 3D 空間中把起點到終點拉一條直直的線
                    for t in np.linspace(0, 1, steps):
                        interp_xyz = xyz_start + t * (xyz_end - xyz_start)
                        trajectory_points.append(interp_xyz)
                    
        if trajectory_points:
            self.sim.draw_trajectory(trajectory_points)
        else:
            # 如果算完發現沒有點 (或不到兩個點)，就傳送空陣列把線清掉
            self.sim.draw_trajectory([])

        # --- 觸發路徑執行函式 ---
    def trigger_run_path(self, loop=False):
        """觸發執行記錄的路徑"""
        if self.path_manager.worker and self.path_manager.worker.isRunning():
            self.log("[Warning] Path is already running!")
            return
            
        active_points = [pt for pt in self.path_manager.waypoints if pt.get('active', True)]
        if not active_points:
            self.log("[Warning] No active waypoints to run!")
            return
            
        self.log(f"Starting path execution... (Loop: {loop})")

        # 抓取目前的使用者 TCP 與硬體修正量
        user_offset = self.tcp_manager.get_active_matrix()
        T_total_offset = self.T_hw_fix @ user_offset

        if hasattr(self.path_manager, 'run_path'):
            self.path_manager.run_path(self.current_joints, loop=loop, tcp_offset=T_total_offset)
        elif hasattr(self.path_manager, 'execute_path'):
            self.path_manager.execute_path(self.current_joints, loop=loop, tcp_offset=T_total_offset)
        else:
            self.log("[Error] Cannot find the run method in path_manager!")