# gui.py
import copy
import time
import numpy as np

from PySide6.QtWidgets import (QMainWindow, QMenu, QMessageBox, QWidget, 
                               QVBoxLayout, QSplitter, QApplication)
from PySide6.QtCore import QEasingCurve, QVariantAnimation, Qt, QTimer
from PySide6.QtGui import QIcon, QShortcut, QKeySequence 
import qtawesome as qta

# ==========================================
# 內部引擎與設定模組
# ==========================================
import config
import kinematics
from serial_manager import SerialManager
import styles
from path_manager import PathManager

from tcp_manager import TCPManager
from tcp_dialog import TCPManagerDialog
from base_manager import BaseManager
from base_dialog import BaseManagerDialog
from widgets import (app_settings, apply_windows_dark_titlebar, 
                     SplitterDoubleClickListener, GlobalClickFilter, 
                     CustomTopBar, View3DWidget, LogWidget)
from jog_panel import JogWidget
from waypoint_panel import WaypointPanel 

class RobotControllerGUI(QMainWindow):
    # =========================================================
    # [1] 初始化與 UI 佈局 (Initialization & Layout)
    # =========================================================
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Parol Stream")
        self.setWindowIcon(QIcon("assets/logo.ico"))
        self.resize(1200, 700)
        self.setStyleSheet(styles.WINDOW_STYLE)
        
        # --- 核心變數初始化 ---
        self.current_float_joints = [0.0] * 6
        self.prev_rpy = None 
        self.is_simulation_mode = False 
        self.pending_3d_update = False
        self._ui_throttle_counter = 0 
        self._is_paused = False

        # --- UI 佈局組裝 ---
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        self.top_bar = CustomTopBar(self)
        main_layout.addWidget(self.top_bar)

        content_container = QWidget()
        content_layout = QVBoxLayout(content_container)
        content_layout.setContentsMargins(4, 4, 4, 4) 
        content_layout.setSpacing(0)

        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        main_splitter.setHandleWidth(4)

        self.jog_widget = JogWidget()
        self.waypoint_panel = WaypointPanel()
        
        right_splitter = QSplitter(Qt.Orientation.Vertical)
        right_splitter.setHandleWidth(4)
        
        self.view3d_widget = View3DWidget()
        self.log_widget = LogWidget()

        main_splitter.addWidget(self.jog_widget)      
        main_splitter.addWidget(self.waypoint_panel)  
        main_splitter.addWidget(right_splitter)      

        main_splitter.setCollapsible(0, False)        
        main_splitter.setCollapsible(1, True)         
        main_splitter.setCollapsible(2, False)        

        right_splitter.addWidget(self.view3d_widget)  
        right_splitter.addWidget(self.log_widget)     
        right_splitter.setCollapsible(0, False)       
        right_splitter.setCollapsible(1, True)        

        default_right_sizes = [410, 290]      
        default_main_sizes = [300, 450, 450]  
        right_splitter.setSizes(default_right_sizes)
        main_splitter.setSizes(default_main_sizes)

        content_layout.addWidget(main_splitter)
        main_layout.addWidget(content_container)

        # --- 實例化各大腦與管理員 ---
        self.serial_manager = SerialManager()
        self.tcp_manager = TCPManager()
        self.base_manager = BaseManager()
        self.path_manager = PathManager(self)
        self.view3d_widget.set_base_manager(self.base_manager)

        # --- 時鐘與過濾器 ---
        self.global_click_filter = GlobalClickFilter()
        QApplication.instance().installEventFilter(self.global_click_filter)
        
        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.process_3d_update)
        self.render_timer.start(10) 
        
        self.shortcut_space = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        self.shortcut_space.setContext(Qt.ShortcutContext.WindowShortcut)

        # --- 綁定所有訊號與 UI 動作 ---
        self._bind_all_signals(main_splitter, right_splitter, default_main_sizes, default_right_sizes)
        
        # --- 啟動預設狀態 ---
        self.jog_widget.on_joint_slider_changed()
        self.waypoint_panel.add_new_tab("untitled.json") 

    def _bind_all_signals(self, m_splitter, r_splitter, m_sizes, r_sizes):
        """將訊號綁定集中管理"""
        # 工具列
        self.top_bar.btn_tools.clicked.connect(self.open_tcp_manager)
        self.top_bar.btn_base.clicked.connect(self.open_base_manager)
        self.top_bar.btn_play.toggled.connect(self.on_play_toggled)
        self.top_bar.btn_stop.clicked.connect(self.stop_execution)
        self.top_bar.btn_estop_reset.clicked.connect(self.reset_estop)
        self.top_bar.btn_soft_home.clicked.connect(self.go_soft_home)
        self.top_bar.btn_simulation.toggled.connect(self.toggle_simulation_mode)
        self.top_bar.btn_connect.clicked.connect(self.toggle_connection)

        # 系統大腦 & 通訊
        self.serial_manager.log_signal.connect(self.log_widget.append_log)
        self.serial_manager.connection_state_signal.connect(self.update_connection_ui)
        self.serial_manager.estop_state_signal.connect(self.on_estop_state_changed)
        self.serial_manager.real_pose_received.connect(self.handle_hardware_pose_update)
        self.path_manager.log_signal.connect(self.log_widget.append_log)
        self.path_manager.list_update_signal.connect(self.update_path_list_ui)
        self.path_manager.file_loaded_signal.connect(self.waypoint_panel.set_file_name)
        
        # 3D 預覽與 Monitor
        self.tcp_manager.data_changed.connect(self.update_3d_trajectory_preview)
        self.base_manager.data_changed.connect(self.update_3d_trajectory_preview)
        self.view3d_widget.monitor_widget.tcp_edit_requested.connect(self.handle_monitor_tcp_edit)
        self.view3d_widget.monitor_widget.joint_edit_requested.connect(self.handle_monitor_joint_edit)
        self.view3d_widget.robot_view.drag_callback = self.handle_tcp_drag
        self.view3d_widget.robot_view.axis_drag_callback = self.handle_cartesian_jog
        self.view3d_widget.robot_view.cancel_gizmo_callback = self.view3d_widget.reset_gizmo_buttons
        
        # Jogging 面板
        self.jog_widget.update_3d_callback = self.preview_joint_jog 
        self.jog_widget.send_jog_callback = self.send_joint_jog     
        self.jog_widget.cartesian_jog_callback = self.handle_cartesian_jog
        self.jog_widget.continuous_jog_callback = self.handle_continuous_joint_jog
        self.jog_widget.cartesian_jog_stop_callback = self.handle_cartesian_jog_stop
        self.jog_widget.request_pose_callback = self._safe_request_pose
        self.jog_widget.send_gripper_callback = self.handle_gripper_jog
        self.jog_widget.gripper_btn.toggled.connect(lambda c: self.jog_widget.g_slider.setValue(100 if c else 0))
        self.jog_widget.gripper_btn.toggled.connect(lambda c: self.handle_gripper_jog(100 if c else 0))

        # Waypoint 面板
        self.waypoint_panel.copy_requested.connect(self.path_manager.copy_points)
        self.waypoint_panel.paste_requested.connect(self.path_manager.paste_points)
        self.waypoint_panel.batch_base_shift_requested.connect(self.handle_batch_base_shift)
        self.waypoint_panel.block_base_shift_requested.connect(self.handle_block_base_shift)
        self.waypoint_panel.record_pt_requested.connect(self.record_waypoint_action)
        self.waypoint_panel.update_tcp_point_requested.connect(self.update_tcp_point_action)
        self.waypoint_panel.insert_special_requested.connect(self.insert_special_point_action)
        self.waypoint_panel.btn_save.clicked.connect(self.execute_save_process)
        self.waypoint_panel.btn_load.clicked.connect(self.path_manager.load_from_file)
        self.waypoint_panel.clear_all_requested.connect(self.path_manager.delete_all_points)
        self.waypoint_panel.update_pt_requested.connect(lambda idx: self.path_manager.update_point_at_index(idx, self.current_float_joints))
        self.waypoint_panel.toggle_requested.connect(self.path_manager.toggle_point_active)
        self.waypoint_panel.delete_requested.connect(self.path_manager.delete_point)
        self.waypoint_panel.path_list.currentRowChanged.connect(self.handle_waypoint_preview)
        self.waypoint_panel.path_list.itemClicked.connect(lambda item: self.handle_waypoint_preview(self.waypoint_panel.path_list.row(item)))
        self.waypoint_panel.data_changed.connect(self.update_path_list_ui)
        self.waypoint_panel.tab_switch_requested.connect(self.handle_tab_switch)
        self.waypoint_panel.tab_closed_signal.connect(self.handle_tab_closed)

        # 系統事件
        app_settings.setting_changed.connect(lambda key, val: self.update_path_list_ui() if key == "show_comments" else None)
        self.shortcut_space.activated.connect(self.handle_global_spacebar)
        
        # 分割器雙擊重置
        self.main_splitter_listener = SplitterDoubleClickListener(m_splitter, m_sizes)
        m_splitter.handle(1).installEventFilter(self.main_splitter_listener)
        m_splitter.handle(2).installEventFilter(self.main_splitter_listener)
        self.right_splitter_listener = SplitterDoubleClickListener(r_splitter, r_sizes)
        r_splitter.handle(1).installEventFilter(self.right_splitter_listener)


    # =========================================================
    # [2] 硬體通訊與系統狀態 (Hardware & System States)
    # =========================================================
    def _can_send_hardware(self):
        return (self.serial_manager and self.serial_manager.is_connected and not self.is_simulation_mode)
    
    def _safe_request_pose(self):
        if self._can_send_hardware():
            self.serial_manager.request_real_pose()

    def toggle_connection(self):
        if self.serial_manager.is_connected:
            self.serial_manager.disconnect()
        else:
            ports = self.serial_manager.list_ports()
            if not ports:
                self.log_widget.append_log("[HW] 找不到任何可用的 COM Port 裝置。")
                return
            menu = QMenu(self)
            menu.setStyleSheet(styles.MENU_STYLE)
            for port in ports:
                action = menu.addAction(f"Connect to {port}")
                action.triggered.connect(lambda checked, p=port: self.serial_manager.connect(p))
            btn = self.top_bar.btn_connect
            menu.exec(btn.mapToGlobal(btn.rect().bottomLeft()))

    def update_connection_ui(self, is_connected):
        if is_connected:
            self.top_bar.btn_connect.setIcon(qta.icon('mdi.connection', color='#c63bbb'))
            self.top_bar.btn_connect.setToolTip("Disconnect")
            
            self.top_bar.btn_stop.setEnabled(True)
            self.top_bar.btn_estop_reset.setEnabled(False)
            self.log_widget.append_log("[HW] 已連線，請執行原點復歸。若無法移動，請確認是否處於急停鎖存狀態。")
        else:
            self.top_bar.btn_connect.setIcon(qta.icon('mdi.connection', color='#e0e0e0'))
            self.top_bar.btn_connect.setToolTip("Connect to Serial Port")
            
            self.top_bar.btn_stop.setEnabled(False)
            self.top_bar.btn_estop_reset.setEnabled(False)

    def toggle_simulation_mode(self, checked):
        if self.path_manager.is_running():
            self.log_widget.append_log("[警告] 軌跡執行中，禁止切換模擬模式！")
            self.top_bar.btn_simulation.blockSignals(True)
            self.top_bar.btn_simulation.setChecked(not checked)
            self.top_bar.btn_simulation.blockSignals(False)
            return

        self.is_simulation_mode = checked
        self.jog_widget.is_simulation_mode = checked
        
        if checked:
            self._physical_joints_memory = list(self.current_float_joints)
            self.top_bar.btn_simulation.setIcon(qta.icon('mdi.safety-goggles', color='#00e6b8'))
            self.top_bar.btn_simulation.setToolTip("關閉模擬模式")
            self.log_widget.append_log("[System] 模擬模式已開啟：切斷硬體輸出，僅維持 3D 運算。")
        else:
            self.top_bar.btn_simulation.setIcon(qta.icon('mdi.safety-goggles', color='#e0e0e0'))
            self.top_bar.btn_simulation.setToolTip("開啟純模擬模式 (Simulation Mode)")
            
            if hasattr(self, '_physical_joints_memory'):  # 這個是動態生成的變數，必須保留檢查！
                self.handle_system_pose_update(self._physical_joints_memory)
            if self._can_send_hardware():
                self.serial_manager.request_real_pose()
            if hasattr(self.jog_widget, 'cart_worker'):
                self.jog_widget.cart_worker.stop_move()
            self.log_widget.append_log("[System] 模擬模式已關閉：畫面已同步回實體機台姿態，恢復硬體輸出！")

    def reset_estop(self):
        if self._can_send_hardware():
            self.log_widget.append_log(">>> 嘗試解除急停鎖死...")
            self.serial_manager.send_estop_reset()

    def on_estop_state_changed(self, is_latched):
        if is_latched:
            self.top_bar.btn_stop.setEnabled(False)
            self.top_bar.btn_estop_reset.setEnabled(True)
            self.top_bar.btn_play.setEnabled(False) 
            QMessageBox.critical(self, "系統急停 (LATCHED)", "硬體控制器目前處於「急停鎖死狀態」！\n\n請確認實體機台安全後，點擊工具列的「解鎖」按鈕來恢復運作。")
        else:
            self.top_bar.btn_stop.setEnabled(True)
            self.top_bar.btn_estop_reset.setEnabled(False)
            self.top_bar.btn_play.setEnabled(True) 
            QMessageBox.information(self, "系統通知", "警報解除，系統恢復正常就緒。")
            
            if self._can_send_hardware():
                QTimer.singleShot(200, self._safe_request_pose)
            
        self._reset_play_ui()


    # =========================================================
    # [3] UI 畫面與狀態更新 (UI & Pose Updates)
    # =========================================================
    def handle_hardware_pose_update(self, new_joints):
        """過濾硬體回傳的 [POS]：如果在軌跡串流或笛卡爾運算中，拒絕覆寫以維持數學純淨"""
        # 1. 如果 Cartesian Jog 背景引擎正在活躍，拒絕覆寫
        if getattr(self.jog_widget, 'cart_worker', None) and self.jog_widget.cart_worker._is_active:
            return 
        # 2. 如果路徑正在執行中，拒絕覆寫
        if self.path_manager.is_running():
            return 
            
        # 只有在系統真正閒置時，才允許硬體座標同步給 UI
        self.handle_system_pose_update(new_joints)
        
    def handle_system_pose_update(self, new_joints):
        self.current_float_joints = list(new_joints)
        self.pending_3d_update = True
        self.jog_widget.update_joints_from_ik(new_joints)

    def process_3d_update(self):
        if self.pending_3d_update:
            tcp_mat = self.tcp_manager.get_active_matrix()
            self.view3d_widget.robot_view.update_joints(self.current_float_joints, tcp_mat)
            self.pending_3d_update = False
            self._ui_throttle_counter += 1
            if self._ui_throttle_counter >= 3: 
                self._update_monitor_ui() 
        else:
            if self._ui_throttle_counter > 0:
                self._update_monitor_ui() 

    def _update_monitor_ui(self):
        tcp_mat = self.tcp_manager.get_active_matrix()
        T_flange = kinematics.forward_kinematics(self.current_float_joints)
        T_tcp = T_flange @ tcp_mat
        world_mat = self.base_manager.get_matrix(self.base_manager.current_index)
        T_tcp_base = kinematics.remove_base_frame(T_tcp, world_mat)
        
        x, y, z = T_tcp_base[:3, 3] * 1000.0
        self.prev_rpy = kinematics.extract_continuous_rpy(T_tcp_base, self.prev_rpy)
        rx, ry, rz = self.prev_rpy
        
        self.view3d_widget.monitor_widget.update_tcp(x, y, z, rx, ry, rz)
        self.view3d_widget.monitor_widget.update_joints(*self.current_float_joints)
        self._ui_throttle_counter = 0

    def update_3d_trajectory_preview(self):
        if self.path_manager.is_running(): return
        try:
            pts = self.path_manager.get_trajectory_preview(self.tcp_manager.get_active_matrix())
            if hasattr(self.view3d_widget.robot_view, 'draw_trajectory_preview'):
                self.view3d_widget.robot_view.draw_trajectory_preview(pts)
        except Exception:
            pass

    def update_path_list_ui(self):
        if self.path_manager.is_running(): return
        self.waypoint_panel.update_list(self.path_manager.waypoints)
        self.update_3d_trajectory_preview()

    def handle_monitor_tcp_edit(self, axis_name, new_val):
        if self.path_manager.is_running():
            self.log_widget.append_log("[WARNING] Cannot edit position while program is running.")
            self.handle_system_pose_update(self.current_float_joints) 
            return

        T_tool = self.tcp_manager.get_active_matrix()
        T_base = self.base_manager.get_matrix(self.base_manager.current_index)
        T_flange = kinematics.forward_kinematics(self.current_float_joints)
        T_tcp_world = T_flange @ T_tool
        T_tcp_base = kinematics.remove_base_frame(T_tcp_world, T_base)
        
        curr_pos = T_tcp_base[:3, 3] * 1000.0
        curr_rot = kinematics.extract_continuous_rpy(T_tcp_base)
        target_values = list(curr_pos) + list(curr_rot)
        
        axis_map = {"X": 0, "Y": 1, "Z": 2, "Rx": 3, "Ry": 4, "Rz": 5}
        idx = axis_map.get(axis_name)
        if idx is not None: target_values[idx] = new_val

        T_tcp_base_target = kinematics.get_tf_matrix([x / 1000.0 for x in target_values[:3]], np.deg2rad(target_values[3:]))
        T_tcp_world_target = kinematics.apply_base_frame(T_tcp_base_target, T_base)
        T_flange_target = T_tcp_world_target @ np.linalg.inv(T_tool)
        
        new_joints, error = kinematics.inverse_kinematics(T_flange_target, self.current_float_joints)
        if new_joints is not None:
            self.handle_system_pose_update(list(new_joints))
        else:
            self.log_widget.append_log(f"[ERROR] Monitor Edit Failed: Target TCP {axis_name}={new_val} is out of reach or singular.")
            self.handle_system_pose_update(self.current_float_joints)

    def handle_monitor_joint_edit(self, joint_idx, new_val):
        if self.path_manager.is_running():
            self.log_widget.append_log("[WARNING] Cannot edit joints while program is running.")
            self.handle_system_pose_update(self.current_float_joints)
            return

        min_lim, max_lim = config.JOINT_LIMITS[joint_idx]
        if new_val < min_lim or new_val > max_lim:
            self.log_widget.append_log(f"[ERROR] Monitor Edit Failed: J{joint_idx+1} limit is [{min_lim}, {max_lim}].")
            self.handle_system_pose_update(self.current_float_joints) 
            return

        new_joints = list(self.current_float_joints)
        new_joints[joint_idx] = new_val
        self.handle_system_pose_update(new_joints)


    # =========================================================
    # [4] 寸動與手動控制 (Jogging & Manual Control)
    # =========================================================
    def deg_to_steps(self, axis_idx, degrees):
        return int(degrees * config.STEPS_PER_DEG[axis_idx])

    def get_jog_speed_factor(self, level):
        mapping = {1: 0.25, 2: 0.5, 3: 0.75, 4: 1.0}
        return mapping.get(level, 1.0)

    def preview_joint_jog(self, angles):
        self.current_float_joints = list(angles)
        self.pending_3d_update = True

    def send_joint_jog(self, target_angles_deg, speed_factor):
        if self.path_manager.is_running(): return
        if self._can_send_hardware():
            self.serial_manager.send_joints(target_angles_deg, speed_factor=speed_factor, move_mode=0)

    def handle_continuous_joint_jog(self, axis_idx, direction, speed_factor):
        if self._can_send_hardware():
            dir_correct = int(config.STEPS_PER_DEG[axis_idx] / abs(config.STEPS_PER_DEG[axis_idx]))
            real_direction = direction * dir_correct
            self.serial_manager.send_continuous_jog(axis_idx, real_direction, speed_factor)
            return True  
        return False

    def handle_gripper_jog(self, val):
        if self._can_send_hardware():
            self.serial_manager.send_gripper(val)

    def handle_cartesian_jog(self, axis, step_val, frame, is_continuous=True):
        tcp_mat = self.tcp_manager.get_active_matrix()
        world_mat = self.base_manager.get_matrix(self.base_manager.current_index)
            
        actual_frame = "Base" if frame == "World" else frame
        last_ideal = getattr(self, '_active_jog_ideal_tcp', None) 

        # === [核心防線 1] 強制同步真實物理座標 (解決軟硬體離散化誤差) ===
        if last_ideal is None and self._can_send_hardware():
            # 只有在第一幀時，強制向 MCU 請求並等待最新物理座標 (阻塞最多 0.15 秒)
            real_pose = self.serial_manager.sync_get_real_pose(timeout=0.15)
            if real_pose is not None:
                # 無視 UI 當前的顯示值，強行將起點對齊機台的物理齒輪位置
                self.current_float_joints = list(real_pose)

        new_joints, error_msg, ideal_tcp_mat = kinematics.calculate_jog_joints(
            list(self.current_float_joints), axis, step_val, actual_frame, tcp_mat, world_mat, T_last_ideal_tcp=last_ideal
        )
        
        # === [核心防線 2] 消除啟動瞬間的 IK 浮點誤差突波 ===
        if last_ideal is None and abs(step_val) < 1e-6:
            # 第一幀我們放棄 IK 反算結果，強制鎖定為真實關節角度
            new_joints = list(self.current_float_joints)
        
        if new_joints is not None:
            if self._can_send_hardware():
                spd_factor = self.get_jog_speed_factor(self.jog_widget.c_speed_level)
                success = self.serial_manager.send_joints(new_joints, speed_factor=spd_factor, move_mode=1, is_stream=True, is_jog=True)
                if not success: return
            
            self._active_jog_ideal_tcp = ideal_tcp_mat
            current_time = time.time()
            last_ui_update = getattr(self, '_last_jog_ui_update', 0.0)
            
            if current_time - last_ui_update >= 0.04:
                self.handle_system_pose_update(new_joints)
                self._last_jog_ui_update = current_time
            else:
                self.current_float_joints = new_joints
                
            self._last_jog_error = None
        else:
            self._active_jog_ideal_tcp = None
            last_err = getattr(self, '_last_jog_error', None)
            if error_msg != last_err:
                self.log_widget.append_log(f"[Jog Warning] {error_msg}")
                self._last_jog_error = error_msg

    def handle_cartesian_jog_stop(self):
        self._active_jog_ideal_tcp = None
        self.handle_system_pose_update(self.current_float_joints)

    def handle_tcp_drag(self, target_xyz):
        tcp_mat = self.tcp_manager.get_active_matrix()
        T_flange_current = kinematics.forward_kinematics(self.current_float_joints)
        T_tcp_current = T_flange_current @ tcp_mat
        T_tcp_target = np.copy(T_tcp_current)
        T_tcp_target[:3, 3] = target_xyz
        T_flange_target = T_tcp_target @ np.linalg.inv(tcp_mat)
        try:
            new_joints, error_msg = kinematics.inverse_kinematics(T_flange_target, self.current_float_joints)
            if new_joints is not None:
                self.handle_system_pose_update(new_joints)
        except AttributeError:
            pass


    # =========================================================
    # [5] 路徑與設定檔管理 (Waypoint & Profile Management)
    # =========================================================
    def execute_save_process(self):
        self.path_manager.sync_all_base_shifts(self.base_manager)
        self.path_manager.save_to_file()

    def open_tcp_manager(self):
        if self.path_manager.is_running():
            self.log_widget.append_log("[WARNING] Cannot change TCP config while program is running.")
            return
        dialog = TCPManagerDialog(self.tcp_manager, self)
        if dialog.exec():
            self.update_path_list_ui()
            self.handle_system_pose_update(self.current_float_joints) 
            active_tool = self.tcp_manager.get_active_tool_data()
            self.log_widget.append_log(f"[System] UPDATED: Tool '{active_tool.get('name', 'Unknown Tool')}' successfully applied.")

    def open_base_manager(self):
        if self.path_manager.is_running():
            self.log_widget.append_log("[WARNING] Cannot change Base config while program is running.")
            return
        dialog = BaseManagerDialog(self.base_manager, self)
        if dialog.exec():
            active_base = self.base_manager.get_active_base_data()
            self.log_widget.append_log(f"[System] UPDATED: Base '{active_base.get('name', 'World')}' settings applied.")

    def handle_batch_base_shift(self, indices):
        current_idx = self.base_manager.current_index
        target_mat = self.base_manager.get_matrix(current_idx)
        target_name = self.base_manager.bases[current_idx]['name']
        self.path_manager.apply_batch_base_shift(indices, target_mat, target_name)
        
    def handle_block_base_shift(self, set_base_idx):
        bid = self.path_manager.waypoints[set_base_idx].get('value', 0)
        target_mat = self.base_manager.get_matrix(bid)
        target_name = self.base_manager.bases[bid]['name']
        self.path_manager.apply_base_shift_block(set_base_idx, target_mat, target_name)

    def record_waypoint_action(self, index, pt_type="PTP"):
        current_j = [round(j, 4) for j in self.current_float_joints]
        T_flange_world = kinematics.forward_kinematics(current_j)
        recorded_base_mat = self.base_manager.get_active_matrix()

        aux_j = None
        if pt_type == "CIRC":
            if hasattr(self.path_manager, 'temp_aux_joints') and self.path_manager.temp_aux_joints:
                aux_j = [round(j, 4) for j in self.path_manager.temp_aux_joints]
                self.path_manager.temp_aux_joints = None 
            else:
                if hasattr(self, 'log_widget'): self.log_widget.append_log("[ERROR] 無法插入 CIRC：請先移動到中繼點並按下 'Record AUX'！")
                return

        new_wp = {
            "type": pt_type, "name": f"Point", "joints": copy.deepcopy(current_j), "aux_joints": aux_j,
            "cartesian_flange": np.round(T_flange_world, 4).tolist(),       
            "recorded_base_matrix": np.round(recorded_base_mat, 4).tolist(),
            "speed": 50.0, "accel": 50.0, "blend": "FINE", "active": True, "note": ""
        }
        self.path_manager.insert_waypoint(index, new_wp)
        self.waypoint_panel.path_list.setCurrentRow(index)

    def insert_special_point_action(self, index, pt_type):
        if pt_type.startswith("SET_TCP"):
            tool_idx = int(pt_type.split(":")[1]) if ":" in pt_type else self.tcp_manager.current_index
            tool_name = self.tcp_manager.tools[tool_idx].get("name", "Unknown Tool")
            new_wp = {"type": "SET_TCP", "value": tool_idx, "name": f"Tool: {tool_name}", "active": True}

        elif pt_type.startswith("SET_BASE"):
            base_idx = int(pt_type.split(":")[1]) if ":" in pt_type else self.base_manager.current_index
            base_name = self.base_manager.bases[base_idx].get("name", "Unknown Base")
            new_wp = {"type": "SET_BASE", "value": base_idx, "name": f"Base: {base_name}", "active": True}
            
        elif pt_type == "DELAY":
            new_wp = {"type": "DELAY", "value": 1.0, "active": True}
            
        elif pt_type == "IO":
            val = self.jog_widget.g_slider.value()
            new_wp = {"type": "I/O", "action_type": "SERVO", "value": val, "note": f"Grip {val}%", "active": True}
        else: return
            
        self.path_manager.insert_waypoint(index, new_wp)
        if hasattr(self.waypoint_panel, 'select_row_silently'):
            self.waypoint_panel.select_row_silently(index)
        else:
            self.waypoint_panel.path_list.setCurrentRow(index)
        self.update_path_list_ui()

    def update_tcp_point_action(self, index, tool_idx):
        tool_data = self.tcp_manager.tools[tool_idx]
        tool_name = tool_data.get("name", "Unknown Tool")
        self.path_manager.update_special_point(index, "SET_TCP", tool_idx, f"Tool: {tool_name}")

    def handle_tab_switch(self, new_tab):
        if self.waypoint_panel.active_tab:
            self.waypoint_panel.active_tab.waypoints_data = [dict(wp) for wp in self.path_manager.waypoints]
        self.path_manager.waypoints = [dict(wp) for wp in new_tab.waypoints_data]
        self.waypoint_panel.set_active_tab_visuals(new_tab)
        self.update_path_list_ui()


    # =========================================================
    # [6] 任務執行與動畫 (Execution & Animation)
    # =========================================================
    def go_soft_home(self):
        if self.path_manager.is_running():
            self.log_widget.append_log("[System] 路徑執行中，忽略 Soft Home 請求。")
            return

        zero_joints = [0.0] * 6
        if getattr(self, 'is_simulation_mode', False) or not self._can_send_hardware():
            self._play_pose_animation(zero_joints, wp_type='PTP')
            self.log_widget.append_log("[System] 模擬模式/未連線：已觸發 Soft Home 動畫。")
            return

        if self._can_send_hardware():
            self.serial_manager.send_pause() 
            time.sleep(0.05) 
            self.serial_manager.send_stop()  
            time.sleep(0.05) 
            self.serial_manager.send_joints(zero_joints, speed_factor=1.0, move_mode=0)

        self._play_pose_animation(zero_joints, wp_type='PTP')

    def stop_execution(self):
        self._reset_play_ui()
        if self.path_manager.is_running():
            self.path_manager.stop_path()
            
        self._active_jog_ideal_tcp = None
        if self._can_send_hardware():
            self.serial_manager.send_stop()
            QTimer.singleShot(500, self._safe_request_pose)

    def on_play_toggled(self, checked):
        if checked:
            self.top_bar.btn_play.setIcon(qta.icon('mdi.pause-circle-outline', color='#e6a800'))
            self.top_bar.btn_play.setToolTip("暫停執行 (Pause)")

            if self.path_manager.is_running():
                self.log_widget.append_log(">>> 恢復執行...")
                self._is_paused = False
                if self.serial_manager and self.serial_manager.is_connected:
                    self.serial_manager.send_resume()
                    
                worker = getattr(self.path_manager, 'worker', None)
                if worker:
                    worker._is_paused = False
            else:
                valid_types = ["PTP", "LIN", "CIRC", "DELAY", "GRIPPER", "I/O", "LOOP_START", "LOOP_END", "SET_TCP", "SET_BASE", "CAM_PATH"]
                active_points = [pt for pt in self.path_manager.waypoints if pt.get('active', True) and pt.get('type') in valid_types]
                
                if len(active_points) == 0:
                    self.log_widget.append_log("[System] 警告: 沒有可執行的點位。")
                    self._reset_play_ui() 
                    return

                self.log_widget.append_log(">>> 開始執行路徑串流...")
                self.view3d_widget.monitor_widget.set_locked(True)
                self._is_paused = False
                self.top_bar.btn_stop.setEnabled(True)
                
                tcp_mat = self.tcp_manager.get_active_matrix()
                callbacks = {
                    'update': self.handle_system_pose_update, 
                    'error': lambda msg: (self.log_widget.append_log(f"[ERROR] {msg}"), self._reset_play_ui()),
                    'log': self.log_widget.append_log,
                    'finished': self._on_execution_finished,
                    'set_tcp': self.handle_set_tcp_playback,
                    'set_base': self.handle_set_base_playback 
                }
                
                serial_ref = self.serial_manager if self._can_send_hardware() else None
                self.path_manager.execute_streaming_path(
                    active_points=active_points, start_joints=self.current_float_joints,
                    tcp_offset_mat=tcp_mat, loop=False, global_speed=50.0, global_accel=50.0,
                    serial_ref=serial_ref, callbacks=callbacks,
                )
        else:
            self.top_bar.btn_play.setIcon(qta.icon('mdi.motion-play-outline', color='#00e6b8'))
            self.top_bar.btn_play.setToolTip("繼續執行 (Resume)")
            
            if self.path_manager.is_running():
                self.log_widget.append_log(">>> 執行暫停 (Feed Hold)")
                self._is_paused = True
                if self.serial_manager and self.serial_manager.is_connected:
                    self.serial_manager.send_pause()
                    
                worker = getattr(self.path_manager, 'worker', None)
                if worker:
                    worker._is_paused = True

    def _reset_play_ui(self):
        self._is_paused = False 
        
        self.top_bar.btn_play.blockSignals(True)
        self.top_bar.btn_play.setChecked(False)
        self.top_bar.btn_play.setIcon(qta.icon('mdi.motion-play-outline', color='#00e6b8'))
        self.top_bar.btn_play.setToolTip("開始 / 繼續 (Play/Resume)")
        self.top_bar.btn_play.blockSignals(False)
        
        self.view3d_widget.monitor_widget.set_locked(False)

    def _on_execution_finished(self, total_time):
        self.log_widget.append_log(f">>> 執行完成！總耗時 {total_time:.2f} 秒")
        self._reset_play_ui() 

    def handle_set_tcp_playback(self, tool_idx):
        self.tcp_manager.set_current_index(tool_idx)
        self.handle_system_pose_update(self.current_float_joints)

    def handle_set_base_playback(self, base_idx):
        self.base_manager.set_current_index(base_idx)

    def handle_waypoint_preview(self, index):
        current_time = time.time()
        last_time = getattr(self, '_last_preview_time', 0.0)
        last_idx = getattr(self, '_last_preview_index', -1)
        
        if index == last_idx and (current_time - last_time) < 0.5: return  
            
        self._last_preview_time = current_time
        self._last_preview_index = index

        if self.path_manager.is_running(): return
        if index < 0 or index >= len(self.path_manager.waypoints): return

        target_wp = self.path_manager.waypoints[index]
        wp_type = target_wp.get('type', '')

        reference_wp = None
        if wp_type in ["PTP", "LIN", "CIRC"]:
            reference_wp = target_wp
        else:
            search_forward = (wp_type == "SET_BASE") 
            step = 1 if search_forward else -1
            for i in range(index + step, len(self.path_manager.waypoints) if search_forward else -1, step):
                if self.path_manager.waypoints[i].get('joints') is not None:
                    reference_wp = self.path_manager.waypoints[i]
                    break
                    
            if reference_wp is None:
                step = -1 if search_forward else 1
                for i in range(index + step, len(self.path_manager.waypoints) if not search_forward else -1, step):
                    if self.path_manager.waypoints[i].get('joints') is not None:
                        reference_wp = self.path_manager.waypoints[i]
                        break

        if reference_wp is None or reference_wp.get('joints') is None: return

        target_tool_idx = 0  
        for i in range(index, -1, -1):
            if self.path_manager.waypoints[i].get('type') == 'SET_TCP':
                target_tool_idx = int(self.path_manager.waypoints[i].get('value', 0))
                break
        
        target_base_idx = 0  
        for i in range(index, -1, -1):
            if self.path_manager.waypoints[i].get('type') == 'SET_BASE':
                target_base_idx = int(self.path_manager.waypoints[i].get('value', 0))
                break
        
        if self.tcp_manager.current_index != target_tool_idx:
            self.tcp_manager.blockSignals(True)
            self.tcp_manager.set_current_index(target_tool_idx)
            self.tcp_manager.blockSignals(False)
            
        if self.base_manager.current_index != target_base_idx:
            self.base_manager.blockSignals(True)
            self.base_manager.set_current_index(target_base_idx)
            self.base_manager.blockSignals(False)

        if getattr(self, 'is_simulation_mode', False):
            target_joints = reference_wp.get("joints")
            if target_joints:
                self._play_pose_animation(target_joints, wp_type=reference_wp.get("type", "PTP"))
            return

        preview_wp = {
            "type": reference_wp.get("type", "PTP"),
            "joints": reference_wp.get("joints"),
            "aux_joints": reference_wp.get("aux_joints"),
            "speed": reference_wp.get("speed", 50.0),   
            "accel": reference_wp.get("accel", 50.0),   
            "cartesian_flange": reference_wp.get("cartesian_flange"),
            "recorded_base_matrix": reference_wp.get("recorded_base_matrix"),
            "blend": "FINE" 
        }

        callbacks = {
            'update': self.handle_system_pose_update, 
            'error': lambda msg: self.log_widget.append_log(f"[Preview Error] {msg}"),
            'log': lambda msg: None, 
            'finished': lambda t: None, 
        }
        
        serial_ref = self.serial_manager if self._can_send_hardware() else None
        self.path_manager.execute_streaming_path(
            active_points=[preview_wp], start_joints=self.current_float_joints,
            tcp_offset_mat=self.tcp_manager.get_active_matrix(), loop=False, 
            global_speed=50.0, global_accel=50.0, serial_ref=serial_ref, callbacks=callbacks,
        )

    def _play_pose_animation(self, target_joints, wp_type='PTP'):
        target_j_array = np.array(target_joints)
        start_j_array = np.array(self.current_float_joints)
        
        if np.allclose(start_j_array, target_j_array, atol=1e-2):
            self.handle_system_pose_update(target_joints)
            return

        anim = getattr(self, 'preview_animation', None)
        if anim and anim.state() == QVariantAnimation.State.Running:
            anim.stop()

        preview_path = []
        if wp_type == 'LIN':
            from scipy.spatial.transform import Slerp, Rotation as R
            T_start = kinematics.forward_kinematics(start_j_array)
            T_end = kinematics.forward_kinematics(target_j_array)
            pos_s, pos_e = T_start[:3, 3], T_end[:3, 3]
            rot_s, rot_e = T_start[:3, :3], T_end[:3, :3]
            
            try:
                slerp = Slerp([0, 1], R.from_matrix([rot_s, rot_e]))
                steps = 20  
                seed = start_j_array.copy()
                for t in np.linspace(0, 1, steps):
                    T_step = np.eye(4)
                    T_step[:3, 3] = pos_s + t * (pos_e - pos_s)
                    T_step[:3, :3] = slerp(t).as_matrix()
                    
                    res, _ = kinematics.inverse_kinematics(T_step, seed, max_retries=0) 
                    if res is not None:
                        seed = res
                        preview_path.append(res)
                    else:
                        fallback_j = start_j_array + t * (target_j_array - start_j_array)
                        preview_path.append(fallback_j)
                preview_path = np.array(preview_path)
            except Exception:
                preview_path = []

        self.preview_animation = QVariantAnimation(self)
        self.preview_animation.setDuration(600)
        self.preview_animation.setStartValue(0.0)
        self.preview_animation.setEndValue(1.0)
        self.preview_animation.setEasingCurve(QEasingCurve.Type.InOutQuad)

        def on_preview_step(progress):
            if wp_type == 'LIN' and len(preview_path) > 0:
                float_idx = progress * (len(preview_path) - 1)
                idx_floor = int(np.floor(float_idx))
                idx_ceil = min(idx_floor + 1, len(preview_path) - 1)
                weight = float_idx - idx_floor
                current_j = preview_path[idx_floor] * (1.0 - weight) + preview_path[idx_ceil] * weight
            else:
                current_j = start_j_array + (target_j_array - start_j_array) * progress
                
            self.handle_system_pose_update(current_j.tolist())

        self.preview_animation.valueChanged.connect(on_preview_step)
        self.preview_animation.start()


    # =========================================================
    # [7] 系統事件與彈窗 (System Events & Dialogs)
    # =========================================================
    def handle_global_spacebar(self):
        self.waypoint_panel._handle_spacebar()
        self.view3d_widget._handle_spacebar()

    def showEvent(self, event):
        super().showEvent(event)
        apply_windows_dark_titlebar(self)

    def _prompt_unsaved_changes(self, text_message):
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("未儲存提示")
        msg_box.setText(text_message)
        msg_box.setIcon(QMessageBox.Icon.Warning)

        btn_save = msg_box.addButton("儲存", QMessageBox.ButtonRole.AcceptRole)
        btn_discard = msg_box.addButton("不儲存", QMessageBox.ButtonRole.DestructiveRole)
        btn_cancel = msg_box.addButton("取消", QMessageBox.ButtonRole.RejectRole)

        msg_box.setDefaultButton(btn_save) 
        msg_box.exec()
        return msg_box.clickedButton(), btn_save, btn_discard, btn_cancel

    def handle_tab_closed(self, closed_tab):
        if closed_tab == self.waypoint_panel.active_tab:
            if self.path_manager.is_modified:
                choice, btn_save, btn_discard, btn_cancel = self._prompt_unsaved_changes("目前有未儲存的點位變更，請問要儲存檔案嗎？")
                if choice == btn_save:
                    self.execute_save_process()
                    if self.path_manager.is_modified: return           
                elif choice == btn_discard:
                    pass   
                elif choice == btn_cancel:
                    return 

            self.path_manager.waypoints = []
            self.path_manager.is_modified = False  
            self.update_path_list_ui()
            
        self.waypoint_panel.force_close_tab(closed_tab)

    def closeEvent(self, event):
        if self.path_manager.is_modified:
            choice, btn_save, btn_discard, btn_cancel = self._prompt_unsaved_changes("退出前，是否要儲存目前的點位變更？")
            if choice == btn_save:
                self.execute_save_process() 
                if self.path_manager.is_modified: event.ignore() 
                else: event.accept() 
            elif choice == btn_discard:
                event.accept() 
            elif choice == btn_cancel:
                event.ignore() 
        else:
            event.accept()