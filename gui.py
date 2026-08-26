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
        """主視窗初始化：建立全域狀態變數、UI 佈局與模組實例化"""
        super().__init__()
        self.setWindowTitle("Parol Stream")
        self.setWindowIcon(QIcon("assets/logo.ico"))
        self.resize(1200, 700)
        self.setStyleSheet(styles.WINDOW_STYLE)
        
        # --- 核心變數與狀態旗標明確初始化 ---
        self.current_float_joints = [0.0] * 6
        self.prev_rpy = None 
        self.is_simulation_mode = False 
        self.pending_3d_update = False
        self._ui_throttle_counter = 0 
        self._is_paused = False
        
        self._is_estopped_latched = False
        self._last_jog_warning_time = 0.0
        self._is_system_updating = False
        self._active_jog_ideal_tcp = None
        self._last_jog_ui_update = 0.0
        self._last_jog_error = None
        
        self._last_preview_time = 0.0
        self._last_preview_index = -1
        self.preview_animation = None
        self._physical_joints_memory = None
        self._last_monitor_locked = False 

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

        self._bind_all_signals(main_splitter, right_splitter, default_main_sizes, default_right_sizes)
        
        self.jog_widget.on_joint_slider_changed()
        self.waypoint_panel.add_new_tab("untitled.json") 

    # =========================================================
    # 雙層互鎖大腦 (System Core Interlocks)
    # =========================================================
    @property
    def is_system_busy(self):
        """判斷系統是否忙碌 (Worker 運行中、Jog 操作中、或動畫播放中)"""
        is_animating = self.preview_animation is not None and self.preview_animation.state() == QVariantAnimation.State.Running
        return self.path_manager.is_running() or self.jog_widget.is_jogging or is_animating

    @property
    def is_math_engine_running(self):
        """判斷微積分大腦是否正在運作，用於阻擋低階硬體通訊衝突"""
        return self.path_manager.is_running() or self.jog_widget.cart_worker._is_active

    def _bind_all_signals(self, m_splitter, r_splitter, m_sizes, r_sizes):
        """集中綁定所有模組之間的 Signal 與 Callback (神經網路對接)"""
        self.top_bar.btn_tools.clicked.connect(self.open_tcp_manager)
        self.top_bar.btn_base.clicked.connect(self.open_base_manager)
        self.top_bar.btn_play.toggled.connect(self.on_play_toggled)
        self.top_bar.btn_stop.clicked.connect(self.stop_execution)
        self.top_bar.btn_estop_reset.clicked.connect(self.reset_estop)
        self.top_bar.btn_soft_home.clicked.connect(self.go_soft_home)
        self.top_bar.btn_simulation.toggled.connect(self.toggle_simulation_mode)
        self.top_bar.btn_connect.clicked.connect(self.toggle_connection)

        self.serial_manager.log_signal.connect(self.log_widget.append_log)
        self.serial_manager.connection_state_signal.connect(self.update_connection_ui)
        self.serial_manager.estop_state_signal.connect(self.on_estop_state_changed)
        self.serial_manager.real_pose_received.connect(self.handle_hardware_pose_update)
        
        self.path_manager.log_signal.connect(self.log_widget.append_log)
        self.path_manager.list_update_signal.connect(self.update_path_list_ui)
        self.path_manager.file_loaded_signal.connect(self.waypoint_panel.set_file_name)

        self.tcp_manager.data_changed.connect(self.path_manager.invalidate_preview_cache)
        self.tcp_manager.data_changed.connect(self.update_3d_trajectory_preview)
        self.base_manager.data_changed.connect(self.path_manager.invalidate_preview_cache)
        self.base_manager.data_changed.connect(self.update_3d_trajectory_preview)
        
        self.view3d_widget.monitor_widget.tcp_edit_requested.connect(self.handle_monitor_tcp_edit)
        self.view3d_widget.monitor_widget.joint_edit_requested.connect(self.handle_monitor_joint_edit)
        
        self.jog_widget.cartesian_jog_callback = lambda a, s, f, c: self.handle_cartesian_jog(a, s, f, is_continuous=c, _from_worker=True)
        self.view3d_widget.robot_view.axis_drag_callback = lambda a, s, f: self.handle_cartesian_jog(a, s, f, is_continuous=True, _from_worker=False)
        self.view3d_widget.robot_view.drag_callback = self.handle_tcp_drag
        self.view3d_widget.robot_view.cancel_gizmo_callback = self.view3d_widget.reset_gizmo_buttons
        
        self.jog_widget.check_estop_callback = self._check_estop_barrier
        self.jog_widget.restore_ui_callback = lambda: self.handle_system_pose_update(self.current_float_joints)
        self.jog_widget.update_3d_callback = self.preview_joint_jog 
        self.jog_widget.send_jog_callback = self.send_joint_jog     
        self.jog_widget.precheck_cartesian_step_callback = self.precheck_cartesian_step
        self.jog_widget.continuous_jog_callback = self.handle_continuous_joint_jog
        self.jog_widget.cartesian_jog_stop_callback = self.handle_cartesian_jog_stop
        self.jog_widget.request_pose_callback = self._safe_request_pose
        self.jog_widget.send_io_callback = self.handle_gripper_jog
        self.jog_widget.gripper_btn.toggled.connect(lambda c: self.jog_widget.g_slider.setValue(100 if c else 0))
        self.jog_widget.gripper_btn.toggled.connect(lambda c: self.handle_gripper_jog(100 if c else 0))
        self.jog_widget.warning_log_callback = self._log_jog_barrier_warning 

        self.waypoint_panel.copy_requested.connect(self.path_manager.copy_points)
        self.waypoint_panel.paste_requested.connect(self.path_manager.paste_points)
        self.waypoint_panel.batch_base_shift_requested.connect(self.handle_batch_base_shift)
        self.waypoint_panel.block_base_shift_requested.connect(self.handle_block_base_shift)
        self.waypoint_panel.record_pt_requested.connect(self.record_waypoint_action)
        self.waypoint_panel.update_tcp_point_requested.connect(self.update_tcp_point_action)
        self.waypoint_panel.insert_special_requested.connect(self.insert_special_point_action)
        self.waypoint_panel.record_aux_requested.connect(lambda: self.path_manager.set_aux_point(self.current_float_joints))
        self.waypoint_panel.update_aux_requested.connect(lambda idx: self.path_manager.update_aux_joints(idx, self.current_float_joints))
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

        app_settings.setting_changed.connect(lambda key, val: self.update_path_list_ui() if key == "show_comments" else None)
        self.shortcut_space.activated.connect(self.handle_global_spacebar)
        
        self.main_splitter_listener = SplitterDoubleClickListener(m_splitter, m_sizes)
        m_splitter.handle(1).installEventFilter(self.main_splitter_listener)
        m_splitter.handle(2).installEventFilter(self.main_splitter_listener)
        self.right_splitter_listener = SplitterDoubleClickListener(r_splitter, r_sizes)
        r_splitter.handle(1).installEventFilter(self.right_splitter_listener)

    def _set_system_ui_locked(self, locked):
        """鎖定或解鎖主要控制面板 (執行路徑時呼叫此函式)"""
        self.jog_widget.set_locked(locked)
        self.waypoint_panel.set_locked(locked)
        self.top_bar.btn_home.setEnabled(not locked)
        self.top_bar.btn_soft_home.setEnabled(not locked)


    # =========================================================
    # [2] 硬體通訊與系統狀態 (Hardware & System States)
    # =========================================================
    def _can_send_hardware(self):
        """檢查是否允許發送實體硬體指令"""
        return self.serial_manager.is_connected and not self.is_simulation_mode
    
    def _safe_request_pose(self):
        """安全要求硬體回報座標 (確保在連線且非模擬模式下)"""
        if self._can_send_hardware():
            self.serial_manager.request_real_pose()
            return True
        return False

    def toggle_connection(self):
        """處理序列埠連線/斷線邏輯與選單"""
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
        """根據連線狀態更新頂部工具列按鈕樣式"""
        if is_connected:
            self.top_bar.btn_connect.setIcon(qta.icon('mdi.connection', color='#c63bbb'))
            self.top_bar.btn_connect.setToolTip("Disconnect")
            self.top_bar.btn_stop.setEnabled(True)
            self.top_bar.btn_estop_reset.setEnabled(False)
            self.log_widget.append_log("[HW] 已連線，請執行原點復歸。")
        else:
            self.top_bar.btn_connect.setIcon(qta.icon('mdi.connection', color='#e0e0e0'))
            self.top_bar.btn_connect.setToolTip("Connect to Serial Port")
            self.top_bar.btn_stop.setEnabled(False)
            self.top_bar.btn_estop_reset.setEnabled(False)

    def toggle_simulation_mode(self, checked):
        """切換虛擬/實體模式，並鎖死防呆"""
        if self.path_manager.is_running():
            self.log_widget.append_log("[WARNING] Cannot switch simulation mode while path is running.")
            self.top_bar.btn_simulation.blockSignals(True)
            self.top_bar.btn_simulation.setChecked(not checked)
            self.top_bar.btn_simulation.blockSignals(False)
            return

        self.is_simulation_mode = checked
        self.jog_widget.is_simulation_mode = checked
        
        if checked:
            self._physical_joints_memory = list(self.current_float_joints)
            self.top_bar.btn_simulation.setIcon(qta.icon('mdi.safety-goggles', color='#00e6b8'))
            self.top_bar.btn_simulation.setToolTip("Disable Simulation Mode")
            self.log_widget.append_log("[System] Simulation mode enabled: Hardware output disabled, 3D simulation only.")
        else:
            self.top_bar.btn_simulation.setIcon(qta.icon('mdi.safety-goggles', color='#e0e0e0'))
            self.top_bar.btn_simulation.setToolTip("Enable Simulation Mode")
            
            if self._physical_joints_memory is not None: 
                self.handle_system_pose_update(self._physical_joints_memory)
            if self._can_send_hardware():
                self.serial_manager.request_real_pose()
                
            self.jog_widget.cart_worker.stop_move()
            self.log_widget.append_log("[System] Simulation mode disabled: Pose synchronized with physical hardware, output restored.")

    def reset_estop(self):
        """發送解除急停訊號給硬體"""
        if self._can_send_hardware():
            self.log_widget.append_log("[Action] Attempting to clear E-STOP latch...")
            self.serial_manager.send_estop_reset()

    def on_estop_state_changed(self, is_latched):
        """攔截硬體急停鎖存狀態，並連動 UI"""
        self._is_estopped_latched = is_latched 
        if is_latched:
            self.top_bar.btn_stop.setEnabled(False)
            self.top_bar.btn_estop_reset.setEnabled(True)
            self.top_bar.btn_play.setEnabled(False) 
            QMessageBox.critical(self, "E-STOP (LATCHED)", "The hardware controller is locked in an E-STOP state!\n\nPlease verify the physical machine is safe, then click the 'Unlock' button to resume operation.")
        else:
            self.top_bar.btn_stop.setEnabled(True)
            self.top_bar.btn_estop_reset.setEnabled(False)
            self.top_bar.btn_play.setEnabled(True) 
            QMessageBox.information(self, "System Notification", "Alarm cleared. The system is ready.")
            
            if self._can_send_hardware():
                QTimer.singleShot(200, self._safe_request_pose)
            
        self._reset_play_ui()

    def _check_estop_barrier(self, silent=False):
        """內部防護網：阻擋在急停狀態下的任何運動要求"""
        if self._is_estopped_latched:
            if not silent:
                self.log_widget.append_log("[WARNING] Machine is in [Latched] state. Press [Estop Reset] to resume.")
            return True
        return False

    def _log_jog_barrier_warning(self, msg):
        """限制警告訊息的發送頻率，避免洗頻"""
        curr_time = time.time()
        if curr_time - self._last_jog_warning_time > 1.5:  
            self.log_widget.append_log(msg)
            self._last_jog_warning_time = curr_time


    # =========================================================
    # [3] UI 畫面與狀態更新 (UI & Pose Updates)
    # =========================================================
    def handle_hardware_pose_update(self, new_joints):
        """接收實體機台的物理座標並同步至 UI (當微積分引擎閒置時)"""
        if self.is_math_engine_running: return 
        
        if self.jog_widget.active_jog_mode == 'WAIT_BRAKE':
            self.jog_widget.update_joints_from_ik(new_joints)
            return
            
        self.handle_system_pose_update(new_joints)
        
    def handle_system_pose_update(self, new_joints):
        """核心狀態更新：將新關節角度寫入記憶體並觸發 3D 渲染旗標"""
        self.current_float_joints = list(new_joints)
        self.pending_3d_update = True
        self._is_system_updating = True
        self.jog_widget.update_joints_from_ik(new_joints)
        self._is_system_updating = False

    def process_3d_update(self):
        """由 100Hz 的 QTimer 觸發，負責 3D 模型與 Monitor 數值的平滑渲染"""
        current_busy = self.is_system_busy
        if self._last_monitor_locked != current_busy:
            self.view3d_widget.monitor_widget.set_locked(current_busy)
            self._last_monitor_locked = current_busy

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
        """計算並更新監控面板上的笛卡爾座標 (過濾掉震盪頻率)"""
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
        """向 PathManager 取得軌跡點位，並更新 3D 預覽線條"""
        if self.path_manager.is_running(): return
        pts = self.path_manager.get_trajectory_preview(
            initial_tcp_offset=self.tcp_manager.get_active_matrix(),
            initial_base_mat=self.base_manager.get_active_matrix()
        )
        
        if hasattr(self.view3d_widget.robot_view, 'draw_trajectory_preview'):
            self.view3d_widget.robot_view.draw_trajectory_preview(pts)

    def update_path_list_ui(self):
        """通知 WaypointPanel 更新路徑清單，並連動更新 3D 預覽"""
        if self.path_manager.is_running(): return
        self.waypoint_panel.update_list(self.path_manager.waypoints)
        self.update_3d_trajectory_preview()

    def handle_monitor_tcp_edit(self, axis_name, new_val):
        """處理直接在監控面板上修改笛卡爾數值的事件"""
        if self.is_system_busy:
            self.log_widget.append_log("[WARNING] Cannot edit TCP while system is busy.")
            self._update_monitor_ui() 
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
        if idx is None: return
        
        curr_val = target_values[idx]
        delta = new_val - curr_val
        
        if abs(delta) < 1e-4:
            self.handle_system_pose_update(self.current_float_joints)
            return
            
        sign = 1 if delta >= 0 else -1
        fixed_step = abs(delta)
        is_rot = len(axis_name) > 1
        
        self.jog_widget._current_frame = "World" 
        self.jog_widget._current_axis_arg = axis_name if is_rot else axis_name.lower()
        self.jog_widget.active_cart_axis = f"{axis_name}{'+' if sign > 0 else '-'}"
        
        cfg_max_speed = config.MAX_ROT_SPEED if is_rot else config.MAX_LIN_SPEED
        cfg_max_accel = config.MAX_ROT_ACCEL if is_rot else config.MAX_LIN_ACCEL
        max_speed = (cfg_max_speed * 0.5) * (self.jog_widget.c_speed_level / 4.0)
        accel = cfg_max_accel * 1.0 
        
        self.jog_widget.active_jog_mode = 'WAIT_BRAKE'
        self.jog_widget.cart_worker.start_move(
            is_continuous=False, 
            fixed_step=fixed_step, 
            max_speed=max_speed, 
            accel=accel, 
            sign=sign
        )

    def handle_monitor_joint_edit(self, joint_idx, new_val):
        """處理直接在監控面板上修改關節角度的事件"""
        if self.is_system_busy:
            self.log_widget.append_log("[WARNING] Cannot edit joints while system is busy.")
            self._update_monitor_ui()
            return

        min_lim, max_lim = config.JOINT_LIMITS[joint_idx]
        if new_val < min_lim or new_val > max_lim:
            self.log_widget.append_log(f"[ERROR] Monitor Edit Failed: J{joint_idx+1} limit is [{min_lim}, {max_lim}].")
            self.handle_system_pose_update(self.current_float_joints) 
            return

        new_joints = list(self.current_float_joints)
        new_joints[joint_idx] = new_val
        self.handle_system_pose_update(new_joints)
        
        if self._can_send_hardware():
            spd = self.jog_widget.j_speed_level * 0.25
            self.serial_manager.send_joints(new_joints, speed_factor=spd, move_mode=0)


    # =========================================================
    # [4] 寸動與手動控制 (Jogging & Manual Control)
    # =========================================================
    def deg_to_steps(self, axis_idx, degrees):
        """工具函式：角度轉實體步數"""
        return int(degrees * config.STEPS_PER_DEG[axis_idx])

    def get_jog_speed_factor(self, level):
        """工具函式：取得面板設定的速率比例"""
        mapping = {1: 0.25, 2: 0.5, 3: 0.75, 4: 1.0}
        return mapping.get(level, 1.0)

    def preview_joint_jog(self, angles):
        """(無通訊) 即時更新畫面的關節拖曳預覽"""
        if self._is_system_updating: return 
        if self.path_manager.is_running(): return
            
        self.current_float_joints = list(angles)
        self.pending_3d_update = True

    def send_joint_jog(self, target_angles_deg, speed_factor):
        """發送關節級別的手動點動指令"""
        if self.is_math_engine_running: return False 
        
        if self._can_send_hardware():
            self.serial_manager.send_joints(target_angles_deg, speed_factor=speed_factor, move_mode=0)
            return True 
            
        return False 

    def handle_continuous_joint_jog(self, axis_idx, direction, speed_factor):
        """發送連續關節點動指令 (Mode 2)"""
        if self.is_math_engine_running: return False 
        
        if self._can_send_hardware():
            dir_correct = int(config.STEPS_PER_DEG[axis_idx] / abs(config.STEPS_PER_DEG[axis_idx]))
            real_direction = direction * dir_correct
            self.serial_manager.send_continuous_jog(axis_idx, real_direction, speed_factor)
            return True  
        return False

    def handle_gripper_jog(self, val):
        """發送 IO 手動控制指令 (例如夾爪開合)"""
        if self.is_math_engine_running: return
            
        if self._check_estop_barrier(): return
        if self._can_send_hardware():
            self.serial_manager.send_io(val) 

    def handle_cartesian_jog(self, axis, step_val, frame, is_continuous=True, _from_worker=False):
        """處理空間級別的微積分點動運算，並發送軌跡"""
        if self.path_manager.is_running(): return
            
        if not _from_worker and self.is_system_busy:
            self._log_jog_barrier_warning("[WARNING] Cannot drag 3D arrow while system is busy.")
            return
        
        tcp_mat = self.tcp_manager.get_active_matrix()
        world_mat = self.base_manager.get_matrix(self.base_manager.current_index)
            
        actual_frame = "Base" if frame == "World" else frame
        last_ideal = self._active_jog_ideal_tcp 
        new_joints, error_msg, ideal_tcp_mat = kinematics.calculate_jog_joints(
            list(self.current_float_joints), axis, step_val, actual_frame, tcp_mat, world_mat, T_last_ideal_tcp=last_ideal
        )
        
        if last_ideal is None and abs(step_val) < 1e-6:
            new_joints = list(self.current_float_joints)
        
        if new_joints is not None:
            if self._can_send_hardware():
                spd_factor = self.get_jog_speed_factor(self.jog_widget.c_speed_level)
                success = self.serial_manager.send_joints(new_joints, speed_factor=spd_factor, move_mode=1, is_stream=True, is_jog=True)
                if not success: return
            
            self._active_jog_ideal_tcp = ideal_tcp_mat
            current_time = time.time()
            
            if current_time - self._last_jog_ui_update >= 0.04:
                self.handle_system_pose_update(new_joints)
                self._last_jog_ui_update = current_time
            else:
                self.current_float_joints = new_joints
                
            self._last_jog_error = None
        else:
            if self.jog_widget.cart_worker:
                self.jog_widget.cart_worker.stop_move()
                
            if error_msg != self._last_jog_error:
                self._log_jog_barrier_warning(f"[WARNING] Cartesian limit reached; protective brake engaged. ({error_msg})")
                self._last_jog_error = error_msg

    def precheck_cartesian_step(self, axis, total_step_val, frame):
        """在執行大步距點動前，預測路徑是否會超出限制 (Lookahead)"""
        if self.path_manager.is_running(): return False

        tcp_mat = self.tcp_manager.get_active_matrix()
        world_mat = self.base_manager.get_matrix(self.base_manager.current_index)
        actual_frame = "Base" if frame == "World" else frame
        
        steps = 5 
        step_inc = total_step_val / steps
        test_joints = list(self.current_float_joints)
        test_ideal = self._active_jog_ideal_tcp
        
        for _ in range(steps):
            res_joints, err, test_ideal = kinematics.calculate_jog_joints(
                test_joints, axis, step_inc, actual_frame, tcp_mat, world_mat, T_last_ideal_tcp=test_ideal
            )
            if res_joints is None:
                self.log_widget.append_log(f"[WARNING] Step lookahead failed: path exceeds limit ({err}), operation blocked.")
                return False
            test_joints = res_joints
            
        return True

    def handle_cartesian_jog_stop(self):
        """當空間點動放開時，清除理想姿態記憶，強制回歸現實座標"""
        self._active_jog_ideal_tcp = None
        self.handle_system_pose_update(self.current_float_joints)

    def handle_tcp_drag(self, target_xyz):
        """處理 3D 畫面直接拖曳 TCP 工具球的 IK 求解"""
        if self.is_system_busy: return
        
        tcp_mat = self.tcp_manager.get_active_matrix()
        T_flange_current = kinematics.forward_kinematics(self.current_float_joints)
        T_tcp_current = T_flange_current @ tcp_mat
        T_tcp_target = np.copy(T_tcp_current)
        T_tcp_target[:3, 3] = target_xyz
        T_flange_target = T_tcp_target @ np.linalg.inv(tcp_mat)
        
        new_joints, error_msg = kinematics.inverse_kinematics(T_flange_target, self.current_float_joints)
        if new_joints is not None:
            self.handle_system_pose_update(new_joints)


    # =========================================================
    # [5] 路徑與設定檔管理 (Waypoint & Profile Management)
    # =========================================================
    def execute_save_process(self):
        """執行存檔，並觸發基座自動同步 (Auto-Sync Base)"""
        self.path_manager.sync_all_base_shifts(self.base_manager)
        self.path_manager.save_to_file()

    def open_tcp_manager(self):
        """開啟 TCP 管理視窗"""
        if self.is_system_busy:
            self.log_widget.append_log("[WARNING] Cannot change TCP config while system is busy.")
            return
        dialog = TCPManagerDialog(self.tcp_manager, self)
        if dialog.exec():
            self.update_path_list_ui()
            self.handle_system_pose_update(self.current_float_joints) 
            active_tool = self.tcp_manager.get_active_tool_data()
            self.log_widget.append_log(f"[System] UPDATED: Tool '{active_tool.get('name', 'Unknown Tool')}' successfully applied.")

    def open_base_manager(self):
        """開啟基座 (Base) 管理視窗"""
        if self.is_system_busy:
            self.log_widget.append_log("[WARNING] Cannot change Base config while system is busy.")
            return
        dialog = BaseManagerDialog(self.base_manager, self)
        if dialog.exec():
            active_base = self.base_manager.get_active_base_data()
            self.log_widget.append_log(f"[System] UPDATED: Base '{active_base.get('name', 'World')}' settings applied.")

    def handle_batch_base_shift(self, indices):
        """處理多選點位的基座批量轉移"""
        current_idx = self.base_manager.current_index
        target_mat = self.base_manager.get_matrix(current_idx)
        target_name = self.base_manager.bases[current_idx]['name']
        self.path_manager.apply_batch_base_shift(indices, target_mat, target_name)
        
    def handle_block_base_shift(self, set_base_idx):
        """處理整個 Block 的基座同步更新"""
        bid = self.path_manager.waypoints[set_base_idx].get('value', 0)
        target_mat = self.base_manager.get_matrix(bid)
        target_name = self.base_manager.bases[bid]['name']
        self.path_manager.apply_base_shift_block(set_base_idx, target_mat, target_name)

    def record_waypoint_action(self, index, pt_type="PTP"):
        """記錄當前姿態，並加入至路徑清單中"""
        current_j = [round(j, 4) for j in self.current_float_joints]
        T_flange_world = kinematics.forward_kinematics(current_j)
        recorded_base_mat = self.base_manager.get_active_matrix()

        aux_j = None
        if pt_type == "CIRC":
            if self.path_manager.temp_aux_joints:
                aux_j = [round(j, 4) for j in self.path_manager.temp_aux_joints]
                self.path_manager.temp_aux_joints = None 
            else:
                self.log_widget.append_log("[ERROR] 無法插入 CIRC：請先移動到中繼點並按下 'Record AUX'！")
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
        """處理各種非運動點位 (TCP, BASE, I/O, LOOP) 的插入操作"""
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
            
        elif pt_type == "LOOP_START":
            new_wp = {"type": "LOOP_START", "value": 1, "active": True}
            
        elif pt_type == "LOOP_END":
            new_wp = {"type": "LOOP_END", "active": True}
            
        elif pt_type == "LOOP_BLOCK":
            end_wp = {"type": "LOOP_END", "active": True}
            start_wp = {"type": "LOOP_START", "value": 1, "active": True}
            self.path_manager.insert_waypoint(index, end_wp)
            self.path_manager.insert_waypoint(index, start_wp)
            self.waypoint_panel.select_row_silently(index) 
            self.update_path_list_ui()
            return
            
        else: return
            
        self.path_manager.insert_waypoint(index, new_wp)
        self.waypoint_panel.select_row_silently(index)  
        self.update_path_list_ui()

    def update_tcp_point_action(self, index, tool_idx):
        """更新已存在的 SET_TCP 點位"""
        tool_data = self.tcp_manager.tools[tool_idx]
        tool_name = tool_data.get("name", "Unknown Tool")
        self.path_manager.update_special_point(index, "SET_TCP", tool_idx, f"Tool: {tool_name}")

    def handle_tab_switch(self, new_tab):
        """處理編輯分頁切換時的資料轉移"""
        if self.waypoint_panel.active_tab:
            self.waypoint_panel.active_tab.waypoints_data = [dict(wp) for wp in self.path_manager.waypoints]
        self.path_manager.waypoints = [dict(wp) for wp in new_tab.waypoints_data]
        self.waypoint_panel.set_active_tab_visuals(new_tab)
        self.path_manager.temp_aux_joints = None
        self.path_manager.invalidate_preview_cache()
        
        self.update_path_list_ui()
        self.update_3d_trajectory_preview()


    # =========================================================
    # [6] 任務執行與動畫 (Execution & Animation)
    # =========================================================
    def go_soft_home(self):
        """觸發安全歸零指令 (Soft Home)"""
        if self._check_estop_barrier(): return
        if self.is_system_busy:
            self.log_widget.append_log("[System] Soft Home request ignored while system is busy.")
            return

        zero_joints = [0.0] * 6
        self._set_system_ui_locked(True) 
        
        if self.is_simulation_mode or not self._can_send_hardware():
            self.log_widget.append_log("[System] Simulation / Disconnected: Soft Home animation triggered.")
            self._play_pose_animation(zero_joints, wp_type='PTP')
            return

        if self._can_send_hardware():
            self.serial_manager.send_pause() 
            time.sleep(0.05) 
            self.serial_manager.send_stop()  
            time.sleep(0.05) 
            self.serial_manager.send_joints(zero_joints, speed_factor=1.0, move_mode=0)

        self._play_pose_animation(zero_joints, wp_type='PTP')

    def stop_execution(self):
        """全面中止機台動作與清空緩衝區"""
        self._reset_play_ui()
        self._is_paused = False
        
        if self.preview_animation and self.preview_animation.state() == QVariantAnimation.State.Running:
            self.preview_animation.stop()
            self._set_system_ui_locked(False)
            
        worker = self.path_manager.worker
        if worker:
            worker._is_paused = False
            
        if self.path_manager.is_running():
            self.path_manager.stop_path()
            
        self._active_jog_ideal_tcp = None
        if self._can_send_hardware():
            self.serial_manager.send_stop()
            QTimer.singleShot(500, self._safe_request_pose)

    def on_play_toggled(self, checked):
        """處理執行與暫停路徑的邏輯樞紐"""
        if checked:
            self.top_bar.btn_play.setIcon(qta.icon('mdi.pause-circle-outline', color='#e6a800'))
            self.top_bar.btn_play.setToolTip("暫停執行 (Pause)")

            if self.path_manager.is_running():
                self.log_widget.append_log("[Stream] Execution resumed...")
                self._is_paused = False
                if self.serial_manager and self.serial_manager.is_connected:
                    self.serial_manager.send_resume()
                    
                worker = self.path_manager.worker 
                if worker:
                    worker._is_paused = False
            else:
                if self.jog_widget.is_jogging:
                    self.log_widget.append_log("[WARNING] Cannot start path while Jogging.")
                    self._reset_play_ui() 
                    return
                
                valid_types = ["PTP", "LIN", "CIRC", "DELAY", "I/O", "LOOP_START", "LOOP_END", "SET_TCP", "SET_BASE", "CAM_PATH"]
                active_points = [pt for pt in self.path_manager.waypoints if pt.get('active', True) and pt.get('type') in valid_types]
                
                if len(active_points) == 0:
                    self.log_widget.append_log("[System] No executable waypoints.")
                    self._reset_play_ui() 
                    return

                self.log_widget.append_log("[Stream] Path streaming started...")
                self._set_system_ui_locked(True)
                
                self._is_paused = False
                self.top_bar.btn_stop.setEnabled(True)

                tcp_mat = self.tcp_manager.get_active_matrix()
                callbacks = {
                    'update': self.handle_system_pose_update, 
                    'error': lambda msg: (self.log_widget.append_log(f"[ERROR] {msg}"), self.stop_execution()),
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
                self.log_widget.append_log("[Stream] Feed Hold (Paused)...")
                self._is_paused = True
                if self.serial_manager and self.serial_manager.is_connected:
                    self.serial_manager.send_pause()
                    
                worker = self.path_manager.worker 
                if worker:
                    worker._is_paused = True

    def _reset_play_ui(self):
        """將 Play 按鈕與系統 UI 重置為非執行狀態"""
        self._is_paused = False 
        self.top_bar.btn_play.blockSignals(True)
        self.top_bar.btn_play.setChecked(False)
        self.top_bar.btn_play.setIcon(qta.icon('mdi.motion-play-outline', color='#00e6b8'))
        self.top_bar.btn_play.setToolTip("開始 / 繼續 (Play/Resume)")
        self.top_bar.btn_play.blockSignals(False)
        
        self._set_system_ui_locked(False)

    def _on_execution_finished(self, total_time):
        """處理 Worker 執行緒完成任務的後續工作"""
        self.log_widget.append_log(f"[Stream] Execution completed! Total time: {total_time:.2f} s")
        self._reset_play_ui() 

    def handle_set_tcp_playback(self, tool_idx):
        """在產線執行途中遇到 SET_TCP 時，同步更新 UI 與模型"""
        self.tcp_manager.set_current_index(tool_idx)
        self.handle_system_pose_update(self.current_float_joints)

    def handle_set_base_playback(self, base_idx):
        """在產線執行途中遇到 SET_BASE 時，同步更新 UI 與模型"""
        self.base_manager.set_current_index(base_idx)

    def handle_waypoint_preview(self, index):
        """單擊列表時的單點預覽引擎 (支援微積分引擎的精準偏移)"""
        current_time = time.time()
        
        if index == self._last_preview_index and (current_time - self._last_preview_time) < 0.5: 
            return  
            
        self._last_preview_time = current_time
        self._last_preview_index = index
            
        if self.is_system_busy or self.is_math_engine_running: 
            self._log_jog_barrier_warning("[WARNING] Cannot preview waypoint while Cart-Worker is moving.")
            return

        if index < 0 or index >= len(self.path_manager.waypoints): return

        target_wp = self.path_manager.waypoints[index]
        wp_type = target_wp.get('type', '')

        # 對於所有非運動指令 (SET_BASE, DELAY, I/O 等)，優先「往上找」物理參考點
        reference_wp = None
        if wp_type in ["PTP", "LIN", "CIRC", "CAM_PATH"]:
            reference_wp = target_wp
        else:
            for i in range(index - 1, -1, -1):
                if self.path_manager.waypoints[i].get('joints') is not None:
                    reference_wp = self.path_manager.waypoints[i]
                    break
                    
            if reference_wp is None:
                for i in range(index + 1, len(self.path_manager.waypoints)):
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

        self.waypoint_panel.set_locked(True)          # 預覽期間鎖定面板，防打滑

        # 1. 區分模擬與非模擬模式
        if self.is_simulation_mode:
            target_joints = reference_wp.get("joints")
            if target_joints:
                self._play_pose_animation(target_joints, wp_type=reference_wp.get("type", "PTP"))
            else:
                self.waypoint_panel.set_locked(False) # 確保解鎖
            return

        # 2. 準備送往微積分引擎的預覽膠囊
        preview_wp = {
            "type": reference_wp.get("type", "PTP"),
            "joints": reference_wp.get("joints"),
            "aux_joints": reference_wp.get("aux_joints"),
            "speed": reference_wp.get("speed", 50.0),   
            "accel": reference_wp.get("accel", 50.0),   
            "cartesian_flange": reference_wp.get("cartesian_flange"),
            "recorded_base_matrix": reference_wp.get("recorded_base_matrix"),
            "blend": "FINE",
            "active": True
        }

        if wp_type not in ["PTP", "LIN", "CIRC", "CAM_PATH"]:
            preview_wp["recorded_base_matrix"] = self.base_manager.get_active_matrix()

        callbacks = {
            'update': self.handle_system_pose_update, 
            'error': lambda msg: (
                self.log_widget.append_log(f"[Preview Error] {msg}"), 
                self.stop_execution(),
                self.waypoint_panel.set_locked(False)
            ),
            'log': lambda msg: None, 
            'finished': lambda t: self.waypoint_panel.set_locked(False), 
        }
        
        serial_ref = self.serial_manager if self._can_send_hardware() else None
        
        # 3. 注入 Base 矩陣當作全域參數發送！
        self.path_manager.execute_streaming_path(
            active_points=[preview_wp], 
            start_joints=self.current_float_joints,
            tcp_offset_mat=self.tcp_manager.get_active_matrix(), 
            base_matrix=self.base_manager.get_active_matrix(),
            loop=False, 
            global_speed=50.0, global_accel=50.0, 
            serial_ref=serial_ref, callbacks=callbacks,
        )

    def _play_pose_animation(self, target_joints, wp_type='PTP'):
        """負責純軟體模擬模式下的補間動畫生成"""
        target_j_array = np.array(target_joints)
        start_j_array = np.array(self.current_float_joints)
        
        if np.allclose(start_j_array, target_j_array, atol=1e-2):
            self.handle_system_pose_update(target_joints)
            self._set_system_ui_locked(False) 
            return

        if self.preview_animation and self.preview_animation.state() == QVariantAnimation.State.Running:
            self.preview_animation.stop()

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
        self.preview_animation.finished.connect(lambda: self._set_system_ui_locked(False))
        self.preview_animation.start()


    # =========================================================
    # [7] 系統事件與彈窗 (System Events & Dialogs)
    # =========================================================
    def handle_global_spacebar(self):
        """派發空白鍵的快速點擊事件 (錄製/選單)"""
        self.waypoint_panel._handle_spacebar()
        self.view3d_widget._handle_spacebar()

    def showEvent(self, event):
        """主視窗顯示事件 (套用 Windows 11 深色標題列)"""
        super().showEvent(event)
        apply_windows_dark_titlebar(self)

    def _prompt_unsaved_changes(self, text_message):
        """共用彈出視窗：攔截未儲存狀態"""
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("Unsaved Changes")
        msg_box.setText(text_message)
        msg_box.setIcon(QMessageBox.Icon.Warning)

        btn_save = msg_box.addButton("Save", QMessageBox.ButtonRole.AcceptRole)
        btn_discard = msg_box.addButton("Don't Save", QMessageBox.ButtonRole.DestructiveRole)
        btn_cancel = msg_box.addButton("Cancel", QMessageBox.ButtonRole.RejectRole)

        msg_box.setDefaultButton(btn_save) 
        msg_box.exec()
        return msg_box.clickedButton(), btn_save, btn_discard, btn_cancel

    def handle_tab_closed(self, closed_tab):
        """處理分頁關閉時的未存檔驗證與記憶體回收"""
        if closed_tab == self.waypoint_panel.active_tab:
            if self.path_manager.is_modified:
                choice, btn_save, btn_discard, btn_cancel = self._prompt_unsaved_changes("Unsaved waypoint changes. Save file?")
                if choice == btn_save:
                    self.execute_save_process()
                    if self.path_manager.is_modified: return           
                elif choice == btn_discard:
                    pass   
                elif choice == btn_cancel:
                    return 

            self.path_manager.waypoints = []
            self.path_manager.is_modified = False  
            self.path_manager.temp_aux_joints = None
            self.path_manager.invalidate_preview_cache()
            
            self.update_path_list_ui()
            self.update_3d_trajectory_preview()
            
        self.waypoint_panel.force_close_tab(closed_tab)

    def closeEvent(self, event):
        """主程式關閉事件：阻擋危險退出與未存檔驗證"""
        if self.path_manager.is_running() or self.jog_widget.is_jogging:
            msg_box = QMessageBox(self)
            msg_box.setWindowTitle("Warning: System Running")
            msg_box.setText("Automatic or manual paths are currently executing!\nPlease stop the machine operations before closing the software.")
            msg_box.setIcon(QMessageBox.Icon.Critical)
            msg_box.setStyleSheet(styles.DARK_MESSAGE_BOX_STYLE)
            
            msg_box.setStandardButtons(QMessageBox.StandardButton.Ok)
            msg_box.exec()
            event.ignore()
            return

        if self.path_manager.is_modified:
            choice, btn_save, btn_discard, btn_cancel = self._prompt_unsaved_changes("Save waypoint changes before exiting?")
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