# gui.py
import copy
import os

import numpy as np
from PySide6.QtWidgets import QMainWindow, QMenu, QMessageBox, QWidget, QVBoxLayout, QSplitter, QApplication
from PySide6.QtCore import QEasingCurve, QVariantAnimation, Qt, QTimer
from PySide6.QtGui import QCursor, QIcon, QShortcut, QKeySequence 

import qtawesome as qta

# ==========================================
# 內部引擎與設定模組
# ==========================================
import kinematics
from serial_manager import SerialManager
import styles
from path_manager import PathManager

# ==========================================
# 客製化 UI 模組
# ==========================================
from tcp_manager import TCPManager
from tcp_dialog import TCPManagerDialog
from base_manager import BaseManager
from base_dialog import BaseManagerDialog
from widgets import (app_settings, apply_windows_dark_titlebar, 
                     SplitterDoubleClickListener, GlobalClickFilter, 
                     CustomTopBar, JogWidget, View3DWidget, LogWidget
                     )
from waypoint_panel import WaypointPanel 

# --- 主視窗排版組裝 ---
class RobotControllerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Parol Stream")
        self.setWindowIcon(QIcon("assets/logo.ico"))
        self.resize(1200, 700)
        self.setStyleSheet(styles.WINDOW_STYLE)
        

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

        # === 左右主分割器 ===
        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        main_splitter.setHandleWidth(4)

        self.jog_widget = JogWidget()
        self.waypoint_panel = WaypointPanel()
        
        # === 上下右分割器 ===
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

        # ==========================================
        # 實例化路徑總管 (Path Manager) 大腦
        # ==========================================
        self.path_manager = PathManager(self)
        self.path_manager.log_signal.connect(self.log_widget.append_log)
        self.path_manager.list_update_signal.connect(self.update_path_list_ui)

        # 掛載全域點擊雷達
        self.global_click_filter = GlobalClickFilter()
        QApplication.instance().installEventFilter(self.global_click_filter)

        # ==========================================
        # 綁定 3D 畫布與笛卡爾 IK 運算引擎
        # ==========================================
        self.current_float_joints = [0.0] * 6
        self.prev_rpy = None 
        self.tcp_manager = TCPManager()
        self.base_manager = BaseManager() 
        self.tcp_manager.data_changed.connect(self.update_3d_trajectory_preview)
        self.base_manager.data_changed.connect(self.update_3d_trajectory_preview)

        self.view3d_widget.set_base_manager(self.base_manager)

        self.serial_manager = SerialManager()
        self.serial_manager.log_signal.connect(self.log_widget.append_log)
        self.serial_manager.connection_state_signal.connect(self.update_connection_ui)
        
        self.path_manager.serial_manager = self.serial_manager

        self.top_bar.btn_tools.clicked.connect(self.open_tcp_manager)
        self.top_bar.btn_base.clicked.connect(self.open_base_manager)
        self.pending_3d_update = False
        self._ui_throttle_counter = 0 
        
        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.process_3d_update)
        self.render_timer.start(16) 

        # 綁定 JogWidget 的各項神經
        self.jog_widget.update_3d_callback = self.preview_joint_jog 
        self.jog_widget.send_jog_callback = self.send_joint_jog     
        self.jog_widget.cartesian_jog_callback = self.handle_cartesian_jog
        self.jog_widget.continuous_jog_callback = self.handle_continuous_joint_jog

        # 夾爪改為綁定 send_gripper_callback
        self.jog_widget.send_gripper_callback = self.handle_gripper_jog
        
        # 夾爪 Toggle 按鈕連動
        self.jog_widget.gripper_btn.toggled.connect(
            lambda checked: self.jog_widget.g_slider.setValue(100 if checked else 0)
        )
        self.jog_widget.gripper_btn.toggled.connect(
            lambda checked: self.handle_gripper_jog(100 if checked else 0)
        )
        
        self.jog_widget.on_joint_slider_changed()
        
        self.view3d_widget.robot_view.drag_callback = self.handle_tcp_drag
        self.view3d_widget.robot_view.axis_drag_callback = self.handle_cartesian_jog
        self.view3d_widget.robot_view.cancel_gizmo_callback = self.view3d_widget.reset_gizmo_buttons

        self.main_splitter_listener = SplitterDoubleClickListener(main_splitter, default_main_sizes)
        main_splitter.handle(1).installEventFilter(self.main_splitter_listener)
        main_splitter.handle(2).installEventFilter(self.main_splitter_listener)

        self.right_splitter_listener = SplitterDoubleClickListener(right_splitter, default_right_sizes)
        right_splitter.handle(1).installEventFilter(self.right_splitter_listener)

        content_layout.addWidget(main_splitter)
        main_layout.addWidget(content_container)

        # ==========================================
        # 綁定按鈕動作 (與 Waypoint Panel 對接)
        # ==========================================
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
        self.waypoint_panel.update_pt_requested.connect(
            lambda idx: self.path_manager.update_point_at_index(idx, self.current_float_joints)
        )
        self.waypoint_panel.toggle_requested.connect(self.path_manager.toggle_point_active)
        self.waypoint_panel.delete_requested.connect(self.path_manager.delete_point)
        self.waypoint_panel.path_list.currentRowChanged.connect(self.handle_waypoint_preview)
        self.waypoint_panel.data_changed.connect(self.update_path_list_ui)
        self.path_manager.file_loaded_signal.connect(self.waypoint_panel.set_file_name)
        
        self.waypoint_panel.tab_switch_requested.connect(self.handle_tab_switch)
        self.waypoint_panel.tab_closed_signal.connect(self.handle_tab_closed)
        self.waypoint_panel.add_new_tab("untitled.json") 

        app_settings.setting_changed.connect(
            lambda key, val: self.update_path_list_ui() if key == "show_comments" else None
        )

        self.top_bar.btn_play.toggled.connect(self.toggle_execution)
        self.top_bar.btn_stop.clicked.connect(self.stop_execution)
        self.top_bar.btn_soft_home.clicked.connect(self.go_soft_home)
        self.top_bar.btn_connect.clicked.connect(self.toggle_connection)

        self.view3d_widget.monitor_widget.tcp_edit_requested.connect(self.handle_monitor_tcp_edit)
        self.view3d_widget.monitor_widget.joint_edit_requested.connect(self.handle_monitor_joint_edit)

        #全域唯一的空白鍵分發中心
        self.shortcut_space = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        self.shortcut_space.setContext(Qt.ShortcutContext.WindowShortcut)
        self.shortcut_space.activated.connect(self.handle_global_spacebar)

    def handle_global_spacebar(self):
        """將唯一攔截到的空白鍵，派發給各個面板自行判斷滑鼠座標"""
        if hasattr(self, 'waypoint_panel'):
            self.waypoint_panel._handle_spacebar()
        if hasattr(self, 'view3d_widget'):
            self.view3d_widget._handle_spacebar()

    # ==========================================
    # Jogging 硬體通訊核心
    # ==========================================
    def get_jog_speed_factor(self, level):
        """將 UI 的 1~4 檔位轉換為速度比例"""
        mapping = {1: 0.25, 2: 0.5, 3: 0.75, 4: 1.0}
        return mapping.get(level, 1.0)

    def preview_joint_jog(self, angles):
        """處理關節寸動 (拖動中)：只更新 3D 畫面不發送，避免塞爆 MCU"""
        self.current_float_joints = list(angles)
        self.pending_3d_update = True

    def send_joint_jog(self, angles):
        """處理關節寸動 (放開滑鼠/按鍵點擊)：更新 3D 並發送實機訊號"""
        self.current_float_joints = list(angles)
        self.pending_3d_update = True
        
        if hasattr(self, 'serial_manager') and self.serial_manager.is_connected:
            spd_factor = self.get_jog_speed_factor(self.jog_widget.j_speed_level)
            self.serial_manager.send_joints(angles, speed_factor=spd_factor, move_mode=0)

    def handle_continuous_joint_jog(self, axis, direction, speed_factor):
        """通知 MCU 進入 Mode 2：交出控制權，由硬體接管連續旋轉"""
        if hasattr(self, 'serial_manager') and self.serial_manager.is_connected:
            cmd = f"<{axis},{direction},0,0,0,0,{speed_factor:.2f},2>\n"
            self.serial_manager.send_command(cmd)

    def handle_gripper_jog(self, val):
        """處理夾爪作動"""
        if hasattr(self, 'serial_manager') and self.serial_manager.is_connected:
            cmd = f"<EE,{val}>\n"
            self.serial_manager.send_command(cmd)

    def handle_cartesian_jog(self, axis, step_val, frame):
        """處理空間寸動"""
        tcp_mat = self.tcp_manager.get_active_matrix()
        world_mat = np.eye(4)
        if hasattr(self, 'base_manager'):
            world_mat = self.base_manager.get_matrix(self.base_manager.current_index) # 使用當前生效的基座
            
        actual_frame = "Base" if frame == "World" else frame
        new_joints, error_msg = kinematics.calculate_jog_joints(
            self.current_float_joints, axis, step_val, actual_frame, tcp_mat, world_mat
        )
        
        if new_joints is not None:
            self.handle_system_pose_update(new_joints) # 這會同步 UI 上的滑桿
            
            # 將算出來的新角度發送給實機
            if hasattr(self, 'serial_manager') and self.serial_manager.is_connected:
                spd_factor = self.get_jog_speed_factor(self.jog_widget.c_speed_level)
                self.serial_manager.send_joints(new_joints, speed_factor=spd_factor, move_mode=0)
        else:
            self.log_widget.append_log(f"[Jog Warning] {error_msg}")

    # ==========================================
    # 其他核心控制功能
    # ==========================================
    def handle_system_pose_update(self, new_joints):
        self.current_float_joints = list(new_joints)
        self.pending_3d_update = True
        if app_settings.get("sync_sliders"):
            self.jog_widget.update_joints_from_ik(new_joints)

    def handle_monitor_tcp_edit(self, axis_name, new_val):
        """處理 Monitor 區的笛卡爾座標 (XYZRxRyRz) 手動輸入"""
        if self.path_manager.is_running():
            self.log_widget.append_log("[WARNING] Cannot edit position while program is running.")
            self.handle_system_pose_update(self.current_joints) 
            return

        # 1. 取得當下「真實的」 TCP 與 Base 矩陣
        T_tool = self.tcp_manager.get_active_matrix()
        T_base = self.base_manager.get_matrix(self.base_manager.current_index) if hasattr(self, 'base_manager') else np.eye(4)
        
        # 2. 算出相對於當前 Base 的座標
        T_flange = kinematics.forward_kinematics(self.current_float_joints)
        T_tcp_world = T_flange @ T_tool
        T_tcp_base = kinematics.remove_base_frame(T_tcp_world, T_base)
        
        # 將矩陣轉回 XYZRxRyRz 以替換數值
        curr_pos = T_tcp_base[:3, 3] * 1000.0
        curr_rot = kinematics.extract_continuous_rpy(T_tcp_base)
        
        # 3. 將使用者修改的特定軸替換掉
        target_values = list(curr_pos) + list(curr_rot)
        axis_map = {"X": 0, "Y": 1, "Z": 2, "Rx": 3, "Ry": 4, "Rz": 5}
        idx = axis_map.get(axis_name)
        if idx is not None:
            target_values[idx] = new_val

        # 4. 重組為目標矩陣，轉回 World 座標系後，呼叫 IK
        T_tcp_base_target = kinematics.get_tf_matrix(
            [x / 1000.0 for x in target_values[:3]], 
            np.deg2rad(target_values[3:])
        )
        T_tcp_world_target = kinematics.apply_base_frame(T_tcp_base_target, T_base)
        T_flange_target = T_tcp_world_target @ np.linalg.inv(T_tool)
        
        new_joints, error = kinematics.inverse_kinematics(T_flange_target, self.current_float_joints)
        
        # 5. 判定與退回機制
        if new_joints is not None:
            self.handle_system_pose_update(list(new_joints))
        else:
            self.log_widget.append_log(f"[ERROR] Monitor Edit Failed: Target TCP {axis_name}={new_val} is out of reach or singular.")
            self.handle_system_pose_update(self.current_float_joints)

    def handle_monitor_joint_edit(self, joint_idx, new_val):
        """處理 Monitor 區的關節角度手動輸入"""
        if self.path_manager.is_running():
            self.log_widget.append_log("[WARNING] Cannot edit joints while program is running.")
            self.handle_system_pose_update(self.current_float_joints)
            return

        import config
        min_lim, max_lim = config.JOINT_LIMITS[joint_idx]
        if new_val < min_lim or new_val > max_lim:
            self.log_widget.append_log(f"[ERROR] Monitor Edit Failed: J{joint_idx+1} limit is [{min_lim}, {max_lim}].")
            self.handle_system_pose_update(self.current_float_joints) 
            return

        new_joints = list(self.current_float_joints)
        new_joints[joint_idx] = new_val
        self.handle_system_pose_update(new_joints)

    def update_path_list_ui(self):
        """只在新增/刪除點位時，才重繪左側 UI 清單 (高耗能)"""
        if self.path_manager.is_running():
            return
            
        self.waypoint_panel.update_list(self.path_manager.waypoints)
        self.update_3d_trajectory_preview()

    def update_3d_trajectory_preview(self):
        """只負責更新 3D 畫面中的綠色軌跡線 (極低耗能，純 GPU/Numpy 運算)"""
        if self.path_manager.is_running():
            return
            
        try:
            pts = self.path_manager.get_trajectory_preview(self.tcp_manager.get_active_matrix())
            if hasattr(self.view3d_widget.robot_view, 'draw_trajectory_preview'):
                self.view3d_widget.robot_view.draw_trajectory_preview(pts)
        except Exception:
            pass

    def handle_tab_switch(self, new_tab):
        if self.waypoint_panel.active_tab:
            self.waypoint_panel.active_tab.waypoints_data = [dict(wp) for wp in self.path_manager.waypoints]
        self.path_manager.waypoints = [dict(wp) for wp in new_tab.waypoints_data]
        self.waypoint_panel.set_active_tab_visuals(new_tab)
        self.update_path_list_ui()

    def execute_save_process(self):
        """統包存檔流程：先執行全域 Base 同步，再正式寫入硬碟"""
        if hasattr(self, 'base_manager') and hasattr(self, 'path_manager'):
            self.path_manager.sync_all_base_shifts(self.base_manager)
            
        self.path_manager.save_to_file()

    def update_tcp_point_action(self, index, tool_idx):
        tool_data = self.tcp_manager.tools[tool_idx]
        tool_name = tool_data.get("name", "Unknown Tool")
        
        self.path_manager.update_special_point(index, "SET_TCP", tool_idx, f"Tool: {tool_name}")

    def record_waypoint_action(self, index, pt_type="PTP"):
        """處理來自清單的點位錄製請求 (PTP, LIN, CIRC)"""
        current_j = [round(j, 4) for j in self.current_float_joints]
        T_flange_world = kinematics.forward_kinematics(current_j)
        recorded_base_mat = np.eye(4)
        if hasattr(self, 'base_manager'):
            recorded_base_mat = self.base_manager.get_active_matrix()

        aux_j = None
        if pt_type == "CIRC":
            if hasattr(self.path_manager, 'temp_aux_joints') and self.path_manager.temp_aux_joints:
                aux_j = [round(j, 4) for j in self.path_manager.temp_aux_joints]
                self.path_manager.temp_aux_joints = None 
            else:
                if hasattr(self, 'log_widget'):
                    self.log_widget.append_log("[ERROR] 無法插入 CIRC：請先移動到中繼點並按下 'Record AUX'！")
                return

        new_wp = {
            "type": pt_type, 
            "name": f"Point", 
            "joints": copy.deepcopy(current_j), 
            "aux_joints": aux_j,
            "cartesian_flange": np.round(T_flange_world, 4).tolist(),       
            "recorded_base_matrix": np.round(recorded_base_mat, 4).tolist(),
            "speed": 50.0,
            "accel": 50.0,
            "blend": "FINE",
            "active": True,
            "note": ""
        }
        
        self.path_manager.insert_waypoint(index, new_wp)
        
        if hasattr(self, 'waypoint_panel') and hasattr(self.waypoint_panel, 'path_list'):
            self.waypoint_panel.path_list.setCurrentRow(index)

    def insert_special_point_action(self, index, pt_type):
        """處理來自清單的特殊點位插入請求 (SET_TCP, SET_BASE, DELAY, IO)"""
        if pt_type.startswith("SET_TCP"):
            if ":" in pt_type:
                tool_idx = int(pt_type.split(":")[1])
            else:
                tool_idx = self.tcp_manager.current_index
                
            tool_data = self.tcp_manager.tools[tool_idx]
            tool_name = tool_data.get("name", "Unknown Tool")
            
            new_wp = {
                "type": "SET_TCP",
                "value": tool_idx,
                "name": f"Tool: {tool_name}",
                "active": True
            }

        elif pt_type.startswith("SET_BASE"):
            if ":" in pt_type:
                base_idx = int(pt_type.split(":")[1])
            else:
                base_idx = self.base_manager.current_index
                
            base_data = self.base_manager.bases[base_idx]
            base_name = base_data.get("name", "Unknown Base")
            
            new_wp = {
                "type": "SET_BASE",
                "value": base_idx,
                "name": f"Base: {base_name}",
                "active": True
            }
            
        elif pt_type == "DELAY":
            new_wp = {
                "type": "DELAY",
                "value": 1.0, 
                "active": True
            }
            
        elif pt_type == "IO":
            val = 0
            if hasattr(self, 'jog_widget') and hasattr(self.jog_widget, 'g_slider'):
                val = self.jog_widget.g_slider.value()
                
            new_wp = {
                "type": "I/O",
                "action_type": "SERVO",
                "value": val,
                "note": f"Grip {val}%",
                "active": True
            }
        else:
            return
            
        self.path_manager.insert_waypoint(index, new_wp)
        if hasattr(self.waypoint_panel, 'select_row_silently'):
            self.waypoint_panel.select_row_silently(index)
        else:
            self.waypoint_panel.path_list.setCurrentRow(index)
        self.update_path_list_ui()

    def open_tcp_manager(self):
        if self.path_manager.is_running():
            self.log_widget.append_log("[WARNING] Cannot change TCP config while program is running.")
            return
            
        dialog = TCPManagerDialog(self.tcp_manager, self)
        if dialog.exec():
            self.update_path_list_ui()
            self.handle_system_pose_update(self.current_float_joints) 
            active_tool = self.tcp_manager.get_active_tool_data()
            tool_name = active_tool.get("name", "Unknown Tool")
            self.log_widget.append_log(f"[System] UPDATED: Tool '{tool_name}' successfully applied.")

    def open_base_manager(self):
        if self.path_manager.is_running():
            self.log_widget.append_log("[WARNING] Cannot change Base config while program is running.")
            return
            
        dialog = BaseManagerDialog(self.base_manager, self)
        if dialog.exec():
            active_base = self.base_manager.get_active_base_data()
            base_name = active_base.get("name", "World")
            self.log_widget.append_log(f"[System] UPDATED: Base '{base_name}' settings applied.")

    def handle_batch_base_shift(self, indices):
        """去 BaseManager 拿矩陣，然後交給 PathManager 算數學"""
        current_idx = self.base_manager.current_index
        target_mat = self.base_manager.get_matrix(current_idx)
        target_name = self.base_manager.bases[current_idx]['name']
        self.path_manager.apply_batch_base_shift(indices, target_mat, target_name)
        
    def handle_block_base_shift(self, set_base_idx):
        """找出該點位對應的 Base，拿矩陣交給 PathManager 算數學"""
        bid = self.path_manager.waypoints[set_base_idx].get('value', 0)
        target_mat = self.base_manager.get_matrix(bid)
        target_name = self.base_manager.bases[bid]['name']
        self.path_manager.apply_base_shift_block(set_base_idx, target_mat, target_name)

    def _update_monitor_ui(self):
        """共用函式：更新儀表板顯示的 TCP 數值與關節角"""
        tcp_mat = self.tcp_manager.get_active_matrix()
        T_flange = kinematics.forward_kinematics(self.current_float_joints)
        T_tcp = T_flange @ tcp_mat
        
        world_mat = self.base_manager.get_matrix(self.base_manager.current_index) if hasattr(self, 'base_manager') else np.eye(4)
        T_tcp_base = kinematics.remove_base_frame(T_tcp, world_mat)
        
        x, y, z = T_tcp_base[:3, 3] * 1000.0
        self.prev_rpy = kinematics.extract_continuous_rpy(T_tcp_base, self.prev_rpy)
        rx, ry, rz = self.prev_rpy
        
        self.view3d_widget.monitor_widget.update_tcp(x, y, z, rx, ry, rz)
        self.view3d_widget.monitor_widget.update_joints(*self.current_float_joints)
        self._ui_throttle_counter = 0

    def process_3d_update(self):
        if self.pending_3d_update:
            tcp_mat = self.tcp_manager.get_active_matrix()
            self.view3d_widget.robot_view.update_joints(self.current_float_joints, tcp_mat)
            self.pending_3d_update = False
            
            self._ui_throttle_counter += 1
            if self._ui_throttle_counter >= 3: 
                self._update_monitor_ui() # 呼叫共用函式
        else:
            if self._ui_throttle_counter > 0:
                self._update_monitor_ui() # 呼叫共用函式

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
                        
            # 改為印出安全提示，提醒操作者手動對齊姿態
            if hasattr(self, 'log_widget'):
                self.log_widget.append_log("[HW] 已連線：實機保持靜默。請執行原點復歸Homing")
        else:
            self.top_bar.btn_connect.setIcon(qta.icon('mdi.connection', color='#e0e0e0'))
            self.top_bar.btn_connect.setToolTip("Connect to Serial Port")

    def _play_pose_animation(self, target_joints, wp_type='PTP'):
        """處理 3D 畫面的平滑過渡動畫"""
        from PySide6.QtCore import QVariantAnimation, QEasingCurve
        import numpy as np

        target_j_array = np.array(target_joints)
        start_j_array = np.array(self.current_float_joints)
        
        # 1. 如果已經在目標點附近，直接更新並結束，節省資源
        if np.allclose(start_j_array, target_j_array, atol=1e-2):
            self.handle_system_pose_update(target_joints)
            return

        # 2. 中斷可能正在播放的上一段動畫
        if hasattr(self, 'preview_animation') and self.preview_animation.state() == QVariantAnimation.State.Running:
            self.preview_animation.stop()

        # 3. LIN 直線模式專屬：預先烘焙 20 個空間關鍵影格
        preview_path = []
        if wp_type == 'LIN':
            import kinematics
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

        # 4. 建立與設定動畫實體 (統一 450 毫秒)
        self.preview_animation = QVariantAnimation(self)
        self.preview_animation.setDuration(450)
        self.preview_animation.setStartValue(0.0)
        self.preview_animation.setEndValue(1.0)
        self.preview_animation.setEasingCurve(QEasingCurve.Type.InOutQuad)

        # 5. 定義影格更新回呼函式
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

    def go_soft_home(self):
        """觸發 Soft Home，發送實體指令並以平滑動畫更新 3D 畫面"""
        
        # 【防護】：如果路徑正在執行，禁止歸零干擾
        if hasattr(self, 'path_manager') and self.path_manager.is_running():
            return

        zero_joints = [0.0] * 6
        
        # 1. 發送給實體硬體
        if hasattr(self, 'serial_manager') and self.serial_manager.is_connected:
            self.serial_manager.send_joints(zero_joints)

        # 2. 呼叫共用動畫引擎 (Home 使用預設的 PTP 模式即可)
        self._play_pose_animation(zero_joints, wp_type='PTP')

    def handle_set_tcp_playback(self, tool_idx):
        """處理腳本播放時的動態換刀請求"""
        self.tcp_manager.set_current_index(tool_idx)
        self.handle_system_pose_update(self.current_float_joints)

    def handle_set_base_playback(self, base_idx):
        """處理腳本播放時的動態基座切換請求"""
        if hasattr(self, 'base_manager'):
            self.base_manager.set_current_index(base_idx)

    def handle_waypoint_preview(self, index):
        """當使用者點擊清單行數時，啟動共用動畫引擎平滑過渡至 3D 姿態預覽"""
        
        # 【防護 1】路徑執行中，禁止預覽干擾
        if self.path_manager.is_running():
            return

        # 【防護 2】確保索引有效
        if index < 0 or index >= len(self.path_manager.waypoints):
            return

        target_wp = self.path_manager.waypoints[index]
        wp_type = target_wp.get('type', '')

        # 1. 系統性雙向尋找邏輯 (尋找實體關節角度)
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

        if reference_wp is None or reference_wp.get('joints') is None:
            return
            
        preview_joints = list(reference_wp['joints'])

        # 2. 往前追溯當下應該生效的 TCP 與 Base
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
        
        # 3. 自動切換大腦狀態以符合歷史軌跡
        if self.tcp_manager.current_index != target_tool_idx:
            # 加上阻斷器：靜默切換，避免觸發全域 data_changed 導致軌跡重算
            self.tcp_manager.blockSignals(True)
            self.tcp_manager.set_current_index(target_tool_idx)
            self.tcp_manager.blockSignals(False)
            
        if hasattr(self, 'base_manager') and self.base_manager.current_index != target_base_idx:
            # 加上阻斷器：靜默切換
            self.base_manager.blockSignals(True)
            self.base_manager.set_current_index(target_base_idx)
            self.base_manager.blockSignals(False)

        # 4. 動態預覽基座偏移
        if hasattr(self, 'base_manager'):
            current_base_mat = self.base_manager.get_matrix(target_base_idx)
            import numpy as np
            recorded_base_mat = np.array(reference_wp.get('recorded_base_matrix', np.eye(4)))
            
            if not np.allclose(current_base_mat, recorded_base_mat, atol=1e-4):
                import kinematics
                T_flange_world_old = np.array(reference_wp.get('cartesian_flange', kinematics.forward_kinematics(preview_joints)))
                new_joints, _ = kinematics.calculate_base_shift_ik(
                    T_flange_world_old, recorded_base_mat, current_base_mat, preview_joints
                )
                if new_joints is not None:
                    preview_joints = list(new_joints)

        # 5.呼叫動畫引擎！
        self._play_pose_animation(preview_joints, wp_type=wp_type)

    def toggle_execution(self, checked):
        if checked:
            valid_types = ["PTP", "LIN", "CIRC", "DELAY", "GRIPPER", "I/O", "LOOP_START", "LOOP_END", "SET_TCP", "SET_BASE", "CAM_PATH"]
            
            active_points = [
                pt for pt in self.path_manager.waypoints 
                if pt.get('active', True) and pt.get('type') in valid_types
            ]
            
            if len(active_points) == 0:
                self.log_widget.append_log("[System] 警告: 沒有可執行的點位。")
                self._reset_play_ui() # 呼叫共用函式
                return

            self.log_widget.append_log(">>> 開始執行路徑串流...")
            # 鎖定 Monitor：執行期間全面禁止編輯！
            self.view3d_widget.monitor_widget.set_locked(True)
            
            tcp_mat = self.tcp_manager.get_active_matrix()
            callbacks = {
                'update': self.handle_system_pose_update, 
                'error': lambda msg: self.log_widget.append_log(f"[ERROR] {msg}"),
                'log': self.log_widget.append_log,
                'finished': self._on_execution_finished,
                'set_tcp': self.handle_set_tcp_playback,
                'set_base': self.handle_set_base_playback 
            }
            self.path_manager.execute_streaming_path(
                active_points=active_points,
                start_joints=self.current_float_joints,
                tcp_offset_mat=tcp_mat,
                loop=False, 
                global_speed=50.0,
                global_accel=50.0,
                serial_ref=None, 
                callbacks=callbacks,
            )
        else:
            if self.path_manager.is_running():
                self.path_manager.stop_path()
                self.log_widget.append_log(">>> 執行暫停")
            self.view3d_widget.monitor_widget.set_locked(False)

    def _reset_play_ui(self):
        """共用函式：重置播放按鈕狀態並解鎖 UI"""
        self.top_bar.btn_play.blockSignals(True)
        self.top_bar.btn_play.setChecked(False)
        self.top_bar.btn_play.setIcon(qta.icon('mdi.motion-play-outline', color='#00e6b8'))
        self.top_bar.btn_play.blockSignals(False)
        if hasattr(self, 'view3d_widget') and hasattr(self.view3d_widget, 'monitor_widget'):
            self.view3d_widget.monitor_widget.set_locked(False)

    def stop_execution(self):
        if self.path_manager.is_running():
            self.path_manager.stop_path()
            self.log_widget.append_log(">>> 執行終止")
            
        self._reset_play_ui() # 呼叫共用函式

    def _on_execution_finished(self, total_time):
        self.log_widget.append_log(f">>> 執行完成！總耗時 {total_time:.2f} 秒")
        self._reset_play_ui() # 呼叫共用函式

    def showEvent(self, event):
        super().showEvent(event)
        apply_windows_dark_titlebar(self)

    def _prompt_unsaved_changes(self, text_message):
        """共用函式：防呆警告彈窗，回傳使用者的選擇與按鈕物件"""
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
        """處理分頁關閉邏輯，決定是否放行刪除 UI"""
        if closed_tab == self.waypoint_panel.active_tab:
            
            # 1. 攔截：詢問 PathManager 是否有未儲存的變更
            if hasattr(self.path_manager, 'is_modified') and self.path_manager.is_modified:
                choice, btn_save, btn_discard, btn_cancel = self._prompt_unsaved_changes("目前有未儲存的點位變更，請問要儲存檔案嗎？")

                if choice == btn_save:
                    self.execute_save_process()
                    if self.path_manager.is_modified:
                        return           
                elif choice == btn_discard:
                    pass   
                elif choice == btn_cancel:
                    return 

            self.path_manager.waypoints = []
            self.path_manager.is_modified = False  
            self.update_path_list_ui()
            
        self.waypoint_panel.force_close_tab(closed_tab)

    def closeEvent(self, event):
        """攔截主視窗關閉事件"""
        if hasattr(self, 'path_manager') and getattr(self.path_manager, 'is_modified', False):
            choice, btn_save, btn_discard, btn_cancel = self._prompt_unsaved_changes("退出前，是否要儲存目前的點位變更？")

            if choice == btn_save:
                # 儲存 > 儲存後關閉主視窗
                self.execute_save_process() 
                if self.path_manager.is_modified:
                    event.ignore() # 存檔被取消，不關閉視窗
                else:
                    event.accept() # 存檔成功，放行關閉 
            elif choice == btn_discard:
                event.accept() 
            elif choice == btn_cancel:
                event.ignore() 
                
        else:
            event.accept()