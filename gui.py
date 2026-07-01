import numpy as np
from PySide6.QtWidgets import QMainWindow, QMenu, QWidget, QVBoxLayout, QSplitter, QApplication
from PySide6.QtCore import Qt, QTimer
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
        self.serial_manager = SerialManager()
        self.serial_manager.log_signal.connect(self.log_widget.append_log)
        self.serial_manager.connection_state_signal.connect(self.update_connection_ui)
        
        # 將 Serial Manager 交給 Path Manager (大腦)
        self.path_manager.serial_manager = self.serial_manager

        self.top_bar.btn_tools.clicked.connect(self.open_tcp_manager)
        self.pending_3d_update = False
        self._ui_throttle_counter = 0 
        
        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.process_3d_update)
        self.render_timer.start(16) 

        self.jog_widget.update_3d_callback = self.handle_3d_update
        self.jog_widget.cartesian_jog_callback = self.handle_cartesian_jog
        
        self.view3d_widget.robot_view.drag_callback = self.handle_tcp_drag
        self.view3d_widget.robot_view.axis_drag_callback = self.handle_cartesian_jog
        self.view3d_widget.robot_view.cancel_gizmo_callback = self.view3d_widget.reset_gizmo_buttons
        
        self.jog_widget.on_joint_slider_changed()

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
        # 👇 徹底將錄製權力交接給 Waypoint Panel
        self.waypoint_panel.record_pt_requested.connect(self.record_waypoint_action)
        self.waypoint_panel.update_tcp_point_requested.connect(self.update_tcp_point_action)
        self.waypoint_panel.insert_special_requested.connect(self.insert_special_point_action)

        self.waypoint_panel.btn_save.clicked.connect(self.path_manager.save_to_file)
        self.waypoint_panel.btn_load.clicked.connect(self.path_manager.load_from_file)

        self.waypoint_panel.clear_all_requested.connect(self.path_manager.delete_all_points)
        self.waypoint_panel.update_pt_requested.connect(
            lambda idx: self.path_manager.update_point_at_index(idx, self.current_float_joints)
        )
        self.waypoint_panel.insert_pt_requested.connect(self.insert_point_action)
        self.waypoint_panel.toggle_requested.connect(self.path_manager.toggle_point_active)
        self.waypoint_panel.delete_requested.connect(self.path_manager.delete_point)
        self.waypoint_panel.path_list.currentRowChanged.connect(self.handle_waypoint_preview)
        self.waypoint_panel.data_changed.connect(self.update_path_list_ui)
        self.path_manager.file_loaded_signal.connect(self.waypoint_panel.set_file_name)
        
        self.waypoint_panel.tab_switch_requested.connect(self.handle_tab_switch)
        self.waypoint_panel.tab_closed_signal.connect(self.handle_tab_closed)
        
        # 確保神經都接好後，我們才呼叫產生第一個分頁
        self.waypoint_panel.add_new_tab("untitled.json") 

        app_settings.setting_changed.connect(
            lambda key, val: self.update_path_list_ui() if key == "show_comments" else None
        )

        self.top_bar.btn_play.toggled.connect(self.toggle_execution)
        self.top_bar.btn_stop.clicked.connect(self.stop_execution)
        self.top_bar.btn_soft_home.clicked.connect(self.go_soft_home)
        self.top_bar.btn_connect.clicked.connect(self.toggle_connection)

    # ==========================================
    # 核心邏輯函式
    # ==========================================
    def update_path_list_ui(self):
        """刷新清單與 3D 預覽線"""
        self.waypoint_panel.update_list(self.path_manager.waypoints)
        try:
            pts = self.path_manager.get_trajectory_preview(self.tcp_manager.get_active_matrix())
            if hasattr(self.view3d_widget.robot_view, 'draw_trajectory_preview'):
                self.view3d_widget.robot_view.draw_trajectory_preview(pts)
        except Exception:
            pass

    def handle_tab_switch(self, new_tab):
        """將舊分頁資料備份，並把新分頁資料讀入 PathManager 大腦"""
        # 1. 備份當前大腦資料到舊分頁
        if self.waypoint_panel.active_tab:
            # 拷貝一份存進舊 Tab，避免被污染
            self.waypoint_panel.active_tab.waypoints_data = [dict(wp) for wp in self.path_manager.waypoints]
            
        # 2. 載入新分頁資料到大腦
        self.path_manager.waypoints = [dict(wp) for wp in new_tab.waypoints_data]
        
        # 3. 切換 UI 狀態與刷新畫面
        self.waypoint_panel.set_active_tab_visuals(new_tab)
        self.update_path_list_ui()

    def handle_tab_closed(self, closed_tab):
        """如果關閉的是當前作用中的分頁，將大腦清空，等待跳到下一個分頁"""
        if closed_tab == self.waypoint_panel.active_tab:
            self.path_manager.waypoints = []
            self.update_path_list_ui()

    def insert_point_action(self, index):
        """一般插入：往前尋找最近的實體座標，建立一個全新的 PTP 點位插在前面"""
        nearest_joints = None
        
        for i in range(index, -1, -1):
            wp = self.path_manager.waypoints[i]
            if 'joints' in wp and wp['joints'] is not None:
                nearest_joints = wp['joints']
                break
                
        if nearest_joints is None:
            for i in range(index + 1, len(self.path_manager.waypoints)):
                wp = self.path_manager.waypoints[i]
                if 'joints' in wp and wp['joints'] is not None:
                    nearest_joints = wp['joints']
                    break
                    
        if nearest_joints is None:
            nearest_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        import copy
        new_wp = {
            "type": "PTP",
            "name": f"Point",
            "joints": copy.deepcopy(nearest_joints), 
            "speed": 50,
            "accel": 50,
            "blend": "FINE",
            "active": True
        }
        
        self.path_manager.insert_waypoint(index, new_wp)
        self.waypoint_panel.path_list.setCurrentRow(index)

    def update_tcp_point_action(self, index, tool_idx):
        """更新指定行的 SET_TCP 點位為工具箱內的新刀具"""
        
        # 1. 根據傳過來的 tool_idx 直接去倉庫拿資料
        tool_data = self.tcp_manager.tools[tool_idx]
        tool_name = tool_data.get("name", "Unknown Tool")
        
        # 2. 寫入新的工具資料到該行路徑點
        if 0 <= index < len(self.path_manager.waypoints):
            self.path_manager.waypoints[index]["value"] = tool_idx
            self.path_manager.waypoints[index]["name"] = f"Tool: {tool_name}"
            
            # 3. 廣播 UI 重繪與 Log
            self.path_manager.list_update_signal.emit()
            self.path_manager.log_signal.emit(f">>> UPDATED: [SET_TCP] at line {index + 1} to {tool_name}")

    # 👇 增加 pt_type 參數來接收選單傳來的 "PTP", "LIN", "CIRC"
    def record_waypoint_action(self, index, pt_type="PTP"):
        """錄製目前機器的真實/模擬座標，並以指定的運動模式插入"""
        import copy
        current_j = [round(j, 4) for j in self.current_float_joints]
        
        new_wp = {
            "type": pt_type,       # 👈 動態套用傳進來的運動模式
            "name": f"Point",
            "joints": copy.deepcopy(current_j), 
            "speed": 50,
            "accel": 50,
            "blend": "FINE",
            "active": True
        }
        
        # 寫入大腦並自動將焦點移過去
        self.path_manager.insert_waypoint(index, new_wp)
        self.waypoint_panel.path_list.setCurrentRow(index)

    def insert_special_point_action(self, index, pt_type):
        """處理來自清單的特殊點位插入請求 (單純版)"""
        
        if pt_type == "SET_TCP":
            # 💡 直接抓取目前正在使用的刀具 (與三點功能表完全相同的邏輯)
            tool_idx = self.tcp_manager.current_index
            tool_data = self.tcp_manager.tools[tool_idx]
            tool_name = tool_data.get("name", "Unknown Tool")
            
            new_wp = {
                "type": "SET_TCP",
                "value": tool_idx,
                "name": f"Tool: {tool_name}",
                "active": True
            }
            
        elif pt_type == "DELAY":
            new_wp = {
                "type": "DELAY",
                "value": 1.0, 
                "active": True
            }
            
        else:
            return
            
        # 寫入大腦並自動將焦點移過去 (這會觸發 UI 清單重繪)
        self.path_manager.insert_waypoint(index, new_wp)
        self.waypoint_panel.path_list.setCurrentRow(index)
        
    def handle_3d_update(self, angles):
        self.current_float_joints = list(angles)
        self.pending_3d_update = True

    def handle_system_pose_update(self, new_joints):
        self.current_float_joints = list(new_joints)
        self.pending_3d_update = True
        
        if app_settings.get("sync_sliders"):
            self.jog_widget.update_joints_from_ik(new_joints)

    def open_tcp_manager(self):
        dialog = TCPManagerDialog(self.tcp_manager, self)
        if dialog.exec():
            self.update_path_list_ui()
            self.handle_system_pose_update(self.current_float_joints) 
            active_tool = self.tcp_manager.get_active_tool_data()
            tool_name = active_tool.get("name", "Unknown Tool")
            
            self.log_widget.append_log(f"[System] UPDATED: Tool '{tool_name}' successfully applied.")

    def process_3d_update(self):
        if self.pending_3d_update:
            tcp_mat = self.tcp_manager.get_active_matrix()
            self.view3d_widget.robot_view.update_joints(self.current_float_joints, tcp_mat)
            
            # 狀態先重置，代表這幀的 3D 已經處理完
            self.pending_3d_update = False
            
            self._ui_throttle_counter += 1
            if self._ui_throttle_counter >= 3: 
                T_flange = kinematics.forward_kinematics(self.current_float_joints)
                T_tcp = T_flange @ tcp_mat
                x, y, z = T_tcp[:3, 3] * 1000.0
                
                self.prev_rpy = kinematics.extract_continuous_rpy(T_tcp, self.prev_rpy)
                rx, ry, rz = self.prev_rpy
                
                self.view3d_widget.monitor_widget.update_tcp(x, y, z, rx, ry, rz)
                self.view3d_widget.monitor_widget.update_joints(*self.current_float_joints)
                self._ui_throttle_counter = 0
                
        else:
            # 【關鍵修復】當手臂停下時，若還有沒發布的 UI 尾數，強制發布最終精準座標！
            if self._ui_throttle_counter > 0:
                tcp_mat = self.tcp_manager.get_active_matrix()
                T_flange = kinematics.forward_kinematics(self.current_float_joints)
                T_tcp = T_flange @ tcp_mat
                x, y, z = T_tcp[:3, 3] * 1000.0
                
                self.prev_rpy = kinematics.extract_continuous_rpy(T_tcp, self.prev_rpy)
                rx, ry, rz = self.prev_rpy
                
                self.view3d_widget.monitor_widget.update_tcp(x, y, z, rx, ry, rz)
                self.view3d_widget.monitor_widget.update_joints(*self.current_float_joints)
                self._ui_throttle_counter = 0

    def handle_cartesian_jog(self, axis, step_val, frame):
        tcp_mat = self.tcp_manager.get_active_matrix()
        new_joints, error_msg = kinematics.calculate_jog_joints(
            self.current_float_joints, axis, step_val, frame, tcp_mat
        )
        if new_joints is not None:
            self.handle_system_pose_update(new_joints)

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

    # ==========================================
    # 硬體連線管理 (Serial Connection)
    # ==========================================
    def toggle_connection(self):
        """處理頂部連線按鈕的點擊事件 (彈出 COM Port 選單)"""
        if self.serial_manager.is_connected:
            self.serial_manager.disconnect()
        else:
            # 取得所有可用的 COM Ports
            ports = self.serial_manager.list_ports()
            if not ports:
                self.log_widget.append_log("[System] 找不到任何可用的 COM Port 裝置。")
                return

            # 動態生成優雅的深色下拉選單
            menu = QMenu(self)
            menu.setStyleSheet(styles.MENU_STYLE)
            
            for port in ports:
                action = menu.addAction(f"Connect to {port}")
                # 綁定點擊事件，傳入選擇的 port
                action.triggered.connect(lambda checked, p=port: self.serial_manager.connect(p))
                
            # 將選單顯示在連線按鈕的正下方
            btn = self.top_bar.btn_connect
            menu.exec(btn.mapToGlobal(btn.rect().bottomLeft()))

    def update_connection_ui(self, is_connected):
        """根據連線狀態，自動更新按鈕顏色與 Tooltip"""
        if is_connected:
            self.top_bar.btn_connect.setIcon(qta.icon('mdi.connection', color='#00e6b8')) # 科技綠
            self.top_bar.btn_connect.setToolTip("Disconnect")
            # 連線成功時，順便把當前 GUI 上的角度同步給硬體
            self.serial_manager.send_joints(self.current_float_joints)
        else:
            self.top_bar.btn_connect.setIcon(qta.icon('mdi.connection', color='#e0e0e0')) # 恢復灰色
            self.top_bar.btn_connect.setToolTip("Connect to Serial Port")

    def go_soft_home(self):
        """軟歸零：將所有的軸回歸到 0 度狀態"""
        zero_joints = [0.0] * 6
        
        # 1. 更新 UI 與 3D 畫面
        self.handle_system_pose_update(zero_joints)
        
        # 2. 如果硬體已連線，立刻將 0 度的指令推送到實體機器人上
        if hasattr(self, 'serial_manager') and self.serial_manager.is_connected:
            self.serial_manager.send_joints(zero_joints)
            
        # 3. 在 Log 中留下精美的藍色系統提示
        self.log_widget.append_log("[System] Soft Home: All axes have been reset to 0.")

    # ==========================================
    # 串流播放控制核心 (Playback Execution)
    # ==========================================
    def toggle_execution(self, checked):
        if checked:
            valid_types = ["PTP", "LIN", "CIRC", "DELAY", "GRIPPER", "I/O", "LOOP_START", "LOOP_END", "SET_TCP"]
            active_points = [
                pt for pt in self.path_manager.waypoints 
                if pt.get('active', True) and pt.get('type') in valid_types
            ]
            
            if len(active_points) == 0:
                self.log_widget.append_log("[System] 警告: 沒有可執行的點位。")
                self.top_bar.btn_play.blockSignals(True)
                self.top_bar.btn_play.setChecked(False)
                self.top_bar.btn_play.setIcon(qta.icon('mdi.motion-play-outline', color='#00e6b8'))
                self.top_bar.btn_play.blockSignals(False)
                return

            self.log_widget.append_log(">>> 開始執行路徑串流...")
            
            tcp_mat = self.tcp_manager.get_active_matrix()
            
            callbacks = {
                'update': self.handle_system_pose_update, 
                'error': lambda msg: self.log_widget.append_log(f"[ERROR] {msg}"),
                'log': self.log_widget.append_log,
                'finished': self._on_execution_finished,
                'set_tcp': self.handle_set_tcp_playback
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

    def handle_set_tcp_playback(self, tool_idx):
        """處理腳本播放時的動態換刀請求"""
        self.tcp_manager.set_current_index(tool_idx)
        self.handle_system_pose_update(self.current_float_joints)

    def insert_special_point_action(self, index, pt_type):
        """處理來自清單的特殊點位插入請求 (SET_TCP, DELAY, IO)"""
        
        if pt_type.startswith("SET_TCP"):
            # 💡 兼容機制：判斷字串裡有沒有夾帶 ":"
            if ":" in pt_type:
                # 來自底部墊片選單的抽屜 (帶有選擇的刀具編號)
                tool_idx = int(pt_type.split(":")[1])
            else:
                # 來自單行點位 Row Widget 的三點選單 (未帶編號，直接使用當前刀具)
                tool_idx = self.tcp_manager.current_index
                
            tool_data = self.tcp_manager.tools[tool_idx]
            tool_name = tool_data.get("name", "Unknown Tool")
            
            new_wp = {
                "type": "SET_TCP",
                "value": tool_idx,
                "name": f"Tool: {tool_name}",
                "active": True
            }
            
        elif pt_type == "DELAY":
            new_wp = {
                "type": "DELAY",
                "value": 1.0, 
                "active": True
            }
            
        elif pt_type == "IO":
            # 🤖 防呆機制：確保能正確讀取到夾爪數值
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
            
        # 寫入大腦並自動將焦點移過去 (觸發 UI 重繪)
        self.path_manager.insert_waypoint(index, new_wp)
        self.waypoint_panel.path_list.setCurrentRow(index)

    def stop_execution(self):
        if self.path_manager.is_running():
            self.path_manager.stop_path()
            self.log_widget.append_log(">>> 執行終止")
            
        self.top_bar.btn_play.blockSignals(True)
        self.top_bar.btn_play.setChecked(False)
        self.top_bar.btn_play.setIcon(qta.icon('mdi.motion-play-outline', color='#00e6b8'))
        self.top_bar.btn_play.blockSignals(False)

    def _on_execution_finished(self, total_time):
        self.log_widget.append_log(f">>> 執行完成！總耗時 {total_time:.2f} 秒")
        self.top_bar.btn_play.blockSignals(True)
        self.top_bar.btn_play.setChecked(False)
        self.top_bar.btn_play.setIcon(qta.icon('mdi.motion-play-outline', color='#00e6b8'))
        self.top_bar.btn_play.blockSignals(False)

    def handle_waypoint_preview(self, index):
        if 0 <= index < len(self.path_manager.waypoints):
            
            # 1. 啟動時光倒流：從你點擊的點位往前找，確認「當時」是拿哪一把刀
            target_tool_idx = 0  # 預設為 Tool 0
            for i in range(index, -1, -1):
                prev_wp = self.path_manager.waypoints[i]
                if prev_wp.get('type') == 'SET_TCP':
                    target_tool_idx = int(prev_wp.get('value', 0))
                    break
            
            # 2. 如果發現當時的刀具跟現在不一樣，就幫使用者「自動換刀」
            if self.tcp_manager.current_index != target_tool_idx:
                self.tcp_manager.set_current_index(target_tool_idx)
                
                # 印出貼心提示，讓你知道系統幫你自動切換了
                active_tool = self.tcp_manager.get_active_tool_data()
                self.log_widget.append_log(f"[System] Preview: Auto-switched to Tool '{active_tool.get('name')}'.")
                
            # 3. 正常更新手臂關節 (這會觸發 3D 畫布去抓取剛才切換好的新刀具矩陣，箭頭就會跳位了！)
            wp = self.path_manager.waypoints[index]
            
            # 判斷當前點位是否有關節數據 (PTP/LIN/CIRC 才有)
            if wp.get('joints') is not None:
                self.handle_system_pose_update(wp['joints'])
            else:
                # 如果沒有 joints (例如 DELAY 或 SET_TCP)，往前搜尋最近的一個有效位置
                for i in range(index - 1, -1, -1):
                    prev_wp = self.path_manager.waypoints[i]
                    if prev_wp.get('joints') is not None:
                        self.handle_system_pose_update(prev_wp['joints'])
                        break

    def showEvent(self, event):
        super().showEvent(event)
        apply_windows_dark_titlebar(self)