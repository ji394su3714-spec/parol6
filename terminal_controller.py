# terminal_controller.py
import numpy as np
import kinematics
import path_manager

class TerminalController:
    def __init__(self, gui_main):
        """
        初始化時接收 RobotGUI 的實體 (Reference)，
        這樣就能存取 gui_main 的 log, current_joints, serial_manager 等屬性。
        """
        self.gui = gui_main

    def handle_terminal_command(self, cmd):
        # 1. 將使用者的輸入回顯到 Log 畫面上
        self.gui.log(f"➜ {cmd}")
        
        # 2. 轉大寫去空白，準備解析
        parts = cmd.strip().upper().split()
        if not parts: return
        
        action = parts[0]
        
        # --- 內建系統指令攔截 ---
        if action == "CLEAR":
            self.gui.clear_log()
            
        elif action == "HELP":
            self.gui.log(">> 可用指令: CLEAR, STOP, HOME, PTP [X Y Z], LIN [X Y Z]")
            
        elif action == "STOP":
            self.gui.emergency_stop()
            
        elif action == "HOME":
            self.gui.home_robot()
            
        # --- 空間移動指令 ---
        elif action in ["PTP", "LIN", "SLIN"]:
            if len(parts) == 1:
                self.gui.log(f"[Error] 請輸入目標座標，例如: {action} X100 Y50 Z200")
                return
            
            # 把指令類型 (LIN/PTP) 和後面的參數陣列交給專屬引擎處理
            self._jog_to_cartesian_target(move_type=action, args=parts[1:])
            
        # --- 未知指令：當作硬體 G-Code 直接發送 ---
        else:
            if hasattr(self.gui, 'serial_manager') and self.gui.serial_manager.is_connected:
                # 確保格式符合你的 Arduino 接收標準 (例如包上角括號)
                formatted_cmd = f"<{cmd.upper()}>" if not cmd.startswith('<') else cmd
                self.gui.serial_manager.send_command(formatted_cmd)
            else:
                self.gui.log("[Error] 硬體未連線，無法發送硬體指令。")

    def _jog_to_cartesian_target(self, move_type, args):
        """處理終端機輸入的空間坐標，算出 IK 後派發給執行器"""
        
        # 1. 取得當前的 TCP 絕對坐標與姿態 (作為局部更新的基準底板)
        user_offset = self.gui.tcp_manager.get_active_matrix()
        T_total_offset = self.gui.T_hw_fix @ user_offset
        
        T_flange_curr = kinematics.forward_kinematics(self.gui.current_joints)
        T_tcp_curr = T_flange_curr @ T_total_offset
        
        # 將當前矩陣拆解為 XYZ (mm) 與 RPY 尤拉角 (度)
        curr_pos = T_tcp_curr[:3, 3] * 1000.0
        curr_rpy = kinematics.R.from_matrix(T_tcp_curr[:3, :3]).as_euler('xyz', degrees=True)
        
        target_pos = curr_pos.copy()
        target_rpy = curr_rpy.copy()

        # 2. 解析使用者輸入的參數 (局部覆蓋)
        try:
            for arg in args:
                axis = arg[0].upper()
                # 處理 RX, RY, RZ
                if arg.startswith("RX"): target_rpy[0] = float(arg[2:])
                elif arg.startswith("RY"): target_rpy[1] = float(arg[2:])
                elif arg.startswith("RZ"): target_rpy[2] = float(arg[2:])
                # 處理 X, Y, Z
                elif axis == 'X': target_pos[0] = float(arg[1:])
                elif axis == 'Y': target_pos[1] = float(arg[1:])
                elif axis == 'Z': target_pos[2] = float(arg[1:])
                else:
                    self.gui.log(f"[Warning] 忽略未知的參數: {arg}")
        except ValueError:
            self.gui.log("[Error] 參數格式錯誤！請確保字母後直接接數字 (例如: X150.5)")
            return

        # 3. 將目標坐標組裝回 4x4 TCP 矩陣
        T_tcp_target = np.eye(4)
        T_tcp_target[:3, :3] = kinematics.R.from_euler('xyz', target_rpy, degrees=True).as_matrix()
        T_tcp_target[:3, 3] = target_pos / 1000.0  # 轉回公尺
        
        # 4. 剝離 TCP 偏移，換算成目標法蘭面矩陣
        T_flange_target = T_tcp_target @ np.linalg.inv(T_total_offset)
        
        # 5. 終極防護網：計算 IK
        pos_str = f"[{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]"
        rpy_str = f"[{target_rpy[0]:.2f}, {target_rpy[1]:.2f}, {target_rpy[2]:.2f}]"
        self.gui.log(f">> 計算 IK 中: XYZ={pos_str}, RPY={rpy_str}")
        
        target_joints, error = kinematics.inverse_kinematics(T_flange_target, self.gui.current_joints)
        
        if target_joints is None:
            self.gui.log("[Error] 目標超出工作範圍，或無法找到安全的姿態解！")
            return
            
        # 檢查是否跟現在的位置根本一樣
        if np.allclose(self.gui.current_joints, target_joints, atol=1e-4):
            self.gui.log("[Info] 目標位置與當前位置相同，無需移動。")
            return

        # 6. 安全過關！派發給執行緒發射
        if self.gui.path_manager.worker and self.gui.path_manager.worker.isRunning():
            self.gui.log("[Error] 系統正在執行其他路徑，請先 STOP。")
            return

        self.gui.log(f">> [Success] IK 解算成功，啟動 {move_type} 執行器。")
        
        # 套用畫面上設定的「連續 Jog 速度比例」
        speed_str = self.gui.jog_speed_combo.currentText().replace("%", "")
        speed_pct = float(speed_str) / 100.0

        if move_type == "SLIN":  # 🌟 測試我們的新管線！
            # 將單點指令偽裝成一個 Waypoint 清單
            wp_list = [{
                "move_type": "LIN",
                "target_joints": target_joints,
                "tcp_offset_mat": T_total_offset,
                "speed_factor": speed_pct
            }]
            self.gui.path_manager.worker = path_manager.StreamingPathExecutor(
                waypoint_list=wp_list,
                start_joints=self.gui.current_joints,
                serial_ref=self.gui.serial_manager,
                global_speed_factor=speed_pct
            )
        elif move_type == "LIN":  # 原本的舊版不變
            self.gui.path_manager.worker = path_manager.CartesianExecutor(
                start_joints=self.gui.current_joints, 
                target_joints=target_joints, 
                tcp_offset_mat=T_total_offset, 
                serial_ref=self.gui.serial_manager,  
                speed_factor=speed_pct,
                move_type="LIN"
            )
        else: # PTP
            self.gui.path_manager.worker = path_manager.PTPExecutor(
                start_joints=self.gui.current_joints, 
                end_joints=target_joints, 
                serial_ref=self.gui.serial_manager,  
                speed_factor=speed_pct,     
            )
            
        # 綁定訊號並開始執行
        self.gui.path_manager.worker.update_signal.connect(self.gui.on_manager_update_joints)
        self.gui.path_manager.worker.error_signal.connect(lambda msg: self.gui.log(f"[執行器錯誤] {msg}"))
        self.gui.path_manager.worker.log_signal.connect(self.gui.log)
        self.gui.path_manager.worker.finished_signal.connect(lambda: self.gui.log(f"([END]) {move_type} 移動完成。"))
        
        self.gui.path_manager.worker.start()