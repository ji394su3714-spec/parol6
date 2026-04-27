# terminal_controller.py
import numpy as np
import kinematics
import path_manager
import re

class TerminalController:
    def __init__(self, gui_main):
        self.gui = gui_main
        self.pending_cmd = None  

    def handle_terminal_command(self, cmd):
        self.gui.log(f"➜ {cmd}")
        cmd_lower = cmd.strip().lower()
        if not cmd_lower: return
        
        # 階段二：如果正在等待座標參數輸入
        if self.pending_cmd:
            if cmd_lower == "c":
                self.gui.log(">> [Terminal] 指令已取消。")
                self.pending_cmd = None
                return
            
            # 將第二段參數交給引擎解析並發射
            self._execute_two_stage_cmd(self.pending_cmd, cmd.strip().upper())
            self.pending_cmd = None
            return

        # 階段一：第一段指令攔截
        if cmd_lower == "clear":
            self.gui.clear_log()
            
        elif cmd_lower == "help":
            self.gui.log(">> 可用指令: CLEAR, STOP, HOME, UPD T, UPD A, LIN, REL")
            
        elif cmd_lower == "stop":
            self.gui.emergency_stop()
            
        elif cmd_lower == "home":
            self.gui.home_robot()
            
        elif cmd_lower == "upd p":
            row = self.gui.waypoint_list.currentRow()
            if row >= 0:
                self.gui.path_manager.update_target_joints(row, self.gui.current_joints)
            else:
                self.gui.log("[Terminal Error] 請先在左側清單點擊選擇一個點位！")
                
        elif cmd_lower == "upd a":
            row = self.gui.waypoint_list.currentRow()
            if row >= 0:
                self.gui.path_manager.update_aux_joints(row, self.gui.current_joints)
            else:
                self.gui.log("[Terminal Error] 請先在左側清單點擊選擇一個點位！")
                
        # 啟動 MDI 兩段式輸入模式
        elif cmd_lower == "lin":
            self.pending_cmd = "LIN"
            self.gui.log(">> [MDI 模式] 請輸入坐標系與【絕對位置】(例如: W X100 Y50)，或輸入 C 取消。")
            
        elif cmd_lower == "rel":
            self.pending_cmd = "REL"
            self.gui.log(">> [MDI 模式] 請輸入坐標系與【相對位移】(例如: T Z10 或 W X-5)，或輸入 C 取消。")
            
        else:
            if hasattr(self.gui, 'serial_manager') and self.gui.serial_manager.is_connected:
                formatted_cmd = f"<{cmd.upper()}>" if not cmd.startswith('<') else cmd
                self.gui.serial_manager.send_command(formatted_cmd)
            else:
                self.gui.log("[Error] 無效的指令，且硬體未連線，無法當作 G-code 發送。")


    # 核心：解析兩段式指令並派發給 Streaming 管線
    def _execute_two_stage_cmd(self, move_type, args_str):
        parts = args_str.strip().upper().split()
        if not parts: return
        frame = parts[0]
        
        if frame not in ['W', 'T']:
            self.gui.log("[Error] 第一個字必須是座標系 W (World) 或 T (Tool)！例如: T Z10")
            return

        if move_type == "LIN" and frame == 'T':
            self.gui.log("[Error] 絕對位置 (LIN) 僅支援 W (世界坐標系)。如需沿工具軸移動請改用 REL T。")
            return

        # 全新升級：防呆正則表達式解析器
        updates = {'X': None, 'Y': None, 'Z': None, 'RX': None, 'RY': None, 'RZ': None}
        
        # 把後面的參數全部接成一個字串 (例如 "X 100 RY -10")
        params_str = " ".join(parts[1:])
        
        # 魔法：自動抓取所有 (軸名稱) + (可選空格) + (數值)
        matches = re.findall(r'(RX|RY|RZ|X|Y|Z)\s*([+-]?\d*\.?\d+)', params_str)

        if not matches and params_str:
            self.gui.log("[Error] 參數格式無法辨識！請輸入如: X100 Y50 或 RX 10")
            return

        # 將抓到的數值填入字典
        for axis, val_str in matches:
            updates[axis] = float(val_str)

        # 1. 取得當前的 TCP 絕對矩陣
        user_offset = self.gui.tcp_manager.get_active_matrix()
        T_total_offset = self.gui.T_hw_fix @ user_offset
        T_flange_curr = kinematics.forward_kinematics(self.gui.current_joints)
        T_tcp_curr = T_flange_curr @ T_total_offset

        T_tcp_target = T_tcp_curr.copy()

        # 2. 矩陣幾何運算 (包含完美的旋轉邏輯)
        if move_type == "REL":
            dx = updates['X']/1000.0 if updates['X'] is not None else 0.0
            dy = updates['Y']/1000.0 if updates['Y'] is not None else 0.0
            dz = updates['Z']/1000.0 if updates['Z'] is not None else 0.0

            T_delta = np.eye(4)
            T_delta[0,3], T_delta[1,3], T_delta[2,3] = dx, dy, dz

            # 處理 RX, RY, RZ 的旋轉位移
            drx = updates['RX'] if updates['RX'] is not None else 0.0
            dry = updates['RY'] if updates['RY'] is not None else 0.0
            drz = updates['RZ'] if updates['RZ'] is not None else 0.0
            if drx != 0 or dry != 0 or drz != 0:
                # 將尤拉角變化量轉換為旋轉矩陣
                T_delta[:3, :3] = kinematics.R.from_euler('xyz', [drx, dry, drz], degrees=True).as_matrix()

            # 世界坐標系(左乘) vs 工具坐標系(右乘)
            if frame == 'W':
                T_tcp_target = T_delta @ T_tcp_curr  
            else:
                T_tcp_target = T_tcp_curr @ T_delta  

        elif move_type == "LIN":
            curr_pos = T_tcp_curr[:3, 3] * 1000.0
            curr_rpy = kinematics.R.from_matrix(T_tcp_curr[:3, :3]).as_euler('xyz', degrees=True)

            if updates['X'] is not None: curr_pos[0] = updates['X']
            if updates['Y'] is not None: curr_pos[1] = updates['Y']
            if updates['Z'] is not None: curr_pos[2] = updates['Z']
            if updates['RX'] is not None: curr_rpy[0] = updates['RX']
            if updates['RY'] is not None: curr_rpy[1] = updates['RY']
            if updates['RZ'] is not None: curr_rpy[2] = updates['RZ']

            T_tcp_target[:3, :3] = kinematics.R.from_euler('xyz', curr_rpy, degrees=True).as_matrix()
            T_tcp_target[:3, 3] = curr_pos / 1000.0

        # 3. 逆運動學求解
        T_flange_target = T_tcp_target @ np.linalg.inv(T_total_offset)
        target_joints, error = kinematics.inverse_kinematics(T_flange_target, self.gui.current_joints)

        if target_joints is None:
            self.gui.log("[Error] 目標超出工作範圍，或發生奇異點！")
            return

        # 4. 委派給終極雙緩衝管線 (把單點偽裝成陣列)
        if self.gui.path_manager.worker and self.gui.path_manager.worker.isRunning():
            self.gui.log("[Error] 系統正在執行其他路徑，請先 STOP。")
            return

        speed_str = self.gui.jog_speed_combo.currentText().replace("%", "")
        speed_pct = float(speed_str) / 100.0

        wp_list = [{
            "move_type": "LIN",  # 無論 REL 或 LIN，空間移動都走直線插補
            "target_joints": target_joints,
            "tcp_offset_mat": T_total_offset,
            "speed_factor": speed_pct,
            "accel_factor": speed_pct  
        }]

        #self.gui.log(f">> [MDI] 軌跡規劃成功，啟動 {move_type} 執行器。")

        self.gui.path_manager.worker = path_manager.StreamingPathExecutor(
            waypoint_list=wp_list,
            start_joints=self.gui.current_joints,
            serial_ref=self.gui.serial_manager
        )
        
        self.gui.path_manager.worker.update_signal.connect(self.gui.on_manager_update_joints)
        self.gui.path_manager.worker.error_signal.connect(lambda msg: self.gui.log(f"[執行器錯誤] {msg}"))
        self.gui.path_manager.worker.log_signal.connect(self.gui.log)
        self.gui.path_manager.worker.finished_signal.connect(lambda t: self.gui.log(f"([END]) 終端機 MDI 移動完成。"))
        
        self.gui.path_manager.worker.start()