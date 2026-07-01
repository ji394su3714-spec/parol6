import json
import os
import time
import numpy as np
import math
import queue
import threading
from scipy.signal import savgol_filter
from PySide6.QtCore import QObject, QThread, Signal
from PySide6.QtWidgets import QFileDialog
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

import kinematics
from motion_profile import SCurveProfile 

# 關節專屬極限 (J1~J6)
MAX_JOINT_SPEEDS = np.array([175.78, 56.25, 62.17, 281.25, 281.25, 112.50])     
MAX_JOINT_ACCELS = np.array([703.13, 225.00, 248.69, 1125.00, 1125.00, 450.00])   
MAX_JOINT_JERKS = MAX_JOINT_ACCELS * 10.0                         

# 直角空間極限 (LIN/CIRC 用)
MAX_LIN_SPEED = 200.0    
MAX_LIN_ACCEL = 500.0    
MAX_LIN_JERK = MAX_LIN_ACCEL * 10.0    

MAX_ROT_SPEED = 100.0     
MAX_ROT_ACCEL = 200.0     
MAX_ROT_JERK = MAX_ROT_ACCEL * 10.0     

# 晶片總體算力防護網參數
MAX_TOTAL_PULSE_SLICE = 50000

GEAR_RATIOS = np.array([6.4, 20.0, 18.095, 4.0, 4.0, 10.0])
STEPS_PER_DEG = (6400.0 * GEAR_RATIOS) / 360.0


def update_advanced_settings(lin_spd, lin_acc, rot_spd, rot_acc, slice_pulse):
    global MAX_LIN_SPEED, MAX_LIN_ACCEL, MAX_LIN_JERK
    global MAX_ROT_SPEED, MAX_ROT_ACCEL, MAX_ROT_JERK
    global MAX_TOTAL_PULSE_SLICE
    
    MAX_LIN_SPEED, MAX_LIN_ACCEL, MAX_LIN_JERK = lin_spd, lin_acc, lin_acc * 10.0
    MAX_ROT_SPEED, MAX_ROT_ACCEL, MAX_ROT_JERK = rot_spd, rot_acc, rot_acc * 10.0
    MAX_TOTAL_PULSE_SLICE = slice_pulse

# --- 1. 雙緩衝預讀執行器 (Streaming Pipeline) ---
class StreamingPathExecutor(QThread):
    update_signal = Signal(list)
    finished_signal = Signal(float) 
    error_signal = Signal(str)
    log_signal = Signal(str)
    set_tcp_signal = Signal(int) # 新增：換刀專屬訊號

    def __init__(self, waypoint_list, start_joints, serial_ref=None, loop=False):
        super().__init__()
        self.waypoint_list = waypoint_list
        self.start_joints = np.array(start_joints)
        self.serial_ref = serial_ref
        self.loop = loop 
        self._is_running = True
        self.point_queue = queue.Queue(maxsize=300) # 核心水庫：最大容量 300 點 (3 秒)，自帶背壓防堵機制
        self.producer_finished = False
        self.producer_error = False

    def run(self):
        prod_thread = threading.Thread(target=self._producer_task, daemon=True)
        prod_thread.start()

        self.log_signal.emit("[System] Compiling path, waiting for safe buffer level...")
        while self.point_queue.qsize() < 290 and not self.producer_finished and self._is_running: 
            if self.producer_error: return
            self.msleep(10)

        if not self._is_running or self.producer_error: return

        real_start_time = time.time()
        counter = 0
        interval = 0.010
        gui_skip_frames = 6
        self.update_signal.emit(list(self.start_joints))
        
        last_ik_joints = None

        while self._is_running:
            try:
                # 先用 item 接收
                item = self.point_queue.get(timeout=0.1)
                
                # 攔截機制：檢查這是不是大腦塞進來的「日誌膠囊」
                if isinstance(item, dict):
                    if item.get("type") == "LOG":
                        self.log_signal.emit(item["msg"])

                    # 新增：處理換刀膠囊 (呼叫 GUI 更新畫面)
                    elif item.get("type") == "SET_TCP_CMD":
                        self.log_signal.emit(item["msg"])
                        self.set_tcp_signal.emit(item["tool_idx"])
                        continue

                    # 處理夾爪膠囊
                    elif item.get("type") == "EE_CMD":
                        self.log_signal.emit(item["msg"])
                        if self.serial_ref and self.serial_ref.is_connected:
                            
                            # 步驟 1：強制 Python 停止送單，並等待 6 軸馬達實體煞車停穩
                            if hasattr(self.serial_ref, 'wait_for_motion_complete'):
                                self.serial_ref.wait_for_motion_complete(timeout=10.0)
                            
                            # 步驟 2：發射夾爪指令
                            self.serial_ref.send_command(item["cmd"])
                            
                            # 步驟 3：關鍵修復：等待夾爪「專屬的」完成訊號！絕對不會被誤觸！
                            if hasattr(self.serial_ref, 'wait_for_ee_done'):
                                self.serial_ref.wait_for_ee_done(timeout=10.0)
                            
                    continue
                    
                # 如果不是膠囊，那就是真實的 6 軸座標，正常執行
                ik_joints = item
                last_ik_joints = ik_joints
                
                if counter % gui_skip_frames == 0:
                    self.update_signal.emit(list(ik_joints))
                    
                if self.serial_ref and self.serial_ref.is_connected:
                    self.serial_ref.send_joints(list(ik_joints), interval, move_mode=1)
                    self.serial_ref.wait_for_ok(timeout=3.0)
                else:
                    time.sleep(interval)
                    
                counter += 1
            except queue.Empty:
                if self.producer_finished: 
                    if last_ik_joints is not None:
                        self.update_signal.emit(list(last_ik_joints))
                    break
                else: 
                    # 只有在連線實體手臂時，才跳出斷炊警告
                    if self.serial_ref and self.serial_ref.is_connected:
                        self.log_signal.emit("[Warning] CPU computing too slowly, buffer underflow! Arm paused and waiting...")

        if self.serial_ref and self.serial_ref.is_connected and not self.producer_error:
            if hasattr(self.serial_ref, 'wait_for_motion_complete'):
                self.serial_ref.wait_for_motion_complete(timeout=10.0)
                
        real_total_time = time.time() - real_start_time
        self.finished_signal.emit(real_total_time)

    def _producer_task(self):
        current_seed = self.start_joints.copy()
        prev_seed = self.start_joints.copy()
        
        pending_trajectory = None       # 「扣留在手上」的上一段軌跡
        pending_blend_str = 'FINE'      # 上一段軌跡要求的融合百分比

        loop_count = 1  # 1. 新增迴圈計數器

        # 用 while 實現真正的無限迴圈
        while self._is_running:
        
            # 2. 只有在 Loop 模式下，才把「日誌膠囊」塞進水庫排隊
            if self.loop:
                self.point_queue.put({
                    "type": "LOG", 
                    "msg": f">> Start executing the path for loop {loop_count}..."
                }, block=True)
        
            for wp_idx, wp in enumerate(self.waypoint_list):
                if not self._is_running: return
                
                move_type = wp.get("move_type", "LIN")
                speed_factor = wp.get("speed_factor", 1.0)
                accel_factor = wp.get("accel_factor", 1.0)
                tcp_offset_mat = wp.get("tcp_offset_mat", np.eye(4))
                
                curr_trajectory = []
                msg = ""
                
                # 1. 正常計算當前路段 (軌跡 B)
                if move_type == "DELAY":
                    delay_time = wp.get("value", 0.0)
                    delay_steps = max(1, int(delay_time / 0.010))
                    curr_trajectory = [current_seed] * delay_steps
                    msg = f"wait {delay_time} seconds..."

                # 換刀動作專區
                elif move_type == "SET_TCP":
                    if pending_trajectory is not None:
                        for ik_joints in pending_trajectory:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                        pending_trajectory = None
                    
                    tool_idx = int(wp.get("value", 0))
                    tool_name = wp.get("name", "") # 抓取名稱
                    
                    self.point_queue.put({
                        "type": "SET_TCP_CMD",
                        "tool_idx": tool_idx,
                        "msg": f">> 執行換刀: 切換至 [{tool_idx}] {tool_name}" # 顯示出 Index 與名稱！
                    }, block=True)
                    
                    pending_blend_str = 'FINE' 
                    continue

                # 修復：夾爪動作專區
                elif move_type == "I/O":
                    # 1. 確保手臂先完全停下 (把手上扣留的軌跡全部清空倒進水庫)
                    if pending_trajectory is not None:
                        for ik_joints in pending_trajectory:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                        pending_trajectory = None
                    
                    # 2. 生成並放入專屬的 EE 膠囊
                    ee_type = wp.get("action_type", "DIGITAL")
                    ee_val = int(wp.get("value", 0))
                    self.point_queue.put({
                        "type": "EE_CMD",
                        "cmd": f"<EE,{ee_type},{ee_val}>",
                        "msg": f">> 🔧 執行工具動作: {ee_type} -> {ee_val}"
                    }, block=True)
                    
                    # 3. 關鍵修復：因為沒有實體移動，我們強制切斷融合狀態，並「直接跳過」這回合！
                    # 這樣就不會產生任何幽靈點位干擾 Arduino 的 isArmIdle 狀態！
                    pending_blend_str = 'FINE' 
                    continue
                    
                    # 給予一個假的原地停留點，維持時間軸與空間座標，避免被當作 empty list 報錯
                    curr_trajectory = [current_seed]
                    msg = "SUCCESS"
                else:
                    target_joints = np.array(wp.get("target_joints", current_seed))
                    
                    # 呼叫 TrajectoryMathEngine 的對應方法
                    if move_type == "PTP":
                        exact_traj, t_tot, msg = TrajectoryMathEngine.calculate_ptp_trajectory(
                            current_seed, target_joints, speed_factor, accel_factor
                        )
                        curr_trajectory = exact_traj if exact_traj is not None else []
                    elif move_type == "LIN":
                        exact_traj, t_tot, msg = TrajectoryMathEngine.calculate_lin_trajectory(
                            current_seed, target_joints, tcp_offset_mat, speed_factor, accel_factor
                        )
                        curr_trajectory = exact_traj if exact_traj is not None else []
                    elif move_type == "CIRC":
                        if "aux_joints" not in wp:
                            self.error_signal.emit(f"Waypoint {wp_idx+1} failed: CIRC missing AUX")
                            self.producer_error = True
                            return
                        exact_traj, t_tot, msg = TrajectoryMathEngine.calculate_circ_trajectory(
                            current_seed, wp["aux_joints"], target_joints, tcp_offset_mat, speed_factor, accel_factor
                        )
                        curr_trajectory = exact_traj if exact_traj is not None else []
                    else:
                        self.error_signal.emit(f"Unsupported move type {move_type}")
                        self.producer_error = True
                        return
                    
                curr_trajectory = [list(pt) for pt in curr_trajectory]

                if len(curr_trajectory) == 0:
                    self.error_signal.emit(f"Waypoint {wp_idx+1} failed: {msg}")
                    self.producer_error = True
                    return
                    
                if msg and msg != "SUCCESS" and not msg.startswith("wait"):
                    self.log_signal.emit(f"Waypoint {wp_idx+1}: {msg}")

                # 2. 時間軸向量疊加融合 (DJ Crossfade)
                if pending_trajectory is not None:
                    N = 0
                    if pending_blend_str != 'FINE' and move_type != 'DELAY':
                        try:
                            pct = float(pending_blend_str.replace('%', '')) / 100.0
                            # 允許高達 100% 的融合，但不能超過 1.0 (避免索引越界)
                            safe_pct = min(max(pct, 0.0), 1.0)
                            N = int(min(len(pending_trajectory), len(curr_trajectory)) * safe_pct)
                        except ValueError:
                            pass
                    
                    if N > 0:
                        M = len(pending_trajectory)
                        
                        # 擷取 A 的尾巴 (準備減速到 0 的部分)
                        arr_A = np.array(pending_trajectory[M - N : M])
                        
                        # 擷取 B 的開頭 (準備從 0 加速起步的部分)
                        arr_B = np.array(curr_trajectory[:N])
                        
                        # 擷取轉角的絕對頂點 (Target A)
                        vertex = np.array(pending_trajectory[-1])
                        
                        # 數學魔法：向量解析解 (Vectorized Analytic Solution)
                        blended_section = arr_A + arr_B - vertex
                        
                        # 流水線機制：先發射已經「安全 (不會再被下一段融合)」的前半段
                        for ik_joints in pending_trajectory[: M - N]:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                            
                        # 將「融合段 + B 的剩餘段」結合成新的 pending，扣在手上等下一回合
                        pending_trajectory = blended_section.tolist() + curr_trajectory[N:]
                        
                    else:
                        # 如果不需要融合 (N=0) 或遇到 DELAY，直接發射手上整條 pending
                        for ik_joints in pending_trajectory:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                        
                        # 換手
                        pending_trajectory = curr_trajectory
                        
                    self.msleep(1)
                else:
                    # 第一回合，直接把 curr 扣在手上
                    pending_trajectory = curr_trajectory

                # 4. 準備下一回合交接
                pending_blend_str = wp.get('blend', 'FINE')
                
                # 修復瞬移魔法：因為流水線結合了融合段與剩餘段，長度永遠等於 curr_trajectory
                # 所以 pending_trajectory[-1] 永遠完美對齊空間中真實的幾何終點！
                if len(pending_trajectory) > 0:
                    current_seed = pending_trajectory[-1]

                time.sleep(0.001)

            # 關鍵邏輯：跑完一趟 list 後，如果不需要 Loop，就打破 while 迴圈
            if not self.loop:
                break
                
            loop_count += 1  # 3. 跑完一趟，圈數 +1
            
        # 迴圈結束，把手上最後一段軌跡倒進水庫
        if pending_trajectory is not None:
            for ik_joints in pending_trajectory:
                if not self._is_running: return
                self.point_queue.put(ik_joints, block=True)
                
        self.producer_finished = True

# --- 2. 純粹數學兵工廠 (Trajectory Math Engine) ---
class TrajectoryMathEngine:
    
    @staticmethod
    def calculate_lin_trajectory(start_joints, target_joints, tcp_offset_mat, speed_factor, accel_factor=1.0):
        """LIN 直線空間向量化引擎"""
        try:
            tcp_inv = np.linalg.inv(tcp_offset_mat)
        except:
            return None, 0, "Invalid TCP Matrix"
    
        T_flange_start = kinematics.forward_kinematics(start_joints)
        T_tcp_start = T_flange_start @ tcp_offset_mat
        T_flange_end = kinematics.forward_kinematics(target_joints)
        T_tcp_end = T_flange_end @ tcp_offset_mat
        
        pos_start, pos_end = T_tcp_start[:3, 3], T_tcp_end[:3, 3]
        key_rots = R.from_matrix([T_tcp_start[:3, :3], T_tcp_end[:3, :3]])
        slerp = Slerp([0, 1], key_rots)
        rot_diff = key_rots[0].inv() * key_rots[1]
        dist_deg = np.linalg.norm(rot_diff.as_rotvec()) * (180.0 / math.pi)
        dist_mm = np.linalg.norm(pos_end - pos_start) * 1000.0
            
        if dist_mm < 0.1 and dist_deg < 0.1:
            return [start_joints], 0, "SUCCESS"

        # 2. 直接使用全域的 LIN/ROT 極限常數
        time_for_lin = dist_mm / (MAX_LIN_SPEED * speed_factor) if MAX_LIN_SPEED > 0 else 0
        time_for_rot = dist_deg / (MAX_ROT_SPEED * speed_factor) if MAX_ROT_SPEED > 0 else 0

        if time_for_lin >= time_for_rot:
            dist_main = dist_mm
            target_speed = MAX_LIN_SPEED * speed_factor
            target_accel = MAX_LIN_ACCEL * accel_factor
            target_jerk = MAX_LIN_JERK * accel_factor
        else:
            dist_main = dist_deg
            target_speed = MAX_ROT_SPEED * speed_factor
            target_accel = MAX_ROT_ACCEL * accel_factor
            target_jerk = MAX_ROT_JERK * accel_factor

        # Dry-Run 軌跡預掃描
        sample_steps = max(15, int(dist_main / 1.0))        
        progress_samples = np.linspace(0, 1.0, sample_steps)
        delta_progress = 1.0 / (sample_steps - 1)

        curr_pos_samples = pos_start + np.outer(progress_samples, (pos_end - pos_start))
        curr_rot_samples = slerp(progress_samples).as_matrix()

        T_tcp_targets = np.zeros((sample_steps, 4, 4))
        T_tcp_targets[:, 3, 3] = 1.0                
        T_tcp_targets[:, :3, :3] = curr_rot_samples     
        T_tcp_targets[:, :3, 3] = curr_pos_samples      
        T_flange_targets = T_tcp_targets @ tcp_inv  

        trajectory_joints = []
        current_seed = start_joints.copy()
        for i in range(sample_steps):
            if i % 10 == 0:
                time.sleep(0.001)

            ik_result, error = kinematics.inverse_kinematics(T_flange_targets[i], current_seed)
            if ik_result is None:
                return None, 0, f"Dry-Run failed at {progress_samples[i]*100:.0f}%: No IK solution found! Action canceled."
            trajectory_joints.append(ik_result)
            current_seed = ik_result

        trajectory_joints = np.array(trajectory_joints) 

        # 優化 2：動態智慧 SG 濾波器 (防震盪)
        # window_len 絕對不能超過資料長度的 1/3，否則會產生多項式變形。
        max_safe_window = max(5, int(len(trajectory_joints) / 3))
        if max_safe_window % 2 == 0: max_safe_window += 1 # 保證是奇數
        window_len = min(15, max_safe_window) 

        trajectory_joints_smooth = savgol_filter(trajectory_joints, window_len, 3, axis=0) if len(trajectory_joints) >= 5 else trajectory_joints    

        d_joint_dp = np.gradient(trajectory_joints_smooth, delta_progress, axis=0)
        d2_joint_dp2 = np.gradient(d_joint_dp, delta_progress, axis=0)
        if len(trajectory_joints) >= 5: 
            d2_joint_dp2 = savgol_filter(d2_joint_dp2, window_len, 3, axis=0)

        # 優化 3：擴大邊界幽靈切除範圍
        # 濾波器跟微積分在頭尾產生的誤差會蔓延，切掉頭尾 5% 才是最純淨的物理數據！
        trim_idx = max(2, int(sample_steps * 0.05))
        if len(d_joint_dp) > trim_idx * 2:
            d_joint_dp_core = d_joint_dp[trim_idx:-trim_idx]
            d2_joint_dp2_core = d2_joint_dp2[trim_idx:-trim_idx]
        else:
            d_joint_dp_core = d_joint_dp
            d2_joint_dp2_core = d2_joint_dp2

        max_d_joint_dp = np.max(np.abs(d_joint_dp_core), axis=0)
        max_d2_joint_dp2 = np.max(np.abs(d2_joint_dp2_core), axis=0)

        peak_prog_v = target_speed / dist_main if dist_main > 0 else 0
        peak_prog_a = target_accel / dist_main if dist_main > 0 else 0

        # 將原本的 overspeed_ratio 拆成兩個獨立的守門員
        v_overspeed_ratio = 1.0
        a_overspeed_ratio = 1.0

        # 各軸速度與加速度防護
        for i in range(6):
            # 1. 速度審查
            peak_joint_v = max_d_joint_dp[i] * peak_prog_v  
            allowed_v = MAX_JOINT_SPEEDS[i] 
            if peak_joint_v > allowed_v and allowed_v > 0:
                v_overspeed_ratio = max(v_overspeed_ratio, peak_joint_v / allowed_v)

            # 2. 加速度審查
            peak_joint_a_cruise = max_d2_joint_dp2[i] * (peak_prog_v ** 2) 
            peak_joint_a_accel = max_d_joint_dp[i] * peak_prog_a
            peak_joint_a = max(peak_joint_a_cruise, peak_joint_a_accel)
            
            allowed_a = MAX_JOINT_ACCELS[i] 
            if peak_joint_a > allowed_a and allowed_a > 0:
                a_overspeed_ratio = max(a_overspeed_ratio, math.sqrt(peak_joint_a / allowed_a))
                
        # 3. 晶片算力防護
        instant_pulse_freqs = np.sum(np.abs(d_joint_dp_core) * peak_prog_v * STEPS_PER_DEG, axis=1)
        peak_pulse_freq = np.max(instant_pulse_freqs) if len(instant_pulse_freqs) > 0 else 0
        
        pulse_ratio = peak_pulse_freq / MAX_TOTAL_PULSE_SLICE if MAX_TOTAL_PULSE_SLICE > 0 else 0
        if pulse_ratio > 1.0:
            v_overspeed_ratio = max(v_overspeed_ratio, pulse_ratio)
            
        msg = "SUCCESS"

        # 終極修正：獨立執行降速，互不連坐
        if v_overspeed_ratio > 1.0 or a_overspeed_ratio > 1.0:
            
            target_speed /= v_overspeed_ratio          
            target_accel /= (a_overspeed_ratio ** 2)
            target_jerk /= (a_overspeed_ratio ** 3)
            
            # 如果只是算力超載，訊息只顯示降速；如果同時加速超載，才顯示雙重降速訊息
            if pulse_ratio >= v_overspeed_ratio and pulse_ratio > 1.0:
                msg = f"[LIN] 算力超載 ({peak_pulse_freq:.0f}Hz)! Spd_Scale: {(1.0/v_overspeed_ratio):.2f}X"
            else:
                msg = f"[LIN] Overspeed! Spd_Scale: {(1.0/v_overspeed_ratio):.2f}X, Acc_Scale: {(1.0/(a_overspeed_ratio**2)):.2f}X"
                
        final_profile = SCurveProfile(dist_main, target_speed, target_accel, target_jerk)

        # 3. 向量化切片
        interval = 0.010  
        t_steps = np.arange(interval, final_profile.T_total, interval)
        if len(t_steps) == 0 or t_steps[-1] < final_profile.T_total:
            t_steps = np.append(t_steps, final_profile.T_total)

        prog_arr = np.array([final_profile.get_progress(t) for t in t_steps])
        N = len(prog_arr)

        curr_pos_arr = pos_start + np.outer(prog_arr, (pos_end - pos_start))
        curr_rot_arr = slerp(prog_arr).as_matrix()

        T_tcp_targets = np.zeros((N, 4, 4))
        T_tcp_targets[:, 3, 3] = 1.0                
        T_tcp_targets[:, :3, :3] = curr_rot_arr     
        T_tcp_targets[:, :3, 3] = curr_pos_arr      
        T_flange_targets = T_tcp_targets @ tcp_inv  

        exact_trajectory = []
        current_seed = start_joints.copy()
        
        # 4. IK 計算 (加入嚴格的 0.1mm 奇異點防護)
        for i in range(N):
            if i % 10 == 0:
                time.sleep(0.001)

            ik_result, ik_error = kinematics.inverse_kinematics(T_flange_targets[i], current_seed)
            if ik_result is None or ik_error > 0.1:
                return None, 0, f"軌跡 {prog_arr[i]*100:.1f}% 處遭遇死角或奇異點！"
            current_seed = ik_result
            exact_trajectory.append(ik_result)
            
        return exact_trajectory, final_profile.T_total, msg

    @staticmethod
    def calculate_ptp_trajectory(start_joints, target_joints, speed_factor, accel_factor=1.0):
        """PTP 關節空間向量化引擎 (包含精準加減速瓶頸演算法)"""
        start_joints, target_joints = np.array(start_joints), np.array(target_joints)

        # 1. 計算 6 個關節各自要走多遠
        delta_joints = target_joints - start_joints
        
        if np.max(np.abs(delta_joints)) < 0.1:
            return [start_joints], 0, "SUCCESS"
        
        # 2. 終極瓶頸計算：同時考量極速與加速度
        d = np.abs(delta_joints)
        v = np.where(MAX_JOINT_SPEEDS == 0, 1e-6, MAX_JOINT_SPEEDS) * speed_factor
        a = np.where(MAX_JOINT_ACCELS == 0, 1e-6, MAX_JOINT_ACCELS) * accel_factor
        
        is_triangle = d <= (v**2) / a
        time_needed = np.where(is_triangle, 2 * np.sqrt(d / a), (d / v) + (v / a))
        # 找出真正需要花最久時間的「瓶頸馬達」
        bottleneck_idx = np.argmax(time_needed)
        dist_main = np.abs(delta_joints[bottleneck_idx])
        target_speed = MAX_JOINT_SPEEDS[bottleneck_idx] * speed_factor
        target_accel = MAX_JOINT_ACCELS[bottleneck_idx] * accel_factor
        target_jerk = MAX_JOINT_JERKS[bottleneck_idx] * accel_factor

        # PTP 的晶片算力防護網
        msg = "SUCCESS"
        peak_progress_rate = target_speed / dist_main if dist_main > 0 else 0
        peak_pulse_freq = np.sum(np.abs(delta_joints) * peak_progress_rate * STEPS_PER_DEG)
        
        if peak_pulse_freq > MAX_TOTAL_PULSE_SLICE:
            v_overspeed_ratio = peak_pulse_freq / MAX_TOTAL_PULSE_SLICE
            
            # 終極修正：算力超載純粹是頻率問題，絕對只降速，不碰加速度！
            target_speed /= v_overspeed_ratio
            
            msg = f"[PTP] 算力超載 ({peak_pulse_freq:.0f}Hz)！Speed Scale: {(1.0/v_overspeed_ratio):.2f}X"

        final_profile = SCurveProfile(dist_main, target_speed, target_accel, target_jerk)

        # 3. 向量化切片
        interval = 0.010
        t_steps = np.arange(interval, final_profile.T_total, interval)
        if len(t_steps) == 0 or t_steps[-1] < final_profile.T_total:
            t_steps = np.append(t_steps, final_profile.T_total)

        prog_arr = np.array([final_profile.get_progress(t) for t in t_steps])

        # 4. 終極向量化插值：瞬間算出 N x 6 的軌跡矩陣！
        exact_trajectory = start_joints + np.outer(prog_arr, delta_joints)

        return exact_trajectory, final_profile.T_total, msg

    @staticmethod
    def calculate_circ_trajectory(start_joints, aux_joints, target_joints, tcp_offset_mat, speed_factor, accel_factor=1.0):
        """CIRC 圓弧空間向量化引擎 (包含 Dry-Run 防護)"""
        try:
            tcp_inv = np.linalg.inv(tcp_offset_mat)
        except:
            return None, 0, "Invalid TCP Matrix"
        
        # 1. 計算空間中的三個點 (P1起點, P2輔助點, P3終點)
        T_tcp_start = kinematics.forward_kinematics(start_joints) @ tcp_offset_mat
        T_tcp_aux = kinematics.forward_kinematics(aux_joints) @ tcp_offset_mat
        T_tcp_end = kinematics.forward_kinematics(target_joints) @ tcp_offset_mat
        
        P1, P2, P3 = T_tcp_start[:3, 3], T_tcp_end[:3, 3], T_tcp_end[:3, 3]
        
        # 2. 三點求圓心與半徑
        v1, v2 = P2 - P1, P3 - P1
        cross_v1_v2 = np.cross(v1, v2)
        cross_norm = np.linalg.norm(cross_v1_v2)
        if cross_norm < 1e-6:
            return None, 0, "CIRC Error: Three points are collinear, cannot form an arc!"
            
        normal = cross_v1_v2 / cross_norm
        v1_sq, v2_sq = np.dot(v1, v1), np.dot(v2, v2)
        center_rel = np.cross((v1_sq * v2 - v2_sq * v1), cross_v1_v2) / (2.0 * cross_norm**2)
        center = P1 + center_rel
        radius = np.linalg.norm(center - P1)
        # 計算夾角與弧長
        u = (P1 - center) / radius
        w = np.cross(normal, u)
        
        v_p3 = (P3 - center) / radius
        total_angle = math.atan2(np.dot(v_p3, w), np.dot(v_p3, u))
        if total_angle < 0: total_angle += 2 * math.pi
        
        arc_length_mm = radius * total_angle * 1000.0 

        # 3. S-Curve 規劃 (基於弧長)
        target_speed = MAX_LIN_SPEED * speed_factor
        target_accel = MAX_LIN_ACCEL * accel_factor
        target_jerk = MAX_LIN_JERK * accel_factor
        # (此處為簡化版：省略了 LIN 那落落長的 Dry-Run 計算，實務上可共用該降速模組)
        final_profile = SCurveProfile(arc_length_mm, target_speed, target_accel, target_jerk)

        # 4. 向量化切片
        interval = 0.010  
        t_steps = np.arange(interval, final_profile.T_total, interval)
        if len(t_steps) == 0 or t_steps[-1] < final_profile.T_total:
            t_steps = np.append(t_steps, final_profile.T_total)

        prog_arr = np.array([final_profile.get_progress(t) for t in t_steps])
        N = len(prog_arr)

        # 5. 計算圓弧上的每一點座標
        angles = prog_arr * total_angle
        curr_pos_arr = center + radius * (np.outer(np.cos(angles), u) + np.outer(np.sin(angles), w))
        # 旋轉姿態插值 (使用 Slerp)
        key_rots = R.from_matrix([T_tcp_start[:3, :3], T_tcp_end[:3, :3]])
        slerp = Slerp([0, 1], key_rots)
        curr_rot_arr = slerp(prog_arr).as_matrix()

        # 6. 組裝矩陣與 IK 解算
        T_tcp_targets = np.zeros((N, 4, 4))
        T_tcp_targets[:, 3, 3] = 1.0                
        T_tcp_targets[:, :3, :3] = curr_rot_arr     
        T_tcp_targets[:, :3, 3] = curr_pos_arr      
        T_flange_targets = T_tcp_targets @ tcp_inv  

        exact_trajectory = []
        current_seed = start_joints.copy()
        
        for i in range(N):
            if i % 10 == 0:
                time.sleep(0.001)
                
            ik_result, ik_error = kinematics.inverse_kinematics(T_flange_targets[i], current_seed)
            if ik_result is None or ik_error > 0.1:
                return None, 0, f"CIRC 軌跡 {prog_arr[i]*100:.1f}% 處遭遇死角！"
            current_seed = ik_result
            exact_trajectory.append(ik_result)
            
        return exact_trajectory, final_profile.T_total, "SUCCESS"

# --- 3. 總管大人 PathManager ---
class PathManager(QObject):
    log_signal = Signal(str)
    joint_update_signal = Signal(list)
    list_update_signal = Signal()
    file_loaded_signal = Signal(str) # 新增這行：用來廣播檔案名稱
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.waypoints = []
        self.worker = None
        self.parent_widget = parent
        
        self.temp_aux_joints = None 
        self.serial_manager = None
        if hasattr(parent, 'serial_manager'):
            self.serial_manager = parent.serial_manager

    def is_running(self):
        return hasattr(self, 'worker') and self.worker is not None and self.worker.isRunning()

    def set_aux_point(self, joints):
        self.temp_aux_joints = list(joints)
        self.log_signal.emit(">> [CIRC] AUX point saved! Move to END point and press Record.")

    def _get_next_point_number(self):
        """計算下一個實體動作點位 (PTP, LIN, CIRC) 的 Point 序號"""
        count = 0
        for wp in self.waypoints:
            if wp.get('type') in ['PTP', 'LIN', 'CIRC']:
                count += 1
        return count + 1

    def record_point(self, current_joints, move_type="PTP", speed=50.0, accel=50.0):
        aux = None
        if self.temp_aux_joints is not None:
            aux = [round(j, 4) for j in self.temp_aux_joints]
            move_type = "CIRC"
            self.temp_aux_joints = None

        # 呼叫專屬計數器，只追蹤實體點位！
        pt_num = self._get_next_point_number()
        name = f"Point {pt_num}"
        
        data = {
            "name": name,
            "joints": [round(j, 4) for j in current_joints],
            "aux_joints": aux,
            "type": move_type,
            "speed": float(speed), 
            "accel": float(accel),
            "blend": "FINE", 
            "active": True,
            "note": ""
        }
        self.waypoints.append(data)
        self.list_update_signal.emit()
        
        msg = f"Recorded: {name} [{move_type}]"
        if move_type == "CIRC": msg += " (with AUX point)"
        self.log_signal.emit(msg)

    def update_point_at_index(self, index, current_joints):
        """將目前手臂的座標，覆蓋掉指定的實體動作點位"""
        if 0 <= index < len(self.waypoints):
            wp = self.waypoints[index]
            # 檢查是不是帶有座標的實體點位
            if wp.get('type') in ['PTP', 'LIN', 'CIRC']:
                # 重新寫入當前座標 (四捨五入到小數點後 4 位)
                wp['joints'] = [round(j, 4) for j in current_joints]
                
                # 通知 UI 與 3D 預覽線重新繪製
                self.list_update_signal.emit()
                
                # 印出成功通知
                self.log_signal.emit(f">>> UPDATED: [{wp.get('name')}] position updated to current pose.")
            else:
                self.log_signal.emit(f"[WARNING] 無法更新座標：{wp.get('type')} 不是實體動作點位。")

    def insert_waypoint(self, index, waypoint_data):
        """統一的插入接口：將新點位插入到指定的 index (即原本點位的前面)"""
        # Python 的 list.insert(index, obj) 原生就是插在該 index 的前面
        self.waypoints.insert(index, waypoint_data)
        
        # 如果你原本的架構有 _renumber_points，可以保留呼叫
        if hasattr(self, '_renumber_points'):
            self._renumber_points()
            
        self.list_update_signal.emit()
        self.log_signal.emit(f">>> INSERTED: [{waypoint_data['type']}] at line {index + 1}")

    def record_delay(self, time_sec=2.0):
        idx = len(self.waypoints) + 1
        name = f"Wait {time_sec}s"
        data = {
            "name": name,
            "type": "DELAY",
            "value": float(time_sec), 
            "active": True,
            "note": ""
        }
        self.waypoints.append(data)
        self.list_update_signal.emit()
        self.log_signal.emit(f"Recorded: {name}")

    def record_tool_action(self, action_dict):
        """將夾爪動作寫入教導清單中"""
        idx = len(self.waypoints) + 1
        name = f"EE: {action_dict.get('note', 'Action')}"
        data = {
            "name": name,
            "type": "I/O",
            "action_type": action_dict['action_type'],
            "value": action_dict['value'],
            "active": True,
            "note": action_dict.get('note', '')
        }
        self.waypoints.append(data)
        self.list_update_signal.emit()
        self.log_signal.emit(f"Recorded Tool Action: {name}")

    def record_tcp_switch(self, tool_index, tool_name):
        """錄製 TCP (刀具) 切換指令"""
        wp = {
            "type": "SET_TCP",
            "value": tool_index,
            "name": f"Tool: {tool_name}",
            "active": True
        }
        self.waypoints.append(wp)
        self.list_update_signal.emit()
        self.log_signal.emit(f">>> RECORDED: [SET_TCP] Switched to {tool_name}")

    def update_tcp_command(self, index, new_tool_idx, new_tool_name):
        """專門用來更新已錄製的 SET_TCP 指令"""
        if 0 <= index < len(self.waypoints):
            wp = self.waypoints[index]
            if wp.get('type') == "SET_TCP":
                wp['value'] = new_tool_idx
                wp['name'] = f"Tool: {new_tool_name}"
                
                # 發送訊號刷新 UI
                self.list_update_signal.emit()
                self.log_signal.emit(f">>> UPDATED: [SET_TCP] changed to {new_tool_name}")

    def delete_point(self, index):
        if 0 <= index < len(self.waypoints):
            removed = self.waypoints.pop(index)
            point_name = removed.get('name', f"Type: {removed.get('type', 'Unknown')}")
            
            self.log_signal.emit(f">>> DELETED: {point_name} (Line {index + 1})")
            self.list_update_signal.emit()

    def delete_all_points(self):
        self.waypoints.clear()
        self.list_update_signal.emit()
        self.log_signal.emit("All waypoints deleted.")

    def update_aux_joints(self, index, joints):
        """後期編輯：更新指定點位的輔助座標"""
        if 0 <= index < len(self.waypoints):
            self.waypoints[index]['type'] = 'CIRC'
            self.waypoints[index]['aux_joints'] = [round(j, 4) for j in joints]
            self.list_update_signal.emit()
            self.log_signal.emit(f"Updated: {self.waypoints[index]['name']} (AUX Pos Added)")

    def toggle_point_active(self, index):
        if 0 <= index < len(self.waypoints):
            current_state = self.waypoints[index].get('active', True)
            self.waypoints[index]['active'] = not current_state
            self.list_update_signal.emit()

    def _renumber_points(self):
        """刪除點位後重新編號 (只對實體動作點位生效)"""
        count = 1
        for pt in self.waypoints:
            if pt.get('type') in ['PTP', 'LIN', 'CIRC']:
                pt['name'] = f"Point {count}"
                count += 1

    def save_to_file(self):
        filename, _ = QFileDialog.getSaveFileName(self.parent_widget, "Save Path", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.waypoints, f, indent=4)
                
                base_name = os.path.basename(filename)
                self.log_signal.emit(f"[System] UPDATED: Waypoints successfully saved to {base_name}")
                
                if hasattr(self, 'file_loaded_signal'):
                    self.file_loaded_signal.emit(base_name)
                
            except Exception as e:
                self.log_signal.emit(f"[ERROR] Save failed: {e}")

    def load_from_file(self):
        filename, _ = QFileDialog.getOpenFileName(self.parent_widget, "Load Path", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                
                if isinstance(data, list):
                    self.waypoints = data
                    self.list_update_signal.emit()
                    
                    base_name = os.path.basename(filename)
                    self.log_signal.emit(f"[System] UPDATED: Waypoints successfully loaded from {base_name}")
                    
                    if hasattr(self, 'file_loaded_signal'):
                        self.file_loaded_signal.emit(base_name)
                else:
                    self.log_signal.emit("[ERROR] Invalid file format: Expected a list of waypoints.")
                    
            except Exception as e:
                self.log_signal.emit(f"[ERROR] Load failed: {e}")

    # 讓 3D 模擬畫面畫出線條 (支援動態換刀預覽與安全防護)
    def get_trajectory_preview(self, initial_tcp_offset=None):
        trajectory_points = []
        valid_physical_pts = []
        
        # 🌟 關鍵修復 1：建立一個內建的「安全 TCP 矩陣產生器」
        def get_tcp_matrix(tool_idx):
            # 預設的法蘭盤偏移 (X 軸轉 -90 度)
            flange_offset = np.eye(4)
            flange_offset[:3, :3] = R.from_euler('x', -90, degrees=True).as_matrix()
            
            # 嘗試去 Tool Manager 抓取指定的刀具參數
            if hasattr(self, 'parent_widget') and hasattr(self.parent_widget, 'tcp_manager'):
                tcp_mgr = self.parent_widget.tcp_manager
                if 0 <= tool_idx < len(tcp_mgr.tools):
                    vals = tcp_mgr.tools[tool_idx]["values"]
                    x, y, z, rx, ry, rz = vals
                    user_mat = np.eye(4)
                    user_mat[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
                    user_mat[:3, 3] = [x/1000.0, y/1000.0, z/1000.0]
                    return flange_offset @ user_mat
                    
            # 如果找不到刀具，安全地回傳純法蘭盤矩陣，避免程式崩潰
            return flange_offset

        # 🌟 關鍵修復 2：預設起點無條件使用 Tool 0 的矩陣
        current_tcp = get_tcp_matrix(0)
        
        # 1. 掃描所有點位，為每個實體動作點綁定「當下」的 TCP 矩陣
        for wp in self.waypoints:
            if not wp.get('active', True):
                continue
                
            m_type = wp.get('type')
            
            # 遇到換刀指令，呼叫安全產生器更新 current_tcp
            if m_type == "SET_TCP":
                tool_idx = int(wp.get("value", 0))
                current_tcp = get_tcp_matrix(tool_idx)
                        
            # 如果是實體移動點，就把「當下使用的 TCP」跟著座標一起綁定存起來
            elif m_type in ['PTP', 'LIN', 'CIRC'] and 'joints' in wp:
                valid_physical_pts.append({
                    'joints': wp['joints'],
                    'type': m_type,
                    'aux_joints': wp.get('aux_joints'),
                    'tcp': current_tcp
                })

        # 2. 開始依照綁定的 TCP 畫線
        if len(valid_physical_pts) >= 2:
            for i in range(len(valid_physical_pts) - 1):
                p_start = valid_physical_pts[i]
                p_end = valid_physical_pts[i+1]
                
                j_start = np.array(p_start['joints'])
                j_end = np.array(p_end['joints'])
                m_type = p_end['type']
                aux_joints = p_end['aux_joints']
                
                active_tcp = p_end['tcp']
                
                steps = 20
                if m_type in ['PTP']:
                    for t in np.linspace(0, 1, steps):
                        interp = j_start + t * (j_end - j_start)
                        T_tcp = kinematics.forward_kinematics(interp) @ active_tcp
                        trajectory_points.append(T_tcp[:3, 3])
                        
                elif m_type == 'CIRC' and aux_joints is not None:
                    T_s = kinematics.forward_kinematics(j_start) @ active_tcp
                    T_a = kinematics.forward_kinematics(aux_joints) @ active_tcp
                    T_e = kinematics.forward_kinematics(j_end) @ active_tcp
                    ps, pa, pe = T_s[:3, 3], T_a[:3, 3], T_e[:3, 3]
                    
                    u = pa - ps
                    w = pe - ps
                    cross_uw = np.cross(u, w)
                    cross_norm = np.linalg.norm(cross_uw)
                    
                    if cross_norm > 1e-6:
                        u2, w2 = np.dot(u, u), np.dot(w, w)
                        C = ps + np.cross((u2 * w - w2 * u), cross_uw) / (2.0 * cross_norm**2)
                        r = np.linalg.norm(ps - C)
                        
                        n = cross_uw / cross_norm
                        x_axis = (ps - C) / r
                        y_axis = np.cross(n, x_axis)
                        
                        theta_e = math.atan2(np.dot(pe - C, y_axis), np.dot(pe - C, x_axis))
                        if theta_e < 0: theta_e += 2 * math.pi
                        
                        for t in np.linspace(0, 1, steps):
                            theta = t * theta_e
                            pt = C + r * math.cos(theta) * x_axis + r * math.sin(theta) * y_axis
                            trajectory_points.append(pt)
                    else:
                        for t in np.linspace(0, 1, steps):
                            trajectory_points.append(ps + t * (pe - ps))
                else: 
                    T_s = kinematics.forward_kinematics(j_start) @ active_tcp
                    T_e = kinematics.forward_kinematics(j_end) @ active_tcp
                    xs, xe = T_s[:3, 3], T_e[:3, 3]
                    for t in np.linspace(0, 1, steps):
                        trajectory_points.append(xs + t * (xe - xs))
                        
        return trajectory_points

    # 核心：接管所有 UI 傳來的執行任務
    def execute_streaming_path(self, active_points, start_joints, tcp_offset_mat, loop, global_speed, global_accel, serial_ref, callbacks):
        wp_list = []
        current_tcp_mat = tcp_offset_mat 
        tcp_mgr = self.parent_widget.tcp_manager 
        
        for pt in active_points:
            move_type = pt.get('type', 'LIN')  
            
            if move_type == "SET_TCP":
                tool_idx = int(pt.get("value", 0))
                if 0 <= tool_idx < len(tcp_mgr.tools):
                    vals = tcp_mgr.tools[tool_idx]["values"]
                    x, y, z, rx, ry, rz = vals
                    
                    user_mat = np.eye(4)
                    r_mat = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
                    user_mat[:3, :3] = r_mat
                    user_mat[:3, 3] = [x/1000.0, y/1000.0, z/1000.0]
                    
                    flange_offset = np.eye(4)
                    flange_offset[:3, :3] = R.from_euler('x', -90, degrees=True).as_matrix()
                    
                    current_tcp_mat = flange_offset @ user_mat

            pt_speed = pt.get('speed', global_speed * 100) / 100.0
            pt_accel = pt.get('accel', global_accel * 100) / 100.0
            
            wp = {
                "move_type": move_type,
                "name": pt.get('name', ''), 
                "target_joints": pt.get('joints', []), 
                "tcp_offset_mat": current_tcp_mat, 
                "speed_factor": pt_speed,
                "accel_factor": pt_accel,
                "value": pt.get('value', 0.0), 
                "blend": pt.get('blend', 'FINE')
            }
            if move_type == "CIRC" and 'aux_joints' in pt:
                wp["aux_joints"] = pt['aux_joints']
                
                # 新增這裡：如果是夾爪動作，要把 action_type (SERVO/DIGITAL) 也傳進去
            if move_type == "I/O":
                wp["action_type"] = pt.get("action_type", "DIGITAL")
                
            wp_list.append(wp)

        self.worker = StreamingPathExecutor(
            waypoint_list=wp_list,
            start_joints=start_joints,
            serial_ref=serial_ref,
            loop=loop 
        )  

        self.worker.update_signal.connect(callbacks['update'])
        self.worker.error_signal.connect(callbacks['error'])
        self.worker.log_signal.connect(callbacks['log'])
        self.worker.finished_signal.connect(callbacks['finished'])
        if 'set_tcp' in callbacks:
            self.worker.set_tcp_signal.connect(callbacks['set_tcp'])

        self.worker.start()

    def _on_worker_error(self, msg):
        self.log_signal.emit(f"[STOP] {msg}")
        self.stop_path()

    def stop_path(self):
        if self.worker:
            self.worker._is_running = False 

        if self.serial_manager and self.serial_manager.is_connected:
            self.serial_manager.send_command("<STOP>")
            
        if self.serial_manager:
            if hasattr(self.serial_manager, 'ok_semaphore'):
                self.serial_manager.ok_semaphore.release()
                
            self.serial_manager.motion_done_event.set()
            
        if self.worker and self.worker.isRunning():
            self.worker.wait() 
            
        self.worker = None
        self.log_signal.emit("([STOP]) Execution Halted.")