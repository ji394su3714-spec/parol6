import json
import time
import numpy as np
import math
import queue
import threading
from scipy.signal import savgol_filter
from PyQt6.QtCore import QObject, QThread, pyqtSignal, QTimer
from PyQt6.QtWidgets import QFileDialog
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

import kinematics
from motion_profile import SCurveProfile 

# 關節專屬極限 (J1~J6)
MAX_JOINT_SPEEDS = np.array([158.20, 50.63, 55.94, 253.13, 253.13, 101.25])     
MAX_JOINT_ACCELS = np.array([527.34, 168.75, 186.46, 843.75, 843.75, 337.50])   
MAX_JOINT_JERKS = MAX_JOINT_ACCELS *5.0                               

# 直角空間極限 (LIN/CIRC 用)
MAX_LIN_SPEED = 150.0    
MAX_LIN_ACCEL = 300.0    
MAX_LIN_JERK = MAX_LIN_ACCEL * 5.0    

MAX_ROT_SPEED = 100.0     
MAX_ROT_ACCEL = 200.0     
MAX_ROT_JERK = MAX_ROT_ACCEL * 5.0     

# 晶片總體算力防護網參數
MAX_TOTAL_PULSE_SLICE = 15000   
MAX_TOTAL_PULSE_NATIVE = 65000  

N_PTP_T_ACC = 0.2  # N_PTP 預設起步時間

GEAR_RATIOS = np.array([6.4, 20.0, 18.095, 4.0, 4.0, 10.0])
STEPS_PER_DEG = (1600.0 * GEAR_RATIOS) / 360.0

# 對應 C++ JOINTS 陣列裡 mode=2 的硬體極速 (max_spd)
HW_MAX_STEPS = np.array([65000, 65000, 65000, 65000, 65000, 65000])

def update_advanced_settings(lin_spd, lin_acc, rot_spd, rot_acc, slice_pulse, native_pulse, hw_max, t_acc):
    global MAX_LIN_SPEED, MAX_LIN_ACCEL, MAX_LIN_JERK
    global MAX_ROT_SPEED, MAX_ROT_ACCEL, MAX_ROT_JERK
    global MAX_TOTAL_PULSE_SLICE, MAX_TOTAL_PULSE_NATIVE
    global HW_MAX_STEPS, N_PTP_T_ACC
    
    MAX_LIN_SPEED, MAX_LIN_ACCEL, MAX_LIN_JERK = lin_spd, lin_acc, lin_acc * 5.0
    MAX_ROT_SPEED, MAX_ROT_ACCEL, MAX_ROT_JERK = rot_spd, rot_acc, rot_acc * 5.0
    MAX_TOTAL_PULSE_SLICE, MAX_TOTAL_PULSE_NATIVE = slice_pulse, native_pulse
    HW_MAX_STEPS = np.array([hw_max]*6, dtype=float)
    N_PTP_T_ACC = t_acc

# 讓 PathManager 可以發送設定給 MCU
def send_config_to_mcu(serial_manager):
    if serial_manager and serial_manager.is_connected:
        cmd = f"<SET_CFG,{int(HW_MAX_STEPS[0])},{N_PTP_T_ACC}>"
        serial_manager.send_command(cmd)
        print(f">> [Config] Sent to MCU: {cmd}")

# --- 1. 原生 PTP 執行器 ---
class NativePTPExecutor(QThread):
    update_signal = pyqtSignal(list)
    finished_signal = pyqtSignal()
    error_signal = pyqtSignal(str)
    log_signal = pyqtSignal(str)

    def __init__(self, start_joints, target_joints, serial_ref=None, speed_factor=1.0, accel_factor=1.0):
        super().__init__()
        self.start_joints = np.array(start_joints)
        self.target_joints = np.array(target_joints)
        self.serial_ref = serial_ref
        self.speed_factor = speed_factor
        self.accel_factor = accel_factor 
        self._is_running = True

    def run(self):
        if self.serial_ref and self.serial_ref.is_connected:
            self.serial_ref.send_joints(list(self.target_joints), self.speed_factor, move_mode=2)
            if not self.serial_ref.wait_for_ok(timeout=3.0):
                self.error_signal.emit("Timeout: Arduino did not ack Native PTP.")
                return
            
            diffs = np.abs(self.target_joints - self.start_joints)
            max_diff = np.max(diffs)
            base_speed = max(300.0 * self.speed_factor, 1.0)
            steps = max(int(((max_diff / base_speed) + 0.05) / 0.05), 1)
            
            for i in range(steps):
                if not self._is_running: return  
                current_j = self.start_joints + (self.target_joints - self.start_joints) * ((i + 1) / steps)
                self.update_signal.emit(list(current_j))
                time.sleep(0.03) 
            
            self.update_signal.emit(list(self.target_joints))
            if hasattr(self.serial_ref, 'wait_for_motion_complete'):
                self.serial_ref.wait_for_motion_complete(timeout=60.0)
        self.finished_signal.emit()

# --- 2. 雙緩衝預讀執行器 (Streaming Pipeline) ---
class StreamingPathExecutor(QThread):
    update_signal = pyqtSignal(list)
    finished_signal = pyqtSignal(float) 
    error_signal = pyqtSignal(str)
    log_signal = pyqtSignal(str)

    def __init__(self, waypoint_list, start_joints, serial_ref=None):
        super().__init__()
        self.waypoint_list = waypoint_list
        self.start_joints = np.array(start_joints)
        self.serial_ref = serial_ref
        self._is_running = True
        # 核心水庫：最大容量 1000 點 (約 15 秒物理時間)，自帶背壓防堵機制
        self.point_queue = queue.Queue(maxsize=1000)
        self.producer_finished = False
        self.producer_error = False

    def run(self):
        prod_thread = threading.Thread(target=self._producer_task, daemon=True)
        prod_thread.start()

        self.log_signal.emit("[System] Compiling path, waiting for safe buffer level...")
        while self.point_queue.qsize() < 120 and not self.producer_finished and self._is_running:
            if self.producer_error: return
            self.msleep(10)

        if not self._is_running or self.producer_error: return
        #self.log_signal.emit(f"[System] Buffer level reached ({self.point_queue.qsize()} pts), starting rapid execution!")

        real_start_time = time.time()
        counter = 0
        interval = 0.015
        gui_skip_frames = 5
        self.update_signal.emit(list(self.start_joints))
        
        last_ik_joints = None

        while self._is_running:
            try:
                ik_joints = self.point_queue.get(timeout=0.1)
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
                delay_steps = max(1, int(delay_time / 0.015))
                curr_trajectory = [current_seed] * delay_steps
                msg = f"wait {delay_time} seconds..."
            else:
                target_joints = np.array(wp.get("target_joints", current_seed))
                
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

            if len(curr_trajectory) == 0:
                self.error_signal.emit(f"Waypoint {wp_idx+1} failed: {msg}")
                self.producer_error = True
                return
                
            if msg and msg != "SUCCESS" and not msg.startswith("wait"):
                self.log_signal.emit(f"Waypoint {wp_idx+1}: {msg}")

            # 2. 終極魔法：空間幾何貝茲切片 (Geometric Bezier Splicing)
            # 徹底解決速度與長度不均導致的軌跡變形問題！
            if pending_trajectory is not None:
                if pending_blend_str != 'FINE' and move_type != 'DELAY':
                    try:
                        pct = float(pending_blend_str.replace('%', '')) / 100.0
                    except ValueError:
                        pct = 0.0
                        
                    if pct > 0:
                        # --- TCP 空間融合升級版 ---
                        # 0. 準備 TCP 轉換矩陣
                        try:
                            tcp_inv = np.linalg.inv(tcp_offset_mat)
                        except:
                            tcp_inv = np.eye(4)
                            
                        # 1. 依然使用「法蘭面 (Flange)」的座標來計算物理距離 (因為定點繞軸時，法蘭面才有物理位移)
                        pos_A_start_flange = kinematics.forward_kinematics(pending_trajectory[0])[:3, 3]
                        pos_mid_flange = kinematics.forward_kinematics(pending_trajectory[-1])[:3, 3] # P2 點
                        pos_B_end_flange = kinematics.forward_kinematics(curr_trajectory[-1])[:3, 3]

                        # 2. 計算法蘭面的物理長度 (mm)
                        len_A = np.linalg.norm(pos_mid_flange - pos_A_start_flange) * 1000.0
                        len_B = np.linalg.norm(pos_B_end_flange - pos_mid_flange) * 1000.0

                        # 3. 嚴格基於短邊距離計算融合半徑
                        safe_pct = min(pct, 0.5)
                        blend_dist_mm = min(len_A, len_B) * safe_pct

                        if blend_dist_mm > 0.5: # 距離大於 0.5mm 才做融合
                            M = len(pending_trajectory)
                            L = len(curr_trajectory)
                            
                            # 4. 找尋切斷點 i_A (基於法蘭面距離)
                            i_A = M - 1
                            for i in range(M - 1, 0, -1):
                                pos_i = kinematics.forward_kinematics(pending_trajectory[i])[:3, 3]
                                if np.linalg.norm(pos_mid_flange - pos_i) * 1000.0 >= blend_dist_mm:
                                    i_A = i
                                    break
                            
                            # 5. 找尋切斷點 i_B (基於法蘭面距離)
                            i_B = 0
                            for i in range(0, L):
                                pos_i = kinematics.forward_kinematics(curr_trajectory[i])[:3, 3]
                                if np.linalg.norm(pos_i - pos_mid_flange) * 1000.0 >= blend_dist_mm:
                                    i_B = i
                                    break
                                    
                            # 核心改變：將橋樑的起、中、終點，全部轉換到 TCP 空間！
                            T_flange_start = kinematics.forward_kinematics(pending_trajectory[i_A])
                            T_tcp_start = T_flange_start @ tcp_offset_mat
                            P_start_tcp = T_tcp_start[:3, 3]
                            rot_start_tcp = R.from_matrix(T_tcp_start[:3, :3])
                            
                            T_flange_mid = kinematics.forward_kinematics(pending_trajectory[-1])
                            T_tcp_mid = T_flange_mid @ tcp_offset_mat
                            P_mid_tcp = T_tcp_mid[:3, 3]
                            
                            T_flange_end = kinematics.forward_kinematics(curr_trajectory[i_B])
                            T_tcp_end = T_flange_end @ tcp_offset_mat
                            P_end_tcp = T_tcp_end[:3, 3]
                            rot_end_tcp = R.from_matrix(T_tcp_end[:3, :3])

                            # 6. 計算過彎所需的時間 (必須使用法蘭面的弦長，否則定點繞軸時時間會變成0)
                            P_start_flange = T_flange_start[:3, 3]
                            P_end_flange = T_flange_end[:3, 3]
                            chord_flange = np.linalg.norm(P_end_flange - P_start_flange) * 1000.0
                            legs_flange = (np.linalg.norm(pos_mid_flange - P_start_flange) + np.linalg.norm(P_end_flange - pos_mid_flange)) * 1000.0
                            curve_len_flange = (chord_flange + legs_flange) / 2.0 
                            
                            # 7. 讀取兩端的法蘭面真實車速 (mm / 15ms)
                            if i_A > 0:
                                pos_prev_A = kinematics.forward_kinematics(pending_trajectory[i_A - 1])[:3, 3]
                                v_A = np.linalg.norm(P_start_flange - pos_prev_A) * 1000.0
                            else:
                                v_A = 0.05
                            v_A = max(v_A, 0.05) 
                            
                            if i_B < L - 1:
                                pos_next_B = kinematics.forward_kinematics(curr_trajectory[i_B + 1])[:3, 3]
                                v_B = np.linalg.norm(pos_next_B - P_end_flange) * 1000.0
                            else:
                                v_B = 0.05
                            v_B = max(v_B, 0.05) 
                                
                            v_avg = (v_A + v_B) / 2.0
                            N_blend = int(curve_len_flange / v_avg)
                            N_blend = max(N_blend, 3)

                            # 8. 生成實體空間的貝茲曲線
                            blended_section = []
                            current_seed = pending_trajectory[i_A] 
                            slerp_engine = Slerp([0, 1], R.from_matrix([rot_start_tcp.as_matrix(), rot_end_tcp.as_matrix()]))

                            for k in range(N_blend):
                                x = k / (N_blend - 1) if N_blend > 1 else 1.0
                                numerator = v_A * x + 0.5 * (v_B - v_A) * (x ** 2)
                                denominator = (v_A + v_B) / 2.0
                                t = numerator / denominator
                                t = max(0.0, min(1.0, t)) 
                                
                                # 空間位置：在 TCP 空間執行二階貝茲！
                                # 如果是定點繞軸，這三點座標相同，算出來保證還是同一個點。
                                P_bezier_tcp = ((1 - t)**2) * P_start_tcp + 2 * (1 - t) * t * P_mid_tcp + (t**2) * P_end_tcp
                                
                                # 空間姿態：在 TCP 空間執行 SLERP
                                rot_bezier_tcp = slerp_engine(t).as_matrix()
                                
                                T_tcp_target = np.eye(4)
                                T_tcp_target[:3, :3] = rot_bezier_tcp
                                T_tcp_target[:3, 3] = P_bezier_tcp
                                
                                # 最後一擊：將完美融合的 TCP 目標，反推回法蘭面丟給 IK
                                T_flange_target = T_tcp_target @ tcp_inv
                                
                                ik_result, err = kinematics.inverse_kinematics(T_flange_target, current_seed)
                                if ik_result is not None:
                                    current_seed = ik_result
                                    blended_section.append(ik_result)
                                else:
                                    break
                            
                            # 10. 外科手術級拼接
                            if len(blended_section) == N_blend:
                                pending_trajectory = list(pending_trajectory[:i_A]) + list(blended_section)
                                curr_trajectory = list(curr_trajectory[i_B:])

                # 3. 發射！把處理好的 pending 軌跡倒進水庫
                for ik_joints in pending_trajectory:
                    if not self._is_running: return
                    self.point_queue.put(ik_joints, block=True)
                
                self.msleep(1)

            # 4. 準備下一回合交接
            pending_trajectory = curr_trajectory
            pending_blend_str = wp.get('blend', 'FINE')
            
            prev_seed = current_seed 
            if len(pending_trajectory) > 0:
                current_seed = pending_trajectory[-1]
            
        # 5. 迴圈結束，把手上最後一段軌跡倒進水庫
        if pending_trajectory is not None:
            for ik_joints in pending_trajectory:
                if not self._is_running: return
                self.point_queue.put(ik_joints, block=True)
                
        self.producer_finished = True

# --- 3. 純粹數學兵工廠 (Trajectory Math Engine) ---
class TrajectoryMathEngine:
    
    @staticmethod
    def calculate_lin_trajectory(start_joints, target_joints, tcp_offset_mat, speed_factor, accel_factor=1.0):
        """LIN 直線空間向量化引擎"""
        try:
            tcp_inv = np.linalg.inv(tcp_offset_mat)
        except:
            return None, 0, "Invalid TCP Matrix"

        # 1. 幾何運算
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
            ik_result, error = kinematics.inverse_kinematics(T_flange_targets[i], current_seed)
            if ik_result is None:
                return None, 0, f"Dry-Run failed at {progress_samples[i]*100:.0f}%: No IK solution found! Action canceled."
            trajectory_joints.append(ik_result)
            current_seed = ik_result

        trajectory_joints = np.array(trajectory_joints) 

        # 🌟 優化 2：動態智慧 SG 濾波器 (防震盪)
        # window_len 絕對不能超過資料長度的 1/3，否則會產生多項式變形。
        max_safe_window = max(5, int(len(trajectory_joints) / 3))
        if max_safe_window % 2 == 0: max_safe_window += 1 # 保證是奇數
        window_len = min(15, max_safe_window) 

        trajectory_joints_smooth = savgol_filter(trajectory_joints, window_len, 3, axis=0) if len(trajectory_joints) >= 5 else trajectory_joints    

        d_joint_dp = np.gradient(trajectory_joints_smooth, delta_progress, axis=0)
        d2_joint_dp2 = np.gradient(d_joint_dp, delta_progress, axis=0)
        if len(trajectory_joints) >= 5: 
            d2_joint_dp2 = savgol_filter(d2_joint_dp2, window_len, 3, axis=0)

        # 🌟 優化 3：擴大邊界幽靈切除範圍
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

        overspeed_ratio = 1.0

        # 各軸速度與加速度防護
        for i in range(6):
            peak_joint_v = max_d_joint_dp[i] * peak_prog_v  
            allowed_v = MAX_JOINT_SPEEDS[i] 
            if peak_joint_v > allowed_v and allowed_v > 0:
                overspeed_ratio = max(overspeed_ratio, peak_joint_v / allowed_v)

            peak_joint_a_cruise = max_d2_joint_dp2[i] * (peak_prog_v ** 2) 
            peak_joint_a_accel = max_d_joint_dp[i] * peak_prog_a
            peak_joint_a = max(peak_joint_a_cruise, peak_joint_a_accel)
            
            allowed_a = MAX_JOINT_ACCELS[i] 
            if peak_joint_a > allowed_a and allowed_a > 0:
                overspeed_ratio = max(overspeed_ratio, math.sqrt(peak_joint_a / allowed_a))
                
        # ==========================================
        # 🌟 優化 4：修正算力防護網的「假性超載」
        # 我們不能把 6 軸的「歷史最大值」相加。
        # 必須算出每一個「瞬間」的總脈衝數 (axis=1)，再抓出其中最大的一瞬！
        # ==========================================
        instant_pulse_freqs = np.sum(np.abs(d_joint_dp_core) * peak_prog_v * STEPS_PER_DEG, axis=1)
        peak_pulse_freq = np.max(instant_pulse_freqs) if len(instant_pulse_freqs) > 0 else 0
        
        if peak_pulse_freq > MAX_TOTAL_PULSE_SLICE:
            overspeed_ratio = max(overspeed_ratio, peak_pulse_freq / MAX_TOTAL_PULSE_SLICE)
            
        msg = "SUCCESS"

        # 執行降速
        if overspeed_ratio > 1.0:
            target_speed /= overspeed_ratio
            target_accel /= (overspeed_ratio ** 2)
            target_jerk /= (overspeed_ratio ** 3)
            if peak_pulse_freq > MAX_TOTAL_PULSE_SLICE and (peak_pulse_freq / MAX_TOTAL_PULSE_SLICE) >= overspeed_ratio:
                msg = f"[LIN] 算力超載 ({peak_pulse_freq:.0f}Hz)！Scale down to {(1.0/overspeed_ratio):.2f}X"
            else:
                msg = f"[LIN] Safety deceleration triggered! Scale down to {(1.0/overspeed_ratio):.2f}X"
                
        final_profile = SCurveProfile(dist_main, target_speed, target_accel, target_jerk)

        # 3. 向量化切片
        interval = 0.015  
        t_steps = np.arange(0, final_profile.T_total, interval)
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
            overspeed_ratio = peak_pulse_freq / MAX_TOTAL_PULSE_SLICE
            target_speed /= overspeed_ratio
            target_accel /= (overspeed_ratio ** 2)
            target_jerk /= (overspeed_ratio ** 3)
            msg = f"[PTP] 算力超載 ({peak_pulse_freq:.0f}Hz)！Scale down to {(1.0/overspeed_ratio):.2f}X"

        final_profile = SCurveProfile(dist_main, target_speed, target_accel, target_jerk)

        # 3. 向量化切片
        interval = 0.015
        t_steps = np.arange(0, final_profile.T_total, interval)
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
        
        P1, P2, P3 = T_tcp_start[:3, 3], T_tcp_aux[:3, 3], T_tcp_end[:3, 3]
        
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
        interval = 0.015  
        t_steps = np.arange(0, final_profile.T_total, interval)
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
            ik_result, ik_error = kinematics.inverse_kinematics(T_flange_targets[i], current_seed)
            if ik_result is None or ik_error > 0.1:
                return None, 0, f"CIRC 軌跡 {prog_arr[i]*100:.1f}% 處遭遇死角！"
            current_seed = ik_result
            exact_trajectory.append(ik_result)
            
        return exact_trajectory, final_profile.T_total, "SUCCESS"

# --- 4. 總管大人 PathManager ---
class PathManager(QObject):
    log_signal = pyqtSignal(str)
    joint_update_signal = pyqtSignal(list)
    list_update_signal = pyqtSignal()
    
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

    def record_point(self, current_joints, move_type="PTP", speed=50.0, accel=50.0):
        aux = None
        if self.temp_aux_joints is not None:
            aux = [round(j, 4) for j in self.temp_aux_joints]
            move_type = "CIRC"
            self.temp_aux_joints = None

        idx = len(self.waypoints) + 1
        name = f"Point {idx}"
        data = {
            "name": name,
            "joints": [round(j, 4) for j in current_joints],
            "aux_joints": aux,
            "type": move_type,
            "speed": float(speed), 
            "accel": float(accel),
            "blend": "FINE",   # 預設新增點位都是 FINE
            "active": True,
            "note": ""
        }
        self.waypoints.append(data)
        self.list_update_signal.emit()
        
        msg = f"Recorded: {name} [{move_type}]"
        if move_type == "CIRC": msg += " (with AUX point)"
        self.log_signal.emit(msg)

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

    def delete_point(self, index):
        if 0 <= index < len(self.waypoints):
            removed = self.waypoints.pop(index)
            self._renumber_points()
            self.list_update_signal.emit()
            self.log_signal.emit(f"Deleted: {removed['name']}")

    def delete_all_points(self):
        self.waypoints.clear()
        self.list_update_signal.emit()
        self.log_signal.emit("All waypoints deleted.")

    def update_target_joints(self, index, joints):
        """後期編輯：更新指定點位的目標座標"""
        if 0 <= index < len(self.waypoints):
            # 強制更新座標，並保留小數點後四位
            self.waypoints[index]['joints'] = [round(j, 4) for j in joints]
            
            # 如果原本不小心對 DELAY 點按了更新，自動把它轉回實體點位
            if self.waypoints[index].get('type') == 'DELAY':
                self.waypoints[index]['type'] = 'PTP'
                
            self.list_update_signal.emit()
            self.log_signal.emit(f"Updated: {self.waypoints[index]['name']} (Target Pos)")

    def update_aux_joints(self, index, joints):
        """後期編輯：更新/新增指定點位的 AUX 輔助點，並自動轉為 CIRC"""
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
        for i, pt in enumerate(self.waypoints):
            pt['name'] = f"Point {i+1}"

    def save_to_file(self):
        filename, _ = QFileDialog.getSaveFileName(self.parent_widget, "Save Path", "", "JSON Files (*.json)")
        if filename:
            try:
                # 存檔前的「全自動清洗機」
                for pt in self.waypoints:
                    if 'accel' not in pt and pt.get('type') != 'DELAY':
                        pt['accel'] = 50.0
                    if 'blend' not in pt and pt.get('type') != 'DELAY':
                        pt['blend'] = "FINE"
                    if 'joints' in pt and pt['joints'] is not None:
                        pt['joints'] = [round(float(j), 4) for j in pt['joints']]
                    if 'aux_joints' in pt and pt['aux_joints'] is not None:
                        pt['aux_joints'] = [round(float(j), 4) for j in pt['aux_joints']]

                with open(filename, 'w') as f:
                    json.dump(self.waypoints, f, indent=4)
                self.log_signal.emit(f"Path saved to {filename}")
            except Exception as e:
                self.log_signal.emit(f"[Error] Save failed: {e}")

    def load_from_file(self):
        filename, _ = QFileDialog.getOpenFileName(self.parent_widget, "Load Path", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'r') as f:
                    self.waypoints = json.load(f)
                    for pt in self.waypoints:
                        if 'active' not in pt: pt['active'] = True
                        if 'type' not in pt: pt['type'] = "PTP"
                        if 'blend' not in pt: pt['blend'] = "FINE"
                self.list_update_signal.emit()
                self.log_signal.emit(f"Path loaded from {filename}")
            except Exception as e:
                self.log_signal.emit(f"[Error] Load failed: {e}")

    # 讓 3D 模擬畫面畫出線條
    def get_trajectory_preview(self, tcp_offset):
        trajectory_points = []
        active_wps = [pt for pt in self.waypoints if pt.get('active', True) and pt.get('type') != 'DELAY' and 'joints' in pt]

        if len(active_wps) >= 2:
            for i in range(len(active_wps) - 1):
                j_start = np.array(active_wps[i]['joints'])
                j_end = np.array(active_wps[i+1]['joints'])
                m_type = active_wps[i+1].get('type', 'LIN') 
                aux_joints = active_wps[i+1].get('aux_joints', None)
                
                steps = 20
                if m_type in ['PTP', 'N_PTP', 'NATIVE_PTP']:
                    for t in np.linspace(0, 1, steps):
                        interp = j_start + t * (j_end - j_start)
                        T_tcp = kinematics.forward_kinematics(interp) @ tcp_offset
                        trajectory_points.append(T_tcp[:3, 3])
                        
                elif m_type == 'CIRC' and aux_joints is not None:
                    T_s = kinematics.forward_kinematics(j_start) @ tcp_offset
                    T_a = kinematics.forward_kinematics(aux_joints) @ tcp_offset
                    T_e = kinematics.forward_kinematics(j_end) @ tcp_offset
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
                    T_s = kinematics.forward_kinematics(j_start) @ tcp_offset
                    T_e = kinematics.forward_kinematics(j_end) @ tcp_offset
                    xs, xe = T_s[:3, 3], T_e[:3, 3]
                    for t in np.linspace(0, 1, steps):
                        trajectory_points.append(xs + t * (xe - xs))
                        
        return trajectory_points

    # 核心：接管所有 UI 傳來的執行任務
    def execute_streaming_path(self, active_points, start_joints, tcp_offset_mat, loop, global_speed, global_accel, serial_ref, callbacks):
        wp_list = []
        for pt in active_points:
            move_type = pt.get('type', 'LIN')  
            pt_speed = pt.get('speed', global_speed * 100) / 100.0
            pt_accel = pt.get('accel', global_accel * 100) / 100.0
            
            wp = {
                "move_type": move_type,
                "target_joints": pt.get('joints', []), 
                "tcp_offset_mat": tcp_offset_mat,
                "speed_factor": pt_speed,
                "accel_factor": pt_accel,
                "value": pt.get('value', 0.0), 
                "blend": pt.get('blend', 'FINE') # 🌟 確保把 UI 的融合參數傳進大腦！
            }
            if move_type == "CIRC" and 'aux_joints' in pt:
                wp["aux_joints"] = pt['aux_joints']
                
            wp_list.append(wp)

        if loop:
            wp_list = wp_list * 10  

        self.worker = StreamingPathExecutor(
            waypoint_list=wp_list,
            start_joints=start_joints,
            serial_ref=serial_ref,
            )

        self.worker.update_signal.connect(callbacks['update'])
        self.worker.error_signal.connect(callbacks['error'])
        self.worker.log_signal.connect(callbacks['log'])
        self.worker.finished_signal.connect(callbacks['finished'])

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
            self.serial_manager.ok_event.set()
            self.serial_manager.motion_done_event.set()
            
        if self.worker and self.worker.isRunning():
            self.worker.wait() 
            
        self.worker = None
        self.log_signal.emit("([STOP]) Execution Halted.")