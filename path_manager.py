import threading # 用於執行緒鎖
import json
import time
import numpy as np
import math
from scipy.signal import savgol_filter
from PyQt6.QtCore import QObject, QThread, pyqtSignal, QTimer
from PyQt6.QtWidgets import QFileDialog
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

import kinematics
from motion_profile import SCurveProfile 

# 關節專屬極限 (J1~J6)
MAX_JOINT_SPEEDS = np.array([158.20, 50.63, 55.94, 253.13, 253.13, 101.25])     # 各軸最高轉速 (度/秒)
MAX_JOINT_ACCELS = np.array([527.34, 168.75, 186.46, 843.75, 843.75, 337.50])   # 各軸最高加速度 (度/秒^2)
MAX_JOINT_JERKS = MAX_JOINT_ACCELS * 5.0                               # 各軸最高加加速度 (Jerk, 度/秒^3)

# 直角空間極限 (LIN/CIRC 用)
MAX_LIN_SPEED = 100.0    # TCP 直線極速 (mm/秒) 
MAX_LIN_ACCEL = 200.0    # TCP 直線加速度 (mm/秒^2)
MAX_LIN_JERK = MAX_LIN_ACCEL * 5.0    # TCP 直線加加速度 (mm/秒^3)

MAX_ROT_SPEED = 90.0     # TCP 旋轉極速 (度/秒)
MAX_ROT_ACCEL = 180.0     # TCP 旋轉加速度 (度/秒^2)
MAX_ROT_JERK = MAX_ROT_ACCEL * 5.0     # TCP 旋轉加加速度 (度/秒^3)

# 晶片總體算力防護網參數
MAX_TOTAL_PULSE_SLICE = 10000   # 切片模式極限 (CPU 負載重，邊跑邊解碼)
MAX_TOTAL_PULSE_NATIVE = 65000  # 原生模式極限 (CPU 負載極輕，專注發射脈衝)

# N_PTP 預設起步時間
N_PTP_T_ACC = 0.2

# 步數換算：定義減速比，計算所有軸的 (微步數 * 減速比) / 360度
GEAR_RATIOS = np.array([6.4, 20.0, 18.1, 4.0, 4.0, 10.0])
STEPS_PER_DEG = (1600.0 * GEAR_RATIOS) / 360.0

# 對應 C++ JOINTS 陣列裡 mode=2 的專屬硬體極速 (max_spd)
HW_MAX_STEPS = np.array([65000, 65000, 65000, 65000, 65000, 65000])

# 更新與通訊函數
def update_advanced_settings(lin_spd, lin_acc, rot_spd, rot_acc, slice_pulse, native_pulse, hw_max, t_acc):
    global MAX_LIN_SPEED, MAX_LIN_ACCEL, MAX_LIN_JERK
    global MAX_ROT_SPEED, MAX_ROT_ACCEL, MAX_ROT_JERK
    global MAX_TOTAL_PULSE_SLICE, MAX_TOTAL_PULSE_NATIVE
    global HW_MAX_STEPS, N_PTP_T_ACC
    
    # 1. 更新 Python 大腦裡的速限
    MAX_LIN_SPEED = lin_spd
    MAX_LIN_ACCEL = lin_acc
    MAX_LIN_JERK = lin_acc * 5.0
    
    MAX_ROT_SPEED = rot_spd
    MAX_ROT_ACCEL = rot_acc
    MAX_ROT_JERK = rot_acc * 5.0
    
    MAX_TOTAL_PULSE_SLICE = slice_pulse
    MAX_TOTAL_PULSE_NATIVE = native_pulse
    HW_MAX_STEPS = np.array([hw_max]*6, dtype=float)
    N_PTP_T_ACC = t_acc
    
    print(f">> [Config] Advanced Settings Updated. Native Hz: {native_pulse}, T_acc: {t_acc}s")

# 讓 PathManager 可以發送設定給 MCU
def send_config_to_mcu(serial_manager):
    if serial_manager and serial_manager.is_connected:
        # 封包格式: <SET_CFG, HW_MAX_STEPS, T_ACC>
        cmd = f"<SET_CFG,{int(HW_MAX_STEPS[0])},{N_PTP_T_ACC}>"
        serial_manager.send_command(cmd)
        print(f">> [Config] Sent to MCU: {cmd}")

# --- 1. PTP 執行器 (軟體切片 S-Curve) ---
class PTPExecutor(QThread):
    update_signal = pyqtSignal(list)
    finished_signal = pyqtSignal()
    log_signal = pyqtSignal(str)  
    error_signal = pyqtSignal(str) 
    ready_signal = pyqtSignal()
    
    def __init__(self, start_joints, end_joints, serial_ref=None, speed_factor=1.0, accel_factor=1.0, blend_str="FINE", next_joints=None):
        super().__init__()
        self.start_joints = np.array(start_joints)
        self.end_joints = np.array(end_joints)
        self.serial_ref = serial_ref
        self.speed_factor = speed_factor
        self.accel_factor = accel_factor 
        self._is_running = True 
        self.blend_str = blend_str
        self.next_joints = np.array(next_joints) if next_joints is not None else None
        self.execute_event = threading.Event() # 新增開火許可證

    def run(self):
        # 防呆：一開始就把接力棒設好，就算提早結束也不會閃退
        self.actual_end_joints = self.end_joints
        self.actual_end_velocity = 0.0
        
        diffs = np.abs(self.end_joints - self.start_joints)
        
        if np.max(diffs) < 0.1:
            self.ready_signal.emit() # 提早結束也要通知大腦
            self.execute_event.wait()
            self.finished_signal.emit()
            return
            
        # 1. 找出瓶頸軸 (個體物理極限防護)
        bottleneck_profile = None
        bottleneck_idx = -1
        max_duration = 0.0
        base_allowed_v, base_allowed_a, base_allowed_j = 0.0, 0.0, 0.0
        
        for i in range(6):
            if diffs[i] > 1e-6:
                mech_limit_v = MAX_JOINT_SPEEDS[i] * self.speed_factor
                soft_limit_v = (HW_MAX_STEPS[i] * self.speed_factor) / STEPS_PER_DEG[i]
                allowed_v = min(mech_limit_v, soft_limit_v)
                allowed_a = MAX_JOINT_ACCELS[i] * self.accel_factor
                allowed_j = MAX_JOINT_JERKS[i] * self.accel_factor
                
                p = SCurveProfile(diffs[i], allowed_v, allowed_a, allowed_j)
                if p.T_total > max_duration:
                    max_duration = p.T_total
                    bottleneck_profile = p
                    bottleneck_idx = i
                    base_allowed_v, base_allowed_a, base_allowed_j = allowed_v, allowed_a, allowed_j
                    
        if bottleneck_profile is None:
            self.ready_signal.emit()
            self.execute_event.wait()
            self.finished_signal.emit()
            return

        # 2. 晶片算力防護網
        if bottleneck_profile.T_total > 0:
            peak_progress_rate = base_allowed_v / diffs[bottleneck_idx]
            peak_pulse_freq = np.sum(diffs * peak_progress_rate * STEPS_PER_DEG)
                
            if peak_pulse_freq > MAX_TOTAL_PULSE_SLICE:
                overspeed_ratio = peak_pulse_freq / MAX_TOTAL_PULSE_SLICE
                new_v = base_allowed_v / overspeed_ratio
                new_a = base_allowed_a / (overspeed_ratio ** 2)
                new_j = base_allowed_j / (overspeed_ratio ** 3)
                bottleneck_profile = SCurveProfile(diffs[bottleneck_idx], new_v, new_a, new_j)
                self.log_signal.emit(f"[PTP] Overload ({peak_pulse_freq:.0f}Hz)！Scale down to {(1.0/overspeed_ratio):.2f}X")
        
        # 通知大腦算完了，準備發射
        self.ready_signal.emit()
        self.execute_event.wait()
        if not self._is_running: return

        interval = 0.015
        counter = 0
        gui_skip_frames = 10
 
        # 核心修復：精確時間陣列，保證最後一個點剛好等於 T_total (解決扭動與暴衝)
        t_steps = np.arange(0, bottleneck_profile.T_total, interval).tolist()
        if not t_steps or t_steps[-1] < bottleneck_profile.T_total:
            t_steps.append(bottleneck_profile.T_total)

        for t in t_steps:
            if not self._is_running: return  

            progress = bottleneck_profile.get_progress(t)
            current = self.start_joints + (self.end_joints - self.start_joints) * progress
            
            if counter % gui_skip_frames == 0:
                self.update_signal.emit(list(current))
                
            if self.serial_ref and self.serial_ref.is_connected:
                self.serial_ref.send_joints(list(current), interval, move_mode=1)
                self.serial_ref.wait_for_ok(timeout=3.0)
            else:
                # 模擬器專屬：強迫降速，模擬實體手臂真實的物理移動時間 (15ms)
                time.sleep(interval)
                
            counter += 1

        self.update_signal.emit(list(self.end_joints))
        self.finished_signal.emit()

# --- 獨立延遲執行緒 (統一架構用) ---
class DelayExecutor(QThread):
    ready_signal = pyqtSignal()
    update_signal = pyqtSignal(list)
    finished_signal = pyqtSignal()
    log_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)
    
    def __init__(self, delay_time, current_joints):
        super().__init__()
        self.delay_time = delay_time
        self.actual_end_joints = current_joints
        self.actual_end_velocity = 0.0
        self.execute_event = threading.Event()
        self._is_running = True
        
    def run(self):
        self.ready_signal.emit() # 秒算完！
        self.execute_event.wait() # 等待大腦下令開火
        if not self._is_running: return
        
        self.log_signal.emit(f"  -> Waiting {self.delay_time}s...")
        time.sleep(self.delay_time)
        self.finished_signal.emit()

# --- 2. 笛卡爾空間執行器 (處理 LIN 與 CIRC，搭載 Dry-Run 預掃描引擎) ---
class CartesianExecutor(QThread):
    update_signal = pyqtSignal(list)
    finished_signal = pyqtSignal()
    error_signal = pyqtSignal(str)
    log_signal = pyqtSignal(str)
    ready_signal = pyqtSignal()

    def __init__(self, start_joints, target_joints, tcp_offset_mat, serial_ref=None, speed_factor=1.0, accel_factor=1.0, move_type="LIN", aux_joints=None, blend_str="FINE", next_joints=None, v_start=0.0): # 加在最後面
        super().__init__()
        self.start_joints = np.array(start_joints)
        self.target_joints = np.array(target_joints)
        self.tcp_offset_mat = tcp_offset_mat if tcp_offset_mat is not None else np.eye(4)
        self.speed_factor = speed_factor
        self.accel_factor = accel_factor
        self.serial_ref = serial_ref
        self.move_type = move_type
        self.aux_joints = np.array(aux_joints) if aux_joints is not None else None
        self._is_running = True 
        self.blend_str = blend_str
        self.next_joints = np.array(next_joints) if next_joints is not None else None
        self.v_start = v_start  # 存下初速接力棒
        self.execute_event = threading.Event() # 新增開火許可證

    def run(self):
        try:
            tcp_inv = np.linalg.inv(self.tcp_offset_mat)
        except:
            self.error_signal.emit("Invalid TCP Matrix")
            return

        T_flange_start = kinematics.forward_kinematics(self.start_joints)
        T_tcp_start = T_flange_start @ self.tcp_offset_mat
        T_flange_end = kinematics.forward_kinematics(self.target_joints)
        T_tcp_end = T_flange_end @ self.tcp_offset_mat
        
        pos_start = T_tcp_start[:3, 3]
        pos_end = T_tcp_end[:3, 3]
        
        # 姿態旋轉預備 (四元數 Slerp)
        key_rots = R.from_matrix([T_tcp_start[:3, :3], T_tcp_end[:3, :3]])
        slerp = Slerp([0, 1], key_rots)
        rot_diff = key_rots[0].inv() * key_rots[1]
        dist_deg = np.linalg.norm(rot_diff.as_rotvec()) * (180.0 / math.pi)

        # --- CIRC 圓弧數學引擎 ---
        is_circ = False
        dist_mm = 0.0

        if self.move_type == "CIRC" and self.aux_joints is not None:
            T_flange_aux = kinematics.forward_kinematics(self.aux_joints)
            T_tcp_aux = T_flange_aux @ self.tcp_offset_mat
            pos_aux = T_tcp_aux[:3, 3]

            u = pos_aux - pos_start
            w = pos_end - pos_start
            cross_uw = np.cross(u, w)
            cross_norm = np.linalg.norm(cross_uw)

            if cross_norm > 1e-6:
                is_circ = True
                u2, w2 = np.dot(u, u), np.dot(w, w)
                C = pos_start + np.cross((u2 * w - w2 * u), cross_uw) / (2.0 * cross_norm**2)
                r = np.linalg.norm(pos_start - C)
                
                n = cross_uw / cross_norm
                x_axis = (pos_start - C) / r
                y_axis = np.cross(n, x_axis)
                
                theta_e = math.atan2(np.dot(pos_end - C, y_axis), np.dot(pos_end - C, x_axis))
                if theta_e < 0: theta_e += 2 * math.pi
                
                dist_mm = r * theta_e * 1000.0
            else:
                print("[Warning] Points are collinear. Falling back to LIN.")
                dist_mm = np.linalg.norm(pos_end - pos_start) * 1000.0
        else:
            dist_mm = np.linalg.norm(pos_end - pos_start) * 1000.0

        # ==========================================
        # 新增：幾何引擎 (計算直線與圓角複合長度)
        # ==========================================
        self.is_blending = False
        self.total_dist = dist_mm  # 預設為純直線長度 (mm)
        
        if self.blend_str != "FINE" and self.next_joints is not None and not is_circ:
            T_flange_next = kinematics.forward_kinematics(self.next_joints)
            pos_next = (T_flange_next @ self.tcp_offset_mat)[:3, 3]
            self.pos_next = pos_next # 存起來，等一下算夾角要用
            
            try:
                blend_ratio = float(self.blend_str.replace('%', '')) / 100.0
            except ValueError:
                blend_ratio = 0.0
                
            if blend_ratio > 0:
                dist_in = np.linalg.norm(pos_end - pos_start) * 1000.0
                dist_out = np.linalg.norm(pos_next - pos_end) * 1000.0
                
                if dist_in > 1.0 and dist_out > 1.0:
                    self.is_blending = True
                    r_blend = min(dist_in, dist_out) * blend_ratio
                    self.r_blend = r_blend # 存起融合半徑 (mm)，等一下算向心力要用
                    
                    # 算出 mm 單位的錨點座標 (為了與 pos 單位 meter 匹配，需除以 1000)
                    r_blend_m = r_blend / 1000.0
                    self.p_blend_start = pos_end - ((pos_end - pos_start) / (dist_in/1000.0)) * r_blend_m
                    self.p_blend_end = pos_end + ((pos_next - pos_end) / (dist_out/1000.0)) * r_blend_m
                    self.dist_straight = dist_in - r_blend
                    
                    # 近似貝茲曲線的弧長 (mm)
                    self.curve_len = 0.0
                    prev_pt = self.p_blend_start
                    for i in range(1, 11):
                        t_b = i / 10.0
                        pt = ((1-t_b)**2)*self.p_blend_start + 2*(1-t_b)*t_b*pos_end + (t_b**2)*self.p_blend_end
                        self.curve_len += np.linalg.norm(pt - prev_pt) * 1000.0
                        prev_pt = pt
                        
                    # 覆蓋總長度，欺騙後面的 S-Curve 引擎
                    self.total_dist = self.dist_straight + self.curve_len
                    dist_mm = self.total_dist
            
        if dist_mm < 0.1 and dist_deg < 0.1:
            self.ready_signal.emit() # 提早結束也要通知大腦
            self.execute_event.wait()
            self.finished_signal.emit()
            return
            
        # --- 基礎極限設定 ---
        time_for_lin = dist_mm / (MAX_LIN_SPEED * self.speed_factor) if MAX_LIN_SPEED > 0 else 0
        time_for_rot = dist_deg / (MAX_ROT_SPEED * self.speed_factor) if MAX_ROT_SPEED > 0 else 0

        if time_for_lin >= time_for_rot:
            dist_main = dist_mm
            target_speed = MAX_LIN_SPEED * self.speed_factor
            target_accel = MAX_LIN_ACCEL * self.accel_factor
            target_jerk = MAX_LIN_JERK * self.accel_factor
        else:
            dist_main = dist_deg
            target_speed = MAX_ROT_SPEED * self.speed_factor
            target_accel = MAX_ROT_ACCEL * self.accel_factor
            target_jerk = MAX_ROT_JERK * self.accel_factor

        base_profile = SCurveProfile(dist_main, target_speed, target_accel, target_jerk)

        # 核心進化：Dry-Run 軌跡預掃描 (找出空間隱藏的超速奇異點)
        sample_steps = max(100, int(dist_main * 1.5))
        progress_samples = np.linspace(0, 1.0, sample_steps)
        delta_progress = 1.0 / (sample_steps - 1)
        
        trajectory_joints = []
        current_seed = self.start_joints.copy()
        
        for p in progress_samples:
            # 修改：動態空間映射 (支援直線、圓弧與貝茲圓角)
            if is_circ:
                theta = p * theta_e
                curr_pos = C + r * math.cos(theta) * x_axis + r * math.sin(theta) * y_axis
            elif not getattr(self, 'is_blending', False):
                curr_pos = pos_start + (pos_end - pos_start) * p
            else:
                # 融合模式座標計算
                s_current = p * self.total_dist
                if s_current <= self.dist_straight:
                    prog_straight = s_current / self.dist_straight if self.dist_straight > 0 else 1.0
                    curr_pos = pos_start + prog_straight * (self.p_blend_start - pos_start)
                else:
                    s_curve = s_current - self.dist_straight
                    t_b = s_curve / self.curve_len if self.curve_len > 0 else 1.0
                    if t_b > 1.0: t_b = 1.0
                    curr_pos = ((1-t_b)**2)*self.p_blend_start + 2*(1-t_b)*t_b*pos_end + (t_b**2)*self.p_blend_end
                
            curr_rot = slerp([p]).as_matrix()[0]
            T_tcp_target = np.eye(4)
            T_tcp_target[:3, :3] = curr_rot
            T_tcp_target[:3, 3] = curr_pos
            T_flange_target = T_tcp_target @ tcp_inv
            
            ik_result, error = kinematics.inverse_kinematics(T_flange_target, current_seed)
            if ik_result is None:
                self.error_signal.emit(f"防撞系統啟動：軌跡 {p*100:.0f}% 處 IK 無解！動作已取消。")
                return
                
            trajectory_joints.append(ik_result)
            current_seed = ik_result
            
        trajectory_joints = np.array(trajectory_joints)
        
        # 特效藥 1：加入 Savitzky-Golay 濾波器，消除微積分產生的「幽靈加速度」
        # 設定窗格長度 (必須是奇數)
        window_len = 11 if len(trajectory_joints) >= 11 else (len(trajectory_joints) - (1 if len(trajectory_joints)%2==0 else 0))
        
        if window_len >= 5:
            trajectory_joints_smooth = savgol_filter(trajectory_joints, window_len, 3, axis=0)
        else:
            trajectory_joints_smooth = trajectory_joints

        # 用「過濾後的平滑軌跡」來算微積分
        d_joint_dp = np.gradient(trajectory_joints_smooth, delta_progress, axis=0)
        max_d_joint_dp = np.max(np.abs(d_joint_dp), axis=0)
        
        d2_joint_dp2 = np.gradient(d_joint_dp, delta_progress, axis=0)
        if window_len >= 5:
             # 二階微分再濾一次殘餘雜訊
             d2_joint_dp2 = savgol_filter(d2_joint_dp2, window_len, 3, axis=0)
        max_d2_joint_dp2 = np.max(np.abs(d2_joint_dp2), axis=0)

        peak_prog_v = base_profile.v_max / dist_main if dist_main > 0 else 0
        peak_prog_a = base_profile.a_max / dist_main if dist_main > 0 else 0
        
        overspeed_ratio = 1.0
        
        # 1 & 2. 各軸速度與加速度防護
        for i in range(6):
            # --- 速度把關 ---
            peak_joint_v = max_d_joint_dp[i] * peak_prog_v
            allowed_v = min(MAX_JOINT_SPEEDS[i] * self.speed_factor,
                            (HW_MAX_STEPS[i] * self.speed_factor) / STEPS_PER_DEG[i])
            
            if peak_joint_v > allowed_v:
                overspeed_ratio = max(overspeed_ratio, peak_joint_v / allowed_v)

            # --- 加速度把關 (連鎖律) ---
            peak_joint_a = max_d2_joint_dp2[i] * (peak_prog_v ** 2) + max_d_joint_dp[i] * peak_prog_a
            
            # 這裡也要改成 accel_factor
            allowed_a = MAX_JOINT_ACCELS[i] * self.accel_factor * 2.0 
            
            if peak_joint_a > allowed_a:
                overspeed_ratio = max(overspeed_ratio, math.sqrt(peak_joint_a / allowed_a))

        # 3. 整體算力防護
        peak_pulse_freq = np.sum(max_d_joint_dp * peak_prog_v * STEPS_PER_DEG)
        if peak_pulse_freq > MAX_TOTAL_PULSE_SLICE:
            overspeed_ratio = max(overspeed_ratio, peak_pulse_freq / MAX_TOTAL_PULSE_SLICE)

        # 執行降速與統一 Log
        if overspeed_ratio > 1.0:
            target_speed /= overspeed_ratio
            target_accel /= (overspeed_ratio ** 2)
            target_jerk /= (overspeed_ratio ** 3)
            self.log_signal.emit(f"[{self.move_type}] Overload ({peak_pulse_freq:.0f}Hz)！Scale down to {(1.0/overspeed_ratio):.2f}X")
            
        # ========================================================
        # 動力學核心：計算完美過彎末速 (Cornering Velocity)
        # ========================================================
        self.v_end = 0.0
        
        # 只有在符合融合條件時，才允許不煞車過彎
        if getattr(self, 'is_blending', False) and getattr(self, 'r_blend', 0.0) > 0.0 and getattr(self, 'pos_next', None) is not None:
            v_in = pos_end - pos_start
            v_out = self.pos_next - pos_end
            norm_in = np.linalg.norm(v_in)
            norm_out = np.linalg.norm(v_out)
            
            if norm_in > 1e-6 and norm_out > 1e-6:
                # 計算兩個向量的夾角 theta
                cos_theta = np.dot(v_in, v_out) / (norm_in * norm_out)
                cos_theta = np.clip(cos_theta, -1.0, 1.0)
                theta = math.acos(cos_theta)
                
                # 1. 幾何夾角限速 (Junction Deviation): V_max * sin(theta/2)
                corner_ratio = math.sin(theta / 2.0)
                
                # 2. 物理向心力限速: sqrt(A_max * r_blend)
                v_phys_limit = math.sqrt(target_accel * self.r_blend)
                
                # 綜合三者取最小值：我們設的巡航速限 vs 幾何過彎極限 vs 物理不失步極限
                self.v_end = min(target_speed, target_speed * corner_ratio, v_phys_limit)

        # 終極防呆：確保接力棒傳來的初速和剛算出的末速，都不會大於這一段降速後的最高極速
        final_v_start = min(self.v_start, target_speed)
        final_v_end = min(self.v_end, target_speed)

        # 換上我們親自測試過的新引擎！
        final_profile = SCurveProfile(dist_main, target_speed, target_accel, target_jerk, v_start=final_v_start, v_end=final_v_end)
        
        # 將真正的末速存起來，交給 PathManager 的 on_worker_finished 去傳給下一棒！
        self.actual_end_velocity = final_v_end

        # ========================================================
        # 階段一：軌跡全快取預算
        # ========================================================
        interval = 0.015  
        current_seed = self.start_joints.copy()
        exact_trajectory = []

        # 精確時間陣列：保證 100% 抵達終點
        t_steps = np.arange(0, final_profile.T_total, interval).tolist()
        if not t_steps or t_steps[-1] < final_profile.T_total:
            t_steps.append(final_profile.T_total)

        for t in t_steps:
            if not self._is_running: return
            progress = final_profile.get_progress(t)
            
            # 修改：跟上面一樣的動態空間映射
            if is_circ:
                theta = progress * theta_e
                curr_pos = C + r * math.cos(theta) * x_axis + r * math.sin(theta) * y_axis
            elif not getattr(self, 'is_blending', False):
                curr_pos = pos_start + (pos_end - pos_start) * progress
            else:
                s_current = progress * self.total_dist
                if s_current <= self.dist_straight:
                    prog_straight = s_current / self.dist_straight if self.dist_straight > 0 else 1.0
                    curr_pos = pos_start + prog_straight * (self.p_blend_start - pos_start)
                else:
                    s_curve = s_current - self.dist_straight
                    t_b = s_curve / self.curve_len if self.curve_len > 0 else 1.0
                    if t_b > 1.0: t_b = 1.0
                    curr_pos = ((1-t_b)**2)*self.p_blend_start + 2*(1-t_b)*t_b*pos_end + (t_b**2)*self.p_blend_end
                
            curr_rot = slerp([progress]).as_matrix()[0]
            
            T_tcp_target = np.eye(4)
            T_tcp_target[:3, :3] = curr_rot
            T_tcp_target[:3, 3] = curr_pos
            T_flange_target = T_tcp_target @ tcp_inv
            
            ik_result, _ = kinematics.inverse_kinematics(T_flange_target, current_seed)
            current_seed = ik_result
            
            exact_trajectory.append(ik_result)
            t += interval

        # ========================================================
        # 神奇交界點：先算出終點存起來，然後通知大腦卡住！
        # ========================================================
        self.exact_trajectory = exact_trajectory

        # 提早把真正的終點算出來
        if getattr(self, 'is_blending', False):
            self.actual_end_joints = exact_trajectory[-1]
        else:
            self.actual_end_joints = self.target_joints
            
        # 將真正的末速存起來，交給 PathManager 去傳給下一棒
        self.actual_end_velocity = final_v_end

        self.ready_signal.emit() # 通知大腦算完了！
        self.execute_event.wait() # 卡住，等待大腦下令開火！

        if not self._is_running: return

        # ========================================================
        # 階段二：無腦極速發射 (Zero-Compute Execution)
        # ========================================================
        counter = 0
        gui_skip_frames = 10
        self.update_signal.emit(list(self.start_joints))

        for ik_joints in self.exact_trajectory:
            if not self._is_running: return  

            # 更新 3D 畫面
            if counter % gui_skip_frames == 0:
                self.update_signal.emit(list(ik_joints))
                
            # 極速發射封包給 Arduino
            if self.serial_ref and self.serial_ref.is_connected:
                self.serial_ref.send_joints(list(ik_joints), interval, move_mode=1)
                self.serial_ref.wait_for_ok(timeout=3.0)
            else:
                # 模擬器專屬：強迫降速，模擬實體手臂真實的物理移動時間
                time.sleep(interval)
                
            counter += 1
        
        # ========================================================
        # 結尾：精準錨定與動畫收尾
        # ========================================================
        self.update_signal.emit(list(self.actual_end_joints))
        
        if hasattr(self.serial_ref, 'wait_for_motion_complete'):
            self.serial_ref.wait_for_motion_complete(timeout=10.0)

        self.finished_signal.emit()

# --- 3. 原生 PTP 執行器 (零切片，硬體直驅 + 虛擬動畫) ---
class NativePTPExecutor(QThread):
    update_signal = pyqtSignal(list)
    finished_signal = pyqtSignal()
    error_signal = pyqtSignal(str)
    log_signal = pyqtSignal(str)
    ready_signal = pyqtSignal()

    def __init__(self, start_joints, target_joints, serial_ref=None, speed_factor=1.0, accel_factor=1.0, blend_str="FINE", next_joints=None):
        super().__init__()
        self.start_joints = np.array(start_joints)
        self.target_joints = np.array(target_joints)
        self.serial_ref = serial_ref
        self.speed_factor = speed_factor
        self.accel_factor = accel_factor # 備用，等待下一階段與 C++ 對接
        self._is_running = True
        self.blend_str = blend_str
        self.next_joints = np.array(next_joints) if next_joints is not None else None

    def run(self):
        if self.serial_ref and self.serial_ref.is_connected:
            
            # 算力防護網：預判 C++ 原生引擎的運算結果
            diffs = np.abs(self.target_joints - self.start_joints)
            delta_steps = diffs * STEPS_PER_DEG
            max_time = 0.0
            
            for i in range(6):
                v_max = HW_MAX_STEPS[i] * self.speed_factor
                if v_max > 0 and delta_steps[i] > 0:
                    t_needed = delta_steps[i] / v_max
                    if t_needed > max_time:
                        max_time = t_needed
                        
            if max_time > 0:
                total_freq = np.sum(delta_steps / max_time)
                if total_freq > MAX_TOTAL_PULSE_NATIVE:
                    overspeed_ratio = total_freq / MAX_TOTAL_PULSE_NATIVE
                    self.speed_factor /= overspeed_ratio
                    self.log_signal.emit(f"[PTP Native] Overload ({total_freq:.0f}Hz)！Scale down to {(1.0/overspeed_ratio):.2f}X")

            self.ready_signal.emit()
            self.execute_event.wait()
            if not self._is_running: return

            # 1. 發射指令，讓 C++ 去爆發！
            self.serial_ref.send_joints(list(self.target_joints), self.speed_factor, move_mode=2)
            
            if not self.serial_ref.wait_for_ok(timeout=3.0):
                self.error_signal.emit("Timeout: Arduino did not ack Native PTP.")
                return
            
            # 2. 幽靈手臂引擎 (虛擬動畫)
            max_diff = np.max(diffs)
            
            # 解除動畫限速，匹配硬體的高極速
            base_speed = 300.0 * self.speed_factor  
            if base_speed < 1.0: base_speed = 1.0
            
            estimated_time = (max_diff / base_speed) + 0.05
            
            steps = int(estimated_time / 0.05)
            if steps < 1: steps = 1
            
            for i in range(steps):
                if not self._is_running: return  

                progress = (i + 1) / steps
                current_j = self.start_joints + (self.target_joints - self.start_joints) * progress
                self.update_signal.emit(list(current_j))
                time.sleep(0.03) 
            
            # 動畫畫完，確保精準對齊終點
            self.update_signal.emit(list(self.target_joints))
            
            # 3. 等待實體手臂真正跑完回傳 "Done"
            if hasattr(self.serial_ref, 'wait_for_motion_complete'):
                self.serial_ref.wait_for_motion_complete(timeout=60.0)
                
        self.finished_signal.emit()
        

# --- PathManager (邏輯核心) ---
class PathManager(QObject):
    log_signal = pyqtSignal(str)
    joint_update_signal = pyqtSignal(list)
    list_update_signal = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.waypoints = []
        self.execution_queue = [] 
        self.worker = None
        self.path_index = 0
        self.parent_widget = parent
        
        self.is_looping = False
        self.global_speed = 1.0 
        self.current_tcp_offset = np.eye(4)
        # 1. 加入這行：防止幽靈計時器作祟的總開關
        self._is_path_active = False
        
        # 暫存區：用來裝載 CIRC 的中繼點
        self.temp_aux_joints = None 
        
        self.serial_manager = None
        if hasattr(parent, 'serial_manager'):
            self.serial_manager = parent.serial_manager

    # 設定中繼點
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
            "blend": "FINE",   # 新增這行：預設為最安全的精準到位模式
            "speed": float(speed), 
            "accel": float(accel), 
            "active": True,
            "note": ""
        }
        self.waypoints.append(data)
        self.list_update_signal.emit()
        
        msg = f"Recorded: {name} [{move_type}]"
        if move_type == "CIRC":
            msg += " (with AUX point)"
        self.log_signal.emit(msg)

    # 新增獨立封包生成方法
    def record_delay(self, time_sec=2.0):
        idx = len(self.waypoints) + 1
        name = f"Wait {time_sec}s"
        data = {
            "name": name,
            "type": "DELAY",
            "value": float(time_sec), # 延遲時間存在 value 裡
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
                    # 1. 確保所有舊點位都有新的 accel 參數
                    if 'accel' not in pt and pt.get('type') != 'DELAY':
                        pt['accel'] = 50.0
                    # 補齊舊版的 blend
                    if 'blend' not in pt:
                        pt['blend'] = "FINE"
                    # 2. 強制清洗 joints，限制為 4 位小數
                    if 'joints' in pt and pt['joints'] is not None:
                        # 加上 float() 是為了確保 numpy 資料型態能被乾淨轉換
                        pt['joints'] = [round(float(j), 4) for j in pt['joints']]
                    
                    # 3. 如果有 CIRC 的輔助點，一併清洗
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
                self.list_update_signal.emit()
                self.log_signal.emit(f"Path loaded from {filename}")
            except Exception as e:
                self.log_signal.emit(f"[Error] Load failed: {e}")

    def get_trajectory_preview(self, tcp_offset):
        trajectory_points = []
        active_wps = [pt for pt in self.waypoints if pt.get('active', True) and pt.get('type') != 'DELAY' and 'joints' in pt]
        
        if len(active_wps) < 2:
            return trajectory_points

        from kinematics import forward_kinematics

        # 預先收集座標
        cartesian_points = []
        joint_points = []
        for wp in active_wps:
            j_arr = np.array(wp['joints'])
            joint_points.append(j_arr)
            T = forward_kinematics(j_arr)
            cartesian_points.append((T @ tcp_offset)[:3, 3])
            
        # ==========================================
        # 1. 計算每個點的「融合半徑 (Blend Radius)」
        # ==========================================
        blend_radii = [0.0] * len(active_wps)
        for i in range(1, len(active_wps) - 1):
            blend_str = active_wps[i].get('blend', 'FINE')
            if blend_str == "FINE" or not blend_str.endswith('%'):
                continue
            
            try:
                blend_ratio = float(blend_str.replace('%', '')) / 100.0
            except ValueError:
                blend_ratio = 0.0
                
            if blend_ratio > 0:
                blend_ratio = min(blend_ratio, 1.0)
                v_in = cartesian_points[i-1] - cartesian_points[i]
                v_out = cartesian_points[i+1] - cartesian_points[i]
                blend_radii[i] = min(np.linalg.norm(v_in), np.linalg.norm(v_out)) * blend_ratio

        # ==========================================
        # 2. 參數化精確繪製 (保證 100% 無縫連接)
        # ==========================================
        for i in range(len(active_wps) - 1):
            p_start = cartesian_points[i]
            p_end = cartesian_points[i+1]
            j_start = joint_points[i]
            j_end = joint_points[i+1]
            move_type = active_wps[i+1].get('type', 'PTP')
            
            r_start = blend_radii[i]
            r_end = blend_radii[i+1]
            
            dist_main = np.linalg.norm(p_end - p_start)
            if dist_main < 1e-3:
                continue
                
            # 核心魔法：直接算出線段「精確起筆與收筆」的比例 (t)
            t_start = r_start / dist_main
            t_end = 1.0 - (r_end / dist_main)
            
            if t_start > t_end:
                t_center = 0.5 * (t_start + t_end)
                t_start = t_end = t_center
                
            # --- A. 繪製主線段 ---
            num_steps = 30
            segment_pts = []
            for step in range(num_steps + 1):
                # t 值被嚴格限制在我們精算的範圍內
                t = t_start + (step / float(num_steps)) * (t_end - t_start)
                if move_type == "PTP":
                    j_interp = j_start + t * (j_end - j_start)
                    pt = (forward_kinematics(j_interp) @ tcp_offset)[:3, 3]
                else: 
                    pt = p_start + t * (p_end - p_start)
                segment_pts.append(pt)
                trajectory_points.append(pt.tolist())
                
            # --- B. 繪製終點的融合圓角 (強制錨定法) ---
            if r_end > 0 and i + 2 < len(active_wps):
                # 1. 錨定起點：絕對使用這條線段的「最後一滴墨水」作為圓角的起點
                blend_start_pt = segment_pts[-1]
                
                # 2. 錨定終點：偷看下一條線的「第一滴墨水」作為圓角的終點
                p_next_next = cartesian_points[i+2]
                j_next_next = joint_points[i+2]
                next_move_type = active_wps[i+2].get('type', 'PTP')
                dist_next = np.linalg.norm(p_next_next - p_end)
                
                if dist_next > 1e-3:
                    t_start_next = r_end / dist_next
                    if next_move_type == "PTP":
                        j_interp_next = j_end + t_start_next * (j_next_next - j_end)
                        blend_end_pt = (forward_kinematics(j_interp_next) @ tcp_offset)[:3, 3]
                    else:
                        blend_end_pt = p_end + t_start_next * (p_next_next - p_end)
                        
                    # 3. 畫出完美的貝茲曲線，強制連結上述兩個錨點！
                    num_blend_steps = 15
                    for step in range(1, num_blend_steps + 1):
                        t_b = step / float(num_blend_steps)
                        p_t = ((1 - t_b) ** 2) * blend_start_pt + (2 * (1 - t_b) * t_b) * p_end + (t_b ** 2) * blend_end_pt
                        trajectory_points.append(p_t.tolist())

        return trajectory_points

    # --- 路徑執行邏輯 (工業級滑動預讀引擎) ---
    def run_path(self, current_joints_start, loop=False, tcp_offset=None):
        active_points = [pt for pt in self.waypoints if pt.get('active', True)]
        if not active_points: return
        
        self.is_looping = loop
        self.execution_queue = active_points
        self.current_tcp_offset = tcp_offset if tcp_offset is not None else np.eye(4)
        self._is_path_active = True 
        
        self.current_v_start = 0.0 
        self.path_index = 0
        
        self.is_hardware_busy = False # 實體手臂狀態燈
        self.active_worker = None     # 正在「發射中」的工人
        self.preloaded_worker = None  # 正在「背景偷算」的工人
        
        self.log_signal.emit(f"([START]) 滑動預讀引擎啟動 (Sliding Window Look-ahead)...")
        self._preload_next(current_joints_start)

    def _preload_next(self, current_joints):
        """指派下一個工人去背景偷算"""
        if not self._is_path_active: return
        
        if self.path_index >= len(self.execution_queue):
            if self.is_looping and not self.is_hardware_busy:
                self.log_signal.emit(">> Looping...")
                self.path_index = 0
                self._preload_next(current_joints)
            return

        target_data = self.execution_queue[self.path_index]
        move_type = target_data.get('type', "PTP")
        name = target_data.get('name', str(self.path_index))
        
        speed_factor = min((target_data.get('speed', 50.0) / 100.0) * self.global_speed, 1.0)
        accel_factor = min((target_data.get('accel', 50.0) / 100.0) * self.global_speed, 1.0)
        
        blend_str = target_data.get('blend', 'FINE')
        next_target_joints = None
        if blend_str != "FINE" and self.path_index + 1 < len(self.execution_queue):
            next_data = self.execution_queue[self.path_index + 1]
            if next_data.get('type') == 'DELAY': blend_str = "FINE"
            else: next_target_joints = next_data.get('joints', None)

        self.log_signal.emit(f"  [Pre-loading] -> {name} ({move_type}, SPD:{speed_factor*100:.0f}%, BLEND:{blend_str})...")
        
        # 建立對應的 Worker (保有原汁原味的串列通訊)
        if move_type == "DELAY":
            worker = DelayExecutor(target_data.get('value', 0.0), current_joints)
        elif move_type in ["LIN", "CIRC"]:
            worker = CartesianExecutor(
                start_joints=current_joints, target_joints=target_data.get('joints', current_joints), 
                tcp_offset_mat=self.current_tcp_offset, serial_ref=self.serial_manager, 
                speed_factor=speed_factor, accel_factor=accel_factor, move_type=move_type, 
                aux_joints=target_data.get('aux_joints', None), blend_str=blend_str, 
                next_joints=next_target_joints, v_start=self.current_v_start
            )
        elif move_type == "N_PTP":
            worker = NativePTPExecutor(start_joints=current_joints, target_joints=target_data.get('joints', current_joints), serial_ref=self.serial_manager, speed_factor=speed_factor, accel_factor=accel_factor)
        else:
            worker = PTPExecutor(start_joints=current_joints, end_joints=target_data.get('joints', current_joints), serial_ref=self.serial_manager, speed_factor=speed_factor, accel_factor=accel_factor)
            
        self.preloaded_worker = worker
        
        worker.error_signal.connect(self._on_worker_error)
        worker.update_signal.connect(self.joint_update_signal.emit)
        worker.log_signal.connect(self.log_signal.emit)
        
        # 核心：連接雙重緩衝訊號
        worker.ready_signal.connect(self._on_worker_ready)
        worker.finished_signal.connect(self._on_worker_finished)
        
        self.path_index += 1
        worker.start() # 啟動！但他算完會自己卡住

    def _on_worker_ready(self):
        """當工人算完 IK 與微積分時觸發"""
        if not self.is_hardware_busy and self.preloaded_worker:
            self._fire_worker()

    def _fire_worker(self):
        """下令開火，開始實體發射"""
        self.is_hardware_busy = True
        worker = self.preloaded_worker
        self.active_worker = worker
        self.preloaded_worker = None
        
        # 核心修復：在準備偷算「下一段」之前，提早把這一段算好的末速拿出來當接力棒！
        self.current_v_start = getattr(worker, 'actual_end_velocity', 0.0)
        
        worker.execute_event.set() # 解除封印，實體開始跑！
        
        # 馬上派下一個工人去背景偷算！(此時他就會拿到正確的 current_v_start 了)
        self._preload_next(getattr(worker, 'actual_end_joints', []))

    def _on_worker_finished(self):
        """實體手臂跑完一段時觸發"""
        self.is_hardware_busy = False
        # 移除原本在這裡拿接力棒的錯誤邏輯
        next_start = getattr(self.active_worker, 'actual_end_joints', [])
        
        # 跑完的瞬間，下一個工人如果已經偷算好，直接 0 毫秒無縫開火！
        if self.preloaded_worker:
            if not self.preloaded_worker.execute_event.is_set():
                if hasattr(self.preloaded_worker, 'actual_end_joints'):
                    self._fire_worker()
                else:
                    self.log_signal.emit("  [Wait] 背景運算中，等待就緒...")
                    
        elif self.path_index >= len(self.execution_queue):
            if self.is_looping:
                self.log_signal.emit(">> Looping...")
                self.path_index = 0
                self._preload_next(next_start)
            else:
                self.log_signal.emit("([END]) Path Completed.")
                self._is_path_active = False

    def _on_worker_error(self, msg):
        self.log_signal.emit(f"[STOP] {msg}")
        self.stop_path()

    def stop_path(self):
        self._is_path_active = False 
        
        # 溫柔且安全地殺死兩個執行緒
        if getattr(self, 'active_worker', None):
            self.active_worker._is_running = False
            if hasattr(self.active_worker, 'execute_event'): self.active_worker.execute_event.set() 
            self.active_worker.wait()
            self.active_worker = None
            
        if getattr(self, 'preloaded_worker', None):
            self.preloaded_worker._is_running = False
            if hasattr(self.preloaded_worker, 'execute_event'): self.preloaded_worker.execute_event.set()
            self.preloaded_worker.wait()
            self.preloaded_worker = None
            
        self.is_hardware_busy = False
        if self.serial_manager and self.serial_manager.is_connected:
            self.serial_manager.send_command("<STOP>")
            self.serial_manager.ok_event.set()
            self.serial_manager.motion_done_event.set()
            
        self.is_looping = False
        self.log_signal.emit("([STOP]) Execution Halted.")

    def set_speed(self, value_0_to_100):
        self.global_speed = max(0.1, value_0_to_100 / 50.0)