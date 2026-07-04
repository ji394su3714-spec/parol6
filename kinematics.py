# kinematics.py
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.signal import savgol_filter

import config
from motion_profile import SCurveProfile 

# ==========================================
# 基礎幾何與旋轉矩陣運算
# ==========================================
def get_tf_matrix(xyz, rpy_rad):
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', rpy_rad, degrees=False).as_matrix()
    T[:3, 3] = xyz
    return T

def get_rotation_matrix(axis, angle_deg):
    return fast_rotation_matrix(axis, angle_deg)

def fast_rotation_matrix(axis, angle_deg):
    """超輕量級旋轉矩陣生成：拔除 scipy 開銷，速度提升百倍"""
    rad = np.deg2rad(angle_deg)
    c, s = np.cos(rad), np.sin(rad)
    if axis == 'z':
        return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    elif axis == 'y':
        return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
    elif axis == 'x':
        return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
    return np.eye(4)

# ==========================================
# 靜態矩陣快取 (Cache)
# ==========================================
T_BASE_FIXED = None
T_FIXED_LIST = []

def init_kinematics_cache():
    """將永遠不會變的偏移量在啟動時先算好，不再重複計算"""
    global T_BASE_FIXED, T_FIXED_LIST
    if T_BASE_FIXED is not None: 
        return 

    base_xyz = [x * config.SCALE_FACTOR for x in config.BASE_MESH_OFFSET['xyz']]
    T_BASE_FIXED = get_tf_matrix(base_xyz, config.BASE_MESH_OFFSET['rpy'])

    for params in config.URDF_PARAMS:
        xyz = [x * config.SCALE_FACTOR for x in params['xyz']]
        T_FIXED_LIST.append(get_tf_matrix(xyz, params['rpy']))

# ==========================================
# 正向運動學 (Forward Kinematics)
# ==========================================
def forward_kinematics(joint_angles):
    init_kinematics_cache()
    T_current = T_BASE_FIXED.copy()

    for i, params in enumerate(config.URDF_PARAMS):
        raw_angle = joint_angles[i]
        angle = -raw_angle if params.get('invert', False) else raw_angle
        
        T_rot = fast_rotation_matrix(params['axis'], angle)
        T_current = T_current @ T_FIXED_LIST[i] @ T_rot
        
    return T_current

def forward_kinematics_all(joint_angles):
    init_kinematics_cache()
    matrices = []
    T_current = T_BASE_FIXED.copy()
    matrices.append(T_current)

    for i, params in enumerate(config.URDF_PARAMS):
        raw_angle = joint_angles[i]
        angle = -raw_angle if params.get('invert', False) else raw_angle
        T_rot = fast_rotation_matrix(params['axis'], angle)
        
        T_current = T_current @ T_FIXED_LIST[i] @ T_rot
        matrices.append(T_current)
        
    return matrices

# ==========================================
# 逆向運動學 (Inverse Kinematics) 與 雅可比矩陣
# ==========================================
def compute_numerical_jacobian(joints, T_current=None):
    """接收已計算好的 T_current，省下 1/7 的 FK 算力"""
    epsilon = 1e-4 
    J = np.zeros((6, 6))
    
    if T_current is None:
        T_current = forward_kinematics(joints)
        
    current_pos = T_current[:3, 3]
    R_curr_T = T_current[:3, :3].T 
    
    for i in range(6):
        perturbed_joints = list(joints)
        perturbed_joints[i] += np.rad2deg(epsilon) 
        
        T_new = forward_kinematics(perturbed_joints)
        new_pos = T_new[:3, 3]
        
        J[:3, i] = (new_pos - current_pos) / epsilon
        
        R_new = T_new[:3, :3]
        R_diff = R_new @ R_curr_T
        rot_vec = R.from_matrix(R_diff).as_rotvec()
        J[3:, i] = rot_vec / epsilon
        
    return J

# 👇 1. 將你原本的 IK 改名為 _core_inverse_kinematics (作為純粹的底層運算核心)
def _core_inverse_kinematics(target_matrix, seed_joints, max_retries=1):
    target_pos = target_matrix[:3, 3]
    target_rot = target_matrix[:3, :3]
    
    max_iter = 50       
    tolerance = 1e-5  
    lambda_val = 0.01   
    lambda_sq_eye = (lambda_val**2) * np.eye(6) 

    for attempt in range(max_retries + 1):
        if attempt > 0:
            noise = np.random.uniform(-5.0, 5.0, 6)
            current_joints = np.array(seed_joints, dtype=float) + noise
        else:
            current_joints = np.array(seed_joints, dtype=float)
            
        for _ in range(max_iter):
            T_curr = forward_kinematics(current_joints)
            curr_pos = T_curr[:3, 3]
            curr_rot = T_curr[:3, :3]
            
            err_pos = target_pos - curr_pos
            R_err = target_rot @ curr_rot.T
            err_rot = R.from_matrix(R_err).as_rotvec()
            
            error_vector = np.concatenate((err_pos, err_rot))
            
            if np.linalg.norm(error_vector) < tolerance:
                return current_joints, np.linalg.norm(error_vector)

            J = compute_numerical_jacobian(current_joints, T_curr)
            
            XtX = J @ J.T + lambda_sq_eye
            y = np.linalg.solve(XtX, error_vector)
            delta_theta = J.T @ y
            
            current_joints += np.rad2deg(delta_theta)
            
            for i in range(6):
                min_lim, max_lim = config.JOINT_LIMITS[i]
                if current_joints[i] < min_lim: current_joints[i] = min_lim
                if current_joints[i] > max_lim: current_joints[i] = max_lim

        final_error = np.linalg.norm(error_vector)
        if final_error < 0.1: 
            return current_joints, final_error
            
    return None, None

# 👇 2. 新增包裝器：同名的 inverse_kinematics (對外接口完全不變)
def inverse_kinematics(target_matrix, seed_joints, max_retries=1):
    """
    【姿態突變終結者】
    智慧偵測目標距離。如果距離過大 (如 Base 轉 45 度)，
    自動在底層使用 SLERP 將路徑切碎，確保關節姿態 100% 完美繼承，避免手腕翻轉！
    """
    T_start = forward_kinematics(seed_joints)
    pos_start = T_start[:3, 3]
    pos_end = target_matrix[:3, 3]
    dist_mm = np.linalg.norm(pos_end - pos_start) * 1000.0
    
    R_start = T_start[:3, :3]
    R_end = target_matrix[:3, :3]
    R_diff = R_start.T @ R_end
    dist_deg = np.rad2deg(np.linalg.norm(R.from_matrix(R_diff).as_rotvec()))
    
    # 動態決定切分步數 (每 20mm 或 10度 切一刀)
    steps = max(1, int(np.ceil(dist_mm / 20.0)), int(np.ceil(dist_deg / 10.0)))
    
    # 💡 效能防護：如果是 LIN 軌跡引擎傳進來的極短距離，直接秒算，0 效能損耗！
    if steps == 1:
        return _core_inverse_kinematics(target_matrix, seed_joints, max_retries)
        
    curr_seed = np.array(seed_joints, dtype=float)
    last_err = 0.0
    
    key_rots = R.from_matrix([R_start, R_end])
    slerp = Slerp([0, 1], key_rots)
    
    # 💡 遇到大距離 (如 Update Position 時)，沿路丟麵包屑引導 IK
    for i in range(1, steps + 1):
        fraction = i / steps
        T_step = np.eye(4)
        T_step[:3, 3] = pos_start + fraction * (pos_end - pos_start)
        T_step[:3, :3] = slerp(fraction).as_matrix()
        
        # 嚴格禁止加噪聲，確保姿態絕對不跳躍
        res, err = _core_inverse_kinematics(T_step, curr_seed, max_retries=0)
        if res is not None:
            curr_seed = res
            last_err = err
        else:
            # 萬一卡死，才允許稍微加點噪聲掙扎
            res, err = _core_inverse_kinematics(T_step, curr_seed, max_retries=1)
            if res is not None:
                curr_seed = res
                last_err = err
            else:
                return None, None
                
    return curr_seed, last_err

# ==========================================
def calculate_base_shift_ik(T_flange_old, recorded_base_mat, target_base_mat, seed_joints):
    """
    【陣列加工神級演算法：基座球面插值】
    專門用於處理 SET_BASE 變更時的座標投影。
    不走笛卡爾直線(切弦)，而是讓整個空間跟隨基座旋轉與平移，
    確保 IK 求解時 100% 繼承母工位的關節姿態 (例如純粹只轉 J1)。
    """
    pos_start = recorded_base_mat[:3, 3]
    pos_end = target_base_mat[:3, 3]
    pos_dist = np.linalg.norm(pos_end - pos_start) * 1000.0
    
    R_start = recorded_base_mat[:3, :3]
    R_end = target_base_mat[:3, :3]
    rot_diff = R_start.T @ R_end
    rot_dist_deg = np.rad2deg(np.linalg.norm(R.from_matrix(rot_diff).as_rotvec()))
    
    # 依據平移與旋轉距離，決定切碎的刀數
    steps = max(1, int(np.ceil(pos_dist / 10.0)), int(np.ceil(rot_dist_deg / 5.0)))
    T_user = np.linalg.inv(recorded_base_mat) @ T_flange_old
    
    if steps == 1:
        return inverse_kinematics(target_base_mat @ T_user, seed_joints)
        
    curr_seed = np.array(seed_joints, dtype=float)
    last_err = 0.0
    
    key_rots = R.from_matrix([R_start, R_end])
    slerp = Slerp([0, 1], key_rots)
    
    for i in range(1, steps + 1):
        fraction = i / steps
        T_interp_base = np.eye(4)
        T_interp_base[:3, 3] = pos_start + fraction * (pos_end - pos_start)
        T_interp_base[:3, :3] = slerp(fraction).as_matrix()
        
        T_target_flange = T_interp_base @ T_user
        
        # 嚴格禁止加噪聲，確保姿態絕對不跳躍
        next_joints, err = inverse_kinematics(T_target_flange, curr_seed, max_retries=0)
        if next_joints is not None:
            curr_seed = next_joints
            last_err = err
        else:
            # 萬一卡死，才允許稍微加點噪聲掙扎
            next_joints, err = inverse_kinematics(T_target_flange, curr_seed, max_retries=1)
            if next_joints is not None:
                curr_seed = next_joints
                last_err = err
            else:
                return None, f"Base Shift Singularity at {int(fraction*100)}%"
                
    return curr_seed, last_err

# ==========================================

def calculate_jog_joints(current_joints, axis, step_val, frame, T_total_offset, T_base_matrix=None):
    if T_base_matrix is None:
        T_base_matrix = np.eye(4)

    T_math_flange = forward_kinematics(current_joints)
    T_tcp_curr = T_math_flange @ T_total_offset
    T_tcp_target = np.copy(T_tcp_curr)
    T_step = np.eye(4)
    
    if axis in ['x', 'y', 'z']:
        step_m = step_val / 1000.0
        idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        
        step_vec_local = np.zeros(3)
        step_vec_local[idx] = step_m
        
        if frame == "Tool":
            T_step[idx, 3] = step_m
            T_tcp_target = T_tcp_curr @ T_step 
        elif frame == "Base":
            step_vec_world = T_base_matrix[:3, :3] @ step_vec_local
            T_tcp_target[:3, 3] += step_vec_world
        else: # World
            T_tcp_target[idx, 3] += step_m     
            
    else: # 旋轉 (Rx, Ry, Rz)
        step_rad = np.deg2rad(step_val)
        vec = np.zeros(3)
        rot_idx = {'x': 0, 'y': 1, 'z': 2}[axis[1]] 
        vec[rot_idx] = step_rad
        R_step = R.from_rotvec(vec).as_matrix()
        
        if frame == "Tool":
            T_step[:3, :3] = R_step
            T_tcp_target = T_tcp_curr @ T_step 
        elif frame == "Base":
            R_step_world = T_base_matrix[:3, :3] @ R_step @ T_base_matrix[:3, :3].T
            T_tcp_target[:3, :3] = R_step_world @ T_tcp_curr[:3, :3]
        else: # World
            T_tcp_target[:3, :3] = R_step @ T_tcp_curr[:3, :3]

    T_flange_target = T_tcp_target @ np.linalg.inv(T_total_offset)
    new_joints, error_score = inverse_kinematics(T_flange_target, current_joints)
    
    if new_joints is None:
        return None, "IK Failed"
        
    T_check_flange = forward_kinematics(new_joints)
    pos_diff = np.linalg.norm(T_check_flange[:3, 3] - T_flange_target[:3, 3]) * 1000.0

    if pos_diff > config.IK_POS_TOLERANCE:
        return None, f"IK Inaccurate! Diff: {pos_diff:.3f}mm"

    for i, angle in enumerate(new_joints):
        min_lim, max_lim = config.JOINT_LIMITS[i]
        if angle < (min_lim - 0.1) or angle > (max_lim + 0.1):
            return None, f"Limit Hit J{i+1}"

    return list(new_joints), None

# ==========================================
# 尤拉角濾波器與基座座標轉換工具
# ==========================================
def extract_continuous_rpy(T_matrix, prev_rpy_deg=None):
    """終極尤拉角連續化濾波器 (包含萬向鎖防護 + 孿生解平滑 + 強制收束)"""
    r = R.from_matrix(T_matrix[:3, :3])
    e1 = r.as_euler('xyz', degrees=True)

    if prev_rpy_deg is None:
        return (e1 + 180.0) % 360.0 - 180.0

    if abs(abs(e1[1]) - 90.0) < 0.1:
        prev_yaw = prev_rpy_deg[2]
        for sign in [1, -1]:
            const_val = e1[0] + sign * e1[2]
            test_roll = const_val - sign * prev_yaw
            
            test_rpy = [test_roll, e1[1], prev_yaw]
            test_matrix = R.from_euler('xyz', test_rpy, degrees=True).as_matrix()
            
            if np.allclose(test_matrix, T_matrix[:3, :3], atol=1e-3):
                e1 = np.array(test_rpy)
                break

    e2 = np.array([e1[0] + 180.0, 180.0 - e1[1], e1[2] + 180.0])

    def unwrap_to_nearest(target, ref):
        diff = (target - ref) % 360.0
        diff = np.where(diff > 180.0, diff - 360.0, diff)
        return ref + diff

    e1_unwrapped = unwrap_to_nearest(e1, prev_rpy_deg)
    e2_unwrapped = unwrap_to_nearest(e2, prev_rpy_deg)

    dist1 = np.linalg.norm(e1_unwrapped - prev_rpy_deg)
    dist2 = np.linalg.norm(e2_unwrapped - prev_rpy_deg)

    best_rpy = e1_unwrapped if dist1 < dist2 else e2_unwrapped
    normalized_rpy = (best_rpy + 180.0) % 360.0 - 180.0
    
    return normalized_rpy

def apply_base_frame(T_user_target, T_base_matrix):
    if T_base_matrix is None:
        return T_user_target
    return T_base_matrix @ T_user_target

def remove_base_frame(T_world_current, T_base_matrix):
    if T_base_matrix is None:
        return T_world_current
    return np.linalg.inv(T_base_matrix) @ T_world_current

# ==========================================
# 純粹數學兵工廠 (Trajectory Math Engine)
# ==========================================
class TrajectoryMathEngine:
    
    @staticmethod
    def calculate_lin_trajectory(start_joints, target_joints, tcp_offset_mat, speed_factor, accel_factor=1.0):
        """LIN 直線空間向量化引擎"""
        try:
            tcp_inv = np.linalg.inv(tcp_offset_mat)
        except np.linalg.LinAlgError:
            return None, 0, "Invalid TCP Matrix"
    
        T_flange_start = forward_kinematics(start_joints)
        T_tcp_start = T_flange_start @ tcp_offset_mat
        T_flange_end = forward_kinematics(target_joints)
        T_tcp_end = T_flange_end @ tcp_offset_mat
        
        pos_start, pos_end = T_tcp_start[:3, 3], T_tcp_end[:3, 3]
        key_rots = R.from_matrix([T_tcp_start[:3, :3], T_tcp_end[:3, :3]])
        slerp = Slerp([0, 1], key_rots)
        rot_diff = key_rots[0].inv() * key_rots[1]
        dist_deg = np.linalg.norm(rot_diff.as_rotvec()) * (180.0 / math.pi)
        dist_mm = np.linalg.norm(pos_end - pos_start) * 1000.0
            
        if dist_mm < 0.1 and dist_deg < 0.1:
            return [start_joints], 0, "SUCCESS"

        time_for_lin = dist_mm / (config.MAX_LIN_SPEED * speed_factor) if config.MAX_LIN_SPEED > 0 else 0
        time_for_rot = dist_deg / (config.MAX_ROT_SPEED * speed_factor) if config.MAX_ROT_SPEED > 0 else 0

        if time_for_lin >= time_for_rot:
            dist_main = dist_mm
            target_speed = config.MAX_LIN_SPEED * speed_factor
            target_accel = config.MAX_LIN_ACCEL * accel_factor
            target_jerk = config.MAX_LIN_JERK * accel_factor
        else:
            dist_main = dist_deg
            target_speed = config.MAX_ROT_SPEED * speed_factor
            target_accel = config.MAX_ROT_ACCEL * accel_factor
            target_jerk = config.MAX_ROT_JERK * accel_factor

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

            ik_result, error = inverse_kinematics(T_flange_targets[i], current_seed)
            if ik_result is None:
                return None, 0, f"Dry-Run failed at {progress_samples[i]*100:.0f}%: No IK solution found! Action canceled."
            trajectory_joints.append(ik_result)
            current_seed = ik_result

        trajectory_joints = np.array(trajectory_joints) 

        max_safe_window = max(5, int(len(trajectory_joints) / 3))
        if max_safe_window % 2 == 0: max_safe_window += 1 
        window_len = min(15, max_safe_window) 

        trajectory_joints_smooth = savgol_filter(trajectory_joints, window_len, 3, axis=0) if len(trajectory_joints) >= 5 else trajectory_joints    

        d_joint_dp = np.gradient(trajectory_joints_smooth, delta_progress, axis=0)
        d2_joint_dp2 = np.gradient(d_joint_dp, delta_progress, axis=0)
        if len(trajectory_joints) >= 5: 
            d2_joint_dp2 = savgol_filter(d2_joint_dp2, window_len, 3, axis=0)

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

        v_overspeed_ratio = 1.0
        a_overspeed_ratio = 1.0

        for i in range(6):
            peak_joint_v = max_d_joint_dp[i] * peak_prog_v  
            allowed_v = config.MAX_JOINT_SPEEDS[i] 
            if peak_joint_v > allowed_v and allowed_v > 0:
                v_overspeed_ratio = max(v_overspeed_ratio, peak_joint_v / allowed_v)

            peak_joint_a_cruise = max_d2_joint_dp2[i] * (peak_prog_v ** 2) 
            peak_joint_a_accel = max_d_joint_dp[i] * peak_prog_a
            peak_joint_a = max(peak_joint_a_cruise, peak_joint_a_accel)
            
            allowed_a = config.MAX_JOINT_ACCELS[i] 
            if peak_joint_a > allowed_a and allowed_a > 0:
                a_overspeed_ratio = max(a_overspeed_ratio, math.sqrt(peak_joint_a / allowed_a))
                
        instant_pulse_freqs = np.sum(np.abs(d_joint_dp_core) * peak_prog_v * config.STEPS_PER_DEG, axis=1)
        peak_pulse_freq = np.max(instant_pulse_freqs) if len(instant_pulse_freqs) > 0 else 0
        
        pulse_ratio = peak_pulse_freq / config.MAX_TOTAL_PULSE_SLICE if config.MAX_TOTAL_PULSE_SLICE > 0 else 0
        if pulse_ratio > 1.0:
            v_overspeed_ratio = max(v_overspeed_ratio, pulse_ratio)
            
        msg = "SUCCESS"

        if v_overspeed_ratio > 1.0 or a_overspeed_ratio > 1.0:
            target_speed /= v_overspeed_ratio          
            target_accel /= (a_overspeed_ratio ** 2)
            target_jerk /= (a_overspeed_ratio ** 3)
            
            if pulse_ratio >= v_overspeed_ratio and pulse_ratio > 1.0:
                msg = f"[LIN] 算力超載 ({peak_pulse_freq:.0f}Hz)! Spd_Scale: {(1.0/v_overspeed_ratio):.2f}X"
            else:
                msg = f"[LIN] Overspeed! Spd_Scale: {(1.0/v_overspeed_ratio):.2f}X, Acc_Scale: {(1.0/(a_overspeed_ratio**2)):.2f}X"
                
        final_profile = SCurveProfile(dist_main, target_speed, target_accel, target_jerk)

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
        
        for i in range(N):
            if i % 10 == 0:
                time.sleep(0.001)

            ik_result, ik_error = inverse_kinematics(T_flange_targets[i], current_seed)
            if ik_result is None or ik_error > 0.1:
                return None, 0, f"軌跡 {prog_arr[i]*100:.1f}% 處遭遇死角或奇異點！"
            current_seed = ik_result
            exact_trajectory.append(ik_result)
            
        return exact_trajectory, final_profile.T_total, msg

    @staticmethod
    def calculate_ptp_trajectory(start_joints, target_joints, speed_factor, accel_factor=1.0):
        """PTP 關節空間向量化引擎"""
        start_joints, target_joints = np.array(start_joints), np.array(target_joints)

        delta_joints = target_joints - start_joints
        
        if np.max(np.abs(delta_joints)) < 0.1:
            return [start_joints], 0, "SUCCESS"
        
        d = np.abs(delta_joints)
        v = np.where(config.MAX_JOINT_SPEEDS == 0, 1e-6, config.MAX_JOINT_SPEEDS) * speed_factor
        a = np.where(config.MAX_JOINT_ACCELS == 0, 1e-6, config.MAX_JOINT_ACCELS) * accel_factor
        
        is_triangle = d <= (v**2) / a
        time_needed = np.where(is_triangle, 2 * np.sqrt(d / a), (d / v) + (v / a))
        
        bottleneck_idx = np.argmax(time_needed)
        dist_main = np.abs(delta_joints[bottleneck_idx])
        target_speed = config.MAX_JOINT_SPEEDS[bottleneck_idx] * speed_factor
        target_accel = config.MAX_JOINT_ACCELS[bottleneck_idx] * accel_factor
        target_jerk = config.MAX_JOINT_JERKS[bottleneck_idx] * accel_factor

        msg = "SUCCESS"
        peak_progress_rate = target_speed / dist_main if dist_main > 0 else 0
        peak_pulse_freq = np.sum(np.abs(delta_joints) * peak_progress_rate * config.STEPS_PER_DEG)
        
        if peak_pulse_freq > config.MAX_TOTAL_PULSE_SLICE:
            v_overspeed_ratio = peak_pulse_freq / config.MAX_TOTAL_PULSE_SLICE
            target_speed /= v_overspeed_ratio
            msg = f"[PTP] 算力超載 ({peak_pulse_freq:.0f}Hz)！Speed Scale: {(1.0/v_overspeed_ratio):.2f}X"

        final_profile = SCurveProfile(dist_main, target_speed, target_accel, target_jerk)

        interval = 0.010
        t_steps = np.arange(interval, final_profile.T_total, interval)
        if len(t_steps) == 0 or t_steps[-1] < final_profile.T_total:
            t_steps = np.append(t_steps, final_profile.T_total)

        prog_arr = np.array([final_profile.get_progress(t) for t in t_steps])
        exact_trajectory = start_joints + np.outer(prog_arr, delta_joints)

        return exact_trajectory, final_profile.T_total, msg

    @staticmethod
    def calculate_circ_trajectory(start_joints, aux_joints, target_joints, tcp_offset_mat, speed_factor, accel_factor=1.0):
        """CIRC 圓弧空間向量化引擎"""
        try:
            tcp_inv = np.linalg.inv(tcp_offset_mat)
        except np.linalg.LinAlgError:
            return None, 0, "Invalid TCP Matrix"
        
        T_tcp_start = forward_kinematics(start_joints) @ tcp_offset_mat
        T_tcp_aux = forward_kinematics(aux_joints) @ tcp_offset_mat
        T_tcp_end = forward_kinematics(target_joints) @ tcp_offset_mat
        
        P1, P2, P3 = T_tcp_start[:3, 3], T_tcp_aux[:3, 3], T_tcp_end[:3, 3]
        
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
        
        u = (P1 - center) / radius
        w = np.cross(normal, u)
        
        v_p3 = (P3 - center) / radius
        total_angle = math.atan2(np.dot(v_p3, w), np.dot(v_p3, u))
        if total_angle < 0: total_angle += 2 * math.pi
        
        arc_length_mm = radius * total_angle * 1000.0 

        target_speed = config.MAX_LIN_SPEED * speed_factor
        target_accel = config.MAX_LIN_ACCEL * accel_factor
        target_jerk = config.MAX_LIN_JERK * accel_factor
        final_profile = SCurveProfile(arc_length_mm, target_speed, target_accel, target_jerk)

        interval = 0.010  
        t_steps = np.arange(interval, final_profile.T_total, interval)
        if len(t_steps) == 0 or t_steps[-1] < final_profile.T_total:
            t_steps = np.append(t_steps, final_profile.T_total)

        prog_arr = np.array([final_profile.get_progress(t) for t in t_steps])
        N = len(prog_arr)

        angles = prog_arr * total_angle
        curr_pos_arr = center + radius * (np.outer(np.cos(angles), u) + np.outer(np.sin(angles), w))
        
        key_rots = R.from_matrix([T_tcp_start[:3, :3], T_tcp_end[:3, :3]])
        slerp = Slerp([0, 1], key_rots)
        curr_rot_arr = slerp(prog_arr).as_matrix()

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
                
            ik_result, ik_error = inverse_kinematics(T_flange_targets[i], current_seed)
            if ik_result is None or ik_error > 0.1:
                return None, 0, f"CIRC 軌跡 {prog_arr[i]*100:.1f}% 處遭遇死角！"
            current_seed = ik_result
            exact_trajectory.append(ik_result)
            
        return exact_trajectory, final_profile.T_total, "SUCCESS"