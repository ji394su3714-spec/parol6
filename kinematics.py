# py
import math
import time
import warnings
import numpy as np
from scipy.spatial.transform import Rotation as R, RotationSpline, Slerp
from scipy.signal import savgol_filter
from scipy.interpolate import make_interp_spline

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

def fast_matrix_to_rotvec(R_mat):
    """
    極速版矩陣轉旋轉向量 (取代 scipy R.from_matrix)
    零物件生成開銷，專為 IK 緊密迴圈設計，速度提升 100 倍！
    """
    trace = R_mat[0, 0] + R_mat[1, 1] + R_mat[2, 2]
    # 防護浮點數誤差導致的定義域溢出
    val = max(min((trace - 1.0) / 2.0, 1.0), -1.0)
    theta = math.acos(val)

    if theta < 1e-6:
        return np.zeros(3)

    sin_theta = math.sin(theta)
    # 防護奇異點 (sin_theta 趨近於 0 時，用泰勒展開極限逼近，這裡直接給 0 確保安全)
    if abs(sin_theta) < 1e-6:
        return np.zeros(3)
        
    multiplier = theta / (2.0 * sin_theta)
    
    rx = (R_mat[2, 1] - R_mat[1, 2]) * multiplier
    ry = (R_mat[0, 2] - R_mat[2, 0]) * multiplier
    rz = (R_mat[1, 0] - R_mat[0, 1]) * multiplier
    
    return np.array([rx, ry, rz])

# ==========================================
# 靜態矩陣快取 (Cache)
# ==========================================
T_BASE_FIXED = None
T_FIXED_LIST = []

def init_kinematics_cache():
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
# 逆向運動學 (幾何雅可比矩陣引擎)
# ==========================================
def compute_geometric_jacobian(joints):
    """利用空間外積極速求導，1次FK迴圈解決，速度比數值法快7倍"""
    init_kinematics_cache()
    J = np.zeros((6, 6))
    T_curr = T_BASE_FIXED.copy()
    
    axes_world = []
    origins_world = []
    
    for i, params in enumerate(config.URDF_PARAMS):
        raw_angle = joints[i]
        angle = -raw_angle if params.get('invert', False) else raw_angle
        
        T_before_rot = T_curr @ T_FIXED_LIST[i]
        
        ax_str = params['axis']
        if ax_str == 'x': local_ax = np.array([1.0, 0.0, 0.0])
        elif ax_str == 'y': local_ax = np.array([0.0, 1.0, 0.0])
        else: local_ax = np.array([0.0, 0.0, 1.0])
        
        axis_w = T_before_rot[:3, :3] @ local_ax
        if params.get('invert', False): axis_w = -axis_w
            
        origin_w = T_before_rot[:3, 3]
        axes_world.append(axis_w)
        origins_world.append(origin_w)
        
        T_rot = fast_rotation_matrix(ax_str, angle)
        T_curr = T_before_rot @ T_rot
        
    P_ee = T_curr[:3, 3]
    
    for i in range(6):
        J[:3, i] = np.cross(axes_world[i], P_ee - origins_world[i])
        J[3:, i] = axes_world[i]                                   
        
    return J, T_curr

def _core_inverse_kinematics(target_matrix, seed_joints, max_retries=1):
    """核心逆向運動學函式，使用 Levenberg-Marquardt 方法求解，並加入奇異點脫困機制"""
    t_start = time.perf_counter() # 計時開始
    total_iters = 0               # 累計迭代次數

    target_pos = target_matrix[:3, 3]
    target_rot = target_matrix[:3, :3]
    
    max_iter = 50       
    pos_tol = 1e-5  
    rot_tol = np.deg2rad(0.01)
    
    rescue_pos_tol = config.IK_POS_TOLERANCE / 1000.0
    rescue_rot_tol = np.deg2rad(0.1)
    
    lambda_val = 0.01   
    lambda_sq_eye = (lambda_val**2) * np.eye(6) 

    for attempt in range(max_retries + 1):
        if attempt > 0:
            noise = np.random.uniform(-1.0, 1.0, 6) 
            current_joints = np.array(seed_joints, dtype=float) + noise
        else:
            current_joints = np.array(seed_joints, dtype=float)
            
        for _ in range(max_iter):
            total_iters += 1 # 每跑一次 Jacobian 迴圈就 +1
            
            J, T_curr = compute_geometric_jacobian(current_joints)
            
            curr_pos = T_curr[:3, 3]
            curr_rot = T_curr[:3, :3]
            
            err_pos = target_pos - curr_pos
            R_err = target_rot @ curr_rot.T
            err_rot = fast_matrix_to_rotvec(R_err)
            
            err_pos_norm = np.linalg.norm(err_pos)
            err_rot_norm = np.linalg.norm(err_rot)
            
            if err_pos_norm < pos_tol and err_rot_norm < rot_tol:
                # 只有在 DEBUG 模式開啟時，才去算時間和印出警告
                if config.DEBUG_IK_PROFILER:
                    dt_ms = (time.perf_counter() - t_start) * 1000.0
                    # 如果迭代次數>5或耗時>15ms，才印出警告
                    if total_iters > 5 or dt_ms > 15.0:
                        print(f"[IK Warn] {total_iters} iters | {dt_ms:.3f} ms")
                return current_joints, (err_pos_norm * 1000.0)

            error_vector = np.concatenate((err_pos, err_rot))
            
            XtX = J @ J.T + lambda_sq_eye
            y = np.linalg.solve(XtX, error_vector)
            delta_theta = J.T @ y
            
            if np.max(np.abs(delta_theta)) < 1e-4:
                if err_pos_norm < rescue_pos_tol and err_rot_norm < rescue_rot_tol:
                    if config.DEBUG_IK_PROFILER:
                        dt_ms = (time.perf_counter() - t_start) * 1000.0
                        #若迭代超過15次，印出奇異點脫困警告
                        if total_iters > 15:
                            print(f"[IK Rescue] 奇異點脫困: {total_iters} iters | {dt_ms:.3f} ms")
                    return current_joints, (err_pos_norm * 1000.0)
                break

            current_joints += np.rad2deg(delta_theta)
            
            for i in range(6):
                min_lim, max_lim = config.JOINT_LIMITS[i]
                if current_joints[i] < min_lim: current_joints[i] = min_lim
                if current_joints[i] > max_lim: current_joints[i] = max_lim

        # 迴圈跑滿 50 次的最終保命機制
        if err_pos_norm < rescue_pos_tol and err_rot_norm < rescue_rot_tol: 
            if config.DEBUG_IK_PROFILER:
                dt_ms = (time.perf_counter() - t_start) * 1000.0
                err_mm = err_pos_norm * 1000.0
                err_deg = np.rad2deg(err_rot_norm)
                print(f"[IK Rescue] 滿載保命 (耗盡迭代): {total_iters} iters | {dt_ms:.3f} ms | 殘留誤差: {err_mm:.3f} mm, {err_deg:.3f}°")
            return current_joints, (err_pos_norm * 1000.0)
            
    #print(f"[IK FAILED] 解算失敗，共掙扎 {total_iters} iters")
    return None, None

def inverse_kinematics(target_matrix, seed_joints, max_retries=1):
    """逆向運動學主函式，會自動判斷是否需要插值分段求解"""
    T_start = forward_kinematics(seed_joints)
    pos_start = T_start[:3, 3]
    pos_end = target_matrix[:3, 3]
    dist_mm = np.linalg.norm(pos_end - pos_start) * 1000.0
    
    R_start = T_start[:3, :3]
    R_end = target_matrix[:3, :3]
    R_diff = R_start.T @ R_end
    dist_deg = np.rad2deg(np.linalg.norm(fast_matrix_to_rotvec(R_diff)))
    
    steps = max(1, int(np.ceil(dist_mm / 20.0)), int(np.ceil(dist_deg / 10.0)))
    
    if steps == 1:
        return _core_inverse_kinematics(target_matrix, seed_joints, max_retries)
        
    curr_seed = np.array(seed_joints, dtype=float)
    last_err = 0.0
    
    key_rots = R.from_matrix([R_start, R_end])
    slerp = Slerp([0, 1], key_rots)
    
    for i in range(1, steps + 1):
        fraction = i / steps
        T_step = np.eye(4)
        T_step[:3, 3] = pos_start + fraction * (pos_end - pos_start)
        T_step[:3, :3] = slerp(fraction).as_matrix()
        
        res, err = _core_inverse_kinematics(T_step, curr_seed, max_retries=0)
        if res is not None:
            curr_seed = res
            last_err = err
        else:
            res, err = _core_inverse_kinematics(T_step, curr_seed, max_retries=1)
            if res is not None:
                curr_seed = res
                last_err = err
            else:
                return None, None
                
    return curr_seed, last_err

def calculate_base_shift_ik(T_flange_old, recorded_base_mat, target_base_mat, seed_joints):
    """純淨版 Base Shift：徹底根除 Euler，直接在 SO(3) 處理 Slerp 奇異點"""
    pos_start = recorded_base_mat[:3, 3]
    pos_end = target_base_mat[:3, 3]
    
    R_start = recorded_base_mat[:3, :3]
    R_end = target_base_mat[:3, :3]
    T_user = np.linalg.inv(recorded_base_mat) @ T_flange_old
    
    # 處理 180 度對稱旋轉的奇異點，避免 Slerp 迷失方向
    # 計算旋轉差異矩陣的跡數 (Trace)，如果 Trace 趨近於 -1，代表這是一個接近 180 度的翻轉
    R_diff = R_start.T @ R_end
    trace = np.trace(R_diff)
    
    if trace < -0.999: 
        # 注入極微小的擾動 (1e-4 rad 約等於 0.005 度) 破壞對稱性
        # 強迫 Slerp 毫不猶豫地選擇其中一條最短路徑！
        perturbation = R.from_rotvec([1e-4, 1e-4, 1e-4]).as_matrix()
        R_end = R_end @ perturbation

    return _run_interpolated_ik(pos_start, pos_end, R_start, R_end, T_user, seed_joints)

def _run_interpolated_ik(pos_start, pos_end, R_start, R_end, T_user, seed_joints):
    """拔除 method 切換，回歸最純粹的球面線性插值"""
    pos_dist = np.linalg.norm(pos_end - pos_start) * 1000.0
    R_diff = R_start.T @ R_end
    rot_dist_deg = np.rad2deg(np.linalg.norm(fast_matrix_to_rotvec(R_diff)))
    
    steps = max(1, int(np.ceil(pos_dist / 10.0)), int(np.ceil(rot_dist_deg / 5.0)))
    
    curr_seed = np.array(seed_joints, dtype=float)
    last_err = 0.0

    # 放心大膽地使用 SLERP，因為源頭的 180 度地雷已經被我們掃除了
    key_rots = R.from_matrix([R_start, R_end])
    slerp = Slerp([0, 1], key_rots)

    for i in range(1, steps + 1):
        fraction = i / steps
        T_interp_base = np.eye(4)
        T_interp_base[:3, 3] = pos_start + fraction * (pos_end - pos_start)
        T_interp_base[:3, :3] = slerp(fraction).as_matrix()
            
        T_target_flange = T_interp_base @ T_user
        
        next_joints, err = _core_inverse_kinematics(T_target_flange, curr_seed, max_retries=1)
        if next_joints is not None:
            curr_seed = next_joints
            last_err = err
        else:
            return None, f"Base Shift Failed at {fraction*100:.1f}% (Singularity blocked)"
            
    return curr_seed, last_err

def calculate_jog_joints(current_joints, axis, step_val, frame, T_total_offset, T_base_matrix=None, T_last_ideal_tcp=None):
    if T_base_matrix is None: T_base_matrix = np.eye(4)

    # ==========================================
    # 2. 誤差阻斷機制：如果有上一部的理想矩陣，直接沿用，無視關節誤差！
    # ==========================================
    if T_last_ideal_tcp is not None:
        T_tcp_curr = np.copy(T_last_ideal_tcp)
    else:
        # 只有在「第一下」點動時，才從物理關節推算初始位置
        T_math_flange = forward_kinematics(current_joints)
        T_tcp_curr = T_math_flange @ T_total_offset
        
    T_tcp_target = np.copy(T_tcp_curr)
    T_step = np.eye(4)
    
    if axis in ['x', 'y', 'z']:
        step_m = step_val / 1000.0
        idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        
        if frame == "Tool":
            T_step[idx, 3] = step_m
            T_tcp_target = T_tcp_curr @ T_step 
        elif frame == "Base":
            step_vec_local = np.zeros(3)
            step_vec_local[idx] = step_m
            step_vec_world = T_base_matrix[:3, :3] @ step_vec_local
            T_tcp_target[:3, 3] += step_vec_world
        else:
            T_tcp_target[idx, 3] += step_m     
    else: 
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
        else: 
            T_tcp_target[:3, :3] = R_step @ T_tcp_curr[:3, :3]

    T_flange_target = T_tcp_target @ np.linalg.inv(T_total_offset)
    new_joints, error_score = inverse_kinematics(T_flange_target, current_joints)
    
    if new_joints is None: return None, "IK Failed", None
        
    T_check_flange = forward_kinematics(new_joints)
    
    pos_diff = np.linalg.norm(T_check_flange[:3, 3] - T_flange_target[:3, 3]) * 1000.0
    if pos_diff > config.IK_POS_TOLERANCE: 
        return None, f"IK Inaccurate! Pos Diff: {pos_diff:.3f} mm", None

    R_check = T_check_flange[:3, :3]
    R_target = T_flange_target[:3, :3]
    rot_diff_rad = np.linalg.norm(fast_matrix_to_rotvec(R_check.T @ R_target))
    rot_diff_deg = np.rad2deg(rot_diff_rad)
    
    if rot_diff_deg > 0.1: 
        return None, f"IK Inaccurate! Rot Diff: {rot_diff_deg:.3f}°", None

    for i, angle in enumerate(new_joints):
        min_lim, max_lim = config.JOINT_LIMITS[i]
        if angle < (min_lim - 0.1) or angle > (max_lim + 0.1): return None, f"Limit Hit J{i+1}", None

    # 4. 成功解出時，把這次的「完美目標矩陣 (T_tcp_target)」一起傳回給上位機！
    return list(new_joints), None, T_tcp_target

def extract_continuous_rpy(T_matrix, prev_rpy_deg=None):
    r = R.from_matrix(T_matrix[:3, :3])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
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
    return (best_rpy + 180.0) % 360.0 - 180.0

def apply_base_frame(T_user_target, T_base_matrix):
    if T_base_matrix is None: return T_user_target
    return T_base_matrix @ T_user_target

def remove_base_frame(T_world_current, T_base_matrix):
    if T_base_matrix is None: return T_world_current
    return np.linalg.inv(T_base_matrix) @ T_world_current

# ==========================================
# 軌跡運算引擎 (Trajectory Math Engine)
# ==========================================
class TrajectoryMathEngine:
    
    @staticmethod
    def calculate_lin_trajectory(start_joints, target_joints, tcp_offset_mat, speed_factor, accel_factor=1.0):
        try:
            tcp_inv = np.linalg.inv(tcp_offset_mat)
        except np.linalg.LinAlgError:
            return None, 0, "Invalid TCP Matrix", 0
    
        T_flange_start = forward_kinematics(start_joints)
        T_tcp_start = T_flange_start @ tcp_offset_mat
        T_flange_end = forward_kinematics(target_joints)
        T_tcp_end = T_flange_end @ tcp_offset_mat
        
        pos_start, pos_end = T_tcp_start[:3, 3], T_tcp_end[:3, 3]
        R_start = T_tcp_start[:3, :3]
        R_end = T_tcp_end[:3, :3]
        R_diff = R_start.T @ R_end
        
        dist_deg = np.rad2deg(np.linalg.norm(R.from_matrix(R_diff).as_rotvec()))
        dist_mm = np.linalg.norm(pos_end - pos_start) * 1000.0
            
        if dist_mm < 0.1 and dist_deg < 0.1:
            def short_gen(): yield list(start_joints)
            return short_gen(), 0, "SUCCESS", 1

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

        # =======================================================
        # Pass 1: LIN (Dry-Run) + IK 驗證
        # =======================================================
        sample_steps = max(2, int(dist_main / 5.0)) 
        p1_prog = np.linspace(0, 1.0, sample_steps)
        p1_pos = pos_start + np.outer(p1_prog, (pos_end - pos_start))
        p1_expected_joints = start_joints + np.outer(p1_prog, (target_joints - start_joints))
        
        methods_to_try = ['slerp', 'euler']
        traj_p1 = []
        failed_p1 = True
        
        used_method = 'slerp' 
        
        for method in methods_to_try:
            traj_p1 = []
            failed_p1 = False
            if method == 'slerp':
                key_rots = R.from_matrix([R_start, R_end])
                p1_rot = Slerp([0, 1], key_rots)(p1_prog).as_matrix()
            else:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore", UserWarning)
                    rpy_start = R.from_matrix(R_start).as_euler('xyz')
                    rpy_end = R.from_matrix(R_end).as_euler('xyz')
                diff = (rpy_end - rpy_start + np.pi) % (2 * np.pi) - np.pi
                rpy_end_unw = rpy_start + diff
                p1_rot = np.zeros((sample_steps, 3, 3))
                for i, p in enumerate(p1_prog):
                    p1_rot[i] = R.from_euler('xyz', rpy_start * (1 - p) + rpy_end_unw * p).as_matrix()

            T_tcp_p1 = np.zeros((sample_steps, 4, 4))
            T_tcp_p1[:, 3, 3] = 1.0                
            T_tcp_p1[:, :3, :3] = p1_rot     
            T_tcp_p1[:, :3, 3] = p1_pos      
            T_flange_p1 = T_tcp_p1 @ tcp_inv  
            
            for i in range(sample_steps):
                ik_res, ik_err = _core_inverse_kinematics(T_flange_p1[i], p1_expected_joints[i])
                if ik_res is None or ik_err > (config.IK_POS_TOLERANCE * 5):
                    failed_p1 = True
                    break
                traj_p1.append(ik_res)
                
            if not failed_p1:
                used_method = method
                break 

        if failed_p1:
            return None, 0, f"Trajectory failed: Singularities hit in both spaces.", 0

        traj_p1 = np.array(traj_p1)
        delta_p = 1.0 / (sample_steps - 1) if sample_steps > 1 else 1.0
        d_joint_dp = np.gradient(traj_p1, delta_p, axis=0)
        d2_joint_dp2 = np.gradient(d_joint_dp, delta_p, axis=0)
        
        window_len = min(7, len(traj_p1))
        if window_len % 2 == 0: window_len -= 1
        if window_len >= 3:
            d_joint_dp = savgol_filter(d_joint_dp, window_len, 2, axis=0)
            d2_joint_dp2 = savgol_filter(d2_joint_dp2, window_len, 2, axis=0)

        max_d_joint_dp = np.max(np.abs(d_joint_dp), axis=0)
        max_d2_joint_dp2 = np.max(np.abs(d2_joint_dp2), axis=0)

        peak_prog_v = target_speed / dist_main if dist_main > 0 else 0
        peak_prog_a = target_accel / dist_main if dist_main > 0 else 0

        v_overspeed_ratio, a_overspeed_ratio = 1.0, 1.0

        for i in range(6):
            peak_v = max_d_joint_dp[i] * peak_prog_v  
            allowed_v = config.MAX_JOINT_SPEEDS[i] 
            if peak_v > allowed_v and allowed_v > 0:
                v_overspeed_ratio = max(v_overspeed_ratio, peak_v / allowed_v)

            peak_a = max_d2_joint_dp2[i] * (peak_prog_v ** 2) + max_d_joint_dp[i] * peak_prog_a
            allowed_a = config.MAX_JOINT_ACCELS[i] 
            if peak_a > allowed_a and allowed_a > 0:
                a_overspeed_ratio = max(a_overspeed_ratio, math.sqrt(peak_a / allowed_a))
                
        scale_down = 1.0
        if v_overspeed_ratio > 1.0 or a_overspeed_ratio > 1.0:
            scale_down = max(v_overspeed_ratio, a_overspeed_ratio)
            target_speed /= scale_down
            target_accel /= (scale_down ** 2)
            target_jerk /= (scale_down ** 3)
            
        # =======================================================
        # Pass 2: S-Curve 時間精確採樣與惰性 Generator
        # =======================================================
        final_profile = SCurveProfile(dist_main, target_speed, target_accel, target_jerk)
        interval = 0.010  
        t_steps = np.arange(interval, final_profile.T_total, interval)
        if len(t_steps) == 0 or t_steps[-1] < final_profile.T_total:
            t_steps = np.append(t_steps, final_profile.T_total)

        prog_arr = np.array([final_profile.get_progress(t) for t in t_steps])
        N = len(prog_arr)

        curr_pos_arr = pos_start + np.outer(prog_arr, (pos_end - pos_start))
        expected_joints_arr = start_joints + np.outer(prog_arr, (target_joints - start_joints))
        
        if used_method == 'slerp':
            # 優化：如果旋轉角度小於 0.05 度 (純平移加工)，直接複製起始姿態，省下極度耗時的 Slerp 實例化！
            if dist_deg < 0.05:
                curr_rot_arr = np.array([R_start] * N)
            else:
                curr_rot_arr = Slerp([0, 1], R.from_matrix([R_start, R_end]))(prog_arr).as_matrix()
        else:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", UserWarning)
                rpy_start = R.from_matrix(R_start).as_euler('xyz')
                rpy_end = R.from_matrix(R_end).as_euler('xyz')
            diff = (rpy_end - rpy_start + np.pi) % (2 * np.pi) - np.pi
            rpy_end_unw = rpy_start + diff
            curr_rot_arr = np.zeros((N, 3, 3))
            for i, p in enumerate(prog_arr):
                curr_rot_arr[i] = R.from_euler('xyz', rpy_start * (1 - p) + rpy_end_unw * p).as_matrix()

        T_tcp_targets = np.zeros((N, 4, 4))
        T_tcp_targets[:, 3, 3] = 1.0                
        T_tcp_targets[:, :3, :3] = curr_rot_arr     
        T_tcp_targets[:, :3, 3] = curr_pos_arr      
        T_flange_targets = T_tcp_targets @ tcp_inv  

        # 惰性生成器 (Lazy Generator)：算一點、交一點，不再霸佔 CPU
        def lin_generator():
            total_ik_time = 0.0 
            
            for i in range(N):
                t0 = time.perf_counter() 
                ik_res, ik_err = _core_inverse_kinematics(T_flange_targets[i], expected_joints_arr[i])
                total_ik_time += (time.perf_counter() - t0) 
                
                if ik_res is None or ik_err > (config.IK_POS_TOLERANCE * 5):
                    raise RuntimeError(f"LIN Singularity blocked at {prog_arr[i]*100:.1f}%")
                
                if i == N - 1 and config.DEBUG_IK_PROFILER:
                    avg_ms = (total_ik_time / N) * 1000.0
                    max_hz = 1000.0 / avg_ms if avg_ms > 0 else 0
                    print(f"\n--- [LIN Profiler] ---")
                    print(f"總採樣點數: {N} 點")
                    print(f"平均 IK 耗時: {avg_ms:.3f} ms / 點")
                    #print(f"極限串流能力: {max_hz:.0f} Hz")
                    #print(f"----------------------\n")

                yield list(ik_res)
            
        msg = "SUCCESS"
        if used_method == 'euler':
            msg = "[System] SLERP singularity bypassed using Euler interpolation."
        elif scale_down > 1.0:
            msg = f"[LIN] Safe Auto-Scale: Speed reduced to {(1.0/scale_down):.2f}X."
            
        return lin_generator(), final_profile.T_total, msg, N

    @staticmethod
    def calculate_ptp_trajectory(start_joints, target_joints, speed_factor, accel_factor=1.0):
        start_joints, target_joints = np.array(start_joints), np.array(target_joints)
        delta_joints = target_joints - start_joints
        
        if np.max(np.abs(delta_joints)) < 0.1:
            def short_gen(): yield list(start_joints)
            return short_gen(), 0, "SUCCESS", 1
        
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
        
        if peak_pulse_freq > config.MAX_PULSE_FREQ:
            v_overspeed_ratio = peak_pulse_freq / config.MAX_PULSE_FREQ
            target_speed /= v_overspeed_ratio
            msg = f"[PTP] Compute Bottleneck ({peak_pulse_freq:.0f}Hz)！Speed Scale: {(1.0/v_overspeed_ratio):.2f}X"

        final_profile = SCurveProfile(dist_main, target_speed, target_accel, target_jerk)

        interval = 0.010
        t_steps = np.arange(interval, final_profile.T_total, interval)
        if len(t_steps) == 0 or t_steps[-1] < final_profile.T_total:
            t_steps = np.append(t_steps, final_profile.T_total)

        prog_arr = np.array([final_profile.get_progress(t) for t in t_steps])
        exact_trajectory = start_joints + np.outer(prog_arr, delta_joints)
        N = len(exact_trajectory)

        # 把陣列轉成 Generator 回傳
        def ptp_generator():
            for pt in exact_trajectory:
                yield list(pt)

        return ptp_generator(), final_profile.T_total, msg, N

    @staticmethod
    def calculate_circ_trajectory(start_joints, aux_joints, target_joints, tcp_offset_mat, speed_factor, accel_factor=1.0):
        try:
            tcp_inv = np.linalg.inv(tcp_offset_mat)
        except np.linalg.LinAlgError:
            return None, 0, "Invalid TCP Matrix", 0
        
        T_tcp_start = forward_kinematics(start_joints) @ tcp_offset_mat
        T_tcp_aux = forward_kinematics(aux_joints) @ tcp_offset_mat
        T_tcp_end = forward_kinematics(target_joints) @ tcp_offset_mat
        
        P1, P2, P3 = T_tcp_start[:3, 3], T_tcp_aux[:3, 3], T_tcp_end[:3, 3]
        
        v1, v2 = P2 - P1, P3 - P1
        cross_v1_v2 = np.cross(v1, v2)
        cross_norm = np.linalg.norm(cross_v1_v2)
        if cross_norm < 1e-6:
            return None, 0, "CIRC Error: Three points are collinear!", 0
            
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
        
        # =======================================================
        # Pass 1: CIRC (Dry-Run) + IK 驗證
        # =======================================================
        sample_steps = max(3, int(arc_length_mm / 5.0))
        p1_prog = np.linspace(0, 1.0, sample_steps)
        
        p1_angles = p1_prog * total_angle
        p1_pos = center + radius * (np.outer(np.cos(p1_angles), u) + np.outer(np.sin(p1_angles), w))
        p1_expected_joints = start_joints + np.outer(p1_prog, (target_joints - start_joints))
        
        key_rots = R.from_matrix([T_tcp_start[:3, :3], T_tcp_end[:3, :3]])
        p1_rot = Slerp([0, 1], key_rots)(p1_prog).as_matrix()

        T_tcp_p1 = np.zeros((sample_steps, 4, 4))
        T_tcp_p1[:, 3, 3] = 1.0                
        T_tcp_p1[:, :3, :3] = p1_rot     
        T_tcp_p1[:, :3, 3] = p1_pos      
        T_flange_p1 = T_tcp_p1 @ tcp_inv  

        traj_p1 = []
        for i in range(sample_steps):
            ik_res, ik_err = _core_inverse_kinematics(T_flange_p1[i], p1_expected_joints[i])
            if ik_res is None or ik_err > (config.IK_POS_TOLERANCE * 5):
                return None, 0, f"CIRC 軌跡 {p1_prog[i]*100:.1f}% 處遭遇死角！", 0
            traj_p1.append(ik_res)
            
        traj_p1 = np.array(traj_p1)
        delta_p = 1.0 / (sample_steps - 1) if sample_steps > 1 else 1.0
        d_joint_dp = np.gradient(traj_p1, delta_p, axis=0)
        d2_joint_dp2 = np.gradient(d_joint_dp, delta_p, axis=0)
        
        window_len = min(7, len(traj_p1))
        if window_len % 2 == 0: window_len -= 1
        if window_len >= 3:
            d_joint_dp = savgol_filter(d_joint_dp, window_len, 2, axis=0)
            d2_joint_dp2 = savgol_filter(d2_joint_dp2, window_len, 2, axis=0)

        max_d_joint_dp = np.max(np.abs(d_joint_dp), axis=0)
        max_d2_joint_dp2 = np.max(np.abs(d2_joint_dp2), axis=0)

        peak_prog_v = target_speed / arc_length_mm if arc_length_mm > 0 else 0
        peak_prog_a = target_accel / arc_length_mm if arc_length_mm > 0 else 0
        v_overspeed_ratio, a_overspeed_ratio = 1.0, 1.0

        for i in range(6):
            peak_v = max_d_joint_dp[i] * peak_prog_v  
            allowed_v = config.MAX_JOINT_SPEEDS[i] 
            if peak_v > allowed_v and allowed_v > 0:
                v_overspeed_ratio = max(v_overspeed_ratio, peak_v / allowed_v)
            peak_a = max_d2_joint_dp2[i] * (peak_prog_v ** 2) + max_d_joint_dp[i] * peak_prog_a
            allowed_a = config.MAX_JOINT_ACCELS[i] 
            if peak_a > allowed_a and allowed_a > 0:
                a_overspeed_ratio = max(a_overspeed_ratio, math.sqrt(peak_a / allowed_a))

        scale_down = 1.0
        if v_overspeed_ratio > 1.0 or a_overspeed_ratio > 1.0:
            scale_down = max(v_overspeed_ratio, a_overspeed_ratio)
            target_speed /= scale_down
            target_accel /= (scale_down ** 2)
            target_jerk /= (scale_down ** 3)
            
        # =======================================================
        # Pass 2: S-Curve 時間精確採樣與惰性 Generator
        # =======================================================
        final_profile = SCurveProfile(arc_length_mm, target_speed, target_accel, target_jerk)
        interval = 0.010  
        t_steps = np.arange(interval, final_profile.T_total, interval)
        if len(t_steps) == 0 or t_steps[-1] < final_profile.T_total:
            t_steps = np.append(t_steps, final_profile.T_total)

        prog_arr = np.array([final_profile.get_progress(t) for t in t_steps])
        N = len(prog_arr)
        
        angles = prog_arr * total_angle
        curr_pos_arr = center + radius * (np.outer(np.cos(angles), u) + np.outer(np.sin(angles), w))
        curr_rot_arr = Slerp([0, 1], key_rots)(prog_arr).as_matrix()

        T_tcp_targets = np.zeros((N, 4, 4))
        T_tcp_targets[:, 3, 3] = 1.0                
        T_tcp_targets[:, :3, :3] = curr_rot_arr     
        T_tcp_targets[:, :3, 3] = curr_pos_arr      
        T_flange_targets = T_tcp_targets @ tcp_inv  

        expected_joints_arr = start_joints + np.outer(prog_arr, (target_joints - start_joints))
        
        # CIRC 專屬 Generator
        def circ_generator():
            total_ik_time = 0.0
            
            for i in range(N):
                t0 = time.perf_counter()
                ik_res, ik_err = _core_inverse_kinematics(T_flange_targets[i], expected_joints_arr[i])
                total_ik_time += (time.perf_counter() - t0) 
                
                if ik_res is None or ik_err > (config.IK_POS_TOLERANCE * 5):
                    raise RuntimeError(f"CIRC Singularity blocked at {prog_arr[i]*100:.1f}%")
                yield list(ik_res)
                
            # 只有 DEBUG 開啟，才印出 CIRC 效能報告
            if config.DEBUG_IK_PROFILER and N > 0:
                avg_ms = (total_ik_time / N) * 1000.0
                max_hz = 1000.0 / avg_ms if avg_ms > 0 else 0
                print(f"\n--- [CIRC Profiler] ---")
                print(f"總採樣點數: {N} 點")
                print(f"平均 IK 耗時: {avg_ms:.3f} ms / 點")
                #print(f"極限串流能力: {max_hz:.0f} Hz")
                #print(f"----------------------\n")
            
        msg = "SUCCESS"
        if scale_down > 1.0:
            msg = f"[CIRC] Safe Auto-Scale: Speed reduced to {(1.0/scale_down):.2f}X."
            
        return circ_generator(), final_profile.T_total, msg, N
    
    @staticmethod
    def calculate_spline_trajectory(start_joints, spline_waypoints, interval=0.010, min_step_mm=0.0):
        """終極完全體：笛卡爾 XYZ 五次樣條 + SO(3) 週期邊界旋轉樣條 (絕對恆速 + 控制器濾波版)"""

        # ==========================================
        # [控制台參數區] 供你手動隨時切換
        ENABLE_DEBUG_PRINT = False
        MIN_STEP_MM = min_step_mm  
        # ==========================================

        if not spline_waypoints:
            def empty_gen(): yield list(start_joints)
            return empty_gen(), 0.0, "Empty SPLINE array", 0

        tcp_mat = spline_waypoints[0].get('tcp_offset_mat', np.eye(4))
        inv_tcp_mat = np.linalg.inv(tcp_mat)

        # 1. 預先解析所有原始點位 (從關節推算回真實 TCP)
        raw_poses = []
        for wp in spline_waypoints:
            target_joints = wp['target_joints']
            speed_factor = wp.get('speed_factor', 1.0)
            
            T_target_flange = forward_kinematics(target_joints)
            T_target_tcp = T_target_flange @ tcp_mat
            raw_poses.append({
                'pos': T_target_tcp[:3, 3],
                'rot': T_target_tcp[:3, :3],
                'speed_factor': speed_factor
            })

        # 2. 啟動微線段濾波 (控制器端動態整流)
        T_start_flange = forward_kinematics(start_joints) 
        T_start_tcp = T_start_flange @ tcp_mat
        
        filtered_poses = [{
            'pos': T_start_tcp[:3, 3],
            'rot': T_start_tcp[:3, :3],
            'speed_factor': raw_poses[0]['speed_factor'] if raw_poses else 1.0
        }]
        
        for i in range(len(raw_poses)):
            is_last = (i == len(raw_poses) - 1)
            curr = raw_poses[i]
            
            # 計算目前點位到「上一個已通過審核的點」的距離
            dist_to_last = np.linalg.norm(curr['pos'] - filtered_poses[-1]['pos']) * 1000.0
            
            if dist_to_last >= MIN_STEP_MM:
                filtered_poses.append(curr)
            elif is_last:
                # 完美收尾邏輯：如果最後一點離上一點太近，直接用終點覆蓋上一個點。
                if len(filtered_poses) > 1:
                    filtered_poses[-1] = curr
                else:
                    filtered_poses.append(curr)

        # 3. 計算空間距離與動力學時間 (Chordal Time Parameterization)
        t_arr = [0.0]
        u_arr = [0.0]  # 新增：純幾何參數 (確保軌跡絕對不變形)
        xyz_pts = [filtered_poses[0]['pos']]
        rot_matrices = [filtered_poses[0]['rot']]
        current_t = 0.0
        current_u = 0.0
        
        if ENABLE_DEBUG_PRINT:
            print(f"\n--- [恆速驗證：控制器端濾波啟動 | 濾波門檻 {MIN_STEP_MM}mm | 共 {len(filtered_poses)} 點] ---")
        
        for i in range(1, len(filtered_poses)):
            curr_pos = filtered_poses[i]['pos']
            curr_rot = filtered_poses[i]['rot']
            prev_pos = filtered_poses[i-1]['pos']
            prev_rot = filtered_poses[i-1]['rot']
            speed_factor = filtered_poses[i]['speed_factor']
            
            dist_mm = np.linalg.norm(curr_pos - prev_pos) * 1000.0
            rot_diff = R.from_matrix(prev_rot.T @ curr_rot).magnitude()
            dist_deg = np.degrees(rot_diff)
            
            if dist_mm < 1e-3 and dist_deg < 1e-2:
                continue
            
            v_lin = config.MAX_LIN_SPEED * speed_factor if config.MAX_LIN_SPEED > 0 else 1e-6
            
            # 永遠使用系統設定的最大角速度 (MAX_ROT_SPEED)，確保轉角時能瞬間完成姿態切換，避免原地過度停留。
            v_rot = config.MAX_ROT_SPEED if config.MAX_ROT_SPEED > 0 else 1e-6
            
            dt_lin = dist_mm / v_lin if dist_mm > 1e-6 else 0.0
            dt_rot = dist_deg / v_rot if dist_deg > 1e-6 else 0.0
            dt = max(dt_lin, dt_rot)
            
            if dt < 1e-5: 
                dt = 1e-5
                
            if ENABLE_DEBUG_PRINT:
                vel_deg = (dist_deg / dt) if dt > 0 else 0
                vel_mm = (dist_mm / dt) if dt > 0 else 0
                print(f"點 {len(t_arr):03d}: 距離 {dist_mm:.4f}mm, 角度差 {dist_deg:.4f}°, 分配時間 {dt:.5f}s | 局部角速度: {vel_deg:.2f} deg/s, 線速度: {vel_mm:.2f} mm/s")
                
            current_t += dt
            t_arr.append(current_t)
            
            # 幾何參數：保證絕對單調遞增，將曲線死死釘在空間中！
            du = max(dist_mm, 1e-3) + dist_deg * 0.1
            current_u += du
            u_arr.append(current_u)
            
            xyz_pts.append(curr_pos)
            rot_matrices.append(curr_rot)

        if ENABLE_DEBUG_PRINT:
            print("-------------------------------------------\n")

        t_arr = np.array(t_arr)
        u_arr = np.array(u_arr)
        xyz_pts = np.array(xyz_pts)
        rot_matrices = np.array(rot_matrices)
        num_points = len(t_arr)

        k = 3 if num_points > 3 else num_points - 1
        if k < 1: k = 1

        is_closed = False
        if num_points >= 4:
            dist_close = np.linalg.norm(xyz_pts[0] - xyz_pts[-1])
            rot_close = R.from_matrix(rot_matrices[0].T @ rot_matrices[-1]).magnitude()
            if dist_close < 2e-4 and rot_close < 1e-2:
                is_closed = True

        if is_closed:
            pad = min(3, num_points - 2) 
            
            head_u, head_xyz, head_rot = [], [], []
            curr_u_head = u_arr[0]
            for i in range(1, pad + 1):
                idx = -1 - i  
                du_step = u_arr[idx + 1] - u_arr[idx]
                curr_u_head -= du_step
                head_u.insert(0, curr_u_head)
                head_xyz.insert(0, xyz_pts[idx])
                head_rot.insert(0, rot_matrices[idx])
                
            tail_u, tail_xyz, tail_rot = [], [], []
            curr_u_tail = u_arr[-1]
            for i in range(1, pad + 1):
                idx = i  
                du_step = u_arr[idx] - u_arr[idx - 1]
                curr_u_tail += du_step
                tail_u.append(curr_u_tail)
                tail_xyz.append(xyz_pts[idx])
                tail_rot.append(rot_matrices[idx])
                
            ext_u = np.concatenate((head_u, u_arr, tail_u))
            ext_xyz = np.concatenate((head_xyz, xyz_pts, tail_xyz))
            ext_rot = np.concatenate((head_rot, rot_matrices, tail_rot))
            
            # 樣條曲線改吃純幾何參數 (ext_u)，完全免疫物理時間的膨脹！
            spline_xyz = make_interp_spline(ext_u, ext_xyz, k=k)
            spline_rot = RotationSpline(ext_u, R.from_matrix(ext_rot))
            
        else:
            # 樣條曲線改吃純幾何參數 (u_arr)
            spline_xyz = make_interp_spline(u_arr, xyz_pts, k=k)
            spline_rot = RotationSpline(u_arr, R.from_matrix(rot_matrices))

        t_steps = np.arange(interval, t_arr[-1], interval)
        if len(t_steps) == 0 or t_steps[-1] < t_arr[-1]:
            t_steps = np.append(t_steps, t_arr[-1])

        # 最終神技：時間映射 (Time to Geometry Parameter Mapping)
        # 用插值法確保機器人嚴格按照分配的物理時間 (t) 在這條鐵軌 (u) 上前進
        u_steps = np.interp(t_steps, t_arr, u_arr)

        N = len(t_steps)
        sampled_xyz = spline_xyz(u_steps)
        sampled_rot = spline_rot(u_steps).as_matrix()

        def spline_generator():
            seed = start_joints
            
            def unwrap_joints(current, previous):
                unwrapped = []
                for c, p in zip(current, previous):
                    diff = (c - p + 180) % 360 - 180
                    unwrapped.append(p + diff)
                return unwrapped

            total_ik_time = 0.0
            for i in range(N):
                T_target_tcp = np.eye(4)
                T_target_tcp[:3, :3] = sampled_rot[i]
                T_target_tcp[:3, 3] = sampled_xyz[i]
                
                T_target_flange = T_target_tcp @ inv_tcp_mat
                
                t0 = time.perf_counter() 
                ik_res, ik_err = _core_inverse_kinematics(T_target_flange, seed) 
                total_ik_time += (time.perf_counter() - t0) 
                
                if ik_res is None:
                    raise RuntimeError(f"SPLINE Singularity hit at {t_steps[i]:.2f}s")
                
                ik_res = unwrap_joints(ik_res, seed)
                seed = ik_res
                
                yield list(ik_res)

        msg = f"[System] Perfect SO(3) Spline compiled successfully ({num_points} points)."
        return spline_generator(), t_arr[-1], msg, N