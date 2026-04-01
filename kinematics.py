import numpy as np
from scipy.spatial.transform import Rotation as R
import config

def get_tf_matrix(xyz, rpy_rad):
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', rpy_rad, degrees=False).as_matrix()
    T[:3, 3] = xyz
    return T

def get_rotation_matrix(axis, angle_deg):
    return fast_rotation_matrix(axis, angle_deg)

# ==========================================
# 優化 1：靜態矩陣快取 (Cache)
# 將永遠不會變的偏移量在啟動時先算好，不再重複計算
# ==========================================
T_BASE_FIXED = None
T_FIXED_LIST = []

def init_kinematics_cache():
    global T_BASE_FIXED, T_FIXED_LIST
    if T_BASE_FIXED is not None: 
        return # 已經初始化過了

    # (原本寫在這裡的 get_tf_matrix 已經移到外面了)
    
    base_xyz = [x * config.SCALE_FACTOR for x in config.BASE_MESH_OFFSET['xyz']]
    T_BASE_FIXED = get_tf_matrix(base_xyz, config.BASE_MESH_OFFSET['rpy'])

    for params in config.URDF_PARAMS:
        xyz = [x * config.SCALE_FACTOR for x in params['xyz']]
        T_FIXED_LIST.append(get_tf_matrix(xyz, params['rpy']))

# ==========================================
# 優化 2：超輕量級旋轉矩陣生成
# 徹底拔除 scipy 在核心迴圈的開銷，速度提升百倍！
# ==========================================
def fast_rotation_matrix(axis, angle_deg):
    rad = np.deg2rad(angle_deg)
    c, s = np.cos(rad), np.sin(rad)
    if axis == 'z':
        return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    elif axis == 'y':
        return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
    elif axis == 'x':
        return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
    return np.eye(4)

def forward_kinematics(joint_angles):
    init_kinematics_cache()
    T_current = T_BASE_FIXED.copy()

    for i, params in enumerate(config.URDF_PARAMS):
        raw_angle = joint_angles[i]
        angle = -raw_angle if params.get('invert', False) else raw_angle
        
        # 直接拿快取的固定矩陣，乘上瞬間生成的純量旋轉矩陣
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
# 優化 3：雅可比矩陣參數複用
# 接收已計算好的 T_current，省下 1/7 的 FK 算力
# ==========================================
def compute_numerical_jacobian(joints, T_current=None):
    epsilon = 1e-4 
    J = np.zeros((6, 6))
    
    if T_current is None:
        T_current = forward_kinematics(joints)
        
    current_pos = T_current[:3, 3]
    R_curr_T = T_current[:3, :3].T # 提前轉置，節省迴圈內開銷
    
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

def inverse_kinematics(target_matrix, seed_joints, max_retries=1):
    target_pos = target_matrix[:3, 3]
    target_rot = target_matrix[:3, :3]
    
    max_iter = 50       
    tolerance = 1e-3
    lambda_val = 0.01   
    lambda_sq_eye = (lambda_val**2) * np.eye(6) # 提前算出常數矩陣

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
            
            # 優化：一旦達標立刻跳出，不浪費算力去算下面的雅可比
            if np.linalg.norm(error_vector) < tolerance:
                return current_joints, np.linalg.norm(error_vector)

            # 傳入 T_curr 避免重複計算 FK
            J = compute_numerical_jacobian(current_joints, T_curr)
            
            # 優化 4：使用 np.linalg.solve 取代 inv，速度更快且數值更穩定
            XtX = J @ J.T + lambda_sq_eye
            # J_inv = J.T @ inv(XtX) => delta = J.T @ (XtX \ error)
            y = np.linalg.solve(XtX, error_vector)
            delta_theta = J.T @ y
            
            current_joints += np.rad2deg(delta_theta)
            
            # 限制在關節極限內
            for i in range(6):
                min_lim, max_lim = config.JOINT_LIMITS[i]
                if current_joints[i] < min_lim: current_joints[i] = min_lim
                if current_joints[i] > max_lim: current_joints[i] = max_lim

        # 若迴圈跑滿仍未完全收斂，但誤差可接受 (<0.1)，仍視為成功
        final_error = np.linalg.norm(error_vector)
        if final_error < 0.1: 
            return current_joints, final_error
            
    return None, None

def calculate_jog_joints(current_joints, axis, step_val, frame, T_total_offset):
    # 此部分邏輯良好，不需大改。直接享受底層 FK/IK 優化帶來的加速即可。
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
        else: 
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