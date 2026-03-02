import numpy as np
from scipy.spatial.transform import Rotation as R
import config

def get_tf_matrix(xyz, rpy_rad):
    T = np.eye(4)
    rot = R.from_euler('xyz', rpy_rad, degrees=False)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = xyz
    return T

def get_rotation_matrix(axis, angle_deg):
    T = np.eye(4)
    if axis == 'z':
        rot = R.from_euler('z', angle_deg, degrees=True)
        T[:3, :3] = rot.as_matrix()
    return T

def forward_kinematics(joint_angles):
    T_current = np.eye(4)
    base_xyz = [x * config.SCALE_FACTOR for x in config.BASE_MESH_OFFSET['xyz']]
    T_base = get_tf_matrix(base_xyz, config.BASE_MESH_OFFSET['rpy'])
    T_current = T_base

    for i, params in enumerate(config.URDF_PARAMS):
        raw_angle = joint_angles[i]
        angle = -raw_angle if params.get('invert', False) else raw_angle
        xyz = [x * config.SCALE_FACTOR for x in params['xyz']]
        T_fixed = get_tf_matrix(xyz, params['rpy'])
        T_rot = get_rotation_matrix(params['axis'], angle)
        T_current = T_current @ T_fixed @ T_rot
        
    return T_current

# --- 全新核心：數值雅可比矩陣計算 ---
def compute_numerical_jacobian(joints):
    epsilon = 1e-4 # 微小擾動量 (弧度)
    J = np.zeros((6, 6))
    
    # 1. 取得當前 TCP 位置與姿態
    T_current = forward_kinematics(joints)
    current_pos = T_current[:3, 3]
    
    # 2. 對每個關節進行擾動
    for i in range(6):
        perturbed_joints = list(joints)
        perturbed_joints[i] += np.rad2deg(epsilon) # FK 需要輸入角度
        
        T_new = forward_kinematics(perturbed_joints)
        new_pos = T_new[:3, 3]
        
        # --- 線性速度 (v) ---
        J[:3, i] = (new_pos - current_pos) / epsilon
        
        # --- 角速度 (w) ---
        R_curr = T_current[:3, :3]
        R_new = T_new[:3, :3]
        R_diff = R_new @ R_curr.T
        rot_vec = R.from_matrix(R_diff).as_rotvec()
        J[3:, i] = rot_vec / epsilon
        
    return J

def inverse_kinematics(target_matrix, seed_joints, max_retries=1):
    """
    使用【阻尼最小平方法】進行迭代求解。
    若單次求解失敗，會對 seed_joints 加入微小隨機擾動並重試，增加收斂機率。
    """
    target_pos = target_matrix[:3, 3]
    target_rot = target_matrix[:3, :3]
    
    for attempt in range(max_retries + 1):
        # 如果是重試 (attempt > 0)，為種子點加入正負 5 度的隨機擾動
        if attempt > 0:
            noise = np.random.uniform(-5.0, 5.0, 6)
            current_joints = np.array(seed_joints, dtype=float) + noise
        else:
            current_joints = np.array(seed_joints, dtype=float)
            
        max_iter = 50       
        tolerance = 1e-5    
        lambda_val = 0.01   

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

            J = compute_numerical_jacobian(current_joints)
            XtX = J @ J.T + lambda_val**2 * np.eye(6)
            J_inv = J.T @ np.linalg.inv(XtX)
            
            delta_theta = J_inv @ error_vector
            current_joints += np.rad2deg(delta_theta)
            
            # 限制在關節極限內
            for i in range(6):
                min_lim, max_lim = config.JOINT_LIMITS[i]
                if current_joints[i] < min_lim: current_joints[i] = min_lim
                if current_joints[i] > max_lim: current_joints[i] = max_lim

        final_error = np.linalg.norm(error_vector)
        if final_error < 0.1: 
            return current_joints, final_error
            
    # 如果經過重試仍無法收斂，才回傳 None
    return None, None

def forward_kinematics_all(joint_angles):
    """
    回傳所有關節的轉換矩陣列表 [T_base, T1, T2, ... T6]
    用於骨架繪製
    """
    matrices = []
    
    # Base
    base_xyz = [x * config.SCALE_FACTOR for x in config.BASE_MESH_OFFSET['xyz']]
    T_current = get_tf_matrix(base_xyz, config.BASE_MESH_OFFSET['rpy'])
    matrices.append(T_current) # 儲存基座原點

    # Links
    for i, params in enumerate(config.URDF_PARAMS):
        raw_angle = joint_angles[i]
        angle = -raw_angle if params.get('invert', False) else raw_angle
        xyz = [x * config.SCALE_FACTOR for x in params['xyz']]
        T_fixed = get_tf_matrix(xyz, params['rpy'])
        T_rot = get_rotation_matrix(params['axis'], angle)
        
        T_current = T_current @ T_fixed @ T_rot
        matrices.append(T_current)
        
    return matrices

def calculate_jog_joints(current_joints, axis, step_val, frame, T_total_offset):
    """
    計算 Cartesian JOG 之後的目標關節角度。
    
    :param current_joints: 基礎關節角度 (list 或 array)
    :param axis: 移動軸 ('x', 'y', 'z', 'rx', 'ry', 'rz')
    :param step_val: 移動量包含正負號 (mm 或 degree)
    :param frame: 座標系 ('Base' 或 'Tool')
    :param T_total_offset: TCP 與硬體的總偏移矩陣
    :return: (new_joints, error_msg) 成功時回傳新陣列，失敗回傳 None 與錯誤字串
    """
    
    # 1. 正向運動學推算當前 TCP
    T_math_flange = forward_kinematics(current_joints)
    T_tcp_curr = T_math_flange @ T_total_offset
    T_tcp_target = np.copy(T_tcp_curr)
    T_step = np.eye(4)
    
    # 2. 矩陣轉換 (Base vs Tool)
    if axis in ['x', 'y', 'z']:
        step_m = step_val / 1000.0
        idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        
        if frame == "Tool":
            T_step[idx, 3] = step_m
            T_tcp_target = T_tcp_curr @ T_step 
        else: # Base
            T_tcp_target[idx, 3] += step_m     
    else:
        step_rad = np.deg2rad(step_val)
        vec = np.zeros(3)
        rot_idx = {'x': 0, 'y': 1, 'z': 2}[axis[1]] 
        vec[rot_idx] = step_rad
        from scipy.spatial.transform import Rotation as R
        R_step = R.from_rotvec(vec).as_matrix()
        
        if frame == "Tool":
            T_step[:3, :3] = R_step
            T_tcp_target = T_tcp_curr @ T_step 
        else: # Base (原地以 World 基準旋轉)
            T_tcp_target[:3, :3] = R_step @ T_tcp_curr[:3, :3]

    # 3. 逆向運動學求解
    T_flange_target = T_tcp_target @ np.linalg.inv(T_total_offset)
    new_joints, error_score = inverse_kinematics(T_flange_target, current_joints)
    
    if new_joints is None:
        return None, "IK Failed"
        
    # 4. 安全性驗證 (精度與極限)
    T_check_flange = forward_kinematics(new_joints)
    pos_diff = np.linalg.norm(T_check_flange[:3, 3] - T_flange_target[:3, 3]) * 1000.0

    if pos_diff > config.IK_POS_TOLERANCE:
        return None, f"IK Inaccurate! Diff: {pos_diff:.3f}mm"

    for i, angle in enumerate(new_joints):
        min_lim, max_lim = config.JOINT_LIMITS[i]
        if angle < (min_lim - 0.1) or angle > (max_lim + 0.1):
            return None, f"Limit Hit J{i+1}"

    return list(new_joints), None