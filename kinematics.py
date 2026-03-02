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

def inverse_kinematics(target_matrix, seed_joints):
    """
    使用【阻尼最小平方法 (Damped Least Squares)】進行迭代求解
    """
    current_joints = np.array(seed_joints, dtype=float)
    
    max_iter = 50       
    tolerance = 1e-5    
    lambda_val = 0.01   

    target_pos = target_matrix[:3, 3]
    target_rot = target_matrix[:3, :3]

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
        
        for i in range(6):
            min_lim, max_lim = config.JOINT_LIMITS[i]
            if current_joints[i] < min_lim: current_joints[i] = min_lim
            if current_joints[i] > max_lim: current_joints[i] = max_lim

    final_error = np.linalg.norm(error_vector)
    if final_error < 0.1: 
        return current_joints, final_error
        
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