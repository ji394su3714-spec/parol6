import numpy as np

# --- 視窗設定 ---
APP_TITLE = "PAROL6 Robot Controller"

# --- 模型路徑 ---
URDF_PATH = 'assets/urdf/urdf.urdf'

# 依照順序：Base, Link1, Link2, Link3, Link4, Link5, Link6
STL_FILES = [
    'base_link_1.STL', 'Link1.STL', 'Link2.STL', 'Link3.STL', 'Link4.STL', 'Link5.STL', 'Link6.STL'
]

URDF_PARAMS = [
    # Joint 1 (Base -> Link1)
    {'xyz': [0, 0, 0.06], 'rpy': [0 , 0, -3.1416], 'axis': 'z', 'invert': True},
    # Joint 2 (Link1 -> Link2)
    {'xyz': [0, 0.0234, 0.0505], 'rpy': [-1.5708, 0, 1.5708], 'axis': 'z'},
    # Joint 3 (Link2 -> Link3)
    {'xyz': [0.0002, -0.18, 0], 'rpy': [-3.1416, 0, 0], 'axis': 'z', 'invert': True},
    # Joint 4 (Link3 -> Link4)
    {'xyz': [0.071, 0.0435, 0], 'rpy': [3.1416, -1.5708, 3.1416], 'axis': 'z'},
    # Joint 5 (Link4 -> Link5)
    {'xyz': [0, 0, -0.1053], 'rpy': [0, -1.5708, 3.1416], 'axis': 'z', 'invert': True},
    # Joint 6 (Link5 -> Link6)
    {'xyz': [-0.0741, 0, 0], 'rpy': [3.1416, 1.5708, 0], 'axis': 'z', 'invert': True}
]

IK_POS_TOLERANCE = 0.05  # 設定為 0.05 mm (比 0.1 更嚴格)
IK_ROT_TOLERANCE = 0.2   # 設定為 0.2 度 (角度誤差)

JOINT_LIMITS = [
    (-165, 165), (-50, 60), (-60, 70), (-95, 185), (-110, 110), (-120, 210)   
]

BASE_MESH_OFFSET = {'xyz': [0, 0, 0], 'rpy': [0, 0, 3.1416]}

# 單位轉換：URDF(公尺)
SCALE_FACTOR = 1

