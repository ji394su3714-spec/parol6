# processor.py
import json
import os
import numpy as np
import kinematics 
from cam_settings import DEFAULT_IK_SEED

class PostProcessor:
    def __init__(self, tcp_manager):
        self.tcp_manager = tcp_manager

    def bake_trajectory(self, cam_tcp_matrices):
        """執行 IK 運算並匯出工業腳本"""
        if not cam_tcp_matrices:
            raise ValueError("沒有可用的 CAM 軌跡資料，請先執行幾何萃取。")

        print("\n[Processor] 開始烘焙六軸 IK 軌跡...")
        current_seed = np.array(DEFAULT_IK_SEED)
        spline_joints_list = []
        T_tcp_offset = self.tcp_manager.get_active_matrix()

        for i, T_tcp in enumerate(cam_tcp_matrices):
            T_target_tcp = T_tcp.copy()
            T_target_tcp[:3, 3] = T_target_tcp[:3, 3] / 1000.0 
            
            # 扣除工具偏移，還原法蘭盤矩陣
            T_flange_target = T_target_tcp @ np.linalg.inv(T_tcp_offset)
            
            new_joints, err = kinematics._core_inverse_kinematics(T_flange_target, current_seed)
            if new_joints is None:
                raise RuntimeError(f"IK 無解！在點位 {i} 遭遇死角。請嘗試調整 UI 上的加工姿態微調。")
                
            current_seed = new_joints 
            spline_joints_list.append(list(np.round(new_joints, 4)))

        # 打包 JSON 指令
        set_tcp_cmd = {
            "type": "SET_TCP",
            "name": self.tcp_manager.get_active_tool_data().get("name", "Tool"),
            "value": self.tcp_manager.current_index,
            "active": True
        }

        approach_ptp = {
            "type": "PTP",               
            "name": "Safe approach",            
            "joints": spline_joints_list[0], 
            "speed": 30.0,                    
            "accel": 50.0,
            "blend": "FINE",
            "active": True
        }

        cam_path_block = {
            "type": "CAM_PATH",               
            "name": "CAM_軌跡_001",            
            "point_count": len(spline_joints_list),
            "speed": 30.0,                    
            "accel": 50.0,
            "active": True,
            "recorded_base_matrix": np.eye(4).tolist(),
            "path_data": spline_joints_list   
        }

        output_file = "parol_cam_export.json"
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump([set_tcp_cmd, approach_ptp, cam_path_block], f, indent=4, ensure_ascii=False)

        print(f"[Success] 烘焙完成！檔案儲存至: {os.path.abspath(output_file)}")
        
        # 回傳起始姿態供 UI 預覽
        return spline_joints_list[0]