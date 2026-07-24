# processor.py
import numpy as np
import kinematics 

DEFAULT_IK_SEED = [0.0, 0.0, 30.0, 0.0, 90.0, 0.0]
class PostProcessor:
    def __init__(self, tcp_manager):
        self.tcp_manager = tcp_manager

    def bake_trajectory(self, tagged_path, min_step_mm=0.0, cut_speed=30, cut_accel=100, move_speed=100, move_accel=100): 
        if not tagged_path:
            raise ValueError("沒有可用的 CAM 軌跡資料。")

        #print("\n[Processor] 開始編譯六軸 IK 軌跡與分段腳本...")
        current_seed = np.array(DEFAULT_IK_SEED)
        T_tcp_offset = self.tcp_manager.get_active_matrix()
        
        base_matrix_list = np.eye(4).tolist()

        script_data = [{
            "type": "SET_TCP",
            "name": self.tcp_manager.get_active_tool_data().get("name", "Tool"),
            "value": self.tcp_manager.current_index
        }]

        all_joints_for_preview = []
        cut_segment_count = 1

        for seg in tagged_path:
            if seg['type'] in ['RETRACT', 'APPROACH', 'PTP_APPROACH', 'PLUNGE']:
                T_target = seg['matrix'].copy()
                T_target[:3, 3] /= 1000.0 
                T_flange = T_target @ np.linalg.inv(T_tcp_offset)
                
                new_joints, err = kinematics._core_inverse_kinematics(T_flange, current_seed)
                if new_joints is None: raise RuntimeError(f"IK 無解！發生在 {seg['type']} 點。")
                current_seed = new_joints
                joints_list = list(np.round(new_joints, 3))
                
                cmd_type = "PTP" if seg['type'] == 'PTP_APPROACH' else "LIN"
                name_str = "Safe approach" if seg['type'] == 'PTP_APPROACH' else seg['type']

                # 依據路徑類型套用對應的速度與加速度
                if seg['type'] == 'PLUNGE':
                    spd = cut_speed
                    acc = cut_accel
                else:
                    spd = move_speed
                    acc = move_accel

                script_data.append({
                    "type": cmd_type,
                    "name": name_str,
                    "joints": joints_list,
                    "speed": spd, 
                    "accel": acc,
                    "recorded_base_matrix": base_matrix_list
                })
                all_joints_for_preview.append(joints_list)

            elif seg['type'] == 'CUT':
                path_data = []
                for T_tcp in seg['matrices']:
                    T_target = T_tcp.copy()
                    T_target[:3, 3] /= 1000.0 
                    T_flange = T_target @ np.linalg.inv(T_tcp_offset)
                    
                    new_joints, err = kinematics._core_inverse_kinematics(T_flange, current_seed)
                    if new_joints is None: raise RuntimeError("IK 無解！發生在切削點。")
                    current_seed = new_joints
                    path_data.append(list(np.round(new_joints, 3)))
                
                script_data.append({
                    "type": "CAM_PATH",
                    "name": f"CUT_SEGMENT_{cut_segment_count}",
                    "point_count": len(path_data),
                    "speed": cut_speed,  
                    "accel": cut_accel,  
                    "recorded_base_matrix": base_matrix_list,
                    "path_data": path_data
                })
                all_joints_for_preview.extend(path_data)
                cut_segment_count += 1

        print("[Success] IK 腳本分類編譯完成！")
        return all_joints_for_preview[0], script_data, all_joints_for_preview