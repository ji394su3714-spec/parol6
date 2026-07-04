import json
import os
import time
import numpy as np
import math
import queue
import threading
from PySide6.QtCore import QObject, QThread, Signal
from PySide6.QtWidgets import QFileDialog
from scipy.spatial.transform import Rotation as R
import kinematics


# --- 1. 雙緩衝預讀執行器 (Streaming Pipeline) ---
class StreamingPathExecutor(QThread):
    update_signal = Signal(list)
    finished_signal = Signal(float) 
    error_signal = Signal(str)
    log_signal = Signal(str)
    set_tcp_signal = Signal(int) # 換刀專屬訊號
    set_base_signal = Signal(int) # 基座切換訊號

    def __init__(self, waypoint_list, start_joints, serial_ref=None, loop=False):
        super().__init__()
        self.waypoint_list = waypoint_list
        self.start_joints = np.array(start_joints)
        self.serial_ref = serial_ref
        self.loop = loop 
        self._is_running = True
        self.point_queue = queue.Queue(maxsize=300) 
        self.producer_finished = False
        self.producer_error = False

    def run(self):
        prod_thread = threading.Thread(target=self._producer_task, daemon=True)
        prod_thread.start()

        self.log_signal.emit("[System] Compiling path, waiting for safe buffer level...")
        while self.point_queue.qsize() < 290 and not self.producer_finished and self._is_running: 
            if self.producer_error: return
            self.msleep(10)

        if not self._is_running or self.producer_error: return

        real_start_time = time.time()
        counter = 0
        interval = 0.010
        gui_skip_frames = 6
        self.update_signal.emit(list(self.start_joints))
        
        last_ik_joints = None

        while self._is_running:
            try:
                item = self.point_queue.get(timeout=0.1)
                
                # ==========================================
                # 🛡️ 狀態膠囊過濾網：嚴格攔截所有 dict，並強制 continue
                # ==========================================
                if isinstance(item, dict):
                    cmd_type = item.get("type")
                    
                    if cmd_type == "LOG":
                        self.log_signal.emit(item["msg"])
                        
                    elif cmd_type == "SET_TCP_CMD":
                        self.log_signal.emit(item["msg"])
                        self.set_tcp_signal.emit(item["tool_idx"]) # 觸發 UI 換刀與箭頭重繪
                        
                    elif cmd_type == "SET_BASE_CMD":
                        self.log_signal.emit(item["msg"])
                        self.set_base_signal.emit(item["base_idx"]) # 觸發 UI 換基座
                        
                    elif cmd_type == "EE_CMD":
                        self.log_signal.emit(item["msg"])
                        if self.serial_ref and self.serial_ref.is_connected:
                            if hasattr(self.serial_ref, 'wait_for_motion_complete'):
                                self.serial_ref.wait_for_motion_complete(timeout=10.0)
                            
                            self.serial_ref.send_command(item["cmd"])
                            
                            if hasattr(self.serial_ref, 'wait_for_ee_done'):
                                self.serial_ref.wait_for_ee_done(timeout=10.0)
                                
                    # 💡 最關鍵的一行：只要是字典膠囊，處理完絕對要跳過這個迴圈，不准往下送給手臂！
                    continue 
                # ==========================================

                # 如果不是膠囊，那就是真實的 6 軸座標，正常執行
                ik_joints = item
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
        
        pending_trajectory = None       
        pending_blend_str = 'FINE'      

        loop_count = 1  

        while self._is_running:
            if self.loop:
                self.point_queue.put({
                    "type": "LOG", 
                    "msg": f">> Start executing the path for loop {loop_count}..."
                }, block=True)
        
            for wp_idx, wp in enumerate(self.waypoint_list):
                if not self._is_running: return
                
                move_type = wp.get("move_type", "LIN")
                speed_factor = wp.get("speed_factor", 1.0)
                accel_factor = wp.get("accel_factor", 1.0)
                tcp_offset_mat = wp.get("tcp_offset_mat", np.eye(4))
                
                curr_trajectory = []
                msg = ""
                
                if move_type == "DELAY":
                    delay_time = wp.get("value", 0.0)
                    delay_steps = max(1, int(delay_time / 0.010))
                    curr_trajectory = [current_seed] * delay_steps
                    msg = f"wait {delay_time} seconds..."

                elif move_type == "SET_TCP":
                    if pending_trajectory is not None:
                        for ik_joints in pending_trajectory:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                        pending_trajectory = None
                    
                    tool_idx = int(wp.get("value", 0))
                    tool_name = wp.get("name", "") 
                    
                    self.point_queue.put({
                        "type": "SET_TCP_CMD",
                        "tool_idx": tool_idx,
                        "msg": f">> 執行換刀: 切換至 [{tool_idx}] {tool_name}" 
                    }, block=True)
                    
                    pending_blend_str = 'FINE' 
                    continue

                elif move_type == "SET_BASE":
                    if pending_trajectory is not None:
                        for ik_joints in pending_trajectory:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                        pending_trajectory = None
                    
                    base_idx = int(wp.get("value", 0))
                    base_name = wp.get("name", "") 
                    
                    self.point_queue.put({
                        "type": "SET_BASE_CMD",
                        "base_idx": base_idx,
                        "msg": f">> 執行基座切換: 切換至 [{base_idx}] {base_name}" 
                    }, block=True)
                    
                    pending_blend_str = 'FINE' 
                    continue

                elif move_type == "I/O":
                    if pending_trajectory is not None:
                        for ik_joints in pending_trajectory:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                        pending_trajectory = None
                    
                    ee_type = wp.get("action_type", "DIGITAL")
                    ee_val = int(wp.get("value", 0))
                    self.point_queue.put({
                        "type": "EE_CMD",
                        "cmd": f"<EE,{ee_type},{ee_val}>",
                        "msg": f">> 執行工具動作: {ee_type} -> {ee_val}"
                    }, block=True)
                    
                    pending_blend_str = 'FINE' 
                    continue
                    
                else:
                    target_joints = np.array(wp.get("target_joints", current_seed))
                    
                    if move_type == "PTP":
                        exact_traj, t_tot, msg = kinematics.TrajectoryMathEngine.calculate_ptp_trajectory(
                            current_seed, target_joints, speed_factor, accel_factor
                        )
                        curr_trajectory = exact_traj if exact_traj is not None else []
                    elif move_type == "LIN":
                        exact_traj, t_tot, msg = kinematics.TrajectoryMathEngine.calculate_lin_trajectory(
                            current_seed, target_joints, tcp_offset_mat, speed_factor, accel_factor
                        )
                        curr_trajectory = exact_traj if exact_traj is not None else []
                    elif move_type == "CIRC":
                        if "aux_joints" not in wp:
                            self.error_signal.emit(f"Waypoint {wp_idx+1} failed: CIRC missing AUX")
                            self.producer_error = True
                            return
                        exact_traj, t_tot, msg = kinematics.TrajectoryMathEngine.calculate_circ_trajectory(
                            current_seed, wp["aux_joints"], target_joints, tcp_offset_mat, speed_factor, accel_factor
                        )
                        curr_trajectory = exact_traj if exact_traj is not None else []
                    else:
                        self.error_signal.emit(f"Unsupported move type {move_type}")
                        self.producer_error = True
                        return
                    
                curr_trajectory = [list(pt) for pt in curr_trajectory]

                if len(curr_trajectory) == 0:
                    self.error_signal.emit(f"Waypoint {wp_idx+1} failed: {msg}")
                    self.producer_error = True
                    return
                    
                if msg and msg != "SUCCESS" and not msg.startswith("wait"):
                    self.log_signal.emit(f"Waypoint {wp_idx+1}: {msg}")

                if pending_trajectory is not None:
                    N = 0
                    if pending_blend_str != 'FINE' and move_type != 'DELAY':
                        try:
                            pct = float(pending_blend_str.replace('%', '')) / 100.0
                            safe_pct = min(max(pct, 0.0), 1.0)
                            N = int(min(len(pending_trajectory), len(curr_trajectory)) * safe_pct)
                        except ValueError:
                            pass
                    
                    if N > 0:
                        M = len(pending_trajectory)
                        arr_A = np.array(pending_trajectory[M - N : M])
                        arr_B = np.array(curr_trajectory[:N])
                        vertex = np.array(pending_trajectory[-1])
                        
                        blended_section = arr_A + arr_B - vertex
                        
                        for ik_joints in pending_trajectory[: M - N]:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                            
                        pending_trajectory = blended_section.tolist() + curr_trajectory[N:]
                        
                    else:
                        for ik_joints in pending_trajectory:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                        pending_trajectory = curr_trajectory
                        
                    self.msleep(1)
                else:
                    pending_trajectory = curr_trajectory

                pending_blend_str = wp.get('blend', 'FINE')
                
                if len(pending_trajectory) > 0:
                    current_seed = pending_trajectory[-1]

                time.sleep(0.001)

            if not self.loop:
                break
                
            loop_count += 1 
            
        if pending_trajectory is not None:
            for ik_joints in pending_trajectory:
                if not self._is_running: return
                self.point_queue.put(ik_joints, block=True)
                
        self.producer_finished = True


# --- 3. 總管大人 PathManager ---
class PathManager(QObject):
    log_signal = Signal(str)
    joint_update_signal = Signal(list)
    list_update_signal = Signal()
    file_loaded_signal = Signal(str) # 新增這行：用來廣播檔案名稱
    
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

    def _get_next_point_number(self):
        """計算下一個實體動作點位 (PTP, LIN, CIRC) 的 Point 序號"""
        count = 0
        for wp in self.waypoints:
            if wp.get('type') in ['PTP', 'LIN', 'CIRC']:
                count += 1
        return count + 1

    def record_point(self, current_joints, move_type="PTP", speed=50.0, accel=50.0):
        aux = None
        if self.temp_aux_joints is not None:
            aux = [round(j, 4) for j in self.temp_aux_joints]
            move_type = "CIRC"
            self.temp_aux_joints = None

        pt_num = self._get_next_point_number()
        name = f"Point {pt_num}"
        
        T_flange_world = kinematics.forward_kinematics(current_joints)
        
        # 呼叫 GUI 裡的 base_manager，取得錄製當下的基座矩陣
        recorded_base_mat = np.eye(4)
        if hasattr(self.parent_widget, 'base_manager'):
            recorded_base_mat = self.parent_widget.base_manager.get_active_matrix()
        
        data = {
            "name": name,
            "joints": [round(j, 4) for j in current_joints],
            "aux_joints": aux,
            "cartesian_flange": T_flange_world.tolist(), 
            "recorded_base_matrix": recorded_base_mat.tolist(), 
            "type": move_type,
            "speed": float(speed), 
            "accel": float(accel),
            "blend": "FINE", 
            "active": True,
            "note": ""
        }
        self.waypoints.append(data)
        self.list_update_signal.emit()
        
        msg = f"Recorded: {name} [{move_type}]"
        if move_type == "CIRC": msg += " (with AUX point)"
        self.log_signal.emit(msg)

    def update_point_at_index(self, index, current_joints):
        """將目前手臂的座標，覆蓋掉指定的實體動作點位"""
        if 0 <= index < len(self.waypoints):
            wp = self.waypoints[index]
            # 檢查是不是帶有座標的實體點位
            if wp.get('type') in ['PTP', 'LIN', 'CIRC']:
                # 重新寫入當前座標 (四捨五入到小數點後 4 位)
                wp['joints'] = [round(j, 4) for j in current_joints]
                
                # 通知 UI 與 3D 預覽線重新繪製
                self.list_update_signal.emit()
                
                # 印出成功通知
                self.log_signal.emit(f">>> UPDATED: [{wp.get('name')}] position updated to current pose.")
            else:
                self.log_signal.emit(f"[WARNING] 無法更新座標：{wp.get('type')} 不是實體動作點位。")

    def insert_waypoint(self, index, waypoint_data):
        """統一的插入接口：將新點位插入到指定的 index (即原本點位的前面)"""
        # Python 的 list.insert(index, obj) 原生就是插在該 index 的前面
        self.waypoints.insert(index, waypoint_data)
        
        # 如果你原本的架構有 _renumber_points，可以保留呼叫
        if hasattr(self, '_renumber_points'):
            self._renumber_points()
            
        self.list_update_signal.emit()
        self.log_signal.emit(f">>> INSERTED: [{waypoint_data['type']}] at line {index + 1}")

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

    def record_tool_action(self, action_dict):
        """將夾爪動作寫入教導清單中"""
        idx = len(self.waypoints) + 1
        name = f"EE: {action_dict.get('note', 'Action')}"
        data = {
            "name": name,
            "type": "I/O",
            "action_type": action_dict['action_type'],
            "value": action_dict['value'],
            "active": True,
            "note": action_dict.get('note', '')
        }
        self.waypoints.append(data)
        self.list_update_signal.emit()
        self.log_signal.emit(f"Recorded Tool Action: {name}")

    def record_tcp_switch(self, tool_index, tool_name):
        """錄製 TCP (刀具) 切換指令"""
        wp = {
            "type": "SET_TCP",
            "value": tool_index,
            "name": f"Tool: {tool_name}",
            "active": True
        }
        self.waypoints.append(wp)
        self.list_update_signal.emit()
        self.log_signal.emit(f">>> RECORDED: [SET_TCP] Switched to {tool_name}")

    def update_tcp_command(self, index, new_tool_idx, new_tool_name):
        """專門用來更新已錄製的 SET_TCP 指令"""
        if 0 <= index < len(self.waypoints):
            wp = self.waypoints[index]
            if wp.get('type') == "SET_TCP":
                wp['value'] = new_tool_idx
                wp['name'] = f"Tool: {new_tool_name}"
                
                # 發送訊號刷新 UI
                self.list_update_signal.emit()
                self.log_signal.emit(f">>> UPDATED: [SET_TCP] changed to {new_tool_name}")

    def delete_point(self, index):
        if 0 <= index < len(self.waypoints):
            removed = self.waypoints.pop(index)
            point_name = removed.get('name', f"Type: {removed.get('type', 'Unknown')}")
            
            self.log_signal.emit(f">>> DELETED: {point_name} (Line {index + 1})")
            self.list_update_signal.emit()

    def delete_all_points(self):
        self.waypoints.clear()
        self.list_update_signal.emit()
        self.log_signal.emit("All waypoints deleted.")

    def update_aux_joints(self, index, joints):
        """後期編輯：更新指定點位的輔助座標"""
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
        """刪除點位後重新編號 (只對實體動作點位生效)"""
        count = 1
        for pt in self.waypoints:
            if pt.get('type') in ['PTP', 'LIN', 'CIRC']:
                pt['name'] = f"Point {count}"
                count += 1

    def save_to_file(self):
        filename, _ = QFileDialog.getSaveFileName(self.parent_widget, "Save Path", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.waypoints, f, indent=4)
                
                base_name = os.path.basename(filename)
                self.log_signal.emit(f"[System] UPDATED: Waypoints successfully saved to {base_name}")
                
                if hasattr(self, 'file_loaded_signal'):
                    self.file_loaded_signal.emit(base_name)
                
            except Exception as e:
                self.log_signal.emit(f"[ERROR] Save failed: {e}")

    def load_from_file(self):
        filename, _ = QFileDialog.getOpenFileName(self.parent_widget, "Load Path", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                
                if isinstance(data, list):
                    self.waypoints = data
                    self.list_update_signal.emit()
                    
                    base_name = os.path.basename(filename)
                    self.log_signal.emit(f"[System] UPDATED: Waypoints successfully loaded from {base_name}")
                    
                    if hasattr(self, 'file_loaded_signal'):
                        self.file_loaded_signal.emit(base_name)
                else:
                    self.log_signal.emit("[ERROR] Invalid file format: Expected a list of waypoints.")
                    
            except Exception as e:
                self.log_signal.emit(f"[ERROR] Load failed: {e}")

    def get_trajectory_preview(self, initial_tcp_offset=None):
        trajectory_points = []
        valid_physical_pts = []
        
        def get_tcp_matrix(tool_idx):
            flange_offset = np.eye(4)
            flange_offset[:3, :3] = R.from_euler('x', -90, degrees=True).as_matrix()
            if hasattr(self, 'parent_widget') and hasattr(self.parent_widget, 'tcp_manager'):
                tcp_mgr = self.parent_widget.tcp_manager
                if 0 <= tool_idx < len(tcp_mgr.tools):
                    vals = tcp_mgr.tools[tool_idx]["values"]
                    x, y, z, rx, ry, rz = vals
                    user_mat = np.eye(4)
                    user_mat[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
                    user_mat[:3, 3] = [x/1000.0, y/1000.0, z/1000.0]
                    return flange_offset @ user_mat
            return flange_offset

        current_tcp = get_tcp_matrix(0)
        current_base_mat = np.eye(4)
        base_mgr = getattr(self.parent_widget, 'base_manager', None)
        
        last_valid_actual_joints = None
        
        for wp in self.waypoints:
            if not wp.get('active', True):
                continue
                
            m_type = wp.get('type')
            
            if m_type == "SET_TCP":
                tool_idx = int(wp.get("value", 0))
                current_tcp = get_tcp_matrix(tool_idx)
                
            elif m_type == "SET_BASE":
                base_idx = int(wp.get("value", 0))
                if base_mgr and 0 <= base_idx < len(base_mgr.bases):
                    current_base_mat = base_mgr.get_matrix(base_idx)
                        
            elif m_type in ['PTP', 'LIN', 'CIRC'] and 'joints' in wp:
                target_joints = wp['joints']
                
                T_flange_world_old = np.array(wp['cartesian_flange']) if 'cartesian_flange' in wp else kinematics.forward_kinematics(target_joints)
                recorded_base_mat = np.array(wp['recorded_base_matrix']) if 'recorded_base_matrix' in wp else np.eye(4)
                
                # ==========================================
                # 陣列加工空間轉移 (含 180 度死角破解)
                # ==========================================
                if not np.allclose(current_base_mat, recorded_base_mat, atol=1e-4):
                    T_user = np.linalg.inv(recorded_base_mat) @ T_flange_world_old
                    T_flange_world_new = current_base_mat @ T_user
                    
                    # 破解法：提取基座旋轉差，建立「智慧種子」
                    T_rel = current_base_mat @ np.linalg.inv(recorded_base_mat)
                    rz_diff = R.from_matrix(T_rel[:3, :3]).as_euler('xyz', degrees=True)[2]
                    
                    smart_seed = list(target_joints)
                    smart_seed[0] += rz_diff
                    smart_seed[0] = (smart_seed[0] + 180.0) % 360.0 - 180.0 # 保持在 -180 到 180 之間
                    
                    # 1. 優先使用轉向後的智慧種子
                    new_j, _ = kinematics.inverse_kinematics(T_flange_world_new, smart_seed)
                    
                    # 2. 如果失敗，退回使用連續性的前一點角度
                    if new_j is None and last_valid_actual_joints is not None:
                        new_j, _ = kinematics.inverse_kinematics(T_flange_world_new, last_valid_actual_joints)
                        
                    actual_joints = list(new_j) if new_j is not None else target_joints
                else:
                    actual_joints = target_joints

                last_valid_actual_joints = actual_joints

                actual_aux_joints = wp.get('aux_joints')
                if m_type == 'CIRC' and actual_aux_joints is not None:
                    if not np.allclose(current_base_mat, recorded_base_mat, atol=1e-4):
                        T_aux_world_old = kinematics.forward_kinematics(actual_aux_joints)
                        T_aux_user = np.linalg.inv(recorded_base_mat) @ T_aux_world_old
                        T_aux_world_new = current_base_mat @ T_aux_user
                        
                        smart_aux_seed = list(actual_aux_joints)
                        smart_aux_seed[0] += rz_diff
                        smart_aux_seed[0] = (smart_aux_seed[0] + 180.0) % 360.0 - 180.0
                        
                        new_aux_j, _ = kinematics.inverse_kinematics(T_aux_world_new, smart_aux_seed)
                        if new_aux_j is None:
                            new_aux_j, _ = kinematics.inverse_kinematics(T_aux_world_new, actual_joints)
                        if new_aux_j is not None:
                            actual_aux_joints = list(new_aux_j)

                valid_physical_pts.append({
                    'joints': actual_joints,
                    'type': m_type,
                    'aux_joints': actual_aux_joints,
                    'tcp': current_tcp
                })

        # 2. 開始依照綁定並轉換後的座標畫線
        if len(valid_physical_pts) >= 2:
            for i in range(len(valid_physical_pts) - 1):
                p_start = valid_physical_pts[i]
                p_end = valid_physical_pts[i+1]
                
                j_start = np.array(p_start['joints'])
                j_end = np.array(p_end['joints'])
                m_type = p_end['type']
                aux_joints = p_end['aux_joints']
                active_tcp = p_end['tcp']
                
                steps = 20
                if m_type in ['PTP']:
                    for t in np.linspace(0, 1, steps):
                        interp = j_start + t * (j_end - j_start)
                        T_tcp = kinematics.forward_kinematics(interp) @ active_tcp
                        trajectory_points.append(T_tcp[:3, 3])
                        
                elif m_type == 'CIRC' and aux_joints is not None:
                    T_s = kinematics.forward_kinematics(j_start) @ active_tcp
                    T_a = kinematics.forward_kinematics(aux_joints) @ active_tcp
                    T_e = kinematics.forward_kinematics(j_end) @ active_tcp
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
                    T_s = kinematics.forward_kinematics(j_start) @ active_tcp
                    T_e = kinematics.forward_kinematics(j_end) @ active_tcp
                    xs, xe = T_s[:3, 3], T_e[:3, 3]
                    for t in np.linspace(0, 1, steps):
                        trajectory_points.append(xs + t * (xe - xs))
                        
        return trajectory_points

    def execute_streaming_path(self, active_points, start_joints, tcp_offset_mat, loop, global_speed, global_accel, serial_ref, callbacks):
        """編譯執行路徑：無懼 180 度死角的終極陣列加工引擎"""
        wp_list = []
        current_tcp_mat = tcp_offset_mat 
        tcp_mgr = self.parent_widget.tcp_manager 
        
        current_base_mat = np.eye(4) 
        base_mgr = getattr(self.parent_widget, 'base_manager', None)
        
        last_valid_actual_joints = None # 記憶連續角度
        
        for pt in active_points:
            move_type = pt.get('type', 'LIN')  
            
            if move_type == "SET_BASE":
                base_idx = int(pt.get("value", 0))
                if base_mgr and 0 <= base_idx < len(base_mgr.bases):
                    current_base_mat = base_mgr.get_matrix(base_idx)
            
            elif move_type == "SET_TCP":
                tool_idx = int(pt.get("value", 0))
                if 0 <= tool_idx < len(tcp_mgr.tools):
                    vals = tcp_mgr.tools[tool_idx]["values"]
                    x, y, z, rx, ry, rz = vals
                    user_mat = np.eye(4)
                    user_mat[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
                    user_mat[:3, 3] = [x/1000.0, y/1000.0, z/1000.0]
                    flange_offset = np.eye(4)
                    flange_offset[:3, :3] = R.from_euler('x', -90, degrees=True).as_matrix()
                    current_tcp_mat = flange_offset @ user_mat

            pt_speed = pt.get('speed', global_speed * 100) / 100.0
            pt_accel = pt.get('accel', global_accel * 100) / 100.0
            
            target_joints = pt.get('joints', [])
            
            # ==========================================
            # 執行層的防護網與智慧預旋轉
            # ==========================================
            if move_type in ["PTP", "LIN", "CIRC"] and len(target_joints) == 6:
                T_flange_world_old = np.array(pt['cartesian_flange']) if 'cartesian_flange' in pt else kinematics.forward_kinematics(target_joints)
                recorded_base_mat = np.array(pt['recorded_base_matrix']) if 'recorded_base_matrix' in pt else np.eye(4)
                
                if not np.allclose(current_base_mat, recorded_base_mat, atol=1e-4):
                    T_user = np.linalg.inv(recorded_base_mat) @ T_flange_world_old
                    T_flange_world_new = current_base_mat @ T_user
                    
                    # 提取基座旋轉差，建立「智慧種子」
                    T_rel = current_base_mat @ np.linalg.inv(recorded_base_mat)
                    rz_diff = R.from_matrix(T_rel[:3, :3]).as_euler('xyz', degrees=True)[2]
                    
                    smart_seed = list(target_joints)
                    smart_seed[0] += rz_diff
                    smart_seed[0] = (smart_seed[0] + 180.0) % 360.0 - 180.0
                    
                    new_joints, _ = kinematics.inverse_kinematics(T_flange_world_new, smart_seed)
                    if new_joints is None and last_valid_actual_joints is not None:
                        new_joints, _ = kinematics.inverse_kinematics(T_flange_world_new, last_valid_actual_joints)
                        
                    if new_joints is not None:
                        target_joints = list(new_joints)
                
                last_valid_actual_joints = target_joints

            wp = {
                "move_type": move_type,
                "name": pt.get('name', ''), 
                "target_joints": target_joints, 
                "tcp_offset_mat": current_tcp_mat, 
                "speed_factor": pt_speed,
                "accel_factor": pt_accel,
                "value": pt.get('value', 0.0), 
                "blend": pt.get('blend', 'FINE')
            }
            if move_type == "CIRC" and 'aux_joints' in pt:
                aux_j = pt['aux_joints']
                if not np.allclose(current_base_mat, recorded_base_mat, atol=1e-4):
                    T_aux_world_old = kinematics.forward_kinematics(aux_j)
                    T_aux_user = np.linalg.inv(recorded_base_mat) @ T_aux_world_old
                    T_aux_world_new = current_base_mat @ T_aux_user
                    
                    smart_aux_seed = list(aux_j)
                    smart_aux_seed[0] += rz_diff
                    smart_aux_seed[0] = (smart_aux_seed[0] + 180.0) % 360.0 - 180.0
                    
                    new_aux_j, _ = kinematics.inverse_kinematics(T_aux_world_new, smart_aux_seed)
                    if new_aux_j is None:
                        new_aux_j, _ = kinematics.inverse_kinematics(T_aux_world_new, target_joints)
                        
                    if new_aux_j is not None: 
                        aux_j = list(new_aux_j)
                wp["aux_joints"] = aux_j
                
            if move_type == "I/O":
                wp["action_type"] = pt.get("action_type", "DIGITAL")
                
            wp_list.append(wp)

        self.worker = StreamingPathExecutor(
            waypoint_list=wp_list,
            start_joints=start_joints,
            serial_ref=serial_ref,
            loop=loop 
        )  

        self.worker.update_signal.connect(callbacks['update'])
        self.worker.error_signal.connect(callbacks['error'])
        self.worker.log_signal.connect(callbacks['log'])
        self.worker.finished_signal.connect(callbacks['finished'])
        if 'set_tcp' in callbacks:
            self.worker.set_tcp_signal.connect(callbacks['set_tcp'])
        if 'set_base' in callbacks:
            self.worker.set_base_signal.connect(callbacks['set_base'])

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
            if hasattr(self.serial_manager, 'ok_semaphore'):
                self.serial_manager.ok_semaphore.release()
                
            self.serial_manager.motion_done_event.set()
            
        if self.worker and self.worker.isRunning():
            self.worker.wait() 
            
        self.worker = None
        self.log_signal.emit("([STOP]) Execution Halted.")