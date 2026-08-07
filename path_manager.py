# path_manager.py
import copy
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


# =========================================================
# [1] 雙緩衝串流執行器 (Streaming Pipeline)
# =========================================================
class StreamingPathExecutor(QThread):
    """
    背景軌跡執行緒：
    包含生產者 (計算軌跡) 與消費者 (精準時序發送指令)。
    """
    update_signal = Signal(list)
    finished_signal = Signal(float) 
    error_signal = Signal(str)
    log_signal = Signal(str)
    set_tcp_signal = Signal(int) 
    set_base_signal = Signal(int) 

    def __init__(self, waypoint_list, start_joints, serial_ref=None, loop=False):
        super().__init__()
        self.waypoint_list = waypoint_list
        self.start_joints = np.array(start_joints)
        self.serial_ref = serial_ref
        self.loop = loop 
        self._is_running = True
        
        self.point_queue = queue.Queue(maxsize=100) 
        self.producer_finished = False
        self.producer_error = False
        self._is_paused = False
        self._is_estopped = False 
        
        if self.serial_ref and hasattr(self.serial_ref, 'estop_state_signal'):
            self.serial_ref.estop_state_signal.connect(self._handle_estop_signal)

    def _handle_estop_signal(self, is_latched):
        self._is_estopped = is_latched
        if is_latched:
            self._is_running = False

    def _flush_pending_trajectory(self, trajectory):
        """將暫存的軌跡倒進傳輸佇列中"""
        if trajectory is None: return
        for ik_joints in trajectory:
            if not self._is_running: break
            self.point_queue.put(ik_joints, block=True)

    def run(self):
        """消費者主迴圈：精準時序控管"""
        prod_thread = threading.Thread(target=self._producer_task, daemon=True)
        prod_thread.start()

        # 暖機等待：緩衝區填滿 99 個點或生產完畢
        while self.point_queue.qsize() < 99 and not self.producer_finished and self._is_running: 
            if self.producer_error: return
            self.msleep(10)

        if not self._is_running or self.producer_error: 
            return

        real_start_time = time.time()
        counter = 0
        interval = 0.010
        gui_skip_frames = 3
        self.update_signal.emit(list(self.start_joints))
        last_ik_joints = None

        # 建立絕對時間基準
        absolute_target_time = time.perf_counter()

        while self._is_running:
            if self._is_estopped:
                break
                
            if getattr(self, '_is_paused', False):
                time.sleep(0.05)
                absolute_target_time = time.perf_counter() 
                continue 

            absolute_target_time += interval 
            
            try:
                item = self.point_queue.get(timeout=0.1)
                
                if isinstance(item, dict):
                    cmd_type = item.get("type")
                    if cmd_type == "LOG":
                        self.log_signal.emit(item.get("msg", ""))
                    elif cmd_type == "DELAY_CMD":
                        delay_time = float(item.get("value", 0.0))
                        absolute_target_time += delay_time
                        time.sleep(delay_time)
                    elif cmd_type == "SET_TCP_CMD":
                        self.log_signal.emit(item.get("msg", "執行: 設定 TCP 偏移"))
                        self.set_tcp_signal.emit(item["tool_idx"])
                    elif cmd_type == "SET_BASE_CMD":
                        self.log_signal.emit(item.get("msg", "執行: 設定 Base 座標系"))
                        self.set_base_signal.emit(item["base_idx"])
                    elif cmd_type == "EE_CMD":
                        self.log_signal.emit(item.get("msg", "執行: 末端效應器指令"))
                        if self.serial_ref and self.serial_ref.is_connected:
                            time.sleep(0.5) 
                            if hasattr(self.serial_ref, 'send_gripper'):
                                ee_val = 1 if "1" in item["cmd"] else 0
                                self.serial_ref.send_gripper(ee_val)
                            if hasattr(self.serial_ref, 'wait_for_ee_done'):
                                self.serial_ref.wait_for_ee_done(timeout=10.0)
                    continue

                ik_joints = item
                last_ik_joints = ik_joints
                
                if counter % gui_skip_frames == 0:
                    self.update_signal.emit(list(ik_joints))
                    
                if self.serial_ref and self.serial_ref.is_connected:
                    self.serial_ref.send_joints(list(ik_joints), speed_factor=1.0, move_mode=1, is_stream=True)
                    self.serial_ref.wait_for_ok(timeout=3.0)
                
                # 絕對時間鎖定
                while time.perf_counter() < absolute_target_time:
                    time.sleep(0)  
                    
                counter += 1

            except queue.Empty:
                if self.producer_finished: 
                    if last_ik_joints is not None:
                        self.update_signal.emit(list(last_ik_joints))
                    break
                else: 
                    if self.serial_ref and self.serial_ref.is_connected:
                        self.log_signal.emit("[Warning] CPU computing too slowly, buffer underflow! Arm paused and waiting...")

        if self._is_estopped:
            self.error_signal.emit("執行已強制中斷：偵測到硬體急停鎖死 (E-STOP)！")
            return  
            
        elif not self._is_running and not self.producer_finished:
            self.error_signal.emit("執行已由使用者手動中斷。")
            return  

        if self.serial_ref and self.serial_ref.is_connected and not self.producer_error:
            time.sleep(0.5)
                
        real_total_time = time.time() - real_start_time
        self.finished_signal.emit(real_total_time)

    def toggle_pause(self):
        self._is_paused = not self._is_paused
        if self._is_paused:
            self.log_signal.emit("[System] Pausing... Arm will decelerate smoothly.")
        else:
            self.log_signal.emit("[System] Resuming trajectory...")

    # =========================================================
    # 全新架構：神經中樞 (Dispatcher) 與三大處理器 (Processors)
    # =========================================================
    def _producer_task(self):
        """ 生產者中樞：只負責讀取清單並派發任務，極度乾淨 """
        # 將狀態變數升級為類別屬性，讓三大處理器可以共用
        self._prod_seed = list(self.start_joints)
        self._prod_pending_traj = None       
        self._prod_blend_str = 'FINE'      
        loop_count = 1  

        while self._is_running:
            if self.loop:
                self.point_queue.put({"type": "LOG", "msg": f">> Start executing loop {loop_count}..."})
        
            for wp_idx, wp in enumerate(self.waypoint_list):
                if not self._is_running: return
                
                move_type = wp.get("move_type", "LIN")
                display_label = f"[{move_type}] {wp.get('name', f'Point {wp_idx+1}')}"
                
                # --- 任務派發站 ---
                ok = True
                if move_type in ["SET_TCP", "SET_BASE", "I/O"]:
                    ok = self._process_aux_command(wp, move_type)
                elif move_type == "CAM_PATH":
                    ok = self._process_cam_path(wp)
                else:
                    ok = self._process_standard_motion(wp, move_type, display_label)

                # 如果處理器回報失敗或中止，直接阻斷整個生產者
                if not ok:
                    return 
                    
            if not self.loop:
                break
            loop_count += 1 
            
        self._flush_pending_trajectory(self._prod_pending_traj)
        self.producer_finished = True

    def _process_aux_command(self, wp, move_type):
        """ 處理器 A：處理輔助指令 (IO / TCP / BASE) """
        self._flush_pending_trajectory(self._prod_pending_traj)
        self._prod_pending_traj = None
        if not self._is_running: return False
            
        if move_type == "I/O":
            ee_type = wp.get("action_type", "DIGITAL")
            ee_val = int(wp.get("value", 0))
            self.point_queue.put({
                "type": "EE_CMD", 
                "cmd": f"<EE,{ee_type},{ee_val}>",
                "msg": f">> 執行工具動作: {ee_type} -> {ee_val}"
            }, block=True)
            
        elif move_type == "SET_TCP":
            self.point_queue.put({
                "type": "SET_TCP_CMD", 
                "tool_idx": int(wp.get("value", 0)),
                "msg": f">> 執行換刀: 切換至 [{wp.get('value')}] {wp.get('name', '')}" 
            }, block=True)
            self.point_queue.put({"type": "DELAY_CMD", "value": 0.5}, block=True)
            
        elif move_type == "SET_BASE":
            self.point_queue.put({
                "type": "SET_BASE_CMD", 
                "base_idx": int(wp.get("value", 0)),
                "msg": f">> 執行基座切換: 切換至 [{wp.get('value')}] {wp.get('name', '')}" 
            }, block=True)
            self.point_queue.put({"type": "DELAY_CMD", "value": 0.5}, block=True)
            
        self._prod_blend_str = 'FINE'
        return True

    def _process_cam_path(self, wp):
        """ 處理器 B：處理 CAM 軌跡包 (連續微小線段聚合) """
        self._flush_pending_trajectory(self._prod_pending_traj)
        self._prod_pending_traj = None
        if not self._is_running: return False

        path_points = wp.get("path_data", [])
        if not path_points:
            return True 

        current_tcp_mat = wp.get("tcp_offset_mat", np.eye(4))
        pt_speed_factor = wp.get("speed_factor", 0.3)
        pt_accel_factor = wp.get("accel_factor", 0.5)
        global_speed = wp.get("speed", 30.0)

        spline_buffer = []
        for pt_joints in path_points:
            spline_buffer.append({
                "target_joints": pt_joints, "joints": pt_joints, "speed": global_speed,
                "tcp_offset_mat": current_tcp_mat, "speed_factor": pt_speed_factor,
                "accel_factor": pt_accel_factor, "move_type": "SPLINE"
            })

        gen, t_tot, msg, total_N = kinematics.TrajectoryMathEngine.calculate_spline_trajectory(
            self._prod_seed, spline_buffer, interval=0.010
        )
        
        if gen is None:
            self.error_signal.emit(f"[CAM_PATH Block] failed: {msg}")
            self.producer_error = True
            return False

        if msg and msg != "SUCCESS":
            self.log_signal.emit(msg)
            
        try:
            for _ in range(total_N):
                if not self._is_running: return False
                pt = next(gen)
                self.point_queue.put(pt, block=True)
                self._prod_seed = pt
        except Exception as e:
            self.error_signal.emit(f"[CAM_PATH Block] IK Error: {e}")
            self.producer_error = True
            return False
        
        self._prod_blend_str = 'FINE'
        return True

    def _process_standard_motion(self, wp, move_type, display_label):
        """ 處理器 C：處理傳統軌跡與交融運算 (PTP, LIN, CIRC, DELAY) """
        gen, total_N, msg = None, 0, ""
        
        if move_type == "DELAY":
            delay_time = wp.get("value", 0.0)
            delay_steps = max(1, int(delay_time / 0.010))
            def delay_gen(seed, steps):
                for _ in range(steps): yield list(seed)
            gen = delay_gen(self._prod_seed, delay_steps)
            total_N = delay_steps
            msg = f"wait {delay_time} seconds..."
        else:
            speed_factor = wp.get("speed_factor", 1.0)
            accel_factor = wp.get("accel_factor", 1.0)
            tcp_offset_mat = wp.get("tcp_offset_mat", np.eye(4))
            target_joints = np.array(wp.get("target_joints", self._prod_seed))
            
            if move_type == "PTP":
                gen, t_tot, msg, total_N = kinematics.TrajectoryMathEngine.calculate_ptp_trajectory(
                    self._prod_seed, target_joints, speed_factor, accel_factor)
            elif move_type == "LIN":
                gen, t_tot, msg, total_N = kinematics.TrajectoryMathEngine.calculate_lin_trajectory(
                    self._prod_seed, target_joints, tcp_offset_mat, speed_factor, accel_factor)
            elif move_type == "CIRC":
                gen, t_tot, msg, total_N = kinematics.TrajectoryMathEngine.calculate_circ_trajectory(
                    self._prod_seed, wp["aux_joints"], target_joints, tcp_offset_mat, speed_factor, accel_factor)

        if gen is None:
            self.error_signal.emit(f"{display_label} failed: {msg}")
            self.producer_error = True
            return False

        if msg and msg != "SUCCESS" and not msg.startswith("wait"):
            self.log_signal.emit(f"{display_label}: {msg}")

        # --- 交融處理 (Blending) ---
        N_blend = 0
        if self._prod_pending_traj is not None and self._prod_blend_str != 'FINE' and move_type != 'DELAY':
            N_blend = min(len(self._prod_pending_traj), total_N)
        
        curr_head = []
        try:
            for _ in range(N_blend):
                curr_head.append(next(gen))
        except Exception as e:
            self.error_signal.emit(f"{display_label} failed: {e}")
            self.producer_error = True
            return False
        
        if self._prod_pending_traj is not None:
            M = len(self._prod_pending_traj)
            if N_blend > 0:
                arr_A = np.array(self._prod_pending_traj[M - N_blend : M])
                arr_B = np.array(curr_head)
                vertex = np.array(self._prod_pending_traj[-1])
                blended_section = (arr_A + arr_B - vertex).tolist()
                
                self._flush_pending_trajectory(self._prod_pending_traj[: M - N_blend])
                if not self._is_running: return False
                self._flush_pending_trajectory(blended_section)
                if not self._is_running: return False
            else:
                self._flush_pending_trajectory(self._prod_pending_traj)
                if not self._is_running: return False

        curr_blend_str = wp.get('blend', 'FINE') if move_type != 'DELAY' else 'FINE'
        H_hold = 0
        if curr_blend_str != 'FINE':
            try:
                curr_pct = float(curr_blend_str.replace('%', '')) / 100.0
                curr_pct = min(max(curr_pct, 0.0), 0.5)
                H_hold = int(total_N * curr_pct)
            except ValueError: pass
        
        H_hold = min(H_hold, total_N - N_blend)
        stream_count = total_N - N_blend - H_hold

        try:
            for _ in range(stream_count):
                if not self._is_running: return False
                pt = next(gen)
                self.point_queue.put(pt, block=True) 
                self._prod_seed = pt
                
            self._prod_pending_traj = []
            for _ in range(H_hold):
                pt = next(gen)
                self._prod_pending_traj.append(pt)
                self._prod_seed = pt
        except Exception as e:
            self.error_signal.emit(f"{display_label} failed: {e}")
            self.producer_error = True
            return False
        
        self._prod_blend_str = curr_blend_str
        return True


# =========================================================
# [2] 路徑與設定檔管理器 (Path Manager)
# =========================================================
class PathManager(QObject):
    log_signal = Signal(str)
    joint_update_signal = Signal(list)
    list_update_signal = Signal()
    file_loaded_signal = Signal(str) 
    
    # --- 2.1 初始化 ---
    def __init__(self, parent=None):
        super().__init__(parent)
        self.waypoints = []
        self.worker = None
        self.parent_widget = parent
        self.is_modified = False
        self.clipboard = []
        self.temp_aux_joints = None 
        self.serial_manager = parent.serial_manager if parent else None

        self._preview_cache = None
        self.list_update_signal.connect(self.invalidate_preview_cache)

    def invalidate_preview_cache(self):
        self._preview_cache = None

    def is_running(self):
        return hasattr(self, 'worker') and self.worker is not None and self.worker.isRunning()
        
    def _get_tcp_matrix(self, tool_idx):
        """取得指定刀具的法蘭偏移矩陣"""
        flange_offset = np.eye(4)
        flange_offset[:3, :3] = R.from_euler('x', -90, degrees=True).as_matrix()
        
        if self.parent_widget:
            tcp_mgr = self.parent_widget.tcp_manager
            if 0 <= tool_idx < len(tcp_mgr.tools):
                x, y, z, rx, ry, rz = tcp_mgr.tools[tool_idx]["values"]
                user_mat = np.eye(4)
                user_mat[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
                user_mat[:3, 3] = [x/1000.0, y/1000.0, z/1000.0]
                return flange_offset @ user_mat
        return flange_offset

    # --- 2.2 點位增刪改查 ---
    def set_aux_point(self, joints):
        self.temp_aux_joints = list(joints)
        self.log_signal.emit(">> [CIRC] AUX point saved! Move to END point and press Record.")
        self.is_modified = True

    def update_point_at_index(self, index, current_joints):
        if 0 <= index < len(self.waypoints):
            wp = self.waypoints[index]
            if wp.get('type') in ['PTP', 'LIN', 'CIRC']:
                wp['joints'] = [round(j, 4) for j in current_joints]
                self.list_update_signal.emit()
                self.log_signal.emit(f">>> UPDATED: [{wp.get('name')}] position updated to current pose.")
                self.is_modified = True

    def insert_waypoint(self, index, waypoint_data):
        self.waypoints.insert(index, waypoint_data)
        self._renumber_points()
        self.list_update_signal.emit()
        self.log_signal.emit(f">>> INSERTED: [{waypoint_data['type']}] at line {index + 1}")
        self.is_modified = True

    def update_special_point(self, index, pt_type, new_value, new_name):
        if 0 <= index < len(self.waypoints):
            wp = self.waypoints[index]
            if wp.get('type') == pt_type:
                wp['value'] = new_value
                wp['name'] = new_name
                self.is_modified = True
                self.list_update_signal.emit()
                self.log_signal.emit(f">>> UPDATED: [{pt_type}] at line {index + 1} to {new_name}")

    def delete_point(self, index):
        if 0 <= index < len(self.waypoints):
            removed = self.waypoints.pop(index)
            point_name = removed.get('name', f"Type: {removed.get('type', 'Unknown')}")
            self.log_signal.emit(f">>> DELETED: {point_name} (Line {index + 1})")
            self._renumber_points()
            self.list_update_signal.emit()
            self.is_modified = True

    def delete_all_points(self):
        self.waypoints.clear()
        self.list_update_signal.emit()
        self.log_signal.emit("All waypoints deleted.")
        self.is_modified = True

    def update_aux_joints(self, index, joints):
        if 0 <= index < len(self.waypoints):
            self.waypoints[index]['type'] = 'CIRC'
            self.waypoints[index]['aux_joints'] = [round(j, 4) for j in joints]
            self.list_update_signal.emit()
            self.log_signal.emit(f"Updated: {self.waypoints[index]['name']} (AUX Pos Added)")
            self.is_modified = True

    def toggle_point_active(self, index):
        if 0 <= index < len(self.waypoints):
            current_state = self.waypoints[index].get('active', True)
            self.waypoints[index]['active'] = not current_state
            self.list_update_signal.emit()
            self.is_modified = True

    def _renumber_points(self):
        count = 1
        for pt in self.waypoints:
            if pt.get('type') in ['PTP', 'LIN', 'CIRC']:
                pt['name'] = f"Point {count}"
                count += 1

    # --- 2.3 剪貼簿操作 ---
    def copy_points(self, indices):
        valid_indices = [i for i in indices if 0 <= i < len(self.waypoints)]
        valid_indices.sort()
        if not valid_indices: return
        self.clipboard = [copy.deepcopy(self.waypoints[i]) for i in valid_indices]
        self.log_signal.emit(f"[System] Copied {len(self.clipboard)} waypoints.")

    def paste_points(self, index=-1):
        if not getattr(self, 'clipboard', []): return
        target_idx = index if index >= 0 else len(self.waypoints)
        for wp in self.clipboard:
            new_wp = copy.deepcopy(wp)
            self.waypoints.insert(target_idx, new_wp)
            target_idx += 1
        self._renumber_points()
        self.is_modified = True
        self.list_update_signal.emit()
        self.log_signal.emit(f"[System] Pasted {len(self.clipboard)} waypoints at line {index + 1 if index >= 0 else 'end'}.")

    # --- 2.4 基座演算與同步 ---
    def apply_batch_base_shift(self, indices, target_base_mat, target_base_name):
        valid_indices = [i for i in indices if 0 <= i < len(self.waypoints) and self.waypoints[i].get('type') in ["PTP", "LIN", "CIRC"]]
        if not valid_indices: return
        self._execute_base_shift(valid_indices, target_base_mat, target_base_name)

    def apply_base_shift_block(self, set_base_idx, target_base_mat, target_base_name):
        target_indices = []
        for i in range(set_base_idx + 1, len(self.waypoints)):
            if self.waypoints[i].get('type') == 'SET_BASE':
                if self.waypoints[i].get('active', True): break
                else: continue
            if self.waypoints[i].get('type') in ["PTP", "LIN", "CIRC"]:
                target_indices.append(i)
                
        if not target_indices:
            self.log_signal.emit("[System] No motion points found in this SET_BASE block.")
            return
        self._execute_base_shift(target_indices, target_base_mat, target_base_name)

    # 專職處理「單一點位」座標系轉換的共用大腦
    def _shift_single_waypoint(self, wp, target_base_mat, recorded_base_mat):
        """計算單一軌跡點的 Base Shift，回傳 (是否成功轉換, 錯誤訊息)"""
        if np.allclose(target_base_mat, recorded_base_mat, atol=1e-4):
            return False, "" # 矩陣幾乎一樣，略過計算

        T_flange_old = np.array(wp.get('cartesian_flange', kinematics.forward_kinematics(wp['joints'])))
        new_joints, err = kinematics.calculate_base_shift_ik(
            T_flange_old, recorded_base_mat, target_base_mat, wp['joints']
        )
        
        if new_joints is None:
            return False, "IK Failed"
            
        wp['joints'] = np.round(new_joints, 4).tolist()
        wp['cartesian_flange'] = np.round(kinematics.forward_kinematics(new_joints), 4).tolist()

        if wp.get('type') == 'CIRC' and 'aux_joints' in wp:
            T_aux_old = np.array(wp.get('aux_cartesian_flange', kinematics.forward_kinematics(wp['aux_joints'])))
            new_aux, err_aux = kinematics.calculate_base_shift_ik(
                T_aux_old, recorded_base_mat, target_base_mat, wp['aux_joints']
            )
            if new_aux is None:
                return False, "(Aux) IK Failed"
            wp['aux_joints'] = np.round(new_aux, 4).tolist()
            wp['aux_cartesian_flange'] = np.round(kinematics.forward_kinematics(new_aux), 4).tolist()

        wp['recorded_base_matrix'] = np.round(target_base_mat, 4).tolist()
        return True, ""

    def _execute_base_shift(self, indices, target_base_mat, target_base_name):
        success_count = 0
        error_msg = ""
        
        for idx in indices:
            wp = self.waypoints[idx]
            recorded_base_mat = np.array(wp.get('recorded_base_matrix', np.eye(4)))
            
            is_shifted, err = self._shift_single_waypoint(wp, target_base_mat, recorded_base_mat)
            
            if err:
                error_msg = f"Point {idx+1} {err}."
                break
            if is_shifted:
                success_count += 1

        if success_count > 0:
            self.is_modified = True
            self.list_update_signal.emit()

        if error_msg:
            self.log_signal.emit(f"[ERROR] Base Shift: {error_msg}")
        elif success_count > 0:
            self.log_signal.emit(f"[System] Successfully shifted {success_count} points to '{target_base_name}'.")
        else:
            self.log_signal.emit(f"[System] Selected points are already in '{target_base_name}'.")

    def sync_all_base_shifts(self, base_manager):
        if not self.waypoints: return

        current_base_mat = np.eye(4)
        shifted_count = 0
        error_msg = ""

        for idx, wp in enumerate(self.waypoints):
            m_type = wp.get('type')
            
            if m_type == 'SET_BASE':
                if wp.get('active', True):
                    bid = wp.get('value', 0)
                    if bid < len(base_manager.bases):
                        current_base_mat = base_manager.get_matrix(bid)
                continue 

            if m_type not in ["PTP", "LIN", "CIRC"]:
                continue

            recorded_base_mat = np.array(wp.get('recorded_base_matrix', np.eye(4)))
            
            # 一行呼叫，負責執行自動同步
            is_shifted, err = self._shift_single_waypoint(wp, current_base_mat, recorded_base_mat)
            
            if err:
                error_msg = f"Point {idx+1} {err} during Auto-Sync."
                break
                
            if is_shifted:
                shifted_count += 1
            else:
                if 'joints' in wp: wp['joints'] = np.round(wp['joints'], 4).tolist()
                if 'cartesian_flange' in wp: wp['cartesian_flange'] = np.round(wp['cartesian_flange'], 4).tolist()
                wp['recorded_base_matrix'] = np.round(current_base_mat, 4).tolist()

        if shifted_count > 0:
            self.is_modified = True

        if error_msg:
            self.log_signal.emit(f"[ERROR] Save Auto-Sync: {error_msg}")
        elif shifted_count > 0:
            self.log_signal.emit(f"[System] Auto-synced {shifted_count} points to match their local SET_BASE before saving.")
            self.list_update_signal.emit()

    # --- 2.5 檔案管理 ---
    def save_to_file(self):
        filename, _ = QFileDialog.getSaveFileName(self.parent_widget, "Save Path", "", "JSON Files (*.json)")
        if filename:
            try:
                import re
                import copy
                
                export_waypoints = []
                for wp in self.waypoints:
                    clean_wp = copy.deepcopy(wp)
                    
                    if clean_wp.get("type") != "CIRC" and "aux_joints" in clean_wp:
                        del clean_wp["aux_joints"]
                    if "note" in clean_wp and not clean_wp["note"]:
                        del clean_wp["note"]
                    if clean_wp.get("active") is True:
                        del clean_wp["active"]
                        
                    for key in ["speed", "accel"]:
                        if key in clean_wp:
                            val = clean_wp[key]
                            if isinstance(val, float) and val.is_integer():
                                clean_wp[key] = int(val)
                                
                    if "joints" in clean_wp:
                        clean_wp["joints"] = [round(j, 3) for j in clean_wp["joints"]]
                    if "aux_joints" in clean_wp and clean_wp["aux_joints"]:
                        clean_wp["aux_joints"] = [round(j, 3) for j in clean_wp["aux_joints"]]
                        
                    for mat_key in ["cartesian_flange", "recorded_base_matrix"]:
                        if mat_key in clean_wp and isinstance(clean_wp[mat_key], list):
                            clean_wp[mat_key] = [[round(val, 4) for val in row] for row in clean_wp[mat_key]]
                            
                    export_waypoints.append(clean_wp)

                raw_json = json.dumps(export_waypoints, indent=4)
                
                compact_json = re.sub(
                    r'\[\s+([-0-9.eE]+(?:,\s*[-0-9.eE]+)*)\s+\]',
                    lambda m: '[' + re.sub(r'\s+', '', m.group(1)).replace(',', ', ') + ']',
                    raw_json
                )
                
                compact_json = re.sub(
                    r'\[\s+((?:\[[-0-9.eE, ]+\]\s*,\s*)*\[[-0-9.eE, ]+\])\s+\]',
                    lambda m: '[' + re.sub(r'\s*\n\s*', ' ', m.group(1)) + ']',
                    compact_json
                )
                
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(compact_json)
                
                self.is_modified = False
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
                with open(filename, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                if isinstance(data, list):
                    self.waypoints = data
                    self.is_modified = False
                    self.list_update_signal.emit()
                    
                    base_name = os.path.basename(filename)
                    self.log_signal.emit(f"[System] UPDATED: Waypoints successfully loaded from {base_name}")
                    
                    if hasattr(self, 'file_loaded_signal'):
                        self.file_loaded_signal.emit(base_name)
                else:
                    self.log_signal.emit("[ERROR] Invalid file format: Expected a list of waypoints.")
                    
            except Exception as e:
                self.log_signal.emit(f"[ERROR] Load failed: {e}")

    # --- 2.6 軌跡預覽與執行 ---
    def _resolve_waypoint_kinematics(self, waypoint_list, initial_tcp_mat=None):
        """
        核心解析引擎：統一處理動態基座切換、TCP 偏移與逆運動學。
        保證「3D 預覽畫面」與「硬體實體運行」採用絕對一致的數學邏輯。
        """
        resolved_list = []
        current_tcp_mat = initial_tcp_mat if initial_tcp_mat is not None else self._get_tcp_matrix(0)
        current_base_mat = np.eye(4)
        base_mgr = self.parent_widget.base_manager if self.parent_widget else None
        last_valid_actual_joints = None

        for pt in waypoint_list:
            wp = copy.deepcopy(pt)  # 避免汙染原始資料
            m_type = wp.get('type', 'LIN')
            wp['move_type'] = m_type
            
            if m_type == "SET_BASE":
                base_idx = int(wp.get("value", 0))
                if base_mgr and 0 <= base_idx < len(base_mgr.bases):
                    current_base_mat = base_mgr.get_matrix(base_idx)
            
            elif m_type == "SET_TCP":
                current_tcp_mat = self._get_tcp_matrix(int(wp.get("value", 0)))
                
            wp['active_tcp_mat'] = current_tcp_mat
            target_joints = wp.get('joints', [])
            
            if m_type in ["PTP", "LIN", "CIRC"] and len(target_joints) == 6:
                T_flange_world_old = np.array(wp['cartesian_flange']) if 'cartesian_flange' in wp else kinematics.forward_kinematics(target_joints)
                recorded_base_mat = np.array(wp['recorded_base_matrix']) if 'recorded_base_matrix' in wp else np.eye(4)
                
                # 若記錄時的基座與當前基座不同，啟動動態 Base Shift 轉換
                if not np.allclose(current_base_mat, recorded_base_mat, atol=1e-4):
                    T_user = np.linalg.inv(recorded_base_mat) @ T_flange_world_old
                    T_flange_world_new = current_base_mat @ T_user
                    
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
                wp['resolved_joints'] = target_joints
                
                if m_type == "CIRC" and 'aux_joints' in wp:
                    aux_j = wp['aux_joints']
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
                    wp["resolved_aux_joints"] = aux_j

            elif m_type == "CAM_PATH":
                path_data = wp.get("path_data", [])
                if path_data:
                    last_valid_actual_joints = path_data[-1]

            resolved_list.append(wp)
            
        return resolved_list

    def get_trajectory_preview(self, initial_tcp_offset=None):
        """ 呼叫解析引擎後，將座標轉換為 3D 繪圖點位陣列 """
        # O(1) 快取攔截：如果點位沒變過，直接回傳上次算好的結果，秒殺卡頓！
        if initial_tcp_offset is None and getattr(self, '_preview_cache', None) is not None:
            return self._preview_cache
        
        trajectory_points = []
        valid_physical_pts = []
        
        active_wps = [wp for wp in self.waypoints if wp.get('active', True)]
        resolved_wps = self._resolve_waypoint_kinematics(active_wps)
        
        for wp in resolved_wps:
            m_type = wp.get('move_type')
            current_tcp = wp.get('active_tcp_mat', np.eye(4))
            
            if m_type in ['PTP', 'LIN', 'CIRC'] and 'resolved_joints' in wp:
                valid_physical_pts.append({
                    'joints': wp['resolved_joints'],
                    'type': m_type,
                    'aux_joints': wp.get('resolved_aux_joints'),
                    'tcp': current_tcp
                })
            elif m_type == "CAM_PATH":
                for pt_joints in wp.get("path_data", []):
                    valid_physical_pts.append({
                        'joints': pt_joints, 'type': 'LIN', 
                        'aux_joints': None, 'tcp': current_tcp
                    })

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
        
        # 離開前將結果存入快取
        if initial_tcp_offset is None:
            self._preview_cache = trajectory_points
                
        return trajectory_points

    def execute_streaming_path(self, active_points, start_joints, tcp_offset_mat, loop, global_speed, global_accel, serial_ref, callbacks):
        """ 呼叫解析引擎後，將標準化點位封裝並派發給背景執行緒 """
        resolved_wps = self._resolve_waypoint_kinematics(active_points, initial_tcp_mat=tcp_offset_mat)
        wp_list = []
        
        for wp in resolved_wps:
            m_type = wp.get('move_type', 'LIN')
            pt_speed = wp.get('speed', global_speed * 100) / 100.0
            pt_accel = wp.get('accel', global_accel * 100) / 100.0
            
            formatted_wp = {
                "move_type": m_type,
                "name": wp.get('name', ''), 
                "target_joints": wp.get('resolved_joints', wp.get('joints', [])), 
                "tcp_offset_mat": wp.get('active_tcp_mat', np.eye(4)), 
                "speed_factor": pt_speed,
                "accel_factor": pt_accel,
                "value": wp.get('value', 0.0), 
                "blend": wp.get('blend', 'FINE')
            }
            
            if m_type == "CIRC" and 'resolved_aux_joints' in wp:
                formatted_wp["aux_joints"] = wp['resolved_aux_joints']
                
            if m_type == "I/O":
                formatted_wp["action_type"] = wp.get("action_type", "DIGITAL")

            if m_type == "CAM_PATH":
                formatted_wp["path_data"] = wp.get("path_data", [])
                formatted_wp["speed"] = wp.get("speed", 30.0)
                
            wp_list.append(formatted_wp)

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
            
        if serial_ref:
            serial_ref.reset_semaphore() 
            
        self.worker.start()

    def _on_worker_error(self, msg):
        self.log_signal.emit(f"[STOP] {msg}")
        self.stop_path()

    def stop_path(self):
        if self.worker:
            self.worker._is_running = False 

        if self.serial_manager and self.serial_manager.is_connected:
            self.serial_manager.send_stop()
            
        if self.serial_manager:
            if hasattr(self.serial_manager, 'ok_semaphore'):
                self.serial_manager.ok_semaphore.release()
            self.serial_manager.motion_done_event.set()
            
        if self.worker and self.worker.isRunning():
            self.worker.wait() 
            
        self.worker = None