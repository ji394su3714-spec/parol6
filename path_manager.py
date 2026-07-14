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
        self.point_queue = queue.Queue(maxsize=100) 
        self.producer_finished = False
        self.producer_error = False

    def run(self):
        prod_thread = threading.Thread(target=self._producer_task, daemon=True)
        prod_thread.start()

        self.log_signal.emit("[System] Compiling path, waiting for safe buffer level...")
        while self.point_queue.qsize() < 99 and not self.producer_finished and self._is_running: 
            if self.producer_error: return
            self.msleep(10)

        if not self._is_running or self.producer_error: return

        real_start_time = time.time()
        counter = 0
        interval = 0.010
        gui_skip_frames =5
        self.update_signal.emit(list(self.start_joints))
        
        last_ik_joints = None

        while self._is_running:
            # 優化 2：紀錄迴圈起始的絕對精確時間
            loop_start = time.perf_counter() 
            
            try:
                item = self.point_queue.get(timeout=0.1)
                
                # ==========================================
                # 狀態膠囊過濾網：嚴格攔截所有 dict，並強制 continue
                # ==========================================
                if isinstance(item, dict):
                    cmd_type = item.get("type")
                    if cmd_type == "LOG":
                        self.log_signal.emit(item["msg"])
                    elif cmd_type == "SET_TCP_CMD":
                        self.log_signal.emit(item["msg"])
                        self.set_tcp_signal.emit(item["tool_idx"])
                    elif cmd_type == "SET_BASE_CMD":
                        self.log_signal.emit(item["msg"])
                        self.set_base_signal.emit(item["base_idx"])
                    elif cmd_type == "DELAY_CMD":
                        self.log_signal.emit(item["msg"])
                        if self.serial_ref and self.serial_ref.is_connected:
                            # 1. 確保 Arduino 把前面的動作全部走完，水桶徹底淨空
                            if hasattr(self.serial_ref, 'wait_for_motion_complete'):
                                self.serial_ref.wait_for_motion_complete(timeout=10.0)
                        # 2. 物理鎮定休眠 (讓馬達慣性與微震動完全停止)
                        time.sleep(item.get("value", 0.5))
                        
                    elif cmd_type == "EE_CMD":
                        self.log_signal.emit(item["msg"])
                        if self.serial_ref and self.serial_ref.is_connected:
                            if hasattr(self.serial_ref, 'wait_for_motion_complete'):
                                self.serial_ref.wait_for_motion_complete(timeout=10.0)
                            self.serial_ref.send_command(item["cmd"])
                            if hasattr(self.serial_ref, 'wait_for_ee_done'):
                                self.serial_ref.wait_for_ee_done(timeout=10.0)
                    continue 
                # ==========================================

                ik_joints = item
                last_ik_joints = ik_joints
                
                if counter % gui_skip_frames == 0:
                    self.update_signal.emit(list(ik_joints))
                    
                if self.serial_ref and self.serial_ref.is_connected:
                    self.serial_ref.send_joints(list(ik_joints), interval, move_mode=1)
                    self.serial_ref.wait_for_ok(timeout=3.0)
                else:
                    # 優化 3：高精度補償延遲 (Busy-wait + Micro-sleep)
                    # 徹底解決 Windows time.sleep 造成的 15.6ms 系統性頓挫
                    target_time = loop_start + interval
                    while time.perf_counter() < target_time:
                        time.sleep(0.001) 
                    
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
        current_seed = list(self.start_joints)
        pending_trajectory = None       
        pending_blend_str = 'FINE'      
        loop_count = 1  

        while self._is_running:
            if self.loop:
                self.point_queue.put({"type": "LOG", "msg": f">> Start executing loop {loop_count}..."})
        
            for wp_idx, wp in enumerate(self.waypoint_list):
                if not self._is_running: return
                
                move_type = wp.get("move_type", "LIN")
                # 新增這兩行：組合出 "型態 + 名稱" 的標籤 (例如 "[LIN] Point 3")
                wp_name = wp.get("name", f"Point {wp_idx+1}")
                display_label = f"[{move_type}] {wp_name}"
                
                # 處理不需要 IK 的輔助指令
                if move_type in ["SET_TCP", "SET_BASE", "I/O"]:
                    
                    # 1. 遇到切換指令前，先將前面累積的軌跡結算並送出
                    if pending_trajectory is not None:
                        for ik_joints in pending_trajectory:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                        pending_trajectory = None
                        
                    # 2. 處理 I/O
                    if move_type == "I/O":
                        ee_type = wp.get("action_type", "DIGITAL")
                        ee_val = int(wp.get("value", 0))
                        self.point_queue.put({
                            "type": "EE_CMD", "cmd": f"<EE,{ee_type},{ee_val}>",
                            "msg": f">> 執行工具動作: {ee_type} -> {ee_val}"
                        }, block=True)
                        
                    # 3. 處理 SET_TCP，並自動注入 0.5s 物理鎮定延遲
                    elif move_type == "SET_TCP":
                        self.point_queue.put({
                            "type": "SET_TCP_CMD", "tool_idx": int(wp.get("value", 0)),
                            "msg": f">> 執行換刀: 切換至 [{wp.get('value')}] {wp.get('name', '')}" 
                        }, block=True)
                        
                        self.point_queue.put({
                            "type": "DELAY_CMD", "value": 0.5,
                            #"msg": ">> [系統自動防護] TCP 切換，物理鎮定 0.5 秒"
                        }, block=True)
                        
                    # 4. 處理 SET_BASE，並自動注入 0.5s 物理鎮定延遲
                    elif move_type == "SET_BASE":
                        self.point_queue.put({
                            "type": "SET_BASE_CMD", "base_idx": int(wp.get("value", 0)),
                            "msg": f">> 執行基座切換: 切換至 [{wp.get('value')}] {wp.get('name', '')}" 
                        }, block=True)
                        
                        self.point_queue.put({
                            "type": "DELAY_CMD", "value": 0.5,
                            #"msg": ">> [系統自動防護] Base 切換，物理鎮定 0.5 秒"
                        }, block=True)
                        
                    # 強制切斷連續融合
                    pending_blend_str = 'FINE' 
                    continue

                # ===================================================
                # 👑 2. 全新引擎：五次 B-樣條 (Look-ahead 貪吃蛇邏輯)
                # ===================================================
                if move_type == "SPLINE":
                    spline_buffer.append(wp)
                    is_last = (wp_idx == len(self.waypoint_list) - 1)
                    next_is_spline = not is_last and (self.waypoint_list[wp_idx+1].get("move_type") == "SPLINE")

                    if next_is_spline:
                        continue # 💡 繼續吃點，不急著進入運算！

                    # 收集完畢，準備計算！
                    # 遇到 Spline 區塊前，強制將前面的尾巴結算 (Spline 不需要舊版疊加融合)
                    if pending_trajectory is not None:
                        for ik_joints in pending_trajectory:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                        pending_trajectory = None

                    # 呼叫五次 B-樣條數學引擎
                    gen, t_tot, msg, total_N = kinematics.TrajectoryMathEngine.calculate_spline_trajectory(
                        current_seed, spline_buffer, interval=0.010
                    )
                    spline_buffer = [] # 清空肚子，等待下一批
                    
                    if gen is None:
                        self.error_signal.emit(f"[SPLINE Block] failed: {msg}")
                        self.producer_error = True
                        return

                    if msg and msg != "SUCCESS":
                        self.log_signal.emit(msg)
                        
                    # 直接將完美平滑的曲線推播給發送佇列 (Bypass 掉舊版的 blend 區塊)
                    try:
                        for _ in range(total_N):
                            if not self._is_running: return
                            pt = next(gen)
                            self.point_queue.put(pt, block=True)
                            current_seed = pt
                    except Exception as e:
                        self.error_signal.emit(f"[SPLINE Block] IK Error: {e}")
                        self.producer_error = True
                        return
                    
                    pending_blend_str = 'FINE' # Spline 跑完後，強制煞車等待下一個指令
                    continue 
                # ===================================================

                # ===================================================
                # 3. 處理傳統的 PTP, LIN, CIRC, DELAY
                # ===================================================
                gen, total_N, msg = None, 0, ""
                
                if move_type == "DELAY":
                    delay_time = wp.get("value", 0.0)
                    delay_steps = max(1, int(delay_time / 0.010))
                    def delay_gen(seed, steps):
                        for _ in range(steps): yield list(seed)
                    gen = delay_gen(current_seed, delay_steps)
                    total_N = delay_steps
                    msg = f"wait {delay_time} seconds..."
                else:
                    speed_factor = wp.get("speed_factor", 1.0)
                    accel_factor = wp.get("accel_factor", 1.0)
                    tcp_offset_mat = wp.get("tcp_offset_mat", np.eye(4))
                    target_joints = np.array(wp.get("target_joints", current_seed))
                    
                    if move_type == "PTP":
                        gen, t_tot, msg, total_N = kinematics.TrajectoryMathEngine.calculate_ptp_trajectory(
                            current_seed, target_joints, speed_factor, accel_factor)
                    elif move_type == "LIN":
                        gen, t_tot, msg, total_N = kinematics.TrajectoryMathEngine.calculate_lin_trajectory(
                            current_seed, target_joints, tcp_offset_mat, speed_factor, accel_factor)
                    elif move_type == "CIRC":
                        gen, t_tot, msg, total_N = kinematics.TrajectoryMathEngine.calculate_circ_trajectory(
                            current_seed, wp["aux_joints"], target_joints, tcp_offset_mat, speed_factor, accel_factor)

                if gen is None:
                    self.error_signal.emit(f"{display_label} failed: {msg}")
                    self.producer_error = True
                    return

                if msg and msg != "SUCCESS" and not msg.startswith("wait"):
                    self.log_signal.emit(f"{display_label}: {msg}")

                # ----------------------------------------------------
                # 1. 提取要跟「上一個點」融合的頭部點數 (N_blend)
                # ----------------------------------------------------
                N_blend = 0
                if pending_trajectory is not None and pending_blend_str != 'FINE' and move_type != 'DELAY':
                    # 修正：pending_trajectory 本身就已經是切好的尾巴了，直接比對長度即可！
                    N_blend = min(len(pending_trajectory), total_N)
                
                curr_head = []
                try:
                    for _ in range(N_blend):
                        curr_head.append(next(gen))
                except Exception as e:
                    self.error_signal.emit(f"{display_label} failed: {e}")
                    self.producer_error = True
                    return
                
                # 執行融合並推播
                if pending_trajectory is not None:
                    M = len(pending_trajectory)
                    if N_blend > 0:
                        arr_A = np.array(pending_trajectory[M - N_blend : M])
                        arr_B = np.array(curr_head)
                        vertex = np.array(pending_trajectory[-1])
                        blended_section = (arr_A + arr_B - vertex).tolist()
                        
                        for ik_joints in pending_trajectory[: M - N_blend]:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                        for ik_joints in blended_section:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)
                    else:
                        for ik_joints in pending_trajectory:
                            if not self._is_running: return
                            self.point_queue.put(ik_joints, block=True)

                # ----------------------------------------------------
                # 2. 計算要保留給「下一個點」融合的尾巴點數 (H_hold)
                # ----------------------------------------------------
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

                # ----------------------------------------------------
                # 3. 終極惰性求值 (Lazy Evaluation Streaming)
                # 從生成器中抽出一個點，就算一個，然後立刻塞進 Queue！
                # ----------------------------------------------------
                try:
                    for _ in range(stream_count):
                        if not self._is_running: return
                        pt = next(gen)
                        self.point_queue.put(pt, block=True) # 隊列滿了就會在此完美休眠，不吃效能！
                        current_seed = pt
                        
                    pending_trajectory = []
                    for _ in range(H_hold):
                        pt = next(gen)
                        pending_trajectory.append(pt)
                        current_seed = pt
                except Exception as e:
                    self.error_signal.emit(f"{display_label} failed: {e}")
                    self.producer_error = True
                    return
                
                pending_blend_str = curr_blend_str

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
    file_loaded_signal = Signal(str) 
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.waypoints = []
        self.worker = None
        self.parent_widget = parent
        self.is_modified = False
        self.clipboard = []
        
        self.temp_aux_joints = None 
        self.serial_manager = None
        if hasattr(parent, 'serial_manager'):
            self.serial_manager = parent.serial_manager

    def is_running(self):
        return hasattr(self, 'worker') and self.worker is not None and self.worker.isRunning()

    def set_aux_point(self, joints):
        self.temp_aux_joints = list(joints)
        self.log_signal.emit(">> [CIRC] AUX point saved! Move to END point and press Record.")
        self.is_modified = True

    def update_point_at_index(self, index, current_joints):
        """將目前手臂的座標，覆蓋掉指定的實體動作點位"""
        if 0 <= index < len(self.waypoints):
            wp = self.waypoints[index]
            if wp.get('type') in ['PTP', 'LIN', 'CIRC']:
                wp['joints'] = [round(j, 4) for j in current_joints]
                
                self.list_update_signal.emit()
                self.log_signal.emit(f">>> UPDATED: [{wp.get('name')}] position updated to current pose.")
                self.is_modified = True

    def insert_waypoint(self, index, waypoint_data):
        """統一的插入接口：將新點位插入到指定的 index (即原本點位的前面)"""
        self.waypoints.insert(index, waypoint_data)
        
        if hasattr(self, '_renumber_points'):
            self._renumber_points()
            
        self.list_update_signal.emit()
        self.log_signal.emit(f">>> INSERTED: [{waypoint_data['type']}] at line {index + 1}")
        self.is_modified = True

    def update_special_point(self, index, pt_type, new_value, new_name):
        """專門用來更新非實體點位 (SET_TCP, SET_BASE, DELAY, IO)"""
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
        """後期編輯：更新指定點位的輔助座標"""
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
        """刪除點位後重新編號 (只對實體動作點位生效)"""
        count = 1
        for pt in self.waypoints:
            if pt.get('type') in ['PTP', 'LIN', 'CIRC']:
                pt['name'] = f"Point {count}"
                count += 1

    # ==========================================
    # 複製與貼上引擎
    # ==========================================
    def copy_points(self, indices):
        """複製指定的點位"""
        import copy
        valid_indices = [i for i in indices if 0 <= i < len(self.waypoints)]
        valid_indices.sort()
        if not valid_indices: return
        
        self.clipboard = [copy.deepcopy(self.waypoints[i]) for i in valid_indices]
        self.log_signal.emit(f"[System] Copied {len(self.clipboard)} waypoints.")

    def paste_points(self, index=-1):
        """貼上點位到指定位置"""
        import copy
        if not getattr(self, 'clipboard', []): return
        
        target_idx = index if index >= 0 else len(self.waypoints)
        
        for wp in self.clipboard:
            new_wp = copy.deepcopy(wp)
            self.waypoints.insert(target_idx, new_wp)
            target_idx += 1
            
        if hasattr(self, '_renumber_points'):
            self._renumber_points()
            
        self.is_modified = True
        self.list_update_signal.emit()
        self.log_signal.emit(f"[System] Pasted {len(self.clipboard)} waypoints at line {index + 1 if index >= 0 else 'end'}.")

    # ==========================================
    # Base Shift 空間轉換引擎
    # ==========================================
    def apply_batch_base_shift(self, indices, target_base_mat, target_base_name):
        """處理選定點位的 Base Shift"""
        valid_indices = [i for i in indices if 0 <= i < len(self.waypoints) and self.waypoints[i].get('type') in ["PTP", "LIN", "CIRC"]]
        if not valid_indices: return
        self._execute_base_shift(valid_indices, target_base_mat, target_base_name)

    def apply_base_shift_block(self, set_base_idx, target_base_mat, target_base_name):
        """處理整個 SET_BASE 區塊的 Base Shift"""
        target_indices = []
        for i in range(set_base_idx + 1, len(self.waypoints)):
            if self.waypoints[i].get('type') == 'SET_BASE':
                if self.waypoints[i].get('active', True):
                    break
                else:
                    continue
            if self.waypoints[i].get('type') in ["PTP", "LIN", "CIRC"]:
                target_indices.append(i)
                
        if not target_indices:
            self.log_signal.emit("[System] No motion points found in this SET_BASE block.")
            return
            
        self._execute_base_shift(target_indices, target_base_mat, target_base_name)

    def _execute_base_shift(self, indices, target_base_mat, target_base_name):
        """核心共用轉換引擎"""
        import kinematics
        import numpy as np
        success_count = 0
        error_msg = ""
        
        for idx in indices:
            wp = self.waypoints[idx]
            recorded_base_mat = np.array(wp.get('recorded_base_matrix', np.eye(4)))
            
            if np.allclose(target_base_mat, recorded_base_mat, atol=1e-4):
                continue

            T_flange_old = np.array(wp.get('cartesian_flange', kinematics.forward_kinematics(wp['joints'])))
            new_joints, err = kinematics.calculate_base_shift_ik(
                T_flange_old, recorded_base_mat, target_base_mat, wp['joints']
            )
            
            if new_joints is None:
                error_msg = f"Point {idx+1} IK Failed."
                break
                
            wp['joints'] = np.round(new_joints, 4).tolist()
            wp['cartesian_flange'] = np.round(kinematics.forward_kinematics(new_joints), 4).tolist()

            if wp.get('type') == 'CIRC' and 'aux_joints' in wp:
                T_aux_old = np.array(wp.get('aux_cartesian_flange', kinematics.forward_kinematics(wp['aux_joints'])))
                new_aux, err_aux = kinematics.calculate_base_shift_ik(
                    T_aux_old, recorded_base_mat, target_base_mat, wp['aux_joints']
                )
                if new_aux is None:
                    error_msg = f"Point {idx+1} (Aux) IK Failed."
                    break
                wp['aux_joints'] = np.round(new_aux, 4).tolist()
                wp['aux_cartesian_flange'] = np.round(kinematics.forward_kinematics(new_aux), 4).tolist()

            wp['recorded_base_matrix'] = np.round(target_base_mat, 4).tolist()
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
        """終極防呆：在 Save 觸發前，由上到下掃描整個路徑，確保所有點位與其上方的 SET_BASE 絕對同步"""
        import kinematics
        import numpy as np
        
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
            
            if 'joints' in wp: wp['joints'] = np.round(wp['joints'], 4).tolist()
            if 'cartesian_flange' in wp: wp['cartesian_flange'] = np.round(wp['cartesian_flange'], 4).tolist()
            wp['recorded_base_matrix'] = np.round(current_base_mat, 4).tolist()

            if np.allclose(current_base_mat, recorded_base_mat, atol=1e-4):
                continue

            T_flange_old = np.array(wp.get('cartesian_flange', kinematics.forward_kinematics(wp['joints'])))
            new_joints, err = kinematics.calculate_base_shift_ik(
                T_flange_old, recorded_base_mat, current_base_mat, wp['joints']
            )
            
            if new_joints is None:
                error_msg = f"Point {idx+1} IK Failed during Auto-Sync."
                break
                
            wp['joints'] = np.round(new_joints, 4).tolist()
            wp['cartesian_flange'] = np.round(kinematics.forward_kinematics(new_joints), 4).tolist()

            if m_type == 'CIRC' and 'aux_joints' in wp:
                T_aux_old = np.array(wp.get('aux_cartesian_flange', kinematics.forward_kinematics(wp['aux_joints'])))
                new_aux, err_aux = kinematics.calculate_base_shift_ik(
                    T_aux_old, recorded_base_mat, current_base_mat, wp['aux_joints']
                )
                if new_aux is None:
                    error_msg = f"Point {idx+1} (Aux) IK Failed."
                    break
                
                wp['aux_joints'] = np.round(new_aux, 4).tolist()
                wp['aux_cartesian_flange'] = np.round(kinematics.forward_kinematics(new_aux), 4).tolist()

            shifted_count += 1

        if shifted_count > 0:
            self.is_modified = True

        if error_msg:
            self.log_signal.emit(f"[ERROR] Save Auto-Sync: {error_msg}")
        elif shifted_count > 0:
            self.log_signal.emit(f"[System] Auto-synced {shifted_count} points to match their local SET_BASE before saving.")
            self.list_update_signal.emit()

    def save_to_file(self):
        filename, _ = QFileDialog.getSaveFileName(self.parent_widget, "Save Path", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.waypoints, f, indent=4)
                
                # 檔案成功寫入硬碟，洗白標記！
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
                with open(filename, 'r') as f:
                    data = json.load(f)
                
                # 確保讀進來的是正確的陣列格式
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
                
                # 陣列加工空間轉移 (含 180 度死角破解)
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