#path_manager.py
import json
import time
import numpy as np
from PyQt5.QtCore import QObject, QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QFileDialog
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

import kinematics
import math

# --- 1. PTP 執行器 ---
class PTPExecutor(QThread):
    update_signal = pyqtSignal(list)
    finished_signal = pyqtSignal()
    
    def __init__(self, start_joints, end_joints, serial_ref=None, speed_factor=1.0, animation_time=1.0):
        super().__init__()
        self.start_joints = np.array(start_joints)
        self.end_joints = np.array(end_joints)
        self.serial_ref = serial_ref
        self.speed_factor = speed_factor
        self.animation_time = animation_time

    def run(self):
        # 1. 【硬體端：發送指令】
        if self.serial_ref and self.serial_ref.is_connected:
            # 開跑前先清空舊的到位旗標
            if hasattr(self.serial_ref, 'motion_done_event'):
                self.serial_ref.motion_done_event.clear()
            # PTP 模式對應 move_mode = 1
            self.serial_ref.send_joints(list(self.end_joints), self.speed_factor, move_mode=1)

        # 2. 【軟體端：乖乖播完 UI 動畫】(不提早打斷，避免滑桿瞬間跳躍引發爆衝)
        effective_duration = max(0.1, float(self.animation_time))
        steps = int(effective_duration * 30) 
        
        for i in range(steps + 1):
            t = i / steps
            current = self.start_joints + (self.end_joints - self.start_joints) * t
            self.update_signal.emit(list(current)) 
            time.sleep(effective_duration / steps)
        
        # 3. 【等待硬體到位】
        if self.serial_ref and self.serial_ref.is_connected:
            if hasattr(self.serial_ref, 'wait_for_motion_complete'):
                self.serial_ref.wait_for_motion_complete(timeout=10.0)
            
        self.finished_signal.emit()

# --- 2. LIN 執行器 ---
class CartesianExecutor(QThread):
    update_signal = pyqtSignal(list)
    finished_signal = pyqtSignal()
    error_signal = pyqtSignal(str)

    def __init__(self, start_joints, target_joints, tcp_offset_mat, serial_ref=None, speed_factor=1.0, animation_time=2.0):
        super().__init__()
        self.start_joints = np.array(start_joints)
        self.target_joints = np.array(target_joints)
        self.tcp_offset_mat = tcp_offset_mat if tcp_offset_mat is not None else np.eye(4)
        self.speed_factor = speed_factor
        self.animation_time = animation_time
        self.serial_ref = serial_ref

    def run(self):
        try:
            tcp_inv = np.linalg.inv(self.tcp_offset_mat)
        except:
            self.error_signal.emit("Invalid TCP Matrix")
            return

        T_flange_start = kinematics.forward_kinematics(self.start_joints)
        T_tcp_start = T_flange_start @ self.tcp_offset_mat
        T_flange_end = kinematics.forward_kinematics(self.target_joints)
        T_tcp_end = T_flange_end @ self.tcp_offset_mat
        
        pos_start = T_tcp_start[:3, 3]
        pos_end = T_tcp_end[:3, 3]
        
        key_rots = R.from_matrix([T_tcp_start[:3, :3], T_tcp_end[:3, :3]])
        key_times = [0, 1]
        
        try:
            slerp = Slerp(key_times, key_rots)
        except Exception as e:
            self.error_signal.emit(f"Slerp Init Failed: {e}")
            return

        dist_mm = np.linalg.norm(pos_end - pos_start) * 1000.0
        R_diff = T_tcp_end[:3, :3] @ T_tcp_start[:3, :3].T
        angle_rad = np.linalg.norm(R.from_matrix(R_diff).as_rotvec())
        angle_deg = np.degrees(angle_rad)
        
        # 核心修正：捨棄不合理的「固定動畫時間」！
        # 改為：根據「實際物理距離」與「速度百分比」來計算真實需要的時間

        MAX_LIN_SPEED = 150.0  # 手臂的最高直線極速 (mm/s)，你可依實機手感微調
        MAX_ROT_SPEED = 60.0   # 手臂的最高旋轉極速 (deg/s)
        
        target_lin_speed = MAX_LIN_SPEED * self.speed_factor
        target_rot_speed = MAX_ROT_SPEED * self.speed_factor
        
        time_for_lin = dist_mm / max(0.1, target_lin_speed)
        time_for_rot = angle_deg / max(0.1, target_rot_speed)
        
        # 總時間取「平移」和「旋轉」兩者中較耗時的那個，確保兩者都不會超速
        effective_duration = max(0.1, time_for_lin, time_for_rot)

        # 1. 畫質控：維持高解析度的空間切片 (每 2mm 或 1度 切一刀)
        ideal_spatial_steps = max(2, int(dist_mm / 2.0), int(angle_deg / 1.0))
        
        # 2. 硬體算力極限：不再管通訊延遲，只管 Arduino 的算力極限！
        # 壓榨到極限的 0.015 秒 (相當於 66Hz 更新率)，這已經是工業級的插補頻率了
        hardware_limit_steps = max(2, int(effective_duration / 0.015))
        
        # 3. 決策：兩者取小
        steps = min(ideal_spatial_steps, hardware_limit_steps)
        
        # Phase 1: 預先計算 (Planning) 加上 S-Curve
        trajectory_points = []
        current_seed = self.start_joints.copy()
        
        self.update_signal.emit(list(self.start_joints))
        
        for i in range(1, steps + 1):
            linear_t = i / steps 
            # 【核心】：Sine Ease-in-out，讓軌跡頭尾點距縮短，達到自然的加減速
            t = (1 - math.cos(linear_t * math.pi)) / 2.0
            
            curr_pos = pos_start + (pos_end - pos_start) * t
            curr_rot = slerp([t]).as_matrix()[0]
            
            T_tcp_target = np.eye(4)
            T_tcp_target[:3, :3] = curr_rot
            T_tcp_target[:3, 3] = curr_pos
            
            T_flange_target = T_tcp_target @ tcp_inv
            ik_result, error = kinematics.inverse_kinematics(T_flange_target, current_seed)
            
            if ik_result is not None:
                current_seed = ik_result
                trajectory_points.append(list(ik_result))
            else:
                self.error_signal.emit(f"LIN Error: Unreachable at step {i}")
                return

        # Phase 2: 穩定串流 (Streaming)
        interval = effective_duration / steps
        # 動態計算 GUI 更新間隔：目標維持大約 10 FPS
        points_per_sec = steps / effective_duration
        gui_skip_frames = max(1, int(points_per_sec / 10.0)) 
        
        counter = 0 
        
        for joints in trajectory_points:
            # 使用動態計算出來的間隔來更新畫面
            if counter % gui_skip_frames == 0 or counter == len(trajectory_points) - 1:
                self.update_signal.emit(joints)
            counter += 1
            
            if self.serial_ref and self.serial_ref.is_connected:
                self.serial_ref.send_joints(joints, interval, move_mode=2)
                self.serial_ref.wait_for_ok(timeout=1.0)
            
        # 整個 LIN 陣列射完後，等待馬達把最後一段路走完
        if self.serial_ref and self.serial_ref.is_connected:
             if hasattr(self.serial_ref, 'wait_for_motion_complete'):
                self.serial_ref.wait_for_motion_complete(timeout=10.0)

        self.finished_signal.emit()

# --- PathManager (邏輯核心) ---
class PathManager(QObject):
    log_signal = pyqtSignal(str)
    joint_update_signal = pyqtSignal(list)
    list_update_signal = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.waypoints = []
        self.execution_queue = [] 
        self.worker = None
        self.path_index = 0
        self.parent_widget = parent
        
        self.is_looping = False
        self.global_speed = 1.0 
        self.current_tcp_offset = np.eye(4)
        
        self.serial_manager = None
        if hasattr(parent, 'serial_manager'):
            self.serial_manager = parent.serial_manager

    # --- 數據管理 ---
    def record_point(self, current_joints, delay=0.0, move_type="PTP", speed=50.0):
        idx = len(self.waypoints) + 1
        name = f"Point {idx}"
        data = {
            "name": name,
            "joints": list(current_joints),
            "delay": float(delay),
            "type": move_type,
            "speed": float(speed), 
            "active": True,
            "note": ""
        }
        self.waypoints.append(data)
        self.list_update_signal.emit()
        
        msg = f"Recorded: {name} [{move_type}]"
        if delay > 0: msg += f" (Wait {delay}s)"
        self.log_signal.emit(msg)

    def delete_point(self, index):
        if 0 <= index < len(self.waypoints):
            removed = self.waypoints.pop(index)
            self._renumber_points()
            self.list_update_signal.emit()
            self.log_signal.emit(f"Deleted: {removed['name']}")

    def delete_all_points(self):
        self.waypoints.clear()
        self.list_update_signal.emit()
        self.log_signal.emit("All waypoints deleted.")

    def toggle_point_active(self, index):
        if 0 <= index < len(self.waypoints):
            current_state = self.waypoints[index].get('active', True)
            self.waypoints[index]['active'] = not current_state
            self.list_update_signal.emit()

    def _renumber_points(self):
        for i, pt in enumerate(self.waypoints):
            pt['name'] = f"Point {i+1}"

    # --- 檔案存取 ---
    def save_to_file(self):
        filename, _ = QFileDialog.getSaveFileName(self.parent_widget, "Save Path", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.waypoints, f, indent=4)
                self.log_signal.emit(f"Path saved to {filename}")
            except Exception as e:
                self.log_signal.emit(f"[Error] Save failed: {e}")

    def load_from_file(self):
        filename, _ = QFileDialog.getOpenFileName(self.parent_widget, "Load Path", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'r') as f:
                    self.waypoints = json.load(f)
                    for pt in self.waypoints:
                        if 'active' not in pt: pt['active'] = True
                        if 'type' not in pt: pt['type'] = "PTP"
                        # 防呆機制：如果讀取舊存檔沒有速度，自動補上 50.0
                        if 'speed' not in pt: pt['speed'] = 50.0
                
                self.list_update_signal.emit()
                self.log_signal.emit(f"Path loaded from {filename}")
            except Exception as e:
                self.log_signal.emit(f"[Error] Load failed: {e}")

    def get_trajectory_preview(self, tcp_offset):
        """職權分離：由 PathManager 負責計算 3D 預覽軌跡"""
        trajectory_points = []
        active_wps = [pt for pt in self.waypoints if pt.get('active', True)]
        
        if len(active_wps) >= 2:
            for i in range(len(active_wps) - 1):
                j_start = np.array(active_wps[i]['joints'])
                j_end = np.array(active_wps[i+1]['joints'])
                m_type = active_wps[i+1].get('type', 'LIN') 
                
                steps = 20
                if m_type == 'PTP':
                    for t in np.linspace(0, 1, steps):
                        interp = j_start + t * (j_end - j_start)
                        T_tcp = kinematics.forward_kinematics(interp) @ tcp_offset
                        trajectory_points.append(T_tcp[:3, 3])
                else:
                    T_s = kinematics.forward_kinematics(j_start) @ tcp_offset
                    T_e = kinematics.forward_kinematics(j_end) @ tcp_offset
                    xs, xe = T_s[:3, 3], T_e[:3, 3]
                    for t in np.linspace(0, 1, steps):
                        trajectory_points.append(xs + t * (xe - xs))
                        
        return trajectory_points

    # --- 路徑執行邏輯 ---
    def run_path(self, current_joints_start, loop=False, tcp_offset=None):
        active_points = [pt for pt in self.waypoints if pt.get('active', True)]
        
        if not active_points:
            self.log_signal.emit("[Error] No active waypoints.")
            return
            
        if self.worker and self.worker.isRunning():
            self.log_signal.emit("[Info] Already running.")
            return

        self.is_looping = loop
        self.execution_queue = active_points
        self.current_tcp_offset = tcp_offset if tcp_offset is not None else np.eye(4)
        
        self.log_signal.emit(f"([START]) Executing {len(active_points)} points...")
        self.path_index = 0
        self._execute_next(current_joints_start)

    def _execute_next(self, current_joints):
        if self.path_index >= len(self.execution_queue):
            if self.is_looping:
                self.log_signal.emit(">> Looping...")
                self.path_index = 0
            else:
                self.log_signal.emit("([END]) Path Completed.")
                return

        target_data = self.execution_queue[self.path_index]
        target_joints = target_data['joints']
        name = target_data.get('name', str(self.path_index))
        move_type = target_data.get('type', "PTP")
        
        
        # 1. 計算硬體要用的「實際速度比例」 (0.01 ~ 1.0)
        point_speed_pct = target_data.get('speed', 50.0)
        speed_factor = (point_speed_pct / 100.0) * self.global_speed
        if speed_factor > 1.0: speed_factor = 1.0 # 最高就是 100%
        
        # 2. 計算軟體要用的「動畫播放時間」
        base_time = 100.0 / max(1.0, point_speed_pct) 
        animation_time = base_time / max(0.1, self.global_speed)
        
        self.log_signal.emit(f"Moving -> {name} ({move_type}, SPD:{speed_factor*100:.0f}%)...")
        
        if move_type == "LIN":
            self.worker = CartesianExecutor(
                start_joints=current_joints, 
                target_joints=target_joints, 
                tcp_offset_mat=self.current_tcp_offset, 
                serial_ref=self.serial_manager,  
                speed_factor=speed_factor,     # 傳入速度比例
                animation_time=animation_time  # 傳入動畫時間
            )
            self.worker.error_signal.connect(self._on_worker_error)
        else:
            self.worker = PTPExecutor(
                start_joints=current_joints, 
                end_joints=target_joints, 
                serial_ref=self.serial_manager,  
                speed_factor=speed_factor,     # 傳入速度比例
                animation_time=animation_time  # 傳入動畫時間
            )
            
        self.worker.update_signal.connect(self.joint_update_signal.emit)
        
        delay = target_data.get('delay', 0.0)
        self.worker.finished_signal.connect(lambda: self._on_point_finished(target_joints, delay))
        self.worker.start()

    def _on_point_finished(self, last_joints, delay):
        if delay > 0:
            self.log_signal.emit(f"Waiting {delay}s...")
            QTimer.singleShot(int(delay * 1000), lambda: self._trigger_next_step(last_joints))
        else:
            # 強制推遲 10 毫秒，讓舊的 QThread 走完結束程序並安全釋放資源。
            QTimer.singleShot(10, lambda: self._trigger_next_step(last_joints))

    def _trigger_next_step(self, last_joints):
        self.path_index += 1
        self._execute_next(last_joints)
        
    def _on_worker_error(self, msg):
        self.log_signal.emit(f"[STOP] {msg}")
        self.stop_path()

    def stop_path(self):
        # 強制發送急停指令，瞬間倒掉 Arduino 裡的水桶並煞車！
        if self.serial_manager and self.serial_manager.is_connected:
            self.serial_manager.send_command("<STOP>")
            
        if self.worker and self.worker.isRunning():
            self.worker.terminate()
            self.worker.wait()  # 等待執行緒真正被強制終止後再放手
            self.worker = None
            self.is_looping = False
            self.log_signal.emit("([STOP]) Execution Halted.")
        else:
            self.log_signal.emit("[Info] No path running.")
            
    def set_speed(self, value_0_to_100):
        self.global_speed = max(0.1, value_0_to_100 / 50.0)