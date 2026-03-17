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

import kinematics
import math
from motion_profile import TrapezoidalProfile

# 關節極限 (PTP 用)
MAX_JOINT_SPEED = 60.0   # 關節最高轉速 (度/秒)
MAX_JOINT_ACCEL = 60.0  # 關節最高加速度 (度/秒^2)

# 直角空間極限 (LIN 用)
MAX_LIN_SPEED = 100.0    # TCP 直線極速 (mm/秒)
MAX_LIN_ACCEL = 100.0    # TCP 直線加速度 (mm/秒^2)
MAX_ROT_SPEED = 60.0     # TCP 旋轉極速 (度/秒)
MAX_ROT_ACCEL = 60.0    # TCP 旋轉加速度 (度/秒^2)

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
        # 1. 找出移動量最大的一個關節，作為計算時間的基準 (Leader Joint)
        diffs = np.abs(self.end_joints - self.start_joints)
        max_dist_deg = np.max(diffs)
        
        # 如果距離太短，直接結束
        if max_dist_deg < 0.1:
            self.finished_signal.emit()
            return
            
        # 2. 啟動梯形加減速引擎！(套用 GUI 傳來的速度百分比)
        target_speed = MAX_JOINT_SPEED * self.speed_factor
        target_accel = MAX_JOINT_ACCEL * self.speed_factor
        profile = TrapezoidalProfile(max_dist_deg, target_speed, target_accel)
        
        # 3. 核心切片迴圈 
        interval = 0.040
        t = 0.0
        counter = 0
        gui_skip_frames = 1 
        
        while t <= profile.T_total:
            # 向引擎詢問現在的進度 (0.0 ~ 1.0)
            progress = profile.get_progress(t)
            
            # 線性內插算出當下 6 顆馬達的角度
            current = self.start_joints + (self.end_joints - self.start_joints) * progress
            
            # 更新 GUI
            if counter % gui_skip_frames == 0:
                self.update_signal.emit(list(current))
                
            # 發送給 Arduino (注意：現在 PTP 也改用 move_mode=2 塞水桶了！)
            if self.serial_ref and self.serial_ref.is_connected:
                self.serial_ref.send_joints(list(current), interval, move_mode=2)
                self.serial_ref.wait_for_ok(timeout=3.0)
                
            t += interval
            counter += 1
        
        # 算一下最後一個切片跟真正終點的誤差
        current_err = np.abs(np.array(self.end_joints) - current)
        max_err = np.max(current_err)

        self.update_signal.emit(list(self.end_joints))
        
        if self.serial_ref and self.serial_ref.is_connected:
            # 如果誤差大於 0.01 度，代表切片沒切乾淨，用極短時間(5ms)瞬間吸附
            if max_err > 0.01:
                self.serial_ref.send_joints(list(self.end_joints), 0.005, move_mode=2)
                self.serial_ref.wait_for_ok(timeout=3.0)
                
            # 整個水桶射完後，等待實體馬達把最後一點跑完
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
        
        # 1. 計算直線距離 (mm)
        dist_mm = np.linalg.norm(pos_end - pos_start) * 1000.0
        
        # 2. 計算旋轉角度 (度)
        key_rots = R.from_matrix([T_tcp_start[:3, :3], T_tcp_end[:3, :3]])
        slerp = Slerp([0, 1], key_rots)
        rot_diff = key_rots[0].inv() * key_rots[1]
        dist_deg = np.linalg.norm(rot_diff.as_rotvec()) * (180.0 / math.pi)
        
        # 3. 如果沒位移也沒旋轉，才真的不跑
        if dist_mm < 0.1 and dist_deg < 0.1:
            self.finished_signal.emit()
            return
            
        # 4. 決定這次是「平移為主」還是「旋轉為主」，套用不同的物理極限
        if dist_mm >= dist_deg:
            target_speed = MAX_LIN_SPEED * self.speed_factor
            target_accel = MAX_LIN_ACCEL * self.speed_factor
            profile = TrapezoidalProfile(dist_mm, target_speed, target_accel)
        else:
            target_speed = MAX_ROT_SPEED * self.speed_factor
            target_accel = MAX_ROT_ACCEL * self.speed_factor
            profile = TrapezoidalProfile(dist_deg, target_speed, target_accel)
            
        # 1. 啟動梯形加減速引擎！
        target_speed = MAX_LIN_SPEED * self.speed_factor
        target_accel = MAX_LIN_ACCEL * self.speed_factor
        profile = TrapezoidalProfile(dist_mm, target_speed, target_accel)

        # 2. 核心切片迴圈 
        interval = 0.040
        t = 0.0
        counter = 0
        gui_skip_frames = 1
        
        current_seed = self.start_joints.copy()
        self.update_signal.emit(list(self.start_joints))

        while t <= profile.T_total:
            progress = profile.get_progress(t)
            
            # 算出 XYZ 與姿態
            curr_pos = pos_start + (pos_end - pos_start) * progress
            curr_rot = slerp([progress]).as_matrix()[0]
            
            T_tcp_target = np.eye(4)
            T_tcp_target[:3, :3] = curr_rot
            T_tcp_target[:3, 3] = curr_pos
            T_flange_target = T_tcp_target @ tcp_inv
            
            # 逆運動學求解
            ik_result, error = kinematics.inverse_kinematics(T_flange_target, current_seed)
            if ik_result is not None:

                # 奇異點主動防禦檢查
                is_singular, warning_msg = kinematics.check_singularity(ik_result)
                if is_singular:
                    # 只要碰到奇異點，立刻拋出錯誤訊號並強制中斷整條軌跡！
                    error_str = f"軌跡中斷：在時間 {t:.2f}s 處遭遇 {warning_msg}"
                    self.error_signal.emit(error_str)
                    
                    # 為了安全，還可以補發一個 STOP 訊號給 Arduino
                    if self.serial_ref and self.serial_ref.is_connected:
                        self.serial_ref.send_command("STOP")
                        
                    return  # 直接結束 run()，絕對不把這包毒藥發給硬體！

                current_seed = ik_result
            else:
                self.error_signal.emit(f"LIN 運算錯誤: 位置無法到達 (時間 {t:.2f}s)")
                return
                
            # 發送與更新
            if counter % gui_skip_frames == 0:
                self.update_signal.emit(list(ik_result))
                
            if self.serial_ref and self.serial_ref.is_connected:
                self.serial_ref.send_joints(list(ik_result), interval, move_mode=2)
                self.serial_ref.wait_for_ok(timeout=3.0)
                
            t += interval
            counter += 1
        
        # 算一下最後一個 IK 切片跟真正終點的誤差
        current_err = np.abs(np.array(self.target_joints) - ik_result)
        max_err = np.max(current_err)

        self.update_signal.emit(list(self.target_joints))
        
        if self.serial_ref and self.serial_ref.is_connected:
            if max_err > 0.01:
                self.serial_ref.send_joints(list(self.target_joints), 0.005, move_mode=2)
                self.serial_ref.wait_for_ok(timeout=3.0)
                
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