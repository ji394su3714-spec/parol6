# path_manager.py
import json
import time
import numpy as np
import math
from PyQt5.QtCore import QObject, QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QFileDialog
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

import kinematics
from motion_profile import TrapezoidalProfile

# 關節極限 (PTP 用)
MAX_JOINT_SPEED = 180.0   # 關節最高轉速 (度/秒)
MAX_JOINT_ACCEL = 180.0   # 關節最高加速度 (度/秒^2)

# 直角空間極限 (LIN/CIRC 用)
MAX_LIN_SPEED = 200.0    # TCP 直線極速 (mm/秒)
MAX_LIN_ACCEL = 200.0    # TCP 直線加速度 (mm/秒^2)
MAX_ROT_SPEED = 180.0     # TCP 旋轉極速 (度/秒)
MAX_ROT_ACCEL = 180.0     # TCP 旋轉加速度 (度/秒^2)


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

    def run(self):
        diffs = np.abs(self.end_joints - self.start_joints)
        max_dist_deg = np.max(diffs)
        
        if max_dist_deg < 0.1:
            self.finished_signal.emit()
            return
            
        target_speed = MAX_JOINT_SPEED * self.speed_factor
        target_accel = MAX_JOINT_ACCEL * self.speed_factor
        profile = TrapezoidalProfile(max_dist_deg, target_speed, target_accel)
        
        interval = 0.030
        t = 0.0
        counter = 0
        gui_skip_frames = 10 
        
        while t <= profile.T_total:
            progress = profile.get_progress(t)
            current = self.start_joints + (self.end_joints - self.start_joints) * progress
            
            if counter % gui_skip_frames == 0:
                self.update_signal.emit(list(current))
                
            if self.serial_ref and self.serial_ref.is_connected:
                self.serial_ref.send_joints(list(current), interval, move_mode=1)
                self.serial_ref.wait_for_ok(timeout=3.0)
                
            t += interval
            counter += 1
        
        current_err = np.abs(np.array(self.end_joints) - current)
        max_err = np.max(current_err)

        self.update_signal.emit(list(self.end_joints))
        
        if self.serial_ref and self.serial_ref.is_connected:
            if max_err > 0.01:
                self.serial_ref.send_joints(list(self.end_joints), 0.005, move_mode=1)
                self.serial_ref.wait_for_ok(timeout=3.0)
                
            if hasattr(self.serial_ref, 'wait_for_motion_complete'):
                self.serial_ref.wait_for_motion_complete(timeout=10.0)
                
        self.finished_signal.emit()


# --- 2. 笛卡爾空間執行器 (處理 LIN 與 CIRC) ---
class CartesianExecutor(QThread):
    update_signal = pyqtSignal(list)
    finished_signal = pyqtSignal()
    error_signal = pyqtSignal(str)

    def __init__(self, start_joints, target_joints, tcp_offset_mat, serial_ref=None, speed_factor=1.0, animation_time=1.0, move_type="LIN", aux_joints=None):
        super().__init__()
        self.start_joints = np.array(start_joints)
        self.target_joints = np.array(target_joints)
        self.tcp_offset_mat = tcp_offset_mat if tcp_offset_mat is not None else np.eye(4)
        self.speed_factor = speed_factor
        self.serial_ref = serial_ref
        self.move_type = move_type
        self.aux_joints = np.array(aux_joints) if aux_joints is not None else None

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
        
        # 姿態旋轉預備 (四元數 Slerp)
        key_rots = R.from_matrix([T_tcp_start[:3, :3], T_tcp_end[:3, :3]])
        slerp = Slerp([0, 1], key_rots)
        rot_diff = key_rots[0].inv() * key_rots[1]
        dist_deg = np.linalg.norm(rot_diff.as_rotvec()) * (180.0 / math.pi)

        # --- CIRC 圓弧數學引擎 ---
        is_circ = False
        dist_mm = 0.0

        if self.move_type == "CIRC" and self.aux_joints is not None:
            T_flange_aux = kinematics.forward_kinematics(self.aux_joints)
            T_tcp_aux = T_flange_aux @ self.tcp_offset_mat
            pos_aux = T_tcp_aux[:3, 3]

            # 計算空間三點的三角形向量
            u = pos_aux - pos_start
            w = pos_end - pos_start
            cross_uw = np.cross(u, w)
            cross_norm = np.linalg.norm(cross_uw)

            if cross_norm > 1e-6: # 防呆：確保三點不共線
                is_circ = True
                u2 = np.dot(u, u)
                w2 = np.dot(w, w)
                
                # 計算外心 (Center)
                C = pos_start + np.cross((u2 * w - w2 * u), cross_uw) / (2.0 * cross_norm**2)
                r = np.linalg.norm(pos_start - C)
                
                # 建立圓平面的局部座標系 (以 C 為原點)
                n = cross_uw / cross_norm     # Z 軸 (法向量)
                x_axis = (pos_start - C) / r  # X 軸 (指向起點)
                y_axis = np.cross(n, x_axis)  # Y 軸
                
                # 計算終點所在的角度
                theta_e = math.atan2(np.dot(pos_end - C, y_axis), np.dot(pos_end - C, x_axis))
                if theta_e < 0: 
                    theta_e += 2 * math.pi # 確保順著外積法線方向總是正角
                
                # 弧長 = 圓心角(弧度) * 半徑
                dist_mm = r * theta_e * 1000.0 
            else:
                print("[Warning] Points are collinear. Falling back to LIN.")
                dist_mm = np.linalg.norm(pos_end - pos_start) * 1000.0
        else:
            dist_mm = np.linalg.norm(pos_end - pos_start) * 1000.0
            
        # 距離太短防呆
        if dist_mm < 0.1 and dist_deg < 0.1:
            self.finished_signal.emit()
            return
            
        # --- 瓶頸時間同步法 ---
        time_for_lin = dist_mm / (MAX_LIN_SPEED * self.speed_factor) if MAX_LIN_SPEED > 0 else 0
        time_for_rot = dist_deg / (MAX_ROT_SPEED * self.speed_factor) if MAX_ROT_SPEED > 0 else 0

        if time_for_lin >= time_for_rot:
            target_speed = MAX_LIN_SPEED * self.speed_factor
            target_accel = MAX_LIN_ACCEL * self.speed_factor
            profile = TrapezoidalProfile(dist_mm, target_speed, target_accel)
        else:
            target_speed = MAX_ROT_SPEED * self.speed_factor
            target_accel = MAX_ROT_ACCEL * self.speed_factor
            profile = TrapezoidalProfile(dist_deg, target_speed, target_accel)

        interval = 0.030
        t = 0.0
        counter = 0
        gui_skip_frames = 10
        
        current_seed = self.start_joints.copy()
        self.update_signal.emit(list(self.start_joints))

        while t <= profile.T_total:
            progress = profile.get_progress(t) 
            
            # --- 位置內插：圓弧 or 直線 ---
            if is_circ:
                theta = progress * theta_e
                curr_pos = C + r * math.cos(theta) * x_axis + r * math.sin(theta) * y_axis
            else:
                curr_pos = pos_start + (pos_end - pos_start) * progress
                
            # --- 姿態內插：永遠走 Slerp 最短路徑 ---
            curr_rot = slerp([progress]).as_matrix()[0]
            
            T_tcp_target = np.eye(4)
            T_tcp_target[:3, :3] = curr_rot
            T_tcp_target[:3, 3] = curr_pos
            T_flange_target = T_tcp_target @ tcp_inv
            
            ik_result, error = kinematics.inverse_kinematics(T_flange_target, current_seed)
            if ik_result is not None:
                is_singular, warning_msg = kinematics.check_singularity(ik_result)
                if is_singular:
                    self.error_signal.emit(f"軌跡中斷：遭遇 {warning_msg}")
                    if self.serial_ref and self.serial_ref.is_connected:
                        self.serial_ref.send_command("STOP")
                    return 

                current_seed = ik_result
            else:
                self.error_signal.emit(f"軌跡運算錯誤: 位置無法到達 (時間 {t:.2f}s)")
                return
                
            if counter % gui_skip_frames == 0:
                self.update_signal.emit(list(ik_result))
                
            if self.serial_ref and self.serial_ref.is_connected:
                self.serial_ref.send_joints(list(ik_result), interval, move_mode=1)
                self.serial_ref.wait_for_ok(timeout=3.0)
                
            t += interval
            counter += 1
        
        current_err = np.abs(np.array(self.target_joints) - ik_result)
        max_err = np.max(current_err)

        self.update_signal.emit(list(self.target_joints))
        
        if self.serial_ref and self.serial_ref.is_connected:
            if max_err > 0.01:
                self.serial_ref.send_joints(list(self.target_joints), 0.005, move_mode=1)
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
        
        # 暫存區：用來裝載 CIRC 的中繼點
        self.temp_aux_joints = None 
        
        self.serial_manager = None
        if hasattr(parent, 'serial_manager'):
            self.serial_manager = parent.serial_manager

    # 設定中繼點
    def set_aux_point(self, joints):
        self.temp_aux_joints = list(joints)
        self.log_signal.emit(">> [CIRC] AUX point saved! Move to END point and press Record.")

    def record_point(self, current_joints, delay=0.0, move_type="PTP", speed=50.0):
        aux = None
        # 智慧判定：如果剛剛有設定 AUX，這次的記錄強制轉為 CIRC 圓弧
        if self.temp_aux_joints is not None:
            move_type = "CIRC"
            aux = self.temp_aux_joints
            self.temp_aux_joints = None # 用完即銷毀，保持乾淨

        idx = len(self.waypoints) + 1
        name = f"Point {idx}"
        data = {
            "name": name,
            "joints": list(current_joints),
            "aux_joints": aux,  # 存入中繼點
            "delay": float(delay),
            "type": move_type,
            "speed": float(speed), 
            "active": True,
            "note": ""
        }
        self.waypoints.append(data)
        self.list_update_signal.emit()
        
        msg = f"Recorded: {name} [{move_type}]"
        if move_type == "CIRC":
            msg += " (with AUX point)"
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
                        if 'speed' not in pt: pt['speed'] = 50.0
                
                self.list_update_signal.emit()
                self.log_signal.emit(f"Path loaded from {filename}")
            except Exception as e:
                self.log_signal.emit(f"[Error] Load failed: {e}")

    # 讓 3D 模擬畫面畫出線條
    def get_trajectory_preview(self, tcp_offset):
        trajectory_points = []
        active_wps = [pt for pt in self.waypoints if pt.get('active', True)]
        
        if len(active_wps) >= 2:
            for i in range(len(active_wps) - 1):
                j_start = np.array(active_wps[i]['joints'])
                j_end = np.array(active_wps[i+1]['joints'])
                m_type = active_wps[i+1].get('type', 'LIN') 
                aux_joints = active_wps[i+1].get('aux_joints', None)
                
                steps = 20
                if m_type == 'PTP':
                    for t in np.linspace(0, 1, steps):
                        interp = j_start + t * (j_end - j_start)
                        T_tcp = kinematics.forward_kinematics(interp) @ tcp_offset
                        trajectory_points.append(T_tcp[:3, 3])
                        
                elif m_type == 'CIRC' and aux_joints is not None:
                    T_s = kinematics.forward_kinematics(j_start) @ tcp_offset
                    T_a = kinematics.forward_kinematics(aux_joints) @ tcp_offset
                    T_e = kinematics.forward_kinematics(j_end) @ tcp_offset
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
                            
                else: # LIN
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
        aux_joints = target_data.get('aux_joints', None) # 提取 AUX 點
        
        point_speed_pct = target_data.get('speed', 50.0)
        speed_factor = (point_speed_pct / 100.0) * self.global_speed
        if speed_factor > 1.0: speed_factor = 1.0 
        
        self.log_signal.emit(f"Moving -> {name} ({move_type}, SPD:{speed_factor*100:.0f}%)...")
        
        if move_type in ["LIN", "CIRC"]:
            self.worker = CartesianExecutor(
                start_joints=current_joints, 
                target_joints=target_joints, 
                tcp_offset_mat=self.current_tcp_offset, 
                serial_ref=self.serial_manager,  
                speed_factor=speed_factor,
                move_type=move_type,
                aux_joints=aux_joints  # 把 AUX 交給執行器
            )
            self.worker.error_signal.connect(self._on_worker_error)
        else:
            self.worker = PTPExecutor(
                start_joints=current_joints, 
                end_joints=target_joints, 
                serial_ref=self.serial_manager,  
                speed_factor=speed_factor,     
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
            QTimer.singleShot(10, lambda: self._trigger_next_step(last_joints))

    def _trigger_next_step(self, last_joints):
        self.path_index += 1
        self._execute_next(last_joints)
        
    def _on_worker_error(self, msg):
        self.log_signal.emit(f"[STOP] {msg}")
        self.stop_path()

    def stop_path(self):
        if self.serial_manager and self.serial_manager.is_connected:
            self.serial_manager.send_command("STOP")
            
        if self.worker and self.worker.isRunning():
            self.worker.terminate()
            self.worker.wait()  
            self.worker = None
            self.is_looping = False
            self.log_signal.emit("([STOP]) Execution Halted.")
        else:
            self.log_signal.emit("[Info] No path running.")
            
    def set_speed(self, value_0_to_100):
        self.global_speed = max(0.1, value_0_to_100 / 50.0)