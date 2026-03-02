#path_manager.py
import json
import time
import numpy as np
from PyQt5.QtCore import QObject, QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QFileDialog
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import kinematics

# --- 1. PTP 執行器 (關節插值) ---
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
            self.serial_ref.send_joints(list(self.end_joints), self.speed_factor)

        # 2. 【軟體端：乖乖播完 UI 動畫】(不提早打斷，避免滑桿瞬間跳躍引發爆衝)
        effective_duration = max(0.1, self.animation_time)
        steps = int(effective_duration * 30) 
        
        for i in range(steps + 1):
            t = i / steps
            current = self.start_joints + (self.end_joints - self.start_joints) * t
            self.update_signal.emit(list(current)) 
            time.sleep(effective_duration / steps)
        
        # 3. 【等待硬體到位】(動畫播完後，如果硬體還沒到就乖乖等它)
        if self.serial_ref and self.serial_ref.is_connected:
            if hasattr(self.serial_ref, 'wait_for_motion_complete'):
                self.serial_ref.wait_for_motion_complete(timeout=60.0)
            
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

        # 決定動畫時間
        effective_duration = max(0.1, self.animation_time)
        steps = int(effective_duration * 30)
        
        current_seed = self.start_joints.copy()
        import config
        for i in range(steps + 1):
            t = i / steps 
            curr_pos = pos_start + (pos_end - pos_start) * t
            curr_rot = slerp([t]).as_matrix()[0]
            
            T_tcp_target = np.eye(4)
            T_tcp_target[:3, :3] = curr_rot
            T_tcp_target[:3, 3] = curr_pos
            
            T_flange_target = T_tcp_target @ tcp_inv
            ik_result, error = kinematics.inverse_kinematics(T_flange_target, current_seed)
            
            if ik_result is not None:
                T_check = kinematics.forward_kinematics(ik_result)
                dist_err = np.linalg.norm(T_check[:3, 3] - T_flange_target[:3, 3]) * 1000.0
                
                if dist_err > config.IK_POS_TOLERANCE:
                    self.error_signal.emit(f"LIN Accuracy Error: {dist_err:.3f}mm at step {i}")
                    return 
                current_seed = ik_result
                
                # 1. 更新 UI 畫面
                self.update_signal.emit(list(ik_result))
                
                # 2. 🌟 【新增】：因為防護罩擋住了 UI，我們必須在這裡手動把碎點傳給 Arduino！
                if self.serial_ref and self.serial_ref.is_connected:
                    self.serial_ref.send_joints(list(ik_result), self.speed_factor)
                    
            else:
                self.error_signal.emit(f"LIN Error: Unreachable at step {i}")
                return

            time.sleep(effective_duration / steps)
            
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
            "speed": float(speed), # 將速度存入字典
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
                        # 【新增】防呆機制：如果讀取舊存檔沒有速度，自動補上 50.0
                        if 'speed' not in pt: pt['speed'] = 50.0
                
                self.list_update_signal.emit()
                self.log_signal.emit(f"Path loaded from {filename}")
            except Exception as e:
                self.log_signal.emit(f"[Error] Load failed: {e}")

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
            self._trigger_next_step(last_joints)

    def _trigger_next_step(self, last_joints):
        self.path_index += 1
        self._execute_next(last_joints)
        
    def _on_worker_error(self, msg):
        self.log_signal.emit(f"[STOP] {msg}")
        self.stop_path()

    def stop_path(self):
        if self.worker and self.worker.isRunning():
            self.worker.terminate()
            self.worker = None
            self.is_looping = False
            self.log_signal.emit("([STOP]) Execution Halted.")
        else:
            self.log_signal.emit("[Info] No path running.")
            
    def set_speed(self, value_0_to_100):
        self.global_speed = max(0.1, value_0_to_100 / 50.0)