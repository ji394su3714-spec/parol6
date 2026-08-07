# pursuit_control.py
import time
import threading
import numpy as np
from PySide6.QtCore import QThread, Signal

class PursuitWorker(QThread):
    """
    獨立於 UI 之外的 100Hz 幽靈追隨引擎 (Ghost Pursuit Engine)。
    專職處理 3D 空間拖曳時的高頻滑鼠位移，透過雙軌制 (UI無限速 / MCU嚴格限速)
    消滅緩衝區暴雨 (Buffer Bloat) 與浮點數震盪。
    """
    # 參數: (軸名稱, UI目標步數, MCU目標步數, 座標系, MCU退回步數)
    pursuit_axis_signal = Signal(str, float, object, str, float)
    # 參數: (UI目標矩陣, MCU目標矩陣, MCU退回向量)
    pursuit_tcp_signal = Signal(object, object, object) 

    def __init__(self, parent=None):
        super().__init__(parent)
        self._is_running = True
        self.lock = threading.Lock()
        
        self.mode = None
        self.axis = None
        self.frame = "Tool"
        
        self.target_step = 0.0
        self.current_step = 0.0
        
        self.target_xyz = None
        self.current_xyz = None
        
        self.max_lin_speed = 0.0
        self.max_rot_speed = 0.0
        self.speed_factor = 1.0

    def set_speed(self, max_lin, max_rot, spd_factor):
        with self.lock:
            self.max_lin_speed = max_lin
            self.max_rot_speed = max_rot
            self.speed_factor = spd_factor

    def request_axis(self, axis, step, frame):
        with self.lock:
            self.mode = 'axis'
            self.axis = axis
            self.frame = frame
            self.target_step += step 

    def request_free(self, target_xyz, current_xyz=None):
        with self.lock:
            self.mode = 'free'
            self.target_xyz = target_xyz
            if current_xyz is not None and self.current_xyz is None:
                self.current_xyz = current_xyz

    def rollback_axis(self, mcu_delta):
        with self.lock:
            self.current_step -= mcu_delta 

    def rollback_free(self, move_vec):
        with self.lock:
            if self.current_xyz is not None:
                self.current_xyz -= move_vec

    def stop_pursuit(self):
        with self.lock:
            self.mode = None
            self.target_step = 0.0
            self.current_step = 0.0
            self.current_xyz = None
            self.target_xyz = None

    def run(self):
        target_time = time.perf_counter()
        dt = 0.01 # 絕對穩定的 100Hz 節拍

        while self._is_running:
            target_time += dt
            
            with self.lock:
                mode = self.mode
                tgt_step, cur_step = self.target_step, self.current_step
                tgt, cur = self.target_xyz, self.current_xyz
                m_lin, m_rot, spd = self.max_lin_speed, self.max_rot_speed, self.speed_factor
                ax, fm = self.axis, self.frame

            if mode == 'axis':
                dist = tgt_step - cur_step
                with self.lock:
                    ui_tgt_step = tgt_step 
                    
                if abs(dist) >= 0.001:
                    max_lin_step = (m_lin * 0.5 * spd) * dt
                    max_rot_step = (m_rot * 0.5 * spd) * dt
                    limit = max_rot_step if ax and 'r' in ax else max_lin_step
                    
                    mcu_delta = max(min(dist, limit), -limit)
                    
                    with self.lock:
                        self.current_step += mcu_delta
                        new_cur = self.current_step
                        
                    self.pursuit_axis_signal.emit(ax, ui_tgt_step, new_cur, fm, mcu_delta)
                else:
                    self.pursuit_axis_signal.emit(ax, ui_tgt_step, None, fm, 0.0)

            elif mode == 'free':
                if tgt is not None and cur is not None:
                    vec = tgt - cur
                    dist = np.linalg.norm(vec)
                    
                    with self.lock:
                        ui_tgt = tgt.copy() 
                        
                    if dist >= 0.001:
                        max_lin_step = (m_lin * 0.5 * spd) * dt
                        mcu_step = min(dist, max_lin_step)
                        move_vec = (vec / dist) * mcu_step
                        
                        with self.lock:
                            self.current_xyz += move_vec
                            new_cur = self.current_xyz.copy()
                        
                        self.pursuit_tcp_signal.emit(ui_tgt, new_cur, move_vec)
                    else:
                        self.pursuit_tcp_signal.emit(ui_tgt, None, None)
            
            # 智慧等待：過濾掉 UI 執行緒的干擾
            sleep_time = target_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                target_time = time.perf_counter()