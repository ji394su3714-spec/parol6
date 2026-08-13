import threading  
import serial
import serial.tools.list_ports
import time
import struct
from PySide6.QtCore import QObject, Signal

class SerialManager(QObject):
    log_signal = Signal(str)
    connection_state_signal = Signal(bool)
    real_pose_received = Signal(list)

    # 廣播「急停鎖存狀態」給 UI (True=鎖定中, False=已解除)
    estop_state_signal = Signal(bool)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.is_connected = False
        self.last_sent_time = 0
        
        self._tx_lock = threading.Lock() 
        
        self.read_thread = None
        self.running = False
        self.motion_done_event = threading.Event()
        
        self.ok_semaphore = threading.Semaphore(40) 
        self.ee_done_event = threading.Event() 
        self._latched_reported = False
        
        # === 用於強制同步的旗標與變數 ===
        self.pose_received_event = threading.Event()
        self.last_real_pose = None

    def list_ports(self):
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def connect(self, port, baudrate=250000):
        if self.is_connected:
            self.disconnect()
        try:
            self.ser = serial.Serial()
            self.ser.port = port
            self.ser.baudrate = baudrate
            self.ser.timeout = 1
            
            self.ser.dtr = False 
            self.ser.rts = False 
            
            self.ser.open()            
            self.is_connected = True
            self.running = True
            
            self.motion_done_event.clear()  
            self.ee_done_event.clear() 
            
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            self.connection_state_signal.emit(True)
            self.log_signal.emit(f"Connected to {port} ({baudrate}) [Binary Mode & CRC8 Ready]")
            return True
        except Exception as e:
            self.log_signal.emit(f"Connection Failed: {e}")
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
            return False

    def disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except:
                pass
        self.is_connected = False
        self.ser = None
        self.connection_state_signal.emit(False)
        self.log_signal.emit("Disconnected.")

    def reset_semaphore(self):
        self.ok_semaphore = threading.Semaphore(40)

    # ==========================================
    # 工業級 CRC-8 計算引擎
    # ==========================================
    def _calculate_crc8(self, data_bytes: bytes) -> int:
        """與 STM32 對齊的 CRC-8 演算法 (多項式 0x07)"""
        crc = 0x00
        for byte in data_bytes:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    # ==========================================
    # 二進制發送核心
    # ==========================================
    def _send_binary_packet(self, target_steps, speed_factor, move_mode):
        """ 負責將整數陣列與參數打包成 32 Bytes 送出 """
        if not self.is_connected or not self.ser: return False
        
        try:
            header = 0x55AA
            spd = float(speed_factor)
            mode = int(move_mode)
            
            # 打包：2Bytes + 6*4Bytes + 4Bytes + 1Byte = 31 Bytes
            payload_bytes = struct.pack('<H6ifB', header, *target_steps, spd, mode)
            
            # 計算 CRC8
            crc_value = self._calculate_crc8(payload_bytes)
            
            # 附加 CRC8 成為完美的 32 Bytes
            packet = payload_bytes + struct.pack('<B', crc_value)
            
            with self._tx_lock:
                self.ser.write(packet)
                self.ser.flush()
            return True
            
        except Exception as e:
            self.log_signal.emit(f"Send Binary Error: {e}")
            return False

    # ==========================================
    # 提供給 UI 大腦呼叫的專屬 API (Public Methods)
    # ==========================================
    def send_joints(self, joints, speed_factor=1.0, move_mode=0, is_stream=False, is_jog=False):
        """ 
        is_stream: 是否為連續串流 (若為 False，則視為單點移動，會重置系統狀態)
        is_jog: 標示這是否來自於高頻點動 (UI 手動操作)
        """
        if not is_stream:
            self.motion_done_event.clear()  
            self.ee_done_event.clear() 
            self.reset_semaphore()     
        else:
            if is_jog:
                # 路線 A：點動模式 (Jog) - 極速非阻塞檢查
                if not self.ok_semaphore.acquire(blocking=False):
                    return False

            else:
                # 路線 B：CAM 路徑串流 (StreamingPathExecutor) - 絕不丟點
                pass
        
        steps = []
        import config
        for i in range(6):
            if joints[i] == 999.0 or joints[i] == 999999:
                steps.append(999999) 
            else:
                steps.append(int(round(joints[i] * config.STEPS_PER_DEG[i])))
                
        return self._send_binary_packet(steps, speed_factor, move_mode)

    def send_continuous_jog(self, axis_idx, direction, speed_factor):
        """ 發送連續寸動 (Mode 2) """
        if not self.is_connected: return False
        self.reset_semaphore() # 執行前強制重置號碼牌，消滅通膨隱患！
        
        targets = [int(axis_idx), int(direction), 0, 0, 0, 0]
        return self._send_binary_packet(targets, speed_factor, move_mode=2)

    def send_homing(self):
        """ 觸發全軸歸零 (Mode 3) """
        self.log_signal.emit("[System] 發送全軸歸零指令...")
        self.motion_done_event.clear()
        self.reset_semaphore()
        # 6 個軸皆為 999999，速度 1.0，Mode 3
        return self._send_binary_packet([999999] * 6, 1.0, 3)

    def send_stop(self):
        """ 觸發硬體急停 (Mode 4) """
        self.log_signal.emit("[System] 發送急停指令！")
        
        if self.ser and self.ser.is_open:
            try:
                self.ser.reset_output_buffer()
            except Exception:
                pass
                
        return self._send_binary_packet([0] * 6, 1.0, 4)

    def request_real_pose(self):
        """透過標準二進制通訊，發送 Mode 6 查詢硬體絕對步數"""
        if self.is_connected:
            self._send_binary_packet([0, 0, 0, 0, 0, 0], speed_factor=1.0, move_mode=6)

    def sync_get_real_pose(self, timeout=0.15):
        """同步阻塞獲取真實座標 (專給首幀對齊使用)"""
        if not self.is_connected:
            return None
            
        self.pose_received_event.clear()

        if self.ser and self.ser.is_open:
            try: self.ser.reset_input_buffer()
            except: pass

        self._send_binary_packet([0, 0, 0, 0, 0, 0], speed_factor=1.0, move_mode=6)
        
        if self.pose_received_event.wait(timeout):
            return self.last_real_pose
        return None

    def send_pause(self):
        """發送暫停指令 (Mode 7)"""
        if self.is_connected:
            self.log_signal.emit(">>> [SYS] 機台減速中...")
            # 步數陣列全填 0 即可，因為 Mode 5 只看 Mode 標籤
            self._send_binary_packet([0, 0, 0, 0, 0, 0], speed_factor=1.0, move_mode=7)

    def send_resume(self):
        """發送繼續指令 (Mode 8)"""
        if self.is_connected:
            self.log_signal.emit(">>> [SYS] 機台恢復運作...")
            self._send_binary_packet([0, 0, 0, 0, 0, 0], speed_factor=1.0, move_mode=8)
    
    def send_estop_reset(self):
        """ 發送人工解鎖指令 (Mode 9) """
        self.log_signal.emit("[System] 發送解除急停復歸訊號...")
        
        if self.ser and self.ser.is_open:
            try:
                self.ser.reset_output_buffer()
                self.ser.reset_input_buffer()
            except Exception:
                pass
                
        self.reset_semaphore() 
        self.motion_done_event.clear()
        self.ee_done_event.clear()
        
        return self._send_binary_packet([0] * 6, 1.0, 9)

    def send_gripper(self, ee_value):
        """ 觸發夾爪動作 (Mode 5) """
        self.ee_done_event.clear()
        return self._send_binary_packet([int(ee_value), 0, 0, 0, 0, 0], 1.0, 5)

    # ==========================================
    # 接收與等待區塊
    # ==========================================
    def _read_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                if not self.ser.in_waiting:
                    time.sleep(0.01)
                    continue

                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                if not line:
                    continue 

                # ==========================================
                # 遙測攔截：攔截 MCU 回傳的真實物理步數，並轉為角度
                # ==========================================
                if line.startswith("[POS]"):
                    try:
                        import config
                        data_str = line.replace("[POS]", "").strip()
                        steps = [float(x) for x in data_str.split(",")]
                        
                        if len(steps) == 6:
                            angles = [steps[i] / config.STEPS_PER_DEG[i] for i in range(6)]
                            
                            # === 新增：儲存給同步獲取使用，並觸發旗標 ===
                            self.last_real_pose = angles  
                            self.pose_received_event.set() 
                            
                            self.real_pose_received.emit(angles)
                    except Exception as e:
                        self.log_signal.emit(f"座標解析錯誤: {e}")
                    
                    continue

                # ==========================================
                # 主邏輯：平坦化的條件攔截 (Early Return / Continue)
                # ==========================================
                if line == "Done":
                    self.motion_done_event.set() 
                    continue 

                if line == "HomingDone":
                    self.motion_done_event.set() 
                    self.log_signal.emit(">> [HW] All Axes Homing Completed!") 
                    continue 

                if line == "OK":
                    self.ok_semaphore.release()
                    continue
                    
                if line == "<EE_DONE>":
                    self.ee_done_event.set()
                    continue

                # 攔截急停鎖存的錯誤與觸發訊息
                if "LATCHED" in line:
                    if not self._latched_reported:
                        self.estop_state_signal.emit(True) 
                        self.log_signal.emit("[警告] 機器處於「急停鎖存狀態」！請按「解除急停」恢復運作。")
                        self._latched_reported = True
                    
                    if "REJECTED" in line:
                        self.ok_semaphore.release()
                    continue
                    
                # 攔截急停解除訊息
                if "RESET SUCCESS" in line:
                    if self._latched_reported:
                        self.estop_state_signal.emit(False) 
                        self.log_signal.emit("[系統] 急停鎖存已解除，系統恢復就緒。")
                        self._latched_reported = False
                    continue

                # 其他未攔截的訊息，當作一般 Log 印出
                self.log_signal.emit(f"[HW] {line}")
                
            except Exception as e:
                if self.running: 
                    self.log_signal.emit(f"Read Error: {e}")
                    break

    def wait_for_motion_complete(self, timeout=10.0):
        if not self.is_connected: return False
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if not self.is_connected or not self.running:
                self.log_signal.emit("[Warning] Wait aborted.")
                return False
            if self.motion_done_event.wait(0.1):
                return True 
        self.log_signal.emit("[Warning] Wait Done Timeout!")
        return False
    
    def wait_for_ok(self, timeout=2.0):
        if not self.is_connected: return False
        return self.ok_semaphore.acquire(timeout=timeout)
        
    def wait_for_ee_done(self, timeout=10.0):
        if not self.is_connected: return False
        self.ee_done_event.clear() 
        success = self.ee_done_event.wait(timeout)
        if not success:
            self.log_signal.emit("[Warning] Wait EE_DONE Timeout!")
        return success