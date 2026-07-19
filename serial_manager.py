import threading  
import serial
import serial.tools.list_ports
import time
import struct
from PySide6.QtCore import QObject, Signal

class SerialManager(QObject):
    log_signal = Signal(str)
    connection_state_signal = Signal(bool) 

    # 新增：專門用來廣播「急停鎖存狀態」給 UI 的訊號 (True=鎖定中, False=已解除)
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
        
        # 與 MCU (50+10) 完美搭配的令牌桶上限
        self.ok_semaphore = threading.Semaphore(40) 
        self.ee_done_event = threading.Event() 

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
        """ 底層引擎：只負責將整數陣列與參數打包成 32 Bytes 送出 """
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
    def send_joints(self, joints, speed_factor=1.0, move_mode=0, is_stream=False):
        """ 一般運動發送 (自動將角度轉為絕對步數)。若為串流，不可重置號碼牌與事件"""
        # 只有在「非串流」的全新指令時，才重置事件與號碼牌
        if not is_stream:
            self.motion_done_event.clear()  
            self.ee_done_event.clear() 
            self.reset_semaphore()     
        
        steps = []
        import config
        
        for i in range(6):
            if joints[i] == 999.0 or joints[i] == 999999:
                steps.append(999999) 
            else:
                steps.append(int(round(joints[i] * config.STEPS_PER_DEG[i])))
                
        return self._send_binary_packet(steps, speed_factor, move_mode)

    def send_homing(self):
        """ 觸發全軸歸零 (對應 C++ 的 Mode 3) """
        self.log_signal.emit("[System] 發送全軸歸零指令...")
        self.motion_done_event.clear()
        self.reset_semaphore()
        # 6 個軸皆為 999999，速度 1.0，Mode 3
        return self._send_binary_packet([999999] * 6, 1.0, 3)

    def send_stop(self):
        """ 觸發硬體急停 (對應 C++ 的 Mode 4) """
        self.log_signal.emit("[System] 發送急停指令！")
        return self._send_binary_packet([0] * 6, 1.0, 4)
    
    def send_estop_reset(self):
        """ 明確向 MCU 發送人工解鎖指令 (Mode 9) """
        self.log_signal.emit("[System] 正在發送解除急停復歸訊號...")
        return self._send_binary_packet([0] * 6, 1.0, 9)

    def send_gripper(self, ee_value):
        """ 觸發夾爪動作 (對應 C++ 的 Mode 5) """
        self.ee_done_event.clear()
        # 夾爪值放在 targets[0]，Mode 5
        return self._send_binary_packet([int(ee_value), 0, 0, 0, 0, 0], 1.0, 5)

    # ==========================================
    # 接收與等待區塊
    # ==========================================
    def _read_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line: 
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
                            self.estop_state_signal.emit(True) # 廣播給 UI：系統鎖死了！
                            self.log_signal.emit("[警告] 機器處於「急停鎖存狀態」！請按「解除急停」恢復運作。")
                            
                            # 如果是因為被拒絕而產生的錯誤，釋放一個號碼牌，避免 Python 端的 Semaphore 卡死
                            if "REJECTED" in line:
                                self.ok_semaphore.release()
                            continue
                            
                        # 攔截急停解除訊息
                        if "RESET SUCCESS" in line:
                            self.estop_state_signal.emit(False) # 廣播給 UI：警報解除！
                            self.log_signal.emit("[系統] 急停鎖存已解除，系統恢復就緒。")
                            continue

                        # 其他未攔截的訊息，當作一般 Log 印出
                        self.log_signal.emit(f"[HW] {line}")
                else:
                    time.sleep(0.01)
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