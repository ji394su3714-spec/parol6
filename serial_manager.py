import threading  
import serial
import serial.tools.list_ports
import time
from PySide6.QtCore import QObject, Signal

GEAR_RATIOS = [6.4, 20.0, 18.095, 4.0, 4.0, 10.0]
MICROSTEPS = 32 
STEPS_PER_DEG = [(200.0 * MICROSTEPS * gr) / 360.0 for gr in GEAR_RATIOS]

class SerialManager(QObject):
    log_signal = Signal(str)
    connection_state_signal = Signal(bool) 

    def __init__(self):
        super().__init__()
        self.ser = None
        self.is_connected = False
        self.last_sent_time = 0
        
        # 新增：這是防止執行緒打架的終極武器
        self._tx_lock = threading.Lock() 
        
        self.read_thread = None
        self.running = False
        self.motion_done_event = threading.Event()
        self.ok_semaphore = threading.Semaphore(50) # 允許最多 50 個 OK 信號在管道中等待，防止過度積壓
        
        # 🌟 新增：夾爪專用的完成事件鎖
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
            
            # 防止 Arduino 自動重啟
            self.ser.dtr = False 
            self.ser.rts = False 
            
            self.ser.open()            
            self.is_connected = True
            self.running = True
            
            # 連線時清空旗標 
            self.motion_done_event.clear()  
            self.ee_done_event.clear() # 🌟 清空夾爪旗標
            
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            self.connection_state_signal.emit(True)
            self.log_signal.emit(f"Connected to {port} ({baudrate}) [DTR Disabled]")
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

    # ==========================================
    # 加上保護鎖的發送區塊
    # ==========================================
    def send_command(self, cmd_str):
        """ 發送特殊指令 (例如急停 <STOP>, 回原點 <HOMING>, 夾爪 <EE...>) """
        if not self.is_connected or not self.ser: return False
        
        # 使用 with 確保同一時間只有一個執行緒可以發送指令
        with self._tx_lock:
            try:
                # 自動補上換行符號
                if not cmd_str.endswith('\n'):
                    cmd_str += '\n'
                self.ser.write(cmd_str.encode('utf-8'))
                self.ser.flush() # 強制清空緩衝區，立刻送出
                return True
            except Exception as e:
                self.log_signal.emit(f"Send Command Error: {e}")
                return False

    def send_joints(self, joints, speed_factor=None, move_mode=0):
        """ 發送步數指令給 Arduino """
        if not self.is_connected or not self.ser: return False
        
        self.motion_done_event.clear()
        
        try:
            steps = []
            for i in range(6):
                if joints[i] == 999.0:
                    steps.append(999999) # 忽略訊號
                else:
                    steps.append(int(round(joints[i] * STEPS_PER_DEG[i])))
                    
            data_str = ",".join([str(s) for s in steps])
            
            if speed_factor is not None:
                if move_mode == 1:
                    param7 = str(int(speed_factor * 1000000))
                else:
                    param7 = f"{speed_factor:.3f}"
                    
                packet = f"<{data_str},{param7},{move_mode}>\n"
            else:
                packet = f"<{data_str},1.000,0>\n"
                
            # 關鍵保護：將真正寫入 Serial 的行為鎖起來
            with self._tx_lock:
                self.ser.write(packet.encode('utf-8'))
            return True
                
        except Exception as e:
            self.log_signal.emit(f"Send Error: {e}")
            return False

    # ==========================================
    # 接收與等待區塊
    # ==========================================
    def _read_loop(self):
        """ 背景讀取 Arduino 回傳的訊息 """
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
                            
                        # 🌟 新增：攔截夾爪專屬的完成暗號
                        if line == "<EE_DONE>":
                            self.ee_done_event.set()
                            continue

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
                self.log_signal.emit("[Warning] Wait aborted (Disconnected or Stopped).")
                return False
            if self.motion_done_event.wait(0.1):
                return True 
        self.log_signal.emit("[Warning] Wait Done Timeout!")
        return False
    
    def wait_for_ok(self, timeout=2.0):
        if not self.is_connected: return False
        return self.ok_semaphore.acquire(timeout=timeout)
        
    # 🌟 新增：專門讓大腦等待夾爪完成的方法
    def wait_for_ee_done(self, timeout=10.0):
        """ 等待 Arduino 回傳 <EE_DONE>，具有超時保護 """
        if not self.is_connected: return False
        
        self.ee_done_event.clear() # 等待前先確保鎖是乾淨的
        success = self.ee_done_event.wait(timeout)
        
        if not success:
            self.log_signal.emit("[Warning] Wait EE_DONE Timeout! (夾爪可能卡住或未回應)")
            
        return success