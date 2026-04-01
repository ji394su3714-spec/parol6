import serial
import serial.tools.list_ports
import time
import threading
from PyQt6.QtCore import QObject, pyqtSignal

GEAR_RATIOS = [6.4, 20.0, 18.1, 4.0, 4.0, 10.0]
MICROSTEPS = 8 
STEPS_PER_DEG = [(200.0 * MICROSTEPS * gr) / 360.0 for gr in GEAR_RATIOS]

class SerialManager(QObject):
    log_signal = pyqtSignal(str)
    connection_state_signal = pyqtSignal(bool) 

    def __init__(self):
        super().__init__()
        self.ser = None
        self.is_connected = False
        self.last_sent_time = 0
        self.send_interval = 0.05 
        
        self.read_thread = None
        self.running = False
        self.motion_done_event = threading.Event()
        self.ok_event = threading.Event() 

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
            self.ok_event.clear() 
            
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

    # 專門用來發送特殊指令 (例如急停 <STOP>) 的函式
    def send_command(self, cmd_str):
        if not self.is_connected or not self.ser: return
        try:
            # 自動補上換行符號
            if not cmd_str.endswith('\n'):
                cmd_str += '\n'
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            self.log_signal.emit(f"Send Command Error: {e}")

    def send_joints(self, joints, speed_factor=None, move_mode=0):
        """ 發送指令給 Arduino (全面升級為整數步數通訊) """
        if not self.is_connected or not self.ser: return
        self.ok_event.clear()
        self.motion_done_event.clear()
        
        try:
            steps = []
            for i in range(6):
                if joints[i] == 999.0:
                    steps.append(999999) # 用 999999 代表特殊忽略訊號
                else:
                    # 核心：直接在這裡算好步數，並強制轉為整數
                    steps.append(int(joints[i] * STEPS_PER_DEG[i]))
                    
            data_str = ",".join([str(s) for s in steps])
            
            if speed_factor is not None:
                if move_mode == 1:
                    # 在切片模式下，speed_factor 其實是 interval (秒)，直接換算成微秒(整數)
                    param7 = str(int(speed_factor * 1000000))
                else:
                    # 在 PTP 或 Jog 模式下，它是速度倍率 (浮點數)
                    param7 = f"{speed_factor:.3f}"
                    
                packet = f"<{data_str},{param7},{move_mode}>\n"
            else:
                packet = f"<{data_str},1.000,0>\n"
                
            self.ser.write(packet.encode('utf-8'))
        except Exception as e:
            self.log_signal.emit(f"Send Error: {e}")

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
                            self.ok_event.set() 
                            continue

                        self.log_signal.emit(f"[HW] {line}")
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running: 
                    self.log_signal.emit(f"Read Error: {e}")
                    break

    def wait_for_motion_complete(self, timeout=10.0):
        """ 改良版：分段式等待，直到收到 "Done" 或發生異常 """
        if not self.is_connected:
            return False

        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if not self.is_connected or not self.running:
                self.log_signal.emit("[Warning] Wait aborted (Disconnected or Stopped).")
                return False
                
            if self.motion_done_event.wait(0.1):
                return True 
                
        self.log_signal.emit("[Warning] Wait Done Timeout!")
        return False
    
    def wait_for_ok(self, timeout=0.5):
        """等待 Arduino 回傳 OK"""
        if not self.is_connected: return False
        return self.ok_event.wait(timeout)