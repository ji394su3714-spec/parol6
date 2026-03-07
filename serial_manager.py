# serial_manager.py
import serial
import serial.tools.list_ports
import time
import threading
from PyQt5.QtCore import QObject, pyqtSignal

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
        self.ok_event = threading.Event() # 【新增】用於攔截 OK 訊號

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
            self.running = True
            self.motion_done_event.clear()  
            self.ok_event.clear() # 【新增】連線時清空
            
            # 連線時先重置旗標
            self.motion_done_event.clear()  
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            self.connection_state_signal.emit(True)
            self.log_signal.emit(f"Connected to {port} ({baudrate}) [DTR Disabled]")
            return True
        except Exception as e:
            self.log_signal.emit(f"Connection Failed: {e}")
            if hasattr(self, 'ser') and self.ser.is_open:
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

    def send_joints(self, joints, speed_factor=None):
        """ 發送指令給 Arduino (支援第 7 參數：全局速度比例) """
        if not self.is_connected or not self.ser: return
        
        try:
            data_str = ",".join([f"{angle:.2f}" for angle in joints])
            
            # 如果有傳入速度比例 (例如 0.5 代表 50%)
            if speed_factor is not None:
                packet = f"<{data_str},{speed_factor:.2f}>\n"
            else:
                packet = f"<{data_str}>\n"
                
            self.ser.write(packet.encode('utf-8'))
        except Exception as e:
            self.log_signal.emit(f"Send Error: {e}")

    def _read_loop(self):
        """
        背景讀取 Arduino 回傳的訊息
        """
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # 1. 攔截一般移動到位訊號 "Done"
                        if line == "Done":
                            self.motion_done_event.set() 
                            # 簡化 Log，保持介面乾淨
                            # self.log_signal.emit(">> [HW] Done") 
                            continue 

                        # 2. 攔截歸零專屬完成訊號 "HomingDone"
                        if line == "HomingDone":
                            # 如果您 Python 端有專門等待歸零的機制，可以在這裡觸發
                            # self.homing_done_event.set() 
                            
                            # 順便解鎖 motion_done_event，防止 Python 不小心卡在等待狀態
                            self.motion_done_event.set() 
                            
                            self.log_signal.emit(">> [HW] All Axes Homing Completed!") 
                            continue 

                        # 3. 攔截 "OK" 
                        if line == "OK":
                            self.ok_event.set() # 【新增】觸發 OK 旗標
                            continue

                        # 4. 其他訊息正常顯示 (包含您硬體端印出的 Timeout 警告！)
                        self.log_signal.emit(f"[HW] {line}")
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running: 
                    self.log_signal.emit(f"Read Error: {e}")
                    break

    def wait_for_motion_complete(self, timeout=10.0):
        """
        改良版：分段式等待，直到收到 "Done" 或發生異常
        """
        if not self.is_connected:
            return False

        start_time = time.time()
        
        # 將原本的一波長等待，切分成每 0.1 秒檢查一次的迴圈
        while (time.time() - start_time) < timeout:
            # 1. 容錯檢查：如果等待途中發生斷線或系統關閉，立刻跳出，不再死等
            if not self.is_connected or not self.running:
                self.log_signal.emit("[Warning] Wait aborted (Disconnected or Stopped).")
                return False
                
            # 2. 短暫等待旗標 (0.1秒)
            if self.motion_done_event.wait(0.1):
                return True # 成功收到 Done 旗標
                
        # 超過總 timeout 時間
        self.log_signal.emit("[Warning] Wait Done Timeout!")
        return False
    
    # 4. 在類別底部新增 wait_for_ok 函式
    def wait_for_ok(self, timeout=0.5):
        """等待 Arduino 回傳 OK"""
        if not self.is_connected: return False
        return self.ok_event.wait(timeout)