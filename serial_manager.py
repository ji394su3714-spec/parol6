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

                        # 3. 攔截 "OK" (優化效能，不顯示)
                        if line == "OK":
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
        阻塞式等待，直到收到 "Done"
        """
        if not self.is_connected:
            return False

        # ⛔ 絕對不能在這裡 clear()！否則會把提早收到的 Done 刪掉！
        
        # 直接等待旗標
        arrived = self.motion_done_event.wait(timeout)
        
        if not arrived:
            self.log_signal.emit("[Warning] Wait Done Timeout!")
        
        return arrived