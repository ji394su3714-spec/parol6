# serial_manager.py
import serial
import serial.tools.list_ports
import time
import threading
from PyQt5.QtCore import QObject, pyqtSignal

class SerialManager(QObject):
    log_signal = pyqtSignal(str)
    connection_state_signal = pyqtSignal(bool) # True=Connected, False=Disconnected

    def __init__(self):
        super().__init__()
        self.ser = None
        self.is_connected = False
        self.last_sent_time = 0
        self.send_interval = 0.05 # 限制發送頻率 (秒)，例如 50ms 一次，避免塞爆 Buffer
        
        # 讀取執行緒
        self.read_thread = None
        self.running = False

    def list_ports(self):
        """回傳可用 COM Ports 列表"""
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def connect(self, port, baudrate=115200):
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
            
            # 啟動讀取執行緒
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

    def send_joints(self, joints):
        """
        發送關節角度指令
        格式: <J1,J2,J3,J4,J5,J6> (單位: 度, 小數點後2位)
        狀態: Fire-and-Forget (只管送，不等待)
        """
        if not self.is_connected or not self.ser:
            return

        # ========================================================
        # 【重要】請註解掉或移除頻率控制
        # 原因：PathManager 已經有 time.sleep 控制速度了 (例如 30Hz)
        # 如果這裡又 Drop frame，路徑點會遺失，導致動作不連續
        # ========================================================
        # current_time = time.time()
        # if current_time - self.last_sent_time < self.send_interval:
        #     return
        # ========================================================

        try:
            # 格式化字串
            # joints 是 list [deg1, deg2, ...]
            # 轉成字串: "0.00,45.50,-30.20,..."
            data_str = ",".join([f"{angle:.2f}" for angle in joints])
            packet = f"<{data_str}>\n"
            
            # 直接寫入，不等待，不清除 Buffer
            self.ser.write(packet.encode('utf-8'))
            
            # 更新時間 (雖然現在沒用到頻率控制，但留著無妨)
            self.last_sent_time = time.time()
            
        except Exception as e:
            self.log_signal.emit(f"Send Error: {e}")
            # 遇到錯誤不要馬上斷線，有時候只是暫時的 Serial 雜訊
            # self.disconnect()

    def _read_loop(self):
        """背景讀取 Arduino 回傳的訊息"""
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # 可以在這裡過濾訊息，例如只顯示 Error 或 Info
                        # 這裡簡單全部印出，前面加個 [HW] 標記
                        self.log_signal.emit(f"[HW] {line}")
                else:
                    time.sleep(0.1)
            except Exception as e:
                if self.running: # 如果不是主動斷線造成的錯誤
                    self.log_signal.emit(f"Read Error: {e}")
                    break