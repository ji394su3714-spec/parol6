# end_effector.py
from PySide6.QtWidgets import QWidget, QPushButton, QVBoxLayout, QSlider, QLabel
from PySide6.QtCore import Qt

class BaseEndEffector:
    """所有末端執行器的基礎介面"""
    def __init__(self, name):
        self.name = name
        self.type_id = "BASE"

    def get_control_widget(self, main_gui=None):
        raise NotImplementedError

    def get_action_dict(self):
        raise NotImplementedError

# --- 具體實作 1：數位開關 (氣壓夾爪 / 吸盤) ---
class DigitalGripper(BaseEndEffector):
    def __init__(self, name="氣壓夾爪 (Digital)"):
        super().__init__(name)
        self.type_id = "DIGITAL"
        self.is_closed = False

    def get_control_widget(self, main_gui=None):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.btn = QPushButton("🔴 夾緊 (CLOSE)" if not self.is_closed else "🟢 鬆開 (OPEN)")
        self.btn.setMinimumHeight(40)
        
        def toggle():
            self.is_closed = not self.is_closed
            self.btn.setText("🟢 鬆開 (OPEN)" if self.is_closed else "🔴 夾緊 (CLOSE)")
            # [預留] 傳送即時指令給 Arduino
            if main_gui and main_gui.serial_manager and main_gui.serial_manager.is_connected:
                main_gui.serial_manager.send_command(f"<EE,DIGITAL,{1 if self.is_closed else 0}>")
                
        self.btn.clicked.connect(toggle)
        layout.addWidget(self.btn)
        return widget

    def get_action_dict(self):
        # 回傳要存進 JSON 路徑檔的資料
        return {
            "action_type": "DIGITAL", 
            "value": 1 if self.is_closed else 0, 
            "note": "夾緊" if self.is_closed else "鬆開"
        }

# --- 具體實作 2：伺服控制 (PWM 夾爪) ---
class ServoGripper(BaseEndEffector):
    def __init__(self, name="伺服夾爪 (Servo)"):
        super().__init__(name)
        self.type_id = "SERVO"
        self.position = 0 # 開合度 0~100%

    def get_control_widget(self, main_gui=None):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.lbl = QLabel(f"開合度: {self.position}%")
        
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(self.position)
        
        def on_change(val):
            self.position = val
            self.lbl.setText(f"開合度: {self.position}%")
            
        def on_release():
            # [預留] 放開滑桿時，傳送即時指令給 Arduino
            if main_gui and main_gui.serial_manager and main_gui.serial_manager.is_connected:
                main_gui.serial_manager.send_command(f"<EE,SERVO,{self.position}>")
                
        slider.valueChanged.connect(on_change)
        slider.sliderReleased.connect(on_release)
        
        layout.addWidget(self.lbl)
        layout.addWidget(slider)
        return widget

    def get_action_dict(self):
        return {
            "action_type": "SERVO", 
            "value": self.position, 
            "note": f"開合度 {self.position}%"
        }

# --- 管理員：負責切換當前工具 ---
class EndEffectorManager:
    def __init__(self):
        self.available_tools = {
            "DIGITAL": DigitalGripper(),
            "SERVO": ServoGripper()
        }
        self.active_tool_id = "SERVO" # 預設使用伺服夾爪

    def get_active_tool(self):
        return self.available_tools[self.active_tool_id]
        
    def set_active_tool(self, tool_id):
        if tool_id in self.available_tools:
            self.active_tool_id = tool_id