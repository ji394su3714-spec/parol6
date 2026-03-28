from PyQt6.QtWidgets import (QDialog, QVBoxLayout, QFormLayout, QLineEdit, 
                             QPushButton, QHBoxLayout, QMessageBox, QGroupBox)
from PyQt6.QtGui import QDoubleValidator, QIntValidator
import path_manager

class AdvancedSettingsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("進階運動參數設定 (Advanced Settings)")
        self.setMinimumWidth(400)
        
        layout = QVBoxLayout(self)
        
        # 限制只能輸入數字 (浮點數與整數)
        double_val = QDoubleValidator(0.0, 100000.0, 2)
        int_val = QIntValidator(1, 200000)
        
        # --- 1. 笛卡爾空間極限 ---
        group_cartesian = QGroupBox("笛卡爾空間極限 (Cartesian Limits)")
        form_cartesian = QFormLayout()
        
        self.lin_spd = QLineEdit(str(path_manager.MAX_LIN_SPEED))
        self.lin_spd.setValidator(double_val)
        form_cartesian.addRow("TCP 直線極速 (mm/s):", self.lin_spd)
        
        self.lin_acc = QLineEdit(str(path_manager.MAX_LIN_ACCEL))
        self.lin_acc.setValidator(double_val)
        form_cartesian.addRow("TCP 直線加速度 (mm/s²):", self.lin_acc)
        
        self.rot_spd = QLineEdit(str(path_manager.MAX_ROT_SPEED))
        self.rot_spd.setValidator(double_val)
        form_cartesian.addRow("TCP 旋轉極速 (度/s):", self.rot_spd)
        
        self.rot_acc = QLineEdit(str(path_manager.MAX_ROT_ACCEL))
        self.rot_acc.setValidator(double_val)
        form_cartesian.addRow("TCP 旋轉加速度 (度/s²):", self.rot_acc)
        
        group_cartesian.setLayout(form_cartesian)
        layout.addWidget(group_cartesian)
        
        # --- 2. 晶片總體算力防護網 ---
        group_pulse = QGroupBox("晶片總體算力防護 (Pulse Limits)")
        form_pulse = QFormLayout()
        
        self.slice_pulse = QLineEdit(str(path_manager.MAX_TOTAL_PULSE_SLICE))
        self.slice_pulse.setValidator(double_val)
        form_pulse.addRow("切片模式算力極限 (Hz):", self.slice_pulse)
        
        self.native_pulse = QLineEdit(str(path_manager.MAX_TOTAL_PULSE_NATIVE))
        self.native_pulse.setValidator(double_val)
        form_pulse.addRow("原生模式算力極限 (Hz):", self.native_pulse)
        
        group_pulse.setLayout(form_pulse)
        layout.addWidget(group_pulse)
        
        # --- 3. 底層硬體物理天花板與動態參數 ---
        group_hw = QGroupBox("硬體底層設定 (Hardware Config)")
        form_hw = QFormLayout()
        
        self.hw_max = QLineEdit(str(int(path_manager.HW_MAX_STEPS[0])))
        self.hw_max.setValidator(int_val)
        form_hw.addRow("馬達物理極限 (Steps/s):", self.hw_max)
        
        self.t_acc = QLineEdit(str(getattr(path_manager, 'N_PTP_T_ACC', 0.15)))
        self.t_acc.setValidator(double_val)
        form_hw.addRow("N_PTP 起步加減速時間 (秒):", self.t_acc)
        
        group_hw.setLayout(form_hw)
        layout.addWidget(group_hw)
        
        # --- 按鈕區 ---
        btn_layout = QHBoxLayout()
        save_btn = QPushButton("儲存並套用硬體 (Save & Apply)")
        save_btn.clicked.connect(self.save_settings)
        cancel_btn = QPushButton("取消 (Cancel)")
        cancel_btn.clicked.connect(self.reject)
        
        btn_layout.addWidget(save_btn)
        btn_layout.addWidget(cancel_btn)
        layout.addLayout(btn_layout)
        
    def save_settings(self):
        try:
            # 讀取輸入框的數值
            vals = {
                'lin_spd': float(self.lin_spd.text()),
                'lin_acc': float(self.lin_acc.text()),
                'rot_spd': float(self.rot_spd.text()),
                'rot_acc': float(self.rot_acc.text()),
                'slice_pulse': float(self.slice_pulse.text()),
                'native_pulse': float(self.native_pulse.text()),
                'hw_max': float(self.hw_max.text()),
                't_acc': float(self.t_acc.text())
            }
            # 傳給 path_manager 進行全域更新與硬體同步
            path_manager.update_advanced_settings(**vals)
            self.accept()
        except ValueError:
            QMessageBox.warning(self, "錯誤", "請確保所有欄位皆為有效的數字！")