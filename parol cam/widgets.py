# widgets.py
import qtawesome as qta
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                               QLabel, QComboBox, QDoubleSpinBox, QGridLayout, 
                               QScrollArea, QListWidget, QSlider, QSizePolicy)
from PySide6.QtCore import Qt, QSize
import styles
import sys

def apply_windows_dark_titlebar(window):
    """將 Windows 原生標題列設為深色模式"""
    if sys.platform == "win32":
        try:
            import ctypes
            DWMWA_USE_IMMERSIVE_DARK_MODE = 20
            hwnd = int(window.winId())
            value = ctypes.c_int(2) 
            ctypes.windll.dwmapi.DwmSetWindowAttribute(
                hwnd, DWMWA_USE_IMMERSIVE_DARK_MODE, ctypes.byref(value), ctypes.sizeof(value)
            )
        except Exception as e:
            print(f"[UI] 無法套用深色標題列: {e}")

class CollapsibleSection(QWidget):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.btn_toggle = QPushButton(f"▼ {title}")
        self.btn_toggle.setCheckable(True)
        self.btn_toggle.setChecked(True)
        self.btn_toggle.setCursor(Qt.PointingHandCursor)
        self.btn_toggle.setStyleSheet(styles.STYLE_COLLAPSIBLE_BTN)
        self.btn_toggle.toggled.connect(self.toggle_content)

        self.content_area = QWidget()
        self.content_layout = QVBoxLayout(self.content_area)
        self.content_layout.setContentsMargins(10, 10, 10, 12)
        self.content_layout.setSpacing(8) 
        self.content_area.setStyleSheet("background-color: transparent;")

        layout.addWidget(self.btn_toggle)
        layout.addWidget(self.content_area)

    def toggle_content(self, checked):
        self.content_area.setVisible(checked)
        title_text = self.btn_toggle.text()[2:] 
        self.btn_toggle.setText(f"▼ {title_text}" if checked else f"▶ {title_text}")


class FloatingToolbarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("FloatingToolbar")
        self.setStyleSheet(styles.STYLE_TOOLBAR)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 5, 8, 5) 
        layout.setSpacing(5)
        
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(["平面特徵", "3D 輪廓"])
        self.cb_mode.setStyleSheet("""
            QComboBox { background-color: #333; color: white; border-radius: 4px; padding: 4px 8px; font-weight: bold; border: 1px solid #555; }
            QComboBox::drop-down { border: none; }
        """)

        # 新增：封閉/開放強制切換選單
        self.cb_loop_mode = QComboBox()
        self.cb_loop_mode.addItems(["自動判定", "強制封閉", "強制開放"])
        self.cb_loop_mode.setStyleSheet("""
            QComboBox { background-color: #2c3e50; color: #ecf0f1; border-radius: 4px; padding: 4px 8px; font-weight: bold; border: 1px solid #34495e; }
            QComboBox::drop-down { border: none; }
        """)
        
        self.btn_magic = QPushButton(qta.icon('mdi.auto-fix', color='white'), "")
        self.btn_magic.setCheckable(True) 
        self.btn_magic.setToolTip("幾何萃取")
        
        self.btn_move = QPushButton(qta.icon('mdi.cursor-move', color='white'), "")
        self.btn_scale = QPushButton(qta.icon('mdi.arrow-expand-all', color='white'), "")
        self.btn_mirror = QPushButton(qta.icon('mdi.flip-horizontal', color='white'), "")
        self.btn_add = QPushButton(qta.icon('mdi.shape-square-plus', color='white'), "")
        
        layout.addWidget(self.cb_mode)
        layout.addWidget(self.cb_loop_mode) 
        
        for btn in [self.btn_magic, self.btn_move, self.btn_scale, self.btn_mirror, self.btn_add]:
            btn.setFixedSize(40, 40)       
            btn.setIconSize(QSize(22, 22)) 
            btn.setStyleSheet(styles.STYLE_TOOLBAR_BTN)
            layout.addWidget(btn)
            
        layout.addStretch()


class PreparePanelWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff) 
        scroll.setStyleSheet(styles.STYLE_SCROLL_AREA)

        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(0)

        # 1. 系統與刀具
        sec_sys = CollapsibleSection("系統與刀具 (TCP) 綁定")
        btn_layout = QVBoxLayout() 
        btn_layout.setSpacing(5)
        self.btn_tcp_config = QPushButton("打開 TCP Manager")
        self.btn_tcp_config.setStyleSheet(styles.STYLE_BTN_NORMAL)
        self.btn_zero_joints = QPushButton("手臂歸零 (Home Pose)")
        self.btn_zero_joints.setStyleSheet(styles.STYLE_BTN_NORMAL)
        btn_layout.addWidget(self.btn_tcp_config)
        btn_layout.addWidget(self.btn_zero_joints)
        
        tool_layout = QHBoxLayout()
        tool_layout.addWidget(QLabel("綁定刀具:"))
        self.cb_tool_select = QComboBox()
        self.cb_tool_select.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        tool_layout.addWidget(self.cb_tool_select) 
        sec_sys.content_layout.addLayout(btn_layout)
        sec_sys.content_layout.addLayout(tool_layout)
        scroll_layout.addWidget(sec_sys)

        # 2. 模型位置 (增加網格面數顯示)
        sec_model = CollapsibleSection("模型位置與姿態校正")
        self.btn_load_stl = QPushButton("📂 載入 STL 檔案")
        self.btn_load_stl.setStyleSheet(styles.STYLE_BTN_LOAD)
        
        self.lbl_face_count = QLabel("網格面數: 尚未載入")
        self.lbl_face_count.setStyleSheet("color: #AAAAAA; font-size: 12px; padding-left: 2px;")
        
        sec_model.content_layout.addWidget(self.btn_load_stl)
        sec_model.content_layout.addWidget(self.lbl_face_count)
        
        grid = QGridLayout()
        grid.setSpacing(5) 
        grid.setContentsMargins(0, 5, 0, 0)
        self.model_spins = []
        labels_model = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        for i in range(6):
            lbl = QLabel(labels_model[i])
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl.setFixedWidth(20) 
            spin = QDoubleSpinBox()
            spin.setRange(-2000.0 if i < 3 else -180.0, 2000.0 if i < 3 else 180.0)
            spin.setSingleStep(10.0 if i < 3 else 5.0)
            spin.setMinimumWidth(50)
            spin.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.model_spins.append(spin)
            row = i // 2
            col = i % 2
            grid.addWidget(lbl, row, 2 * col)
            grid.addWidget(spin, row, 2 * col + 1)
        sec_model.content_layout.addLayout(grid)
        scroll_layout.addWidget(sec_model)

        # 3. 加工參數
        sec_offset = CollapsibleSection("加工姿態微調與過渡參數")
        align_layout = QHBoxLayout()
        align_layout.addWidget(QLabel("姿態計算:"))
        self.cb_align_mode = QComboBox()
        self.cb_align_mode.addItems(["最小扭轉 (雷射/塗膠/銑削)", "切線對齊 (銲接/指向性刀具)"])
        self.cb_align_mode.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        align_layout.addWidget(self.cb_align_mode)
        sec_offset.content_layout.addLayout(align_layout)
        
        safe_z_layout = QHBoxLayout()
        safe_z_layout.addWidget(QLabel("安全抬刀 (mm):"))
        self.spin_safe_z = QDoubleSpinBox()
        self.spin_safe_z.setRange(0.0, 200.0)
        self.spin_safe_z.setValue(20.0) 
        self.spin_safe_z.setMinimumWidth(50)
        self.spin_safe_z.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        safe_z_layout.addWidget(self.spin_safe_z)
        sec_offset.content_layout.addLayout(safe_z_layout)
        
        grid_offset = QGridLayout()
        grid_offset.setSpacing(5) 
        grid_offset.setContentsMargins(0, 5, 0, 0)
        self.offset_spins = []
        labels_offset = ["Rx", "Ry", "Rz"]
        for i in range(3):
            lbl = QLabel(labels_offset[i])
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl.setFixedWidth(20)
            spin = QDoubleSpinBox()
            spin.setRange(-180.0, 180.0)
            spin.setSingleStep(5.0) 
            spin.setMinimumWidth(50)
            spin.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.offset_spins.append(spin)
            row = i // 2
            col = i % 2
            grid_offset.addWidget(lbl, row, 2 * col)
            grid_offset.addWidget(spin, row, 2 * col + 1)
        sec_offset.content_layout.addLayout(grid_offset)
        scroll_layout.addWidget(sec_offset)

        # 4. 進階路徑與動態參數
        sec_adv = CollapsibleSection("進階幾何與路徑參數")
        grid_adv = QGridLayout()
        grid_adv.setSpacing(5)
        grid_adv.setContentsMargins(0, 5, 0, 0)
        
        def create_spin(val, min_v, max_v, step, decimals=2):
            s = QDoubleSpinBox()
            s.setRange(min_v, max_v)
            s.setSingleStep(step)
            s.setDecimals(decimals)
            s.setValue(val)
            s.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            s.setMinimumWidth(60)
            return s
            
        self.spin_chordal = create_spin(0.05, 0.001, 5.0, 0.01, 3)
        self.spin_max_step = create_spin(5.0, 0.1, 50.0, 0.5)
        self.spin_min_step = create_spin(0.0, 0.0, 10.0, 0.1)
        self.spin_lead_dist = create_spin(2.0, 0.0, 50.0, 0.5)
        self.spin_lead_angle = create_spin(45.0, -360.0, 360.0, 5.0)
        self.spin_overcut = create_spin(2.0, 0.0, 50.0, 0.5)
        
        adv_params = [
            ("弦向誤差 (Chordal):", self.spin_chordal),
            ("最大點距 (Max Step):", self.spin_max_step),
            ("微段濾波 (Min Step):", self.spin_min_step),
            ("引入距離 (Lead-in):", self.spin_lead_dist),
            ("引入角度 (Angle):", self.spin_lead_angle),
            ("過切距離 (Overcut):", self.spin_overcut),
        ]
        
        for i, (label_text, spin_widget) in enumerate(adv_params):
            lbl = QLabel(label_text)
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid_adv.addWidget(lbl, i, 0)
            grid_adv.addWidget(spin_widget, i, 1)
            
        sec_adv.content_layout.addLayout(grid_adv)
        scroll_layout.addWidget(sec_adv)

        # 5. 路徑管理
        sec_path = CollapsibleSection("特徵路徑清單 (Feature Manager)")
        path_btn_layout = QHBoxLayout()
        path_btn_layout.setSpacing(5)
        
        self.btn_path_merge = QPushButton("合併") 
        self.btn_path_del = QPushButton(qta.icon('mdi.trash-can-outline', color='white'), " 刪除")
        self.btn_path_up = QPushButton(qta.icon('mdi.arrow-up', color='white'), " 上移")
        self.btn_path_down = QPushButton(qta.icon('mdi.arrow-down', color='white'), " 下移")
        
        for btn in [self.btn_path_merge, self.btn_path_del, self.btn_path_up, self.btn_path_down]:
            btn.setStyleSheet(styles.STYLE_BTN_NORMAL)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed) 
            path_btn_layout.addWidget(btn)
            
        self.list_paths = QListWidget()
        self.list_paths.setMinimumHeight(180) 
        self.list_paths.setStyleSheet(styles.STYLE_LIST_WIDGET)
        self.list_paths.setSelectionMode(QListWidget.ExtendedSelection) 
        
        sec_path.content_layout.addLayout(path_btn_layout)
        sec_path.content_layout.addWidget(self.list_paths)
        scroll_layout.addWidget(sec_path)
        
        scroll_layout.addStretch() 
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)


from PySide6.QtWidgets import QSpinBox # 確保頂部 import 有包含 QSpinBox

class PreviewPanelWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self) 
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff) 
        scroll.setStyleSheet(styles.STYLE_SCROLL_AREA)

        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(0)

        sec_calc = CollapsibleSection("軌跡運算與編譯")
        
        # 動態百分比控制區 (取代寫死的參數)
        grid_spd = QGridLayout()
        grid_spd.setSpacing(5)
        grid_spd.setContentsMargins(0, 5, 0, 10)
        
        def create_spin(val):
            s = QSpinBox()
            s.setRange(1, 100)
            s.setValue(val)
            s.setSuffix(" %")
            s.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            return s
            
        self.spin_cut_speed = create_spin(30)
        self.spin_cut_accel = create_spin(100)
        self.spin_move_speed = create_spin(100)
        self.spin_move_accel = create_spin(100)
        
        grid_spd.addWidget(QLabel("切削速度 (Cut):"), 0, 0)
        grid_spd.addWidget(self.spin_cut_speed, 0, 1)
        grid_spd.addWidget(QLabel("切削加速度:"), 1, 0)
        grid_spd.addWidget(self.spin_cut_accel, 1, 1)
        
        grid_spd.addWidget(QLabel("空駛速度 (Move):"), 2, 0)
        grid_spd.addWidget(self.spin_move_speed, 2, 1)
        grid_spd.addWidget(QLabel("空駛加速度:"), 3, 0)
        grid_spd.addWidget(self.spin_move_accel, 3, 1)
        
        sec_calc.content_layout.addLayout(grid_spd)

        # 按鈕與狀態標籤
        self.btn_bake_ik = QPushButton("計算 IK 並編譯腳本 (暫存)")
        self.btn_bake_ik.setStyleSheet(styles.STYLE_BTN_PRIMARY)
        self.lbl_bake_status = QLabel("狀態：等待運算...")
        sec_calc.content_layout.addWidget(self.btn_bake_ik)
        sec_calc.content_layout.addWidget(self.lbl_bake_status)
        scroll_layout.addWidget(sec_calc)

        sec_preview = CollapsibleSection("軌跡動畫預覽")
        slider_layout = QHBoxLayout()
        self.lbl_frame = QLabel("0 / 0")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(0)
        self.slider.setEnabled(False) 
        slider_layout.addWidget(self.slider)
        slider_layout.addWidget(self.lbl_frame)
        
        self.btn_play = QPushButton("▶️ 播放預覽")
        self.btn_play.setStyleSheet(styles.STYLE_BTN_NORMAL)
        self.btn_play.setEnabled(False)
        sec_preview.content_layout.addLayout(slider_layout)
        sec_preview.content_layout.addWidget(self.btn_play)
        scroll_layout.addWidget(sec_preview)

        sec_export = CollapsibleSection("檔案匯出 (Save)")
        self.btn_save_script = QPushButton("💾 另存腳本 (匯出 JSON)")
        self.btn_save_script.setStyleSheet(styles.STYLE_BTN_SUCCESS)
        sec_export.content_layout.addWidget(self.btn_save_script)
        scroll_layout.addWidget(sec_export)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)