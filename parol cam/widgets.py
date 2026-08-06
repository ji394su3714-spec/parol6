# widgets.py
import qtawesome as qta
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                               QLabel, QComboBox, QDoubleSpinBox, QGridLayout, 
                               QScrollArea, QListWidget, QSlider, QSizePolicy, 
                               QTextEdit, QSpinBox, QButtonGroup, QStackedWidget)
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QTextCursor
import styles
import sys

def apply_windows_dark_titlebar(window):
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

        self.btn_toggle = QPushButton(f" {title}")
        self.btn_toggle.setIcon(qta.icon('mdi.chevron-down', color='#ffffff'))
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
        icon_name = 'mdi.chevron-down' if checked else 'mdi.chevron-right'
        icon_color = '#ffffff' if checked else '#dddddd'
        self.btn_toggle.setIcon(qta.icon(icon_name, color=icon_color))


class LogPanelWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, False)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setFixedSize(450, 250)
        self.text_edit.setStyleSheet(styles.STYLE_LOG_PANEL)
        self.text_edit.setVisible(False)
        layout.addWidget(self.text_edit)
        
    def toggle_log(self, checked):
        self.text_edit.setVisible(checked)
        
    def append_log(self, text):
        self.text_edit.moveCursor(QTextCursor.End)
        self.text_edit.insertPlainText(text)
        self.text_edit.moveCursor(QTextCursor.End)

# (在 widgets.py 裡面，完全替換 ModelTransformWidget)

class ModelTransformWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, False)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.container = QWidget()
        self.container.setObjectName("TransformContainer")
        self.container.setStyleSheet(styles.STYLE_TRANSFORM_PANEL)
        self.container.setVisible(False)
        self.container.setFixedWidth(250) # 👑 嚴格限制整個面板的寬度為 250px
        
        grid = QGridLayout(self.container)
        grid.setContentsMargins(12, 12, 12, 12)
        grid.setSpacing(8)
        
        self.spins = []
        labels = ["X:", "Y:", "Z:", "Rx:", "Ry:", "Rz:"]
        
        # 👑 變更為 2欄 3列 的排版 (左欄: XYZ, 右欄: RxRyRz)
        for i in range(6):
            lbl = QLabel(labels[i])
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            spin = QDoubleSpinBox()
            spin.setRange(-2000.0 if i < 3 else -180.0, 2000.0 if i < 3 else 180.0)
            spin.setSingleStep(10.0 if i < 3 else 5.0)
            spin.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed) # 👑 彈性填滿剩餘空間，防止爆框
            
            self.spins.append(spin)
            
            row = i % 3  # 會變成 0, 1, 2, 0, 1, 2
            col = i // 3 # 會變成 0, 0, 0, 1, 1, 1
            
            grid.addWidget(lbl, row, col * 2)
            grid.addWidget(spin, row, col * 2 + 1)
            
        layout.addWidget(self.container)
        
    def set_panel_visible(self, visible):
        self.container.setVisible(visible)
        
    @property
    def is_panel_visible(self):
        return self.container.isVisible()

class FloatingToolbarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("FloatingToolbar")
        self.setStyleSheet(styles.STYLE_TOOLBAR)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4) 
        layout.setSpacing(4)
        
        self.btn_load_stl = QPushButton(qta.icon('mdi.folder-open', color='white'), "")
        self.btn_load_stl.setToolTip("載入 STL 模型")
        
        self.cb_tool_select = QComboBox()
        self.cb_tool_select.setToolTip("選擇工作刀具")
        self.cb_tool_select.setStyleSheet(styles.STYLE_TOOLBAR_COMBO)
        
        self.btn_tcp_config = QPushButton(qta.icon('mdi.wrench', color='white'), "")
        self.btn_tcp_config.setToolTip("打開 TCP Manager")
        
        self.btn_zero_joints = QPushButton(qta.icon('mdi.home', color='white'), "")
        self.btn_zero_joints.setToolTip("手臂歸零 (Home Pose)")
        
        for btn in [self.btn_load_stl, self.btn_tcp_config, self.btn_zero_joints]:
            btn.setFixedSize(32, 32)
            btn.setIconSize(QSize(18, 18))
            btn.setStyleSheet(styles.STYLE_TOOLBAR_BTN)
            layout.addWidget(btn)
            
        layout.insertWidget(1, self.cb_tool_select)
        
        sep1 = QWidget()
        sep1.setFixedSize(1, 20)
        sep1.setStyleSheet("background-color: #444;")
        layout.addWidget(sep1)
        
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(["平面特徵", "3D 輪廓"])
        self.cb_mode.setStyleSheet(styles.STYLE_TOOLBAR_COMBO)

        self.cb_loop_mode = QComboBox()
        self.cb_loop_mode.addItems(["自動判定", "強制封閉", "強制開放"])
        self.cb_loop_mode.setStyleSheet(styles.STYLE_TOOLBAR_COMBO_HIGHLIGHT)
        
        self.btn_magic = QPushButton(qta.icon('mdi.auto-fix', color='white'), "")
        self.btn_magic.setCheckable(True) 
        self.btn_magic.setToolTip("啟用幾何萃取魔術棒")
        
        self.btn_move = QPushButton(qta.icon('mdi.cursor-move', color='white'), "")
        self.btn_scale = QPushButton(qta.icon('mdi.arrow-expand-all', color='white'), "")
        self.btn_mirror = QPushButton(qta.icon('mdi.flip-horizontal', color='white'), "")
        self.btn_add = QPushButton(qta.icon('mdi.shape-square-plus', color='white'), "")
        
        layout.addWidget(self.cb_mode)
        layout.addWidget(self.cb_loop_mode) 
        
        for btn in [self.btn_magic, self.btn_move, self.btn_scale, self.btn_mirror, self.btn_add]:
            btn.setFixedSize(32, 32)       
            btn.setIconSize(QSize(18, 18)) 
            btn.setStyleSheet(styles.STYLE_TOOLBAR_BTN)
            layout.addWidget(btn)
            
        layout.addStretch()


class FloatingPlaybackWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("FloatingPlayback")
        self.setStyleSheet(styles.STYLE_PLAYBACK_PANEL)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(15, 8, 15, 8) 
        layout.setSpacing(15)
        
        self.btn_play = QPushButton("▶️ 播放")
        self.btn_play.setEnabled(False)
        self.btn_play.setFixedSize(85, 26)
        
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimumWidth(300)
        self.slider.setEnabled(False)
        
        self.lbl_frame = QLabel("0 / 0")
        
        layout.addWidget(self.btn_play)
        layout.addWidget(self.slider)
        layout.addWidget(self.lbl_frame)


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

        # 1. 參數區塊 (單欄排列 + 底部橫線風格)
        sec_params = CollapsibleSection("特徵參數設定 (Properties)")
        
        # --- 小標籤切換列 ---
        tab_layout = QHBoxLayout()
        tab_layout.setSpacing(0)
        tab_layout.setContentsMargins(5, 0, 5, 5) # 底部留一點間隙
        
        self.btn_tab_tcp = QPushButton("TCP 姿態")
        self.btn_tab_geo = QPushButton("幾何路徑")
        self.btn_tab_spd = QPushButton("速度動態")
        
        self.param_tab_group = QButtonGroup(self)
        self.param_tab_group.setExclusive(True)
        
        for i, btn in enumerate([self.btn_tab_tcp, self.btn_tab_geo, self.btn_tab_spd]):
            btn.setCheckable(True)
            btn.setStyleSheet(styles.STYLE_INNER_TAB)
            self.param_tab_group.addButton(btn, i)
            tab_layout.addWidget(btn)
            
        tab_layout.addStretch() # 將標籤靠左對齊
        self.btn_tab_tcp.setChecked(True)
        
        # 加上一條細細的分隔線，讓版面更像專業的分頁
        line = QWidget()
        line.setFixedHeight(1)
        line.setStyleSheet("background-color: #333;")
        
        sec_params.content_layout.addLayout(tab_layout)
        sec_params.content_layout.addWidget(line)
        
        self.param_stacked = QStackedWidget()
        
        # ==========================================
        # Page 1: TCP 姿態
        # ==========================================
        page_tcp = QWidget()
        layout_tcp = QVBoxLayout(page_tcp)
        layout_tcp.setContentsMargins(5, 10, 5, 0)
        layout_tcp.setSpacing(8)
        
        align_layout = QHBoxLayout()
        align_layout.addWidget(QLabel("姿態計算:"))
        self.cb_align_mode = QComboBox()
        self.cb_align_mode.addItems(["最小扭轉 (雷射/塗膠)", "切線對齊 (銲接/指向)"])
        self.cb_align_mode.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        align_layout.addWidget(self.cb_align_mode)
        layout_tcp.addLayout(align_layout)
        
        safe_z_layout = QHBoxLayout()
        safe_z_layout.addWidget(QLabel("安全抬刀 (mm):"))
        self.spin_safe_z = QDoubleSpinBox()
        self.spin_safe_z.setRange(0.0, 200.0)
        self.spin_safe_z.setValue(20.0) 
        self.spin_safe_z.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        safe_z_layout.addWidget(self.spin_safe_z)
        layout_tcp.addLayout(safe_z_layout)
        
        grid_offset = QGridLayout()
        grid_offset.setSpacing(5) 
        self.offset_spins = []
        labels_offset = ["Rx", "Ry", "Rz"]
        for i in range(3):
            lbl = QLabel(labels_offset[i])
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            spin = QDoubleSpinBox()
            spin.setRange(-180.0, 180.0)
            spin.setSingleStep(5.0) 
            spin.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.offset_spins.append(spin)
            grid_offset.addWidget(lbl, i, 0)
            grid_offset.addWidget(spin, i, 1)
        layout_tcp.addLayout(grid_offset)
        layout_tcp.addStretch()
        self.param_stacked.addWidget(page_tcp)

        # ==========================================
        # Page 2: 幾何路徑
        # ==========================================
        page_geo = QWidget()
        layout_geo = QVBoxLayout(page_geo)
        layout_geo.setContentsMargins(5, 10, 5, 0)
        layout_geo.setSpacing(5)
        
        grid_adv = QGridLayout()
        grid_adv.setSpacing(5)
        
        def create_spin(val, min_v, max_v, step, decimals=2):
            s = QDoubleSpinBox()
            s.setRange(min_v, max_v)
            s.setSingleStep(step)
            s.setDecimals(decimals)
            s.setValue(val)
            s.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            return s
            
        self.spin_chordal = create_spin(0.05, 0.001, 5.0, 0.01, 3)
        self.spin_max_step = create_spin(5.0, 0.1, 50.0, 0.5)
        self.spin_min_step = create_spin(0.0, 0.0, 10.0, 0.1)
        self.spin_lead_dist = create_spin(2.0, 0.0, 50.0, 0.5)
        self.spin_lead_angle = create_spin(45.0, -360.0, 360.0, 5.0)
        self.spin_overcut = create_spin(2.0, 0.0, 50.0, 0.5)
        
        adv_params = [
            ("弦向誤差:", self.spin_chordal),
            ("最大點距:", self.spin_max_step),
            ("微段濾波:", self.spin_min_step),
            ("引入距離:", self.spin_lead_dist),
            ("引入角度:", self.spin_lead_angle),
            ("過切距離:", self.spin_overcut),
        ]
        
        for i, (label_text, spin_widget) in enumerate(adv_params):
            lbl = QLabel(label_text)
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid_adv.addWidget(lbl, i, 0)
            grid_adv.addWidget(spin_widget, i, 1)
            
        layout_geo.addLayout(grid_adv)
        layout_geo.addStretch()
        self.param_stacked.addWidget(page_geo)

        # ==========================================
        # Page 3: 速度動態
        # ==========================================
        page_spd = QWidget()
        layout_spd = QVBoxLayout(page_spd)
        layout_spd.setContentsMargins(5, 10, 5, 0)
        layout_spd.setSpacing(5)
        
        grid_spd = QGridLayout()
        grid_spd.setSpacing(5)
        
        def create_spin_int(val):
            s = QSpinBox()
            s.setRange(1, 100)
            s.setValue(val)
            s.setSuffix(" %")
            s.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            return s
            
        self.spin_cut_speed = create_spin_int(10)
        self.spin_cut_accel = create_spin_int(100)
        self.spin_move_speed = create_spin_int(100)
        self.spin_move_accel = create_spin_int(100)
        
        spd_params = [
            ("切削速度:", self.spin_cut_speed),
            ("切削加速:", self.spin_cut_accel),
            ("空駛速度:", self.spin_move_speed),
            ("空駛加速:", self.spin_move_accel),
        ]
        
        for i, (label_text, spin_widget) in enumerate(spd_params):
            lbl = QLabel(label_text)
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid_spd.addWidget(lbl, i, 0)
            grid_spd.addWidget(spin_widget, i, 1)
        
        layout_spd.addLayout(grid_spd)
        layout_spd.addStretch()
        self.param_stacked.addWidget(page_spd)

        # 將 StackedWidget 加入面板，並連接翻頁訊號
        sec_params.content_layout.addWidget(self.param_stacked)
        self.param_tab_group.idClicked.connect(self.param_stacked.setCurrentIndex)
        
        scroll_layout.addWidget(sec_params)

        # ==========================================
        # 路徑管理區
        # ==========================================
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
        self.list_paths.setMinimumHeight(250) 
        self.list_paths.setStyleSheet(styles.STYLE_LIST_WIDGET)
        self.list_paths.setSelectionMode(QListWidget.ExtendedSelection) 
        
        sec_path.content_layout.addLayout(path_btn_layout)
        sec_path.content_layout.addWidget(self.list_paths)
        scroll_layout.addWidget(sec_path)
        
        scroll_layout.addStretch() 
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)

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

        sec_export = CollapsibleSection("機器腳本匯出 (Export)")
        self.btn_save_script = QPushButton("💾 另存機器腳本 (匯出 JSON)")
        self.btn_save_script.setStyleSheet(styles.STYLE_BTN_SUCCESS)
        sec_export.content_layout.addWidget(self.btn_save_script)
        scroll_layout.addWidget(sec_export)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)