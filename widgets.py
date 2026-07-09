# widgets.py
import ctypes
from ctypes import wintypes
import datetime
import os

from PySide6.QtWidgets import (QDoubleSpinBox, QGridLayout, QSizePolicy, QSpacerItem, QWidget, QVBoxLayout, QHBoxLayout, 
                               QFrame, QLabel, QPushButton, QSlider, QTextEdit, QLineEdit, QApplication, QMenu, QMessageBox)
from PySide6.QtCore import QTimer, Qt, QObject, QEvent, QSize, Signal, QRunnable, QThreadPool
from PySide6.QtGui import QAction, QCursor, QDoubleValidator, QImage, QResizeEvent, QPainter, QPen, QColor
import qtawesome as qta

import styles
import config
from config import Robot3DView

# ==========================================
# UI 核心工具箱
# ==========================================
def apply_windows_dark_titlebar(window):
    """將傳入的視窗 (window) 標題列強制轉換為沉浸式深色"""
    try:
        hwnd = wintypes.HWND(int(window.winId()))
        set_window_attribute = ctypes.windll.dwmapi.DwmSetWindowAttribute

        DWMWA_USE_IMMERSIVE_DARK_MODE = 20
        dark_mode_on = ctypes.c_int(1)
        set_window_attribute(hwnd, DWMWA_USE_IMMERSIVE_DARK_MODE, ctypes.byref(dark_mode_on), ctypes.sizeof(dark_mode_on))

        DWMWA_CAPTION_COLOR = 35
        bg_color = ctypes.c_int(0x002B2B2B)
        set_window_attribute(hwnd, DWMWA_CAPTION_COLOR, ctypes.byref(bg_color), ctypes.sizeof(bg_color))

    except Exception as e:
        print(f"Windows 標題列修改失敗: {e}")

# ==========================================
# 全域設定大腦 (Settings Manager)
# ==========================================
class SettingsManager(QObject):
    setting_changed = Signal(str, object)

    def __init__(self):
        super().__init__()
        self._settings = {
            "sync_sliders": True,       
            "lock_splitters": False,    
            "show_comments": True,       
            "default_list_mode": False,  
            "theme_style": "dark"
        }

    def get(self, key):
        return self._settings.get(key)

    def set(self, key, value):
        if self._settings.get(key) != value:
            self._settings[key] = value
            self.setting_changed.emit(key, value)

app_settings = SettingsManager()

# ==========================================
# 工具類元件 (Event Filters)
# ==========================================
class SplitterDoubleClickListener(QObject):
    def __init__(self, splitter, default_sizes):
        super().__init__(splitter)
        self.splitter = splitter
        self.default_sizes = default_sizes

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.MouseButtonDblClick:
            self.splitter.setSizes(self.default_sizes)
            return True
        return False

class GlobalClickFilter(QObject):
    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.MouseButtonPress:
            fw = QApplication.focusWidget()
            if hasattr(fw, "commit_value") and not fw.isReadOnly():
                pos = event.globalPosition().toPoint()
                if not fw.rect().contains(fw.mapFromGlobal(pos)):
                    fw.commit_value() 
                    fw.clearFocus()   
        return False
    
class SplitButtonHoverFilter(QObject):
    def __init__(self, left_btn, right_btn):
        super().__init__(left_btn)
        self.left_btn = left_btn
        self.right_btn = right_btn

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.Enter:
            target = self.right_btn if obj == self.left_btn else self.left_btn
            target.setProperty("dim", True)
            target.style().unpolish(target)
            target.style().polish(target)
            
        elif event.type() == QEvent.Type.Leave:
            target = self.right_btn if obj == self.left_btn else self.left_btn
            target.setProperty("dim", False)
            target.style().unpolish(target)
            target.style().polish(target)
            
        return False

# ==========================================
# 基礎 & 微型 UI 元件
# ==========================================
class EditableValueLabel(QLineEdit):
    def __init__(self, default_text="0.00", parent=None):
        super().__init__(default_text, parent)
        self.slider = None
        self.setFixedHeight(12) 
        self.setFixedWidth(50) 
        self.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        self.set_label_mode()
        self.editingFinished.connect(self.commit_value)

    def set_label_mode(self):
        self.setReadOnly(True)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setStyleSheet(styles.EDITABLE_LABEL_MODE_STYLE)
        self.clearFocus()

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        if event.button() == Qt.MouseButton.LeftButton and self.isReadOnly():
            self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
            self.setReadOnly(False)
            self.setStyleSheet(styles.EDITABLE_EDITOR_MODE_STYLE)
            self.setFocus()
            QTimer.singleShot(0, self.selectAll)

    def commit_value(self):
        if self.isReadOnly(): return
        try:
            val = float(self.text())
            if self.slider:
                self.slider.setValue(int(val * 100.0))
        except ValueError:
            pass 
        self.set_label_mode()

    def focusOutEvent(self, event):
        super().focusOutEvent(event)
        self.commit_value()

class GripperSlider(QSlider):
    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        pen = QPen(QColor("#808080")) 
        pen.setWidth(1)
        painter.setPen(pen)
        
        margin = 7 
        track_width = self.width() - 2 * margin
        y_bottom = self.height() - 2
        y_long_top = y_bottom - 5
        y_short_top = y_bottom - 3
        
        steps = 20
        for i in range(steps + 1):
            x = margin + int(i * (track_width / steps))
            if i % 2 == 0:
                painter.drawLine(x, y_long_top, x, y_bottom)
            else:
                painter.drawLine(x, y_short_top, x, y_bottom)
        painter.end()

class FloatingNavBar(QFrame):
    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.setObjectName("FloatingNavBar")
        self.setStyleSheet(styles.NAVBAR_STYLE)
        self.setFixedHeight(32) 
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 0, 8, 0)
        layout.setSpacing(8)
        
        self.nav_buttons = []
        
        for item in config:
            icon_name = item.get('icon')
            toggle_icon = item.get('toggle_icon')
            icon_color = item.get('color', '#e0e0e0')        
            toggle_color = item.get('toggle_color', '#00e6b8') 
            split_chevron = item.get('split_chevron', False) 
            
            btn = QPushButton(self) 
            btn.setIcon(qta.icon(icon_name, color=icon_color)) 
            btn.setIconSize(QSize(20, 20))
            btn.setFixedSize(26, 26)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            
            if toggle_icon:
                btn.setCheckable(True)
                btn.toggled.connect(lambda checked, b=btn, i1=icon_name, i2=toggle_icon, c1=icon_color, c2=toggle_color: 
                                    b.setIcon(qta.icon(i2 if checked else i1, color=c2 if checked else c1)))
            
            if split_chevron:
                container = QFrame(self)
                container.setStyleSheet("background: transparent; border: none;")
                
                h_layout = QHBoxLayout(container)
                h_layout.setContentsMargins(0, 0, 0, 0)
                h_layout.setSpacing(0) 
                
                btn.setStyleSheet(styles.BTN_SPLIT_LEFT_STYLE)
                h_layout.addWidget(btn)
                
                drop_btn = QPushButton(container)
                drop_btn.setIcon(qta.icon('mdi.chevron-up', color='#e0e0e0'))
                drop_btn.setIconSize(QSize(16, 16))
                drop_btn.setFixedSize(14, 24) 
                drop_btn.setCursor(Qt.CursorShape.PointingHandCursor)
                drop_btn.setStyleSheet(styles.BTN_SPLIT_RIGHT_STYLE)
                h_layout.addWidget(drop_btn)
                
                hover_filter = SplitButtonHoverFilter(btn, drop_btn)
                btn.installEventFilter(hover_filter)
                drop_btn.installEventFilter(hover_filter)
                container.hover_filter = hover_filter 
                
                layout.addWidget(container)
                self.nav_buttons.append(btn)
                self.nav_buttons.append(drop_btn)
            else:
                btn.setStyleSheet(styles.BTN_NAV_GHOST_STYLE)
                layout.addWidget(btn)
                self.nav_buttons.append(btn)

# ==========================================
# 共用 UI 元件 (DRY 重構新增)
# ==========================================
class SpeedControlWidget(QFrame):
    """獨立的速度檔位控制器 (將加減速按鈕與燈號封裝成單一元件)"""
    def __init__(self, parent=None, initial_level=2, max_level=4):
        super().__init__(parent)
        self.level = initial_level
        self.max_level = max_level
        
        self.setFixedHeight(22)
        self.setStyleSheet(styles.SPEED_CAPSULE_STYLE)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self.btn_minus = QPushButton("−")
        self.btn_minus.setFixedSize(30, 22)
        self.btn_minus.setStyleSheet(styles.SPEED_MINUS_STYLE)
        self.btn_minus.clicked.connect(self.decrease)
        layout.addWidget(self.btn_minus)

        seg_container = QWidget()
        seg_layout = QHBoxLayout(seg_container)
        seg_layout.setContentsMargins(0, 0, 0, 0)
        seg_layout.setSpacing(2)
        self.segments = []
        for _ in range(self.max_level):
            seg = QFrame()
            seg.setFixedSize(10, 4) 
            seg_layout.addWidget(seg)
            self.segments.append(seg)
        layout.addWidget(seg_container)

        self.btn_plus = QPushButton("+")
        self.btn_plus.setFixedSize(30, 22)
        self.btn_plus.setStyleSheet(styles.SPEED_PLUS_STYLE)
        self.btn_plus.clicked.connect(self.increase)
        layout.addWidget(self.btn_plus)

        self.update_display()

    def decrease(self):
        if self.level > 1:
            self.level -= 1
            self.update_display()

    def increase(self):
        if self.level < self.max_level:
            self.level += 1
            self.update_display()

    def update_display(self):
        for i, seg in enumerate(self.segments):
            seg.setStyleSheet(styles.SPEED_SEG_ON_STYLE if i < self.level else styles.SPEED_SEG_OFF_STYLE)

# ==========================================
# 大型面板區塊 (Blocks)
# ==========================================
class BaseBlock(QFrame):
    def __init__(self, parent=None, nav_config=None):
        super().__init__(parent)
        self.setObjectName("BlockFrame")
        self.setStyleSheet(styles.BLOCK_STYLE)
        
        if nav_config is None:
            nav_config = [{'icon': 'mdi.dots-horizontal'}]
            
        self.nav_bar = FloatingNavBar(nav_config, self)
        
    def resizeEvent(self, event: QResizeEvent):
        super().resizeEvent(event)
        nav_w = self.nav_bar.sizeHint().width()
        nav_h = self.nav_bar.height()
        self.nav_bar.setGeometry((self.width() - nav_w) // 2, self.height() - nav_h - 10, nav_w, nav_h)
        self.nav_bar.raise_()

class CustomTopBar(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("TopBarFrame")
        self.setFixedHeight(36)
        self.setStyleSheet(styles.TOPBAR_STYLE)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(12) 

        self.btn_menu = self._create_btn('mdi.menu')
        self.btn_menu.clicked.connect(self.show_preferences)
        layout.addWidget(self.btn_menu)

        title_lbl = QLabel("Parol Stream")
        title_lbl.setFont(styles.FONT_TITLE)
        layout.addWidget(title_lbl)
        layout.addStretch(1)

        self.btn_play = self._create_btn('mdi.motion-play-outline')
        self.btn_play.setCheckable(True)
        self.btn_play.toggled.connect(lambda checked: 
            self.btn_play.setIcon(qta.icon(
                'mdi.pause-circle-outline' if checked else 'mdi.motion-play-outline', 
                color='#e6a800' if checked else '#00e6b8'
            ))
        )
        self.btn_play.setIcon(qta.icon('mdi.motion-play-outline', color='#00e6b8'))
        layout.addWidget(self.btn_play)

        self.btn_stop = self._create_btn('mdi.stop-circle-outline') 
        layout.addWidget(self.btn_stop)
        
        self.btn_home = self._create_btn('mdi.home-search')
        self.btn_home.clicked.connect(self.show_homing_warning)
        layout.addWidget(self.btn_home)

        self.btn_soft_home = self._create_btn('mdi.home-outline')
        self.btn_soft_home.setToolTip("Soft Home (Reset all axes to 0)")
        layout.addWidget(self.btn_soft_home)

        layout.addWidget(self._create_separator())
        self.btn_connect = self._create_btn('mdi.connection')
        self.btn_connect.setToolTip("Connect to Serial Port")
        layout.addWidget(self.btn_connect)

        layout.addWidget(self._create_separator())
        self.btn_tools = self._create_btn('mdi.tools') 
        layout.addWidget(self.btn_tools)

        self.btn_base = self._create_btn('mdi.view-grid-outline')
        self.btn_base.setToolTip("Base Frame Manager")
        layout.addWidget(self.btn_base)

    def _create_btn(self, icon_name):
        btn = QPushButton()
        btn.setIcon(qta.icon(icon_name, color='#e0e0e0'))
        btn.setIconSize(QSize(22, 22))
        btn.setFixedSize(28, 28)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        return btn

    def _create_separator(self):
        line = QFrame()
        line.setFixedSize(1, 18)
        line.setStyleSheet("background-color: #555555;") 
        return line

    def show_preferences(self):
        try:
            from preferences import PreferencesDialog
            dialog = PreferencesDialog(self)
            dialog.exec()
        except ImportError:
            pass

    def show_homing_warning(self):        
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("Homing Warning")
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setText(
            "WARNING! Homing sequence is about to begin.\n\n"
            "The robotic arm will perform physical calibration movements.\n"
            "Please ensure the arm's pose and workspace are safe."
        )
        
        msg_box.setStyleSheet(styles.DARK_MESSAGE_BOX_STYLE)
        msg_box.setStandardButtons(QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel)
        msg_box.setDefaultButton(QMessageBox.StandardButton.Cancel)
        
        ok_btn = msg_box.button(QMessageBox.StandardButton.Ok)
        ok_btn.setText("Proceed")
        ok_btn.setStyleSheet("""
            QPushButton {background-color: #b37700; border: 1px solid #d99000; color: #ffffff;}
            QPushButton:hover {background-color: #cc8800;}
        """)
        
        if msg_box.exec() == QMessageBox.StandardButton.Ok:
            main_win = self.window()
            homing_cmd = "<999999,999999,999999,999999,999999,999999,1.0,0>\n"
            
            if hasattr(main_win, 'serial_manager') and main_win.serial_manager.is_connected:
                main_win.serial_manager.send_command(homing_cmd)
                if hasattr(main_win, 'log_widget'):
                    main_win.log_widget.append_log("[System] Homing sequence initiated...")
            else:
                if hasattr(main_win, 'log_widget'):
                    main_win.log_widget.append_log("[ERROR] Homing failed: Controller not connected.")

class MonitorLineEdit(QLineEdit):
    def __init__(self, contents="0.00", parent=None):
        super().__init__(contents, parent)
        self.setReadOnly(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.is_locked = False 

    def mousePressEvent(self, event):
        if self.is_locked:
            event.ignore()
            return
            
        super().mousePressEvent(event)
        if event.button() == Qt.MouseButton.LeftButton and self.isReadOnly():
            self.setReadOnly(False)
            QTimer.singleShot(0, self.selectAll)

    def focusOutEvent(self, event):
        self.setReadOnly(True)
        super().focusOutEvent(event)

class MonitorWidget(QFrame):
    tcp_edit_requested = Signal(str, float)    
    joint_edit_requested = Signal(int, float)  

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(44) 
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(2, 2, 2, 2)
        main_layout.setSpacing(4) 
        
        self.inputs = {} 
        self.validator = QDoubleValidator(-5000.0, 5000.0, 2, self)
        self.validator.setNotation(QDoubleValidator.Notation.StandardNotation)
        
        row1_layout = QHBoxLayout()
        row1_layout.setContentsMargins(0, 0, 0, 0)
        row1_layout.setSpacing(2)
        for name in ["X", "Y", "Z", "Rx", "Ry", "Rz"]:
            row1_layout.addWidget(self._create_box(name, group="TCP"))
        row1_layout.addStretch(1)
        self.btn_copy_tcp = self._create_copy_btn(self.copy_tcp_to_clipboard)
        row1_layout.addWidget(self.btn_copy_tcp)
        main_layout.addLayout(row1_layout)

        row2_layout = QHBoxLayout()
        row2_layout.setContentsMargins(0, 0, 0, 0)
        row2_layout.setSpacing(2)
        for i, name in enumerate(["θ1", "θ2", "θ3", "θ4", "θ5", "θ6"]):
            row2_layout.addWidget(self._create_box(name, group="JOINT", index=i))
        row2_layout.addStretch(1)
        self.btn_copy_joints = self._create_copy_btn(self.copy_joints_to_clipboard)
        row2_layout.addWidget(self.btn_copy_joints)
        main_layout.addLayout(row2_layout)

    def _create_box(self, name, group, index=None):
        frame = QFrame()
        frame.setFixedSize(68, 20) 
        frame.setStyleSheet(styles.MONITOR_BOX_STYLE)
        hbox = QHBoxLayout(frame)
        hbox.setContentsMargins(3, 0, 3, 0)
        hbox.setSpacing(0)
        
        title = QLabel(name)
        title.setStyleSheet(styles.MONITOR_TITLE_STYLE)
        title.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        hbox.addWidget(title)
        hbox.addStretch(1)
        
        value_input = MonitorLineEdit("0.00") 
        value_input.setValidator(self.validator)
        value_input.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        value_input.setStyleSheet("""
            QLineEdit { background: transparent; border: none; color: #00e6b8; font-family: 'Consolas', monospace; font-size: 11px; padding: 0px; }
            QLineEdit:focus { color: #ffffff; background-color: rgba(255, 255, 255, 0.12); border-radius: 2px; }
        """)
        
        if group == "TCP":
            value_input.editingFinished.connect(lambda n=name, inp=value_input: self._on_tcp_edited(n, inp))
        else:
            value_input.editingFinished.connect(lambda i=index, inp=value_input: self._on_joint_edited(i, inp))
            
        hbox.addWidget(value_input)
        self.inputs[name] = value_input 
        return frame

    def set_locked(self, locked):
        for inp in self.inputs.values():
            inp.is_locked = locked
            if locked:
                inp.clearFocus()
                inp.setReadOnly(True)
                inp.setFocusPolicy(Qt.FocusPolicy.NoFocus)
                inp.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
            else:
                inp.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
                inp.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
    
    def _on_tcp_edited(self, name, line_edit):
        text = line_edit.text()
        if text:
            try:
                val = float(text)
                self.tcp_edit_requested.emit(name, val)
            except ValueError: pass
        line_edit.clearFocus() 

    def _on_joint_edited(self, index, line_edit):
        text = line_edit.text()
        if text:
            try:
                val = float(text)
                self.joint_edit_requested.emit(index, val)
            except ValueError: pass
        line_edit.clearFocus() 

    def _create_copy_btn(self, callback):
        btn = QPushButton()
        btn.setIcon(qta.icon('mdi.content-copy', color='#a0a0a0'))
        btn.setIconSize(QSize(12, 14))
        btn.setFixedSize(20, 20)
        btn.setCursor(Qt.CursorShape.PointingHandCursor) 
        btn.setStyleSheet(styles.BTN_GHOST_COPY_STYLE)
        btn.clicked.connect(callback)
        return btn

    def update_tcp(self, x, y, z, rx, ry, rz):
        for name, val in zip(["X", "Y", "Z", "Rx", "Ry", "Rz"], [x, y, z, rx, ry, rz]):
            if not self.inputs[name].hasFocus():
                self.inputs[name].setText(f"{val:.2f}")
        self.current_tcp_str = f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f} Rx:{rx:.2f} Ry:{ry:.2f} Rz:{rz:.2f}"

    def update_joints(self, *joints):
        for name, val in zip(["θ1", "θ2", "θ3", "θ4", "θ5", "θ6"], joints):
            if not self.inputs[name].hasFocus():
                self.inputs[name].setText(f"{val:.2f}")
        self.current_joints_str = " ".join([f"J{i+1}:{j:.2f}" for i, j in enumerate(joints)])

    def copy_tcp_to_clipboard(self):
        if hasattr(self, 'current_tcp_str'): QApplication.clipboard().setText(self.current_tcp_str)

    def copy_joints_to_clipboard(self):
        if hasattr(self, 'current_joints_str'): QApplication.clipboard().setText(self.current_joints_str)

class JogWidget(BaseBlock):
    def __init__(self, parent=None):
        nav_config = [
            {'icon': 'mdi.tune-variant', 'color': '#e6a800', 'toggle_icon': 'mdi.camera-control', 'toggle_color': '#00e6b8'},
            {'icon': 'mdi.lock-outline'},
            {'icon': 'mdi.refresh'},
            {'icon': 'mdi.dots-vertical'}
        ]
        super().__init__(parent=parent, nav_config=nav_config)
        self.setMinimumWidth(260)
        self.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        main_layout = QVBoxLayout(self)
        main_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        def create_separator():
            sep_layout = QHBoxLayout()
            sep_layout.setContentsMargins(15, 5, 15, 5) 
            line = QFrame()
            line.setFrameShape(QFrame.Shape.HLine)
            line.setStyleSheet(styles.SEPARATOR_STYLE) 
            sep_layout.addWidget(line)
            return sep_layout

        # ==========================================
        # Joint 區
        # ==========================================
        joint_title_row = QHBoxLayout()
        joint_title_row.setContentsMargins(0, 0, 10, 5)
        title_joint = QLabel("Joint Jogging")
        title_joint.setFont(styles.FONT_TITLE)
        joint_title_row.addWidget(title_joint)
        joint_title_row.addStretch(1)

        # 使用 DRY 重構後的速度控制器
        self.j_speed_ctrl = SpeedControlWidget()
        joint_title_row.addWidget(self.j_speed_ctrl)
        joint_title_row.addSpacing(10)

        btn_add_joint = QPushButton("+")
        btn_add_joint.setFixedSize(22, 22)
        btn_add_joint.setStyleSheet(styles.BTN_ACTION_STYLE)
        joint_title_row.addWidget(btn_add_joint)
        main_layout.addLayout(joint_title_row)

        upper_layout = QHBoxLayout()
        upper_layout.setContentsMargins(10, 0, 10, 0)
        jog_area = QVBoxLayout()
        jog_area.setAlignment(Qt.AlignmentFlag.AlignTop)
        jog_area.setSpacing(5)
        
        self.joint_sliders = []
        self.joint_labels = [] 
        for i in range(1, 7):
            joint_container = QVBoxLayout()
            joint_container.setSpacing(0)
            
            label_row = QHBoxLayout()
            label_row.setContentsMargins(0, 0, 0, 0)
            lbl = QLabel(f"Joint {i}")
            lbl.setObjectName("JointLabel") 
            lbl.setFixedHeight(12)
            lbl.setAlignment(Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignLeft)
            label_row.addWidget(lbl, 1)
            
            val_lbl = EditableValueLabel("0.0") 
            self.joint_labels.append(val_lbl)
            label_row.addWidget(val_lbl, 1)            
            label_row.addStretch(1)
            joint_container.addLayout(label_row)

            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(10) 
            
            slider = QSlider(Qt.Orientation.Horizontal)
            min_lim, max_lim = config.JOINT_LIMITS[i-1]
            slider.setRange(int(min_lim * 100), int(max_lim * 100))
            slider.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            slider.setStyleSheet(styles.SLIDER_JOINT_STYLE)
            self.joint_sliders.append(slider)
            val_lbl.slider = slider
            slider.valueChanged.connect(self.on_joint_slider_changed)
            slider.valueChanged.connect(lambda val, label=val_lbl: label.setText(f"{val/100:.1f}"))

            # 使用 DRY 共用按鈕創建函式
            btn_minus = self._create_jog_btn('mdi.transfer-left', lambda *args, idx=i-1: self._step_joint(idx, -1))
            row.addWidget(btn_minus)
            row.addWidget(slider)

            btn_plus = self._create_jog_btn('mdi.transfer-right', lambda *args, idx=i-1: self._step_joint(idx, 1))
            row.addWidget(btn_plus)

            row.insertSpacerItem(1, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))
            row.insertSpacerItem(3, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))

            joint_container.addLayout(row)
            jog_area.addLayout(joint_container)

        upper_layout.addLayout(jog_area)
        main_layout.addLayout(upper_layout)
        main_layout.addLayout(create_separator())

        # ==========================================
        # Cartesian 區
        # ==========================================
        cart_title_row = QHBoxLayout()
        cart_title_row.setContentsMargins(0, 0, 10, 5)
        title_cart = QLabel("Cartesian Jogging")
        title_cart.setFont(styles.FONT_TITLE)
        cart_title_row.addWidget(title_cart)
        cart_title_row.addStretch(1)

        self.is_cartesian_continuous = True
        
        # 使用 DRY 重構後的速度控制器
        self.c_speed_ctrl = SpeedControlWidget()
        cart_title_row.addWidget(self.c_speed_ctrl)
        main_layout.addLayout(cart_title_row)

        cart_main_layout = QHBoxLayout()
        cart_main_layout.setContentsMargins(0, 0, 10, 5)
        cart_main_layout.setSpacing(10) 

        cart_ctrl_tower = QVBoxLayout()
        cart_ctrl_tower.setSpacing(6)
        cart_ctrl_tower.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.btn_frame_toggle = QPushButton("WRF")
        self.btn_frame_toggle.setCheckable(True) 
        self.btn_frame_toggle.setFixedSize(45, 32) 
        self.btn_frame_toggle.setStyleSheet(styles.BTN_FRAME_TOGGLE_STYLE)
        self.btn_frame_toggle.toggled.connect(self.toggle_cartesian_frame)
        cart_ctrl_tower.addWidget(self.btn_frame_toggle)

        self.btn_jog_mode = QPushButton("Cont")
        self.btn_jog_mode.setCheckable(True)
        self.btn_jog_mode.setFixedSize(45, 32) 
        self.btn_jog_mode.setStyleSheet(styles.BTN_STEP_TOGGLE_STYLE)
        self.btn_jog_mode.toggled.connect(self.toggle_cartesian_mode)
        cart_ctrl_tower.addWidget(self.btn_jog_mode)

        class SmartStepSpinBox(QDoubleSpinBox):
            def textFromValue(self, value):
                if value == int(value): return str(int(value))
                return str(value)

        self.step_container = QWidget()
        step_layout = QVBoxLayout(self.step_container)
        step_layout.setContentsMargins(0, 0, 0, 0) 
        step_layout.setSpacing(0)
        
        self.spin_step = SmartStepSpinBox()
        self.spin_step.setFixedSize(45, 32)
        self.spin_step.setRange(0.01, 100.0)
        self.spin_step.setValue(1.0)
        self.spin_step.setDecimals(2)
        self.spin_step.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons) 
        self.spin_step.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.spin_step.setStyleSheet(styles.SPIN_STEP_STYLE)
        
        step_layout.addWidget(self.spin_step)
        self.step_container.setVisible(False)
        cart_ctrl_tower.addWidget(self.step_container)

        cart_main_layout.addLayout(cart_ctrl_tower)

        cart_grid = QGridLayout()
        cart_grid.setSpacing(6)
        
        cartesian_labels = [
            ("X+", "X-", "Rx+", "Rx-"), 
            ("Y+", "Y-", "Ry+", "Ry-"), 
            ("Z+", "Z-", "Rz+", "Rz-")
        ]
        self.cart_buttons = [] 
        for row_idx, row_items in enumerate(cartesian_labels):
            for col_idx, base_label in enumerate(row_items):
                btn = QPushButton(f"W{base_label}") 
                btn.setFixedHeight(32)
                btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
                btn.setStyleSheet(styles.BTN_CARTESIAN_STYLE)
                btn.setAutoRepeat(True)
                btn.setAutoRepeatDelay(250)
                btn.setAutoRepeatInterval(10)
                btn.clicked.connect(lambda checked=False, bl=base_label: self.on_cartesian_clicked(bl))
                cart_grid.addWidget(btn, row_idx, col_idx)
                self.cart_buttons.append((btn, base_label))

        cart_main_layout.addLayout(cart_grid)
        main_layout.addLayout(cart_main_layout)
        main_layout.addLayout(create_separator())

        # ==========================================
        # Gripper 區
        # ==========================================
        ee_title_row = QHBoxLayout()
        ee_title_row.setContentsMargins(0, 0, 10, 0) 
        title_ee = QLabel("End Effector")
        title_ee.setFont(styles.FONT_TITLE)
        ee_title_row.addWidget(title_ee)
        ee_title_row.addStretch(1)

        # 使用 DRY 重構後的速度控制器
        self.ee_speed_ctrl = SpeedControlWidget()
        ee_title_row.addWidget(self.ee_speed_ctrl)
        ee_title_row.addSpacing(10)

        btn_add_gripper = QPushButton("+")
        btn_add_gripper.setFixedSize(22, 22)
        btn_add_gripper.setStyleSheet(styles.BTN_ACTION_STYLE)
        ee_title_row.addWidget(btn_add_gripper)
        main_layout.addLayout(ee_title_row)

        lower_layout = QHBoxLayout()
        lower_layout.setContentsMargins(0, 0, 10, 5)
        
        gripper_nav = QVBoxLayout()
        gripper_nav.setAlignment(Qt.AlignmentFlag.AlignBottom) 
        gripper_nav.setContentsMargins(0, 0, 15, 0) 
        
        self.gripper_btn = QPushButton("Gripper")
        self.gripper_btn.setCheckable(True) 
        self.gripper_btn.setFixedSize(60, 22) 
        self.gripper_btn.setStyleSheet(styles.BTN_GRIPPER_TOGGLE_STYLE)
        gripper_nav.addWidget(self.gripper_btn)
        lower_layout.addLayout(gripper_nav)

        gripper_area = QVBoxLayout()
        gripper_area.setSpacing(0)

        g_label_row = QHBoxLayout()
        g_label_row.setContentsMargins(0, 0, 0, 0)
        g_label_row.addStretch(1)
        g_val_lbl = QLabel("0 %")
        g_label_row.addWidget(g_val_lbl) 
        g_label_row.addStretch(1)
        gripper_area.addLayout(g_label_row)

        g_slider_row = QHBoxLayout()
        g_slider_row.setContentsMargins(0, 0, 0, 0)
        g_slider_row.setSpacing(10)

        # 使用 DRY 共用按鈕創建函式
        g_btn_minus = self._create_jog_btn('mdi.transfer-left', lambda checked=False: self.g_slider.setValue(self.g_slider.value() - 1))
        g_slider_row.addWidget(g_btn_minus)

        self.g_slider = QSlider(Qt.Orientation.Horizontal) 
        self.g_slider.setRange(0, 100)
        self.g_slider.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.g_slider.setStyleSheet(styles.SLIDER_GRIPPER_STYLE)
        g_slider_row.addWidget(self.g_slider)

        g_btn_plus = self._create_jog_btn('mdi.transfer-right', lambda checked=False: self.g_slider.setValue(self.g_slider.value() + 1))
        g_slider_row.addWidget(g_btn_plus)

        g_slider_row.insertSpacerItem(1, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))
        g_slider_row.insertSpacerItem(3, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))

        gripper_area.addLayout(g_slider_row)
        lower_layout.addLayout(gripper_area)
        
        self.g_slider.valueChanged.connect(lambda val, label=g_val_lbl: label.setText(f"{val} %"))
        main_layout.addLayout(lower_layout) 

    # ==========================================
    # 屬性代理 (保證與 gui.py 完美相容)
    # ==========================================
    @property
    def j_speed_level(self):
        return self.j_speed_ctrl.level

    @property
    def c_speed_level(self):
        return self.c_speed_ctrl.level

    @property
    def ee_speed_level(self):
        return self.ee_speed_ctrl.level

    # ==========================================
    # 共用 UI 創建函式
    # ==========================================
    def _create_jog_btn(self, icon_name, callback):
        """將重複的手動點動 (Jog) 按鈕邏輯獨立抽取"""
        btn = QPushButton(qta.icon(icon_name, color='#ffffff'), "")
        btn.setIconSize(QSize(16, 16))
        btn.setFixedSize(22, 22)
        btn.setStyleSheet(styles.BTN_JOG_SLIDER_STYLE)
        btn.setAutoRepeat(True)
        btn.setAutoRepeatDelay(250)
        btn.setAutoRepeatInterval(10)
        btn.clicked.connect(callback)
        return btn

    # ==========================================
    # 連動邏輯區
    # ==========================================
    def on_joint_slider_changed(self):
        angles = [float(s.value()) / 100 for s in self.joint_sliders]
        if hasattr(self, 'update_3d_callback') and self.update_3d_callback:
            self.update_3d_callback(angles)

    def _step_joint(self, index, sign):
        step_angle = sign * 0.25 * (2 ** (self.j_speed_level - 1))
        slider = self.joint_sliders[index]
        step_val = int(step_angle * 100.0) 
        new_val = slider.value() + step_val
        new_val = max(slider.minimum(), min(slider.maximum(), new_val)) 
        slider.setValue(new_val)

    def on_cartesian_clicked(self, base_label):
        if hasattr(self, 'cartesian_jog_callback') and self.cartesian_jog_callback:
            axis_str = base_label[:-1] 
            sign = 1 if base_label[-1] == '+' else -1
            is_rot = len(axis_str) > 1
            axis_arg = axis_str if is_rot else axis_str.lower()
            
            if self.is_cartesian_continuous:
                step_val = sign * 0.5 * (2 ** (self.c_speed_level - 1))
            else:
                step_val = sign * self.spin_step.value()
                
            frame = "Tool" if self.btn_frame_toggle.isChecked() else "World"
            self.cartesian_jog_callback(axis_arg, step_val, frame)

    def update_joints_from_ik(self, float_angles):
        for i, slider in enumerate(self.joint_sliders):
            slider.blockSignals(True)
            slider.setValue(int(round(float_angles[i] * 100)))
            slider.blockSignals(False)
            
            if hasattr(self, 'joint_labels') and i < len(self.joint_labels):
                self.joint_labels[i].setText(f"{float_angles[i]:.1f}")

    def toggle_cartesian_frame(self, checked):
        if checked:
            self.btn_frame_toggle.setText("TRF")
            prefix = "T"
        else:
            self.btn_frame_toggle.setText("WRF")
            prefix = "W"
        for btn, base_label in self.cart_buttons:
            btn.setText(f"{prefix}{base_label}")

    def toggle_cartesian_mode(self, checked):
        if checked:
            self.btn_jog_mode.setText("Step")
            self.is_cartesian_continuous = False
            self.step_container.setVisible(True)  
            
            for btn, _ in self.cart_buttons:
                btn.setAutoRepeat(False)
        else:
            self.btn_jog_mode.setText("Cont")
            self.is_cartesian_continuous = True
            self.step_container.setVisible(False)
            
            for btn, _ in self.cart_buttons:
                btn.setAutoRepeat(True)
                btn.setAutoRepeatDelay(250)
                btn.setAutoRepeatInterval(10)

class _ScreenshotSaveSignals(QObject):
    finished = Signal(str)
    error = Signal(str)

class _ScreenshotSaveTask(QRunnable):
    def __init__(self, image: QImage, filepath: str):
        super().__init__()
        self.image = image 
        self.filepath = filepath
        self.signals = _ScreenshotSaveSignals()

    def run(self):
        try:
            ok = self.image.save(self.filepath, "PNG")
            if ok:
                self.signals.finished.emit(self.filepath)
            else:
                self.signals.error.emit(f"儲存失敗: {self.filepath}")
        except Exception as e:
            self.signals.error.emit(str(e))

# ==========================================
# View3DWidget
# ==========================================
class View3DWidget(BaseBlock):    
    def __init__(self, parent=None):
        nav_config = [
            {'icon': 'mdi.axis-arrow', 'toggle_icon': 'mdi.axis-arrow', 'toggle_color': '#00e6b8'},
            {'icon': 'mdi.rotate-orbit', 'toggle_icon': 'mdi.rotate-orbit', 'toggle_color': '#e6a800'},
            {'icon': 'mdi.camera-outline'}, 
            {'icon': 'mdi.dots-vertical'}
        ]
        super().__init__(parent=parent, nav_config=nav_config)
        self.setMinimumHeight(200) 
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4) 
        layout.setSpacing(0) 
        
        self.monitor_widget = MonitorWidget()
        layout.addWidget(self.monitor_widget)
        
        self.robot_view = Robot3DView()
        layout.addWidget(self.robot_view)
        
        self.robot_view.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.robot_view.customContextMenuRequested.connect(lambda pos: self.show_context_menu())
        
        self.btn_translate = self.nav_bar.nav_buttons[0]
        self.btn_rotate = self.nav_bar.nav_buttons[1]
        self.btn_camera = self.nav_bar.nav_buttons[2] 
        self.btn_camera.clicked.connect(self.on_camera_clicked) 

        self._screenshot_pool = QThreadPool() 
        
        self._active_menu = None
        self._updating_btns = False 
        self.btn_translate.toggled.connect(self.on_translate_toggled)
        self.btn_rotate.toggled.connect(self.on_rotate_toggled)

    def set_base_manager(self, base_manager):
        self.robot_view.set_base_manager(base_manager)

    def _handle_spacebar(self):
        focus_w = QApplication.focusWidget()
        if focus_w:
            if focus_w.inherits("QLineEdit") or focus_w.inherits("QAbstractSpinBox") or focus_w.inherits("QTextEdit"):
                return

        local_pos = self.mapFromGlobal(QCursor.pos())
        if not self.rect().contains(local_pos):
            return 

        if getattr(self, '_active_menu', None) is not None:
            self._active_menu.close()
            self._active_menu = None
            return

        self.show_context_menu(pos=None)

    def on_translate_toggled(self, checked):
        if self._updating_btns: return
        self._updating_btns = True
        if checked:
            self.btn_rotate.setChecked(False) 
            self.robot_view.set_gizmo_mode('translate')
        else:
            if not self.btn_rotate.isChecked(): 
                self.robot_view.set_gizmo_mode('free') 
        self._updating_btns = False

    def on_rotate_toggled(self, checked):
        if self._updating_btns: return
        self._updating_btns = True
        if checked:
            self.btn_translate.setChecked(False) 
            self.robot_view.set_gizmo_mode('rotate')
        else:
            if not self.btn_translate.isChecked():
                self.robot_view.set_gizmo_mode('free') 
        self._updating_btns = False

    def reset_gizmo_buttons(self):
        if self.btn_translate.isChecked() or self.btn_rotate.isChecked():
            self.btn_translate.setChecked(False)
            self.btn_rotate.setChecked(False)

    def on_camera_clicked(self):
        QApplication.processEvents() 
        pixmap = self.grab() 
        image = pixmap.toImage() 

        screenshot_dir = os.path.join(os.getcwd(), "screenshots")
        os.makedirs(screenshot_dir, exist_ok=True)
        
        filename = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3] + ".png"
        filepath = os.path.join(screenshot_dir, filename)

        task = _ScreenshotSaveTask(image, filepath)
        task.signals.error.connect(lambda msg: print(f"[截圖錯誤] {msg}"))
        self._screenshot_pool.start(task)

        self._flash_camera_icon('#00e6b8', duration=80)

    def _flash_camera_icon(self, color: str, duration: int = 80):
        if not hasattr(self, '_camera_btn_original_style'):
            self._camera_btn_original_style = self.btn_camera.styleSheet()

        self.btn_camera.setStyleSheet(
            f"QToolButton {{ background-color: {color} !important; border-radius: 4px; }}"
        )
        QTimer.singleShot(duration, self._restore_camera_icon)

    def _restore_camera_icon(self):
        self.btn_camera.setStyleSheet(self._camera_btn_original_style)

    def show_context_menu(self, pos=None):        
        menu_pos = QCursor.pos()
        
        menu = QMenu(self.window())
        self._active_menu = menu
        menu.setStyleSheet(styles.MENU_STYLE)
        menu.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        
        action_reset = QAction(qta.icon('mdi.camera-retake', color='#e0e0e0'), "Reset Camera", self)
        
        grid_visible = self.robot_view.floor_grid.visible
        action_grid = QAction(qta.icon('mdi.grid', color='#e0e0e0'), "Hide Grid" if grid_visible else "Show Grid", self)
        
        is_drag_active = getattr(self.robot_view, '_show_drag_sphere', False)
        action_toggle_drag = QAction(qta.icon('mdi.cursor-move', color='#e0e0e0'), "Disable Drag Sphere" if is_drag_active else "Enable Drag Sphere", self)
        
        path_visible = getattr(self.robot_view, 'show_trajectory', True)
        action_path = QAction(qta.icon('mdi.vector-polyline', color='#e0e0e0'), "Hide Trajectory" if path_visible else "Show Trajectory", self)
        
        menu.addAction(action_reset)
        menu.addSeparator() 
        menu.addAction(action_grid)
        menu.addAction(action_toggle_drag)
        menu.addAction(action_path) 
        
        try:
            selected_action = menu.exec(menu_pos)
        finally:
            self._active_menu = None
            
        if not selected_action:
            return
        
        if selected_action == action_reset:
            self.robot_view.view.camera.center = (0, 0, 0.15)
            self.robot_view.view.camera.elevation = 30
            self.robot_view.view.camera.azimuth = -225
            self.robot_view.view.camera.distance = 2.5
            
        elif selected_action == action_grid:
            new_state = not grid_visible
            self.robot_view.floor_grid.visible = new_state
            self.robot_view.floor.visible = new_state 
            self.robot_view.canvas.update()
            
        elif selected_action == action_toggle_drag:
            self.robot_view._show_drag_sphere = not is_drag_active
            self.robot_view._update_gizmo_visuals()
            
        elif selected_action == action_path:
            self.robot_view.show_trajectory = not path_visible
            if hasattr(self.robot_view, 'path_actor'):
                has_data = getattr(self.robot_view.path_actor, 'pos', None) is not None and len(self.robot_view.path_actor.pos) > 1
                self.robot_view.path_actor.visible = self.robot_view.show_trajectory and has_data
            self.robot_view.canvas.update()

class LogWidget(BaseBlock):
    def __init__(self, parent=None):
        nav_config = [{'icon': 'mdi.delete-outline'}, {'icon': 'mdi.export'}, {'icon': 'mdi.dots-vertical'}]
        super().__init__(parent=parent, nav_config=nav_config)
        self.setMinimumHeight(0) 
        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 60)
        
        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        try:
            self.log_console.setFont(styles.FONT_LOG)
        except AttributeError:
            pass
            
        self.log_console.setStyleSheet(styles.LOG_CONSOLE_STYLE)
        layout.addWidget(self.log_console)

        self.btn_clear = self.nav_bar.nav_buttons[0]
        self.btn_clear.setToolTip("Clear Log")
        self.btn_clear.clicked.connect(self.log_console.clear)

        self.append_log("[System] Parol Stream OS initialized.")

    def append_log(self, msg):
        color = "#d4d4d4" 

        safe_msg = str(msg).replace("<", "&lt;").replace(">", "&gt;").replace("\n", "<br>")
        msg_upper = safe_msg.upper()

        if "[ERROR]" in msg_upper or "[STOP]" in msg_upper or "錯誤" in msg_upper or "FAILED" in msg_upper:
            color = "#ff4444" 
        elif "[WARNING]" in msg_upper or "警告" in msg_upper or "TIMEOUT" in msg_upper:
            color = "#e6a800" 
        elif "[SYSTEM]" in msg_upper or "系統" in msg_upper:
            color = "#00a8e6" 
        elif "[HW]" in msg_upper or "CONNECTED" in msg_upper or "DISCONNECTED" in msg_upper:
            color = "#c63bbb" 
        elif "RECORDED" in msg_upper or "UPDATED" in msg_upper or "DELETED" in msg_upper or "[CODE]" in msg_upper:
            color = "#00e6b8" 
        elif "&GT;&GT;" in safe_msg: 
            color = "#d7ba7d" 

        html_msg = f'<span style="color: {color};">{safe_msg}</span>'
        self.log_console.append(html_msg)