# widgets.py
import ctypes
from ctypes import wintypes
import datetime
import os
import time

from PySide6.QtWidgets import (QDoubleSpinBox, QGridLayout, QSizePolicy, QSpacerItem, 
                               QWidget, QVBoxLayout, QHBoxLayout, QFrame, QLabel, 
                               QPushButton, QSlider, QTextEdit, QLineEdit, QApplication, 
                               QMenu, QMessageBox)
from PySide6.QtCore import QThread, QTimer, Qt, QObject, QEvent, QSize, Signal, QRunnable, QThreadPool
from PySide6.QtGui import QAction, QCursor, QDoubleValidator, QImage, QResizeEvent, QPainter, QPen, QColor
import qtawesome as qta

import styles
import config
from config import Robot3DView

# =========================================================
# [1] 系統全域工具 (Global Utilities & Settings)
# =========================================================
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
    def get(self, key): return self._settings.get(key)
    def set(self, key, value):
        if self._settings.get(key) != value:
            self._settings[key] = value
            self.setting_changed.emit(key, value)

app_settings = SettingsManager()

# =========================================================
# [2.5] 笛卡爾空間運動引擎 (Pure Math & QThread Worker)
# =========================================================
class CartesianMathEngine:
    """專門負責運算梯形加減速與積分，完全不依賴 UI 元件 (純數學)"""
    def __init__(self):
        self.current_speed = 0.0
        self.step_remaining = 0.0
        self.is_stopping = False
        self.is_continuous = True

    def reset(self, is_continuous, fixed_step=0.0):
        self.current_speed = 0.0
        self.step_remaining = abs(fixed_step)
        self.is_stopping = False
        self.is_continuous = is_continuous

    def stop(self):
        self.is_stopping = True

    def compute_step(self, dt, max_speed, accel, sign):
        step_val = 0.0
        is_finished = False

        if self.is_continuous:
            if not self.is_stopping:
                self.current_speed += accel * dt
                if self.current_speed > max_speed:
                    self.current_speed = max_speed
            else:
                self.current_speed -= accel * dt
                if self.current_speed <= 0:
                    self.current_speed = 0.0
                    is_finished = True
            step_val = sign * self.current_speed * dt
        else:
            if self.step_remaining > 0:
                stop_dist = (self.current_speed ** 2) / (2.0 * accel)
                if self.step_remaining <= stop_dist:
                    self.current_speed -= accel * dt
                    if self.current_speed < 1.0: 
                        self.current_speed = 1.0 
                else:
                    self.current_speed += accel * dt
                    if self.current_speed > max_speed:
                        self.current_speed = max_speed

                move_dist = min(self.current_speed * dt, self.step_remaining)
                self.step_remaining -= move_dist
                step_val = sign * move_dist

                if self.step_remaining <= 0:
                    self.current_speed = 0.0
                    is_finished = True
            else:
                is_finished = True

        return step_val, is_finished

class CartesianJogWorker(QThread):
    """獨立的 100Hz 背景執行緒，專職負責數學計算並發送信號"""
    step_computed_signal = Signal(float, bool) # (算出的單步距離, 是否完全煞停)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.math_engine = CartesianMathEngine()
        self._is_running = True
        self._is_active = False 
        
        self.dt = 0.01
        self.max_speed = 0.0
        self.accel = 0.0
        self.sign = 1

    def run(self):
        import time
        absolute_target_time = time.perf_counter()
        
        while self._is_running:
            if self._is_active:
                # 目標時間往後推 10ms (維持完美的 100Hz 基準)
                absolute_target_time += self.dt
                
                # 委託數學大腦算微積分
                step_val, is_finished = self.math_engine.compute_step(
                    self.dt, self.max_speed, self.accel, self.sign
                )
                
                # 透過信號槽投遞給 UI 執行緒 (絕對安全，不會引發崩潰)
                self.step_computed_signal.emit(step_val, is_finished)
                
                if is_finished:
                    self._is_active = False
                    
                # 智慧等待：如果有餘裕就睡覺，如果落後 (被系統卡頓) 就跳過睡覺補上
                sleep_time = absolute_target_time - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    absolute_target_time = time.perf_counter() 
            else:
                time.sleep(0.01) # 閒置時的低耗能待命
                absolute_target_time = time.perf_counter()

    def start_move(self, is_continuous, fixed_step, max_speed, accel, sign):
        """外部呼叫：設定參數並啟動引擎"""
        self.max_speed = max_speed
        self.accel = accel
        self.sign = sign
        self.math_engine.reset(is_continuous, fixed_step)
        self._is_active = True
        
    def stop_move(self):
        """外部呼叫：啟動煞車減速"""
        self.math_engine.stop()
        
    def stop_thread(self):
        """徹底銷毀執行緒 (關閉程式時使用)"""
        self._is_running = False
        self.wait()

# =========================================================
# [2] 全域事件過濾器 (Event Filters)
# =========================================================
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

# =========================================================
# [3] 原子級 UI 元件 (Atomic UI Components)
# =========================================================
class EditableValueLabel(QLineEdit):
    def __init__(self, default_text="0.00", parent=None):
        super().__init__(default_text, parent)
        self.slider = None
        self.on_commit_cb = None  
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
            if self.slider: self.slider.setValue(int(val * 100.0))
            if self.on_commit_cb: self.on_commit_cb()
        except ValueError:
            pass
        self.set_label_mode()

    def focusOutEvent(self, event):
        super().focusOutEvent(event)
        self.commit_value()

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
        
        steps = 20
        for i in range(steps + 1):
            x = margin + int(i * (track_width / steps))
            if i % 2 == 0: painter.drawLine(x, y_bottom - 5, x, y_bottom)
            else: painter.drawLine(x, y_bottom - 3, x, y_bottom)
        painter.end()

class SpeedControlWidget(QFrame):
    def __init__(self, parent=None, initial_level=4, max_level=4):
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

# =========================================================
# [4] 獨立系統面板 (Standalone Panels)
# =========================================================
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
        self.btn_play.setIcon(qta.icon('mdi.motion-play-outline', color='#00e6b8'))
        self.btn_play.setCheckable(True) 
        self.btn_play.setToolTip("開始 / 繼續 (Play/Resume)")
        layout.addWidget(self.btn_play)

        self.btn_stop = self._create_btn('mdi.stop-circle-outline')
        self.btn_stop.setToolTip("全面停止 (E-Stop)")
        self.btn_stop.setEnabled(False) 
        layout.addWidget(self.btn_stop)

        self.btn_estop_reset = self._create_btn('mdi.lock-open-variant-outline')
        self.btn_estop_reset.setToolTip("解除急停鎖定")
        self.btn_estop_reset.setEnabled(False) 
        layout.addWidget(self.btn_estop_reset)
        
        self.btn_home = self._create_btn('mdi.home-search')
        self.btn_home.clicked.connect(self.show_homing_warning)
        layout.addWidget(self.btn_home)

        self.btn_soft_home = self._create_btn('mdi.home-outline')
        self.btn_soft_home.setToolTip("Soft Home")
        layout.addWidget(self.btn_soft_home)

        layout.addWidget(self._create_separator())

        self.btn_simulation = self._create_btn('mdi.safety-goggles')
        self.btn_simulation.setToolTip("開啟純模擬模式 (Simulation Mode)")
        self.btn_simulation.setCheckable(True) 
        layout.addWidget(self.btn_simulation)

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
        except ImportError: pass

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
        ok_btn.setStyleSheet(styles.WARNING_BTN_STYLE)
        
        if msg_box.exec() == QMessageBox.StandardButton.Ok:
            main_win = self.window()
            
            if main_win.serial_manager.is_connected:
                main_win.serial_manager.send_homing() 
                main_win.handle_system_pose_update([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                main_win.log_widget.append_log("[System] Homing sequence initiated...")
            else:
                main_win.log_widget.append_log("[ERROR] Homing failed: Controller not connected.")

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
                self.tcp_edit_requested.emit(name, float(text))
            except ValueError: pass
        line_edit.clearFocus() 

    def _on_joint_edited(self, index, line_edit):
        text = line_edit.text()
        if text:
            try:
                self.joint_edit_requested.emit(index, float(text))
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

class LogWidget(BaseBlock):
    def __init__(self, parent=None):
        nav_config = [{'icon': 'mdi.delete-outline'}, {'icon': 'mdi.export'}, {'icon': 'mdi.dots-vertical'}]
        super().__init__(parent=parent, nav_config=nav_config)
        self.setMinimumHeight(0) 
        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 60)
        
        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        try: self.log_console.setFont(styles.FONT_LOG)
        except AttributeError: pass
            
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

        if "[ERROR]" in msg_upper or "[STOP]" in msg_upper or "錯誤" in msg_upper or "FAILED" in msg_upper: color = "#ff4444" 
        elif "[WARNING]" in msg_upper or "警告" in msg_upper or "TIMEOUT" in msg_upper: color = "#e6a800" 
        elif "[SYSTEM]" in msg_upper or "系統" in msg_upper: color = "#00a8e6" 
        elif "[HW]" in msg_upper or "CONNECTED" in msg_upper or "DISCONNECTED" in msg_upper: color = "#c63bbb" 
        elif "RECORDED" in msg_upper or "UPDATED" in msg_upper or "DELETED" in msg_upper or "[CODE]" in msg_upper: color = "#00e6b8" 
        elif "&GT;&GT;" in safe_msg: color = "#d7ba7d" 

        html_msg = f'<span style="color: {color};">{safe_msg}</span>'
        self.log_console.append(html_msg)

# =========================================================
# [5] 3D 預覽面板 (3D View Widget & Screenshot Task)
# =========================================================
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
            if ok: self.signals.finished.emit(self.filepath)
            else: self.signals.error.emit(f"儲存失敗: {self.filepath}")
        except Exception as e:
            self.signals.error.emit(str(e))

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
            if focus_w.inherits("QLineEdit") or focus_w.inherits("QAbstractSpinBox") or focus_w.inherits("QTextEdit"): return
        
        local_pos = self.mapFromGlobal(QCursor.pos())
        if not self.rect().contains(local_pos): return 
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
            if not self.btn_rotate.isChecked(): self.robot_view.set_gizmo_mode('free') 
        self._updating_btns = False

    def on_rotate_toggled(self, checked):
        if self._updating_btns: return
        self._updating_btns = True
        if checked:
            self.btn_translate.setChecked(False) 
            self.robot_view.set_gizmo_mode('rotate')
        else:
            if not self.btn_translate.isChecked(): self.robot_view.set_gizmo_mode('free') 
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
        self.btn_camera.setStyleSheet(f"QToolButton {{ background-color: {color} !important; border-radius: 4px; }}")
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
        
        try: selected_action = menu.exec(menu_pos)
        finally: self._active_menu = None
            
        if not selected_action: return
        
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

# =========================================================
# [6] 操作面板 (JogWidget)
# =========================================================
class JogWidget(BaseBlock):
    # --- 初始化與時鐘 ---
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
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self._init_timers()
        self._setup_joint_ui()
        self._setup_cartesian_ui()
        self._setup_gripper_ui()

    def _init_timers(self):
        # 1. 關節時鐘
        self.joint_query_timer = QTimer(self)
        self.joint_query_timer.timeout.connect(self._poll_real_pose)
        
        self.joint_sim_timer = QTimer(self)
        self.joint_sim_timer.timeout.connect(self._update_joint_simulation)
        
        self.active_joint_axis = -1
        self.active_joint_sign = 0
        
        self.joint_hold_timer = QTimer(self)
        self.joint_hold_timer.setSingleShot(True)
        self.joint_hold_timer.timeout.connect(self._on_joint_hold_timeout)
        self._joint_press_axis = -1
        self._joint_press_dir = 0

        # ==========================================
        # 2. 升級：Cartesian 專屬背景執行緒引擎
        # ==========================================
        self.cart_worker = CartesianJogWorker(self)
        self.cart_worker.step_computed_signal.connect(self._on_step_computed)
        self.cart_worker.start() # 啟動背景待命
        
        # 關閉程式時，優雅地結束背景執行緒，避免跳紅字警告
        QApplication.instance().aboutToQuit.connect(self.cart_worker.stop_thread)
        
        self.active_cart_axis = None

        self.cart_hold_timer = QTimer(self)
        self.cart_hold_timer.setSingleShot(True)
        self.cart_hold_timer.timeout.connect(self._on_cart_hold_timeout)
        self._cart_press_axis = None
        
        # 3. Gripper 時鐘
        self.gripper_jog_timer = QTimer(self)
        self.gripper_jog_timer.timeout.connect(self._gripper_timer_tick)
        self.active_gripper_sign = 0

    # --- UI 建設函式 ---
    def _create_separator(self):
        sep_layout = QHBoxLayout()
        sep_layout.setContentsMargins(15, 5, 15, 5) 
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet(styles.SEPARATOR_STYLE) 
        sep_layout.addWidget(line)
        self.main_layout.addLayout(sep_layout)

    def _create_header(self, title_text, speed_ctrl):
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 10, 5)
        title = QLabel(title_text)
        title.setFont(styles.FONT_TITLE)
        row.addWidget(title)
        row.addStretch(1)
        row.addWidget(speed_ctrl)
        
        if title_text != "Cartesian Jogging":
            row.addSpacing(10)
            btn_add = QPushButton("+")
            btn_add.setFixedSize(22, 22)
            btn_add.setStyleSheet(styles.BTN_ACTION_STYLE)
            row.addWidget(btn_add)
        self.main_layout.addLayout(row)

    def _create_jog_btn(self, icon_name):
        btn = QPushButton(qta.icon(icon_name, color='#ffffff'), "")
        btn.setIconSize(QSize(16, 16))
        btn.setFixedSize(22, 22)
        btn.setStyleSheet(styles.BTN_JOG_SLIDER_STYLE)
        return btn

    def _bind_joint_controls(self, axis_idx, slider, val_lbl, btn_minus, btn_plus):
        slider.valueChanged.connect(lambda val: val_lbl.setText(f"{val/100:.1f}"))
        btn_minus.pressed.connect(lambda: self._start_joint_jog(axis_idx, -1))
        btn_minus.released.connect(lambda: self._stop_joint_jog(axis_idx))
        btn_plus.pressed.connect(lambda: self._start_joint_jog(axis_idx, 1))
        btn_plus.released.connect(lambda: self._stop_joint_jog(axis_idx))

    def _bind_cartesian_button(self, btn, base_label):
        btn.pressed.connect(lambda: self._start_cartesian_jog(base_label))
        btn.released.connect(lambda: self._stop_cartesian_jog())

    def _setup_joint_ui(self):
        self.j_speed_ctrl = SpeedControlWidget()
        self._create_header("Joint Jogging", self.j_speed_ctrl)

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
            slider.sliderReleased.connect(self.on_joint_slider_released)
            val_lbl.on_commit_cb = self.on_joint_slider_released

            btn_minus = self._create_jog_btn('mdi.transfer-left')
            row.addWidget(btn_minus)
            row.addWidget(slider)

            btn_plus = self._create_jog_btn('mdi.transfer-right')
            row.addWidget(btn_plus)

            row.insertSpacerItem(1, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))
            row.insertSpacerItem(3, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))

            # 呼叫綁定器
            self._bind_joint_controls(i-1, slider, val_lbl, btn_minus, btn_plus)

            joint_container.addLayout(row)
            jog_area.addLayout(joint_container)

        upper_layout.addLayout(jog_area)
        self.main_layout.addLayout(upper_layout)
        self._create_separator()

    def _setup_cartesian_ui(self):
        self.c_speed_ctrl = SpeedControlWidget()
        self._create_header("Cartesian Jogging", self.c_speed_ctrl)
        self.is_cartesian_continuous = True

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
                
                # 呼叫綁定器
                self._bind_cartesian_button(btn, base_label)
                
                cart_grid.addWidget(btn, row_idx, col_idx)
                self.cart_buttons.append((btn, base_label))

        cart_main_layout.addLayout(cart_grid)
        self.main_layout.addLayout(cart_main_layout)
        self._create_separator()

    def _setup_gripper_ui(self):
        self.ee_speed_ctrl = SpeedControlWidget()
        self._create_header("End Effector", self.ee_speed_ctrl)

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

        self.g_slider = QSlider(Qt.Orientation.Horizontal) 
        self.g_slider.setRange(0, 100)
        self.g_slider.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.g_slider.setStyleSheet(styles.SLIDER_GRIPPER_STYLE)
        self.g_slider.sliderReleased.connect(self.on_gripper_slider_released)
        self.g_slider.valueChanged.connect(lambda val, label=g_val_lbl: label.setText(f"{val} %"))

        g_btn_minus = self._create_jog_btn('mdi.transfer-left')
        g_btn_minus.pressed.connect(lambda *args: self._start_gripper_jog(-1))
        g_btn_minus.released.connect(lambda *args: self._stop_gripper_jog())
        g_slider_row.addWidget(g_btn_minus)
        g_slider_row.addWidget(self.g_slider)

        g_btn_plus = self._create_jog_btn('mdi.transfer-right')
        g_btn_plus.pressed.connect(lambda *args: self._start_gripper_jog(1))
        g_btn_plus.released.connect(lambda *args: self._stop_gripper_jog())
        g_slider_row.addWidget(g_btn_plus)

        g_slider_row.insertSpacerItem(1, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))
        g_slider_row.insertSpacerItem(3, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))

        gripper_area.addLayout(g_slider_row)
        lower_layout.addLayout(gripper_area)
        self.main_layout.addLayout(lower_layout) 

    # --- 屬性與 UI 狀態同步 ---
    @property
    def j_speed_level(self): return self.j_speed_ctrl.level
    @property
    def c_speed_level(self): return self.c_speed_ctrl.level
    @property
    def ee_speed_level(self): return self.ee_speed_ctrl.level

    def update_joints_from_ik(self, float_angles):
        for i, slider in enumerate(self.joint_sliders):
            slider.blockSignals(True)
            slider.setValue(int(round(float_angles[i] * 100)))
            slider.blockSignals(False)
            if i < len(self.joint_labels):
                self.joint_labels[i].setText(f"{float_angles[i]:.1f}")

    def toggle_cartesian_frame(self, checked):
        prefix = "T" if checked else "W"
        self.btn_frame_toggle.setText("TRF" if checked else "WRF")
        for btn, base_label in self.cart_buttons:
            btn.setText(f"{prefix}{base_label}")

    def toggle_cartesian_mode(self, checked):
        self.is_cartesian_continuous = not checked
        self.btn_jog_mode.setText("Step" if checked else "Cont")
        self.step_container.setVisible(checked)

    def _get_axis_speed_deg(self, axis_idx, speed_factor):
        return config.JOG_SPEEDS_DEG[axis_idx] * speed_factor

    # --- Joint 運動控制 ---
    def on_joint_slider_changed(self):
        angles = [float(s.value()) / 100.0 for s in self.joint_sliders]
        cb = getattr(self, 'update_3d_callback', None)
        if cb: cb(angles)
            
    def on_joint_slider_released(self):
        angles = [float(s.value()) / 100.0 for s in self.joint_sliders]
        speed_factor = self.j_speed_ctrl.level * 0.25 
        cb = getattr(self, 'send_jog_callback', None)
        if cb: cb(angles, speed_factor)

    def _start_joint_jog(self, axis_idx, direction):
        self._joint_press_axis = axis_idx
        self._joint_press_dir = direction
        self.joint_hold_timer.start(250) 

    def _on_joint_hold_timeout(self):
        axis_idx = self._joint_press_axis
        direction = self._joint_press_dir
        speed_factor = self.j_speed_ctrl.level * 0.25 
        self.active_joint_axis = axis_idx
        self.active_joint_sign = direction

        is_hardware_sent = False
        cb = getattr(self, 'continuous_jog_callback', None)
        if cb:
            is_hardware_sent = cb(axis_idx, direction, speed_factor)

        if is_hardware_sent: 
            self.joint_query_timer.start(20)
        else:
            self._last_joint_tick = time.perf_counter()
            self.joint_sim_timer.setTimerType(Qt.TimerType.PreciseTimer)
            self.joint_sim_timer.start(10)

    def _poll_real_pose(self):
        cb = getattr(self, 'request_pose_callback', None)
        if cb: cb()

    def _stop_joint_jog(self, axis_idx):
        if self.joint_hold_timer.isActive():
            self.joint_hold_timer.stop()
            self._execute_joint_step(axis_idx, self._joint_press_dir)
            if not getattr(self, 'is_simulation_mode', False):
                QTimer.singleShot(500, self._poll_real_pose)
        else:
            is_hardware_stopped = False
            cb = getattr(self, 'continuous_jog_callback', None)
            if cb:
                is_hardware_stopped = cb(axis_idx, 0, 0.0)

            if is_hardware_stopped: QTimer.singleShot(500, self.joint_query_timer.stop)
            else: self.joint_sim_timer.stop()
            
            if self.active_joint_axis == axis_idx:
                self.active_joint_axis = -1

    def _update_joint_simulation(self):
        now = time.perf_counter()
        real_dt = now - self._last_joint_tick
        self._last_joint_tick = now
        if real_dt > 0.1: real_dt = 1.0 / 60.0 

        axis = self.active_joint_axis
        if axis < 0 or axis > 5: return

        speed_factor = self.j_speed_ctrl.level * 0.25
        speed_deg = self._get_axis_speed_deg(axis, speed_factor)
        delta = self.active_joint_sign * speed_deg * real_dt 
        
        slider = self.joint_sliders[axis]
        current_val = slider.value() / 100.0  
        new_val = current_val + delta
        
        min_lim, max_lim = config.JOINT_LIMITS[axis]
        new_val = max(min_lim, min(max_lim, new_val))
        slider.setValue(int(new_val * 100))

    def _execute_joint_step(self, axis_idx, direction):
        step_deg = 0.5 * direction 
        slider = self.joint_sliders[axis_idx]
        current_val = slider.value() / 100.0
        new_val = current_val + step_deg

        min_lim, max_lim = config.JOINT_LIMITS[axis_idx]
        new_val = max(min_lim, min(max_lim, new_val))

        slider.setValue(int(new_val * 100))
        self.on_joint_slider_released()

    # --- Cartesian 運動控制 ---
    def _start_cartesian_jog(self, base_label):
        self._cart_press_axis = base_label
        if self.is_cartesian_continuous: self.cart_hold_timer.start(250)

    def _on_cart_hold_timeout(self):
        self.active_cart_axis = self._cart_press_axis
        self._start_worker_move(is_continuous=True) 

    def _stop_cartesian_jog(self):
        if self.is_cartesian_continuous:
            if self.cart_hold_timer.isActive():
                self.cart_hold_timer.stop()
                return
            else:
                self.cart_worker.stop_move() # 通知背景引擎開始煞車
        else:
            self._execute_cartesian_step(self._cart_press_axis, fixed_step=self.spin_step.value())

    def _execute_cartesian_step(self, base_label, fixed_step):
        if not base_label: return
        if self.cart_worker._is_active or getattr(self, 'active_cart_axis', None) is not None: return
            
        self.active_cart_axis = base_label
        self._start_worker_move(is_continuous=False, fixed_step=fixed_step)

    # 專門負責「讀取 UI 參數並派發任務」的經理
    def _start_worker_move(self, is_continuous, fixed_step=0.0):
        frame = "Tool" if self.btn_frame_toggle.isChecked() else "World"
        axis_str = self.active_cart_axis[:-1] 
        sign = 1 if self.active_cart_axis[-1] == '+' else -1
        is_rot = len(axis_str) > 1
        
        cfg_max_speed = config.MAX_ROT_SPEED if is_rot else config.MAX_LIN_SPEED
        cfg_max_accel = config.MAX_ROT_ACCEL if is_rot else config.MAX_LIN_ACCEL
        max_speed = (cfg_max_speed * 0.5) * (self.c_speed_level / 4.0)
        accel = cfg_max_accel * 1.0 
        
        self._current_frame = frame
        self._current_axis_arg = axis_str if is_rot else axis_str.lower()
        self.cart_worker.start_move(is_continuous, fixed_step, max_speed, accel, sign)

    # 專門負責「接收信號並發送 MCU」的窗口
    def _on_step_computed(self, step_val, is_finished):
        """接收來自背景執行緒的計算結果，安全發送給大腦"""
        axis_arg = getattr(self, '_current_axis_arg', None)
        frame = getattr(self, '_current_frame', "World")

        if is_finished:
            self.active_cart_axis = None
            cb = getattr(self, 'cartesian_jog_callback', None)
            if cb:
                if not self.is_cartesian_continuous:
                    cb(axis_arg, step_val, frame, True)
                
                # 發送 5 個空點位，完美清空 MCU 緩衝區讓馬達煞停
                for _ in range(5): 
                    cb(axis_arg, 0.0, frame, True)
                    
            if getattr(self, 'cartesian_jog_stop_callback', None): 
                self.cartesian_jog_stop_callback()
            if not getattr(self, 'is_simulation_mode', False): 
                QTimer.singleShot(500, self._poll_real_pose)
            return 

        # 正常運算中，發送點位給 IK 引擎
        if getattr(self, 'cartesian_jog_callback', None):
            self.cartesian_jog_callback(axis_arg, step_val, frame, True)

    # --- Gripper 運動控制 ---
    def on_gripper_slider_released(self):
        cb = getattr(self, 'send_gripper_callback', None)
        if cb: cb(self.g_slider.value())

    def _start_gripper_jog(self, sign):
        self.active_gripper_sign = sign
        self._gripper_timer_tick() 
        self.gripper_jog_timer.start(10) 

    def _stop_gripper_jog(self):
        self.gripper_jog_timer.stop()
        self.active_gripper_sign = 0

    def _gripper_timer_tick(self):
        if self.active_gripper_sign == 0: return
        new_val = self.g_slider.value() + self.active_gripper_sign * 5
        new_val = max(self.g_slider.minimum(), min(self.g_slider.maximum(), new_val))
        self.g_slider.setValue(new_val)
        self.on_gripper_slider_released()