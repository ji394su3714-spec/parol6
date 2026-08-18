# widgets.py
import ctypes
from ctypes import wintypes
import datetime
import os

from PySide6.QtWidgets import (QVBoxLayout, QHBoxLayout, QFrame, QLabel, 
                               QPushButton, QTextEdit, QLineEdit, QApplication, 
                               QMenu, QMessageBox)
from PySide6.QtCore import QTimer, Qt, QObject, QEvent, QSize, Signal, QRunnable, QThreadPool
from PySide6.QtGui import QAction, QCursor, QDoubleValidator, QImage, QResizeEvent
import qtawesome as qta

import styles
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

    def commit_value(self):
        if not self.isReadOnly():
            self.editingFinished.emit() # 確保發送修改完成的訊號
            self.clearFocus()           # 清除焦點，這會自動觸發 focusOutEvent 轉回唯讀模式

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
        main_win = self.window()
        if getattr(main_win, '_is_estopped_latched', False):
            main_win.log_widget.append_log("[警告] 機器處於「急停鎖存狀態」！請按「解除急停」恢復運作。")
            return      
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
                disp_str = f"{val:.2f}"
                if disp_str == "-0.00": disp_str = "0.00"
                self.inputs[name].setText(disp_str)
                
        raw_str = f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f} Rx:{rx:.2f} Ry:{ry:.2f} Rz:{rz:.2f}"
        self.current_tcp_str = raw_str.replace("-0.00", "0.00")

    def update_joints(self, *joints):
        for name, val in zip(["θ1", "θ2", "θ3", "θ4", "θ5", "θ6"], joints):
            if not self.inputs[name].hasFocus():
                disp_str = f"{val:.2f}"
                if disp_str == "-0.00": disp_str = "0.00"
                self.inputs[name].setText(disp_str)
                
        raw_str = " ".join([f"J{i+1}:{j:.2f}" for i, j in enumerate(joints)])
        self.current_joints_str = raw_str.replace("-0.00", "0.00")

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
        elif "RECORDED" in msg_upper or "UPDATED" in msg_upper or "DELETED" in msg_upper: color = "#00e6b8" 
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
