# jog_panel.py
import time
from PySide6.QtWidgets import (QDoubleSpinBox, QGridLayout, QSizePolicy, QSpacerItem, 
                               QWidget, QVBoxLayout, QHBoxLayout, QFrame, QLabel, 
                               QPushButton, QSlider, QApplication, QLineEdit)
from PySide6.QtCore import QSize, QThread, QTimer, Qt, Signal
import qtawesome as qta

import styles
import config
from widgets import BaseBlock

# =========================================================
# [1] 笛卡爾空間運動引擎 (Pure Math & QThread Worker)
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
    step_computed_signal = Signal(float, bool) 

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
        absolute_target_time = time.perf_counter()

        while self._is_running:
            if self._is_active:
                absolute_target_time += self.dt
                step_val, is_finished = self.math_engine.compute_step(
                    self.dt, self.max_speed, self.accel, self.sign
                )
                self.step_computed_signal.emit(step_val, is_finished)

                if is_finished:
                    self._is_active = False

                sleep_time = absolute_target_time - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    absolute_target_time = time.perf_counter() 
            else:
                time.sleep(0.01) 
                absolute_target_time = time.perf_counter()

    def start_move(self, is_continuous, fixed_step, max_speed, accel, sign):
        self.max_speed = max_speed
        self.accel = accel
        self.sign = sign
        self.math_engine.reset(is_continuous, fixed_step)
        self._is_active = True

    def stop_move(self):
        self.math_engine.stop()

    def stop_thread(self):
        self._is_running = False
        self.wait()

# =========================================================
# [2] Jog 面板專屬 UI 元件 (EditableValueLabel, SpeedControlWidget)
# =========================================================
class EditableValueLabel(QLineEdit):
    def __init__(self, default_text="0.00", parent=None):
        super().__init__(default_text, parent)
        self.slider = None
        self.on_commit_cb = None  
        self.is_locked = False 

        self.setFixedHeight(12) 
        self.setFixedWidth(50) 
        self.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        self.set_label_mode()
        self.editingFinished.connect(self.commit_value)

    def set_locked(self, locked):
        self.is_locked = locked
        if locked:
            if not self.isReadOnly():
                self.commit_value()
            self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        else:
            self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)

    def set_label_mode(self):
        self.setReadOnly(True)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setStyleSheet(styles.EDITABLE_LABEL_MODE_STYLE)
        self.clearFocus()

    def mousePressEvent(self, event):
        if self.is_locked: return 
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

# =========================================================
# [3] 操作面板 (JogWidget)
# =========================================================
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
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.is_locked = False
        self.is_simulation_mode = False
        self.active_jog_mode = None 
        self._is_braking_monitor_active = False
        self._stable_count = 0
        self._last_checked_angles = None
        self._current_frame = "World"
        self._current_axis_arg = None
        self._last_joint_tick = 0.0
        self._is_gripper_holding = False
        self._cart_press_axis = None
        self._joint_press_axis = -1
        self._joint_press_dir = 0
        
        self.active_joint_axis = -1
        self.active_joint_sign = 0
        self.active_cart_axis = None
        self.active_gripper_sign = 0

        self.check_estop_callback = None
        self.restore_ui_callback = None
        self.update_3d_callback = None
        self.send_jog_callback = None
        self.continuous_jog_callback = None
        self.request_pose_callback = None
        self.precheck_cartesian_step_callback = None
        self.cartesian_jog_callback = None
        self.cartesian_jog_stop_callback = None
        self.send_io_callback = None
        self.warning_log_callback = None 

        self.joint_sliders = []
        self.joint_labels = [] 
        self.joint_jog_btns = [] 
        self.cart_buttons = []
        self.gripper_jog_btns = []

        self._init_timers()
        self._setup_joint_ui()
        self._setup_cartesian_ui()
        self._setup_gripper_ui()

    @property
    def j_speed_level(self): return self.j_speed_ctrl.level
    
    @property
    def c_speed_level(self): return self.c_speed_ctrl.level
    
    @property
    def ee_speed_level(self): return self.ee_speed_ctrl.level

    @property
    def is_jogging(self):
        if self.active_joint_axis != -1: return True
        if self.joint_hold_timer.isActive(): return True
        if self._is_braking_monitor_active: return True
        
        if self.cart_worker._is_active: return True
        if self.active_cart_axis is not None: return True
        if self.cart_hold_timer.isActive(): return True

        if self.active_gripper_sign != 0: return True
        if self.gripper_hold_timer.isActive(): return True
        return False

    def set_locked(self, locked):
        self.is_locked = locked 

        for label in self.joint_labels:
            label.set_locked(locked)

        for slider in self.joint_sliders + [self.g_slider]:
            slider.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, locked)
            slider.setFocusPolicy(Qt.FocusPolicy.NoFocus if locked else Qt.FocusPolicy.StrongFocus)

        self.spin_step.setReadOnly(locked)
        self.spin_step.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, locked)
        self.spin_step.setFocusPolicy(Qt.FocusPolicy.NoFocus if locked else Qt.FocusPolicy.WheelFocus)

        self.btn_frame_toggle.setEnabled(not locked)
        self.btn_jog_mode.setEnabled(not locked)

    def set_cross_interlock(self, active_mode):
        lock_joint = active_mode in ['CARTESIAN', 'GRIPPER']
        lock_cart = active_mode in ['JOINT', 'GRIPPER']
        lock_grip = active_mode in ['JOINT', 'CARTESIAN']

        for slider in self.joint_sliders: 
            slider.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, lock_joint)
            slider.setFocusPolicy(Qt.FocusPolicy.NoFocus if lock_joint else Qt.FocusPolicy.StrongFocus)
        for btn in self.joint_jog_btns: btn.blockSignals(lock_joint)
        for lbl in self.joint_labels: lbl.is_locked = lock_joint

        for btn, _ in self.cart_buttons: btn.blockSignals(lock_cart)
        self.spin_step.blockSignals(lock_cart)
        self.btn_frame_toggle.setEnabled(not lock_cart)
        self.btn_jog_mode.setEnabled(not lock_cart)

        self.g_slider.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, lock_grip)
        self.g_slider.setFocusPolicy(Qt.FocusPolicy.NoFocus if lock_grip else Qt.FocusPolicy.StrongFocus)
        for btn in self.gripper_jog_btns: btn.blockSignals(lock_grip)

    def _is_estop_blocked(self, silent=False):
        if self.check_estop_callback:
            return self.check_estop_callback(silent)
        return False

    def _init_timers(self):
        self.joint_query_timer = QTimer(self)
        self.joint_query_timer.timeout.connect(self._poll_real_pose)

        self.joint_sim_timer = QTimer(self)
        self.joint_sim_timer.timeout.connect(self._update_joint_simulation)

        self.joint_hold_timer = QTimer(self)
        self.joint_hold_timer.setSingleShot(True)
        self.joint_hold_timer.timeout.connect(self._on_joint_hold_timeout)

        self.cart_worker = CartesianJogWorker(self)
        self.cart_worker.step_computed_signal.connect(self._on_step_computed)
        self.cart_worker.start()

        QApplication.instance().aboutToQuit.connect(self.cart_worker.stop_thread)

        self.cart_hold_timer = QTimer(self)
        self.cart_hold_timer.setSingleShot(True)
        self.cart_hold_timer.timeout.connect(self._on_cart_hold_timeout)

        self.gripper_hold_timer = QTimer(self)
        self.gripper_hold_timer.setSingleShot(True)
        self.gripper_hold_timer.timeout.connect(self._on_gripper_hold_timeout)

        self.gripper_jog_timer = QTimer(self)
        self.gripper_jog_timer.timeout.connect(self._gripper_timer_tick)

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

            slider.sliderPressed.connect(lambda: self.set_cross_interlock('JOINT'))
            slider.valueChanged.connect(self.on_joint_slider_changed)
            slider.sliderReleased.connect(self.on_joint_slider_released)
            val_lbl.on_commit_cb = self.on_joint_slider_released

            btn_minus = self._create_jog_btn('mdi.transfer-left')
            row.addWidget(btn_minus)
            row.addWidget(slider)

            btn_plus = self._create_jog_btn('mdi.transfer-right')
            row.addWidget(btn_plus)

            self.joint_jog_btns.extend([btn_minus, btn_plus])

            row.insertSpacerItem(1, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))
            row.insertSpacerItem(3, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))

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
        self.spin_step.setRange(0.01, 999.0)
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
        
        for row_idx, row_items in enumerate(cartesian_labels):
            for col_idx, base_label in enumerate(row_items):
                btn = QPushButton(f"W{base_label}") 
                btn.setFixedHeight(32)
                btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
                btn.setStyleSheet(styles.BTN_CARTESIAN_STYLE)

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

        self.g_slider.sliderPressed.connect(lambda: self.set_cross_interlock('GRIPPER'))
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

        self.gripper_jog_btns.extend([g_btn_minus, g_btn_plus])

        g_slider_row.insertSpacerItem(1, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))
        g_slider_row.insertSpacerItem(3, QSpacerItem(5, 0, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum))

        gripper_area.addLayout(g_slider_row)
        lower_layout.addLayout(gripper_area)
        self.main_layout.addLayout(lower_layout) 

    def update_joints_from_ik(self, float_angles):
        if self.active_jog_mode != 'WAIT_BRAKE':
            for slider, label, angle in zip(self.joint_sliders, self.joint_labels, float_angles):
                slider.blockSignals(True)
                slider.setValue(int(round(angle * 100)))
                disp_str = f"{angle:.1f}"
                if disp_str == "-0.0": disp_str = "0.0"
                label.setText(disp_str) 
                
                slider.blockSignals(False)

        if self._is_braking_monitor_active:
            if self._last_checked_angles is not None:
                max_diff = max(abs(curr - prev) for curr, prev in zip(float_angles, self._last_checked_angles))
                self._stable_count = self._stable_count + 1 if max_diff < 0.02 else 0
            else:
                self._stable_count = 0

            self._last_checked_angles = list(float_angles)

            if self._stable_count >= 3:
                self.joint_query_timer.stop()            
                self._is_braking_monitor_active = False
                self._last_checked_angles = None 

                self.active_jog_mode = None 
                self.set_cross_interlock(None) 

                if not getattr(self, 'is_simulation_mode', False) and self.request_pose_callback:
                    self.request_pose_callback()

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
        if self.is_locked or self._is_estop_blocked(silent=True): 
            return 

        angles = [float(s.value()) / 100.0 for s in self.joint_sliders]
        if self.update_3d_callback:
            self.update_3d_callback(angles)

    def on_joint_slider_released(self):
        if self.is_locked:
            if self.warning_log_callback: 
                self.warning_log_callback("[WARNING] Cannot operate joint sliders while path is running.")
            if self.restore_ui_callback: self.restore_ui_callback()
            return

        if self._is_estop_blocked(silent=False): 
            if self.restore_ui_callback: self.restore_ui_callback()
            self.set_cross_interlock(None) 
            return

        self.set_cross_interlock('JOINT') 
        self.active_jog_mode = 'WAIT_BRAKE' 

        angles = [float(s.value()) / 100.0 for s in self.joint_sliders]
        speed_factor = self.j_speed_ctrl.level * 0.25 

        is_sent = False
        if self.send_jog_callback:
            is_sent = self.send_jog_callback(angles, speed_factor)

        if self.is_simulation_mode or not is_sent:
            self.active_jog_mode = None
            self.set_cross_interlock(None)
            return

        self._is_braking_monitor_active = True
        self._stable_count = 0
        self._last_checked_angles = None
        self.joint_query_timer.start(20)

    def _start_joint_jog(self, axis_idx, direction):
        if self.is_locked:
            if self.warning_log_callback: 
                self.warning_log_callback("[WARNING] Cannot operate joint jog while path is running.")
            return
        if self._is_estop_blocked(): return
        self.set_cross_interlock('JOINT') 

        self._is_braking_monitor_active = False 
        self._joint_press_axis = axis_idx
        self._joint_press_dir = direction
        self.joint_hold_timer.start(250)

    def _on_joint_hold_timeout(self):
        self.active_joint_axis = self._joint_press_axis
        self.active_joint_sign = self._joint_press_dir
        speed_factor = self.j_speed_ctrl.level * 0.25 

        self.active_jog_mode = 'CONT' 

        is_hardware_sent = False
        if self.continuous_jog_callback:
            is_hardware_sent = self.continuous_jog_callback(self.active_joint_axis, self.active_joint_sign, speed_factor)

        if is_hardware_sent: 
            self.joint_query_timer.start(20)
        else:
            self._last_joint_tick = time.perf_counter()
            self.joint_sim_timer.setTimerType(Qt.TimerType.PreciseTimer)
            self.joint_sim_timer.start(10)

    def _poll_real_pose(self):
        if self.request_pose_callback:
            self.request_pose_callback()

    def _stop_joint_jog(self, axis_idx):
        if self.is_locked: 
            return 

        if self._is_estop_blocked(): 
            self.active_jog_mode = None
            self.set_cross_interlock(None)
            self.joint_hold_timer.stop()
            self.joint_sim_timer.stop()
            if self.active_joint_axis == axis_idx:
                self.active_joint_axis = -1
            return

        if self.joint_hold_timer.isActive():
            self.joint_hold_timer.stop()
            self._execute_joint_step(axis_idx, self._joint_press_dir)
        else:
            is_hardware_stopped = False
            if self.continuous_jog_callback:
                is_hardware_stopped = self.continuous_jog_callback(axis_idx, 0, 0.0)

            if is_hardware_stopped: 
                self._is_braking_monitor_active = True
                self._stable_count = 0
                self._last_checked_angles = None
            else: 
                self.joint_sim_timer.stop()
                self.active_jog_mode = None 
                self.set_cross_interlock(None) 

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
        if self.is_locked:
            if self.warning_log_callback: 
                self.warning_log_callback("[WARNING] Cannot operate Cartesian jog while path is running.")
            return
        if self._is_estop_blocked(): return 
        self.set_cross_interlock('CARTESIAN') 

        self._cart_press_axis = base_label
        if self.is_cartesian_continuous: self.cart_hold_timer.start(250)

    def _on_cart_hold_timeout(self):
        self.active_jog_mode = 'CONT' 
        self.active_cart_axis = self._cart_press_axis
        self._start_worker_move(is_continuous=True) 

    def _stop_cartesian_jog(self):
        if self.is_locked: 
            return 

        if self._is_estop_blocked(): 
            self.set_cross_interlock(None)
            self.cart_hold_timer.stop()
            return 
            
        if self.is_cartesian_continuous:
            if self.cart_hold_timer.isActive():
                self.cart_hold_timer.stop()
                self.set_cross_interlock(None) 
            else:
                self.cart_worker.stop_move() 
        else:
            self._execute_cartesian_step(self._cart_press_axis, fixed_step=self.spin_step.value())

    def _execute_cartesian_step(self, base_label, fixed_step):
        if self.is_locked:
            if self.warning_log_callback: 
                self.warning_log_callback("[WARNING] Cannot operate step Cartesian jogging while path is running.")
            return
        if self._is_estop_blocked(): return 
        if not base_label: return
        if self.cart_worker._is_active or self.active_cart_axis is not None: return

        if self.precheck_cartesian_step_callback:
            axis_str = base_label[:-1] 
            sign = 1 if base_label[-1] == '+' else -1
            is_rot = len(axis_str) > 1
            axis_arg = axis_str if is_rot else axis_str.lower()
            frame = "Tool" if self.btn_frame_toggle.isChecked() else "World"

            if not self.precheck_cartesian_step_callback(axis_arg, fixed_step * sign, frame):
                self.set_cross_interlock(None) 
                return 

        self.active_cart_axis = base_label
        self._start_worker_move(is_continuous=False, fixed_step=fixed_step)

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

    def _on_step_computed(self, step_val, is_finished):
        axis_arg = self._current_axis_arg
        frame = self._current_frame

        if is_finished:
            self.active_cart_axis = None
            if self.cartesian_jog_callback:
                if not self.is_cartesian_continuous:
                    self.cartesian_jog_callback(axis_arg, step_val, frame, True)

                for _ in range(20): 
                    self.cartesian_jog_callback(axis_arg, 0.0, frame, True)

            if self.cartesian_jog_stop_callback: 
                self.cartesian_jog_stop_callback()
            
            is_connected = False
            if self.request_pose_callback:
                is_connected = self.request_pose_callback()

            if not getattr(self, 'is_simulation_mode', False) and is_connected: 
                self.active_jog_mode = 'WAIT_BRAKE' 
                self._is_braking_monitor_active = True
                self._stable_count = 0
                self._last_checked_angles = None
                self.joint_query_timer.start(20)
            else:
                # 離線或模擬模式，直接瞬間解鎖！
                self.active_jog_mode = None
                self.set_cross_interlock(None) 
            return

        if self.cartesian_jog_callback:
            self.cartesian_jog_callback(axis_arg, step_val, frame, True)

    # --- Gripper 運動控制 ---
    def on_gripper_slider_released(self):
        if self.is_locked:
            if self.warning_log_callback:
                self.warning_log_callback("[WARNING] Cannot operate gripper while path is running.")
            return 

        self.set_cross_interlock(None) 
        if self._is_estop_blocked(): return
        if self.send_io_callback:
            self.send_io_callback(self.g_slider.value())

    def _start_gripper_jog(self, sign):
        if self.is_locked:
            if self.warning_log_callback:
                self.warning_log_callback("[WARNING] Cannot operate gripper while path is running.")
            return
        if self._is_estop_blocked(): return
        self.set_cross_interlock('GRIPPER') 

        self.active_gripper_sign = sign
        self._is_gripper_holding = False
        self.gripper_hold_timer.start(250) 

    def _on_gripper_hold_timeout(self):
        if self.active_gripper_sign != 0:
            self._is_gripper_holding = True
            self._execute_gripper_step(is_hold=True) 
            self.gripper_jog_timer.start(30)         

    def _stop_gripper_jog(self):
        if self.is_locked: 
            return 

        if self._is_estop_blocked(): 
            self.set_cross_interlock(None)
            self.gripper_hold_timer.stop()
            self.gripper_jog_timer.stop()
            self.active_gripper_sign = 0
            return

        self.gripper_hold_timer.stop()
        self.gripper_jog_timer.stop()

        if not self._is_gripper_holding and self.active_gripper_sign != 0:
            self._execute_gripper_step(is_hold=False)

        self.active_gripper_sign = 0
        self.set_cross_interlock(None)

    def _gripper_timer_tick(self):
        self._execute_gripper_step(is_hold=True)

    def _execute_gripper_step(self, is_hold):
        if not is_hold:
            step_size = self.ee_speed_level 
        else:
            step_size = (self.ee_speed_level / 4.0) * 10.0

        new_val = self.g_slider.value() + self.active_gripper_sign * step_size
        new_val = max(self.g_slider.minimum(), min(self.g_slider.maximum(), new_val))

        if int(new_val) != self.g_slider.value():
            self.g_slider.setValue(int(new_val))
            self.on_gripper_slider_released()