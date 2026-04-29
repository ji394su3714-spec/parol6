# ui/widgets.py
from PyQt6.QtWidgets import (QListWidget, QMenu, QPushButton, QFrame, QHBoxLayout, QVBoxLayout, QSpinBox, QLineEdit,
                             QLabel, QWidget, QSlider, QDoubleSpinBox, QTextEdit, QSizePolicy)
from PyQt6.QtCore import Qt, QSize, pyqtSignal
from PyQt6.QtGui import QRegion
import qtawesome as qta
from ui.styles import JOINT_SLIDER_STYLE

def create_repeat_btn(text, style_class, width, height, delay=200, interval=50):
    """按鈕工廠：快速生成支援連發的按鈕"""
    btn = QPushButton(text)
    btn.setProperty("class", style_class)
    btn.setFixedSize(width, height)
    btn.setAutoRepeat(True)
    btn.setAutoRepeatDelay(delay)
    btn.setAutoRepeatInterval(interval)
    return btn

# --- 圓形按鈕 ---
class CircularButton(QPushButton):
    def __init__(self, icon_name, tooltip, parent=None, checkable=False, checked=False):
        super().__init__(parent)
        self.setFixedSize(55, 55)       
        self.setToolTip(tooltip)
        self.setCursor(Qt.CursorShape.PointingHandCursor)   
        self.setIcon(qta.icon(icon_name, color='#34495e'))
        self.setIconSize(QSize(28, 28))
        if checkable:
            self.setCheckable(True)
            self.setChecked(checked)
            
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.setMask(QRegion(self.rect(), QRegion.RegionType.Ellipse))
        radius = self.width() // 2
        c_normal, c_hover = "#95a5a6", "#7f8c8d"
        self.setStyleSheet(f"""
            QPushButton {{ background-color: rgba(245, 245, 245, 220); border: 2px solid {c_normal}; border-radius: {radius}px; }}
            QPushButton:hover {{ background-color: #ffffff; border: 2px solid {c_hover}; }}
            QPushButton:checked {{ background-color: #bdc3c7; border: 2px solid {c_normal}; padding: 2px 0 0 2px; }}
            QPushButton:pressed {{ background-color: #bdc3c7; border: 2px solid {c_normal}; }}
        """)

class CartesianControlRow(QWidget):
    jogRequested = pyqtSignal(str, int)

    def __init__(self, name, axis, mon_lbl, mon_widget, parent=None):
        super().__init__(parent)
        self.axis = axis
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0) 
        
        self.btn_minus = create_repeat_btn(f"{name}-", "JogBtn", 125, 50)
        self.btn_plus  = create_repeat_btn(f"{name}+", "JogBtn", 125, 50)
        
        self.btn_minus.clicked.connect(lambda: self.jogRequested.emit(self.axis, -1))
        self.btn_plus.clicked.connect(lambda: self.jogRequested.emit(self.axis, 1))
        
        mon_frame = MonitorFrame(mon_lbl, mon_widget)
        layout.addWidget(self.btn_minus)
        layout.addWidget(self.btn_plus)
        layout.addStretch()
        layout.addWidget(mon_frame)
        
    def set_auto_repeat(self, enabled):
        self.btn_minus.setAutoRepeat(enabled)
        self.btn_plus.setAutoRepeat(enabled)

# --- Monitor Frame ---
class MonitorFrame(QFrame):
    def __init__(self, label_text, value_widget, parent=None):
        super().__init__(parent)
        self.setFixedHeight(50)
        self.setFixedWidth(270)
        self.setStyleSheet("QFrame { background-color: #1e1e1e; border: 1px solid #555; border-radius: 5px; }")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 0, 10, 0)
        lbl = QLabel(label_text)
        lbl.setStyleSheet("color: #aaa; font-size: 24px; font-weight: bold; border: none;")
        val = value_widget
        val.setStyleSheet("color: #4cd964; font-size: 24px; font-weight: bold; border: none;") 
        val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(lbl)
        layout.addStretch()
        layout.addWidget(val)

# --- Joint Control Row ---
class JointControlRow(QWidget):
    valueChanged = pyqtSignal(float)  
    editingFinished = pyqtSignal()    

    def __init__(self, index, min_val, max_val, parent=None):
        super().__init__(parent)
        self.index = index
        
        self._layout = QHBoxLayout(self)
        self._layout.setContentsMargins(0, 3, 0, 3) 
        self._layout.setSpacing(10)
        
        self.lbl = QLabel(f"J{index+1}")
        self.lbl.setFixedWidth(30)
        self.lbl.setStyleSheet("font-weight: bold; font-size: 24px;")
        
        self.spin = QDoubleSpinBox()
        self.spin.setRange(min_val, max_val)
        self.spin.setDecimals(1) 
        self.spin.setSingleStep(1.0)
        self.spin.setFixedWidth(85)
        self.spin.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
        self.spin.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.spin.setFixedHeight(40)
        self.spin.setStyleSheet("QDoubleSpinBox { padding: 0px; margin: 0px; }")
        
        self.btn_minus = create_repeat_btn("◀", "JointBtn", 36, 36)
        
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(int(min_val * 100), int(max_val * 100))
        self.slider.setFixedHeight(28)
        self.slider.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.slider.setStyleSheet(JOINT_SLIDER_STYLE)

        self.btn_plus = create_repeat_btn("▶", "JointBtn", 36, 36)

        self._layout.addWidget(self.lbl)
        self._layout.addWidget(self.spin)
        self._layout.addWidget(self.btn_minus)
        self._layout.addWidget(self.slider)
        self._layout.addWidget(self.btn_plus)
        
        self.slider.valueChanged.connect(self._on_slider_change)
        self.spin.valueChanged.connect(self._on_spin_change)
        
        self.btn_minus.clicked.connect(lambda: self._step_value(-0.5))
        self.btn_plus.clicked.connect(lambda: self._step_value(0.5))
        
        self.slider.sliderReleased.connect(self.editingFinished.emit)
        self.spin.editingFinished.connect(self.editingFinished.emit)

    def _step_value(self, step):
        new_val = max(self.spin.minimum(), min(self.spin.maximum(), self.spin.value() + step))
        self.spin.setValue(new_val)
        self.editingFinished.emit()

    def _on_slider_change(self, val):
        real_val = val / 100.0
        self.spin.blockSignals(True)
        self.spin.setValue(real_val)
        self.spin.blockSignals(False)
        self.valueChanged.emit(real_val)

    def _on_spin_change(self, val):
        slider_val = int(val * 100)
        self.slider.blockSignals(True)
        self.slider.setValue(slider_val)
        self.slider.blockSignals(False)
        self.valueChanged.emit(val)

    def set_value(self, val):
        self.spin.blockSignals(True)
        self.slider.blockSignals(True)
        self.spin.setValue(val)
        self.slider.setValue(int(val * 100))
        self.spin.blockSignals(False)
        self.slider.blockSignals(False)

# --- Log Widget ---
class LogWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setObjectName("sys_log")
        layout.addWidget(self.text_edit)
        
        self.btn_clear = QPushButton(self)
        self.btn_clear.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_clear.setIcon(qta.icon('fa5s.trash-alt', color='#7f8c8d'))
        self.btn_clear.setIconSize(QSize(22, 22))
        self.btn_clear.setFixedSize(32, 32)
        self.btn_clear.setToolTip("Clear Log")
        self.btn_clear.setObjectName("btn_clear_log")
        self.btn_clear.clicked.connect(self.text_edit.clear)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        margin = 15 
        x = self.width() - self.btn_clear.width() - margin
        y = 5
        self.btn_clear.move(x, y)

    def append_log(self, text):
        self.text_edit.append(text)
        sb = self.text_edit.verticalScrollBar()
        sb.setValue(sb.maximum())

    def clear_log(self):
        self.text_edit.clear()

# 泛用型內聯編輯器
class InlineEdit(QLineEdit):
    def __init__(self, text, parent_row, save_cb, align=Qt.AlignmentFlag.AlignLeft, click_cb=None):
        super().__init__(text)
        self.parent_row = parent_row
        self.save_cb = save_cb
        self.align_mode = align
        self.click_cb = click_cb 
        self.setReadOnly(True)
        self.set_label_style()
        self.editingFinished.connect(self.finish_edit)

    def set_label_style(self):
        self.setStyleSheet("background: transparent; border: none; padding: 0px; color: black;")
        self.setCursorPosition(0)
        self.setAlignment(self.align_mode)

    def set_edit_style(self):
        self.setStyleSheet("background: white; border: 1px solid #3498db; padding: 0px; color: black;")
        self.setAlignment(self.align_mode)

    def mousePressEvent(self, event):
        if self.isReadOnly():
            if self.click_cb:
                self.click_cb()
            event.accept() 
        else:
            super().mousePressEvent(event)

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton: 
            self.setReadOnly(False)
            self.set_edit_style()
            self.setFocus()
            self.selectAll()

    def focusOutEvent(self, event):
        super().focusOutEvent(event)
        self.finish_edit()

    def finish_edit(self):
        if not self.isReadOnly():
            self.setReadOnly(True)
            self.set_label_style()
            self.save_cb(self.text())

# 極簡下拉選單標籤
class DropdownLabel(QLabel):
    def __init__(self, parent_row, options, save_cb, align=Qt.AlignmentFlag.AlignCenter):
        super().__init__(parent_row) 
        self.parent_row = parent_row
        self.options = options
        self.save_cb = save_cb
        self.setAlignment(align)
        self.setStyleSheet("color: black; background: transparent; font-weight: normal;")

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            event.accept() 
            menu = QMenu() 
            
            menu.setStyleSheet("""
                QMenu { background-color: white; border: 1px solid #c0c0c0; }
                QMenu::item { padding: 4px 15px; }
                QMenu::item:selected { background-color: #3498db; color: white; }
            """)
            for opt in self.options:
                action = menu.addAction(opt)
                action.triggered.connect(lambda checked, o=opt: self.save_cb(o))
            
            menu.exec(event.globalPosition().toPoint())

# 標題列 (Header)
class WaypointHeader(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.setObjectName("waypointHeader")
        self.setStyleSheet("""
            QFrame#waypointHeader {
                background-color: white;      
                border: 1px solid #c0c0c0;    
                border-bottom: 1px solid #e0e0e0; 
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QLabel {
                color: #555555; font-weight: normal; font-size: 24px;
                border: none; background: transparent;
            }
        """)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 6, 5, 6)
        layout.setSpacing(5)
        
        lbl_idx = QLabel("")
        lbl_idx.setFixedWidth(30) 
        layout.addWidget(lbl_idx)
        
        lbl_type = QLabel("")
        lbl_type.setFixedWidth(75) 
        lbl_type.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(lbl_type)
        
        lbl_name = QLabel(" Name")
        layout.addWidget(lbl_name)

        lbl_blend = QLabel("Blend")
        lbl_blend.setFixedWidth(75)
        lbl_blend.setAlignment(Qt.AlignmentFlag.AlignCenter) 
        layout.addWidget(lbl_blend)
        
        lbl_speed = QLabel("Speed")
        lbl_speed.setFixedWidth(75)
        lbl_speed.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(lbl_speed)

        lbl_accel = QLabel("Acc")
        lbl_accel.setFixedWidth(75)
        lbl_accel.setAlignment(Qt.AlignmentFlag.AlignCenter) 
        layout.addWidget(lbl_accel)
        
        lbl_action = QLabel("")
        lbl_action.setFixedWidth(65)
        lbl_action.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(lbl_action)
        
        self.lbl_scrollbar = QLabel("")
        self.lbl_scrollbar.setFixedWidth(0) 
        layout.addWidget(self.lbl_scrollbar)

    def set_scrollbar_width(self, width):
        self.lbl_scrollbar.setFixedWidth(width)

class WaypointRow(QWidget):
    def __init__(self, index, data, on_toggle_cb, on_delete_cb, on_update_cb=None, on_preview_cb=None, parent=None):
        super().__init__(parent)
        self.data = data 
        self.index = index
        self.on_update_cb = on_update_cb
        self.on_preview_cb = on_preview_cb
        
        is_active = data.get('active', True)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(5) 
        
        self.lbl_index = QLabel(f"{index+1}.")
        self.lbl_index.setFixedWidth(30) 
        self.lbl_index.setStyleSheet("color: black; font-weight: normal;" if is_active else "color: gray;")
        layout.addWidget(self.lbl_index)
        
        self.type_lbl = DropdownLabel(self, ["PTP", "N_PTP", "LIN", "CIRC", "I/O"], self.save_type)
        self.type_lbl.setFixedWidth(75) 
        self.update_type_display()
        layout.addWidget(self.type_lbl)
        
        name = data.get('name', f'Point {index+1}')
        self.edit_name = InlineEdit(name, self, self.save_new_name, Qt.AlignmentFlag.AlignLeft, click_cb=self._handle_name_click)
        
        if not is_active:
            self.edit_name.setStyleSheet("background: transparent; border: none; padding: 0px; color: gray;")
            
        layout.addWidget(self.edit_name)

        dw = QWidget()
        dw.setFixedWidth(75) 
        dl = QHBoxLayout(dw)
        dl.setContentsMargins(0, 0, 0, 0)
        dl.setSpacing(2)

        self.blend_lbl = DropdownLabel(self, ["FINE", "10%", "25%", "50%", "75%", "100%"], self.save_blend, align=Qt.AlignmentFlag.AlignRight)
        self.blend_lbl.setFixedWidth(75)
        self.update_blend_display()
        layout.addWidget(self.blend_lbl)
        
        self.speed_lbl = DropdownLabel(self, ["10%", "25%", "50%", "80%", "90%", "100%"], self.save_speed, align=Qt.AlignmentFlag.AlignRight)
        self.speed_lbl.setFixedWidth(75) 
        self.update_speed_display()
        layout.addWidget(self.speed_lbl)

        self.accel_lbl = DropdownLabel(self, ["10%", "25%", "50%", "80%", "90%", "100%"], self.save_accel, align=Qt.AlignmentFlag.AlignRight)
        self.accel_lbl.setFixedWidth(75) 
        self.update_accel_display()
        layout.addWidget(self.accel_lbl)
        
        eye_icon = 'fa5s.eye' if is_active else 'fa5s.eye-slash'
        eye_color = '#34495e' if is_active else '#bdc3c7'
        
        btn_eye = QPushButton()
        btn_eye.setIcon(qta.icon(eye_icon, color=eye_color))
        btn_eye.setIconSize(QSize(24, 24))
        btn_eye.setFixedWidth(30)
        btn_eye.setStyleSheet("border: none; background: transparent;")
        btn_eye.clicked.connect(lambda: on_toggle_cb(index))
        layout.addWidget(btn_eye)

        btn_del = QPushButton()
        btn_del.setIcon(qta.icon('fa5s.trash-alt', color='black'))
        btn_del.setIconSize(QSize(24, 24))
        btn_del.setFixedWidth(30)
        btn_del.setStyleSheet("border: none; background: transparent;")
        btn_del.clicked.connect(lambda: on_delete_cb(index))
        layout.addWidget(btn_del)

        # 在 __init__ 的最後一行加上：
        self.update_ui_by_type()

    def _handle_name_click(self):
        list_widget = self.parentWidget()
        while list_widget is not None and not hasattr(list_widget, 'setCurrentRow'):
            list_widget = list_widget.parentWidget()
        if hasattr(list_widget, 'setCurrentRow'):
            list_widget.setCurrentRow(self.index)
        
        if self.on_preview_cb:
            self.on_preview_cb(self.index)

    def mousePressEvent(self, event):
        event.accept()

    def save_type(self, new_type):
        self.data['type'] = new_type
        self.update_type_display()
        if self.on_update_cb:
            from PyQt6.QtCore import QTimer
            QTimer.singleShot(10, self.on_update_cb)

    # Blend 的儲存與顯示
    def save_blend(self, blend_str):
        self.data['blend'] = blend_str
        self.update_blend_display()

    def update_blend_display(self):
        # 預設為 FINE (安全到位模式)
        blend_val = self.data.get('blend', 'FINE')
        self.blend_lbl.setText(blend_val)
        self.blend_lbl.setStyleSheet("color: black; background: transparent; font-weight: normal;")

    # 新增：根據 Type 動態隱藏 UI
    def update_ui_by_type(self):
        # 安全防護：如果這兩個 UI 元件還沒被建立出來，就直接 Return 跳過
        if not hasattr(self, 'speed_lbl') or not hasattr(self, 'accel_lbl') or not hasattr(self, 'blend_lbl'):
            return
        m_type = self.data.get('type', 'PTP')
        is_delay = (m_type == 'DELAY')
        # 如果是 Delay 獨立封包，就不顯示速度跟加速度
        self.speed_lbl.setVisible(not is_delay)
        self.accel_lbl.setVisible(not is_delay)
        self.blend_lbl.setVisible(not is_delay)

    def update_type_display(self):
        move_type = self.data.get('type', 'PTP')
        is_active = self.data.get('active', True)
        color = "black" if is_active else "gray"
        self.type_lbl.setText(f"{move_type}")
        self.type_lbl.setStyleSheet(f"color: {color}; font-weight: normal; background: transparent;")
        self.update_ui_by_type() # 切換 Type 時同步更新 UI

    # 讓使用者可以直接透過修改 Name 來改變 Delay 時間 (例如輸入 5，自動變成 Wait 5.0s)
    def save_new_name(self, new_text):
        if self.data.get('type') == 'DELAY':
            try:
                # 萃取數字
                val = float(''.join(c for c in new_text if c.isdigit() or c == '.'))
                self.data['value'] = val
                self.data['name'] = f"Wait {val}s"
            except ValueError:
                pass
            self.edit_name.setText(self.data['name'])
        else:
            if new_text.strip():
                self.data['name'] = new_text.strip()
            else:
                self.edit_name.setText(self.data.get('name', f'Point {self.index+1}'))

    def save_accel(self, accel_str):
        val = float(accel_str.replace('%', ''))
        self.data['accel'] = val
        self.update_accel_display()

    def update_accel_display(self):
        acc = self.data.get('accel', 50.0)
        self.accel_lbl.setText(f"{int(acc)}%")
        self.accel_lbl.setStyleSheet("color: black; background: transparent; font-weight: normal;")


    def save_speed(self, speed_str):
        val = float(speed_str.replace('%', ''))
        self.data['speed'] = val
        self.update_speed_display()

    def update_speed_display(self):
        spd = self.data.get('speed', 50.0)
        self.speed_lbl.setText(f"{int(spd)}%")
        self.speed_lbl.setStyleSheet("color: black; background: transparent; font-weight: normal;")

    def update_speed(self, val):
        self.data['speed'] = float(val)

    def update_delay(self, val):
        self.data['delay'] = float(val)

class WaypointListWidget(QListWidget):
    """自訂樣式的路徑點清單元件"""
    def __init__(self, parent=None):
        super().__init__(parent)