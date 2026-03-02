# ui/widgets.py
from PyQt5.QtWidgets import (QListWidget, QMenu, QPushButton, QFrame, QHBoxLayout, QVBoxLayout, QSpinBox, QLineEdit,
                             QLabel, QWidget, QSlider, QDoubleSpinBox, QTextEdit, QSizePolicy)
from PyQt5.QtCore import Qt, QSize, pyqtSignal, QEvent
from PyQt5.QtGui import QRegion
import qtawesome as qta
from ui import styles

# --- 圓形按鈕 (保持不變) ---
class CircularButton(QPushButton):
    def __init__(self, icon_name, tooltip, parent=None, checkable=False, checked=False):
        super().__init__(parent)
        self.setFixedSize(55, 55)       
        self.setToolTip(tooltip)
        self.setCursor(Qt.PointingHandCursor)   
        self.setIcon(qta.icon(icon_name, color='#34495e'))
        self.setIconSize(QSize(28, 28))
        if checkable:
            self.setCheckable(True)
            self.setChecked(checked)
            
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.setMask(QRegion(self.rect(), QRegion.Ellipse))
        radius = self.width() // 2
        c_normal, c_hover = "#95a5a6", "#7f8c8d"
        self.setStyleSheet(f"""
            QPushButton {{ background-color: rgba(245, 245, 245, 220); border: 2px solid {c_normal}; border-radius: {radius}px; }}
            QPushButton:hover {{ background-color: #ffffff; border: 2px solid {c_hover}; }}
            QPushButton:checked {{ background-color: #bdc3c7; border: 2px solid {c_normal}; padding: 2px 0 0 2px; }}
            QPushButton:pressed {{ background-color: #bdc3c7; border: 2px solid {c_normal}; }}
        """)

class CartesianControlRow(QWidget):
    # 定義一個訊號，當按鈕被點擊時，送出 (軸名稱, 方向正負)
    jogRequested = pyqtSignal(str, int)

    def __init__(self, name, axis, mon_lbl, mon_widget, parent=None):
        super().__init__(parent)
        self.axis = axis
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0) # 消除多餘邊界
        
        # 建立按鈕並套用樣式與尺寸
        self.btn_minus = QPushButton(f"{name}-")
        self.btn_plus = QPushButton(f"{name}+")
        self.btn_minus.setStyleSheet(styles.BTN_JOG_STYLE)
        self.btn_plus.setStyleSheet(styles.BTN_JOG_STYLE)
        self.btn_minus.setFixedSize(125, 50)
        self.btn_plus.setFixedSize(125, 50)
        
        # 綁定點擊事件，觸發對外訊號
        self.btn_minus.clicked.connect(lambda: self.jogRequested.emit(self.axis, -1))
        self.btn_plus.clicked.connect(lambda: self.jogRequested.emit(self.axis, 1))
        
        # 建立 Monitor
        mon_frame = MonitorFrame(mon_lbl, mon_widget)
        
        # 排版：按鈕 -> 按鈕 -> 彈簧 -> Monitor
        layout.addWidget(self.btn_minus)
        layout.addWidget(self.btn_plus)
        layout.addStretch()
        layout.addWidget(mon_frame)
        
    def set_auto_repeat(self, enabled, delay=200, interval=50):
        """供外部呼叫，用來統一開關左右按鈕的連發功能"""
        self.btn_minus.setAutoRepeat(enabled)
        self.btn_plus.setAutoRepeat(enabled)
        if enabled:
            self.btn_minus.setAutoRepeatDelay(delay)
            self.btn_minus.setAutoRepeatInterval(interval)
            self.btn_plus.setAutoRepeatDelay(delay)
            self.btn_plus.setAutoRepeatInterval(interval)

# --- Monitor Frame (保持不變) ---
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
        val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
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
        
        # 改為純橫向佈局
        self._layout = QHBoxLayout(self)
        self._layout.setContentsMargins(0, 3, 0, 3) # 上下留一點點呼吸空間
        self._layout.setSpacing(10)
        
        # 1. 標籤 (例如: J1)
        self.lbl = QLabel(f"J{index+1}")
        self.lbl.setFixedWidth(30)
        self.lbl.setStyleSheet("font-weight: bold; font-size: 24px;")
        
        # 2. 數值框 (圖片中緊跟在標籤後)
        self.spin = QDoubleSpinBox()
        self.spin.setRange(min_val, max_val)
        self.spin.setDecimals(1) 
        self.spin.setSingleStep(1.0)
        self.spin.setFixedWidth(85)
        self.spin.setButtonSymbols(QDoubleSpinBox.NoButtons)
        self.spin.setAlignment(Qt.AlignCenter)
        
        # 3. 負方向按鈕 (◀)
        self.btn_minus = QPushButton("◀")
        self.btn_minus.setStyleSheet(styles.BTN_JOINT_MINUS_PLUS)
        self.btn_minus.setAutoRepeat(True)
        self.btn_minus.setAutoRepeatDelay(300)
        self.btn_minus.setAutoRepeatInterval(50)
        
        # 4. 滑桿 (自動延展填滿空間)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(min_val * 100), int(max_val * 100))
        self.slider.setFixedHeight(25)
        self.slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed) # 關鍵：讓滑桿變長
        
        # 5. 正方向按鈕 (▶)
        self.btn_plus = QPushButton("▶")
        self.btn_plus.setStyleSheet(styles.BTN_JOINT_MINUS_PLUS)
        self.btn_plus.setAutoRepeat(True)
        self.btn_plus.setAutoRepeatDelay(300)
        self.btn_plus.setAutoRepeatInterval(50)
        
        # 依序加入佈局
        self._layout.addWidget(self.lbl)
        self._layout.addWidget(self.spin)
        self._layout.addWidget(self.btn_minus)
        self._layout.addWidget(self.slider)
        self._layout.addWidget(self.btn_plus)
        
        # 內部訊號連接
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
        self.text_edit.setStyleSheet(styles.LOG_WINDOW_STYLE)
        layout.addWidget(self.text_edit)
        
        self.btn_clear = QPushButton(self)
        self.btn_clear.setCursor(Qt.PointingHandCursor)
        self.btn_clear.setIcon(qta.icon('fa5s.trash-alt', color='#7f8c8d'))
        self.btn_clear.setIconSize(QSize(22, 22))
        self.btn_clear.setFixedSize(32, 32)
        self.btn_clear.setToolTip("Clear Log")
        self.btn_clear.setStyleSheet(styles.BTN_CLEAR_STYLE)
        self.btn_clear.clicked.connect(self.text_edit.clear)

    # 自動處理按鈕定位 
    def resizeEvent(self, event):
        super().resizeEvent(event)
        margin = 15 # 考慮 scrollbar 寬度與邊距
        x = self.width() - self.btn_clear.width() - margin
        y = 5
        self.btn_clear.move(x, y)

    def append_log(self, text):
        self.text_edit.append(text)
        sb = self.text_edit.verticalScrollBar()
        sb.setValue(sb.maximum())

    def clear_log(self):
        self.text_edit.clear()

# 泛用型內聯編輯器 (支援阻擋列表預覽)
class InlineEdit(QLineEdit):
    def __init__(self, text, parent_row, save_cb, align=Qt.AlignLeft, prevent_select=False):
        super().__init__(text)
        self.parent_row = parent_row
        self.save_cb = save_cb
        self.align_mode = align
        self.prevent_select = prevent_select 
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
            if self.prevent_select:
                event.accept()  
            else:
                event.ignore()
                self.parent_row.force_list_selection(event) 
        else:
            super().mousePressEvent(event)

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
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

# 極簡下拉選單標籤 (共用邏輯：單擊彈出選單)
class DropdownLabel(QLabel):
    def __init__(self, parent_row, options, save_cb, align=Qt.AlignCenter):
        super().__init__()
        self.parent_row = parent_row
        self.options = options
        self.save_cb = save_cb
        self.setAlignment(align)
        self.setStyleSheet("color: black; background: transparent; font-weight: normal;")

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            event.accept() # 阻擋點擊事件穿透，不做 Previewing
            
            menu = QMenu(self)
            menu.setStyleSheet("""
                QMenu { background-color: white; border: 1px solid #c0c0c0; }
                QMenu::item { padding: 4px 15px; }
                QMenu::item:selected { background-color: #3498db; color: white; }
            """)
            for opt in self.options:
                action = menu.addAction(opt)
                action.triggered.connect(lambda checked, o=opt: self.save_cb(o))
            
            menu.exec_(self.mapToGlobal(event.pos()))

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
                color: #555555;               
                font-weight: normal;
                font-size: 24px;
                border: none;
                background: transparent;
            }
        """)
        
        layout = QHBoxLayout(self)
        # 左邊距與 Row 一致(5)，上下(6)，右邊距(5)
        layout.setContentsMargins(5, 6, 5, 6)
        layout.setSpacing(5)
        
        # 1. 序號佔位
        lbl_idx = QLabel("")
        lbl_idx.setFixedWidth(20) # 對齊下方的 lbl_index
        layout.addWidget(lbl_idx)
        
        # 2. 新增：類型 (Type)
        lbl_type = QLabel("")
        lbl_type.setFixedWidth(50) # 對齊下方的 type_lbl
        lbl_type.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_type)
        
        # 3. 名稱 (彈性伸展)
        lbl_name = QLabel(" Name")
        layout.addWidget(lbl_name)
        
        # layout.addStretch() # 名稱標籤會自己填滿剩下的空間
        
        # 4. 延遲
        lbl_delay = QLabel("Delay")
        lbl_delay.setFixedWidth(75) 
        lbl_delay.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_delay)
        
        # 5. 速度
        lbl_speed = QLabel("Speed")
        lbl_speed.setFixedWidth(75)
        lbl_speed.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_speed)
        
        # 6. 操作 (2個按鈕30px + 中間間距5px = 65px)
        lbl_action = QLabel("")
        lbl_action.setFixedWidth(65)
        lbl_action.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_action)
        
        # 7. Scrollbar 預留佔位區
        self.lbl_scrollbar = QLabel("")
        self.lbl_scrollbar.setFixedWidth(0) # 預設寬度為 0 (隱藏狀態)
        layout.addWidget(self.lbl_scrollbar)

    # 讓外部可以控制佔位區的寬度
    def set_scrollbar_width(self, width):
        self.lbl_scrollbar.setFixedWidth(width)

class WaypointRow(QWidget):
    def __init__(self, index, data, on_toggle_cb, on_delete_cb, parent=None):
        super().__init__(parent)
        self.data = data 
        self.index = index
        is_active = data.get('active', True)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(5) 
        
        # --- 1. 序號 ---
        self.lbl_index = QLabel(f"{index+1}.")
        self.lbl_index.setFixedWidth(20) 
        self.lbl_index.setStyleSheet("color: black; font-weight: normal;" if is_active else "color: gray;")
        layout.addWidget(self.lbl_index)
        
        # --- 2. 動作類型 PTP/LIN ---
        self.type_lbl = DropdownLabel(self, ["PTP", "LIN"], self.save_type)
        self.type_lbl.setFixedWidth(50) # 對齊標題的 "類型"
        self.update_type_display()
        layout.addWidget(self.type_lbl)
        
        # --- 3. 名稱區 ---
        name = data.get('name', f'Point {index+1}')
        self.edit_name = InlineEdit(name, self, self.save_new_name, Qt.AlignLeft, prevent_select=False)
        
        if not is_active:
            self.edit_name.setStyleSheet("background: transparent; border: none; padding: 0px; color: gray;")
            
        layout.addWidget(self.edit_name)
        layout.addStretch()

        # --- 4. 延遲 Delay (純文字，雙擊輸入，0秒留白) ---
        dw = QWidget()
        dw.setFixedWidth(75) 
        dl = QHBoxLayout(dw)
        dl.setContentsMargins(0, 0, 0, 0)
        dl.setSpacing(2)
        
        self.edit_delay = InlineEdit("", self, self.save_delay, Qt.AlignRight, prevent_select=True)
        dl.addWidget(self.edit_delay)
        dl.addStretch()
        layout.addWidget(dw)
        
        # 初始化顯示 Delay
        self.update_delay_display(data.get('delay', 0.0))

        # --- 5. 速度 Speed (單擊選單，50%留白) ---
        self.speed_lbl = DropdownLabel(self, ["1%", "10%", "50%", "100%"], self.save_speed, align=Qt.AlignRight)
        self.speed_lbl.setFixedWidth(75) 
        self.update_speed_display()
        layout.addWidget(self.speed_lbl)
        
        # --- 6. 操作按鈕 ---
        eye_icon = 'fa5s.eye' if is_active else 'fa5s.eye-slash'
        eye_color = '#34495e' if is_active else '#bdc3c7'
        
        btn_eye = QPushButton()
        btn_eye.setIcon(qta.icon(eye_icon, color=eye_color))
        btn_eye.setIconSize(QSize(20, 20))
        btn_eye.setFixedWidth(30)
        btn_eye.setStyleSheet("border: none; background: transparent;")
        btn_eye.clicked.connect(lambda: on_toggle_cb(index))
        layout.addWidget(btn_eye)

        btn_del = QPushButton()
        btn_del.setIcon(qta.icon('fa5s.trash-alt', color='black'))
        btn_del.setIconSize(QSize(20, 20))
        btn_del.setFixedWidth(30)
        btn_del.setStyleSheet("border: none; background: transparent;")
        btn_del.clicked.connect(lambda: on_delete_cb(index))
        layout.addWidget(btn_del)

    # --- 穿透選擇函式 ---
    def force_list_selection(self, event):
        list_widget = self.parentWidget()
        while list_widget is not None and not hasattr(list_widget, 'itemAt'):
            list_widget = list_widget.parentWidget()
        if list_widget:
            pos = list_widget.mapFromGlobal(event.globalPos())
            item = list_widget.itemAt(pos)
            if item:
                list_widget.setCurrentItem(item)
                list_widget.itemClicked.emit(item)

    # --- 資料更新邏輯 ---
    def save_type(self, new_type):
        self.data['type'] = new_type
        self.update_type_display()

    def update_type_display(self):
        move_type = self.data.get('type', 'PTP')
        is_active = self.data.get('active', True)
        
        color = "black" if is_active else "gray"
            
        self.type_lbl.setText(f"{move_type}") # 順便把 [ ] 中括號拿掉，看起來更乾淨
        self.type_lbl.setStyleSheet(f"color: {color}; font-weight: normal; background: transparent;")

    def save_new_name(self, new_name):
        if new_name.strip():
            self.data['name'] = new_name.strip()
        else:
            self.edit_name.setText(self.data.get('name', f'Point {self.index+1}'))

    def save_delay(self, text):
        clean_text = text.replace('s', '').strip()
        try:
            val = 0.0 if not clean_text else float(clean_text)
            self.data['delay'] = val
        except ValueError:
            pass 
        self.update_delay_display(self.data.get('delay', 0.0))

    def update_delay_display(self, delay_val):
        # 移除了對 delay_icon 的呼叫
        if delay_val == 0.0:
            self.edit_delay.setText("        ")
        else:
            self.edit_delay.setText(f"{delay_val}s")

    def save_speed(self, speed_str):
        val = float(speed_str.replace('%', ''))
        self.data['speed'] = val
        self.update_speed_display()

    def update_speed_display(self):
        spd = self.data.get('speed', 50.0)
        if spd == 50.0:
            self.speed_lbl.setText("        ") 
        else:
            self.speed_lbl.setText(f"{int(spd)}%")
            self.speed_lbl.setStyleSheet("color: black; background: transparent; font-weight: normal;")
    # --- 數值更新 ---
    def update_speed(self, val):
        self.data['speed'] = float(val)

    def update_delay(self, val):
        self.data['delay'] = float(val)

class WaypointListWidget(QListWidget):
    """自訂樣式的路徑點清單元件"""
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 初始化時就自動套用這些 CSS
        self.setStyleSheet("""
            QListWidget { 
                background-color: white; 
                border: 1px solid #c0c0c0;    
                border-top: none;             
                border-bottom-left-radius: 4px; 
                border-bottom-right-radius: 4px; 
                outline: 0;                   
            }
            QListWidget::item {
                border-bottom: 1px solid #f5f5f5; 
            }
            QListWidget::item:selected {
                background-color: #e3f2fd; 
                color: black;
            }
            
            QScrollBar:vertical {
                border: none;
                background: #f8f9fa; /* 超淡的灰白底色 */
                width: 12px;         /* 【關鍵】強制寬度變為 12px */
                margin: 0px 0px 0px 0px; 
            }
            
            /* 拖曳的那個把手 (Handle) */
            QScrollBar::handle:vertical {
                background: #bdc3c7; /* 質感灰 */
                min-height: 30px;    /* 把手最短長度 */
                border-radius: 6px;  /* 讓它變成圓潤的膠囊狀 */
            }
            QScrollBar::handle:vertical:hover {
                background: #95a5a6; /* 滑鼠移過去變深 */
            }
            
            /* 隱藏老氣的上下點擊箭頭 */
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px; 
                background: none;
            }
            
            /* 點擊軌道空白處的背景 */
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
                background: none;
            }
        """)