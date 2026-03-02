# ui/widgets.py
from PyQt5.QtWidgets import QPushButton, QFrame, QHBoxLayout, QLabel, QWidget
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QRegion
import qtawesome as qta
from ui import styles

# --- 圓形按鈕 ---
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

# --- Monitor Frame ---
class MonitorFrame(QFrame):
    def __init__(self, label_text, value_widget, parent=None):
        super().__init__(parent)
        self.setFixedHeight(55)
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

# --- Waypoint Row Item ---
class WaypointRow(QWidget):
    def __init__(self, index, data, on_toggle_cb, on_delete_cb, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(10)
        
        name = data.get('name', f'Point {index+1}')
        delay = data.get('delay', 0.0)
        is_active = data.get('active', True)
        move_type = data.get('type', 'PTP')
        
        # 顯示名稱與類型
        type_color = "#e67e22" if move_type == "LIN" else "#2980b9"
        display_text = f"{index+1}. <font color='{type_color}'>[{move_type}]</font> {name}"
        if not is_active: display_text += " (Skip)"
        
        lbl_name = QLabel(display_text)
        lbl_name.setStyleSheet(styles.WAYPOINT_TEXT_ACTIVE if is_active else styles.WAYPOINT_TEXT_INACTIVE)
        layout.addWidget(lbl_name)

        # Delay 圖示與文字
        if delay > 0:
            dw = QWidget()
            dl = QHBoxLayout(dw)
            dl.setContentsMargins(5, 0, 0, 0); dl.setSpacing(2)
            icon = QPushButton()
            icon.setIcon(qta.icon('fa5s.hourglass-half', color='#34495e'))
            icon.setIconSize(QSize(22, 22)); icon.setFlat(True)
            icon.setStyleSheet("border: none; background: transparent;")
            val = QLabel(f"{delay}s")
            val.setStyleSheet(styles.WAYPOINT_DELAY_TEXT)
            dl.addWidget(icon); dl.addWidget(val)
            layout.addWidget(dw)

        layout.addStretch()
        
        # 功能按鈕 (Eye / Trash)
        eye_icon = 'fa5s.eye' if is_active else 'fa5s.eye-slash'
        eye_color = '#34495e' if is_active else '#bdc3c7'
        
        btn_eye = QPushButton()
        btn_eye.setIcon(qta.icon(eye_icon, color=eye_color))
        btn_eye.setIconSize(QSize(22, 22))
        btn_eye.setFixedWidth(40)
        btn_eye.setStyleSheet(styles.BTN_ICON_ONLY_STYLE)
        btn_eye.clicked.connect(lambda: on_toggle_cb(index))
        layout.addWidget(btn_eye)

        btn_del = QPushButton()
        btn_del.setIcon(qta.icon('fa5s.trash-alt', color='black'))
        btn_del.setIconSize(QSize(22, 22))
        btn_del.setFixedWidth(40)
        btn_del.setStyleSheet(styles.BTN_ICON_ONLY_STYLE)
        btn_del.clicked.connect(lambda: on_delete_cb(index))
        layout.addWidget(btn_del)