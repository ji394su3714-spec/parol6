# ui/styles.py
import os
import qtawesome as qta
from PyQt5.QtGui import QPixmap

# 定義顏色常數
COLOR_BG_DARK = "#1e1e1e"
COLOR_TEXT_GREEN = "#00ff00"
COLOR_TEXT_PRIMARY = "#2c3e50"
COLOR_BORDER = "#bdc3c7"
COLOR_ACCENT = "#27ae60"
COLOR_RED = "#c0392b"
COLOR_ORANGE = "#f39c12"
COLOR_PURPLE = "#8e44ad"

# --- 自動生成圖示 Helper (維持不動) ---
def _get_file_icon_url(icon_name, filename, color='#2c3e50'):
    base_dir = os.path.dirname(__file__) 
    icon_dir = os.path.join(base_dir, 'icons')
    if not os.path.exists(icon_dir): os.makedirs(icon_dir)
    file_path = os.path.join(icon_dir, filename)
    if not os.path.exists(file_path):
        icon = qta.icon(icon_name, color=color)
        icon.pixmap(32, 32).save(file_path, "PNG")
    clean_path = file_path.replace('\\', '/')
    return f"url('{clean_path}')"

# --- 樣式定義區 ---

def get_global_style():
    url_up = _get_file_icon_url('fa5s.chevron-up', 'chevron_up.png')
    url_down = _get_file_icon_url('fa5s.chevron-down', 'chevron_down.png')

    return f"""
        QWidget {{ font-size: 26px; font-family: "Segoe UI", "Microsoft JhengHei", Arial, sans-serif; }}
        
        QGroupBox {{ font-weight: bold; font-size: 22px; border: 2px solid #bdc3c7; border-radius: 6px; margin-top: 24px; }}
        QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0px 5px; }}
        
        QLabel {{ color: #333; }}
        
        /* 輸入框 */
        QDoubleSpinBox {{
            border: 2px solid #bdc3c7; border-radius: 6px; padding: 0px 3px;
            background-color: #fdfdfd; color: #2c3e50;
        }}
        QDoubleSpinBox:focus {{ border: 2px solid {COLOR_ACCENT}; background-color: #ffffff; }}
        
        QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
            width: 25px; background-color: #ecf0f1; border: none; border-radius: 3px; margin: 2px;
        }}
        QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {{ background-color: #bdc3c7; }}
        QDoubleSpinBox::up-arrow {{ image: {url_up}; width: 16px; height: 16px; }}
        QDoubleSpinBox::down-arrow {{ image: {url_down}; width: 16px; height: 16px; }}

        /* 下拉選單 */
        QComboBox {{
            border: 2px solid #bdc3c7; border-radius: 6px; padding: 0px 3px;
            background-color: #fdfdfd; color: #2c3e50;
        }}
        QComboBox:focus {{ border: 2px solid {COLOR_ACCENT}; background-color: #ffffff; }}
        QComboBox::drop-down {{
            subcontrol-origin: padding; subcontrol-position: top right;
            width: 25px; border: none; background-color: #ecf0f1; 
        }}
        QComboBox::down-arrow {{ image: {url_down}; width: 16px; height: 16px; }}
    """

# --- JOG 區塊樣式 ---
BTN_JOG_STYLE = """
    QPushButton { 
        font-weight: bold; font-size: 24px; 
        background-color: #e0e0e0; border: 1px solid #999; border-radius: 5px; 
        min-width: 125px; max-width: 125px;
        min-height: 50px; max-height: 50px;
    }
    QPushButton:hover { background-color: #d5d5d5; }
    QPushButton:pressed { background-color: #bbb; }
"""

BTN_UNDO_STYLE = """
    QPushButton { border: 1px solid #bdc3c7; border-radius: 4px; background-color: #ecf0f1; }
    QPushButton:hover { background-color: #d5d5d5; }
"""

# --- Top Bar 相關樣式 ---
TOP_BAR_STYLE = "background-color: #2c3e50; border-bottom: 4px solid #1a252f;"
TITLE_LABEL_STYLE = "color: white; font-size: 28px; font-weight: bold; margin-right: 20px; border: none;"

BTN_CTRL_BASE = "QPushButton { border-radius: 6px; padding: 0px 24px; font-size: 24px; font-weight: bold; color: white; border: none; } QPushButton:pressed { background-color: #555; }"
BTN_STOP_STYLE = BTN_CTRL_BASE + f"QPushButton {{ background-color: {COLOR_RED}; }} QPushButton:hover {{ background-color: #e74c3c; }}"
BTN_HOME_STYLE = BTN_CTRL_BASE + f"QPushButton {{ background-color: {COLOR_ORANGE}; }} QPushButton:hover {{ background-color: #f1c40f; }}"
BTN_HOMING_STYLE = BTN_CTRL_BASE + f"QPushButton {{ background-color: {COLOR_PURPLE}; }} QPushButton:hover {{ background-color: #9b59b6; }}"

BTN_TOOL_STYLE = """
    QPushButton { 
        border-radius: 6px; padding: 0px 20px; font-size: 22px; font-weight: bold; 
        color: white; background-color: #34495e; border: none; 
    }
    QPushButton:hover { background-color: #4e6d8d; }
    QPushButton:pressed { background-color: #2c3e50; }
"""
# --- Run Buttons ---
BTN_RUN_MAIN_STYLE = f"""
    QPushButton {{ 
        background-color: {COLOR_ACCENT}; color: white; border: none; font-weight: bold; font-size: 24px;
        border-top-left-radius: 6px; border-bottom-left-radius: 6px; padding: 0px 20px;
    }}
    QPushButton:hover {{ background-color: #2ecc71; }}
    QPushButton:pressed {{ background-color: #1e8449; }}
"""
BTN_RUN_MENU_STYLE = f"""
    QPushButton {{ 
        background-color: {COLOR_ACCENT}; color: white; border: none;
        border-top-right-radius: 6px; border-bottom-right-radius: 6px;
    }}
    QPushButton:hover {{ background-color: #2ecc71; }}
    QPushButton:pressed {{ background-color: #1e8449; }}
"""
MENU_STYLE = f"""
    QMenu {{ background-color: {COLOR_ACCENT}; color: white; font-size: 22px; font-weight: bold;
            border: 1px solid #1e8449; border-radius: 4px; }}
    QMenu::item {{ padding: 8px 25px; background-color: transparent;}}
    QMenu::item:selected {{background-color: #1e8449; }}
"""

# --- Waypoint Manager 樣式 ---
WAYPOINT_TEXT_ACTIVE = "font-size: 22px; font-weight: bold; color: #2c3e50;"
WAYPOINT_TEXT_INACTIVE = "font-size: 22px; font-style: italic; color: #95a5a6;"
WAYPOINT_DELAY_TEXT = "color: #34495e; font-size: 20px; font-weight: bold;"

BTN_ICON_ONLY_STYLE = "QPushButton { border: none; background: transparent; } QPushButton:hover { background-color: #ecf0f1; border-radius: 4px; }"

# --- Log 樣式 ---
LOG_WINDOW_STYLE = f"""
    QTextEdit {{
        background-color: {COLOR_BG_DARK}; color: {COLOR_TEXT_GREEN}; 
        font-family: Consolas, Monospace; font-size: 22px; padding: 10px; border: none; border-radius: 4px;
    }}
    QScrollBar:vertical {{ border: none; background: #2b2b2b; width: 12px; margin: 0px;}}
    QScrollBar::handle:vertical {{ background: #555; min-height: 20px; border-radius: 6px; }}
    QScrollBar::handle:vertical:hover {{ background: #777; }}        
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0px; subcontrol-position: bottom; subcontrol-origin: margin; }}
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background: none; }}
"""
BTN_CLEAR_STYLE = "QPushButton {background-color: transparent; border: none; border-radius: 4px; } QPushButton:hover {background-color: rgba(231, 76, 60, 50); }"