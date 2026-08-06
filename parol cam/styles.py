# styles.py

# 全域純黑背景
STYLE_MAIN_WINDOW = """
    QMainWindow { background-color: #000000; }
    QWidget#CentralWidget { background-color: #000000; }
"""

# 統一色調的頂部分頁列
STYLE_TOP_BAR = """
    QWidget#TopBar { background-color: #1a1a1a; border-bottom: 1px solid #2a2a2a; }
"""

# 緊湊圓角浮島
STYLE_BLOCK = """
    QWidget#LeftPanel, QWidget#RightPanel { 
        background-color: #161616; 
        border-radius: 4px; 
        border: 1px solid #2a2a2a; 
    }
"""

STYLE_TAB_BAR = """
    QTabBar::tab { height: 35px; width: 100px; font-weight: bold; font-size: 14px; background: transparent; color: #777; margin-right: 2px; border: none; }
    QTabBar::tab:selected { color: white; border-bottom: 3px solid #007acc; } /* 換成 VS Code 經典藍 */
    QTabBar::tab:hover { color: white; background-color: #2a2a2a; }
"""

# VS Code 風格狀態列
STYLE_STATUS_BAR = """
    QStatusBar { background-color: #007acc; color: white; font-size: 12px; }
    QStatusBar::item { border: none; }
"""
STYLE_STATUS_BTN = """
    QPushButton { background-color: transparent; color: white; border: none; padding: 4px 10px; font-weight: bold; font-size: 12px; }
    QPushButton:hover { background-color: rgba(255, 255, 255, 50); }
    QPushButton:checked { background-color: rgba(0, 0, 0, 50); }
"""

STYLE_LEFT_PANEL = "background-color: transparent;" 
STYLE_SCROLL_AREA = "QScrollArea { background-color: transparent; border: none; }"

STYLE_COLLAPSIBLE_BTN = """
    QPushButton { 
        text-align: left; 
        padding: 6px 5px; 
        background-color: transparent; 
        border: none; 
        border-bottom: 1px solid #222; 
        
        font-weight: bold; 
        font-size: 14px; 
        color: #dddddd; 
    }
    QPushButton:hover { background-color: #252525; border-radius: 6px; }
    QPushButton:checked { color: #ffffff; border-bottom: 1px solid #007acc; }
"""

STYLE_BTN_NORMAL = "background-color: #3a3a3a; color: white; padding: 6px; border-radius: 4px; border: 1px solid #444;"
STYLE_BTN_PRIMARY = "background-color: #007acc; color: white; padding: 12px; font-weight: bold; border-radius: 4px;"
STYLE_BTN_SUCCESS = "background-color: #1c5234; color: white; padding: 12px; font-weight: bold; border-radius: 4px;"
STYLE_BTN_LOAD = "background-color: #5c3a21; color: white; padding: 8px; border-radius: 4px;"

STYLE_TOOLBAR = "QWidget#FloatingToolbar { background-color: #1a1a1a; border-radius: 4px; border: 1px solid #2a2a2a; }"

STYLE_TOOLBAR_BTN = """
    QPushButton { background-color: transparent; border-radius: 4px; border: none; } 
    QPushButton:hover { background-color: #2a2a2a; }
    QPushButton:pressed { background-color: #007acc; }
    QPushButton:checked { background-color: #007acc; }
"""

STYLE_LIST_WIDGET = """
    QListWidget { background-color: #111; border: 1px solid #2a2a2a; border-radius: 4px; color: white; padding: 4px;}
    QListWidget::item { padding: 6px; border-bottom: 1px solid #222; border-radius: 4px; }
    QListWidget::item:selected { background-color: #007acc; color: white; }
"""
STYLE_INNER_TAB = """
    QPushButton {
        background: transparent;
        color: #777;
        border: none;
        padding: 6px 12px;
        font-weight: bold;
        font-size: 13px;
        border-bottom: 2px solid transparent;
    }
    QPushButton:hover {
        color: #ccc;
    }
    QPushButton:checked {
        color: white;
        border-bottom: 2px solid #007acc;
    }
"""

PREFERENCES_DIALOG_STYLE = """
    QDialog { background-color: #1a1a1a; color: white; }
    QLabel { color: white; font-weight: bold; }
    QLineEdit, QDoubleSpinBox, QSpinBox {
        background-color: #333; color: white;
        border: 1px solid #555; padding: 4px; border-radius: 4px;
    }
    QListWidget {
        background-color: #111; color: white;
        border: 1px solid #333; border-radius: 4px;
    }
    QListWidget::item:selected { background-color: #007acc; }
    QPushButton {
        background-color: #3a3a3a; color: white;
        border: 1px solid #444; padding: 6px 12px; border-radius: 4px;
    }
    QPushButton:hover { background-color: #4a4a4a; }
    QPushButton:pressed { background-color: #007acc; }
"""

# 1. 實體校正懸浮面板 (半透明膠囊)
# 1. 實體校正懸浮面板 (半透明膠囊)
STYLE_TRANSFORM_PANEL = """
    QWidget#TransformContainer {
        background-color: rgba(22, 22, 22, 230); 
        border-radius: 6px; 
        border: 1px solid #333;
    }
"""

# 2. 底部軌跡播放條
STYLE_PLAYBACK_PANEL = """
    QWidget#FloatingPlayback {
        background-color: rgba(30, 30, 30, 220); 
        border-radius: 20px; 
        border: 1px solid #444;
    }
    QPushButton {
        background-color: #007acc; color: white; border: none; 
        border-radius: 12px; font-weight: bold; padding: 4px 16px; font-size: 13px;
    }
    QPushButton:hover { background-color: #0098ff; }
    QPushButton:disabled { background-color: #444; color: #888; }
    QLabel { color: #ddd; font-family: Consolas, monospace; font-weight: bold; font-size: 13px; }
"""

# 3. 系統日誌面板
STYLE_LOG_PANEL = """
    QTextEdit {
        background-color: rgba(20, 20, 20, 240); color: #00ff00; 
        font-family: Consolas, monospace; font-size: 12px; 
        border: 1px solid #333; border-radius: 6px; padding: 10px;
    }
"""

# 4. 工具列下拉式選單 (標準版 與 高亮版)
STYLE_TOOLBAR_COMBO = """
    QComboBox { background-color: #333; color: white; border-radius: 4px; padding: 2px 8px; font-weight: bold; border: 1px solid #555; }
    QComboBox::drop-down { border: none; }
"""
STYLE_TOOLBAR_COMBO_HIGHLIGHT = """
    QComboBox { background-color: #2c3e50; color: #ecf0f1; border-radius: 4px; padding: 2px 8px; font-weight: bold; border: 1px solid #34495e; }
    QComboBox::drop-down { border: none; }
"""