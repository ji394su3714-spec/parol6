# styles.py

STYLE_TAB_BAR = """
    QTabBar::tab { height: 35px; width: 100px; font-weight: bold; font-size: 14px; background: #2b2b2b; color: #888; margin-right: 2px; border-top-left-radius: 4px; border-top-right-radius: 4px; }
    QTabBar::tab:selected { background-color: #444; color: white; border-bottom: 3px solid #b37700; }
    QTabBar::tab:hover { background-color: #3b3b3b; color: white; }
"""

STYLE_LEFT_PANEL = "background-color: #2b2b2b;"

STYLE_SCROLL_AREA = "QScrollArea { background-color: #2b2b2b; }"

STYLE_COLLAPSIBLE_BTN = """
    QPushButton { text-align: left; padding: 10px; background-color: #333333; border: none; border-bottom: 1px solid #222; font-weight: bold; font-size: 13px; color: #dddddd; }
    QPushButton:hover { background-color: #3e3e3e; }
    QPushButton:checked { color: #ffffff; border-bottom: 1px solid #b37700; }
"""

STYLE_BTN_NORMAL = "background-color: #444; color: white; padding: 6px; border-radius: 4px;"
STYLE_BTN_PRIMARY = "background-color: #b37700; color: white; padding: 12px; font-weight: bold; border-radius: 4px;"
STYLE_BTN_SUCCESS = "background-color: #1c5234; color: white; padding: 12px; font-weight: bold; border-radius: 4px;"
STYLE_BTN_LOAD = "background-color: #5c3a21; color: white; padding: 8px; border-radius: 4px;"

STYLE_TOOLBAR = "QWidget#FloatingToolbar { background-color: #222222; border-bottom: 1px solid #111111; border-left: 1px solid #111111; }"
STYLE_TOOLBAR_BTN = """
    QPushButton { background-color: transparent; border-radius: 4px; border: none; } 
    QPushButton:hover { background-color: #444444; }
    QPushButton:pressed { background-color: #b37700; }
    QPushButton:checked { background-color: #b37700; }
"""

STYLE_LIST_WIDGET = """
    QListWidget { background-color: #1a1a1a; border: 1px solid #333; border-radius: 4px; color: white; padding: 4px;}
    QListWidget::item { padding: 4px; border-bottom: 1px solid #333; }
    QListWidget::item:selected { background-color: #b37700; color: white; border-radius: 2px;}
"""