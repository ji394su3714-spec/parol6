# ui/styles.py
import os
import qtawesome as qta

def _get_icon_url(icon_name, filename, color='#2c3e50'):
    """處理圖示並轉為 CSS 可讀取的本地路徑"""
    icon_dir = os.path.join(os.path.dirname(__file__), 'icons')
    os.makedirs(icon_dir, exist_ok=True)
    file_path = os.path.join(icon_dir, filename)
    
    if not os.path.exists(file_path):
        qta.icon(icon_name, color=color).pixmap(32, 32).save(file_path, "PNG")
        
    clean_path = file_path.replace('\\', '/')
    return f"url('{clean_path}')"

def get_global_style():
    """回傳全域統一的 StyleSheet 字串"""
    up = _get_icon_url('fa5s.chevron-up', 'chevron_up.png')
    dn = _get_icon_url('fa5s.chevron-down', 'chevron_down.png')

    return f"""
        /* ==========================================
           1. 全域基礎字體與群組框 (Base & Layout)
           ========================================== */
        QWidget {{ font-size: 26px; font-family: "Segoe UI", "Microsoft JhengHei", Arial, sans-serif; }}
        QLabel {{ color: #333; }}
        
        QGroupBox {{ font-weight: bold; font-size: 22px; border: 2px solid #bdc3c7; border-radius: 6px; margin-top: 24px; }}
        QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 5px; }}

        /* ==========================================
           2. 輸入元件 (SpinBox & ComboBox)
           ========================================== */
        QDoubleSpinBox, QComboBox {{ 
            border: 2px solid #bdc3c7; border-radius: 6px; padding: 0px 3px;
            background-color: #fdfdfd; color: #2c3e50; 
        }}
        QDoubleSpinBox:focus, QComboBox:focus {{ border: 2px solid #27ae60; background-color: #ffffff; }}
        
        QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{ width: 25px; background-color: #ecf0f1; border-radius: 3px; margin: 2px; border: none; }}
        QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {{ background-color: #bdc3c7; }}
        QDoubleSpinBox::up-arrow {{ image: {up}; width: 16px; height: 16px; }}
        QDoubleSpinBox::down-arrow {{ image: {dn}; width: 16px; height: 16px; }}

        QComboBox::drop-down {{ subcontrol-origin: padding; subcontrol-position: top right; width: 25px; border: none; background-color: #ecf0f1; }}
        QComboBox::down-arrow {{ image: {dn}; width: 16px; height: 16px; }}

        /* ==========================================
           3. 頂部控制列區域 (Top Bar)
           ========================================== */
        QFrame#top_bar {{ background-color: #2c3e50; border-bottom: 4px solid #1a252f; }}
        QLabel#title_lbl {{ color: white; font-size: 28px; font-weight: bold; margin-right: 20px; border: none; }}
        QLabel#lbl_port {{ color: #ecf0f1; font-weight: bold; font-size: 16px; }}

        /* 連線區塊專屬下拉選單與按鈕 */
        QComboBox#combo_ports {{ font-size: 20px; padding: 5px; border-radius: 4px; background: white; }}
        QComboBox#combo_ports::drop-down {{ border: 0px; }}
        
        QPushButton#btn_refresh {{ background-color: #7f8c8d; border-radius: 4px; border: none; }}
        QPushButton#btn_refresh:hover {{ background-color: #95a5a6; }}
        QPushButton#btn_refresh:pressed {{ background-color: #5d6d7e; }}

        QPushButton#btn_connect {{ background-color: #27ae60; color: white; font-weight: bold; padding: 0px 15px; border-radius: 4px; font-size: 16px; border: none; }}
        QPushButton#btn_connect:hover {{ background-color: #2ecc71; }}
        QPushButton#btn_disconnect {{ background-color: #c0392b; color: white; font-weight: bold; padding: 0px 15px; border-radius: 4px; font-size: 16px; border: none; }}
        QPushButton#btn_disconnect:hover {{ background-color: #e74c3c; }}

        /* 頂部大按鈕共用 (利用 Class) */
        QPushButton[class="TopBtn"] {{ border-radius: 6px; padding: 0px 24px; font-size: 24px; font-weight: bold; color: white; border: none; }}
        QPushButton[class="TopBtn"]:pressed {{ background-color: #555; }}
        
        QPushButton#btn_stop {{ background-color: #c0392b; }} QPushButton#btn_stop:hover {{ background-color: #e74c3c; }}
        QPushButton#btn_home {{ background-color: #f39c12; }} QPushButton#btn_home:hover {{ background-color: #f1c40f; }}
        QPushButton#btn_homing {{ background-color: #8e44ad; }} QPushButton#btn_homing:hover {{ background-color: #9b59b6; }}
        
        QPushButton#btn_tool {{ border-radius: 6px; padding: 0px 20px; font-size: 22px; font-weight: bold; color: white; background-color: #34495e; border: none; }}
        QPushButton#btn_tool:hover {{ background-color: #4e6d8d; }} QPushButton#btn_tool:pressed {{ background-color: #2c3e50; }}

        /* RUN PATH 與 MENU 按鈕 */
        QPushButton#btn_run_main {{ background-color: #27ae60; color: white; border: none; font-weight: bold; font-size: 24px; border-top-left-radius: 6px; border-bottom-left-radius: 6px; padding: 0px 20px; }}
        QPushButton#btn_run_main:hover {{ background-color: #2ecc71; }} QPushButton#btn_run_main:pressed {{ background-color: #1e8449; }}
        
        QPushButton#btn_run_menu {{ background-color: #27ae60; color: white; border: none; border-top-right-radius: 6px; border-bottom-right-radius: 6px; }}
        QPushButton#btn_run_menu:hover {{ background-color: #2ecc71; }} QPushButton#btn_run_menu:pressed {{ background-color: #1e8449; }}

        QMenu#run_menu {{ background-color: #27ae60; color: white; font-size: 22px; font-weight: bold; border: 1px solid #1e8449; border-radius: 4px; }}
        QMenu#run_menu::item {{ padding: 8px 25px; background-color: transparent; }}
        QMenu#run_menu::item:selected {{ background-color: #1e8449; }}

        /* ==========================================
           4. 左側操作區 (Jog & Joint)
           ========================================== */
        QPushButton[class="JogBtn"] {{ font-weight: bold; font-size: 24px; background-color: #e0e0e0; border: 1px solid #999; border-radius: 5px; }}
        QPushButton[class="JogBtn"]:hover {{ background-color: #d5d5d5; }} QPushButton[class="JogBtn"]:pressed {{ background-color: #bbb; }}
        
        QPushButton[class="JointBtn"] {{ font-weight: bold; font-size: 32px; color: #2c3e50; background-color: #ecf0f1; border: 1px solid #bdc3c7; border-radius: 4px; }}
        QPushButton[class="JointBtn"]:hover {{ background-color: #bdc3c7; }} QPushButton[class="JointBtn"]:pressed {{ background-color: #95a5a6; }}

        /* 特殊小按鈕 (清單管理等) */
        QPushButton[class="IconOnlyBtn"] {{ border: none; background: transparent; }}
        QPushButton[class="IconOnlyBtn"]:hover {{ background-color: #ecf0f1; border-radius: 4px; }}
        QPushButton#btn_clear_log {{ background-color: transparent; border: none; border-radius: 4px; }}
        QPushButton#btn_clear_log:hover {{ background-color: rgba(231, 76, 60, 50); }}

        /* ==========================================
           5. Waypoints 路徑清單 (整合自 widgets.py)
           ========================================== */
        QListWidget {{ background-color: white; border: 1px solid #c0c0c0; border-top: none; border-bottom-left-radius: 4px; border-bottom-right-radius: 4px; outline: 0; }}
        QListWidget::item {{ border-bottom: 1px solid #f5f5f5; }}
        QListWidget::item:selected {{ background-color: #e3f2fd; color: black; }}
        
        QListWidget QScrollBar:vertical {{ border: none; background: #f8f9fa; width: 12px; margin: 0px; }}
        QListWidget QScrollBar::handle:vertical {{ background: #bdc3c7; min-height: 30px; border-radius: 6px; }}
        QListWidget QScrollBar::handle:vertical:hover {{ background: #95a5a6; }}
        QListWidget QScrollBar::add-line:vertical, QListWidget QScrollBar::sub-line:vertical {{ height: 0px; background: none; }}
        QListWidget QScrollBar::add-page:vertical, QListWidget QScrollBar::sub-page:vertical {{ background: none; }}

        /* ==========================================
           6. 系統日誌視窗 (System Log)
           ========================================== */
        QTextEdit#sys_log {{ background-color: #1e1e1e; color: #00ff00; font-family: Consolas, Monospace; font-size: 22px; padding: 10px; border: none; border-radius: 0px; }}
        QTextEdit#sys_log QScrollBar:vertical {{ border: none; background: #2b2b2b; width: 12px; margin: 0px; }}
        QTextEdit#sys_log QScrollBar::handle:vertical {{ background: #555; min-height: 20px; border-radius: 6px; }}
        QTextEdit#sys_log QScrollBar::handle:vertical:hover {{ background: #777; }}        
        QTextEdit#sys_log QScrollBar::add-line:vertical, QTextEdit#sys_log QScrollBar::sub-line:vertical {{ height: 0px; }}
        QTextEdit#sys_log QScrollBar::add-page:vertical, QTextEdit#sys_log QScrollBar::sub-page:vertical {{ background: none; }}
    """

# Joint Control 區域專用的滑桿樣式
JOINT_SLIDER_STYLE = """
    QSlider::groove:horizontal {
        border: 1px solid #bbb;
        background: #e0e0e0; /* 軌道底色 */
        height: 6px;
        border-radius: 0px; /* 軌道也改為直角 */
    }
    QSlider::sub-page:horizontal {
        background: #e0e0e0; /* 與軌道同色，抹除進度條感 */
        border: 1px solid #bbb;
        border-radius: 0px;
    }
    QSlider::handle:horizontal {
        background: #ffffff;
        border: 1px solid #777;
        width: 24px; /* 把手寬度 */
        margin: -10px 0; /* 把手上下凸出軌道，解決被砍頭的問題 */
        border-radius: 3px; /* 稍微有一點圓角，若要全方正改為 0px */
    }
    QSlider::handle:horizontal:hover {
        background: #f0f0f0;
        border: 1px solid #333;
    }
"""