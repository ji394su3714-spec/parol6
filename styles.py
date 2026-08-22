from PySide6.QtGui import QFont

# ==========================================
# 字體設定
# ==========================================
FONT_TITLE = QFont("Segoe UI", 10, QFont.Weight.Bold)
FONT_TEXT = QFont("Segoe UI", 10)
FONT_CODE = QFont("Consolas", 10)
FONT_LOG = QFont("Consolas", 10)

# ==========================================
# 基礎排版與視窗樣式
# ==========================================
WINDOW_STYLE = """
    QMainWindow { background-color: #000000; }
    QSplitter { background-color: #000000; border: none; }
    QSplitter::handle { background-color: #000000; }
"""

TOPBAR_STYLE = """
    QFrame#TopBarFrame { background-color: #2b2b2b; border-bottom: 1px solid #111; }
    QLabel { color: #ffffff; }
    QPushButton { background: transparent; border: none; border-radius: 4px; }
    QPushButton:hover { background-color: rgba(255, 255, 255, 0.1); }
    QPushButton:pressed { background-color: rgba(0, 0, 0, 0.2); }
"""

BLOCK_STYLE = """
    QFrame#BlockFrame { background-color: #262626; border: none; border-radius: 6px; }
    QLabel { color: #ffffff; }
    QLabel#JointLabel { color: #aaa; }
    QTextEdit { background-color: transparent; color: #d4d4d4; border: none; }
"""

NAVBAR_STYLE = """
    QFrame#FloatingNavBar { background-color: rgba(30, 30, 30, 220); border: 1px solid #444; border-radius: 6px; }
"""
BTN_NAV_GHOST_STYLE = """
    QPushButton { background: transparent; border: none; border-radius: 4px; }
    QPushButton:hover { background-color: rgba(255, 255, 255, 0.15); }
    QPushButton:pressed { background-color: rgba(0, 0, 0, 0.3); }
"""

BTN_SPLIT_LEFT_STYLE = """
    QPushButton { 
        background: transparent; border: none; 
        border-top-left-radius: 4px; border-bottom-left-radius: 4px; 
        border-top-right-radius: 0px; border-bottom-right-radius: 0px; /* 強制右側直角 */
    }
    QPushButton[dim="true"] { background-color: rgba(255, 255, 255, 0.06); } /* 暗一階的連動底色 */
    QPushButton:hover { background-color: rgba(255, 255, 255, 0.15); }
    QPushButton:pressed { background-color: rgba(0, 0, 0, 0.3); }
"""

BTN_SPLIT_RIGHT_STYLE = """
    QPushButton { 
        background: transparent; border: none; 
        border-top-right-radius: 4px; border-bottom-right-radius: 4px; 
        border-top-left-radius: 0px; border-bottom-left-radius: 0px; /* 強制左側直角 */
        border-left: 1px solid rgba(255, 255, 255, 0.15); 
    }
    QPushButton[dim="true"] { background-color: rgba(255, 255, 255, 0.06); } /* 暗一階的連動底色 */
    QPushButton:hover { background-color: rgba(255, 255, 255, 0.15); }
    QPushButton:pressed { background-color: rgba(0, 0, 0, 0.3); }
"""

SEPARATOR_STYLE = "border-top: 1px solid #444;"

SPIN_STEP_STYLE = """
    QDoubleSpinBox {
        background-color: #1e1e1e;
        color: #d4d4d4;
        border: 1px solid #444;
        border-radius: 4px;
        padding: 0px; 
        font-family: 'Segoe UI';
        font-size: 12px;
    }
    QDoubleSpinBox:focus {
        border: 1px solid #444; /* 編輯時維持原本邊框，不觸發高亮 */
    }
"""
# ==========================================
# 編輯標籤 (Editable Label) 樣式
# ==========================================
EDITABLE_LABEL_MODE_STYLE = """
    QLineEdit { background: transparent; color: #E0E0E0; border: none; padding: 0px; }
"""
EDITABLE_EDITOR_MODE_STYLE = """
    QLineEdit {
        background-color: #2b2b2b; 
        color: #ffffff;
        border: none;       
        border-radius: 3px; 
        
        /* 關鍵：上/右/下/左 
           我們在底部 (Bottom) 強制墊上 1px 或 2px 的厚度，直接把文字往上頂！
           (你可以根據實際畫面在 1px~2px 之間微調，直到完全不跳動為止) */
        padding: 0px 0px 1px 0px; 
        
        font-family: 'Segoe UI';
    }
    QLineEdit:focus {
        background-color: #383838; 
        border: none;
    }
"""

TCP_EDITOR_STYLE = """
    QLineEdit {
        background-color: #1e1e1e;
        color: #d4d4d4;
        border: 1px solid #444;
        border-radius: 4px;
        padding: 0px 2px;
        font-family: 'Segoe UI';
    }
    /* 當取得焦點(編輯中)時，維持原本的暗色邊框 */
    QLineEdit:focus {
        border: 1px solid #555; 
        background-color: #2b2b2b; /* 編輯時背景稍微提亮一點點，增加層次感 */
    }
"""
# ==========================================
# 監視區 (Monitor) 樣式
# ==========================================
MONITOR_BOX_STYLE = """
    QFrame { background-color: #2b2b2b; border: 1px solid #404040; border-radius: 4px; }
"""
MONITOR_TITLE_STYLE = "color: #a0a0a0; font-family: Consolas, monospace; font-size: 11px; border: none; padding: 0px;"
MONITOR_VALUE_STYLE = "color: #00e6b8; font-family: Consolas, monospace; font-size: 11px; border: none; padding: 0px;"

MONITOR_INPUT_STYLE = """
    QLineEdit { 
        background: transparent; 
        border: none; 
        color: #00e6b8; 
        font-family: 'Consolas', monospace; 
        font-size: 11px; 
        padding: 0px; 
    }
    QLineEdit:focus { 
        color: #ffffff; 
        background-color: rgba(255, 255, 255, 0.12); 
        border-radius: 2px; 
    }
"""
BTN_GHOST_COPY_STYLE = """
    QPushButton { background: transparent; border: none; }
    QPushButton:hover { background-color: rgba(255, 255, 255, 0.08); border-radius: 4px; }
    QPushButton:pressed { background-color: rgba(0, 0, 0, 0.2); }
"""

# ==========================================
# 關節 Slider 區專用樣式
# ==========================================
BTN_JOG_SLIDER_STYLE = """
    QPushButton { 
        background-color: #4a4a4c; /* 比 Cartesian 的 #333 稍淺，帶有一點工業灰 */
        border: 1px solid #5a5a5c; 
        border-radius: 4px; 
    }
    QPushButton:hover { 
        background-color: #5c5c5e; 
        border-color: #777; 
    }
    QPushButton:pressed { 
        background-color: #2c2c2e; 
        margin: 1px 0 0 1px; /* 保留按鈕本身的實體下壓位移感 */
    }
"""

SLIDER_JOINT_STYLE = """
    QSlider { 
        min-height: 22px; /* 強制與旁邊的微動按鈕 (22x22) 等高，嚴防排版擠壓 */
        max-height: 22px;
    }
    
    QSlider::groove:horizontal { 
        border: none; 
        height: 4px; 
        background: #404040; 
        border-radius: 2px; 
    }
    
    QSlider::handle:horizontal { 
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #f0f0f0, stop:1 #999999); 
        border: 1px solid #333; 
        width: 12px;  /* 內部寬度 12 */
        height: 12px; /* 內部高度 12 */
        margin: -5px 0px;   
        border-radius: 7px; 
    }
    
    QSlider::handle:horizontal:hover { 
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffffff, stop:1 #b3b3b3); 
        border: 1px solid #111;
    }
    
    QSlider::handle:horizontal:pressed { 
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #888888, stop:1 #cccccc);
        border: 1px solid #222;
    }
"""

BTN_ACTION_STYLE = """
    QPushButton { background-color: #333; border: none; border-radius: 4px; color: #aaa; font-weight: bold; font-size: 16px; }
    QPushButton:hover { background-color: #555; color: white; }
    QPushButton:pressed { background-color: #111; margin: 1px 0 0 1px; } 
"""

BTN_GRIPPER_TOGGLE_STYLE = """
    QPushButton { background-color: #333; color: #888; border: 1px solid #444; border-radius: 4px; font-weight: bold; font-size: 8pt; }
    QPushButton:checked { background-color: #007acc; color: white; border: none; }
    QPushButton:hover { background-color: #555; color: white; }
    QPushButton:pressed { background-color: #222; margin: 1px 0 0 1px; }
"""

# ==========================================
# 夾爪 Gripper 區專用樣式
# ==========================================
SLIDER_GRIPPER_STYLE = """
    QSlider { 
        min-height: 22px; /* 縮小整體高度，讓上方的 0% 標籤貼得更近 */
        max-height: 22px;
    }
    
    QSlider::groove:horizontal { 
        border: none; 
        height: 4px; 
        background: #404040; 
        border-radius: 2px; 
    }
    
    /* 夾爪把手：金屬感圓角矩形 (混音器推桿風格) */
    QSlider::handle:horizontal { 
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #f0f0f0, stop:1 #999999); 
        border: 1px solid #333; 
        width: 10px;  /* 寬度稍微窄一點 */
        height: 16px; /* 高度拉長，呈現矩形 */
        margin: -7px 0px;   
        
        border-radius: 3px; /* 微小的圓角，保留矩形的俐落感 */
    }
    
    QSlider::handle:horizontal:hover { 
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffffff, stop:1 #b3b3b3); 
        border: 1px solid #111;
    }
    
    QSlider::handle:horizontal:pressed { 
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #888888, stop:1 #cccccc);
        border: 1px solid #222;
    }
"""

SPEED_CAPSULE_STYLE = "QFrame { background-color: #2c2c2e; border-radius: 11px; }"
SPEED_MINUS_STYLE = """
    QPushButton { background: transparent; border: none; color: #fff; font-size: 16px; font-weight: bold; border-top-left-radius: 11px; border-bottom-left-radius: 11px; }
    QPushButton:hover { background-color: rgba(255, 255, 255, 20); }
    QPushButton:pressed { background-color: rgba(255, 255, 255, 40); }
"""
SPEED_PLUS_STYLE = """
    QPushButton { background: transparent; border: none; color: #fff; font-size: 16px; font-weight: bold; border-top-right-radius: 11px; border-bottom-right-radius: 11px; }
    QPushButton:hover { background-color: rgba(255, 255, 255, 20); }
    QPushButton:pressed { background-color: rgba(255, 255, 255, 40); }
"""
SPEED_SEG_ON_STYLE = "background-color: #e5d5ff;"
SPEED_SEG_OFF_STYLE = "background-color: #4a4a4c;"

BTN_PRIMARY_STYLE = """
    QPushButton {
        background-color: #008f72; /* 深一點的青色底 */
        color: #ffffff;
        border: 1px solid #00e6b8; /* 亮青色邊框 */
        border-radius: 4px;
        padding: 5px 15px;
        font-family: 'Segoe UI', sans-serif;
        font-weight: bold;
        font-size: 12px;
    }
    QPushButton:hover {
        background-color: #00ab89;
        border: 1px solid #00ffcc;
    }
    QPushButton:pressed {
        background-color: #007059;
        padding-top: 6px; 
        padding-left: 16px;
    }
    QPushButton:disabled {
        background-color: #2b2b2b;
        color: #666666;
        border: 1px solid #333333;
    }
"""

BTN_SECONDARY_STYLE = """
    QPushButton {
        background-color: #444444;
        color: #dddddd;
        border: 1px solid #555555;
        border-radius: 4px;
        padding: 5px 15px;
        font-family: 'Segoe UI', sans-serif;
        font-weight: bold;
        font-size: 12px;
    }
    QPushButton:hover {
        background-color: #555555;
        border: 1px solid #777777;
        color: #ffffff;
    }
    QPushButton:pressed {
        background-color: #333333;
        border: 1px solid #444444;
        /* 按下時產生微小的視覺下壓感 */
        padding-top: 6px; 
        padding-left: 16px;
    }
    QPushButton:disabled {
        background-color: #2b2b2b;
        color: #666666;
        border: 1px solid #333333;
    }
"""

BTN_CARTESIAN_STYLE = """
    QPushButton { 
        /* 微弱漸層底色，保持沉穩 */
        background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #3E3E3E, stop: 1 #323232);
        /* 統一的 1px 深色細邊框，讓輪廓極度清晰銳利 */
        border: 1px solid #5a5a5c; 
        border-radius: 4px; 
        color: #E0E0E0; 
        font-size: 10px; 
        font-weight: bold; 
    }
    QPushButton:hover { 
        background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #525252, stop: 1 #464646);
        color: #FFFFFF; 
        border: 1px solid #2A2A2A; 
    }
    QPushButton:pressed { 
        /* 按下時直接變回純粹的暗色，取消漸層反轉 */
        background-color: #1A1A1A;
        border: 1px solid #000000;
        margin: 1px 0 0 1px;
    }
"""

BTN_FRAME_TOGGLE_STYLE = """
    QPushButton { background-color: #333; color: #00a8e6; border: 1px solid #00a8e6; border-radius: 4px; font-weight: bold; font-size: 11px; }
    QPushButton:hover { background-color: #444; }
    QPushButton:checked { color: #e6a800; border: 1px solid #e6a800; }
    QPushButton:checked:hover { background-color: #444; }
"""
BTN_STEP_TOGGLE_STYLE = """
    QPushButton { background-color: #333; border: 1px solid #444; border-radius: 4px; color: #ddd; font-size: 10px; font-weight: bold; }
    QPushButton:hover { background-color: #555; color: white; border-color: #666; }
    QPushButton:pressed { background-color: #111; margin: 1px 0 0 1px; }
    
    /* 當按鈕被切換為 Step (Checked 狀態) 時的變化 */
    QPushButton:checked { color: #00e6b8; border: 1px solid #00e6b8; }
    QPushButton:checked:hover { background-color: #444; }
"""

# ==========================================
# 路徑清單與分頁 (Waypoint Panel) 專用樣式
# ==========================================

# 1. 單行功能按鈕的 Hover 反饋
WAYPOINT_ROW_BTN_STYLE = """
    QPushButton { border: none; background: transparent; } 
    QPushButton:hover { background-color: #4a4a4a; border-radius: 4px; }
"""

# 2. 路徑清單 - 區隔 Hover 與 Selected
PATH_LIST_STYLE = """
    QListWidget { 
        background-color: transparent; 
        color: #d4d4d4; 
        border: none; 
        padding: 4px; 
        outline: none;
    }
    QListWidget::item {
        border: none;  
        padding: 2px 0px; 
    }
    /* 選中時：稍微提亮的暗灰底色 + 內斂的邊框 */
    QListWidget::item:selected { 
        background-color: #3a3d41; 
        border: 1px solid #555555;
        border-radius: 4px; 
    }
    /* 滑過但未選中時：最低調的微亮反饋 */
    QListWidget::item:hover:!selected {
        background-color: #2a2d2e; 
        border-radius: 4px;
    }
"""

# 3. 單行點位的共用字體設定
WAYPOINT_FONT_BASE = "font-family: 'Consolas', 'Courier New', monospace; font-size: 13px; background-color: transparent;"

# 4. 分頁標籤 (Editor Tab) 樣式
TAB_TITLE_STYLE = "background-color: transparent; border: none; font-family: 'Segoe UI', sans-serif; font-size: 13px;"

EDITOR_TAB_ACTIVE_STYLE = """
    EditorTab {
        background-color: #161616;
        border: none;
        border-right: 1px solid #2d2d2d;
    }
    QLabel { color: #ffffff; background-color: transparent; border: none; }
"""

EDITOR_TAB_INACTIVE_STYLE = """
    EditorTab {
        background-color: #2d2d2d;
        border: none;
        border-right: 1px solid #1e1e1e;
    }
    QLabel { color: #888888; background-color: transparent; border: none; }
"""

TAB_CLOSE_BTN_STYLE = """
    QPushButton { border: none; background: transparent; border-radius: 4px; } 
    QPushButton:hover { background-color: #555555; }
"""

TAB_NEW_BTN_STYLE = """
    QPushButton { border: none; background: transparent; border-radius: 4px; }
    QPushButton:hover { background-color: #3a3d41; }
"""

WAYPOINT_PANEL_BG_STYLE = "WaypointPanel { background-color: #161616; border-radius: 6px; }"

TAB_BAR_FRAME_STYLE = """
    QFrame {
        background-color: #252526; 
        border-top-left-radius: 6px; 
        border-top-right-radius: 6px;
    }
"""

WARNING_BTN_STYLE = """
    QPushButton { background-color: #b37700; border: 1px solid #d99000; color: #ffffff; } 
    QPushButton:hover { background-color: #cc8800; }
"""

# 3. 日誌控制台 (QTextEdit) 完美去背懸浮樣式
LOG_CONSOLE_STYLE = """
    QTextEdit { 
        background-color: transparent; 
        color: #aaaaaa; 
        border: none; 
        font-family: Consolas; 
        font-size: 11px; 
    }
"""

# ==========================================
# 功能選單樣式
# ==========================================
MENU_STYLE = """
    QMenu {
        background-color: #2b2b2b; /* 恢復實體深灰色 */
        color: #d4d4d4;
        border: 1px solid #444;
        border-radius: 6px;
        padding: 2px; /* 保持緊湊的外框 */
        font-family: "Segoe UI";
        font-size: 11px;
    }
    QMenu::item {
        padding: 4px 24px 4px 8px; /* 保持緊湊的選項高度 */
        border-radius: 4px;
        margin: 1px;
    }
    QMenu::item:selected {
        background-color: #404040; /* 低調的暗灰色 Hover */
        color: #ffffff; 
    }
    QMenu::separator {
        height: 1px;
        background: #444;
        margin: 2px 8px;
    }
"""
# ==========================================
# 獨立彈窗與對話框專用樣式
# ==========================================
PREFERENCES_DIALOG_STYLE = """
    QDialog { 
        background-color: #262626; 
        border: 1px solid #444; 
    }
    QLabel { 
        color: #d4d4d4; 
        font-family: "Segoe UI"; 
        font-size: 12px;
    }
    QCheckBox { 
        color: #d4d4d4; 
        spacing: 8px; 
        font-family: "Segoe UI";
        font-size: 12px;
    }
    QCheckBox::indicator { 
        width: 15px; height: 15px; 
        border-radius: 4px; 
        border: 1px solid #666; 
        background-color: #1e1e1e;
    }
    QCheckBox::indicator:checked { 
        background-color: #00e6b8; /* 科技綠 */
        border: 1px solid #00e6b8; 
    }
    QPushButton {
        background-color: #3a3a3a; 
        color: #e0e0e0;
        border: 1px solid #555; 
        border-radius: 4px; 
        padding: 5px 16px;
        font-family: "Segoe UI"; 
        font-size: 11px;
    }
    QPushButton:hover { 
        background-color: #4a4a4a; 
    }
    QPushButton#btn_apply { 
        background-color: #04395e; 
        border: 1px solid #007acc; 
        color: #ffffff; 
        font-weight: 500;
    }
    QPushButton#btn_apply:hover { 
        background-color: #054b7c; 
    }
"""

DARK_MESSAGE_BOX_STYLE = """
    QMessageBox {
        background-color: #262626;
        border: 1px solid #444;
    }
    QLabel {
        color: #d4d4d4;
        font-family: "Segoe UI";
        font-size: 13px;
    }
    QPushButton {
        background-color: #3a3a3a;
        color: #e0e0e0;
        border: 1px solid #555;
        border-radius: 4px;
        padding: 5px 16px;
        font-family: "Segoe UI";
        font-size: 12px;
        min-width: 70px;
        min-height: 24px;
    }
    QPushButton:hover {
        background-color: #4a4a4a;
    }
"""