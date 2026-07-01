from PySide6.QtWidgets import QDialog, QVBoxLayout, QGridLayout, QLabel, QCheckBox, QHBoxLayout, QPushButton
from PySide6.QtCore import Qt

import styles
from widgets import app_settings, apply_windows_dark_titlebar

# ==========================================
# 偏好設定獨立彈窗 (Preferences Dialog)
# ==========================================
class PreferencesDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Preferences")
        self.setMinimumSize(480, 260)
        self.setStyleSheet(styles.MENU_STYLE + styles.PREFERENCES_DIALOG_STYLE)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 15)
        main_layout.setSpacing(20)

        grid_layout = QGridLayout()
        grid_layout.setVerticalSpacing(20)   
        grid_layout.setHorizontalSpacing(30) 

        lbl_interaction = QLabel("互動操作")
        lbl_interaction.setStyleSheet("font-weight: bold; color: #888888;")
        
        self.chk_sync = QCheckBox("啟用 3D 畫面與關節滑桿連動")
        self.chk_sync.setChecked(app_settings.get("sync_sliders"))
        
        grid_layout.addWidget(lbl_interaction, 0, 0, Qt.AlignmentFlag.AlignTop)
        grid_layout.addWidget(self.chk_sync, 0, 1)

        lbl_layout_title = QLabel("介面排版")
        lbl_layout_title.setStyleSheet("font-weight: bold; color: #888888;")
        
        self.chk_lock = QCheckBox("鎖定所有視窗分割器 (防止誤拉)")
        self.chk_lock.setChecked(app_settings.get("lock_splitters"))
        
        grid_layout.addWidget(lbl_layout_title, 1, 0, Qt.AlignmentFlag.AlignTop)
        grid_layout.addWidget(self.chk_lock, 1, 1)

        # --- 第 3 行：編輯器設定 ---
        lbl_editor = QLabel("編輯器設定")
        lbl_editor.setStyleSheet("font-weight: bold; color: #888888;")
        
        self.chk_comments = QCheckBox("在清單 (List) 模式中顯示註解與自訂代碼")
        self.chk_comments.setChecked(app_settings.get("show_comments"))
        
        self.chk_default_list = QCheckBox("新增腳本時，直接進入編譯預覽 (List 模式)")
        self.chk_default_list.setChecked(app_settings.get("default_list_mode"))
        
        editor_layout = QVBoxLayout()
        editor_layout.addWidget(self.chk_comments)
        editor_layout.addWidget(self.chk_default_list)
        
        grid_layout.addWidget(lbl_editor, 2, 0, Qt.AlignmentFlag.AlignTop)
        grid_layout.addLayout(editor_layout, 2, 1)

        grid_layout.setRowStretch(2, 1)
        grid_layout.setColumnStretch(0, 1)
        grid_layout.setColumnStretch(1, 3)
        
        main_layout.addLayout(grid_layout)

        btn_layout = QHBoxLayout()
        btn_layout.addStretch(1) 
        btn_layout.setSpacing(8)

        self.btn_cancel = QPushButton("取消")
        self.btn_cancel.setCursor(Qt.PointingHandCursor)
        self.btn_cancel.clicked.connect(self.reject) 

        self.btn_apply = QPushButton("套用")
        self.btn_apply.setObjectName("btn_apply")
        self.btn_apply.setCursor(Qt.PointingHandCursor)
        self.btn_apply.clicked.connect(self.save_and_accept) 

        btn_layout.addWidget(self.btn_cancel)
        btn_layout.addWidget(self.btn_apply)
        main_layout.addLayout(btn_layout)

    def save_and_accept(self):
        app_settings.set("sync_sliders", self.chk_sync.isChecked())
        app_settings.set("lock_splitters", self.chk_lock.isChecked())
        app_settings.set("show_comments", self.chk_comments.isChecked())       # 💥 儲存
        app_settings.set("default_list_mode", self.chk_default_list.isChecked()) # 💥 儲存
        self.accept()

    def showEvent(self, event):
        super().showEvent(event)
        apply_windows_dark_titlebar(self)