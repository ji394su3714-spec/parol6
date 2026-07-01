# tcp_dialog.py
from PySide6.QtWidgets import (QCheckBox, QDialog, QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem,
                               QPushButton, QLabel, QLineEdit, QGridLayout, QInputDialog, QMessageBox, QWidget)
from PySide6.QtCore import QSize, Qt
from PySide6.QtGui import QDoubleValidator
import qtawesome as qta
import styles
from widgets import apply_windows_dark_titlebar

# ==========================================
# 刀具清單專用的單行排版元件
# ==========================================
class ToolRowWidget(QWidget):
    # 👇 1. 參數新增 in_box (預設狀態) 與 on_toggle_cb (回呼函式)
    def __init__(self, index, name, in_box=False, on_toggle_cb=None, parent=None):
        super().__init__(parent)
        self.index = index
        self.on_toggle_cb = on_toggle_cb
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 0, 8, 0) 
        layout.setSpacing(10)

        # 1. 獨立的序號欄
        self.lbl_idx = QLabel(f"{index}.")
        self.lbl_idx.setFixedWidth(25)
        self.lbl_idx.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_idx.setStyleSheet("color: #888888; font-family: 'Consolas', monospace; font-size: 13px; background: transparent;")
        layout.addWidget(self.lbl_idx)

        # 2. 刀具名稱欄
        self.lbl_name = QLabel(name)
        self.lbl_name.setStyleSheet("color: #d4d4d4; font-family: 'Segoe UI', sans-serif; font-size: 13px; background: transparent;")
        layout.addWidget(self.lbl_name)
        
        # 加入彈性空間，把 Checkbox 推到最右邊
        layout.addStretch()

        # 👇 3. 建立 Checkbox (加入工具箱)
        self.chk_box = QCheckBox()
        self.chk_box.setChecked(in_box)
        self.chk_box.setToolTip("Add to Tool Box")
        # 讓 Checkbox 稍微大一點點，方便點擊
        self.chk_box.setStyleSheet("""
            QCheckBox::indicator { width: 16px; height: 16px; }
        """)
        # 綁定點擊事件
        self.chk_box.toggled.connect(self._handle_toggle)
        layout.addWidget(self.chk_box)

    # 👇 4. 點擊 Checkbox 時觸發回呼
    def _handle_toggle(self, checked):
        if self.on_toggle_cb:
            self.on_toggle_cb(self.index, checked)

class TCPManagerDialog(QDialog):
    def __init__(self, tcp_manager, parent=None):
        super().__init__(parent)
        self.tcp_manager = tcp_manager
        self.setWindowTitle("TCP / Tool Manager")
        self.setMinimumSize(450, 300)
        self.setStyleSheet(styles.PREFERENCES_DIALOG_STYLE)
        
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # --- 左側清單 ---
        left_layout = QVBoxLayout()
        left_layout.setSpacing(8)
        lbl_list = QLabel("Tool List")
        lbl_list.setFont(styles.FONT_TITLE)
        left_layout.addWidget(lbl_list)

        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet(styles.PATH_LIST_STYLE)
        self.list_widget.currentRowChanged.connect(self.on_tool_selected)
        left_layout.addWidget(self.list_widget)

        btn_layout = QHBoxLayout()
        self.btn_add = QPushButton(qta.icon('mdi.plus', color='#00e6b8'), "Add")
        self.btn_del = QPushButton(qta.icon('mdi.trash-can-outline', color='#e0e0e0'), "Del")
        self.btn_rename = QPushButton(qta.icon('mdi.form-textbox', color='#e0e0e0'), "Rename")
        self.btn_add.clicked.connect(self.add_tool)
        self.btn_del.clicked.connect(self.delete_tool)
        self.btn_rename.clicked.connect(self.rename_tool)
        btn_layout.addWidget(self.btn_add)
        btn_layout.addWidget(self.btn_del)
        btn_layout.addWidget(self.btn_rename)
        left_layout.addLayout(btn_layout)
        main_layout.addLayout(left_layout, 1)

        # --- 右側數值 ---
        right_layout = QVBoxLayout()
        right_layout.setSpacing(15)
        lbl_edit = QLabel("TCP Offset Parameters")
        lbl_edit.setFont(styles.FONT_TITLE)
        right_layout.addWidget(lbl_edit)

        grid = QGridLayout()
        grid.setSpacing(10)
        self.inputs = {}
        labels = ["X (mm)", "Y (mm)", "Z (mm)", "Rx (deg)", "Ry (deg)", "Rz (deg)"]
        keys = ["x", "y", "z", "rx", "ry", "rz"]
        validator = QDoubleValidator(-5000.0, 5000.0, 3, self)
        
        for i, (label_text, key) in enumerate(zip(labels, keys)):
            row, col = divmod(i, 2)
            lbl = QLabel(label_text)
            line_edit = QLineEdit("0.000")
            line_edit.setValidator(validator)
            line_edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            line_edit.setStyleSheet(styles.TCP_EDITOR_STYLE)
            line_edit.setFixedHeight(24)
            line_edit.textChanged.connect(self.on_value_changed)
            grid.addWidget(lbl, row*2, col)
            grid.addWidget(line_edit, row*2+1, col)
            self.inputs[key] = line_edit

        right_layout.addLayout(grid)
        right_layout.addStretch(1)

        self.btn_apply = QPushButton("Apply & Close")
        self.btn_apply.setFixedHeight(30)
        self.btn_apply.clicked.connect(self.apply_and_close)
        right_layout.addWidget(self.btn_apply)
        main_layout.addLayout(right_layout, 1)

        self._is_loading = False
        self.refresh_list()

    def refresh_list(self):
        self.list_widget.clear()
        
        for i, tool in enumerate(self.tcp_manager.tools):
            item = QListWidgetItem()
            
            # 👇 安全讀取 in_box 狀態 (如果舊資料沒有這個 key，預設為 False)
            in_box = tool.get("in_box", False)
            
            # 👇 將狀態與新的回呼函式傳進去
            row_widget = ToolRowWidget(i, tool['name'], in_box, self.on_toolbox_toggled)
            
            item.setSizeHint(QSize(0, 30))
            
            self.list_widget.addItem(item)
            self.list_widget.setItemWidget(item, row_widget)
            
        if len(self.tcp_manager.tools) > 0:
            self.list_widget.setCurrentRow(self.tcp_manager.current_index)

    def on_toolbox_toggled(self, index, checked):
        """當清單中的 Checkbox 被點擊時，更新對應工具的 in_box 狀態"""
        if 0 <= index < len(self.tcp_manager.tools):
            self.tcp_manager.tools[index]["in_box"] = checked

    def on_tool_selected(self, index):
        if self._is_loading or index < 0: return
        self.tcp_manager.set_current_index(index)
        vals = self.tcp_manager.get_active_tool_data()["values"]
        self._is_loading = True
        for k, v in zip(["x", "y", "z", "rx", "ry", "rz"], vals):
            self.inputs[k].setText(f"{v:.3f}")
        self._is_loading = False

    def on_value_changed(self):
        if self._is_loading: return
        index = self.list_widget.currentRow()
        if index < 0: return
        try:
            new_vals = [float(self.inputs[k].text() or 0.0) for k in ["x", "y", "z", "rx", "ry", "rz"]]
            self.tcp_manager.update_tool_values(index, new_vals)
        except ValueError: 
            pass

    def add_tool(self):
        self.tcp_manager.add_tool(f"Tool {len(self.tcp_manager.get_tools()) + 1}")
        self.refresh_list()

    def delete_tool(self):
        if len(self.tcp_manager.get_tools()) <= 1:
            QMessageBox.warning(self, "Warning", "Cannot delete the last TCP tool.")
            return
        self.tcp_manager.delete_tool(self.list_widget.currentRow())
        self.refresh_list()

    def rename_tool(self):
        idx = self.list_widget.currentRow()
        if idx < 0: return
        old_name = self.tcp_manager.get_tools()[idx]["name"]
        new_name, ok = QInputDialog.getText(self, "Rename Tool", "New name:", QLineEdit.EchoMode.Normal, old_name)
        if ok and new_name.strip():
            self.tcp_manager.rename_tool(idx, new_name.strip())
            self.refresh_list()

    def apply_and_close(self):
        self.tcp_manager.save_config()
        self.accept()

    def showEvent(self, event):
        super().showEvent(event)
        apply_windows_dark_titlebar(self)