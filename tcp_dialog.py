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
    def __init__(self, index, name, in_box=False, on_toggle_cb=None, parent=None):
        super().__init__(parent)
        self.index = index
        self.on_toggle_cb = on_toggle_cb
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 0, 8, 0) 
        layout.setSpacing(10)

        self.lbl_idx = QLabel(f"{index}.")
        self.lbl_idx.setFixedWidth(25)
        self.lbl_idx.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_idx.setStyleSheet("color: #888888; font-family: 'Consolas', monospace; font-size: 13px; background: transparent;")
        layout.addWidget(self.lbl_idx)

        self.lbl_name = QLabel(name)
        self.lbl_name.setStyleSheet("color: #d4d4d4; font-family: 'Segoe UI', sans-serif; font-size: 13px; background: transparent;")
        layout.addWidget(self.lbl_name)
        
        layout.addStretch()

        self.chk_box = QCheckBox()
        self.chk_box.setChecked(in_box)
        self.chk_box.setToolTip("Add to Tool Box")
        self.chk_box.setStyleSheet("""
            QCheckBox::indicator { width: 16px; height: 16px; }
        """)
        self.chk_box.toggled.connect(self._handle_toggle)
        layout.addWidget(self.chk_box)

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
        self.setWindowIcon(qta.icon('mdi.tools'))
        
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
        # 初始開啟時，讓選擇框停留在系統當前生效的刀具
        self.refresh_list(target_idx=self.tcp_manager.current_index)

    def refresh_list(self, target_idx=None):
        """刷新清單，並加入防抖動與選取記憶"""
        current_idx = self.list_widget.currentRow() if target_idx is None else target_idx
        if current_idx < 0: 
            current_idx = 0

        # 阻斷訊號：在填入資料時，不要觸發 currentRowChanged
        self.list_widget.blockSignals(True)
        self.list_widget.clear()
        
        for i, tool in enumerate(self.tcp_manager.tools):
            item = QListWidgetItem()
            in_box = tool.get("in_box", True) 
            
            row_widget = ToolRowWidget(i, tool['name'], in_box, self.on_toolbox_toggled)
            item.setSizeHint(QSize(0, 30))
            
            self.list_widget.addItem(item)
            self.list_widget.setItemWidget(item, row_widget)
            
        if len(self.tcp_manager.tools) > 0:
            if current_idx >= len(self.tcp_manager.tools):
                current_idx = len(self.tcp_manager.tools) - 1
            self.list_widget.setCurrentRow(current_idx)
            
        self.list_widget.blockSignals(False)
        # 刷新完畢後，手動觸發一次選取更新右側數值
        self.on_tool_selected(current_idx)

    def on_toolbox_toggled(self, index, checked):
        """僅更新字典的暫存狀態，不發射訊號也不存檔"""
        if 0 <= index < len(self.tcp_manager.tools):
            self.tcp_manager.tools[index]["in_box"] = checked

    def on_tool_selected(self, index):
        if self._is_loading or index < 0: return
        
        # 💡 致命錯誤修正：絕對不要在這裡呼叫 self.tcp_manager.set_current_index(index)
        # 編輯對話框的點擊，不應該干涉系統大腦的狀態！
        
        # 我們只需單純「讀取」陣列即可
        tool_data = self.tcp_manager.tools[index]
        vals = tool_data.get("values", [0.0]*6)
        
        self._is_loading = True
        for k, v in zip(["x", "y", "z", "rx", "ry", "rz"], vals):
            self.inputs[k].setText(f"{v:.3f}")
        self._is_loading = False

    def add_tool(self):
        self.tcp_manager.add_tool(f"Tool {len(self.tcp_manager.get_tools()) + 1}")
        # 新增後直接選中最後一個 (最新) 的工具
        self.refresh_list(target_idx=len(self.tcp_manager.tools) - 1)

    def delete_tool(self):
        if len(self.tcp_manager.get_tools()) <= 1:
            QMessageBox.warning(self, "Warning", "Cannot delete the last TCP tool.")
            return
        self.tcp_manager.delete_tool(self.list_widget.currentRow())
        self.refresh_list()

    def rename_tool(self):
        """僅修改暫存名稱並重繪 List，保持選取狀態不動"""
        idx = self.list_widget.currentRow()
        if idx < 0: return
        old_name = self.tcp_manager.get_tools()[idx]["name"]
        new_name, ok = QInputDialog.getText(self, "Rename Tool", "New name:", QLineEdit.EchoMode.Normal, old_name)
        if ok and new_name.strip():
            self.tcp_manager.tools[idx]["name"] = new_name.strip()
            self.refresh_list(target_idx=idx)

    def apply_and_close(self):
        """將畫面上當前選中的工具輸入框數值打包，透過 update_tool 寫入大腦"""
        idx = self.list_widget.currentRow()
        if idx >= 0:
            try:
                new_vals = [float(self.inputs[k].text() or 0.0) for k in ["x", "y", "z", "rx", "ry", "rz"]]
                
                # 抓取這把刀具目前的名稱跟 in_box 狀態
                current_tool = self.tcp_manager.tools[idx]
                name = current_tool.get("name", f"Tool {idx}")
                in_box = current_tool.get("in_box", True)
                
                # 呼叫 tcp_manager 的唯一更新入口！
                self.tcp_manager.update_tool(idx, name, new_vals, in_box)
            except ValueError:
                pass
                
        self.accept()

    def showEvent(self, event):
        super().showEvent(event)
        apply_windows_dark_titlebar(self)