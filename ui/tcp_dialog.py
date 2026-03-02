# ui/tcp_dialog.py
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QListWidget, 
                             QListWidgetItem, QGroupBox, QFormLayout, QDoubleSpinBox, 
                             QLabel, QPushButton, QInputDialog, QMessageBox, QWidget, QDialogButtonBox)
from PyQt5.QtCore import Qt

class TCPSettingsDialog(QDialog):
    def __init__(self, tcp_manager, parent=None):
        super().__init__(parent)
        self.manager = tcp_manager # 接收 Manager 實例
        self.setWindowTitle("Tool Center Point (TCP) Manager")
        self.resize(700, 420)
        self.setStyleSheet("""
            QDialog { background-color: #ecf0f1; font-family: "Segoe UI"; }
            QListWidget { border: 1px solid #bdc3c7; border-radius: 4px; font-size: 18px; }
            QListWidget::item { padding: 8px; }
            QListWidget::item:selected { background-color: #2980b9; color: white; }
            QGroupBox { font-weight: bold; border: 1px solid #bdc3c7; border-radius: 6px; margin-top: 24px; padding-top: 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
        """)

        self.setup_ui()
        self.load_data()

    def setup_ui(self):
        main_layout = QHBoxLayout(self)

        # --- 左側：工具列表 ---
        left_layout = QVBoxLayout()
        
        lbl_list = QLabel("Available Tools:")
        left_layout.addWidget(lbl_list)

        self.tool_list = QListWidget()
        self.tool_list.currentRowChanged.connect(self.on_tool_selected)
        self.tool_list.itemDoubleClicked.connect(self.rename_current_tool)
        left_layout.addWidget(self.tool_list)

        # 列表操作按鈕
        btn_layout = QHBoxLayout()
        btn_add = QPushButton("+ Add")
        btn_del = QPushButton("- Del")
        btn_add.clicked.connect(self.add_tool)
        btn_del.clicked.connect(self.del_tool)
        
        btn_layout.addWidget(btn_add)
        btn_layout.addWidget(btn_del)
        left_layout.addLayout(btn_layout)
        
        main_layout.addLayout(left_layout, 1) # 左側佔 1 等份

        # --- 右側：參數設定 ---
        right_layout = QVBoxLayout()
        
        group = QGroupBox("Geometry Offset")
        form = QFormLayout()
        form.setSpacing(10)
        
        self.inputs = []
        labels = ["X (mm)", "Y (mm)", "Z (mm)", "Rx (  °)", "Ry (  °)", "Rz (  °)"]
        
        for label in labels:
            spin = QDoubleSpinBox()
            spin.setRange(-1000.0, 1000.0)
            spin.setSingleStep(1.0)
            spin.setDecimals(2)
            # 數值改變時，暫存到記憶體
            spin.valueChanged.connect(self.save_temp_values)
            form.addRow(QLabel(label), spin)
            self.inputs.append(spin)
            
        group.setLayout(form)
        right_layout.addWidget(group)
        
        # 底部按鈕
        right_layout.addStretch()
        btn_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btn_box.accepted.connect(self.accept) # 按 OK 才真正存檔並關閉
        btn_box.rejected.connect(self.reject)
        right_layout.addWidget(btn_box)

        main_layout.addLayout(right_layout, 1) # 右側佔 2 等份

    def load_data(self):
        self.tool_list.blockSignals(True)
        self.tool_list.clear()
        
        tools = self.manager.get_tools()
        for t in tools:
            self.tool_list.addItem(t["name"])
            
        self.tool_list.setCurrentRow(self.manager.current_index)
        self.tool_list.blockSignals(False)
        self.update_inputs()

    def on_tool_selected(self, row):
        if row < 0: return
        self.manager.set_current_index(row)
        self.update_inputs()

    def update_inputs(self):
        # 從 Manager 讀取數值填入 UI
        data = self.manager.get_active_tool_data()
        values = data["values"]
        
        for i, spin in enumerate(self.inputs):
            spin.blockSignals(True)
            spin.setValue(values[i])
            spin.blockSignals(False)

    def save_temp_values(self):
        # 將 UI 數值寫回 Manager (暫存)
        idx = self.tool_list.currentRow()
        if idx < 0: return
        
        new_vals = [spin.value() for spin in self.inputs]
        self.manager.update_tool_values(idx, new_vals)

    def add_tool(self):
        text, ok = QInputDialog.getText(self, "New Tool", "Tool Name:")
        if ok and text:
            self.manager.add_tool(text)
            self.load_data() # 刷新列表

    def del_tool(self):
        row = self.tool_list.currentRow()
        if row < 0: return
        
        if len(self.manager.get_tools()) <= 1:
            QMessageBox.warning(self, "Warning", "Cannot delete the last tool.")
            return

        reply = QMessageBox.question(self, "Delete", "Delete this tool config?", 
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.manager.delete_tool(row)
            self.load_data()

    def rename_current_tool(self, item):
        row = self.tool_list.row(item)
        old_name = item.text()
        text, ok = QInputDialog.getText(self, "Rename", "New Name:", text=old_name)
        if ok and text:
            self.manager.rename_tool(row, text)
            item.setText(text)

    def accept(self):
        # 按下 OK 時，強制存檔到 JSON
        self.manager.save_config()
        super().accept()