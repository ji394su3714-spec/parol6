# base_dialog.py
from PySide6.QtWidgets import (QCheckBox, QDialog, QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem,
                               QPushButton, QLabel, QLineEdit, QGridLayout, QInputDialog, QMessageBox, QWidget)
from PySide6.QtCore import QSize, Qt
from PySide6.QtGui import QDoubleValidator
import qtawesome as qta
import styles
from widgets import apply_windows_dark_titlebar

# ==========================================
# 基座清單專用的單行排版元件 (與 TCP 完全對齊)
# ==========================================
class BaseRowWidget(QWidget):
    def __init__(self, index, name, in_box=True, is_locked=False, on_toggle_cb=None, parent=None):
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

        # 鎖定狀態的視覺提示
        display_name = f"{name} (Locked)" if is_locked else name
        name_color = "#e6a800" if is_locked else "#d4d4d4"
        
        self.lbl_name = QLabel(display_name)
        self.lbl_name.setStyleSheet(f"color: {name_color}; font-family: 'Segoe UI', sans-serif; font-size: 13px; background: transparent;")
        layout.addWidget(self.lbl_name)
        
        layout.addStretch()

        self.chk_box = QCheckBox()
        self.chk_box.setChecked(in_box)
        self.chk_box.setToolTip("Show in Right-Click Menu")
        self.chk_box.setStyleSheet("""
            QCheckBox::indicator { width: 16px; height: 16px; }
        """)
        
        if is_locked:
            self.chk_box.setEnabled(False) # 鎖定的 Base 0 不允許被移出選單
            
        self.chk_box.toggled.connect(self._handle_toggle)
        layout.addWidget(self.chk_box)

    def _handle_toggle(self, checked):
        if self.on_toggle_cb:
            self.on_toggle_cb(self.index, checked)

class BaseManagerDialog(QDialog):
    def __init__(self, base_manager, parent=None):
        super().__init__(parent)
        self.base_manager = base_manager
        self.setWindowTitle("Base Frame Manager")
        self.setMinimumSize(450, 320)
        self.setStyleSheet(styles.PREFERENCES_DIALOG_STYLE)
        self.setWindowIcon(qta.icon('mdi.view-grid-outline'))
        
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # ==========================================
        # 左側清單
        # ==========================================
        left_layout = QVBoxLayout()
        left_layout.setSpacing(8)
        lbl_list = QLabel("Base Frame List")
        lbl_list.setFont(styles.FONT_TITLE)
        left_layout.addWidget(lbl_list)

        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet(styles.PATH_LIST_STYLE)
        self.list_widget.currentRowChanged.connect(self.on_base_selected)
        left_layout.addWidget(self.list_widget)

        btn_layout = QHBoxLayout()
        self.btn_add = QPushButton(qta.icon('mdi.plus', color='#00e6b8'), "Add")
        self.btn_del = QPushButton(qta.icon('mdi.trash-can-outline', color='#e0e0e0'), "Del")
        self.btn_rename = QPushButton(qta.icon('mdi.form-textbox', color='#e0e0e0'), "Rename")
        
        self.btn_add.clicked.connect(self.add_base)
        self.btn_del.clicked.connect(self.delete_base)
        self.btn_rename.clicked.connect(self.rename_base)
        
        btn_layout.addWidget(self.btn_add)
        btn_layout.addWidget(self.btn_del)
        btn_layout.addWidget(self.btn_rename)
        left_layout.addLayout(btn_layout)
        main_layout.addLayout(left_layout, 1)

        # ==========================================
        # 右側數值 (Grid Layout)
        # ==========================================
        right_layout = QVBoxLayout()
        right_layout.setSpacing(15)
        lbl_edit = QLabel("Base Offset Parameters")
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

        # 教導按鈕
        self.btn_teach = QPushButton(" Teach (3-Point Method)")
        self.btn_teach.setIcon(qta.icon('mdi.target', color='#e6a800'))
        self.btn_teach.setStyleSheet(styles.BTN_SECONDARY_STYLE)
        self.btn_teach.setFixedHeight(30)
        self.btn_teach.setEnabled(False) 
        right_layout.addWidget(self.btn_teach)

        # 套用按鈕
        self.btn_apply = QPushButton("Apply & Close")
        self.btn_apply.setFixedHeight(30)
        self.btn_apply.clicked.connect(self.apply_and_close)
        right_layout.addWidget(self.btn_apply)
        
        main_layout.addLayout(right_layout, 1)

        self._is_loading = False
        self.refresh_list(target_idx=self.base_manager.current_index)

    def refresh_list(self, target_idx=None):
        current_idx = self.list_widget.currentRow() if target_idx is None else target_idx
        if current_idx < 0: 
            current_idx = 0

        self.list_widget.blockSignals(True)
        self.list_widget.clear()
        
        for i, base_data in enumerate(self.base_manager.bases):
            item = QListWidgetItem()
            in_box = base_data.get("in_box", True) 
            is_locked = base_data.get("is_locked", False)
            
            row_widget = BaseRowWidget(i, base_data['name'], in_box, is_locked, self.on_box_toggled)
            item.setSizeHint(QSize(0, 30))
            
            self.list_widget.addItem(item)
            self.list_widget.setItemWidget(item, row_widget)
            
        if len(self.base_manager.bases) > 0:
            if current_idx >= len(self.base_manager.bases):
                current_idx = len(self.base_manager.bases) - 1
            self.list_widget.setCurrentRow(current_idx)
            
        self.list_widget.blockSignals(False)
        self.on_base_selected(current_idx)

    def on_box_toggled(self, index, checked):
        if 0 <= index < len(self.base_manager.bases):
            self.base_manager.bases[index]["in_box"] = checked

    def on_base_selected(self, index):
        if self._is_loading or index < 0: return
        
        base_data = self.base_manager.bases[index]
        vals = base_data.get("values", [0.0]*6)
        is_locked = base_data.get("is_locked", False)
        
        self._is_loading = True
        for k, v in zip(["x", "y", "z", "rx", "ry", "rz"], vals):
            self.inputs[k].setText(f"{v:.3f}")
        self._is_loading = False
        
        # 動態保護：如果選到鎖定的 Base 0，不准刪除與改名，按鈕外觀也會變化
        self.btn_del.setEnabled(not is_locked)
        self.btn_rename.setEnabled(not is_locked)
        self.btn_teach.setEnabled(not is_locked)
        
        if is_locked:
            self.btn_apply.setText("Apply World Calibration & Close")
            self.btn_apply.setStyleSheet(styles.BTN_PRIMARY_STYLE) 
        else:
            self.btn_apply.setText("Apply & Close")
            self.btn_apply.setStyleSheet(styles.BTN_SECONDARY_STYLE) 

    def add_base(self):
        # 預設新數值為全 0
        self.base_manager.add_base(f"Base {len(self.base_manager.bases)}", [0.0]*6, True)
        self.refresh_list(target_idx=len(self.base_manager.bases) - 1)

    def delete_base(self):
        if len(self.base_manager.bases) <= 1:
            QMessageBox.warning(self, "Warning", "Cannot delete the last Base frame.")
            return
        self.base_manager.delete_base(self.list_widget.currentRow())
        self.refresh_list()

    def rename_base(self):
        idx = self.list_widget.currentRow()
        if idx < 0: return
        old_name = self.base_manager.bases[idx]["name"]
        new_name, ok = QInputDialog.getText(self, "Rename Base", "New name:", QLineEdit.EchoMode.Normal, old_name)
        if ok and new_name.strip():
            self.base_manager.bases[idx]["name"] = new_name.strip()
            self.refresh_list(target_idx=idx)

    def apply_and_close(self):
        idx = self.list_widget.currentRow()
        if idx >= 0:
            try:
                new_vals = [float(self.inputs[k].text() or 0.0) for k in ["x", "y", "z", "rx", "ry", "rz"]]
                
                current_base = self.base_manager.bases[idx]
                name = current_base.get("name", f"Base {idx}")
                in_box = current_base.get("in_box", True)
                
                # 寫入大腦！
                self.base_manager.update_base(idx, name, new_vals, in_box)
            except ValueError:
                pass
                
        self.accept()

    def showEvent(self, event):
        super().showEvent(event)
        apply_windows_dark_titlebar(self)