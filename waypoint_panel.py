# waypoint_panel.py
import copy 
import numpy as np 
from PySide6.QtWidgets import (QInputDialog, QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem, 
                               QWidget, QLabel, QPushButton, QSizePolicy, QMenu, QFrame, QMessageBox)
from PySide6.QtCore import QItemSelectionModel, Qt, Signal, QSize
from PySide6.QtGui import QCursor, QShortcut, QKeySequence 
import qtawesome as qta

import styles
from widgets import BaseBlock, apply_windows_dark_titlebar

# ==========================================
# 分頁元件 (Tab)
# ==========================================
class EditorTab(QFrame):
    closed = Signal(object)
    clicked_sig = Signal(object)

    def __init__(self, title="untitled.json", parent=None):
        super().__init__(parent)
        self.waypoints_data = []  # 儲存這個分頁專屬的路徑資料
        self.is_modified = False  # 追蹤這個分頁是否被修改過
        self.filepath = ""        # 記錄這個分頁對應的實體檔案路徑 (如果有)
        
        self.setFixedHeight(26) 
        self.title = title
        
        self.layout = QHBoxLayout(self)
        self.layout.setContentsMargins(12, 0, 4, 0)
        self.layout.setSpacing(6)
        
        self.lbl = QLabel(title)
        self.lbl.setStyleSheet("background-color: transparent; border: none; font-family: 'Segoe UI', sans-serif; font-size: 13px;")
        self.layout.addWidget(self.lbl)
        
        self.btn_close = QPushButton()
        self.btn_close.setIcon(qta.icon('mdi.close', color='#cccccc'))
        self.btn_close.setIconSize(QSize(14, 14))
        self.btn_close.setFixedSize(20, 20)
        self.btn_close.setStyleSheet("""
            QPushButton { border: none; background: transparent; border-radius: 4px; } 
            QPushButton:hover { background-color: #555555; }
        """)
        self.btn_close.clicked.connect(lambda: self.closed.emit(self))
        self.layout.addWidget(self.btn_close)
        
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        
    def set_active(self, active):
        if active:
            self.setStyleSheet(styles.EDITOR_TAB_ACTIVE_STYLE)
        else:
            self.setStyleSheet(styles.EDITOR_TAB_INACTIVE_STYLE)
            
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked_sig.emit(self)
        super().mousePressEvent(event)

# ==========================================
# 雙擊觸發的專屬標籤元件
# ==========================================
class DoubleClickLabel(QLabel):
    def __init__(self, text, options, on_change_callback, parent=None):
        super().__init__(text, parent)
        self.options = options
        self.on_change = on_change_callback

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            menu = QMenu()
            menu.setStyleSheet(styles.MENU_STYLE)
            for opt in self.options:
                action = menu.addAction(opt)
                action.triggered.connect(lambda checked, val=opt: self._handle_selection(val))
            menu.exec(QCursor.pos())

    def _handle_selection(self, val):
        self.on_change(val)

# ==========================================
# 單行點位專屬 Widget
# ==========================================
class WaypointRowWidget(QWidget):
    def __init__(self, index, wp_data, toggle_cb, delete_cb, update_cb, update_pt_cb, update_tcp_cb=None, insert_special_cb=None, panel=None, parent=None):
        super().__init__(parent)
        self.index = index
        self.wp_data = wp_data
        self.update_cb = update_cb 
        self.delete_cb = delete_cb
        self.update_pt_cb = update_pt_cb
        self.update_tcp_cb = update_tcp_cb
        self.insert_special_cb = insert_special_cb 
        self.panel = panel 
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(10)

        active = wp_data.get("active", True)
        m_type = wp_data.get("type", "PTP")
        
        text_color = "#cccccc" if active else "#666666"
        if active and m_type in ["LOOP_START", "LOOP_END"]: text_color = "#d7ba7d"
        if active and m_type in ["COMMENT", "RAW_CODE"]: text_color = "#6a9955"

        if active and m_type == "SET_TCP": text_color = "#e6a800"
        if active and m_type == "SET_BASE": text_color = "#00a8e6" 

        self.font_style = f"color: {text_color}; {styles.WAYPOINT_FONT_BASE}"
        self.clickable_style = self.font_style
        
        lbl_idx = QLabel(f"{index+1}.")
        lbl_idx.setStyleSheet(self.font_style)
        lbl_idx.setFixedWidth(25) 
        lbl_idx.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(lbl_idx)
        
        if m_type in ["PTP", "LIN", "CIRC"]:
            self.lbl_type = DoubleClickLabel(m_type, ["PTP", "LIN", "CIRC"], self._change_type)
            self.lbl_type.setStyleSheet(self.clickable_style)
        else:
            self.lbl_type = QLabel(m_type)
            self.lbl_type.setStyleSheet(self.font_style)
        self.lbl_type.setFixedWidth(50)
        layout.addWidget(self.lbl_type)

        lbl_info = QLabel()
        lbl_info.setStyleSheet(self.font_style)
        lbl_info.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        
        self.lbl_blend = QLabel()
        self.lbl_blend.setStyleSheet(self.font_style)
        self.lbl_blend.setFixedWidth(50)

        self.lbl_spd = QLabel()
        self.lbl_spd.setStyleSheet(self.font_style)
        self.lbl_spd.setFixedWidth(50)

        self.lbl_acc = QLabel()
        self.lbl_acc.setStyleSheet(self.font_style)
        self.lbl_acc.setFixedWidth(50)

        if m_type == "DELAY":
            lbl_info.setText(f"Wait {wp_data.get('value', 0.0):.1f}s")
        elif m_type == "GRIPPER":
            lbl_info.setText(f"Grip: {wp_data.get('value', 0)}%")
        elif m_type == "I/O":
            lbl_info.setText(f"{wp_data.get('action_type', '')}={wp_data.get('value', 0)}")
        elif m_type == "SET_TCP":
            lbl_info.setText(f"[{wp_data.get('value')}] {wp_data.get('name')}")
        elif m_type == "SET_BASE": 
            lbl_info.setText(f"[{wp_data.get('value')}] {wp_data.get('name')}")
        elif m_type in ["COMMENT", "RAW_CODE", "LOOP_START", "LOOP_END"]:
            lbl_info.setText(f"// {wp_data.get('value', '')}")
        else:
            lbl_info.setText(wp_data.get('name', f'Point {index+1}'))
            
            blend_val = wp_data.get('blend', 'FINE')
            self.lbl_blend = DoubleClickLabel(f"b:{blend_val}", ["FINE", "10%", "25%", "50%", "75%", "100%"], self._change_blend)
            self.lbl_blend.setStyleSheet(self.clickable_style)
            self.lbl_blend.setFixedWidth(50)

            spd_val = wp_data.get('speed', 50)
            self.lbl_spd = DoubleClickLabel(f"v:{int(spd_val)}%", ["10%", "25%", "50%", "75%", "100%"], self._change_speed)
            self.lbl_spd.setStyleSheet(self.clickable_style)
            self.lbl_spd.setFixedWidth(50)

            acc_val = wp_data.get('accel', 50)
            self.lbl_acc = DoubleClickLabel(f"a:{int(acc_val)}%", ["10%", "25%", "50%", "75%", "100%"], self._change_accel)
            self.lbl_acc.setStyleSheet(self.clickable_style)
            self.lbl_acc.setFixedWidth(50)

        NO_PARAM_TYPES = ["DELAY", "GRIPPER", "I/O", "SET_TCP", "SET_BASE", 
                          "COMMENT", "RAW_CODE", "LOOP_START", "LOOP_END"] 
        if m_type in NO_PARAM_TYPES:
            self.lbl_blend.setVisible(False)
            self.lbl_spd.setVisible(False)
            self.lbl_acc.setVisible(False)

        layout.addWidget(lbl_info)
        layout.addWidget(self.lbl_blend)
        layout.addWidget(self.lbl_spd)
        layout.addWidget(self.lbl_acc)

        eye_color = "#cccccc" if active else "#555555"
        eye_icon = 'mdi.eye-outline' if active else 'mdi.eye-off-outline'
        
        self.btn_eye = QPushButton()
        self.btn_eye.setIcon(qta.icon(eye_icon, color=eye_color))
        self.btn_eye.setIconSize(QSize(18, 18))
        self.btn_eye.setFixedSize(26, 26)
        self.btn_eye.setStyleSheet(styles.WAYPOINT_ROW_BTN_STYLE) 
        self.btn_eye.clicked.connect(lambda: toggle_cb(self.index))
        layout.addWidget(self.btn_eye)
        
        self.btn_menu = QPushButton()
        self.btn_menu.setIcon(qta.icon('mdi.dots-vertical', color='#888888'))
        self.btn_menu.setIconSize(QSize(18, 18))
        self.btn_menu.setFixedSize(26, 26)
        self.btn_menu.setStyleSheet(styles.WAYPOINT_ROW_BTN_STYLE) 
        self.btn_menu.clicked.connect(self.show_row_menu)
        layout.addWidget(self.btn_menu)

    # ==========================================
    # 三點選單功能區
    # ==========================================
    def show_row_menu(self):
        menu = QMenu(self.window()) 
        menu.setStyleSheet(styles.MENU_STYLE)
        
        m_type = self.wp_data.get("type", "PTP")
        main_win = self.window()
        
        # 1. 批量功能區 (Copy, Paste, Base Shift)
        sel_count = len(self.panel.path_list.selectedItems())
        copy_text = f"Copy ({sel_count})" if sel_count > 1 else "Copy"
        action_copy = menu.addAction(qta.icon('mdi.content-copy', color='#00e6b8'), copy_text)
        
        clipboard = []
        if hasattr(main_win, 'path_manager'):
            clipboard = getattr(main_win.path_manager, 'clipboard', [])
            
        paste_count = len(clipboard)
        paste_text = f"Insert Copied ({paste_count})" if paste_count > 1 else "Insert Copied"
        action_paste = menu.addAction(qta.icon('mdi.content-paste', color='#00e6b8'), paste_text)
        
        if paste_count == 0:
            action_paste.setEnabled(False)
            
        action_base_shift = None
        if m_type != "SET_BASE":
            action_base_shift = menu.addAction(qta.icon('mdi.axis-arrow', color='#e6a800'), f"Apply Current Base Shift ({sel_count})" if sel_count > 1 else "Apply Current Base Shift")
            
        menu.addSeparator()
        
        # 2. 針對 SET_BASE 點位的專屬神級功能：區域同步
        action_shift_block = None
        if m_type == "SET_BASE":
            action_shift_block = menu.addAction(qta.icon('mdi.axis-arrow', color='#e6a800'), "Sync Base Shift to Following Points")
            menu.addSeparator()

        # 3. 動作更新與屬性編輯 (實體點位與 Delay)
        action_update = None
        action_edit = None
        if m_type in ["PTP", "LIN", "CIRC"]:
            action_update = menu.addAction("Update Position")
            menu.addSeparator()
        elif m_type == "DELAY":
            action_edit = menu.addAction("Edit Delay Time")
            menu.addSeparator()
            
        # 4. 各類點位插入與更替 (Insert / Update)
        menu_insert_pt = menu.addMenu(qta.icon('mdi.map-marker-plus', color='#e0e0e0'), "Insert Current Position")
        insert_pt_actions = {}
        for pt_type in ["PTP", "LIN", "CIRC"]:
            action = menu_insert_pt.addAction(pt_type)
            insert_pt_actions[action] = pt_type
            
        # ================== TCP 智慧選單 ==================
        # 判斷是 SET_TCP 就顯示橘色的 Update，否則顯示灰色的 Insert
        menu_tcp_title = "Update from Tool Box" if m_type == "SET_TCP" else "Insert SET_TCP"
        menu_tcp_icon = '#e6a800' if m_type == "SET_TCP" else '#d4d4d4'
        menu_tcp = menu.addMenu(qta.icon('mdi.wrench-outline', color=menu_tcp_icon), menu_tcp_title)
        
        tcp_actions = {}
        if hasattr(main_win, 'tcp_manager'):
            for i, t in enumerate(main_win.tcp_manager.tools):
                if t.get('in_box', False):
                    act = menu_tcp.addAction(t['name'])
                    tcp_actions[act] = i
        if not tcp_actions:
            menu_tcp.addAction("Tool Box is Empty").setEnabled(False)
            
        # ================== Base 智慧選單 ==================
        # 判斷是 SET_BASE 就顯示青色的 Update，否則顯示灰色的 Insert
        menu_base_title = "Update Base Frame" if m_type == "SET_BASE" else "Insert SET_BASE"
        menu_base_icon = '#00e6b8' if m_type == "SET_BASE" else '#d4d4d4'
        menu_base = menu.addMenu(qta.icon('mdi.view-grid-outline', color=menu_base_icon), menu_base_title)
        
        base_actions = {}
        if hasattr(main_win, 'base_manager'):
            for i, b in enumerate(main_win.base_manager.bases):
                if b.get('in_box', True):
                    act = menu_base.addAction(b['name'])
                    base_actions[act] = i
        if not base_actions:
            menu_base.addAction("Base Box is Empty").setEnabled(False)
            
        # ================== 雜項插入 ==================
        action_insert_delay = None
        if m_type != "DELAY":
            action_insert_delay = menu.addAction(qta.icon('mdi.timer-outline', color='#e0e0e0'), "Insert Delay")
            
        action_io = menu.addAction(qta.icon('mdi.power-plug-outline', color='#d4d4d4'), "Insert I/O")
        
        menu.addSeparator()
        
        # 5. 批量刪除按鈕
        del_text = f"Delete ({sel_count})" if sel_count > 1 else "Delete"
        action_delete = menu.addAction(qta.icon('mdi.trash-can-outline', color='#ff4d4d'), del_text)
        
        # ==========================================
        # 執行動作配發
        # ==========================================
        selected = menu.exec(QCursor.pos())
        if not selected: return
        
        # 處理第 3 區的動作 (Update Position / Edit Delay)
        if action_update and selected == action_update:
            self.update_pt_cb(self.index)
        elif action_edit and selected == action_edit:
            self._trigger_delay_edit()

        # 處理第 1 區與第 2 區 (Copy / Paste / Base Shift)
        elif selected == action_copy:
            indexes = [self.panel.path_list.row(item) for item in self.panel.path_list.selectedItems()]
            self.panel.copy_requested.emit(indexes) 
        elif selected == action_paste:
            self.panel.paste_requested.emit(self.index) 
        elif action_base_shift and selected == action_base_shift:
            indexes = [self.panel.path_list.row(item) for item in self.panel.path_list.selectedItems()]
            self.panel.batch_base_shift_requested.emit(indexes) 
        elif action_shift_block and selected == action_shift_block:
            self.panel.block_base_shift_requested.emit(self.index)
            
        # 處理第 4 區 (Insert/Update)
        elif selected in insert_pt_actions:
            self.panel.record_pt_requested.emit(self.index, insert_pt_actions[selected])
            
        elif selected in tcp_actions:
            tid = tcp_actions[selected]
            if m_type == "SET_TCP":
                # 執行覆寫更新 (Update)
                if self.update_tcp_cb:
                    self.update_tcp_cb(self.index, tid)
            else:
                # 執行全新插入 (Insert)
                self.panel.insert_special_requested.emit(self.index, f"SET_TCP:{tid}")
                
        elif selected in base_actions:
            bid = base_actions[selected]
            if m_type == "SET_BASE":
                # 執行覆寫更新 (Update)
                self.wp_data['value'] = bid
                self.wp_data['name'] = main_win.base_manager.bases[bid]['name']
                main_win.path_manager.list_update_signal.emit()
                if hasattr(main_win, 'log_widget'):
                    main_win.log_widget.append_log(f"[System] Updated SET_BASE at line {self.index + 1} to '{self.wp_data['name']}'.")
            else:
                # 執行全新插入 (Insert)
                self.panel.insert_special_requested.emit(self.index, f"SET_BASE:{bid}")
                
        elif action_insert_delay and selected == action_insert_delay:
            self.panel.insert_special_requested.emit(self.index, "DELAY")
            
        elif selected == action_io:
            self.panel.insert_special_requested.emit(self.index, "IO")
            
        # 處理第 5 區 (刪除)
        elif selected == action_delete:
            self.panel._handle_delete_key()

    def _trigger_insert_special(self, pt_type):
        if self.insert_special_cb:
            self.insert_special_cb(self.index, pt_type)

    def _trigger_delay_edit(self):
        old_val = self.wp_data.get("value", 0.0)
        new_val, ok = QInputDialog.getDouble(self, "Edit Delay", "Enter seconds:", old_val, 0, 3600, 1)
        if ok:
            self.wp_data["value"] = new_val
            if self.update_cb: self.update_cb()

    def _change_type(self, new_val):
        self.wp_data['type'] = new_val
        self.lbl_type.setText(new_val)
        if self.update_cb: self.update_cb()

    def _change_blend(self, new_val):
        self.wp_data['blend'] = new_val
        self.lbl_blend.setText(f"b:{new_val}")
        if self.update_cb: self.update_cb()

    def _change_speed(self, new_val):
        val = float(new_val.replace('%', ''))
        self.wp_data['speed'] = val
        self.lbl_spd.setText(f"v:{int(val)}%") 
        if self.update_cb: self.update_cb()

    def _change_accel(self, new_val):
        val = float(new_val.replace('%', ''))
        self.wp_data['accel'] = val
        self.lbl_acc.setText(f"a:{int(val)}%") 
        if self.update_cb: self.update_cb()


# ==========================================
# 路徑清單主面板 (多分頁支援)
# ==========================================
class WaypointPanel(BaseBlock):
    toggle_requested = Signal(int)
    delete_requested = Signal(int)
    update_pt_requested = Signal(int) 
    record_pt_requested = Signal(int, str) 
    update_tcp_point_requested = Signal(int, int)
    insert_special_requested = Signal(int, str) 
    clear_all_requested = Signal() 
    data_changed = Signal() 
    tab_switch_requested = Signal(object) 
    tab_closed_signal = Signal(object) 

    copy_requested = Signal(list)       # 傳送被選取的 index 列表
    paste_requested = Signal(int)       # 傳送要貼上的位置
    batch_base_shift_requested = Signal(list) # 傳送被框選的 index 列表
    block_base_shift_requested = Signal(int)  # 傳送 SET_BASE 的 index

    def __init__(self, parent=None):
        nav_config = [
            {'icon': 'mdi.content-save'},    
            {'icon': 'mdi.folder-open'},    
            {'icon': 'mdi.delete-sweep'},
            {'icon': 'mdi.dots-vertical'}    
        ]
        super().__init__(parent=parent, nav_config=nav_config)
        self.setMinimumWidth(360)
        
        self.tabs = []
        self.active_tab = None
        self.copied_waypoints = [] 

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 45) 
        main_layout.setSpacing(0)

        self.tab_bar = QFrame()
        self.tab_bar.setFixedHeight(26)
        self.tab_bar_layout = QHBoxLayout(self.tab_bar)
        self.tab_bar_layout.setContentsMargins(0, 0, 0, 0)
        self.tab_bar_layout.setSpacing(0)
        self.tab_bar_layout.setAlignment(Qt.AlignmentFlag.AlignLeft) 

        self.tabs_container = QWidget()
        self.tabs_layout = QHBoxLayout(self.tabs_container)
        self.tabs_layout.setContentsMargins(0, 0, 0, 0)
        self.tabs_layout.setSpacing(0)
        self.tabs_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)
        
        self.tab_bar_layout.addWidget(self.tabs_container)

        self.separator = QFrame()
        self.separator.setFixedSize(1, 14)
        self.separator.setStyleSheet("background-color: #454545; margin-left: 4px; margin-right: 2px;")
        self.tab_bar_layout.addWidget(self.separator)

        self.btn_new_tab = QPushButton()
        self.btn_new_tab.setIcon(qta.icon('mdi.plus', color='#cccccc'))
        self.btn_new_tab.setIconSize(QSize(16, 16))
        self.btn_new_tab.setFixedSize(26, 26)
        self.btn_new_tab.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_new_tab.setStyleSheet("""
            QPushButton { border: none; background: transparent; border-radius: 4px; }
            QPushButton:hover { background-color: #3a3d41; }
        """)
        self.btn_new_tab.clicked.connect(self.add_new_tab)
        self.tab_bar_layout.addWidget(self.btn_new_tab)

        self.tab_bar_layout.addStretch() 
        main_layout.addWidget(self.tab_bar)

        self.path_list = QListWidget()
        self.path_list.setStyleSheet(styles.PATH_LIST_STYLE) 
        self.path_list.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.path_list.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)
        self.path_list.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.path_list.customContextMenuRequested.connect(self.show_list_context_menu)
        main_layout.addWidget(self.path_list)

        self.shortcut_delete = QShortcut(QKeySequence(Qt.Key.Key_Delete), self.path_list)
        self.shortcut_delete.activated.connect(self._handle_delete_key)
        
        self.btn_save = self.nav_bar.nav_buttons[0]
        self.btn_load = self.nav_bar.nav_buttons[1]
        self.btn_clear = self.nav_bar.nav_buttons[2]
        self.btn_menu = self.nav_bar.nav_buttons[3]

        self.btn_save.setToolTip("Save Path")
        self.btn_load.setToolTip("Load Path")
        self.btn_clear.setToolTip("Clear All Waypoints")
        self._active_menu = None
        self.btn_clear.clicked.connect(self.confirm_clear_all)

    # ==========================================
    # 其他列表事件與 UI 更新
    # ==========================================
    def _handle_delete_key(self):
        selected_items = self.path_list.selectedItems()
        if not selected_items: return

        main_win = self.window()
        if not hasattr(main_win, 'path_manager'): return

        indexes = [self.path_list.row(item) for item in selected_items]
        valid_indexes = [i for i in indexes if 0 <= i < len(main_win.path_manager.waypoints)]
        
        valid_indexes.sort(reverse=True) 

        if not valid_indexes: return

        main_win.path_manager.blockSignals(True)
        for idx in valid_indexes:
            main_win.path_manager.delete_point(idx)
        main_win.path_manager.blockSignals(False)
        
        main_win.path_manager.list_update_signal.emit()

    def confirm_clear_all(self):
        if not self.active_tab:
            return
            
        msg_box = QMessageBox(self.window()) 
        apply_windows_dark_titlebar(msg_box)
        msg_box.setWindowTitle("Clear All Waypoints")
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setText("Are you sure you want to delete ALL waypoints in this tab?\nThis action cannot be undone.")
        msg_box.setStyleSheet(styles.DARK_MESSAGE_BOX_STYLE)
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel)
        
        yes_btn = msg_box.button(QMessageBox.StandardButton.Yes)
        yes_btn.setText("Delete All")
        yes_btn.setStyleSheet("""
            QPushButton { background-color: #a32626; color: white; border: 1px solid #cc3333; }
            QPushButton:hover { background-color: #cc3333; }
        """)
        
        if msg_box.exec() == QMessageBox.StandardButton.Yes:
            self.clear_all_requested.emit()

    def _handle_spacebar(self):
        from PySide6.QtWidgets import QApplication
        
        focus_w = QApplication.focusWidget()
        if focus_w:
            if focus_w.inherits("QLineEdit") or focus_w.inherits("QAbstractSpinBox") or focus_w.inherits("QTextEdit"):
                return

        local_pos = self.path_list.mapFromGlobal(QCursor.pos())
        if not self.path_list.rect().contains(local_pos):
            return 

        item = self.path_list.itemAt(local_pos)
        if item is not None and self.path_list.itemWidget(item) is not None:
            return 

        if getattr(self, '_active_menu', None) is not None:
            self._active_menu.close()
            self._active_menu = None
            return

        self.show_list_context_menu(pos=None)

    def add_new_tab(self, name="untitled.json"):
        if not isinstance(name, str) or not name: name = "untitled.json"
            
        tab = EditorTab(name)
        tab.closed.connect(self.close_tab)
        tab.clicked_sig.connect(self.request_tab_switch)
        
        self.tabs.append(tab)
        self.tabs_layout.addWidget(tab)
        self.request_tab_switch(tab)

    def request_tab_switch(self, new_tab):
        if new_tab != self.active_tab:
            main_win = self.window()
            
            # 👑 1. 如果有舊分頁 (目前的 active_tab)，先把大腦裡的資料備份進去
            if self.active_tab and hasattr(main_win, 'path_manager'):
                self.active_tab.waypoints_data = copy.deepcopy(main_win.path_manager.waypoints)
                self.active_tab.is_modified = main_win.path_manager.is_modified
            
            # 👑 2. 進行 UI 上的切換
            self.set_active_tab_visuals(new_tab)
            
            # 👑 3. 把新分頁的資料倒進大腦裡
            if hasattr(main_win, 'path_manager'):
                # 倒資料前先鎖住信號，避免觸發不必要的 UI 刷新 (待會再一次刷)
                main_win.path_manager.blockSignals(True) 
                main_win.path_manager.waypoints = copy.deepcopy(new_tab.waypoints_data)
                main_win.path_manager.is_modified = new_tab.is_modified
                main_win.path_manager.blockSignals(False)
            
            # 👑 4. 呼叫外界 (gui.py) 進行完整的 UI 連動更新 (3D畫面、清單)
            self.tab_switch_requested.emit(new_tab)

    def set_active_tab_visuals(self, tab):
        for t in self.tabs:
            t.set_active(t == tab)
        self.active_tab = tab
        self._update_theme()

    def close_tab(self, tab):
        """第一階段：只發送「請求關閉」訊號，不再自己動手"""
        self.tab_closed_signal.emit(tab)

    def force_close_tab(self, tab):
        """第二階段：接收主程式 (gui.py) 的命令，真正移除 UI"""
        if tab in self.tabs:
            self.tabs.remove(tab)
            self.tabs_layout.removeWidget(tab)
            tab.deleteLater()
            
            if tab == self.active_tab:
                self.active_tab = None
                if self.tabs:
                    self.request_tab_switch(self.tabs[-1])
                else:
                    self._update_theme()
            else:
                self._update_theme()

    def _update_theme(self):
        if not self.tabs:
            self.setStyleSheet(styles.BLOCK_STYLE)
            self.path_list.setVisible(False)
            self.tabs_container.setVisible(False)
            self.separator.setVisible(False)
            self.tab_bar.setStyleSheet("QFrame { background-color: transparent; }")
        else:
            self.setStyleSheet(styles.WAYPOINT_PANEL_BG_STYLE)
            self.path_list.setVisible(True)
            self.tabs_container.setVisible(True)
            self.separator.setVisible(True)
            self.tab_bar.setStyleSheet(styles.TAB_BAR_FRAME_STYLE)

    def set_file_name(self, filename):
        if self.active_tab:
            self.active_tab.lbl.setText(filename)
            self.active_tab.title = filename
        else:
            self.add_new_tab(filename)

    def update_list(self, waypoints):
        if not self.active_tab:
            return 
            
        v_bar = self.path_list.verticalScrollBar()
        current_scroll = v_bar.value() if v_bar else 0
        current_row = self.path_list.currentRow()
            
        self.path_list.blockSignals(True)
        self.path_list.clear()
        
        for i, wp in enumerate(waypoints):
            item = QListWidgetItem()
            row_widget = WaypointRowWidget(
                i, wp, 
                self.toggle_requested.emit, 
                self.delete_requested.emit, 
                self.data_changed.emit, 
                self.update_pt_requested.emit, 
                self.update_tcp_point_requested.emit,
                self.insert_special_requested.emit,
                self 
            )
            item.setSizeHint(row_widget.sizeHint())
            self.path_list.addItem(item)
            self.path_list.setItemWidget(item, row_widget)
            
        spacer_item = QListWidgetItem()
        spacer_item.setSizeHint(QSize(0, 510))
        spacer_item.setFlags(Qt.ItemFlag.NoItemFlags)  
        self.path_list.addItem(spacer_item)        
        self.path_list.blockSignals(False)

        if current_row >= 0 and current_row < self.path_list.count():
            self.select_row_silently(current_row)
            
        if v_bar:
            v_bar.setValue(current_scroll)

    def show_list_context_menu(self, pos=None):
        if pos is not None:
            item = self.path_list.itemAt(pos)
            if item is not None and self.path_list.itemWidget(item) is not None:
                return 
            menu_pos = self.path_list.mapToGlobal(pos)
        else:
            menu_pos = QCursor.pos()
            
        menu = QMenu(self.window())
        self._active_menu = menu  
        menu.setStyleSheet(styles.MENU_STYLE)
        menu.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose) 
        
        target_idx = self.path_list.count() - 1
        if target_idx < 0: 
            target_idx = 0
            
        # 👑 修正：改向大腦 (PathManager) 讀取剪貼簿狀態
        main_win = self.window()
        clipboard = []
        if hasattr(main_win, 'path_manager'):
            clipboard = getattr(main_win.path_manager, 'clipboard', [])
            
        paste_count = len(clipboard)
        paste_text = f"Paste Copied ({paste_count})" if paste_count > 1 else "Paste Copied"
        action_paste = menu.addAction(qta.icon('mdi.content-paste', color='#00e6b8'), paste_text)
        action_paste.setEnabled(paste_count > 0)
        menu.addSeparator()

        menu_pt = menu.addMenu(qta.icon('mdi.map-marker-plus', color='#d4d4d4'), "Record Current Position")
        pt_actions = {} 
        for m_type in ["PTP", "LIN", "CIRC"]:
            action = menu_pt.addAction(m_type)
            pt_actions[action] = m_type
            
        menu.addSeparator()
        
        action_delay = menu.addAction(qta.icon('mdi.timer-outline', color='#e0e0e0'), "Append Delay")
        action_io = menu.addAction(qta.icon('mdi.power-plug-outline', color='#e0e0e0'), "Append I/O")
        
        menu_tcp = menu.addMenu(qta.icon('mdi.tools', color='#e6a800'), "Append SET_TCP")
        box_tools = []
        main_win = self.window()
        if hasattr(main_win, 'tcp_manager'):
            mgr = main_win.tcp_manager
            box_tools = [(i, t['name']) for i, t in enumerate(mgr.tools) if t.get('in_box', False)]
            
        tcp_actions = {} 
        if not box_tools:
            action_empty = menu_tcp.addAction("Tool Box is Empty")
            action_empty.setEnabled(False)
        else:
            for t_idx, t_name in box_tools:
                action = menu_tcp.addAction(qta.icon('mdi.wrench-outline', color='#d4d4d4'), t_name)
                tcp_actions[action] = t_idx 

        menu_base = menu.addMenu(qta.icon('mdi.view-grid-outline', color='#00a8e6'), "Append SET_BASE")
        box_bases = []
        if hasattr(main_win, 'base_manager'):
            bmgr = main_win.base_manager
            box_bases = [(i, b['name']) for i, b in enumerate(bmgr.bases) if b.get('in_box', True)]
            
        base_actions = {} 
        if not box_bases:
            action_empty_b = menu_base.addAction("Base Box is Empty")
            action_empty_b.setEnabled(False)
        else:
            for b_idx, b_name in box_bases:
                action = menu_base.addAction(qta.icon('mdi.grid', color='#d4d4d4'), b_name)
                base_actions[action] = b_idx
                
        try:
            selected_action = menu.exec(menu_pos)
        finally:
            self._active_menu = None  
        
        if not selected_action:
            return
            
        if selected_action == action_paste:
            self.paste_requested.emit(-1)
        elif selected_action in pt_actions:
            self.record_pt_requested.emit(target_idx, pt_actions[selected_action])
        elif selected_action == action_delay:
            self.insert_special_requested.emit(target_idx, "DELAY")
        elif selected_action == action_io:
            self.insert_special_requested.emit(target_idx, "IO")
        elif selected_action in tcp_actions:
            tid = tcp_actions[selected_action]
            self.insert_special_requested.emit(target_idx, f"SET_TCP:{tid}")
        elif selected_action in base_actions:
            bid = base_actions[selected_action]
            self.insert_special_requested.emit(target_idx, f"SET_BASE:{bid}")

    def select_row_silently(self, index):
        if index < 0 or index >= self.path_list.count():
            return
            
        model_idx = self.path_list.model().index(index, 0)
        
        # 👑 加上阻斷器：純粹改變 UI 選取狀態，絕對不發射 currentRowChanged 信號！
        self.path_list.blockSignals(True)
        self.path_list.selectionModel().setCurrentIndex(
            model_idx, 
            QItemSelectionModel.SelectionFlag.ClearAndSelect
        )
        self.path_list.blockSignals(False)