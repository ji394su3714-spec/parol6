# waypoint_panel.py
import os
from PySide6.QtWidgets import (QInputDialog, QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem, 
                               QWidget, QLabel, QPushButton, QSizePolicy, QMenu, QFrame, QMessageBox)
from PySide6.QtCore import Qt, Signal, QSize
from PySide6.QtGui import QCursor, QShortcut, QKeySequence 
import qtawesome as qta

import styles
from widgets import BaseBlock, apply_windows_dark_titlebar

# ==========================================
# 分頁元件 (VS Code Style Tab)
# ==========================================
class EditorTab(QFrame):
    closed = Signal(object)
    clicked_sig = Signal(object)

    def __init__(self, title="untitled.json", parent=None):
        super().__init__(parent)
        self.waypoints_data = [] 
        
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
    def __init__(self, index, wp_data, toggle_cb, delete_cb, update_cb, update_pt_cb, insert_pt_cb, update_tcp_cb=None, insert_special_cb=None, parent=None):
        super().__init__(parent)
        self.index = index
        self.wp_data = wp_data
        self.update_cb = update_cb 
        self.delete_cb = delete_cb
        self.update_pt_cb = update_pt_cb
        self.insert_pt_cb = insert_pt_cb
        self.update_tcp_cb = update_tcp_cb
        self.insert_special_cb = insert_special_cb 
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(10)

        active = wp_data.get("active", True)
        m_type = wp_data.get("type", "PTP")
        
        text_color = "#cccccc" if active else "#666666"
        if active and m_type in ["LOOP_START", "LOOP_END"]: text_color = "#d7ba7d"
        if active and m_type in ["COMMENT", "RAW_CODE"]: text_color = "#6a9955"

        # 讓換刀指令呈現醒目的亮橘黃色
        if active and m_type == "SET_TCP": text_color = "#e6a800"

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

        NO_PARAM_TYPES = ["DELAY", "GRIPPER", "I/O", "SET_TCP",
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

    def show_row_menu(self):
        menu = QMenu()
        menu.setStyleSheet(styles.MENU_STYLE)
        
        m_type = self.wp_data.get("type", "PTP")
        
        # 1. 動作更新 (Position, Delay, TCP)
        if m_type in ["PTP", "LIN", "CIRC"]:
            action_update = menu.addAction("Update Position")
            action_update.triggered.connect(lambda: self.update_pt_cb(self.index))
            menu.addSeparator()
        elif m_type == "DELAY":
            action_edit = menu.addAction("Edit Delay Time")
            action_edit.triggered.connect(self._trigger_delay_edit)
            menu.addSeparator()
        # 將原本單一的 Update 按鈕，升級為 Tool Box 抽屜
        elif m_type == "SET_TCP":
            menu_update_tcp = menu.addMenu("Update from Tool Box")
            
            # 穿透獲取工具箱資料
            box_tools = []
            main_win = self.window()
            if hasattr(main_win, 'tcp_manager'):
                mgr = main_win.tcp_manager
                box_tools = [(i, t['name']) for i, t in enumerate(mgr.tools) if t.get('in_box', False)]
                
            if not box_tools:
                action_empty = menu_update_tcp.addAction("Tool Box is Empty")
                action_empty.setEnabled(False)
            else:
                for t_idx, t_name in box_tools:
                    action = menu_update_tcp.addAction(qta.icon('mdi.wrench-outline', color='#d4d4d4'), t_name)
                    
                    # 魔法綁定：除了回傳行數 (r_idx)，還要回傳選中的刀具編號 (tid)
                    # 注意：在 for 迴圈中用 lambda，必須明確指定 tid=t_idx 才能避免變數被覆蓋
                    action.triggered.connect(
                        lambda checked=False, r_idx=self.index, tid=t_idx: self.update_tcp_cb(r_idx, tid) if self.update_tcp_cb else None
                    )
                    
            menu.addSeparator()
            
        # 2. 一般插入
        action_insert = menu.addAction("Insert Point")
        action_insert.triggered.connect(lambda: self.insert_pt_cb(self.index))
        
        # 3. 特殊插入 (依條件顯示)
        if m_type != "SET_TCP":
            action_insert_tcp = menu.addAction("Insert SET_TCP")
            action_insert_tcp.triggered.connect(lambda: self._trigger_insert_special("SET_TCP"))
            
        if m_type != "DELAY":
            action_insert_delay = menu.addAction("Insert DELAY")
            action_insert_delay.triggered.connect(lambda: self._trigger_insert_special("DELAY"))
            
        menu.addSeparator()
        
        # 4. 刪除
        action_delete = menu.addAction("Delete Point")
        action_delete.triggered.connect(lambda: self.delete_cb(self.index))
        
        menu.exec(QCursor.pos())

    # 新增觸發函式
    def _trigger_insert_special(self, pt_type):
        if self.insert_special_cb:
            self.insert_special_cb(self.index, pt_type)

    def _trigger_delay_edit(self):
        old_val = self.wp_data.get("value", 0.0)
        new_val, ok = QInputDialog.getDouble(self, "Edit Delay", "Enter seconds:", old_val, 0, 3600, 1)
        if ok:
            self.wp_data["value"] = new_val
            if self.update_cb: self.update_cb()

    def _trigger_tcp_update(self):
        """觸發 TCP 更新回呼"""
        if self.update_tcp_cb:
            self.update_tcp_cb(self.index)

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
# 路徑清單主面板 (真・多分頁支援)
# ==========================================
class WaypointPanel(BaseBlock):
    toggle_requested = Signal(int)
    delete_requested = Signal(int)
    update_pt_requested = Signal(int) 
    insert_pt_requested = Signal(int) 
    
    # 👇 加上 str 參數，用來傳遞 "PTP", "LIN", "CIRC"
    record_pt_requested = Signal(int, str) 
    
    update_tcp_point_requested = Signal(int, int)
    insert_special_requested = Signal(int, str) 
    clear_all_requested = Signal() 
    data_changed = Signal() 
    
    tab_switch_requested = Signal(object) 
    tab_closed_signal = Signal(object) 

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
        
        self.btn_clear.clicked.connect(self.confirm_clear_all)

    def confirm_clear_all(self):
        if not self.active_tab:
            return
            
        msg_box = QMessageBox(self)
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

    def _handle_delete_key(self):
        row = self.path_list.currentRow()
        if row >= 0:
            self.delete_requested.emit(row)

    def add_new_tab(self, name="untitled.json"):
        if not isinstance(name, str) or not name: name = "untitled.json"
            
        tab = EditorTab(name)
        tab.closed.connect(self.close_tab)
        tab.clicked_sig.connect(self.request_tab_switch)
        
        self.tabs.append(tab)
        self.tabs_layout.addWidget(tab)
        self.request_tab_switch(tab)

    def request_tab_switch(self, tab):
        if tab != self.active_tab:
            self.tab_switch_requested.emit(tab)

    def set_active_tab_visuals(self, tab):
        for t in self.tabs:
            t.set_active(t == tab)
        self.active_tab = tab
        self._update_theme()

    def close_tab(self, tab):
        self.tab_closed_signal.emit(tab)
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
            
        self.path_list.blockSignals(True)
        self.path_list.clear()
        
        for i, wp in enumerate(waypoints):
            item = QListWidgetItem()
            # 實例化時，把 self.update_tcp_point_requested.emit 當作參數傳給它！
            row_widget = WaypointRowWidget(
                i, wp, 
                self.toggle_requested.emit, 
                self.delete_requested.emit, 
                self.data_changed.emit, 
                self.update_pt_requested.emit, 
                self.insert_pt_requested.emit,
                self.update_tcp_point_requested.emit,
                self.insert_special_requested.emit 
            )
            item.setSizeHint(row_widget.sizeHint())
            self.path_list.addItem(item)
            self.path_list.setItemWidget(item, row_widget)
            
        # 終極 UX 魔法：塞入 17 行高度的隱形墊片
        spacer_item = QListWidgetItem()
        spacer_item.setSizeHint(QSize(0, 510))
        spacer_item.setFlags(Qt.ItemFlag.NoItemFlags)  
        self.path_list.addItem(spacer_item)        
        self.path_list.blockSignals(False)

    def show_list_context_menu(self, pos):
        item = self.path_list.itemAt(pos)
        
        # 💡 判斷修正：只要該 item 身上有掛載真正的 UI Widget，代表它是正常路徑點，我們不介入
        if item is not None and self.path_list.itemWidget(item) is not None:
            return 
            
        menu = QMenu(self)
        menu.setStyleSheet(styles.MENU_STYLE)
        menu.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose) # 👈 確保選單關閉後徹底銷毀，防止殘影重複彈出
        
        target_idx = self.path_list.count() - 1
        if target_idx < 0: 
            target_idx = 0
            
        # ==========================================
        # 1. 建立錄製點位子選單 (PTP, LIN, CIRC)
        # ==========================================
        menu_pt = menu.addMenu(qta.icon('mdi.map-marker-plus', color='#00e6b8'), "Record Current Position")
        pt_actions = {} # 💡 安全記憶字典
        
        for m_type in ["PTP", "LIN", "CIRC"]:
            action = menu_pt.addAction(m_type)
            pt_actions[action] = m_type
            
        menu.addSeparator()
        
        # ==========================================
        # 2. 插入特殊點 (Delay / I/O)
        # ==========================================
        action_delay = menu.addAction(qta.icon('mdi.timer-outline', color='#e0e0e0'), "Append Delay")
        action_io = menu.addAction(qta.icon('mdi.power-plug-outline', color='#e0e0e0'), "Append I/O")
        
        # ==========================================
        # 3. 插入 TCP 工具箱
        # ==========================================
        menu_tcp = menu.addMenu(qta.icon('mdi.toolbox', color='#00e6b8'), "Append SET_TCP")
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
                
        # ==========================================
        # 💡 同步等待與處理結果
        # ==========================================
        selected_action = menu.exec(self.path_list.mapToGlobal(pos))
        
        if not selected_action:
            return 
            
        # 👇 根據使用者點擊的選項，發射對應的訊號
        if selected_action in pt_actions:
            # 發射: (行數, 運動模式)
            self.record_pt_requested.emit(target_idx, pt_actions[selected_action])
            
        elif selected_action == action_delay:
            self.insert_special_requested.emit(target_idx, "DELAY")
            
        elif selected_action == action_io:
            self.insert_special_requested.emit(target_idx, "IO")
            
        elif selected_action in tcp_actions:
            tid = tcp_actions[selected_action]
            self.insert_special_requested.emit(target_idx, f"SET_TCP:{tid}")