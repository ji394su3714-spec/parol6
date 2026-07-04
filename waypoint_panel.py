# waypoint_panel.py
import os
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
    def __init__(self, index, wp_data, toggle_cb, delete_cb, update_cb, update_pt_cb, insert_pt_cb, update_tcp_cb=None, insert_special_cb=None, panel=None, parent=None):
        super().__init__(parent)
        self.index = index
        self.wp_data = wp_data
        self.update_cb = update_cb 
        self.delete_cb = delete_cb
        self.update_pt_cb = update_pt_cb
        self.insert_pt_cb = insert_pt_cb
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

    def show_row_menu(self):
        menu = QMenu(self.window()) 
        menu.setStyleSheet(styles.MENU_STYLE)
        
        m_type = self.wp_data.get("type", "PTP")
        main_win = self.window()
        
        # ==========================================
        # 1. 批量功能區 (Copy, Paste, Base Shift)
        # ==========================================
        sel_count = len(self.panel.path_list.selectedItems())
        copy_text = f"Copy ({sel_count})" if sel_count > 1 else "Copy"
        action_copy = menu.addAction(qta.icon('mdi.content-copy', color='#00e6b8'), copy_text)
        
        paste_count = len(getattr(self.panel, 'copied_waypoints', []))
        paste_text = f"Insert Copied ({paste_count})" if paste_count > 1 else "Insert Copied"
        action_paste = menu.addAction(qta.icon('mdi.content-paste', color='#00e6b8'), paste_text)
        if paste_count == 0:
            action_paste.setEnabled(False)
            
        # 👑 依據使用者要求：如果是 SET_BASE，就不顯示多餘的 Apply Shift
        action_base_shift = None
        if m_type != "SET_BASE":
            action_base_shift = menu.addAction(qta.icon('mdi.axis-arrow', color='#e6a800'), f"Apply Current Base Shift ({sel_count})" if sel_count > 1 else "Apply Current Base Shift")
            
        menu.addSeparator()
        
        # 👑 2. 針對 SET_BASE 點位的專屬神級功能：區域同步！
        action_shift_block = None
        if m_type == "SET_BASE":
            action_shift_block = menu.addAction(qta.icon('mdi.axis-arrow', color='#e6a800'), "Sync Base Shift to Following Points")
            menu.addSeparator()

        # ==========================================
        # 3. 動作更新與屬性編輯
        # ==========================================
        if m_type in ["PTP", "LIN", "CIRC"]:
            action_update = menu.addAction("Update Position")
            action_update.triggered.connect(lambda: self.update_pt_cb(self.index))
            menu.addSeparator()
        elif m_type == "DELAY":
            action_edit = menu.addAction("Edit Delay Time")
            action_edit.triggered.connect(self._trigger_delay_edit)
            menu.addSeparator()
        elif m_type == "SET_TCP":
            menu_update_tcp = menu.addMenu("Update from Tool Box")
            box_tools = []
            if hasattr(main_win, 'tcp_manager'):
                mgr = main_win.tcp_manager
                box_tools = [(i, t['name']) for i, t in enumerate(mgr.tools) if t.get('in_box', False)]
                
            if not box_tools:
                action_empty = menu_update_tcp.addAction("Tool Box is Empty")
                action_empty.setEnabled(False)
            else:
                for t_idx, t_name in box_tools:
                    action = menu_update_tcp.addAction(qta.icon('mdi.wrench-outline', color='#d4d4d4'), t_name)
                    action.triggered.connect(
                        lambda checked=False, r_idx=self.index, tid=t_idx: self.update_tcp_cb(r_idx, tid) if self.update_tcp_cb else None
                    )
            menu.addSeparator()
            
        # ==========================================
        # 4. 各類點位插入 (Insert / Update)
        # ==========================================
        action_insert = menu.addAction("Insert Point")
        action_insert.triggered.connect(lambda: self.insert_pt_cb(self.index))
        
        if m_type != "SET_TCP":
            action_insert_tcp = menu.addAction("Insert SET_TCP")
            action_insert_tcp.triggered.connect(lambda: self._trigger_insert_special("SET_TCP"))
            
        # 👑 依據使用者要求：智慧變換名稱 (Update vs Insert)
        menu_base_title = "Update Base Frame" if m_type == "SET_BASE" else "Insert SET_BASE"
        menu_base_icon = '#00e6b8' if m_type == "SET_BASE" else '#d4d4d4'
        menu_base = menu.addMenu(qta.icon('mdi.view-grid-outline', color=menu_base_icon), menu_base_title)
        
        base_actions = {}
        if hasattr(main_win, 'base_manager'):
            for i, b in enumerate(main_win.base_manager.bases):
                if b.get('in_box', True):
                    act = menu_base.addAction(b['name'])
                    base_actions[act] = i
                    
        if m_type != "DELAY":
            action_insert_delay = menu.addAction("Insert DELAY")
            action_insert_delay.triggered.connect(lambda: self._trigger_insert_special("DELAY"))
            
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
        
        if selected == action_copy:
            self.panel.copy_waypoints() 
        elif selected == action_paste:
            self.panel.paste_waypoints(self.index) 
        elif action_base_shift and selected == action_base_shift:
            self.panel.apply_batch_base_shift() 
        elif action_shift_block and selected == action_shift_block:
            self.panel.apply_base_shift_block(self.index) 
        elif selected in base_actions:
            bid = base_actions[selected]
            if m_type == "SET_BASE":
                # 👑 這裡是 Update 的邏輯：覆寫目前這個 SET_BASE 點位，並發送重繪廣播！
                self.wp_data['value'] = bid
                self.wp_data['name'] = main_win.base_manager.bases[bid]['name']
                main_win.path_manager.list_update_signal.emit()
                if hasattr(main_win, 'log_widget'):
                    main_win.log_widget.append_log(f"[System] Updated SET_BASE at line {self.index + 1} to '{self.wp_data['name']}'.")
            else:
                # 這裡是原本 Insert 的邏輯
                self.panel.insert_special_requested.emit(self.index, f"SET_BASE:{bid}")
        elif selected == action_io:
            self.panel.insert_special_requested.emit(self.index, "IO")
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
    insert_pt_requested = Signal(int) 
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

        # 👑 防呆機制：因為 Qt 信號的執行順序是「先綁定先執行」，
        # 我們在這裡攔截儲存按鈕，它會在 gui.py 的儲存動作「之前」自動觸發全域同步！
        self.btn_save.clicked.connect(self.sync_all_base_shifts)

    # ==========================================
    # 👑 陣列加工核心功能區
    # ==========================================
    def copy_waypoints(self):
        main_win = self.window()
        if not hasattr(main_win, 'path_manager'): return

        selected_items = self.path_list.selectedItems()
        indexes = [self.path_list.row(item) for item in selected_items]
        valid_indexes = [i for i in indexes if 0 <= i < len(main_win.path_manager.waypoints)]
        valid_indexes.sort() 

        if not valid_indexes: return

        self.copied_waypoints = [copy.deepcopy(main_win.path_manager.waypoints[i]) for i in valid_indexes]
        if hasattr(main_win, 'log_widget'):
            main_win.log_widget.append_log(f"[System] Copied {len(self.copied_waypoints)} waypoints.")

    def paste_waypoints(self, index=-1):
        if not getattr(self, 'copied_waypoints', []):
            return

        main_win = self.window()
        if not hasattr(main_win, 'path_manager'): return

        target_idx = index if index >= 0 else len(main_win.path_manager.waypoints)

        main_win.path_manager.blockSignals(True)
        curr_idx = target_idx
        for wp in self.copied_waypoints:
            new_wp = copy.deepcopy(wp) 
            main_win.path_manager.insert_waypoint(curr_idx, new_wp)
            curr_idx += 1
        main_win.path_manager.blockSignals(False)
        
        main_win.path_manager.list_update_signal.emit()

        if hasattr(main_win, 'log_widget'):
            main_win.log_widget.append_log(f"[System] Pasted {len(self.copied_waypoints)} waypoints at line {target_idx + 1}")

    def apply_batch_base_shift(self):
        """(保留手動框選功能) 針對已選取的點位套用當前 UI 的 Base"""
        main_win = self.window()
        if not hasattr(main_win, 'base_manager') or not hasattr(main_win, 'path_manager'): return

        selected_items = self.path_list.selectedItems()
        indexes = [self.path_list.row(item) for item in selected_items]
        valid_indices = [i for i in indexes if 0 <= i < len(main_win.path_manager.waypoints) and main_win.path_manager.waypoints[i].get('type') in ["PTP", "LIN", "CIRC"]]

        if not valid_indices: return

        current_base_idx = main_win.base_manager.current_index
        target_base_mat = main_win.base_manager.get_matrix(current_base_idx)
        target_base_name = main_win.base_manager.bases[current_base_idx]['name']

        self._execute_base_shift(valid_indices, target_base_mat, target_base_name)

    def apply_base_shift_block(self, set_base_idx):
        """👑 (新功能) 從指定的 SET_BASE 點位向下掃描，自動同步整個區塊的點位"""
        main_win = self.window()
        if not hasattr(main_win, 'base_manager') or not hasattr(main_win, 'path_manager'): return
        
        wp_list = main_win.path_manager.waypoints
        bid = wp_list[set_base_idx].get('value', 0)
        target_base_mat = main_win.base_manager.get_matrix(bid)
        target_base_name = main_win.base_manager.bases[bid]['name']
        
        # 尋找影響範圍 (直到下一個 SET_BASE 或清單結尾)
        target_indices = []
        for i in range(set_base_idx + 1, len(wp_list)):
            if wp_list[i].get('type') == 'SET_BASE':
                break
            if wp_list[i].get('type') in ["PTP", "LIN", "CIRC"]:
                target_indices.append(i)
                
        if not target_indices:
            main_win.log_widget.append_log("[System] No motion points found in this SET_BASE block.")
            return
            
        self._execute_base_shift(target_indices, target_base_mat, target_base_name)

    def _execute_base_shift(self, indices, target_base_mat, target_base_name):
        """核心共用轉換引擎 (重構版)"""
        import kinematics 
        main_win = self.window()
        wp_list = main_win.path_manager.waypoints

        main_win.path_manager.blockSignals(True)
        success_count = 0
        error_msg = ""
        
        for idx in indices:
            wp = wp_list[idx]
            recorded_base_mat = np.array(wp.get('recorded_base_matrix', np.eye(4)))
            
            if np.allclose(target_base_mat, recorded_base_mat, atol=1e-4):
                continue

            T_flange_old = np.array(wp.get('cartesian_flange', kinematics.forward_kinematics(wp['joints'])))
            new_joints, err = kinematics.calculate_base_shift_ik(
                T_flange_old, recorded_base_mat, target_base_mat, wp['joints']
            )
            
            if new_joints is None:
                error_msg = f"Point {idx+1} IK Failed."
                break
                
            wp['joints'] = list(new_joints)
            wp['cartesian_flange'] = kinematics.forward_kinematics(new_joints).tolist()

            if wp.get('type') == 'CIRC' and 'aux_joints' in wp:
                T_aux_old = np.array(wp.get('aux_cartesian_flange', kinematics.forward_kinematics(wp['aux_joints'])))
                new_aux, err_aux = kinematics.calculate_base_shift_ik(
                    T_aux_old, recorded_base_mat, target_base_mat, wp['aux_joints']
                )
                if new_aux is None:
                    error_msg = f"Point {idx+1} (Aux) IK Failed."
                    break
                wp['aux_joints'] = list(new_aux)
                wp['aux_cartesian_flange'] = kinematics.forward_kinematics(new_aux).tolist()

            wp['recorded_base_matrix'] = target_base_mat.tolist()
            success_count += 1

        main_win.path_manager.blockSignals(False)
        main_win.path_manager.list_update_signal.emit()

        if error_msg:
            main_win.log_widget.append_log(f"[ERROR] Base Shift: {error_msg}")
        elif success_count > 0:
            main_win.log_widget.append_log(f"[System] Successfully shifted {success_count} points to '{target_base_name}'.")
        else:
            main_win.log_widget.append_log(f"[System] Selected points are already in '{target_base_name}'.")

    def sync_all_base_shifts(self):
        """👑 終極防呆：在 Save 觸發前，由上到下掃描整個路徑，確保所有點位與其上方的 SET_BASE 絕對同步"""
        import kinematics
        main_win = self.window()
        if not hasattr(main_win, 'base_manager') or not hasattr(main_win, 'path_manager'): return

        wp_list = main_win.path_manager.waypoints
        if not wp_list: return

        current_base_mat = np.eye(4)
        shifted_count = 0
        error_msg = ""
        
        main_win.path_manager.blockSignals(True)

        for idx, wp in enumerate(wp_list):
            m_type = wp.get('type')
            
            # 遇到 SET_BASE 就更換追蹤的基準矩陣
            if m_type == 'SET_BASE':
                bid = wp.get('value', 0)
                if bid < len(main_win.base_manager.bases):
                    current_base_mat = main_win.base_manager.get_matrix(bid)
                continue

            if m_type not in ["PTP", "LIN", "CIRC"]:
                continue

            recorded_base_mat = np.array(wp.get('recorded_base_matrix', np.eye(4)))
            
            if np.allclose(current_base_mat, recorded_base_mat, atol=1e-4):
                continue

            # 發現沒有對齊的點，立刻執行神級同步
            T_flange_old = np.array(wp.get('cartesian_flange', kinematics.forward_kinematics(wp['joints'])))
            new_joints, err = kinematics.calculate_base_shift_ik(
                T_flange_old, recorded_base_mat, current_base_mat, wp['joints']
            )
            
            if new_joints is None:
                error_msg = f"Point {idx+1} IK Failed during Auto-Sync."
                break
                
            wp['joints'] = list(new_joints)
            wp['cartesian_flange'] = kinematics.forward_kinematics(new_joints).tolist()

            if m_type == 'CIRC' and 'aux_joints' in wp:
                T_aux_old = np.array(wp.get('aux_cartesian_flange', kinematics.forward_kinematics(wp['aux_joints'])))
                new_aux, err_aux = kinematics.calculate_base_shift_ik(
                    T_aux_old, recorded_base_mat, current_base_mat, wp['aux_joints']
                )
                if new_aux is None:
                    error_msg = f"Point {idx+1} (Aux) IK Failed."
                    break
                wp['aux_joints'] = list(new_aux)
                wp['aux_cartesian_flange'] = kinematics.forward_kinematics(new_aux).tolist()

            wp['recorded_base_matrix'] = current_base_mat.tolist()
            shifted_count += 1

        main_win.path_manager.blockSignals(False)
        
        if error_msg:
            main_win.log_widget.append_log(f"[ERROR] Save Auto-Sync: {error_msg}")
        elif shifted_count > 0:
            main_win.log_widget.append_log(f"[System] Auto-synced {shifted_count} points to match their local SET_BASE before saving.")
            main_win.path_manager.list_update_signal.emit()

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
                self.insert_pt_requested.emit,
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
            
        paste_count = len(getattr(self, 'copied_waypoints', []))
        paste_text = f"Paste Copied ({paste_count})" if paste_count > 1 else "Paste Copied"
        action_paste = menu.addAction(qta.icon('mdi.content-paste', color='#00e6b8'), paste_text)
        action_paste.setEnabled(paste_count > 0)
        menu.addSeparator()

        menu_pt = menu.addMenu(qta.icon('mdi.map-marker-plus', color='#00e6b8'), "Record Current Position")
        pt_actions = {} 
        for m_type in ["PTP", "LIN", "CIRC"]:
            action = menu_pt.addAction(m_type)
            pt_actions[action] = m_type
            
        menu.addSeparator()
        
        action_delay = menu.addAction(qta.icon('mdi.timer-outline', color='#e0e0e0'), "Append Delay")
        action_io = menu.addAction(qta.icon('mdi.power-plug-outline', color='#e0e0e0'), "Append I/O")
        
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

        menu_base = menu.addMenu(qta.icon('mdi.view-grid-outline', color='#00e6b8'), "Append SET_BASE")
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
            self.paste_waypoints(-1) 
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
        self.path_list.selectionModel().setCurrentIndex(
            model_idx, 
            QItemSelectionModel.SelectionFlag.ClearAndSelect
        )