# waypoint_panel.py
from PySide6.QtWidgets import (QApplication, QInputDialog, QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem, 
                               QWidget, QLabel, QPushButton, QSizePolicy, QMenu, QFrame, QMessageBox)
from PySide6.QtCore import QItemSelectionModel, Qt, Signal, QSize
from PySide6.QtGui import QCursor, QShortcut, QKeySequence 
import qtawesome as qta

import styles
from widgets import BaseBlock, apply_windows_dark_titlebar


# =========================================================
# [1] 基礎與分頁元件 (Tabs & Base UI)
# =========================================================
class EditorTab(QFrame):
    """分頁標籤元件：負責儲存單一檔案的暫存路徑資料與視覺狀態"""
    closed = Signal(object)
    clicked_sig = Signal(object)

    def __init__(self, title="untitled.json", parent=None):
        super().__init__(parent)
        self.waypoints_data = []  
        self.is_modified = False  
        self.filepath = ""        
        self.title = title
        
        self.setFixedHeight(26) 
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        
        self.layout = QHBoxLayout(self)
        self.layout.setContentsMargins(12, 0, 4, 0)
        self.layout.setSpacing(6)
        
        self.lbl = QLabel(title)
        self.lbl.setStyleSheet(styles.TAB_TITLE_STYLE)
        self.layout.addWidget(self.lbl)
        
        self.btn_close = QPushButton()
        self.btn_close.setIcon(qta.icon('mdi.close', color='#cccccc'))
        self.btn_close.setIconSize(QSize(14, 14))
        self.btn_close.setFixedSize(20, 20)
        self.btn_close.setStyleSheet(styles.TAB_CLOSE_BTN_STYLE)
        self.btn_close.clicked.connect(lambda: self.closed.emit(self))
        self.layout.addWidget(self.btn_close)
        
    def set_active(self, active):
        """切換分頁的視覺活躍狀態"""
        if active:
            self.setStyleSheet(styles.EDITOR_TAB_ACTIVE_STYLE)
        else:
            self.setStyleSheet(styles.EDITOR_TAB_INACTIVE_STYLE)
            
    def mousePressEvent(self, event):
        """攔截點擊事件，觸發分頁切換訊號"""
        if getattr(self, 'is_locked', False): return

        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked_sig.emit(self)
        super().mousePressEvent(event)


class DoubleClickLabel(QLabel):
    """自訂標籤：雙擊時會觸發下拉選單，用於快速修改軌跡參數"""
    def __init__(self, text, options, on_change_callback, parent=None):
        super().__init__(text, parent)
        self.options = options
        self.on_change = on_change_callback

    def mouseDoubleClickEvent(self, event):
        """攔截雙擊，展開選項選單 (若 options 為空則無效化)"""
        if getattr(self, 'is_locked', False) or not self.options: return
        
        if event.button() == Qt.MouseButton.LeftButton:
            menu = QMenu()
            menu.setStyleSheet(styles.MENU_STYLE)
            for opt in self.options:
                action = menu.addAction(opt)
                action.triggered.connect(lambda checked, val=opt: self._handle_selection(val))
            menu.exec(QCursor.pos())

    def _handle_selection(self, val):
        """傳遞選擇結果給外部的回呼函式"""
        self.on_change(val)


# =========================================================
# [2] 單行點位 UI (Waypoint Row Widget)
# =========================================================
class WaypointRowWidget(QWidget):
    """渲染於 ListWidget 內的單行路徑面板 (具備元件池更新能力)"""
    toggle_sig = Signal(int)
    data_changed_sig = Signal()
    menu_requested_sig = Signal(int)

    def __init__(self, index, wp_data, parent=None):
        super().__init__(parent)
        self._lockable_widgets = []
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(10)
        
        # --- UI 實體宣告 (僅在此處建立一次，供後續重複使用) ---
        self.lbl_idx = QLabel()
        self.lbl_idx.setFixedWidth(25) 
        self.lbl_idx.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(self.lbl_idx)
        
        # 類型標籤統一使用 DoubleClickLabel，依賴 options 來決定是否可編輯
        self.lbl_type = DoubleClickLabel("", [], self._change_type)
        self.lbl_type.setFixedWidth(50) 
        layout.addWidget(self.lbl_type)

        self.lbl_info = QLabel()
        self.lbl_info.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        layout.addWidget(self.lbl_info)
        
        # 參數標籤
        self.lbl_blend = DoubleClickLabel("", ["FINE", "10%", "25%", "50%", "75%", "100%"], self._change_blend)
        self.lbl_blend.setFixedWidth(50)
        layout.addWidget(self.lbl_blend)
        
        self.lbl_spd = DoubleClickLabel("", ["10%", "25%", "50%", "75%", "100%"], self._change_speed)
        self.lbl_spd.setFixedWidth(50)
        layout.addWidget(self.lbl_spd)
        
        self.lbl_acc = DoubleClickLabel("", ["10%", "25%", "50%", "75%", "100%"], self._change_accel)
        self.lbl_acc.setFixedWidth(50)
        layout.addWidget(self.lbl_acc)

        # 控制按鈕
        self.btn_eye = QPushButton()
        self.btn_eye.setIconSize(QSize(18, 18))
        self.btn_eye.setFixedSize(26, 26)
        self.btn_eye.setStyleSheet(styles.WAYPOINT_ROW_BTN_STYLE) 
        self.btn_eye.clicked.connect(lambda: self.toggle_sig.emit(self.index))
        layout.addWidget(self.btn_eye)
        
        self.btn_menu = QPushButton()
        self.btn_menu.setIcon(qta.icon('mdi.dots-vertical', color='#888888'))
        self.btn_menu.setIconSize(QSize(18, 18))
        self.btn_menu.setFixedSize(26, 26)
        self.btn_menu.setStyleSheet(styles.WAYPOINT_ROW_BTN_STYLE) 
        self.btn_menu.clicked.connect(lambda: self.menu_requested_sig.emit(self.index))
        layout.addWidget(self.btn_menu)
        
        # 註冊所有需要響應系統鎖定的元件
        self._lockable_widgets.extend([self.lbl_type, self.lbl_blend, self.lbl_spd, self.lbl_acc, self.btn_eye, self.btn_menu])
        
        # 初次寫入資料
        self.update_content(index, wp_data)

    def update_content(self, index, wp_data):
        """動態更新現有元件內容，實現元件池 (Widget Pooling) 提升渲染效能"""
        self.index = index
        self.wp_data = wp_data
        
        active = wp_data.get("active", True)
        m_type = wp_data.get("type", "PTP")
        
        # --- 1. 狀態與顏色判定 ---
        text_color = "#cccccc" if active else "#666666"
        if active:
            if m_type in ["LOOP_START", "LOOP_END"]: text_color = "#d7ba7d"
            elif m_type in ["COMMENT"]: text_color = "#6a9955"
            elif m_type == "SET_TCP": text_color = "#e6a800"
            elif m_type == "SET_BASE": text_color = "#00a8e6" 
            elif m_type == "CAM_PATH": text_color = "#8A2BE2"

        font_style = f"color: {text_color}; {styles.WAYPOINT_FONT_BASE}"
        clickable_style = font_style  
        
        # --- 2. 序號與類型更新 ---
        self.lbl_idx.setText(f"{index+1}.")
        self.lbl_idx.setStyleSheet(font_style)
        
        if m_type in ["PTP", "LIN", "CIRC"]:
            self.lbl_type.options = ["PTP", "LIN", "CIRC"]
            self.lbl_type.setStyleSheet(clickable_style)
        else:
            self.lbl_type.options = [] # 清空選項即可防呆雙擊事件
            self.lbl_type.setStyleSheet(font_style)
            
        display_type = "CAM" if m_type == "CAM_PATH" else m_type
        self.lbl_type.setText(display_type)
        
        # --- 3. 資訊與參數更新 ---
        self.lbl_info.setStyleSheet(font_style)
        self.lbl_blend.setStyleSheet(clickable_style)
        self.lbl_spd.setStyleSheet(clickable_style)
        self.lbl_acc.setStyleSheet(clickable_style)
        
        if m_type == "DELAY":
            self.lbl_info.setText(f"Wait {wp_data.get('value', 0.0):.1f}s")
        elif m_type == "I/O":
            self.lbl_info.setText(f"{wp_data.get('action_type', '')}={wp_data.get('value', 0)}")
        elif m_type in ["SET_TCP", "SET_BASE"]:
            self.lbl_info.setText(f"[{wp_data.get('value')}] {wp_data.get('name')}")
        elif m_type in ["COMMENT", "LOOP_START", "LOOP_END"]:
            self.lbl_info.setText(f"// {wp_data.get('value', '')}")
        elif m_type == "CAM_PATH":
            pt_count = wp_data.get("point_count", 0)
            self.lbl_info.setText(f"{wp_data.get('name', 'CAM Path')} ({pt_count} pts)")
        else:
            self.lbl_info.setText(wp_data.get('name', f'Point {index+1}'))
            self.lbl_blend.setText(f"b:{wp_data.get('blend', 'FINE')}")
            self.lbl_spd.setText(f"v:{int(wp_data.get('speed', 50))}%")
            self.lbl_acc.setText(f"a:{int(wp_data.get('accel', 50))}%")

        # 隱藏非運動軌跡點的參數標籤
        NO_PARAM_TYPES = ["DELAY", "I/O", "SET_TCP", "SET_BASE", "COMMENT", "LOOP_START", "LOOP_END", "CAM_PATH"] 
        show_params = m_type not in NO_PARAM_TYPES
        self.lbl_blend.setVisible(show_params)
        self.lbl_spd.setVisible(show_params)
        self.lbl_acc.setVisible(show_params)

        # --- 4. 眼睛狀態切換 ---
        eye_color = "#cccccc" if active else "#555555"
        eye_icon = 'mdi.eye-outline' if active else 'mdi.eye-off-outline'
        self.btn_eye.setIcon(qta.icon(eye_icon, color=eye_color))

    def set_locked(self, locked):
        """切斷神經版上鎖：保留視覺互動性，但阻擋功能訊號的發送"""
        for widget in self._lockable_widgets:
            if isinstance(widget, DoubleClickLabel):
                widget.is_locked = locked
            else:
                widget.blockSignals(locked)

    def _change_type(self, new_val):
        self.wp_data['type'] = new_val
        self.lbl_type.setText(new_val)
        self.data_changed_sig.emit()

    def _change_blend(self, new_val):
        self.wp_data['blend'] = new_val
        self.lbl_blend.setText(f"b:{new_val}")
        self.data_changed_sig.emit()

    def _change_speed(self, new_val):
        val = float(new_val.replace('%', ''))
        self.wp_data['speed'] = val
        self.lbl_spd.setText(f"v:{int(val)}%") 
        self.data_changed_sig.emit()

    def _change_accel(self, new_val):
        val = float(new_val.replace('%', ''))
        self.wp_data['accel'] = val
        self.lbl_acc.setText(f"a:{int(val)}%") 
        self.data_changed_sig.emit()


# =========================================================
# [3] 路徑清單主面板 (Waypoint Panel)
# =========================================================
class WaypointPanel(BaseBlock):
    """路徑點管理主面板：整合分頁系統、ListWidget 及所有清單操作"""
    # --- 信號定義 ---
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
    copy_requested = Signal(list)       
    paste_requested = Signal(int)       
    batch_base_shift_requested = Signal(list) 
    block_base_shift_requested = Signal(int)  

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

        # --- 分頁列區塊 ---
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
        self.btn_new_tab.setStyleSheet(styles.TAB_NEW_BTN_STYLE)
        self.btn_new_tab.clicked.connect(self.add_new_tab)
        self.tab_bar_layout.addWidget(self.btn_new_tab)

        self.tab_bar_layout.addStretch() 
        main_layout.addWidget(self.tab_bar)

        # --- 核心清單建立 ---
        self.path_list = QListWidget()
        self.path_list.setStyleSheet(styles.PATH_LIST_STYLE) 
        self.path_list.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.path_list.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)
        self.path_list.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.path_list.customContextMenuRequested.connect(self.show_list_context_menu)
        main_layout.addWidget(self.path_list)

        # 快捷鍵
        self.shortcut_delete = QShortcut(QKeySequence(Qt.Key.Key_Delete), self.path_list)
        self.shortcut_delete.activated.connect(self._handle_delete_key)

        self.shortcut_up = QShortcut(QKeySequence(Qt.Key.Key_Up), self.path_list)
        self.shortcut_up.activated.connect(lambda: self._navigate_list(-1))
        
        self.shortcut_down = QShortcut(QKeySequence(Qt.Key.Key_Down), self.path_list)
        self.shortcut_down.activated.connect(lambda: self._navigate_list(1))
        
        # --- 下方功能按鈕綁定 ---
        self.btn_save = self.nav_bar.nav_buttons[0]
        self.btn_load = self.nav_bar.nav_buttons[1]
        self.btn_clear = self.nav_bar.nav_buttons[2]
        self.btn_menu = self.nav_bar.nav_buttons[3]

        self.btn_save.setToolTip("Save Path")
        self.btn_load.setToolTip("Load Path")
        self.btn_clear.setToolTip("Clear All Waypoints")
        self._active_menu = None
        self.btn_clear.clicked.connect(self.confirm_clear_all)

    # =========================================================
    # 選單兵工廠 (Menu Builders)
    # =========================================================
    def _build_tcp_submenu(self, parent_menu, title, icon_color):
        """建立 TCP 設定的下拉子選單"""
        menu_tcp = parent_menu.addMenu(qta.icon('mdi.wrench-outline', color=icon_color), title)
        tcp_actions = {}
        main_win = self.window()
        box_tools = [(i, t['name']) for i, t in enumerate(main_win.tcp_manager.tools) if t.get('in_box', False)]
        
        if not box_tools:
            menu_tcp.addAction("Tool Box is Empty").setEnabled(False)
        else:
            for t_idx, t_name in box_tools:
                act = menu_tcp.addAction(qta.icon('mdi.wrench-outline', color='#d4d4d4'), t_name)
                tcp_actions[act] = t_idx 
        return tcp_actions

    def _build_base_submenu(self, parent_menu, title, icon_color):
        """建立 Base 設定的下拉子選單"""
        menu_base = parent_menu.addMenu(qta.icon('mdi.view-grid-outline', color=icon_color), title)
        base_actions = {}
        main_win = self.window()
        box_bases = [(i, b['name']) for i, b in enumerate(main_win.base_manager.bases) if b.get('in_box', True)]
        
        if not box_bases:
            menu_base.addAction("Base Box is Empty").setEnabled(False)
        else:
            for b_idx, b_name in box_bases:
                act = menu_base.addAction(qta.icon('mdi.grid', color='#d4d4d4'), b_name)
                base_actions[act] = b_idx
        return base_actions

    def set_locked(self, locked):
        """切斷全面板的輸入訊號"""
        self.is_locked = locked
        
        for i in range(self.path_list.count()):
            item = self.path_list.item(i)
            widget = self.path_list.itemWidget(item)
            if hasattr(widget, 'set_locked'):
                widget.set_locked(locked)
                
        for btn in getattr(self.nav_bar, 'nav_buttons', []):
            btn.blockSignals(locked)
            
        self.btn_new_tab.blockSignals(locked)
        for tab in self.tabs:
            tab.is_locked = locked
            tab.btn_close.blockSignals(locked)
            
        self.shortcut_delete.setEnabled(not locked)
        self.shortcut_up.setEnabled(not locked)
        self.shortcut_down.setEnabled(not locked)

    def _handle_delete_key(self):
        """處理鍵盤 Delete 刪除點位事件"""
        selected_items = self.path_list.selectedItems()
        if not selected_items: return

        main_win = self.window()
        indexes = [self.path_list.row(item) for item in selected_items]
        valid_indexes = [i for i in indexes if 0 <= i < len(main_win.path_manager.waypoints)]
        valid_indexes.sort(reverse=True) 

        if not valid_indexes: return

        main_win.path_manager.blockSignals(True)
        for idx in valid_indexes:
            main_win.path_manager.delete_point(idx)
        main_win.path_manager.blockSignals(False)
        
        main_win.path_manager.list_update_signal.emit()

    def _navigate_list(self, step):
        """處理鍵盤上下鍵：統一計算並切換選取的點位 (step: -1 為上，1 為下)"""
        focus_w = QApplication.focusWidget()
        if focus_w and (focus_w.inherits("QLineEdit") or focus_w.inherits("QAbstractSpinBox") or focus_w.inherits("QTextEdit")):
            return

        if not self.active_tab: return
        
        curr = self.path_list.currentRow()
        max_valid = self.path_list.count() - 2  # 防呆：扣除墊底用的隱形 Spacer
        
        # 如果目前沒選中任何東西，且清單有內容，預設跳到第 0 項
        if curr == -1:
            if self.path_list.count() > 1:
                self.path_list.setCurrentRow(0)
            return
            
        # 計算下一步並過濾掉越界的情況
        new_row = curr + step
        if 0 <= new_row <= max_valid:
            self.path_list.setCurrentRow(new_row)

    def confirm_clear_all(self):
        """彈出確認視窗，刪除當前分頁的所有點位"""
        if not self.active_tab: return
            
        msg_box = QMessageBox(self.window()) 
        apply_windows_dark_titlebar(msg_box)
        msg_box.setWindowTitle("Clear All Waypoints")
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setText("Are you sure you want to delete ALL waypoints in this tab?\nThis action cannot be undone.")
        msg_box.setStyleSheet(styles.DARK_MESSAGE_BOX_STYLE)
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel)
        
        yes_btn = msg_box.button(QMessageBox.StandardButton.Yes)
        yes_btn.setText("Delete All")
        yes_btn.setStyleSheet(styles.WARNING_BTN_STYLE)
        
        if msg_box.exec() == QMessageBox.StandardButton.Yes:
            self.clear_all_requested.emit()

    def _trigger_delay_edit(self, index):
        """開啟延遲時間編輯對話框"""
        main_win = self.window()
        wp_data = main_win.path_manager.waypoints[index]
        old_val = wp_data.get("value", 0.0)
        new_val, ok = QInputDialog.getDouble(self, "Edit Delay", "Enter seconds:", old_val, 0, 3600, 1)
        if ok:
            wp_data["value"] = new_val
            self.data_changed.emit()

    def _handle_spacebar(self):
        """處理全域空白鍵，呼出整體路徑選單"""
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

    # --- 分頁管理 ---
    def add_new_tab(self, name="untitled.json"):
        """新增並跳轉至新分頁"""
        if not isinstance(name, str) or not name: name = "untitled.json"
            
        tab = EditorTab(name)
        tab.closed.connect(self.close_tab)
        tab.clicked_sig.connect(self.request_tab_switch)
        
        self.tabs.append(tab)
        self.tabs_layout.addWidget(tab)
        self.request_tab_switch(tab)

    def request_tab_switch(self, new_tab):
        """請求切換分頁，並透過淺拷貝消除 Deepcopy 效能風暴"""
        if new_tab != self.active_tab:
            main_win = self.window()
            
            if self.active_tab:
                self.active_tab.waypoints_data = main_win.path_manager.waypoints[:]
                self.active_tab.is_modified = main_win.path_manager.is_modified
            
            self.set_active_tab_visuals(new_tab)
            
            main_win.path_manager.blockSignals(True)
            main_win.path_manager.waypoints = new_tab.waypoints_data[:]
            main_win.path_manager.is_modified = new_tab.is_modified
            main_win.path_manager.blockSignals(False)
            
            self.tab_switch_requested.emit(new_tab)

    def set_active_tab_visuals(self, tab):
        """更新分頁的主題外觀"""
        for t in self.tabs:
            t.set_active(t == tab)
        self.active_tab = tab
        self._update_theme()

    def close_tab(self, tab):
        self.tab_closed_signal.emit(tab)

    def force_close_tab(self, tab):
        """無條件強制關閉分頁並清理記憶體"""
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
        """當清空所有分頁時，隱藏面板並顯示空背景"""
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
        """將存檔檔名寫入目前活躍的分頁"""
        if self.active_tab:
            self.active_tab.lbl.setText(filename)
            self.active_tab.title = filename
        else:
            self.add_new_tab(filename)

    def update_list(self, waypoints):
        """完全重構：導入元件池 (Widget Pooling)，徹底消除高頻重建帶來的 UI 卡頓"""
        if not self.active_tab:
            return 
            
        v_bar = self.path_list.verticalScrollBar()
        current_scroll = v_bar.value() if v_bar else 0
        current_row = self.path_list.currentRow()
            
        self.path_list.blockSignals(True)
        
        if self.path_list.count() > 0:
            last_item = self.path_list.item(self.path_list.count() - 1)
            if not self.path_list.itemWidget(last_item):
                self.path_list.takeItem(self.path_list.count() - 1)

        for i, wp in enumerate(waypoints):
            if i < self.path_list.count():
                item = self.path_list.item(i)
                widget = self.path_list.itemWidget(item)
                widget.update_content(i, wp)
            else:
                item = QListWidgetItem()
                row_widget = WaypointRowWidget(i, wp)
                
                row_widget.toggle_sig.connect(self.toggle_requested.emit)
                row_widget.data_changed_sig.connect(self.data_changed.emit)
                row_widget.menu_requested_sig.connect(self.show_row_context_menu)
                
                item.setSizeHint(row_widget.sizeHint())
                self.path_list.addItem(item)
                self.path_list.setItemWidget(item, row_widget)
                
        while self.path_list.count() > len(waypoints):
            self.path_list.takeItem(self.path_list.count() - 1)
            
        is_sys_locked = getattr(self, 'is_locked', False)
        for i in range(self.path_list.count()):
            widget = self.path_list.itemWidget(self.path_list.item(i))
            if widget: widget.set_locked(is_sys_locked)

        spacer_item = QListWidgetItem()
        spacer_item.setSizeHint(QSize(0, 510))
        spacer_item.setFlags(Qt.ItemFlag.NoItemFlags)  
        self.path_list.addItem(spacer_item)
        self.path_list.blockSignals(False)

        if current_row >= 0 and current_row < len(waypoints):
            self.select_row_silently(current_row)
            
        if v_bar:
            v_bar.setValue(current_scroll)

    # =========================================================
    # 選單呼叫與派發 (Context Menus)
    # =========================================================
    def show_row_context_menu(self, index):
        """由主面板全權接管：顯示單行點位專屬的三點選單"""
        main_win = self.window()
        wp_data = main_win.path_manager.waypoints[index]
        m_type = wp_data.get("type", "PTP")
        sel_count = len(self.path_list.selectedItems())
        
        menu = QMenu(self.window()) 
        menu.setStyleSheet(styles.MENU_STYLE)
        menu.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        
        # --- 剪貼與基座操作 ---
        copy_text = f"Copy ({sel_count})" if sel_count > 1 else "Copy"
        action_copy = menu.addAction(qta.icon('mdi.content-copy', color='#00e6b8'), copy_text)
        
        clipboard = main_win.path_manager.clipboard
        paste_count = len(clipboard)
        paste_text = f"Insert Copied ({paste_count})" if paste_count > 1 else "Insert Copied"
        action_paste = menu.addAction(qta.icon('mdi.content-paste', color='#00e6b8'), paste_text)
        if paste_count == 0: action_paste.setEnabled(False)
            
        menu.addSeparator()
        
        action_shift_block = None
        if m_type == "SET_BASE":
            action_shift_block = menu.addAction(qta.icon('mdi.axis-arrow', color='#e6a800'), "Sync Base Shift to Following Points")
            menu.addSeparator()

        # --- 編輯與更新操作 ---
        action_update = None
        action_edit = None
        if m_type in ["PTP", "LIN", "CIRC"]:
            action_update = menu.addAction("Update Position")
            menu.addSeparator()
        elif m_type == "DELAY":
            action_edit = menu.addAction("Edit Delay Time")
            menu.addSeparator()
                        
        # --- 插入特定指令 ---
        menu_insert_pt = menu.addMenu(qta.icon('mdi.map-marker-plus', color='#e0e0e0'), "Insert Current Position")
        insert_pt_actions = {}
        for pt_type in ["PTP", "LIN", "CIRC"]:
            action = menu_insert_pt.addAction(pt_type)
            insert_pt_actions[action] = pt_type
            
        menu_tcp_title = "Update from Tool Box" if m_type == "SET_TCP" else "Insert SET_TCP"
        menu_tcp_icon = '#e6a800' if m_type == "SET_TCP" else '#d4d4d4'
        tcp_actions = self._build_tcp_submenu(menu, menu_tcp_title, menu_tcp_icon)
            
        menu_base_title = "Update Base Frame" if m_type == "SET_BASE" else "Insert SET_BASE"
        menu_base_icon = '#00a8e6' if m_type == "SET_BASE" else '#d4d4d4'
        base_actions = self._build_base_submenu(menu, menu_base_title, menu_base_icon)
            
        action_insert_delay = None
        if m_type != "DELAY":
            action_insert_delay = menu.addAction(qta.icon('mdi.timer-outline', color='#e0e0e0'), "Insert Delay")
            
        action_io = menu.addAction(qta.icon('mdi.power-plug-outline', color='#d4d4d4'), "Insert I/O")
        
        menu.addSeparator()
        
        # --- 刪除操作 ---
        del_text = f"Delete ({sel_count})" if sel_count > 1 else "Delete"
        action_delete = menu.addAction(qta.icon('mdi.trash-can-outline', color='#ff4d4d'), del_text)
        
        # ==========================
        # 執行動作配發
        # ==========================
        selected = menu.exec(QCursor.pos())
        if not selected: return
        
        if action_update and selected == action_update:
            self.update_pt_requested.emit(index)
        elif action_edit and selected == action_edit:
            self._trigger_delay_edit(index)

        elif selected == action_copy:
            indexes = [self.path_list.row(item) for item in self.path_list.selectedItems()]
            self.copy_requested.emit(indexes) 
        elif selected == action_paste:
            self.paste_requested.emit(index) 
        elif action_shift_block and selected == action_shift_block:
            self.block_base_shift_requested.emit(index)
            
        elif selected in insert_pt_actions:
            self.record_pt_requested.emit(index, insert_pt_actions[selected])
            
        elif selected in tcp_actions:
            tid = tcp_actions[selected]
            if m_type == "SET_TCP":
                self.update_tcp_point_requested.emit(index, tid)
            else:
                self.insert_special_requested.emit(index, f"SET_TCP:{tid}")
                
        elif selected in base_actions:
            bid = base_actions[selected]
            if m_type == "SET_BASE":
                wp_data['value'] = bid
                wp_data['name'] = main_win.base_manager.bases[bid]['name'] 
                main_win.path_manager.list_update_signal.emit()
                main_win.log_widget.append_log(f"[System] Updated SET_BASE at line {index + 1} to '{wp_data['name']}'.") 
            else:
                self.insert_special_requested.emit(index, f"SET_BASE:{bid}")
                
        elif action_insert_delay and selected == action_insert_delay:
            self.insert_special_requested.emit(index, "DELAY")
            
        elif selected == action_io:
            self.insert_special_requested.emit(index, "IO")
            
        elif selected == action_delete:
            self._handle_delete_key()

    def show_list_context_menu(self, pos=None):
        """顯示清單空白處的全局右鍵選單"""
        if getattr(self, 'is_locked', False): return
        
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
        
        main_win = self.window()
        target_idx = len(main_win.path_manager.waypoints)
            
        clipboard = main_win.path_manager.clipboard 
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
        
        tcp_actions = self._build_tcp_submenu(menu, "Append SET_TCP", '#e6a800')
        base_actions = self._build_base_submenu(menu, "Append SET_BASE", '#00a8e6')
                
        # ==========================
        # 執行動作配發
        # ==========================
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
        """安靜地選擇指定列，不觸發多餘的 UI 連動訊號"""
        if index < 0 or index >= self.path_list.count():
            return
            
        model_idx = self.path_list.model().index(index, 0)
        
        self.path_list.blockSignals(True)
        self.path_list.selectionModel().setCurrentIndex(
            model_idx, 
            QItemSelectionModel.SelectionFlag.ClearAndSelect
        )
        self.path_list.blockSignals(False)