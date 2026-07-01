import re
from PySide6.QtWidgets import (QMenu, QWidget, QVBoxLayout, QHBoxLayout, QStackedWidget, 
                               QListWidget, QListWidgetItem, QTabWidget, QPushButton, 
                               QLabel, QPlainTextEdit, QMessageBox) # 💥 新增 QMessageBox
from PySide6.QtCore import Qt, QSize, QRect
from PySide6.QtGui import QAction, QCursor, QPainter, QColor, QSyntaxHighlighter, QTextCharFormat, QFont
import qtawesome as qta

import styles
from widgets import BaseBlock

# ==========================================
# 專屬語法高亮解析器 (VS Code 風格)
# ==========================================
class ScriptHighlighter(QSyntaxHighlighter):
    def __init__(self, document):
        super().__init__(document)
        self.highlighting_rules = []

        # 1. 關鍵字 (藍色)
        keyword_format = QTextCharFormat()
        keyword_format.setForeground(QColor("#569CD6")) 
        keyword_format.setFontWeight(QFont.Bold)
        keywords = [
            "MoveJoints", "MovePose", "MoveCirc", "Delay",
            "SetJointVel", "SetJointAcc", "SetBlend", "SetIO",
            "MoveGripper", "SetGripperVel", "Loop", "EndLoop",
            "SetTrf", "SetWrf"
        ]
        for word in keywords:
            self.highlighting_rules.append((rf'\b{word}\b', keyword_format))

        # 2. 數字 (淺綠色)
        number_format = QTextCharFormat()
        number_format.setForeground(QColor("#B5CEA8")) 
        self.highlighting_rules.append((r'\b[-+]?[0-9]*\.?[0-9]+\b', number_format))

        # 3. 字串 (橘色)
        string_format = QTextCharFormat()
        string_format.setForeground(QColor("#CE9178")) 
        self.highlighting_rules.append((r'".*"', string_format))
        self.highlighting_rules.append((r"'.*'", string_format))

        # 4. 註解 (深綠色斜體)
        comment_format = QTextCharFormat()
        comment_format.setForeground(QColor("#6A9955")) 
        comment_format.setFontItalic(True)
        self.highlighting_rules.append((r'#.*', comment_format))

        # 5. 參數高亮 (淺藍色)
        kwarg_format = QTextCharFormat()
        kwarg_format.setForeground(QColor("#9CDCFE"))
        self.highlighting_rules.append((r'\b(v|a|blend)\b\s*=', kwarg_format))

    def highlightBlock(self, text):
        for pattern, format in self.highlighting_rules:
            for match in re.finditer(pattern, text):
                self.setFormat(match.start(), match.end() - match.start(), format)

# ==========================================
# 負責繪製左側行號的畫布 (升級 Hover 懸停偵測)
# ==========================================
class LineNumberArea(QWidget):
    def __init__(self, editor):
        super().__init__(editor)
        self.code_editor = editor
        self.setMouseTracking(True) 

    def sizeHint(self):
        return QSize(self.code_editor.line_number_area_width(), 0)

    def paintEvent(self, event):
        self.code_editor.lineNumberAreaPaintEvent(event)
        
    def mousePressEvent(self, event):
        self.code_editor.lineNumberAreaMousePressEvent(event)

    def enterEvent(self, event):
        self.code_editor.is_hovering_line_area = True
        self.update()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.code_editor.is_hovering_line_area = False
        self.update()
        super().leaveEvent(event)

# ==========================================
# 自帶行號與「懸停程式碼摺疊」的編輯器
# ==========================================
class CodeTextEdit(QPlainTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.line_number_area = LineNumberArea(self)
        self.blockCountChanged.connect(self.update_line_number_area_width)
        self.updateRequest.connect(self.update_line_number_area)
        self.update_line_number_area_width(0)
        
        self.folded_blocks = set() 
        self.highlighter = ScriptHighlighter(self.document())
        
        self.is_hovering_line_area = False
        self.icon_folded = qta.icon('mdi.chevron-right', color='#858585')
        self.icon_expanded = qta.icon('mdi.chevron-down', color='#858585')

    def line_number_area_width(self):
        digits = 1
        max_val = max(1, self.blockCount())
        while max_val >= 10:
            max_val //= 10
            digits += 1
            
        space = 10 + self.fontMetrics().horizontalAdvance('9') * digits + 18
        return space

    def update_line_number_area_width(self, _):
        self.setViewportMargins(self.line_number_area_width(), 0, 0, 40)

    def update_line_number_area(self, rect, dy):
        if dy: self.line_number_area.scroll(0, dy)
        else: self.line_number_area.update(0, rect.y(), self.line_number_area.width(), rect.height())
        if rect.contains(self.viewport().rect()): self.update_line_number_area_width(0)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.contentsRect()
        safe_height = max(0, cr.height() - 40) 
        self.line_number_area.setGeometry(QRect(cr.left(), cr.top(), self.line_number_area_width(), safe_height))

    def lineNumberAreaMousePressEvent(self, event):
        cursor = self.cursorForPosition(event.pos())
        block = cursor.block()
        block_num = block.blockNumber()
        
        next_block = block.next()
        if next_block.isValid() and (next_block.text().startswith(" ") or next_block.text().startswith("\t")):
            if block_num in self.folded_blocks:
                self.folded_blocks.remove(block_num)
                while next_block.isValid() and (next_block.text().startswith(" ") or next_block.text().startswith("\t")):
                    next_block.setVisible(True)
                    next_block = next_block.next()
            else:
                self.folded_blocks.add(block_num)
                while next_block.isValid() and (next_block.text().startswith(" ") or next_block.text().startswith("\t")):
                    next_block.setVisible(False)
                    next_block = next_block.next()
                    
            self.document().markContentsDirty(block.position(), self.document().characterCount())
            self.viewport().update()
            self.line_number_area.update()

    def lineNumberAreaPaintEvent(self, event):
        painter = QPainter(self.line_number_area)
        painter.fillRect(event.rect(), QColor("#1e1e1e")) 
        
        block = self.firstVisibleBlock()
        block_number = block.blockNumber()
        top = round(self.blockBoundingGeometry(block).translated(self.contentOffset()).top())
        bottom = top + round(self.blockBoundingRect(block).height())

        area_width = self.line_number_area.width()
        chevron_area_width = 18

        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                number = str(block_number + 1)
                
                painter.setPen(QColor("#858585")) 
                painter.drawText(0, top, area_width - chevron_area_width - 4, self.fontMetrics().height(),
                                 Qt.AlignRight | Qt.AlignVCenter, number)
                                 
                next_block = block.next()
                has_child = next_block.isValid() and (next_block.text().startswith(" ") or next_block.text().startswith("\t"))
                
                if has_child:
                    is_folded = block_number in self.folded_blocks
                    if is_folded or self.is_hovering_line_area:
                        icon = self.icon_folded if is_folded else self.icon_expanded
                        icon.paint(painter, 
                                   area_width - chevron_area_width, top, 
                                   chevron_area_width, self.fontMetrics().height(), 
                                   Qt.AlignCenter)
                                     
            block = block.next()
            top = bottom
            bottom = top + round(self.blockBoundingRect(block).height())
            block_number += 1

# ==========================================
# 路徑清單專用行內控制組件 (Row Widget)
# ==========================================
class WaypointRowWidget(QWidget):
    def __init__(self, text, active, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(12)
        
        self.lbl_text = QLabel(text)
        if active:
            self.lbl_text.setStyleSheet("color: #d4d4d4; font-family: 'Consolas', 'Segoe UI'; font-size: 12px;")
        else:
            self.lbl_text.setStyleSheet("color: #666666; font-family: 'Consolas', 'Segoe UI'; font-size: 12px; font-style: italic;")
            
        layout.addWidget(self.lbl_text)
        layout.addStretch(1) 
        
        self.btn_eye = QPushButton()
        self.btn_eye.setIcon(qta.icon('mdi.eye-circle-outline', color='#e0e0e0' if active else '#555555'))
        self.btn_eye.setFixedSize(22, 22)
        self.btn_eye.setCursor(Qt.PointingHandCursor)
        self.btn_eye.setStyleSheet(styles.WAYPOINT_ROW_BTN_STYLE)
        layout.addWidget(self.btn_eye)
        
        self.btn_trash = QPushButton()
        self.btn_trash.setIcon(qta.icon('mdi.trash-can-outline', color='#e0e0e0'))
        self.btn_trash.setFixedSize(22, 22)
        self.btn_trash.setCursor(Qt.PointingHandCursor)
        self.btn_trash.setStyleSheet(styles.WAYPOINT_ROW_BTN_STYLE)
        layout.addWidget(self.btn_trash)


class EditorTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.stack = QStackedWidget()
        
        # --- [0] 程式碼模式 ---
        self.code_editor = CodeTextEdit()
        self.code_editor.setFont(styles.FONT_CODE)
        self.code_editor.setStyleSheet(styles.EDITOR_VIEW_STYLE)
        self.code_editor.setPlainText("") 
        self.code_editor.cursorPositionChanged.connect(self.on_code_cursor_changed)
        self.stack.addWidget(self.code_editor)
        
        # --- [1] 清單模式 ---
        self.path_list = QListWidget()
        self.path_list.setStyleSheet(styles.PATH_LIST_STYLE)
        self.path_list.currentRowChanged.connect(self.on_row_changed)
        self.stack.addWidget(self.path_list)
        
        layout.addWidget(self.stack)
        
    def toggle_mode(self, is_list_mode):
        self.stack.setCurrentIndex(1 if is_list_mode else 0)

    def on_code_cursor_changed(self):
        cursor = self.code_editor.textCursor()
        line_text = cursor.block().text()
        main_win = self.window()
        if hasattr(main_win, 'handle_code_preview'):
            main_win.handle_code_preview(line_text)

    def on_row_changed(self, row):
        if row < 0: return 
        main_win = self.window()
        item = self.path_list.item(row)
        if item and hasattr(main_win, 'handle_waypoint_preview'):
            real_idx = item.data(Qt.UserRole)
            main_win.handle_waypoint_preview(real_idx)

    def update_list(self, waypoints):
        from widgets import app_settings
        show_comments = app_settings.get("show_comments")
        
        self.path_list.blockSignals(True)
        self.path_list.clear()
        
        action_counter = 1 
        move_counter = 1   
        
        for i, wp in enumerate(waypoints):
            m_type = wp.get("type", "PTP")
            active = wp.get("active", True)
            
            if m_type == "BLANK_LINE":
                continue
                
            if m_type in ["COMMENT", "RAW_CODE"]:
                if not show_comments: continue
                num_str = "--" 
            elif m_type in ["LOOP_START", "LOOP_END"]:
                num_str = "  "
            else:
                num_str = f"{action_counter:02d}"
                action_counter += 1
                
            if m_type in ["COMMENT", "RAW_CODE"]:
                val = wp.get("value", "")
                text = f"{num_str}   {'//':<5}   {val}"
                item = QListWidgetItem()
                item.setSizeHint(QSize(100, 32)) 
                item.setData(Qt.UserRole, i)
                self.path_list.addItem(item)
                
                row_widget = WaypointRowWidget(text, True) 
                row_widget.btn_eye.setVisible(False)
                if m_type == "COMMENT":
                    row_widget.lbl_text.setStyleSheet("color: #6A9955; font-family: 'Consolas', 'Segoe UI'; font-size: 12px; font-style: italic;")
                else:
                    row_widget.lbl_text.setStyleSheet("color: #569CD6; font-family: 'Consolas', 'Segoe UI'; font-size: 12px;")
                self.path_list.setItemWidget(item, row_widget)
                
                main_win = self.window()
                if hasattr(main_win, 'path_manager'):
                    row_widget.btn_trash.clicked.connect(lambda checked=False, idx=i: main_win.path_manager.delete_point(idx))
                continue

            blend_str = "--"
            speed_str = "--"
            accel_str = "--"

            if m_type == "LOOP_START":
                count = wp.get("value", -1)
                count_txt = f"{count} times" if count > 0 else "Infinite"
                name_str = f"LOOP: {count_txt}"
                m_type_disp = "LOOP"  
            elif m_type == "LOOP_END":
                name_str = f"END LOOP" 
                m_type_disp = "ENDL"  
            elif m_type == "DELAY":
                val = wp.get("value", 0.0)
                name_str = f"Wait {val:.1f}s"
                m_type_disp = "DELAY"
            elif m_type == "GRIPPER":
                val = wp.get("value", 0)
                name_str = f"Gripper {val}%"
                m_type_disp = "GRIP"
                speed_str = f"v:{wp.get('speed', 50):.0f}%"
            elif m_type == "I/O":
                act = wp.get("action_type", "")
                val = wp.get("value", 0)
                name_str = f"IO:{act}={val}"
                m_type_disp = "I/O"
            else:
                name_str = f"Point {move_counter}"
                move_counter += 1  
                m_type_disp = m_type
                blend = wp.get("blend", "FINE")
                blend_str = str(blend)
                speed_str = f"v:{wp.get('speed', 50):.0f}%"
                accel_str = f"a:{wp.get('accel', 50):.0f}%"
                
            if m_type in ["LOOP_START", "LOOP_END"]:
                text = f"{num_str}   {m_type_disp:<5}   {name_str}"
            else:
                text = f"{num_str}   {m_type_disp:<5}   {name_str:<10}   blend {blend_str:<5}   {speed_str:<6}   {accel_str:<6}"
            
            item = QListWidgetItem()
            item.setSizeHint(QSize(100, 32)) 
            item.setData(Qt.UserRole, i) 
            self.path_list.addItem(item)
            
            row_widget = WaypointRowWidget(text, active)
            if m_type in ["LOOP_START", "LOOP_END"]:
                row_widget.btn_eye.setVisible(False)
                row_widget.lbl_text.setStyleSheet("color: #e6a800; font-family: 'Consolas', 'Segoe UI'; font-size: 12px;")
                
            self.path_list.setItemWidget(item, row_widget)
            
            main_win = self.window()
            if hasattr(main_win, 'path_manager'):
                row_widget.btn_eye.clicked.connect(lambda checked=False, idx=i: main_win.path_manager.toggle_point_active(idx))
                row_widget.btn_trash.clicked.connect(lambda checked=False, idx=i: main_win.path_manager.delete_point(idx))
                
        self.path_list.blockSignals(False)
        
        try:
            import compiler
            main_win = self.window()
            # 確保拿到主視窗的 tcp_manager，不然編譯器會變瞎子！
            tcp_manager = main_win.tcp_manager if hasattr(main_win, 'tcp_manager') else None
            
            new_code = compiler.ScriptCompiler.generate_code(waypoints, tcp_manager)
            
            if self.code_editor.toPlainText() != new_code:
                self.code_editor.blockSignals(True)
                self.code_editor.setPlainText(new_code)
                self.code_editor.blockSignals(False)
        except Exception as e:
            print(f"Update list error: {e}")


class CodeEditorWidget(BaseBlock):
    def __init__(self, parent=None):
        nav_config = [
            {'icon': 'mdi.play'},
            {'icon': 'mdi.skip-next'},
            {'icon': 'mdi.content-save'},
            {'icon': 'mdi.code-braces', 'toggle_icon': 'mdi.format-list-numbered'}, 
            {'icon': 'mdi.check', 'color': '#00e6b8'}, # 💥 Index 4: 編譯按鈕
            {'icon': 'mdi.trash-can-outline'},
            {'icon': 'mdi.dots-vertical'}
        ]
        super().__init__(parent=parent, nav_config=nav_config)
        self.setMinimumWidth(0) 
        
        self.btn_toggle_view = self.nav_bar.nav_buttons[3]
        self.btn_toggle_view.setCheckable(True)
        self.btn_toggle_view.toggled.connect(self.on_mode_toggled)
        
        # 💥 綁定綠色打勾編譯按鈕的神經
        self.btn_compile = self.nav_bar.nav_buttons[4]
        self.btn_compile.clicked.connect(self.compile_code)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 0) 
        layout.setSpacing(0)
        
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(styles.TAB_WIDGET_STYLE)
        self.tabs.setTabsClosable(True)
        self.tabs.tabCloseRequested.connect(self.close_tab)

        self.tabs.tabBar().setContextMenuPolicy(Qt.CustomContextMenu)
        self.tabs.tabBar().customContextMenuRequested.connect(self.show_tab_context_menu)
        
        self.btn_add_tab = QPushButton("+")
        self.btn_add_tab.setStyleSheet(styles.BTN_ADD_TAB_STYLE)
        self.btn_add_tab.setCursor(Qt.PointingHandCursor)
        self.btn_add_tab.clicked.connect(lambda *args: self.show_add_menu())
        self.tabs.setCornerWidget(self.btn_add_tab, Qt.TopRightCorner)
        
        layout.addWidget(self.tabs)
        
        self.tab_counter = 0
        self.add_new_tab()
        
    def compile_code(self):
        main_win = self.window()
        if not hasattr(main_win, 'path_manager'): return
        
        editor = self.code_editor
        if not editor: return
        
        text = editor.toPlainText()
        try:
            import compiler
            tcp_manager = main_win.tcp_manager if hasattr(main_win, 'tcp_manager') else None
            
            new_wps = compiler.ScriptCompiler.parse_code(text, main_win.current_float_joints, tcp_manager)
            
            main_win.path_manager.waypoints = new_wps
            main_win.update_path_list_ui()
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    # ==========================================
    # 分頁右鍵選單功能
    # ==========================================
    def show_tab_context_menu(self, pos):
        tab_index = self.tabs.tabBar().tabAt(pos)
        if tab_index >= 0:
            menu = QMenu(self)
            menu.setStyleSheet(styles.MENU_STYLE) 
            
            rename_action = QAction("Rename", self)
            rename_action.triggered.connect(lambda: self.rename_tab(tab_index))
            
            menu.addAction(rename_action)
            menu.exec(self.tabs.tabBar().mapToGlobal(pos))

    def rename_tab(self, index):
        from PySide6.QtWidgets import QInputDialog, QLineEdit
        current_name = self.tabs.tabText(index)
        new_name, ok = QInputDialog.getText(
            self, "Rename Script", "Enter new name:",
            QLineEdit.Normal, current_name
        )
        if ok and new_name.strip():
            self.tabs.setTabText(index, new_name.strip())

    # ==========================================
    # 分頁控制與匯入引擎
    # ==========================================
    def show_add_menu(self): 
        menu = QMenu(self)
        menu.setStyleSheet(styles.MENU_STYLE)
        action_new = QAction(qta.icon('mdi.file-document-outline', color='#e0e0e0'), "New Blank Script", self)
        action_import = QAction(qta.icon('mdi.folder-download-outline', color='#00e6b8'), "Import JSON...", self)
        action_new.triggered.connect(lambda *args: self.add_new_tab(import_json=False))
        action_import.triggered.connect(lambda *args: self.add_new_tab(import_json=True))
        menu.addAction(action_new)
        menu.addAction(action_import)
        menu.exec(QCursor.pos())

    def add_new_tab(self, import_json=False):
        from widgets import app_settings
        main_win = self.window()
        file_path = None
        code_text_from_file = None 
        
        if import_json:
            from PySide6.QtWidgets import QFileDialog
            file_path, _ = QFileDialog.getOpenFileName(self, "Import JSON", "", "JSON Files (*.json)")
            if not file_path:
                return 
            if hasattr(main_win, 'path_manager'):
                code_text_from_file = main_win.path_manager.load_from_file(file_path)
                if code_text_from_file is None:
                    return 

        self.tab_counter += 1
        tab = EditorTab()
        
        if import_json:
            self.btn_toggle_view.setChecked(False)
            tab.toggle_mode(False)
        else:
            is_list = app_settings.get("default_list_mode")
            self.btn_toggle_view.setChecked(is_list)
            tab.toggle_mode(is_list)
        
        if import_json and code_text_from_file is not None:
            tab.code_editor.setPlainText(code_text_from_file)
            tab_name = file_path.split('/')[-1] 
        else:
            tab_name = f"Untitled_{self.tab_counter}.json"
            if hasattr(main_win, 'path_manager') and self.tabs.count() > 0:
                main_win.path_manager.waypoints = []
                main_win.path_manager.list_update_signal.emit()
            
        idx = self.tabs.addTab(tab, tab_name)
        self.tabs.setCurrentIndex(idx)
        
    def close_tab(self, index):
        if self.tabs.count() > 1:
            self.tabs.removeTab(index)

    def on_mode_toggled(self, checked):
        for i in range(self.tabs.count()):
            widget = self.tabs.widget(i)
            if hasattr(widget, 'toggle_mode'):
                widget.toggle_mode(checked)

    @property
    def code_editor(self):
        tab = self.tabs.currentWidget()
        return tab.code_editor if tab else None

    @property
    def path_list(self):
        tab = self.tabs.currentWidget()
        return tab.path_list if tab else None

    def update_list(self, waypoints):
        tab = self.tabs.currentWidget()
        if tab and hasattr(tab, 'update_list'):
            tab.update_list(waypoints)