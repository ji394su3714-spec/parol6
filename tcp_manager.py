# tcp_manager.py
import json
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from PySide6.QtCore import QObject, Signal

# ==========================================
# 動態絕對路徑設定 (防止終端機啟動位置錯誤)
# ==========================================
# 取得目前這支 Python 檔案所在的「絕對資料夾路徑」
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# 設定子資料夾名稱為 "config"
CONFIG_DIR = os.path.join(BASE_DIR, "config")
# 組合出絕對不會出錯的檔案路徑
TCP_CONFIG_FILE = os.path.join(CONFIG_DIR, "tcp_config.json")

class TCPManager(QObject):
    data_changed = Signal() 

    def __init__(self, parent=None):
        super().__init__(parent) 
        self.tools = []
        self.current_index = 0
        self.load_config()

    def load_config(self):
        """讀取設定檔，若無則建立預設值"""
        if os.path.exists(TCP_CONFIG_FILE):
            try:
                with open(TCP_CONFIG_FILE, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self.tools = data.get("tools", [])
                    self.current_index = data.get("current_index", 0)
            except Exception as e:
                print(f"Error loading TCP config: {e}")
        
        # 確保至少有一個工具
        if not self.tools:
            self.create_default_tool()

    def save_config(self):
        """儲存設定到硬碟"""
        data = {
            "tools": self.tools,
            "current_index": self.current_index
        }
        try:
            # 關鍵防護：確保存檔前，子資料夾 "config" 絕對存在
            os.makedirs(os.path.dirname(TCP_CONFIG_FILE), exist_ok=True)
            
            with open(TCP_CONFIG_FILE, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
        except Exception as e:
            print(f"Error saving TCP config: {e}")

    def create_default_tool(self):
        self.tools = [{
            "name": "Default Tool",
            "values": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "in_box": True
        }]
        self.current_index = 0
        self.save_config()

    # --- 資料操作 ---
    def get_tools(self):
        return self.tools

    def add_tool(self, name="New Tool"):
        self.tools.append({
            "name": name,
            "values": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # 強制賦予全新的獨立陣列
            "in_box": True
        })
        self.current_index = len(self.tools) - 1 
        self.save_config()
        self.data_changed.emit()

    def delete_tool(self, index):
        if len(self.tools) <= 1: return 
        
        self.tools.pop(index)
        if self.current_index >= len(self.tools):
            self.current_index = len(self.tools) - 1
        self.save_config()
        self.data_changed.emit()

    def set_current_index(self, index):
        if 0 <= index < len(self.tools):
            self.current_index = index
            self.save_config()
            self.data_changed.emit()

    # 核心修復：供 Apply 按鈕呼叫的唯一更新入口
    def update_tool(self, index, name, values, in_box=True):
        """按下 Apply 後才呼叫：寫入資料、存檔，並觸發全系統 3D 重繪"""
        if 0 <= index < len(self.tools):
            self.tools[index]["name"] = name
            
            # 終極防禦：使用 list() 強制拷貝陣列，徹底切斷記憶體共用問題！
            self.tools[index]["values"] = list(values) 
            
            self.tools[index]["in_box"] = in_box
            self.save_config()
            self.data_changed.emit()

    # --- 核心運算 ---
    def get_active_tool_data(self):
        """回傳當前工具的 name 與 values"""
        return self.tools[self.current_index]

    def get_active_matrix(self):
        """直接回傳 4x4 矩陣供 Kinematics 與全系統使用"""
        # 加上 .get() 安全防護，避免讀到舊格式資料出錯
        vals = self.tools[self.current_index].get("values", [0.0]*6)
        x, y, z, rx, ry, rz = vals
        
        # 1. 使用者在 UI 設定的 TCP 數值矩陣
        user_mat = np.eye(4)
        r_mat = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
        user_mat[:3, :3] = r_mat
        user_mat[:3, 3] = [x/1000.0, y/1000.0, z/1000.0]
        
        # 2. 法蘭盤全域校正矩陣 (Flange Offset)
        flange_offset = np.eye(4)
        flange_offset[:3, :3] = R.from_euler('x', -90, degrees=True).as_matrix()
        
        # 3. 疊加後回傳 (Flange -> Offset -> Tool)
        return flange_offset @ user_mat