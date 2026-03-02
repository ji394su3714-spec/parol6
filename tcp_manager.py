# tcp_manager.py
import json
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

TCP_CONFIG_FILE = "tcp_config.json"

class TCPManager:
    def __init__(self):
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
            with open(TCP_CONFIG_FILE, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
        except Exception as e:
            print(f"Error saving TCP config: {e}")

    def create_default_tool(self):
        self.tools = [{
            "name": "Default Tool",
            "values": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # X, Y, Z, Rx, Ry, Rz
        }]
        self.current_index = 0
        self.save_config()

    # --- 資料操作 ---
    def get_tools(self):
        return self.tools

    def add_tool(self, name="New Tool"):
        self.tools.append({
            "name": name,
            "values": [0.0]*6
        })
        self.current_index = len(self.tools) - 1 # 切換到新工具
        self.save_config()

    def delete_tool(self, index):
        if len(self.tools) <= 1: return # 至少保留一個
        
        self.tools.pop(index)
        if self.current_index >= len(self.tools):
            self.current_index = len(self.tools) - 1
        self.save_config()

    def set_current_index(self, index):
        if 0 <= index < len(self.tools):
            self.current_index = index
            self.save_config()

    def update_tool_values(self, index, values):
        """更新數值但不存檔 (由 UI 決定何時存檔，避免頻繁寫入)"""
        if 0 <= index < len(self.tools):
            self.tools[index]["values"] = values

    def rename_tool(self, index, new_name):
        if 0 <= index < len(self.tools):
            self.tools[index]["name"] = new_name
            self.save_config()

    # --- 核心運算 ---
    def get_active_tool_data(self):
        """回傳當前工具的 name 與 values"""
        return self.tools[self.current_index]

    def get_active_matrix(self):
        """直接回傳 4x4 矩陣供 Kinematics 使用"""
        vals = self.tools[self.current_index]["values"]
        x, y, z, rx, ry, rz = vals
        
        mat = np.eye(4)
        # 計算旋轉
        r_mat = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
        mat[:3, :3] = r_mat
        # 計算位移 (mm -> m)
        mat[:3, 3] = [x/1000.0, y/1000.0, z/1000.0]
        
        return mat