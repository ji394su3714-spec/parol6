# base_manager.py
import json
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from PySide6.QtCore import QObject, Signal

class BaseManager(QObject):
    data_changed = Signal()

    def __init__(self, filepath="base_config.json", parent=None):
        super().__init__(parent)
        self.filepath = filepath
        self.current_index = 0
        
        # 預設永遠有一個不可刪除的 World Base (世界坐標系)
        self.default_base = {
            "name": "Base 0 (World)",
            "values": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "in_box": True,
            "is_locked": True  # 鎖定標記：防止被使用者刪除
        }
        self.bases = [self.default_base]
        self.load_config()

    def get_matrix(self, index):
        """將指定的 Base 數值轉換為 4x4 齊次變換矩陣"""
        if 0 <= index < len(self.bases):
            vals = self.bases[index]["values"]
        else:
            vals = self.default_base["values"]
            
        x, y, z, rx, ry, rz = vals
        T = np.eye(4)
        # Rz * Ry * Rx 轉換 (單位：度)
        T[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
        # 毫米轉換為公尺
        T[:3, 3] = [x/1000.0, y/1000.0, z/1000.0]
        return T

    def get_active_matrix(self):
        """獲取目前 UI 選中的 Active Base 矩陣 (用於寸動與錄製)"""
        return self.get_matrix(self.current_index)
        
    def get_active_base_data(self):
        if 0 <= self.current_index < len(self.bases):
            return self.bases[self.current_index]
        return self.default_base

    def set_current_index(self, index):
        if 0 <= index < len(self.bases):
            self.current_index = index
            self.data_changed.emit()

    def add_base(self, name, values, in_box=True):
        self.bases.append({
            "name": name,
            "values": values,
            "in_box": in_box,
            "is_locked": False
        })
        self.save_config()
        self.data_changed.emit()

    def update_base(self, index, name, values, in_box):
        """大腦專用的 update_base：負責處理邏輯與寫入硬碟"""
        if 0 <= index < len(self.bases): 
            if index == 0:
                # 絕對防護：如果是 Base 0，只能更新 XYZ/RxRyRz 數值，不能改名！
                self.bases[index]["values"] = values 
            else:
                self.bases[index]["name"] = name
                self.bases[index]["values"] = values
                self.bases[index]["in_box"] = in_box
                
            self.save_config()
            self.data_changed.emit()

    def delete_base(self, index):
        if 0 < index < len(self.bases): # 禁止刪除 World Base
            del self.bases[index]
            if self.current_index >= len(self.bases):
                self.current_index = max(0, len(self.bases) - 1)
            self.save_config()
            self.data_changed.emit()

    def save_config(self):
        try:
            with open(self.filepath, 'w') as f:
                json.dump(self.bases, f, indent=4)
        except Exception as e:
            print(f"Error saving Base config: {e}")

    def load_config(self):
        if os.path.exists(self.filepath):
            try:
                with open(self.filepath, 'r') as f:
                    loaded_bases = json.load(f)
                    
                # 確保第一個永遠是 World Base
                if loaded_bases and loaded_bases[0].get("is_locked"):
                    self.bases = loaded_bases
                else:
                    self.bases = [self.default_base] + loaded_bases
                    
            except Exception as e:
                print(f"Error loading Base config: {e}")
                self.bases = [self.default_base]
        else:
            self.save_config()