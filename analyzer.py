import matplotlib.pyplot as plt
import numpy as np
import re

class KinematicsVisualizer:
    def __init__(self):
        # 自動適配 Windows / Mac 的中文字型
        plt.style.use('seaborn-v0_8-darkgrid')
        plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei', 'PingFang TC', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False

    def parse_log(self, log_text):
        """利用正規表達式自動從終端機輸出中萃取數據"""
        dist_mm, dist_deg, vel_deg, vel_mm = [], [], [], []
        
        # 匹配類似: 距離 1.6509mm, 角度差 3.5954°, ... 局部角速度: 43.56 deg/s, 線速度: 20.00 mm/s
        pattern = r"距離\s+([\d\.]+)mm.*?角度差\s+([\d\.]+)°.*?局部角速度:\s+([\d\.]+)\s+deg/s.*?線速度:\s+([\d\.]+)\s+mm/s"
        
        for line in log_text.strip().split('\n'):
            match = re.search(pattern, line)
            if match:
                dist_mm.append(float(match.group(1)))
                dist_deg.append(float(match.group(2)))
                vel_deg.append(float(match.group(3)))
                vel_mm.append(float(match.group(4)))
                
        return dist_mm, dist_deg, vel_deg, vel_mm

    def plot(self, dist_mm, dist_deg, vel_deg, vel_mm, title="六軸機器人 CAM 軌跡動態驗證報告"):
        """繪製三軸對比圖"""
        if not dist_mm:
            print("[警告] 找不到任何有效的軌跡數據，請檢查 Log 格式是否正確。")
            return

        points = np.arange(1, len(dist_mm) + 1)
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(title, fontsize=16, fontweight='bold', y=0.96)

        # 1. 線速度 (笛卡爾恆速驗證)
        ax1.plot(points, vel_mm, color='#2ca02c', linewidth=3, marker='o', markersize=4)
        ax1.set_ylabel('線速度 (mm/s)', fontsize=12)
        
        # 動態調整 Y 軸，讓 20mm/s 這種平坦線能置中顯示
        y_mean = np.mean(vel_mm)
        y_max = max(vel_mm)
        ax1.set_ylim(max(0, y_mean - 10), y_max + 10)
        
        if y_mean > 0:
            ax1.axhline(y_mean, color='gray', linestyle='--', alpha=0.5)
        ax1.set_title('笛卡爾空間線速度 (恆定狀態檢視)', loc='left', fontsize=11)

        # 2. 角度差 (姿態平滑與波浪特徵驗證)
        ax2.plot(points, dist_deg, color='#1f77b4', linewidth=2.5, marker='s', markersize=5)
        ax2.set_ylabel('TCP 角度差 (度)', fontsize=12)
        ax2.set_title('相鄰微線段姿態變化量 (Z/X-Holonomy 平滑檢視)', loc='left', fontsize=11)

        # 3. 局部角速度 (安全極限與微線段打結驗證)
        ax3.bar(points, vel_deg, color='#ff7f0e', alpha=0.7)
        ax3.plot(points, vel_deg, color='#d62728', linewidth=1.5, marker='.')
        ax3.set_ylabel('角速度 (deg/s)', fontsize=12)
        ax3.set_xlabel('微線段點位序列', fontsize=12)
        ax3.set_title('手腕關節局部角速度動態分佈', loc='left', fontsize=11)

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        plt.show()

    def run(self, log_text, title="六軸機器人 CAM 軌跡動態驗證報告"):
        """一鍵完成解析與繪圖"""
        d_mm, d_deg, v_deg, v_mm = self.parse_log(log_text)
        print(f"[System] 成功解析 {len(d_mm)} 筆點位數據。")
        self.plot(d_mm, d_deg, v_deg, v_mm, title)


# ==========================================
# 實際使用範例
# ==========================================
if __name__ == "__main__":
    # 只要把你從終端機複製下來的文字，貼到這兩個三引號之間即可！
    terminal_output = """
    點 001: 距離 1.7470mm, 角度差 2.6757°, 分配時間 0.08735s | 局部角速度: 30.63 deg/s, 線速度: 20.00 mm/s
    """

    visualizer = KinematicsVisualizer()
    visualizer.run(terminal_output, title="圓孔切削軌跡 (minimum_twist) 測試分析")