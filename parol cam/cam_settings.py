# cam_settings.py

# ==========================================
# CAM 系統預設參數 (常數)
# ==========================================
# 加工預設參數
DEFAULT_CHORDAL_ERROR = 0.02

# 運動學預設參數
DEFAULT_IK_SEED = [0.0, 0.0, 30.0, 0.0, 90.0, 0.0]

try:
    from config import Robot3DView 
except ImportError:
    print("[Warning] 無法從舊版 config 載入 Robot3DView。請確認檔案是否存在。")
    # 如果找不到，可以給一個 Dummy class 避免 IDE 報錯，或直接讓它 pass
    Robot3DView = None