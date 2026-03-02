import sys
import traceback
from PyQt5.QtWidgets import QApplication

# 引入您的 GUI 類別
from gui import RobotGUI

def main():
    # 1. 建立 Qt 應用程式
    app = QApplication(sys.argv)
    
    # 2. 可以在這裡做全域設定 (例如設定 App 圖示、字型等)
    # app.setWindowIcon(...) 

    try:
        # 3. 初始化主視窗
        window = RobotGUI()
        window.show()

        # 4. 進入程式主迴圈
        sys.exit(app.exec_())
        
    except Exception:
        # 這是一個保險：如果程式發生嚴重錯誤，把錯誤訊息印出來，而不是直接無聲閃退
        traceback.print_exc()

if __name__ == "__main__":
    main()