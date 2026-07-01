import sys
from PySide6.QtWidgets import QApplication
import vispy.app

# 確保 VisPy 正確使用 PySide6 後台
vispy.app.use_app('pyside6')

from gui import RobotControllerGUI

def main():
    app = QApplication(sys.argv)
    window = RobotControllerGUI()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()