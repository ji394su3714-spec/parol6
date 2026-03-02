import sys
import traceback
from PyQt5.QtWidgets import QApplication
from gui import RobotGUI

def main():
    app = QApplication(sys.argv)

    try:
        window = RobotGUI()
        window.show()
        sys.exit(app.exec_())
        
    except Exception:
        traceback.print_exc()

if __name__ == "__main__":
    main()