import os
import sys
os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "0"
import traceback

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt
from gui import RobotGUI
app = QApplication(sys.argv)
app.setStyle("windows")
app.styleHints().setColorScheme(Qt.ColorScheme.Light)

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