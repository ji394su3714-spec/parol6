# parol_cam/main.py
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# 必須在設定完 sys.path 之後，才 import 我們的自訂模組
from PySide6.QtWidgets import QApplication
from gui import ParolCamWindow

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion") 
    
    window = ParolCamWindow()
    window.show()
    
    sys.exit(app.exec())

if __name__ == '__main__':
    main()