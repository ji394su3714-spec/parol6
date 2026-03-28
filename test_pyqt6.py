import sys
from PyQt6.QtWidgets import QApplication, QWidget, QLabel
from PyQt6.QtCore import Qt

app = QApplication(sys.argv)

# 建立主視窗
window = QWidget()
window.setWindowTitle('PyQt6 測試成功！')
window.setGeometry(100, 100, 400, 200)

# 建立文字標籤
label = QLabel('歡迎來到 PyQt6 的世界！', parent=window)
# 注意這裡！對齊方式的寫法跟 PyQt5 不一樣了
label.setAlignment(Qt.AlignmentFlag.AlignCenter) 
label.resize(400, 200)

window.show()

# 注意這裡！exec_() 變成了 exec()
sys.exit(app.exec())