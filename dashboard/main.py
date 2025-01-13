import sys
from wren_dashboard import WrenDashboard
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QFont

if __name__ == '__main__':
    app = QApplication(sys.argv)

    font = QFont()
    font.setPointSize(12)  # Adjust the point size as needed
    app.setFont(font)

    window = WrenDashboard()
    if len(sys.argv) > 1:
        window.open_file(sys.argv[1])
    window.show()
    sys.exit(app.exec_())