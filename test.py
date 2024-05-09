import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QUrl
from PyQt5.QtWebEngineWidgets import QWebEngineView

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle("Simple HTML Test")

        self.web_view = QWebEngineView(self)
        self.setCentralWidget(self.web_view)

        # Simple HTML content
        

        # Load the HTML content
        self.web_view.load(QUrl.fromLocalFile("/home/subsea/topside_2024/GUI_2024/map.html"))
        print("done")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

