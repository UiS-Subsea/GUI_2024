import sys
import math
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsRectItem, QGraphicsLineItem, QPushButton


class ROV(QGraphicsRectItem):
    def __init__(self, x, y, width, height):
        super().__init__(x, y, width, height)
        self.setFlag(QGraphicsRectItem.ItemIsSelectable)
        self.setAcceptHoverEvents(True)
        self.setBrush(Qt.blue)

    def update_pitch(self, angle):
        self.setRotation(angle)


class Manipulator(QGraphicsLineItem):
    def __init__(self, joint1_pos, joint2_pos):
        super().__init__()
        self.joint1_pos = joint1_pos
        self.joint2_pos = joint2_pos
        self.setLine(joint1_pos.x(), joint1_pos.y(), joint2_pos.x(), joint2_pos.y())
        self.setPen(Qt.red)

    def update_arm(self, angle1, angle2):
        # Calculate end effector position based on joint angles
        end_effector_pos = QPointF(self.joint2_pos.x() + 50 * math.cos(angle1),
                                   self.joint2_pos.y() + 50 * math.sin(angle1))
        self.setLine(self.joint1_pos.x(), self.joint1_pos.y(), end_effector_pos.x(), end_effector_pos.y())


class GraphicsView(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.setScene(QGraphicsScene(self))
        self.scene().setSceneRect(-200, -200, 400, 400)
        self.rov = ROV(0, 0, 100, 50)
        self.manipulator = Manipulator(self.rov.boundingRect().topRight(), self.rov.boundingRect().topRight() + QPointF(50, 0))
        self.scene().addItem(self.rov)
        self.scene().addItem(self.manipulator)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_R:
            self.rov.update_pitch(self.rov.rotation() + 15)  # Rotate ROV by 15 degrees on pressing 'R'
        else:
            super().keyPressEvent(event)


class ROVWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()
        self.graphicsView = GraphicsView()
        self.btnRotate = QPushButton("Rotate ROV (R key)")
        layout.addWidget(self.graphicsView)
        layout.addWidget(self.btnRotate)

        self.btnRotate.clicked.connect(self.rotateROV)

        self.setLayout(layout)

    def rotateROV(self):
        self.graphicsView.rov.update_pitch(self.graphicsView.rov.rotation() + 15)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.rovWidget = ROVWidget()
        self.setCentralWidget(self.rovWidget)
        self.setWindowTitle("ROV with Manipulator")
        self.resize(400, 400)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    sys.exit(app.exec_())