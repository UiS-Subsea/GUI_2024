# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MainWindow.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1860, 1000)
        MainWindow.setStyleSheet("background-color:rgb(21, 21, 39);\n"
"\n"
"color:white;\n"
"")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setStyleSheet("")
        self.tab.setObjectName("tab")
        self.frame_2 = QtWidgets.QFrame(self.tab)
        self.frame_2.setGeometry(QtCore.QRect(0, 0, 691, 681))
        self.frame_2.setStyleSheet("QFrame{background-color:rgb(23, 27, 48);\n"
"border-radius: 15px; \n"
"border: 2px solid rgb(99, 102, 108);}\n"
"\n"
"QLabel{ background-color:rgb(21,21,39);}")
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.label_2 = QtWidgets.QLabel(self.frame_2)
        self.label_2.setGeometry(QtCore.QRect(160, 10, 351, 41))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setStyleSheet("")
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.button_logger = QtWidgets.QPushButton(self.frame_2)
        self.button_logger.setGeometry(QtCore.QRect(280, 610, 121, 51))
        self.button_logger.setStyleSheet("background-color:green;\n"
"border-radius:15px;")
        self.button_logger.setObjectName("button_logger")
        self.temp_1 = QtWidgets.QLabel(self.frame_2)
        self.temp_1.setGeometry(QtCore.QRect(280, 190, 111, 31))
        self.temp_1.setStyleSheet("\n"
"border: 2px solid green;\n"
"\n"
"")
        self.temp_1.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.temp_1.setAlignment(QtCore.Qt.AlignCenter)
        self.temp_1.setObjectName("temp_1")
        self.temp_2 = QtWidgets.QLabel(self.frame_2)
        self.temp_2.setGeometry(QtCore.QRect(280, 230, 111, 31))
        self.temp_2.setStyleSheet("\n"
"border: 2px solid green;\n"
"\n"
"")
        self.temp_2.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.temp_2.setAlignment(QtCore.Qt.AlignCenter)
        self.temp_2.setObjectName("temp_2")
        self.temp_3 = QtWidgets.QLabel(self.frame_2)
        self.temp_3.setGeometry(QtCore.QRect(280, 270, 111, 31))
        self.temp_3.setStyleSheet("\n"
"border: 2px solid green;\n"
"\n"
"")
        self.temp_3.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.temp_3.setAlignment(QtCore.Qt.AlignCenter)
        self.temp_3.setObjectName("temp_3")
        self.temp_4 = QtWidgets.QLabel(self.frame_2)
        self.temp_4.setGeometry(QtCore.QRect(280, 310, 111, 31))
        self.temp_4.setStyleSheet("\n"
"border: 2px solid green;\n"
"\n"
"")
        self.temp_4.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.temp_4.setAlignment(QtCore.Qt.AlignCenter)
        self.temp_4.setObjectName("temp_4")
        self.temp_5 = QtWidgets.QLabel(self.frame_2)
        self.temp_5.setGeometry(QtCore.QRect(560, 600, 111, 31))
        self.temp_5.setStyleSheet("\n"
"border: 2px solid green;\n"
"\n"
"")
        self.temp_5.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.temp_5.setAlignment(QtCore.Qt.AlignCenter)
        self.temp_5.setObjectName("temp_5")
        self.thruster_1 = QtWidgets.QLabel(self.frame_2)
        self.thruster_1.setGeometry(QtCore.QRect(100, 150, 61, 41))
        self.thruster_1.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.thruster_1.setStyleSheet("")
        self.thruster_1.setAlignment(QtCore.Qt.AlignCenter)
        self.thruster_1.setObjectName("thruster_1")
        self.thruster_5 = QtWidgets.QLabel(self.frame_2)
        self.thruster_5.setGeometry(QtCore.QRect(210, 200, 61, 41))
        self.thruster_5.setStyleSheet("")
        self.thruster_5.setAlignment(QtCore.Qt.AlignCenter)
        self.thruster_5.setObjectName("thruster_5")
        self.thruster_2 = QtWidgets.QLabel(self.frame_2)
        self.thruster_2.setGeometry(QtCore.QRect(520, 150, 61, 41))
        self.thruster_2.setStyleSheet("")
        self.thruster_2.setAlignment(QtCore.Qt.AlignCenter)
        self.thruster_2.setObjectName("thruster_2")
        self.thruster_6 = QtWidgets.QLabel(self.frame_2)
        self.thruster_6.setGeometry(QtCore.QRect(410, 200, 61, 41))
        self.thruster_6.setStyleSheet("")
        self.thruster_6.setAlignment(QtCore.Qt.AlignCenter)
        self.thruster_6.setObjectName("thruster_6")
        self.thruster_3 = QtWidgets.QLabel(self.frame_2)
        self.thruster_3.setGeometry(QtCore.QRect(100, 410, 61, 41))
        self.thruster_3.setStyleSheet("")
        self.thruster_3.setAlignment(QtCore.Qt.AlignCenter)
        self.thruster_3.setObjectName("thruster_3")
        self.thruster_7 = QtWidgets.QLabel(self.frame_2)
        self.thruster_7.setGeometry(QtCore.QRect(210, 380, 61, 41))
        self.thruster_7.setStyleSheet("")
        self.thruster_7.setAlignment(QtCore.Qt.AlignCenter)
        self.thruster_7.setObjectName("thruster_7")
        self.thruster_4 = QtWidgets.QLabel(self.frame_2)
        self.thruster_4.setGeometry(QtCore.QRect(520, 410, 61, 41))
        self.thruster_4.setStyleSheet("")
        self.thruster_4.setAlignment(QtCore.Qt.AlignCenter)
        self.thruster_4.setObjectName("thruster_4")
        self.thruster_8 = QtWidgets.QLabel(self.frame_2)
        self.thruster_8.setGeometry(QtCore.QRect(410, 380, 61, 41))
        self.thruster_8.setStyleSheet("")
        self.thruster_8.setAlignment(QtCore.Qt.AlignCenter)
        self.thruster_8.setObjectName("thruster_8")
        self.rov_img = QtWidgets.QLabel(self.frame_2)
        self.rov_img.setGeometry(QtCore.QRect(80, 120, 601, 431))
        self.rov_img.setStyleSheet("border:none; background-color:none;")
        self.rov_img.setText("")
        self.rov_img.setPixmap(QtGui.QPixmap("images/rov.png"))
        self.rov_img.setIndent(-1)
        self.rov_img.setObjectName("rov_img")
        self.rov_img.raise_()
        self.label_2.raise_()
        self.button_logger.raise_()
        self.temp_1.raise_()
        self.temp_2.raise_()
        self.temp_3.raise_()
        self.temp_4.raise_()
        self.temp_5.raise_()
        self.thruster_1.raise_()
        self.thruster_5.raise_()
        self.thruster_2.raise_()
        self.thruster_6.raise_()
        self.thruster_3.raise_()
        self.thruster_7.raise_()
        self.thruster_4.raise_()
        self.thruster_8.raise_()
        self.frame_6 = QtWidgets.QFrame(self.tab)
        self.frame_6.setGeometry(QtCore.QRect(1250, 0, 571, 531))
        self.frame_6.setStyleSheet("background-color:rgb(23, 27, 48);\n"
"border-radius: 15px; \n"
"border: 2px solid rgb(99, 102, 108);")
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.label_8 = QtWidgets.QLabel(self.frame_6)
        self.label_8.setGeometry(QtCore.QRect(200, 20, 201, 41))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_8.setFont(font)
        self.label_8.setStyleSheet("background-color:rgb(21,21,39);")
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.frame_9 = QtWidgets.QFrame(self.frame_6)
        self.frame_9.setGeometry(QtCore.QRect(280, 90, 271, 221))
        self.frame_9.setStyleSheet("background-color:rgb(21,21,39);\n"
"")
        self.frame_9.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_9.setObjectName("frame_9")
        self.label_18 = QtWidgets.QLabel(self.frame_9)
        self.label_18.setGeometry(QtCore.QRect(70, 20, 141, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.label_18.setFont(font)
        self.label_18.setAlignment(QtCore.Qt.AlignCenter)
        self.label_18.setObjectName("label_18")
        self.button_manual = QtWidgets.QPushButton(self.frame_9)
        self.button_manual.setGeometry(QtCore.QRect(90, 74, 89, 31))
        self.button_manual.setStyleSheet("background-color:green;\n"
"color:white\n"
";")
        self.button_manual.setObjectName("button_manual")
        self.button_pipeline = QtWidgets.QPushButton(self.frame_9)
        self.button_pipeline.setGeometry(QtCore.QRect(90, 124, 89, 31))
        self.button_pipeline.setObjectName("button_pipeline")
        self.button_docking = QtWidgets.QPushButton(self.frame_9)
        self.button_docking.setGeometry(QtCore.QRect(90, 174, 89, 31))
        self.button_docking.setObjectName("button_docking")
        self.frame_7 = QtWidgets.QFrame(self.frame_6)
        self.frame_7.setGeometry(QtCore.QRect(280, 320, 271, 171))
        self.frame_7.setStyleSheet("background-color:rgb(21,21,39);")
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.label_17 = QtWidgets.QLabel(self.frame_7)
        self.label_17.setGeometry(QtCore.QRect(80, 20, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_17.setFont(font)
        self.label_17.setAlignment(QtCore.Qt.AlignCenter)
        self.label_17.setObjectName("label_17")
        self.horizontalSlider = QtWidgets.QSlider(self.frame_7)
        self.horizontalSlider.setGeometry(QtCore.QRect(70, 120, 160, 16))
        self.horizontalSlider.setStyleSheet("border:none;")
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setInvertedControls(False)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.button_light = QtWidgets.QPushButton(self.frame_7)
        self.button_light.setGeometry(QtCore.QRect(90, 70, 91, 31))
        self.button_light.setStyleSheet("background-color:red;")
        self.button_light.setCheckable(True)
        self.button_light.setObjectName("button_light")
        self.frame_8 = QtWidgets.QFrame(self.frame_6)
        self.frame_8.setGeometry(QtCore.QRect(20, 90, 251, 221))
        self.frame_8.setStyleSheet("QFrame{background-color:rgb(21, 21, 39);\n"
"border-radius: 15px; \n"
"border: 2px solid rgb(99, 102, 108);}\n"
"\n"
"QLabel{border:none}")
        self.frame_8.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_8.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_8.setObjectName("frame_8")
        self.label_14 = QtWidgets.QLabel(self.frame_8)
        self.label_14.setGeometry(QtCore.QRect(50, 40, 53, 17))
        self.label_14.setObjectName("label_14")
        self.progressBar = QtWidgets.QProgressBar(self.frame_8)
        self.progressBar.setGeometry(QtCore.QRect(130, 40, 95, 25))
        self.progressBar.setStyleSheet("border:none;\n"
"background-color:rgb(23, 27, 48)")
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")
        self.label_15 = QtWidgets.QLabel(self.frame_8)
        self.label_15.setGeometry(QtCore.QRect(50, 130, 67, 17))
        self.label_15.setObjectName("label_15")
        self.label_16 = QtWidgets.QLabel(self.frame_8)
        self.label_16.setGeometry(QtCore.QRect(160, 130, 67, 17))
        self.label_16.setObjectName("label_16")
        self.button_init_cams = QtWidgets.QPushButton(self.frame_6)
        self.button_init_cams.setGeometry(QtCore.QRect(30, 340, 101, 31))
        self.button_init_cams.setStyleSheet("background-color:green;")
        self.button_init_cams.setObjectName("button_init_cams")
        self.button_init_nodes = QtWidgets.QPushButton(self.frame_6)
        self.button_init_nodes.setGeometry(QtCore.QRect(30, 390, 101, 31))
        self.button_init_nodes.setStyleSheet("background-color:green;")
        self.button_init_nodes.setObjectName("button_init_nodes")
        self.frame_5 = QtWidgets.QFrame(self.tab)
        self.frame_5.setGeometry(QtCore.QRect(1250, 540, 571, 361))
        self.frame_5.setStyleSheet("QFrame{background-color:rgb(23, 27, 48);\n"
"border-radius: 15px; \n"
"border: 2px solid rgb(99, 102, 108);}\n"
"QLabel{border:none;}\n"
"\n"
"")
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.status_icon_ros_rov = QtWidgets.QLabel(self.frame_5)
        self.status_icon_ros_rov.setGeometry(QtCore.QRect(510, 100, 31, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.status_icon_ros_rov.setFont(font)
        self.status_icon_ros_rov.setStyleSheet("color:red")
        self.status_icon_ros_rov.setObjectName("status_icon_ros_rov")
        self.label_ros_controller_status = QtWidgets.QLabel(self.frame_5)
        self.label_ros_controller_status.setGeometry(QtCore.QRect(380, 190, 82, 17))
        self.label_ros_controller_status.setObjectName("label_ros_controller_status")
        self.label_10 = QtWidgets.QLabel(self.frame_5)
        self.label_10.setGeometry(QtCore.QRect(230, 190, 47, 17))
        self.label_10.setObjectName("label_10")
        self.label_ros_rov_status = QtWidgets.QLabel(self.frame_5)
        self.label_ros_rov_status.setGeometry(QtCore.QRect(380, 100, 82, 17))
        self.label_ros_rov_status.setObjectName("label_ros_rov_status")
        self.status_icon_ros_controller = QtWidgets.QLabel(self.frame_5)
        self.status_icon_ros_controller.setGeometry(QtCore.QRect(510, 190, 31, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.status_icon_ros_controller.setFont(font)
        self.status_icon_ros_controller.setStyleSheet("color:red")
        self.status_icon_ros_controller.setObjectName("status_icon_ros_controller")
        self.label_7 = QtWidgets.QLabel(self.frame_5)
        self.label_7.setGeometry(QtCore.QRect(30, 100, 121, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.label_9 = QtWidgets.QLabel(self.frame_5)
        self.label_9.setGeometry(QtCore.QRect(30, 190, 153, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.label_5 = QtWidgets.QLabel(self.frame_5)
        self.label_5.setGeometry(QtCore.QRect(180, 20, 201, 41))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setStyleSheet("border:2px solid rgb(99, 102, 108);\n"
"background-color:rgb(21,21,39);")
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.frame_5)
        self.label_6.setGeometry(QtCore.QRect(230, 100, 47, 17))
        self.label_6.setObjectName("label_6")
        self.frame_4 = QtWidgets.QFrame(self.tab)
        self.frame_4.setGeometry(QtCore.QRect(700, 0, 540, 341))
        self.frame_4.setStyleSheet("background-color:rgb(21, 21, 39);\n"
"border-radius: 15px; \n"
"border: 2px solid rgb(99, 102, 108);")
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.label_4 = QtWidgets.QLabel(self.frame_4)
        self.label_4.setGeometry(QtCore.QRect(170, 20, 211, 41))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.frame_3 = QtWidgets.QFrame(self.tab)
        self.frame_3.setGeometry(QtCore.QRect(700, 350, 541, 271))
        self.frame_3.setStyleSheet("QFrame{background-color:rgb(21, 21, 39);\n"
"border-radius: 15px; \n"
"border: 2px solid rgb(99, 102, 108);}\n"
"QLabel{background-color:rgb(21, 21, 39);\n"
"border-radius: 15px; \n"
"border: 2px solid rgb(99, 102, 108);}\n"
"\n"
"QHorizontalLayout{ \n"
"    border:none;\n"
"}\n"
"QVerticalLayout{ \n"
"    border:none;\n"
"}\n"
"")
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.label = QtWidgets.QLabel(self.frame_3)
        self.label.setGeometry(QtCore.QRect(180, 10, 191, 41))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.frame_3)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(30, 70, 241, 171))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_11 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_11.setObjectName("label_11")
        self.verticalLayout_3.addWidget(self.label_11)
        self.label_3 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_3.addWidget(self.label_3)
        self.label_12 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_12.setObjectName("label_12")
        self.verticalLayout_3.addWidget(self.label_12)
        self.horizontalLayout.addLayout(self.verticalLayout_3)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_pitch = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_pitch.setStyleSheet("border:none;")
        self.label_pitch.setAlignment(QtCore.Qt.AlignCenter)
        self.label_pitch.setObjectName("label_pitch")
        self.verticalLayout_4.addWidget(self.label_pitch)
        self.label_roll = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_roll.setStyleSheet("border:none;")
        self.label_roll.setAlignment(QtCore.Qt.AlignCenter)
        self.label_roll.setObjectName("label_roll")
        self.verticalLayout_4.addWidget(self.label_roll)
        self.label_yaw = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_yaw.setStyleSheet("border:none;")
        self.label_yaw.setAlignment(QtCore.Qt.AlignCenter)
        self.label_yaw.setObjectName("label_yaw")
        self.verticalLayout_4.addWidget(self.label_yaw)
        self.horizontalLayout.addLayout(self.verticalLayout_4)
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(self.frame_3)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(280, 70, 241, 171))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_22 = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label_22.setObjectName("label_22")
        self.verticalLayout_5.addWidget(self.label_22)
        self.label_23 = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label_23.setObjectName("label_23")
        self.verticalLayout_5.addWidget(self.label_23)
        self.label_24 = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label_24.setObjectName("label_24")
        self.verticalLayout_5.addWidget(self.label_24)
        self.horizontalLayout_2.addLayout(self.verticalLayout_5)
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.label_surge = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label_surge.setStyleSheet("border:none;")
        self.label_surge.setAlignment(QtCore.Qt.AlignCenter)
        self.label_surge.setObjectName("label_surge")
        self.verticalLayout_6.addWidget(self.label_surge)
        self.label_sway = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label_sway.setStyleSheet("border:none;")
        self.label_sway.setAlignment(QtCore.Qt.AlignCenter)
        self.label_sway.setObjectName("label_sway")
        self.verticalLayout_6.addWidget(self.label_sway)
        self.label_heave = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label_heave.setStyleSheet("border:none;")
        self.label_heave.setAlignment(QtCore.Qt.AlignCenter)
        self.label_heave.setObjectName("label_heave")
        self.verticalLayout_6.addWidget(self.label_heave)
        self.horizontalLayout_2.addLayout(self.verticalLayout_6)
        self.horizontalSlider_2 = QtWidgets.QSlider(self.tab)
        self.horizontalSlider_2.setGeometry(QtCore.QRect(520, 770, 101, 16))
        self.horizontalSlider_2.setProperty("value", 0)
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setObjectName("horizontalSlider_2")
        self.horizontalSlider_3 = QtWidgets.QSlider(self.tab)
        self.horizontalSlider_3.setGeometry(QtCore.QRect(530, 820, 81, 16))
        self.horizontalSlider_3.setProperty("value", 0)
        self.horizontalSlider_3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_3.setObjectName("horizontalSlider_3")
        self.frame_11 = QtWidgets.QFrame(self.tab)
        self.frame_11.setGeometry(QtCore.QRect(700, 620, 541, 311))
        self.frame_11.setStyleSheet("background-color:rgb(21, 21, 39);\n"
"border-radius: 15px; \n"
"border: 2px solid rgb(99, 102, 108);")
        self.frame_11.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_11.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_11.setObjectName("frame_11")
        self.widget = QtWidgets.QWidget(self.frame_11)
        self.widget.setGeometry(QtCore.QRect(10, 70, 521, 221))
        self.widget.setStyleSheet("border:none;")
        self.widget.setObjectName("widget")
        self.label_21 = QtWidgets.QLabel(self.frame_11)
        self.label_21.setGeometry(QtCore.QRect(190, 20, 181, 41))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_21.setFont(font)
        self.label_21.setAlignment(QtCore.Qt.AlignCenter)
        self.label_21.setObjectName("label_21")
        self.button_manipulator = QtWidgets.QPushButton(self.frame_11)
        self.button_manipulator.setGeometry(QtCore.QRect(430, 24, 89, 31))
        self.button_manipulator.setStyleSheet("background-color:red")
        self.button_manipulator.setCheckable(True)
        self.button_manipulator.setObjectName("button_manipulator")
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.frame = QtWidgets.QFrame(self.tab_2)
        self.frame.setGeometry(QtCore.QRect(0, 10, 901, 661))
        self.frame.setStyleSheet("background-color:rgb(23, 27, 48);\n"
"border-radius: 15px; \n"
"border: 2px solid rgb(99, 102, 108);")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.label_13 = QtWidgets.QLabel(self.frame)
        self.label_13.setGeometry(QtCore.QRect(320, 20, 181, 41))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_13.setFont(font)
        self.label_13.setStyleSheet("background-color:rgb(21,21,39)")
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.tableWidget = QtWidgets.QTableWidget(self.frame)
        self.tableWidget.setGeometry(QtCore.QRect(60, 70, 821, 441))
        self.tableWidget.setStyleSheet("border:none;\n"
"")
        self.tableWidget.setObjectName("tableWidget")
        self.tableWidget.setColumnCount(0)
        self.tableWidget.setRowCount(0)
        self.button_regulator = QtWidgets.QPushButton(self.frame)
        self.button_regulator.setGeometry(QtCore.QRect(410, 560, 101, 41))
        self.button_regulator.setStyleSheet("background-color:green;")
        self.button_regulator.setObjectName("button_regulator")
        self.frame_10 = QtWidgets.QFrame(self.tab_2)
        self.frame_10.setGeometry(QtCore.QRect(910, 10, 921, 661))
        self.frame_10.setStyleSheet("background-color:rgb(21, 21, 39);\n"
"border-radius: 15px; \n"
"border: 2px solid rgb(99, 102, 108);")
        self.frame_10.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_10.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_10.setObjectName("frame_10")
        self.label_19 = QtWidgets.QLabel(self.frame_10)
        self.label_19.setGeometry(QtCore.QRect(380, 20, 211, 41))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_19.setFont(font)
        self.label_19.setAlignment(QtCore.Qt.AlignCenter)
        self.label_19.setObjectName("label_19")
        self.comboBox = QtWidgets.QComboBox(self.frame_10)
        self.comboBox.setGeometry(QtCore.QRect(40, 80, 91, 31))
        self.comboBox.setStyleSheet("border:none;")
        self.comboBox.setFrame(True)
        self.comboBox.setObjectName("comboBox")
        self.label_20 = QtWidgets.QLabel(self.frame_10)
        self.label_20.setGeometry(QtCore.QRect(50, 50, 71, 21))
        self.label_20.setStyleSheet("border:none;")
        self.label_20.setObjectName("label_20")
        self.tableWidget_2 = QtWidgets.QTableWidget(self.frame_10)
        self.tableWidget_2.setGeometry(QtCore.QRect(30, 130, 861, 481))
        self.tableWidget_2.setStyleSheet("border:none;")
        self.tableWidget_2.setObjectName("tableWidget_2")
        self.tableWidget_2.setColumnCount(0)
        self.tableWidget_2.setRowCount(0)
        self.tabWidget.addTab(self.tab_2, "")
        self.gridLayout_2.addWidget(self.tabWidget, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1860, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_2.setText(_translate("MainWindow", "Temperatur og pådrag"))
        self.button_logger.setText(_translate("MainWindow", "Start logging"))
        self.temp_1.setText(_translate("MainWindow", "-------"))
        self.temp_2.setText(_translate("MainWindow", "-------"))
        self.temp_3.setText(_translate("MainWindow", "-------"))
        self.temp_4.setText(_translate("MainWindow", "-------"))
        self.temp_5.setText(_translate("MainWindow", "-------"))
        self.thruster_1.setText(_translate("MainWindow", "0"))
        self.thruster_5.setText(_translate("MainWindow", "0"))
        self.thruster_2.setText(_translate("MainWindow", "0"))
        self.thruster_6.setText(_translate("MainWindow", "0"))
        self.thruster_3.setText(_translate("MainWindow", "0"))
        self.thruster_7.setText(_translate("MainWindow", "0"))
        self.thruster_4.setText(_translate("MainWindow", "0"))
        self.thruster_8.setText(_translate("MainWindow", "0"))
        self.label_8.setText(_translate("MainWindow", "Control Panel"))
        self.label_18.setText(_translate("MainWindow", "Drive mode"))
        self.button_manual.setText(_translate("MainWindow", "Manual"))
        self.button_pipeline.setText(_translate("MainWindow", "Pipeline"))
        self.button_docking.setText(_translate("MainWindow", "Docking"))
        self.label_17.setText(_translate("MainWindow", "Lights"))
        self.button_light.setText(_translate("MainWindow", "Off"))
        self.label_14.setText(_translate("MainWindow", "Battery"))
        self.label_15.setText(_translate("MainWindow", "Voltage:"))
        self.label_16.setText(_translate("MainWindow", "0V"))
        self.button_init_cams.setText(_translate("MainWindow", "Open Cam"))
        self.button_init_nodes.setText(_translate("MainWindow", "Init Nodes"))
        self.status_icon_ros_rov.setText(_translate("MainWindow", "X"))
        self.label_ros_controller_status.setText(_translate("MainWindow", "Not running"))
        self.label_10.setText(_translate("MainWindow", "Status:"))
        self.label_ros_rov_status.setText(_translate("MainWindow", "Not running"))
        self.status_icon_ros_controller.setText(_translate("MainWindow", "X"))
        self.label_7.setText(_translate("MainWindow", "ROS_ROV_node"))
        self.label_9.setText(_translate("MainWindow", "ROS_controller_node"))
        self.label_5.setText(_translate("MainWindow", "Connections"))
        self.label_6.setText(_translate("MainWindow", "Status:"))
        self.label_4.setText(_translate("MainWindow", "Alarms"))
        self.label.setText(_translate("MainWindow", "Navigation"))
        self.label_11.setText(_translate("MainWindow", "Pitch:"))
        self.label_3.setText(_translate("MainWindow", "Roll:"))
        self.label_12.setText(_translate("MainWindow", "Yaw:"))
        self.label_pitch.setText(_translate("MainWindow", "0"))
        self.label_roll.setText(_translate("MainWindow", "0"))
        self.label_yaw.setText(_translate("MainWindow", "0"))
        self.label_22.setText(_translate("MainWindow", "Surge:"))
        self.label_23.setText(_translate("MainWindow", "Sway:"))
        self.label_24.setText(_translate("MainWindow", "Heave:"))
        self.label_surge.setText(_translate("MainWindow", "0"))
        self.label_sway.setText(_translate("MainWindow", "0"))
        self.label_heave.setText(_translate("MainWindow", "0"))
        self.label_21.setText(_translate("MainWindow", "Manipulator"))
        self.button_manipulator.setText(_translate("MainWindow", "Unactive"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Main"))
        self.label_13.setText(_translate("MainWindow", "Regulator"))
        self.button_regulator.setText(_translate("MainWindow", "Send Values"))
        self.label_19.setText(_translate("MainWindow", "Log"))
        self.label_20.setText(_translate("MainWindow", "Trip id:"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Config"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
