
import subprocess
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtWidgets import QWidget
import paramiko
import sys
import rclpy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from PyQt5.QtGui import QPainter, QPen, QTransform
import signal
import sqlite3
import math

from std_msgs.msg import String, Int16
from geometry_msgs.msg import Twist
from cpp_package.msg import Manipulator, PID, ModeControl, LightStrenght
from MainWindow import Ui_MainWindow
from rclpy.node import Node
import time, threading

def handler(signum, frame):
    raise TimeoutError("SSH connection timed out")

class RobotArmView(QWidget):
    def __init__(self, parent=None):
        super(RobotArmView, self).__init__(parent)
        self.setMinimumSize(400, 400)
        self.setWindowTitle("ROV with Robot Arm")
        
        # Define dimensions of the ROV box
        self.rov_width = 100
        self.rov_height = 50
        
        # Joint positions relative to the ROV
        self.joint1_pos = QPointF(100, 30)
        self.joint2_pos = QPointF(150,30)
        self.end_effector_pos = QPointF(200,30)
        
        # Pitch angle of the ROV (in degrees)
        self.pitch_angle = 30

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        
        painter.translate(200, 100)
        painter.rotate(self.pitch_angle)
        painter.translate(-50, -25)  # Adjust position so that the center of the box is at the origin

        # Draw the rotated box
        rect = QRectF(0, 0, self.rov_width, self.rov_height)
        painter.setBrush(Qt.blue)
        painter.setPen(QPen(Qt.black, 2))
        painter.drawRect(rect)

        # Draw joints and arm parts relative to ROV
        painter.drawEllipse(self.joint1_pos , 5, 5)
        painter.drawEllipse(self.joint2_pos , 5, 5)
        painter.drawLine(self.joint1_pos , self.joint2_pos )
        painter.drawLine(self.joint2_pos , self.end_effector_pos )
        painter.drawLine(self.end_effector_pos , self.end_effector_pos )

    def update_arm(self, angle1, angle2):
        # Calculate end effector position based on joint angles
        self.end_effector_pos = QPointF(self.joint2_pos.x() + 50 * math.cos(angle1), self.joint2_pos.y() + 50 * math.sin(angle1))
        self.joint2_pos = QPointF(self.joint1_pos.x() + 50 * math.cos(-angle2), self.joint1_pos.y() + 50 * math.sin(-angle2))
        self.update()
    def setRotation(self, angle):
        self.pitch_angle = angle
        self.update()

class MainWindow(QMainWindow):
    display_message_signal = pyqtSignal(str, str)
    def __init__(self):
        super().__init__()
        self.ui=Ui_MainWindow()
        self.ui.setupUi(self)
        self.ros_rov_status=False
        self.ros_controller_status=False
        self.trip_id=None
        self.logtest1=0
        self.logtest2=0
        self.logtest3=0
        self.robot_arm_view = RobotArmView()
        
        self.connect_ros()
        self.display_message_signal.connect(self.display_message_box)
        self.con=sqlite3.connect('rov_logs.db')
        self.c = self.con.cursor()
        self.log_timer = QTimer(self)  # Create QTimer object
        self.log_timer.timeout.connect(self.log_data)  # Connect timeout signal to log_data method
        self.log_timer.start(1000) 

        # Create table if not exists
        self.c.execute('''CREATE TABLE IF NOT EXISTS rov_trips
                    (id INTEGER PRIMARY KEY AUTOINCREMENT,
                    start_timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    end_timestamp TIMESTAMP,
                    UNIQUE(start_timestamp))''')
        self.c.execute('''CREATE TABLE IF NOT EXISTS rov_logs
                    (id INTEGER PRIMARY KEY AUTOINCREMENT,
                    trip_id INTEGER,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    temp_1 REAL,
                    temp_2 REAL,
                    temp_3 REAL,
                    temp_4 REAL,
                    temp_5 REAL,
                    pressure REAL,
                    leakage STRING)''')
        self.con.commit()
        self.initUI()
    def display_message_box(self,title,message):
        QMessageBox.critical(self,title,message)
    def initUI(self):
        self.node=None
        self.setWindowTitle('ROS Control GUI')
        self.ui.button_init_cams.clicked.connect(self.init_cams)
        self.ui.button_manual.clicked.connect(self.drive_mode)
        self.ui.button_pipeline.clicked.connect(self.drive_mode)
        self.ui.button_docking.clicked.connect(self.drive_mode)
        self.ui.button_logger.clicked.connect(self.start_logging)
        self.ui.button_front_light.clicked.connect(self.toggle_lights)
        self.ui.button_bottom_light.clicked.connect(self.toggle_lights)
        self.ui.button_light_master.clicked.connect(self.toggle_lights)
        self.ui.button_manipulator.clicked.connect(self.manipulator)
        self.ui.horizontalSlider_front_light.valueChanged.connect(self.toggle_lights)
        self.ui.horizontalSlider_front_light.setVisible(False)
        self.ui.horizontalSlider_bottom_light.valueChanged.connect(self.toggle_lights)
        self.ui.horizontalSlider_bottom_light.setVisible(False)
        self.player = QMediaPlayer()
        self.ui.tableWidget.setColumnCount(6)
        self.ui.tableWidget.setHorizontalHeaderLabels(('P','I','D','Verdi','Mål','Ønsket'))
        self.ui.tableWidget_2.setColumnCount(8)  # Adjust column count as per your database structure
        self.ui.tableWidget_2.setHorizontalHeaderLabels(['Timestamp', 'temp1','temp2','temp3','temp4','temp5','pressure','leakage'])  # Set column headers
        self.populate_trip_combobox()  # Populate trip combo box initially
        self.ui.comboBox.currentIndexChanged.connect(self.display_trip)  # Connect combo box index change event
        layout = QVBoxLayout()
        layout.addWidget(self.robot_arm_view,stretch=1)
        
        self.ui.widget.setLayout(layout)
        self.ui.horizontalSlider_2.valueChanged.connect(self.update_arm)
        self.ui.horizontalSlider_3.valueChanged.connect(self.update_arm)
        
        for row in range(6):
            self.ui.tableWidget.insertRow(row)
            for col in range(6):
                item = QTableWidgetItem("0")
                self.ui.tableWidget.setItem(row, col, item)
        self.ui.tableWidget.setVerticalHeaderLabels(('Pitch','Roll','Yaw','Speed X','Speed y','Speed z'))
        self.ui.button_regulator.clicked.connect(self.regulator_pid)
    def regulator_pid(self):
        for row in range(self.ui.tableWidget.rowCount()):
            item = self.ui.tableWidget.item(row, 5)  # Get item from first column
            if item is not None:
                # Set the text of the corresponding item in the second column
                self.ui.tableWidget.setItem(row, 4, item.clone())
                self.ui.tableWidget.setItem(row,5,None)
        msg=PID()

        msg.pitch_p=float(self.ui.tableWidget.item(0,0).text())
        msg.pitch_i=float(self.ui.tableWidget.item(0,1).text())
        msg.pitch_d=float(self.ui.tableWidget.item(0,2).text())
        msg.roll_p=float(self.ui.tableWidget.item(1,0).text())
        msg.roll_i=float(self.ui.tableWidget.item(1,1).text())
        msg.roll_d=float(self.ui.tableWidget.item(1,2).text())
        msg.yaw_p=float(self.ui.tableWidget.item(2,0).text())
        msg.yaw_i=float(self.ui.tableWidget.item(2,1).text())
        msg.yaw_d=float(self.ui.tableWidget.item(2,2).text())
        msg.x_p=float(self.ui.tableWidget.item(3,0).text())
        msg.x_i=float(self.ui.tableWidget.item(3,1).text())
        msg.x_d=float(self.ui.tableWidget.item(3,2).text())
        msg.y_p=float(self.ui.tableWidget.item(4,0).text())
        msg.y_i=float(self.ui.tableWidget.item(4,1).text())
        msg.y_d=float(self.ui.tableWidget.item(4,2).text())
        msg.z_p=float(self.ui.tableWidget.item(5,0).text())
        msg.z_i=float(self.ui.tableWidget.item(5,1).text())
        msg.z_d=float(self.ui.tableWidget.item(5,2).text())
        print(msg.pitch_p)
        self.pub_pid.publish(msg)



    
    #only for testing / convert to a callback for angle topic
    def update_arm(self):
        angle1 = math.radians(self.ui.horizontalSlider_2.value())  # Convert slider value to radians
        angle2 = math.radians(self.ui.horizontalSlider_3.value())  # Convert slider value to radians
        self.robot_arm_view.update_arm(angle1, angle2)
    def drive_mode(self):
        self.ui.button_manual.setStyleSheet(" background-color:rgb(21, 21, 39)")
        self.ui.button_pipeline.setStyleSheet(" background-color:rgb(21, 21, 39)")
        self.ui.button_docking.setStyleSheet("background-color:rgb(21, 21, 39)")
        sender=self.sender()
        sender.setStyleSheet(" background-color:green")
        msg=ModeControl()
        if sender==self.ui.button_manual:
            msg.drive_mode="Manual"
            self.ui.button_manual.setChecked(True)
        if sender==self.ui.button_pipeline:
            msg.drive_mode="Pipeline"
            self.ui.button_manual.setChecked(False)
            self.manipulator()
        if sender==self.ui.button_docking:
            msg.drive_mode="Docking"
            self.ui.button_manual.setChecked(False)
            self.manipulator()
        self.pub_mode.publish(msg)
        
        
    def start_logging(self):
        if self.trip_id is None:  # Start new trip
            self.c.execute('''INSERT INTO rov_trips DEFAULT VALUES''')
            self.con.commit()
            self.trip_id = self.c.lastrowid
            self.ui.button_logger.setText('Stop Logger')
            self.ui.button_logger.setStyleSheet("background-color:red;")
            print("Trip started. Trip ID:", self.trip_id)
        else:  # End current trip
            self.c.execute('''UPDATE rov_trips SET end_timestamp = CURRENT_TIMESTAMP WHERE id = ?''', (self.trip_id,))
            self.con.commit()
            self.trip_id = None
            self.ui.button_logger.setText('Start Logger')
            self.ui.button_logger.setStyleSheet("background-color:green;")
            
            print("Trip ended.")
    def log_data(self):
        if self.trip_id is not None:
            value=float(self.ui.thruster_1.text())
            if value>0.6:
                test='Sensor 1'
            elif value>0.2:
                test='Sensor 2'
            elif value>-0.2:
                test='sensor 3'
            else:
                test='sensor 4'
            self.c.execute('''INSERT INTO rov_logs (trip_id, leakage) VALUES (?, ?)''', (self.trip_id, test))
            self.con.commit()
            self.populate_trip_combobox()
            self.display_trip()
    
    def populate_trip_combobox(self):
        self.ui.comboBox.clear()
        self.c.execute('''SELECT id, start_timestamp FROM rov_trips''')
        trips = self.c.fetchall()
        for trip in trips:
            self.ui.comboBox.addItem(f'Trip {trip[0]} ({trip[1]})')       
    def display_trip(self):
        selected_index = self.ui.comboBox.currentIndex()
        if selected_index >= 0:
            trip_id = selected_index + 1  # Trip ID starts from 1, while combo box index starts from 0
            self.c.execute('''SELECT * FROM rov_logs WHERE trip_id = ?''', (trip_id,))
            rows = self.c.fetchall()
            self.ui.tableWidget_2.setRowCount(len(rows))  # Set row count based on database content
            for i, row in enumerate(rows):
                for j, value in enumerate(row[2:], start=2):
                    item = QTableWidgetItem(str(value))
                    self.ui.tableWidget_2.setItem(i, j-2, item) 
    def toggle_lights(self):
        sender=self.sender()
        msg=LightStrenght()
        if sender== self.ui.button_light_master:
            if self.ui.button_light_master.isChecked():
                self.ui.button_light_master.setText("On")
                self.ui.button_light_master.setStyleSheet("background-color:green;")
                self.ui.button_front_light.setChecked(True)
                self.ui.button_bottom_light.setChecked(True)
            else:
                self.ui.button_light_master.setText("Off")
                self.ui.button_light_master.setStyleSheet("background-color:red;")
                self.ui.button_front_light.setChecked(False)
                self.ui.button_bottom_light.setChecked(False)
        
        if self.ui.button_front_light.isChecked():
            self.ui.button_front_light.setText("On")
            self.ui.button_front_light.setStyleSheet("background-color:green;")
            self.ui.horizontalSlider_front_light.setVisible(True)
            msg.front_lights=self.ui.horizontalSlider_front_light.value()
            self.ui.button_light_master.setChecked(True)
            self.ui.button_light_master.setText("On")
            self.ui.button_light_master.setStyleSheet("background-color:green;")
            
        else:
            self.ui.button_front_light.setText("Off")
            self.ui.button_front_light.setStyleSheet("background-color:red;")
            self.ui.horizontalSlider_front_light.setVisible(False)
            msg.front_lights=0
        
        if self.ui.button_bottom_light.isChecked():
            self.ui.button_bottom_light.setText("On")
            self.ui.button_bottom_light.setStyleSheet("background-color:green;")
            self.ui.horizontalSlider_bottom_light.setVisible(True)
            msg.bottom_lights=self.ui.horizontalSlider_bottom_light.value()
            self.ui.button_light_master.setChecked(True)
            self.ui.button_light_master.setText("On")
            self.ui.button_light_master.setStyleSheet("background-color:green;")
            
        else:
            self.ui.button_bottom_light.setText("Off")
            self.ui.button_bottom_light.setStyleSheet("background-color:red;")
            self.ui.horizontalSlider_bottom_light.setVisible(False)
            msg.bottom_lights=0
        self.pub_light.publish(msg)
        

    def manipulator(self):
        msg=ModeControl()
        if not self.ui.button_manual.isChecked():
            self.ui.button_manipulator.setChecked(False)
            if self.sender()==self.ui.button_manipulator:
                self.display_message_box("Warning","Manual drive mode must be active to engage manipulator")

        if self.ui.button_manipulator.isChecked():
            if int(self.ui.connected_controllers.text())<2:
                self.display_message_box("Warning","Need to have 2 controllers connected to initialize the manipulator")
            else:
                self.ui.button_manipulator.setText("Active")
                self.ui.button_manipulator.setStyleSheet("background-color:green;")
                msg.manipulator=True
                msg.drive_mode="Manual"
        else:
            self.ui.button_manipulator.setText("Unactive")
            self.ui.button_manipulator.setStyleSheet("background-color:red;")
            msg.manipulator=False
            msg.drive_mode= "Manual"
        self.pub_mode.publish(msg)
        
            
    def check_ros_connectivity(self):
        process = subprocess.Popen(['ros2','node', 'list'], stdout=subprocess.PIPE)
        stdout = process.communicate()
        if "rov_node" in stdout[0].decode():
            self.ui.label_ros_rov_status.setText("Running")
            self.ui.status_icon_ros_rov.setText("OK")
            self.ui.status_icon_ros_rov.setStyleSheet("color:green")
            self.ros_rov_status=True
        else:
            if self.ros_rov_status == True:
                self.ui.label_ros_rov_status.setText("Not Running")
                self.ui.status_icon_ros_rov.setText("X")
                self.ui.status_icon_ros_rov.setStyleSheet("color:red")
                self.display_message_signal.emit("NodeError", "ROV node was disconnected")
                self.ros_rov_status=False
        if "controller_node" in stdout[0].decode():
            self.ui.label_ros_controller_status.setText("Running")
            self.ui.status_icon_ros_controller.setText("OK")
            self.ui.status_icon_ros_controller.setStyleSheet("color:green")
            self.ros_controller_status=True
        else:
            if self.ros_controller_status == True:
                self.ui.label_ros_controller_status.setText("Not Running")
                self.ui.status_icon_ros_controller.setText("X")
                self.ui.status_icon_ros_controller.setStyleSheet("color:red")
                self.display_message_signal.emit("NodeError", "Controller node was disconnected")
                self.ros_controller_status=False
        
        
        if self.ros_running==True:
            threading.Timer(1, self.check_ros_connectivity).start()
        
        

    def connect_ros(self):
        rclpy.init(args=None)
        self.node = Node('gui_node')
        self.ros_running=True
        self.check_ros_connectivity()
        self.pub_mode=self.node.create_publisher(ModeControl,'mode_control',10)
        self.pub_light=self.node.create_publisher(LightStrenght,"light_control",10)
        self.sub_connected_controllers=self.node.create_subscription(Int16,"connected_controllers",self.controller_callback,10)
        self.pub_pid=self.node.create_publisher(PID,'test',10)
        self.sub=self.node.create_subscription(PID,'test',self.listener_callback,10)
        self.sub_test_rotation=self.node.create_subscription(Twist,'ROV_movement',self.callback,10)
        ros_thread=threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        ros_thread.start()
        
    
    def listener_callback(self,msg):
        
        #self.ui.thruster_1.setText(str(msg.pitch_p))
        print(msg.pitch_p)
        #self.ui.thruster_2.setText(msg.pitch_i)
        #self.ui.thruster_3.setText(msg.pitch_d)
    
    def callback(self,msg):
        self.robot_arm_view.setRotation(msg.angular.y)
        self.ui.label_roll.setText(str(int(msg.angular.x)))
        self.ui.label_pitch.setText(str(int(msg.angular.y)))
        self.ui.label_yaw.setText(str(int(msg.angular.z)))
        self.ui.label_surge.setText(str(msg.linear.x))
        self.ui.label_sway.setText(str(msg.linear.y))
        self.ui.label_heave.setText(str(msg.linear.z))
        
    def controller_callback(self,msg):
        
        if int(self.ui.connected_controllers.text())> msg.data:
            self.display_message_signal.emit("Warning","Controller disconnected")
        self.ui.connected_controllers.setText(str(msg.data))
        
            

    def closeEvent(self,event):
        if self.node:
            self.node.destroy_node()
            self.node=None
            rclpy.Executor.shutdown()
            rclpy.shutdown()
        self.ros_running=False
        self.con.close()
        event.accept()
    
    
    def init_cams(self):
        
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        try:
            signal.signal(signal.SIGALRM, handler)
            signal.alarm(5) 
            # Connect to the SSH server
            ssh_client.connect(hostname='10.0.0.2', username='subsea', password='subsea88')
            signal.alarm(0)
            # Execute the command
            stdin, stdout, stderr = ssh_client.exec_command('gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink host=10.0.0.1 port=5000')

            # Read the output
            output = stdout.read().decode()
            media_content = QMediaContent(QUrl("gst-pipeline: udpsrc port = 5000 caps= \"application/x-rtp, payload=96\" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! xvimagesink name=\"qtvideosink\""))
            self.player.setMedia(media_content)
            # Play the video
            self.player.play()

        except TimeoutError:
            print("SSH connection timed out")
            QMessageBox.critical(self,"TimeOutError","SSH connection timed out")
            
        
        except Exception as e:
            print(f"Error: {e}")
            QMessageBox.critical(f"Error: {e}")
            
            
        finally:
            # Close the SSH connection
            ssh_client.close()
            # Define the GStreamer command
    
           
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

