import subprocess
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget
import paramiko
import sys
import rclpy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
import signal

from std_msgs.msg import String
from cpp_package.msg import Manipulator
from MainWindow import Ui_MainWindow
from rclpy.node import Node
import time, threading

def handler(signum, frame):
    raise TimeoutError("SSH connection timed out")

class MainWindow(QMainWindow):
    display_message_signal = pyqtSignal(str, str)
    def __init__(self):
        super().__init__()
        self.ui=Ui_MainWindow()
        self.ui.setupUi(self)
        self.initUI()
        self.connect_ros()
        self.display_message_signal.connect(self.display_message_box)
    def display_message_box(self,title,message):
        QMessageBox.critical(self,title,message)
    def initUI(self):
        self.node=None
        self.setWindowTitle('ROS Control GUI')
        self.ui.button_init_cams.clicked.connect(self.init_cams)
        self.ui.button_manual.clicked.connect(self.drive_mode)
        self.ui.button_pipeline.clicked.connect(self.drive_mode)
        self.ui.button_docking.clicked.connect(self.drive_mode)
        self.ui.radioButton.toggled.connect(self.toggle_lights)
        self.ui.horizontalSlider.valueChanged.connect(self.toggle_lights)
        self.player = QMediaPlayer()
        self.ros_sim_status=False
        self.ros_controller_status=False
        self.ui.tableWidget.setColumnCount(6)
        self.ui.tableWidget.setHorizontalHeaderLabels(('P','I','D','Verdi','Mål','Ønsket'))
        
        for row in range(6):
            self.ui.tableWidget.insertRow(row)
            for col in range(6):
                item = QTableWidgetItem("")
                self.ui.tableWidget.setItem(row, col, item)
        self.ui.tableWidget.setVerticalHeaderLabels(('Pitch','Roll','Yaw','Speed X','Speed y','Speed z'))
        self.ui.button_regulator.clicked.connect(self.regulator_pid)
    def regulator_pid(self):
        print(self.ui.tableWidget.item(0,0).text())
    
    def drive_mode(self):
        self.ui.button_manual.setStyleSheet("color:white; background-color:rgb(61, 56, 70)")
        self.ui.button_pipeline.setStyleSheet("color:white; background-color:rgb(61, 56, 70)")
        self.ui.button_docking.setStyleSheet("color:white; background-color:rgb(61, 56, 70)")
        sender=self.sender()
        sender.setStyleSheet("color:black; background-color:rgb(33, 192, 60)")
        msg=String()
        if sender==self.ui.button_manual:
            msg.data="Manual"
            
        if sender==self.ui.button_pipeline:
            msg.data="Pipeline"
        if sender==self.ui.button_docking:
            msg.data="Docking"
        self.pub.publish(msg)
        print(msg)
        

    def toggle_lights(self):
        if self.ui.radioButton.isChecked():
            
            light_strenght=self.ui.horizontalSlider.value()
            
        else:
            light_strenght=0
        print(light_strenght)
        


    def check_ros_connectivity(self):
        process = subprocess.Popen(['ros2','node', 'list'], stdout=subprocess.PIPE)
        stdout = process.communicate()
        if "sim_node" in stdout[0].decode():
            self.ui.label_ros_sim_status.setText("Running")
            self.ui.status_icon_ros_sim.setText("OK")
            self.ui.status_icon_ros_sim.setStyleSheet("color:green")
            self.ros_sim_status=True
        else:
            if self.ros_sim_status == True:
                self.ui.label_ros_sim_status.setText("Not Running")
                self.ui.status_icon_ros_sim.setText("X")
                self.ui.status_icon_ros_sim.setStyleSheet("color:red")
                self.display_message_signal.emit("NodeError", "Sim node was disconnected")
                self.ros_sim_status=False
        if "controller_pub_node" in stdout[0].decode():
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
        self.pub=self.node.create_publisher(String,'mode_control',10)
        self.sub=self.node.create_subscription(Manipulator,'sim_data',self.listener_callback,10)
        ros_thread=threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        ros_thread.start()
        
    
    def listener_callback(self,msg):
        
        self.ui.lcdNumber.display(msg.field1)
        
        
        

    def closeEvent(self,event):
        self.node.destroy_node()
        self.node=None
        rclpy.shutdown()
        self.ros_running=False
        event.accept()
    
    
    def init_cams(self):
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            signal.signal(signal.SIGALRM, handler)
            signal.alarm(5) 
            # Connect to the SSH server
            ssh_client.connect(hostname='10.0.0.2', username='cj', password='doWnhi11')
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

