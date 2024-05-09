
import subprocess
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtWidgets import QWidget
import paramiko
import sys
import rclpy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from PyQt5.QtGui import QPainter, QPen, QDesktopServices,QPixmap
import signal
import sqlite3
import math
import folium
from std_msgs.msg import String, Int16, Int8
from geometry_msgs.msg import Twist
from cpp_package.msg import Kom03, Kom01Kom02, ModeControl, LightStrenght, ThrusterRange, Reg00, Sen02, PingStatus
from MainWindow import Ui_MainWindow
from rclpy.node import Node
import time, threading
from PyQt5.QtWebEngineWidgets import QWebEngineView
import io

def handler(signum, frame):
    raise TimeoutError("SSH connection timed out")
class SSHThread(QThread):
    output_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super(SSHThread, self).__init__(parent)

    def run(self):
        import paramiko
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh_client.connect(hostname='10.0.0.2', username='subsea', password='subsea88')
            command = """
            source /opt/ros/humble/setup.bash && \
            source ~/ros_ws/install/local_setup.bash && \
            /usr/bin/python3 Documents/CanTesting/socketCan.py
            """
            stdin, stdout, stderr = ssh_client.exec_command(command)
            output = stdout.read().decode()
            error = stderr.read().decode()
            if error:
                self.error_signal.emit(error)
            else:
                self.output_signal.emit(output)
        except Exception as e:
            self.error_signal.emit(str(e))
        finally:
            ssh_client.close()
class CriticalMessage(QMessageBox):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Water Leakage Detected")
        self.setText("Water leakage detected! Please take immediate action.")
        self.setIcon(QMessageBox.Warning)
        
        # Set red background color with transparency
        self.setStyleSheet("background-color: rgba(255, 0, 0, 100); color: white;")

        self.flash_timer = QTimer(self)
        self.flash_timer.timeout.connect(self.toggle_color)
        
        # Start the flashing effect
        self.flash_timer.start(500)  # Flash every 500 milliseconds

    def toggle_color(self):
        # Toggle between red and transparent background color
        current_color = self.palette().color(self.backgroundRole())
        if current_color.red() == 255:  # If current color is red, set it to transparent
            self.setStyleSheet("background-color: rgb(21, 21,39); color: white;")
        else:  # If current color is transparent, set it to red
            self.setStyleSheet("background-color: rgba(255, 0, 0, 100); color: white;")

class RobotArmView(QWidget):
    def __init__(self, parent=None):
        super(RobotArmView, self).__init__(parent)
        self.setMinimumSize(531, 321)
        self.setWindowTitle("ROV with Robot Arm")
        
        # Load PNG images for the box and arm
        self.rov_image = QPixmap("images/rov_from_side.png",)  # Adjust the path to your ROV image
        self.arm_segment1_image = QPixmap("images/arm_1.png")  # Adjust the path to your arm segment image
        self.arm_segment2_image = QPixmap("images/arm_2_test.png")
        # Define dimensions of the ROV box
        self.rov_width = 214
        self.rov_height = 126
        
        # Joint positions relative to the ROV
        self.joint1_pos = QPointF(200, 100)
        # Pitch angle of the ROV (in degrees)
        self.pitch_angle = 270
        self.joint1_angle=0
        self.joint2_angle=0

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        if self.pitch_angle>=45 and self.pitch_angle <=135:
            painter.translate(257,103)
        elif self.pitch_angle>=225 and self.pitch_angle <=315:
            painter.translate(257,183)
        else:
            painter.translate(257, 113) #200,100 if 45 degres not working
        painter.rotate(self.pitch_angle)
        painter.translate(-107, -63)  # Adjust position so that the center of the box is at the origin

        
        # Draw arm segments
        painter.translate(self.joint1_pos)  # Move to joint1 position 
        painter.rotate(self.joint1_angle)  # Rotate around joint1
       
        
        # Move to the end of the first arm segment (joint2 position)
        painter.translate(42, -2) 
        # Rotate around joint2
        painter.rotate(self.joint2_angle)  
        
        painter.drawPixmap(0, -10, 55, 30, self.arm_segment2_image)
        painter.rotate(-self.joint2_angle)
        painter.translate(-42,2)
        painter.drawPixmap(0, -10, 50, 30, self.arm_segment1_image)
        painter.rotate(-self.joint1_angle)
        painter.translate(-self.joint1_pos)
        # Draw the ROV box image
        painter.setOpacity(0.5)
        painter.drawPixmap(0, 0, self.rov_width, self.rov_height, self.rov_image)
    def update_arm(self, angle1, angle2):
        # Reset transformation matrix
        self.joint1_angle=-angle1
        self.joint2_angle=angle2
        
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
                    temp_1 FLOAT,
                    temp_2 FLOAT,
                    temp_3 FLOAT,
                    temp_4 FLOAT,
                    temp_5 FLOAT,
                    pressure FLOAT,
                    leakage STRING)''')
        self.con.commit()
        self.initUI()
    def display_message_box(self,title,message):
        QMessageBox.critical(self,title,message)

    def test_popup(self):
        message=CriticalMessage()
        message.exec()
    def initUI(self):
        self.node=None
        self.setWindowTitle('ROS Control GUI')
        self.ui.button_control_rov_node.clicked.connect(self.init_rov_node)
        self.ui.button_init_nodes.clicked.connect(self.test_function)
        self.ui.button_manual.clicked.connect(self.drive_mode)
        self.ui.button_pipeline.clicked.connect(self.drive_mode)
        self.ui.button_docking.clicked.connect(self.drive_mode)
        self.ui.button_logger.clicked.connect(self.start_logging)
        self.ui.button_front_light.clicked.connect(self.toggle_lights)
        self.ui.button_bottom_light.clicked.connect(self.toggle_lights)
        #self.ui.button_pinging.clicked.connect(self.start_pinging)
        self.ui.button_power_off.clicked.connect(self.power_mode)
        self.ui.button_cabel.clicked.connect(self.power_mode)
        self.ui.button_battery.clicked.connect(self.power_mode)
        self.ui.button_hybrid.clicked.connect(self.power_mode)
        self.ui.button_stream_config.clicked.connect(self.open_website)
        self.ui.button_controller_node.clicked.connect(self.init_controller_node)
        self.sensor_callback("test")
        
        
        self.ui.button_manipulator.clicked.connect(self.manipulator)
        self.ui.spinBox.valueChanged.connect(self.update_thruster_range)
        self.ui.spinBox_2.valueChanged.connect(self.update_thruster_range)
        self.ui.horizontalSlider_front_light.valueChanged.connect(self.toggle_lights)
        self.ui.horizontalSlider_front_light.setVisible(False)
        self.ui.horizontalSlider_bottom_light.valueChanged.connect(self.toggle_lights)
        self.ui.horizontalSlider_bottom_light.setVisible(False)
        self.player = QMediaPlayer()
        self.ui.tableWidget.setColumnCount(6)
        self.ui.tableWidget.setHorizontalHeaderLabels(('P','I','D','Verdi','Mål','Ønsket'))
        self.ui.tableWidget_2.setColumnCount(8)  # Adjust column count as per your database structure
        self.ui.tableWidget_2.setHorizontalHeaderLabels(['Timestamp', 'temp1','temp2','temp3','temp4','temp5','pressure','leakage'])  # Set column headers
        self.populate_trip_combobox() # Populate trip combo box initially
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
        #self.ui.gps_widget.setStyleSheet("background-color:red;")
        self.init_map()
    
    def meters_to_latlon(self,start_lat, start_lon, delta_x, delta_y):
        # Earth's radius in meters along one degree of latitude
        lat_meters = 111139

        # Convert latitude and adjust longitude according to the cosine of the average latitude
        delta_lat = delta_y / lat_meters
        delta_lon = delta_x / (lat_meters * math.cos(math.radians(start_lat)))

        new_lat = start_lat + delta_lat
        new_lon = start_lon + delta_lon

        return new_lat, new_lon
    def init_map(self):
        self.start_lat, self.start_lon = 59.092078, 5.910311 
        self.coordinates_list = [(self.start_lat, self.start_lon)] # Example: Portland, OR
        self.m = folium.Map(location=[self.start_lat, self.start_lon], zoom_start=19)
        folium.Marker([self.start_lat, self.start_lon], popup='Start Point', icon=folium.Icon(color='red')).add_to(self.m)
        # Save the map as an HTML file
        data=io.BytesIO()
        self.m.save(data,close_file=False)
        
        #Create a QWebEngineView to display the HTML
        self.map_view = QWebEngineView()

        # Load the HTML file in the QWebEngineView
        self.map_view.setHtml(data.getvalue().decode())
        # Set the QWebEngineView as a widget in gps_widget or replace gps_widget with it
        layout = QVBoxLayout()
        layout.addWidget(self.map_view)
        self.ui.gps_widget.setLayout(layout)
    def draw_map(self,x,y):
        
        new_lat, new_lon = self.meters_to_latlon(self.start_lat, self.start_lon, x, y)
        self.coordinates_list.append((new_lat, new_lon))  # Append the new position to the list
        folium.PolyLine(self.coordinates_list, color="blue", weight=2.5, opacity=1).add_to(self.m)
        data=io.BytesIO()
        self.m.save(data,close_file=False)
        self.map_view.setHtml(data.getvalue().decode())

    def regulator_pid(self):
        for row in range(self.ui.tableWidget.rowCount()):
            item = self.ui.tableWidget.item(row, 5)  # Get item from first column
            if item is not None:
                # Set the text of the corresponding item in the second column
                self.ui.tableWidget.setItem(row, 4, item.clone())
                self.ui.tableWidget.setItem(row,5,None)
        msg=Kom01Kom02()

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
    def toggle_animation(self,button,frame,circle):
        self.anim= QPropertyAnimation(circle,b"pos")
        if button.isChecked():
            self.anim.setEndValue(QPoint(27,3))
            self.anim.setDuration(200)
            self.anim.start()
            frame.setStyleSheet("background-color:green;")
        else:
            self.anim.setEndValue(QPoint(3,3))
            self.anim.setDuration(200)
            self.anim.start()
            frame.setStyleSheet("background-color:grey;")
    def open_website(self):
        # URL you want to open
        url = QUrl("http://10.0.0.2:5000")
        QDesktopServices.openUrl(url)
    def update_thruster_range(self):
        msg=ThrusterRange()
        msg.start=self.ui.spinBox.value()
        msg.end=self.ui.spinBox_2.value()
        print(msg)
        self.pub_thruster_input_range.publish(msg)
    def init_controller_node(self):
        thread = threading.Thread(target=self.control_controller_node,daemon=True)
        thread.start()
    def control_controller_node(self):
        self.toggle_animation(self.ui.button_controller_node,self.ui.togglebutton_background_5,self.ui.togglebutton_circle_5)
        if self.ui.button_controller_node.isChecked():
            command = ["ros2", "run", "robot_controller", "controller_node"]
            # Run the command
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

            # Check if the command was successful
            if result.returncode == 0:
                print("Command executed successfully!")
                
            else:
                print("Error in running command:")
                print(result.stderr)
                        
        else:
            msg=Int8()
            self.pub_stop_controller.publish(msg)
    # def start_pinging(self):
    #     msg=PingStart()
    #     if self.ui.button_pinging.isChecked():
    #         msg.pinging=1
    #     else:
    #         msg.pinging=0
    #     self.pub_ping.publish(msg)

    def test_function(self):
        self.draw_map(2,3)
        self.draw_map(2,8)
        self.draw_map(-5,8)
        self.draw_map(-12,4)
        self.draw_map(-11,-2)
        self.draw_map(-6,-4)
        self.draw_map(2,2)
        self.draw_map(0,0)
        
    
    #only for testing / convert to a callback for angle topic
    def update_arm(self):
        angle1 = self.ui.horizontalSlider_2.value()
        angle2 = self.ui.horizontalSlider_3.value() 
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
        else:
            self.ui.button_manipulator.setChecked(False)
            self.manipulator()
        if sender==self.ui.button_pipeline:
            msg.drive_mode="Pipeline"
            self.ui.button_manual.setChecked(False)
            
        if sender==self.ui.button_docking:
            msg.drive_mode="Docking"
            self.ui.button_manual.setChecked(False)
        self.pub_mode.publish(msg)
    def power_mode(self):
        self.ui.button_power_off.setStyleSheet(" background-color:rgb(21, 21, 39)")
        self.ui.button_cabel.setStyleSheet(" background-color:rgb(21, 21, 39)")
        self.ui.button_battery.setStyleSheet(" background-color:rgb(21, 21, 39)")
        self.ui.button_hybrid.setStyleSheet("background-color:rgb(21, 21, 39)")
        sender=self.sender()
        msg=Int16()
        if sender==self.ui.button_power_off:
            sender.setStyleSheet(" background-color:red")
            msg.data=0
        else:
            sender.setStyleSheet(" background-color:green")
            if sender==self.ui.button_cabel:
                msg.data=1
            if sender==self.ui.button_hybrid:
                msg.data=2
            if sender==self.ui.button_battery:
                msg.data=3
            
                
        self.pub_powermode.publish(msg)
            
        
        
        
        
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
            self.c.execute('''INSERT INTO rov_logs (trip_id,temp_1,temp_2,temp_3,temp_4,temp_5,pressure, leakage) VALUES (?, ?, ?, ?, ?, ?, ?, ?)''', (self.trip_id,float(self.ui.temp_1.text()),float(self.ui.temp_2.text()),float(self.ui.temp_3.text()),float(self.ui.temp_4.text()),float(self.ui.water_temp.text()),float(self.ui.pressure.text()), test)),
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
        
       
        if sender==self.ui.button_front_light:
            self.toggle_animation(sender,self.ui.togglebutton_background_1,self.ui.togglebutton_circle_1)
        if sender==self.ui.button_bottom_light:
            self.toggle_animation(sender,self.ui.togglebutton_background_2,self.ui.togglebutton_circle_2)
        if self.ui.button_front_light.isChecked():
            self.ui.horizontalSlider_front_light.setVisible(True)
            msg.front_light=self.ui.horizontalSlider_front_light.value()
        else:
            self.ui.horizontalSlider_front_light.setVisible(False)
            msg.front_light=0
    
        if self.ui.button_bottom_light.isChecked():
            self.ui.horizontalSlider_bottom_light.setVisible(True)
            msg.bottom_light=self.ui.horizontalSlider_bottom_light.value()
        else:
            self.ui.horizontalSlider_bottom_light.setVisible(False)
            msg.bottom_light=0
        print(msg)
        self.pub_light.publish(msg)
        

    def manipulator(self):
        msg=ModeControl()
        
        if self.ui.button_manipulator.isChecked():
            if self.ui.button_manual.isChecked():
                if int(self.ui.connected_controllers.text())>=2:
                    self.toggle_animation(self.ui.button_manipulator,self.ui.togglebutton_background_3,self.ui.togglebutton_circle_3)
                else:
                    self.display_message_box("Warning", "There needs to be at least 2 controllers connected to engage the manipulator")
                    self.ui.button_manipulator.setChecked(False)
            else:
                self.display_message_box("Warning", "Manual driving must be active to engage the manipulator")
                self.ui.button_manipulator.setChecked(False)
        else:
            self.toggle_animation(self.ui.button_manipulator,self.ui.togglebutton_background_3,self.ui.togglebutton_circle_3)

        
            
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
        self.pub_pid=self.node.create_publisher(Kom01Kom02,'Kom01Kom02',10)
        self.pub_thruster_input_range=self.node.create_publisher(ThrusterRange,"thruster_input_range",10)
        self.pub_stop_rov_node=self.node.create_publisher(Int8,"rov_node_stop",10)
        self.sub_sensor=self.node.create_subscription(Sen02,"Sen02",self.sensor_callback,10)
        self.sub_thruster=self.node.create_subscription(Reg00,'Reg00',self.thruster_callback,10)
        self.sub_ping=self.node.create_subscription(PingStatus,'PingStatus',self.ping_callback,10)
        self.pub_powermode=self.node.create_publisher(Int16,"powermode",10)
        self.pub_stop_controller=self.node.create_publisher(Int8,"controller_node_stop",10)
        ros_thread=threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        ros_thread.start()
        
    def ping_callback(self,msg):
        self.ui.ping_1.setText(str(msg.ping1))
        self.ui.ping_2.setText(str(msg.ping2))
        self.ui.ping_3.setText(str(msg.ping3))
        self.ui.ping_4.setText(str(msg.ping4))
        self.ui.ping_5.setText(str(msg.ping5))
    def battery_callback(self,msg):
        test=0.69
        self.ui.battery_percentage.setText(str(test))
        self.percentage_bar(self.ui.battery_percentage_frame,test)
    def thruster_callback(self,msg):
        thruster1=round(msg.verdi_1/100,2)
        thruster2=round(msg.verdi_2/100,2)
        thruster3=round(msg.verdi_3/100,2)
        thruster4=round(msg.verdi_4/100,2)
        thruster5=round(msg.verdi_5/100,2)
        thruster6=round(msg.verdi_6/100,2)
        thruster7=round(msg.verdi_7/100,2)
        thruster8=round(msg.verdi_8/100,2)
        self.ui.thruster_1.setText(str(thruster1))
        self.percentage_bar(self.ui.thruster_frame_1,thruster1)
        self.ui.thruster_2.setText(str(thruster2))
        self.percentage_bar(self.ui.thruster_frame_2,thruster2)
        self.ui.thruster_3.setText(str(thruster3))
        self.percentage_bar(self.ui.thruster_frame_3,thruster3)
        self.ui.thruster_4.setText(str(thruster4))
        self.percentage_bar(self.ui.thruster_frame_4,thruster4)
        self.ui.thruster_5.setText(str(thruster5))
        self.percentage_bar(self.ui.thruster_frame_5,thruster5)
        self.ui.thruster_6.setText(str(thruster6))
        self.percentage_bar(self.ui.thruster_frame_6,thruster6)
        self.ui.thruster_7.setText(str(thruster7))
        self.percentage_bar(self.ui.thruster_frame_7,thruster7)
        self.ui.thruster_8.setText(str(thruster8))
        self.percentage_bar(self.ui.thruster_frame_8,thruster8)
        
    def sensor_callback(self,msg):
        #internal_temperatures=[msg.temp1,msg.temp2,msg.temp3,msg.temp4]
        internal_temperatures=[30,70,55,42]
        temp_labels=[self.ui.temp_1,self.ui.temp_2,self.ui.temp_3,self.ui.temp_4]
        
        for i in range(4):
            if internal_temperatures[i]<40:
                temp_labels[i].setStyleSheet("border: 2px solid green")
            elif internal_temperatures[i]<50:
                temp_labels[i].setStyleSheet("border: 2px solid yellow")
            elif internal_temperatures[i]<60:
                temp_labels[i].setStyleSheet("border: 2px solid orange")
            else:
                temp_labels[i].setStyleSheet("border: 2px solid red")
            temp_labels[i].setText(str(internal_temperatures[i]))
        self.ui.label_water_temp.setText(str(0))

    
    def percentage_bar(self, frame,value):
        color="green"
        value=round(value,3)
        percentage=1-value
        stop1=percentage-0.001
        stop2=percentage
        if value<0:
            percentage=percentage-1
            stop1=percentage
            stop2=percentage-0.001
            color="red"
        styleSheet = "border:none;background-color: qconicalgradient(cx:0.5, cy:0.5, angle:90, stop:{STOP_1} rgba(77, 77, 127, 100), stop:{STOP_2} {COLOR});"
        styleSheet = styleSheet.replace("{STOP_1}", str(stop1)).replace("{STOP_2}", str(stop2)).replace("{COLOR}", color)
        frame.setStyleSheet(styleSheet)
    
    def callback(self,msg):
        self.robot_arm_view.setRotation(msg.angular.y)
        self.ui.label_roll.setText(str(msg.angular.x))
        self.ui.label_pitch.setText(str(msg.angular.y))
        self.ui.label_yaw.setText(str(msg.angular.z))
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
   
       

    def init_rov_node(self):
        self.toggle_animation(self.ui.button_control_rov_node,self.ui.togglebutton_background_4,self.ui.togglebutton_circle_4)
        if self.ui.button_control_rov_node.isChecked():
            self.ssh_thread = SSHThread()
            self.ssh_thread.output_signal.connect(self.handle_output)
            self.ssh_thread.error_signal.connect(self.handle_error)
            self.ssh_thread.start()
        else:
            msg=Int8()
            msg.data=1
            self.pub_stop_rov_node.publish(msg)
    def handle_output(self, output):
        print(output)  # or update GUI

    def handle_error(self, error):
        QMessageBox.critical(self, "SSH Command Error", error)
   
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

