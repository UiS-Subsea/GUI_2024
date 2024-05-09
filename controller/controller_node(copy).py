import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int16
from cpp_package.msg import Manipulator, ModeControl
import pygame

import threading

pygame.init()

#ROV
surge = 0
sway=0
heave=0
pitch=0
roll=0
yaw=0

#Manipulator
height=0.0
reach=0.0
pinch=0.0
rotate=0.0
angle=0.0
manipulator=False
wait_for_rov_controller=True
man_controller_connected=False
connected_controllers=0

class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller_node")
        self.sub=self.create_subscription(ModeControl,"mode_control",self.listener_callback,10)
        self.ROV_controller = self.create_publisher(Twist, "ROV_movement", 10)
        self.Manipulator_controller = self.create_publisher(Manipulator, "manipulator", 10)
        self.connected_controllers= self.create_publisher(Int16,"connected_controllers",10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Node is alive")
        self.mode="Manual"
        
    def listener_callback(self, msg):
        global manipulator, wait_for_rov_controller
        self.mode=msg.drive_mode
        
        wait_for_rov_controller=True
        manipulator=msg.manipulator
    
    def timer_callback(self):
        global man_controller_connected, connected_controllers
        if self.mode=="Manual":
            rov_msg = Twist()
            if wait_for_rov_controller:
                rov_msg.linear.x = float(0)
                rov_msg.linear.y = float(0)
                rov_msg.linear.z=float(0)
                rov_msg.angular.z=float(yaw)
                rov_msg.angular.x=float(roll)
                rov_msg.angular.y=float(pitch)
                self.get_logger().info("Waiting for controller")
            else:
                
                rov_msg.linear.x = float(surge)
                rov_msg.linear.y = float(sway)
                rov_msg.linear.z=float(heave)
                rov_msg.angular.z=float(yaw)
                rov_msg.angular.x=float(roll)
                rov_msg.angular.y=float(pitch)
                
            self.ROV_controller.publish(rov_msg)
        elif self.mode=="Pipeline":
            self.get_logger().info("Pipeline inspection is active")
        elif self.mode=="Docking":
            self.get_logger().info("Autonomous docking is active")
        
        man_msg=Manipulator()
        if not man_controller_connected:
            man_msg.field1=0.0
            man_msg.field2=0.0
            man_msg.field3=0.0
            man_msg.field4=0.0
            man_msg.field5=0.0
        else:
            
            man_msg.field1=height
            man_msg.field2=float(reach)
            man_msg.field3=pinch
            man_msg.field4=rotate
            man_msg.field5=angle
        self.Manipulator_controller.publish(man_msg)
        msg=Int16()
        msg.data=connected_controllers
        self.connected_controllers.publish(msg)

def joystick_reader():
    
    global wait_for_rov_controller, manipulator, yaw, roll, pitch, surge, sway, heave,man_controller_connected, connected_controllers
    sensitivity = 0.0002
    
    while True:
        
        if wait_for_rov_controller:
            pygame.joystick.quit()
            pygame.joystick.init()
            try:
                rov_stick = pygame.joystick.Joystick(0)
                wait_for_rov_controller = False
                
            except:
                connected_controllers=0
                continue
        if manipulator and not man_controller_connected:
            try:
                man_stick = pygame.joystick.Joystick(1)
                man_controller_connected=True
                
            except:
                man_controller_connected=False
        
        
        connected_controllers=pygame.joystick.get_count()

        if connected_controllers==0:
            wait_for_rov_controller = True
      
            continue
        
        if connected_controllers==1:
            rov_stick=pygame.joystick.Joystick(0)
            man_controller_connected=False
            

        LB=rov_stick.get_button(4)
        RB=rov_stick.get_button(5)
        left_y=rov_stick.get_axis(0)
        left_x=rov_stick.get_axis(1)
        
        
        if RB:
            yaw+=0.0001
        if LB:
            yaw=yaw-0.0001
        yaw %= 360
            
        if left_y<-0.2 or left_y>0.2:
        
            roll+=left_y*sensitivity
        roll %= 360

        if left_x<-0.2 or left_x>0.2:
            pitch+=left_x*sensitivity
        pitch %= 360
            
            
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            
            #kjøre frem
            LT1=rov_stick.get_axis(2)
            RT1=rov_stick.get_axis(5)
            surge=(RT1-LT1)/2
            
            #kjøre sidelengs
            right_y=rov_stick.get_axis(3)
            sway=right_y
        
            #opp / ned
            right_x=rov_stick.get_axis(4)
            heave=right_x

        
            #manipulator
            if manipulator and man_controller_connected:
                try:
                    global rotate
                    global reach
                    reach=0
                    global angle
                    global height
                    #rotere hånd
                    rotate=man_stick.get_axis(3)
                    #teleskop inn / ut
                    LT2=man_stick.get_axis(2)
                    RT2=man_stick.get_axis(5)
                    reach=(RT2-LT2)/2
                    #arm opp / ned
                    height=man_stick.get_axis(1)*-1
                    #vinkel hånd
                    angle=man_stick.get_axis(4)*-1
                except:
                    print("couldnt find controller")
            

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    thread_ros_node = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread_ros_node.start()

    joystick_reader()

    rclpy.shutdown()

if __name__ == "__main__":
    main()