import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cpp_package.msg import Manipulator, ModeControl
import pygame

import threading

pygame.init()

#rov movement
surge = 0
sway=0
heave=0
pitch=0
roll=0
yaw=0

 
#manipulator controll
height=1.0
reach=1.0
pinch=1.0
rotate=1.0
angle=1.0
manipulator=False
wait_for_controllers=True
rov_stick=None
man_stick=None





class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller_node")
        self.sub=self.create_subscription(ModeControl,"mode_control",self.listener_callback,10)
        self.ROV_controller = self.create_publisher(Twist, "ROV_movement", 10)
        self.Manipulator_controller = self.create_publisher(Manipulator, "manipulator", 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Node is alive")
        self.mode="Manual"
        self.manipulator=False
    
    def listener_callback(self, msg):
        global manipulator, wait_for_controllers
        self.mode=msg.drive_mode
        self.manipulator=msg.manipulator
        
        manipulator=msg.manipulator
        wait_for_controllers=True

    
    def timer_callback(self):
        if self.mode=="Manual":
            rov_msg = Twist()
            rov_msg.linear.x = float(surge)
            rov_msg.linear.y = float(sway)
            rov_msg.linear.z=float(heave)
            rov_msg.angular.z=float(yaw)
            rov_msg.angular.x=float(roll)
            rov_msg.angular.y=float(pitch)
            self.ROV_controller.publish(rov_msg)
            self.get_logger().info("Manualdriving is active")
        elif self.mode=="Pipeline":
            self.get_logger().info("Pipeline inspection is active")
        elif self.mode=="Docking":
            self.get_logger().info("Autonomous docking is active")
        if self.manipulator:
            man_msg=Manipulator()
            man_msg.field1=height
            man_msg.field2=float(reach)
            man_msg.field3=pinch
            man_msg.field4=rotate
            man_msg.field5=angle
            self.Manipulator_controller.publish(man_msg)



def joystick_reader():
    
    global wait_for_controllers, manipulator, rov_stick, man_stick, yaw, roll, pitch, surge, sway, heave
    sensitivity = 0.0002

    while True:
        if wait_for_controllers:
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                rov_stick = pygame.joystick.Joystick(0)
                if manipulator:
                    if pygame.joystick.get_count()>1:
                        man_stick = pygame.joystick.Joystick(1)
                    else:
                        pygame.joystick.quit()
                        continue
                wait_for_controllers = False
            else:
                pygame.joystick.quit()
                continue
       
        #snu seg
        if pygame.joystick.get_count()==0:
            wait_for_controllers = True
            surge=0
            sway=0
            heave = 0
            continue
        if pygame.joystick.get_count()==1 and manipulator: 
            wait_for_controllers = True
            continue
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
            surge=1+(RT1-LT1)/2
            
            #kjøre sidelengs
            right_y=rov_stick.get_axis(3)
            sway=right_y+1 
        
            #opp / ned
            right_x=rov_stick.get_axis(4)
            heave=right_x+1

        
            #manipulator
            if manipulator:
                
                global rotate
                global reach
                reach=0
                global angle
                global pinch
                global height
                #rotere hånd
                rotate=man_stick.get_axis(3)+1
                #teleskop inn / ut
                d_pad=man_stick.get_hat(0)
                if d_pad==(0,-1):
                    reach=0
                if d_pad==(0,1):
                    reach=2
                #gripe
                LT2=man_stick.get_axis(2)
                RT2=man_stick.get_axis(5)
                pinch=1+(RT2-LT2)/2
                #arm opp / ned
                height=man_stick.get_axis(1)*-1+1
                #vinkel hånd
                angle=man_stick.get_axis(4)*-1+1
        
            



           

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    # Start the ROS node in a separate thread
    thread_ros_node = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread_ros_node.start()

    # Start the joystick reader in the main thread
    joystick_reader()

    rclpy.shutdown()

if __name__ == "__main__":
    main()