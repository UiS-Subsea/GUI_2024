import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ExampleNode(Node):

    def __init__(self):
        super().__init__("example_node")
        self.sub=self.create_subscription(String,"mode_control",self.listener_callback,10)
        self.pub = self.create_publisher(Twist, "ROV_movement", 10)
        self.get_logger().info("Node is alive")

    def listener_callback(self, msg):
        print(msg)


    def publish_message(self):
        msg= Twist()
        msg.linear.x=1.0
        msg.linear.y=0.0
        msg.linear.z=0.0
        msg.angular.x=10.0
        msg.angular.y=0.0
        msg.angular.z=0.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()