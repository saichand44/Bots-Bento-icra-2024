import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
import threading
import time

class MotorTestNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # publish on the cmd_vel topic
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", QoSProfile(depth=10))

        # define the parameters
        self.declare_parameter("vx", 0.1)
        self.declare_parameter("vy", 0.1)
        self.declare_parameter("wz", 0.1)

        self.vx = self.get_parameter('vx').get_parameter_value().double_value
        self.vy = self.get_parameter('vy').get_parameter_value().double_value
        self.wz = self.get_parameter('wz').get_parameter_value().double_value

        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 25

        self.thread_main.start()

    def publish_cmd_vel(self, publisher, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.x = vy
        msg.angular.z = wz
        publisher.publish(msg)

    def thread_main(self):
        time.sleep(1)

        while not self.thread_exited:
            # publish the x, y , theta for the robot
            self.publish_cmd_vel(self.pub_cmd_vel, self.vx, self.vy, self.wz)

            time.sleep(1 / self.rate_control_hz)

    def __del__(self):
        self.thread_exited = True
        if self.thread_main.is_alive():
            self.thread_main.join()

if __name__ == '__main__':
    rclpy.init()
    motor_test_node = MotorTestNode("motor_test_node")
    print("Bots_Bento_Base_Test_Control_v1.0")
    rclpy.spin(motor_test_node)
    motor_test_node.destroy_node()
    rclpy.shutdown()

