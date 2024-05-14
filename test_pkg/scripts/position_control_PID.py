import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import threading
import time
import math

class PositionControlNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Publishers and Subscribers
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", QoSProfile(depth=10))
        self.sub_imu = self.create_subscription(Imu, "imu/data", self.imu_callback, QoSProfile(depth=10))

        # Position and Orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0

        # Target Position and Orientation
        self.target_x = 1.0  # target x position
        self.target_y = 1.0  # target y position
        self.target_theta = 0.0  # target orientation (yaw)

        # Control Gains
        self.kp_pos = 0.5
        self.ki_pos = 0.1
        self.kd_pos = 0.1
        self.kp_theta = 1.0
        self.ki_theta = 0.1
        self.kd_theta = 0.1

        # PID Controller States
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_theta = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_theta = 0.0

        self.thread_main = threading.Thread(target=self.thread_main)
        self.thread_exited = False
        self.rate_control_hz = 25

        self.thread_main.start()

    def imu_callback(self, msg):
        # Extract orientation and linear acceleration from IMU message
        orientation_q = msg.orientation
        linear_acceleration = msg.linear_acceleration

        # Convert quaternion to Euler angles
        _, _, self.theta = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        # Transform linear acceleration from IMU frame to world frame
        ax_world, ay_world = self.transform_acceleration(linear_acceleration.x, linear_acceleration.y, self.theta)

        # Update position and velocity based on linear acceleration
        dt = 1.0 / self.rate_control_hz
        self.vx += ax_world * dt
        self.vy += ay_world * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def transform_acceleration(self, ax, ay, theta):
        # Rotate acceleration vector to world frame
        ax_world = ax * math.cos(theta) - ay * math.sin(theta)
        ay_world = ax * math.sin(theta) + ay * math.cos(theta)
        return ax_world, ay_world

    def publish_cmd_vel(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.pub_cmd_vel.publish(msg)

    def thread_main(self):
        time.sleep(1)

        while not self.thread_exited:
            # Calculate position errors
            error_x = self.target_x - self.x
            error_y = self.target_y - self.y
            error_theta = self.target_theta - self.theta

            # Update integral terms
            self.integral_x += error_x
            self.integral_y += error_y
            self.integral_theta += error_theta

            # Calculate derivative terms
            derivative_x = error_x - self.prev_error_x
            derivative_y = error_y - self.prev_error_y
            derivative_theta = error_theta - self.prev_error_theta

            # Calculate control commands using PID
            vx = self.kp_pos * error_x + self.ki_pos * self.integral_x + self.kd_pos * derivative_x
            vy = self.kp_pos * error_y + self.ki_pos * self.integral_y + self.kd_pos * derivative_y
            wz = self.kp_theta * error_theta + self.ki_theta * self.integral_theta + self.kd_theta * derivative_theta

            # Update previous errors
            self.prev_error_x = error_x
            self.prev_error_y = error_y
            self.prev_error_theta = error_theta

            # Publish velocity commands
            self.publish_cmd_vel(vx, vy, wz)

            time.sleep(1 / self.rate_control_hz)

    def __del__(self):
        self.thread_exited = True
        if self.thread_main.is_alive():
            self.thread_main.join()

if __name__ == '__main__':
    rclpy.init()
    position_control_node = PositionControlNode("position_control_node")
    print("Position Control Node Started")
    rclpy.spin(position_control_node)
    position_control_node.destroy_node()
    rclpy.shutdown()
