import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
import threading
import time

class RobotPoseNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # subscribe to the april tag info
        self.sub_tag_info = self.create_subscription(AprilTagDetectionArray, "/olive/camera/id20/tags", self.pose_callback, QoSProfile(depth=10))

        self.intrinsic_mtx = np.array([[353.88557, 0.0, 160.14916],
                                       [0.0, 353.7126, 117.27734],
                                       [0.0, 0.0, 1.0]])
        self.distortion_mtx = np.array([-0.428635, 0.167437, 0.0001243, 0.0004107, 0.0])

    def pose_callback(self, msg):
        detections = msg.detections
        corners = detections.corners
        
        rvec, tvec, marker_pts = cv2.aruco.estimatePoseSingleMarkers(corners, 0.04,
                                                                     self.intrinsic_mtx,
                                                                     self.distortion_mtx)
        print(rvec)
        print(tvec)
        
    def __del__(self):
        self.thread_exited = True
        if self.thread_main.is_alive():
            self.thread_main.join()

if __name__ == '__main__':
    rclpy.init()
    robot_pose_node = RobotPoseNode("robot_pose_node")
    rclpy.spin(robot_pose_node)
    robot_pose_node.destroy_node()
    rclpy.shutdown()

        
