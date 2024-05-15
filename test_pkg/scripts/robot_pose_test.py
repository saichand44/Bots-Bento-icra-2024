#!/usr/bin/env python3
import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
import threading
import time

class RobotPoseNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Create a QoS profile with BEST_EFFORT reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = DurabilityPolicy.VOLATILE

        # Subscribe to the AprilTag info
        self.sub_tag_info = self.create_subscription(AprilTagDetectionArray, "/olive/camera/id21/tags", self.pose_callback, qos_profile)

        self.intrinsic_mtx = np.array([[353.88557, 0.0, 160.14916],
                                       [0.0, 353.7126, 117.27734],
                                       [0.0, 0.0, 1.0]])
        self.distortion_mtx = np.array([-0.428635, 0.167437, 0.0001243, 0.0004107, 0.0])

    def pose_callback(self, msg):
            detections = msg.detections
            for detection in detections:
                corners = np.array([[detection.corners[0].x, detection.corners[0].y],
                                    [detection.corners[1].x, detection.corners[1].y],
                                    [detection.corners[2].x, detection.corners[2].y],
                                    [detection.corners[3].x, detection.corners[3].y]])

                corners = corners.reshape((1, 4, 2))  # Reshape for aruco function

                rvec, tvec, marker_pts = cv2.aruco.estimatePoseSingleMarkers(corners, 0.04,
                                                                             self.intrinsic_mtx,
                                                                             self.distortion_mtx)
                # print(f"Rotation Vector: {rvec}")
                # print(f"Translation Vector: {tvec}")

                

if __name__ == '__main__':
    rclpy.init()
    robot_pose_node = RobotPoseNode("robot_pose_node")
    rclpy.spin(robot_pose_node)
    robot_pose_node.destroy_node()
    rclpy.shutdown()