#!/usr/bin/env python3
import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Int32
from std_msgs.msg import String
import threading
import time

class PlannerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Create a QoS profile with BEST_EFFORT reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = DurabilityPolicy.VOLATILE

        # subscriber to nodes
        self.sub_adjust_bot = self.create_subscription(Int32, "adjust_bot", self.adjust_bot_callback, qos_profile)
        self.sub_tag_info = self.create_subscription(AprilTagDetectionArray, "/olive/camera/id20/tags", self.align2klt_callback, qos_profile)
        
        # publish to node
        self.pub_state = self.create_publisher(String, "state", qos_profile)

        # default state of the bot
        self.pub_state = 'sweep_restart'

    def adjust_bot_callback(self, msg):
        if (msg == 1):
            self.pub_state = 'adjust_bot'
        elif(msg == 0):
            self.pub_state = 'sweep_resume'
    
    def align2klt_callback(self, msg):
        detections = msg.detections

        if len(detections) > 2:
            self.pub_state = 'align_bot'
        else:
            self.pub_state = 'sweep_resume'


