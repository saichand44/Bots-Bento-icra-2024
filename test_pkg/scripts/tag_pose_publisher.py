import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray

class TagPosePublisher(Node):
    def __init__(self):
        super().__init__('tag_pose_publisher')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/tag_detections_pose', 10)

    def listener_callback(self, msg):
        if len(msg.detections) > 0:
            detection = msg.detections[0]  # Use the first detected tag
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = detection.pose.header
            pose_msg.pose = detection.pose.pose
            self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TagPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
