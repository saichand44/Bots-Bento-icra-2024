import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml

class TagMarkerPublisher(Node):
    def __init__(self):
        super().__init__('tag_marker_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'tag_markers', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.load_tags()

    def load_tags(self):
        with open('path/to/your_package_name/config/apriltag_map.yaml', 'r') as file:
            self.tags = yaml.safe_load(file)['tags']

    def timer_callback(self):
        marker_array = MarkerArray()
        for tag in self.tags:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = tag['id']
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = tag['position'][0]
            marker.pose.position.y = tag['position'][1]
            marker.pose.position.z = tag['position'][2]
            marker.pose.orientation.x = tag['orientation'][0]
            marker.pose.orientation.y = tag['orientation'][1]
            marker.pose.orientation.z = tag['orientation'][2]
            marker.pose.orientation.w = tag['orientation'][3]
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = TagMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
