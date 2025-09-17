#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty

class TargetGenerator(Node):
    def __init__(self):
        super().__init__('target_generator')

        # Parameters
        self.declare_parameter('min_x', -1.0)
        self.declare_parameter('max_x',  1.0)
        self.declare_parameter('min_y', -1.0)
        self.declare_parameter('max_y',  1.0)
        self.declare_parameter('end_effector_z', 0.5)

        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        self.ee_z  = self.get_parameter('end_effector_z').value

        # Publisher
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Services
        self.create_service(Empty, 'clear_targets', self.clear_targets)
        self.create_service(Empty, 'generate_targets', self.generate_targets)

        self.current_markers = []

        # Predefined colors (RGB)
        self.colors = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0)   # Blue
        ]

    def clear_targets(self, request, response):
        for marker in self.current_markers:
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
        self.current_markers = []
        return response

    def generate_targets(self, request, response):
        # Clear old targets
        self.clear_targets(request, response)

        new_markers = []
        for idx, color in enumerate(self.colors):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'targets'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = random.uniform(self.min_x, self.max_x)
            marker.pose.position.y = random.uniform(self.min_y, self.max_y)
            marker.pose.position.z = self.ee_z
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            self.marker_pub.publish(marker)
            new_markers.append(marker)

        self.current_markers = new_markers
        return response

def main():
    rclpy.init()
    node = TargetGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
