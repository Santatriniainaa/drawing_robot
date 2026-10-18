#!/usr/bin/env python3
"""ROS 2 node publishing visualization markers for the PCB layout."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

DEFAULT_MARKER_Z = 0.02102


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.pub = self.create_publisher(MarkerArray, '/draw_markers', 10)
        self.markers = []
        self.marker_id = 0
        self.get_logger().info('Marker Publisher prêt')

    def add_line(self, x1, y1, x2, y2, z=DEFAULT_MARKER_Z):
        """Ajoute une ligne (trace PCB)."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pcb_traces'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.points = [Point(x=x1, y=y1, z=z), Point(x=x2, y=y2, z=z)]
        marker.scale.x = 0.001
        marker.color = ColorRGBA(r=0.95, g=0.55, b=0.12, a=1.0)

        self.markers.append(marker)
        self.publish_markers()

    def add_circle(self, cx, cy, radius, z=DEFAULT_MARKER_Z, segments=32):
        """Ajoute un cercle (pad)."""
        previous_x = cx + radius
        previous_y = cy

        for index in range(1, segments + 1):
            angle = 2.0 * math.pi * index / segments
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            self.add_line(previous_x, previous_y, x, y, z)
            previous_x, previous_y = x, y

    def publish_markers(self):
        array = MarkerArray()
        array.markers = self.markers
        self.pub.publish(array)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()

    try:
        node.add_line(-0.05, -0.05, 0.05, -0.05)
        node.add_line(0.05, -0.05, 0.05, 0.05)
        node.add_line(0.05, 0.05, -0.05, 0.05)
        node.add_line(-0.05, 0.05, -0.05, -0.05)
        node.add_circle(0.0, 0.0, 0.01)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
