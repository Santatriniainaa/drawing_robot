#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.pub = self.create_publisher(MarkerArray, '/draw_markers', 10)
        self.markers = []
        self.marker_id = 0
        self.get_logger().info('Marker Publisher prêt')

    def add_line(self, x1, y1, x2, y2, z=0.0015):
        """Ajoute une ligne verte (piste PCB)"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pcb_traces"
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        p1 = Point(x=x1, y=y1, z=z)
        p2 = Point(x=x2, y=y2, z=z)
        marker.points = [p1, p2]

        marker.scale.x = 0.001  # Épaisseur 1mm
        marker.color = ColorRGBA(r=0.0, g=0.8, b=0.2, a=1.0)  # Vert cuivre

        self.markers.append(marker)
        self.publish_markers()

    def add_circle(self, cx, cy, radius, z=0.0015, segments=32):
        """Ajoute un cercle (pad)"""
        prev_x = cx + radius
        prev_y = cy
        for i in range(1, segments + 1):
            angle = 2 * math.pi * i / segments
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            self.add_line(prev_x, prev_y, x, y, z)
            prev_x, prev_y = x, y

    def publish_markers(self):
        array = MarkerArray()
        array.markers = self.markers
        self.pub.publish(array)

    def clear(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.ns = "pcb_traces"
        marker.id = 0
        marker.action = Marker.DELETEALL
        array = MarkerArray()
        array.markers = [marker]
        self.pub.publish(array)
        self.markers = []
        self.marker_id = 0

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()

    # Dessiner un exemple statique dans RViz
    # Rectangle
    node.add_line(-0.05, -0.05, 0.05, -0.05)
    node.add_line(0.05, -0.05, 0.05, 0.05)
    node.add_line(0.05, 0.05, -0.05, 0.05)
    node.add_line(-0.05, 0.05, -0.05, -0.05)

    # Cercle au centre
    node.add_circle(0.0, 0.0, 0.01)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    