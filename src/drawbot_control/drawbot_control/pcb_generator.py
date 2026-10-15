#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class PCBDrawer(Node):
    def __init__(self):
        super().__init__('pcb_drawer')
        self.pub = self.create_publisher(String, '/draw_commands', 10)
        self.get_logger().info('PCB Drawer prêt - envoie des commandes JSON sur /draw_commands')

    def send_command(self, command_dict):
        msg = String()
        msg.data = json.dumps(command_dict)
        self.pub.publish(msg)

    def draw_pcb(self, traces, pads):
        """
        traces: liste de listes [[x1,y1], [x2,y2], ...] en mètres
        pads: liste de [x, y, radius] en mètres
        """
        cmd = {
            "type": "pcb",
            "traces": traces,
            "pads": pads,
            "pen_down": -0.0075,
            "pen_up": 0.0
        }
        self.send_command(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PCBDrawer()

    # Exemple de carte simple
    traces = [
        [[-0.04, -0.04], [0.04, -0.04], [0.04, 0.04], [-0.04, 0.04], [-0.04, -0.04]]  # Contour
    ]
    pads = [
        [-0.02, -0.02, 0.002],  # x, y, radius
        [0.02, -0.02, 0.002],
        [0.0, 0.0, 0.003]
    ]

    node.draw_pcb(traces, pads)
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    