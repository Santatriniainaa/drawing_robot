#!/usr/bin/env python3
"""ROS 2 node that executes a predefined PCB drawing pattern."""

from __future__ import annotations

import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from drawbot_control.robot_geometry import PEN_DOWN_POSITION
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PEN_UP = 0.0
PEN_DOWN = PEN_DOWN_POSITION
PEN_SETTLE_TIME = 0.3
XY_CONTROLLER_TOPIC = '/xy_controller/follow_joint_trajectory'
PEN_COMMAND_TOPIC = '/pen_controller/commands'


class DrawNode(Node):
    def __init__(self):
        super().__init__('draw_node')
        self.cb_group = ReentrantCallbackGroup()
        self.xy_client = ActionClient(
            self,
            FollowJointTrajectory,
            XY_CONTROLLER_TOPIC,
            callback_group=self.cb_group,
        )
        self.pen_pub = self.create_publisher(
            Float64MultiArray,
            PEN_COMMAND_TOPIC,
            10,
        )

        self.get_logger().info('Attente du serveur de trajectoire XY...')
        self.xy_client.wait_for_server()
        self.get_logger().info('Connecté au contrôleur XY')

        self.move_pen(PEN_UP)
        time.sleep(0.5)
        self.get_logger().info('Draw Node prêt !')

    @staticmethod
    def _to_duration(seconds: float) -> Duration:
        whole_seconds = int(seconds)
        return Duration(
            sec=whole_seconds,
            nanosec=int((seconds - whole_seconds) * 1_000_000_000),
        )

    def move_pen(self, position: float) -> None:
        """Déplace le stylo à une position donnée."""
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self.pen_pub.publish(msg)
        self.get_logger().info('Stylo -> %.3fm', position)
        time.sleep(PEN_SETTLE_TIME)

    def send_trajectory(self, points):
        """Envoie une trajectoire XY."""
        if not points:
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_x', 'joint_y']

        total_time = 0.0
        start_point = points[0]
        first_point = JointTrajectoryPoint()
        first_point.positions = [float(start_point[0]), float(start_point[1])]
        first_point.velocities = [0.0, 0.0]
        total_time += 1.0
        first_point.time_from_start = self._to_duration(total_time)
        trajectory.points.append(first_point)

        for x, y, pen_down in points[1:]:
            point = JointTrajectoryPoint()
            point.positions = [float(x), float(y)]
            point.velocities = [0.0, 0.0]
            total_time += 0.5 if pen_down else 0.3
            point.time_from_start = self._to_duration(total_time)
            trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info('Envoi trajectoire: %s points', len(points))
        future = self.xy_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, callback_group=self.cb_group)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Le contrôleur XY a refusé la trajectoire.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            callback_group=self.cb_group,
        )
        result = result_future.result()
        if result is None:
            self.get_logger().error('Aucun résultat reçu du contrôleur XY.')
            return False

        successful = (
            result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        if not successful:
            self.get_logger().error(
                'La trajectoire XY a échoué (code %s).',
                result.result.error_code,
            )
        return successful

    def draw_line(self, x1, y1, x2, y2):
        """Dessine une ligne entre deux points."""
        return [
            (x1, y1, False),
            (x1, y1, True),
            (x2, y2, True),
            (x2, y2, False),
        ]

    def draw_rectangle(self, cx, cy, width, height):
        """Dessine un rectangle centré en (cx, cy)."""
        half_width = width / 2.0
        half_height = height / 2.0
        return [
            (cx - half_width, cy - half_height, False),
            (cx - half_width, cy - half_height, True),
            (cx + half_width, cy - half_height, True),
            (cx + half_width, cy + half_height, True),
            (cx - half_width, cy + half_height, True),
            (cx - half_width, cy - half_height, True),
            (cx - half_width, cy - half_height, False),
        ]

    def draw_circle(self, cx, cy, radius, segments=36):
        """Dessine un cercle pour un pad PCB."""
        points = []
        angle = 0.0
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        points.append((x, y, False))

        for index in range(1, segments + 1):
            angle = 2.0 * math.pi * index / segments
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            points.append((x, y, True))

        points.append((points[-1][0], points[-1][1], False))
        return points

    def draw_pcb_test(self):
        """Dessine un motif PCB de test : contour + pads."""
        all_points = []

        all_points.extend(self.draw_rectangle(0.0, 0.0, 0.10, 0.10))
        time.sleep(0.5)

        pad_radius = 0.0015
        positions = [
            (-0.03, -0.03),
            (0.03, -0.03),
            (-0.03, 0.03),
            (0.03, 0.03),
            (0.0, 0.0),
        ]

        for px, py in positions:
            all_points.extend(self.draw_circle(px, py, pad_radius, segments=24))
            time.sleep(0.3)

        traces = [
            [(-0.03, -0.03), (0.0, 0.0), (0.03, -0.03)],
            [(-0.03, 0.03), (0.0, 0.0), (0.03, 0.03)],
        ]

        for trace in traces:
            for index in range(len(trace) - 1):
                x1, y1 = trace[index]
                x2, y2 = trace[index + 1]
                all_points.extend(self.draw_line(x1, y1, x2, y2))

        return all_points

    def execute_drawing(self, points):
        """Exécute le dessin avec gestion du stylo."""
        if not points:
            return False

        first_x, first_y, _ = points[0]
        if not self.send_trajectory([(first_x, first_y, False)]):
            self.move_pen(PEN_UP)
            return False

        current_batch = []
        current_pen_state = False

        for x, y, pen_down in points:
            if pen_down != current_pen_state:
                if current_batch and not self.send_trajectory(current_batch):
                    self.move_pen(PEN_UP)
                    return False
                current_batch = []
                self.move_pen(PEN_DOWN if pen_down else PEN_UP)
                current_pen_state = pen_down
                time.sleep(0.2)

            current_batch.append((x, y, pen_down))

        if current_batch and not self.send_trajectory(current_batch):
            self.move_pen(PEN_UP)
            return False

        self.move_pen(PEN_UP)
        self.get_logger().info('Dessin terminé !')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = DrawNode()

    try:
        node.get_logger().info('Démarrage du dessin PCB...')
        points = node.draw_pcb_test()
        node.execute_drawing(points)
    except KeyboardInterrupt:
        pass
    finally:
        node.move_pen(PEN_UP)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
