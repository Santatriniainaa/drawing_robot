#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
import math
import time

class DrawNode(Node):
    def __init__(self):
        super().__init__('draw_node')
        self.cb_group = ReentrantCallbackGroup()

        # Positions du stylo (en mètres, relatif au joint)
        # La face basse du cylindre touche la face PCB à environ -0.0075 m.
        self.PEN_UP = 0.0
        self.PEN_DOWN = -0.0075

        # Client action pour trajectoire XY
        self.xy_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xy_controller/follow_joint_trajectory',
            callback_group=self.cb_group
        )

        # Publisher pour le contrôle du stylo Z
        self.pen_pub = self.create_publisher(
            Float64MultiArray,
            '/pen_controller/commands',
            10
        )

        # Attendre les serveurs
        self.get_logger().info('Attente du serveur de trajectoire XY...')
        self.xy_client.wait_for_server()
        self.get_logger().info('Connecté au contrôleur XY')

        # S'assurer que le stylo est levé au démarrage
        self.move_pen(self.PEN_UP)
        time.sleep(0.5)

        self.get_logger().info('Draw Node prêt !')

    def move_pen(self, position):
        """Déplace le stylo à une position donnée"""
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self.pen_pub.publish(msg)
        self.get_logger().info(f'Stylo -> {position:.3f}m')
        time.sleep(0.3)  # Temps de stabilisation

    def send_trajectory(self, points):
        """
        Envoie une trajectoire XY
        points: liste de tuples (x, y, pen_down: bool)
        """
        if not points:
            return False

        # Créer la trajectoire
        traj = JointTrajectory()
        traj.joint_names = ['joint_x', 'joint_y']

        # Premier point (position de départ)
        t = 0.0
        p = JointTrajectoryPoint()
        p.positions = [points[0][0], points[0][1]]
        p.velocities = [0.0, 0.0]
        t += 1.0  # 1 seconde pour aller au start
        p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        traj.points.append(p)

        # Points suivants
        for i in range(1, len(points)):
            x, y, pen_down = points[i]
            p = JointTrajectoryPoint()
            p.positions = [x, y]
            p.velocities = [0.0, 0.0]

            # Vitesse plus lente si on dessine
            duration = 0.5 if pen_down else 0.3
            t += duration
            p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            traj.points.append(p)

        # Envoyer la goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(f'Envoi trajectoire: {len(points)} points')
        future = self.xy_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, callback_group=self.cb_group)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Le contrôleur XY a refusé la trajectoire.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, callback_group=self.cb_group
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
                f'La trajectoire XY a échoué (code {result.result.error_code}).'
            )
        return successful

    def draw_line(self, x1, y1, x2, y2):
        """Dessine une ligne entre deux points"""
        points = [
            (x1, y1, False),  # Déplacement vers start (stylo levé)
            (x1, y1, True),   # Stylo down au start
            (x2, y2, True),   # Ligne jusqu'à la fin
            (x2, y2, False)   # Stylo up
        ]
        return points

    def draw_rectangle(self, cx, cy, width, height):
        """Dessine un rectangle centré en (cx, cy)"""
        w2, h2 = width/2, height/2
        points = [
            (cx-w2, cy-h2, False),  # Start coin bas-gauche
            (cx-w2, cy-h2, True),
            (cx+w2, cy-h2, True),   # Bas
            (cx+w2, cy+h2, True),   # Droite
            (cx-w2, cy+h2, True),   # Haut
            (cx-w2, cy-h2, True),   # Gauche (fermeture)
            (cx-w2, cy-h2, False)
        ]
        return points

    def draw_circle(self, cx, cy, radius, segments=36):
        """Dessine un cercle (pour pads PCB)"""
        points = []
        # Premier point (déplacement)
        angle = 0
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        points.append((x, y, False))

        # Cercle
        for i in range(1, segments + 1):
            angle = 2 * math.pi * i / segments
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            points.append((x, y, True))

        points.append((points[-1][0], points[-1][1], False))  # Stylo up
        return points

    def draw_pcb_test(self):
        """Dessine un motif PCB de test : contour + pads"""
        all_points = []

        # Contour de la carte (100x100mm = 0.1m)
        rect = self.draw_rectangle(0.0, 0.0, 0.10, 0.10)
        all_points.extend(rect)
        time.sleep(0.5)

        # Pads (cercles de 3mm de diamètre)
        pad_radius = 0.0015
        positions = [
            (-0.03, -0.03), (0.03, -0.03),
            (-0.03, 0.03), (0.03, 0.03),
            (0.0, 0.0)
        ]

        for px, py in positions:
            circle = self.draw_circle(px, py, pad_radius, segments=24)
            all_points.extend(circle)
            time.sleep(0.3)

        # Pistes (lignes)
        traces = [
            [(-0.03, -0.03), (0.0, 0.0), (0.03, -0.03)],
            [(-0.03, 0.03), (0.0, 0.0), (0.03, 0.03)],
        ]

        for trace in traces:
            for i in range(len(trace)-1):
                x1, y1 = trace[i]
                x2, y2 = trace[i+1]
                line = self.draw_line(x1, y1, x2, y2)
                all_points.extend(line)

        return all_points

    def execute_drawing(self, points):
        """Exécute le dessin avec gestion du stylo"""
        if not points:
            return False

        # Phase 1: Aller au premier point avec stylo levé
        first_x, first_y, _ = points[0]
        if not self.send_trajectory([(first_x, first_y, False)]):
            self.move_pen(self.PEN_UP)
            return False

        # Phase 2: Traiter les segments
        current_batch = []
        current_pen_state = False

        for point in points:
            x, y, pen_down = point

            # Changer d'état du stylo si nécessaire
            if pen_down != current_pen_state:
                # Envoyer le batch précédent
                if current_batch:
                    if not self.send_trajectory(current_batch):
                        self.move_pen(self.PEN_UP)
                        return False
                    current_batch = []

                # Changer le stylo
                self.move_pen(self.PEN_DOWN if pen_down else self.PEN_UP)
                current_pen_state = pen_down
                time.sleep(0.2)

            current_batch.append((x, y, pen_down))

        # Envoyer le dernier batch
        if current_batch:
            if not self.send_trajectory(current_batch):
                self.move_pen(self.PEN_UP)
                return False

        # Lever le stylo à la fin
        self.move_pen(self.PEN_UP)
        self.get_logger().info('Dessin terminé !')
        return True

def main(args=None):
    rclpy.init(args=args)
    node = DrawNode()

    try:
        # Test : dessiner un motif PCB
        node.get_logger().info('Démarrage du dessin PCB...')
        points = node.draw_pcb_test()

        node.execute_drawing(points)

    except KeyboardInterrupt:
        pass
    finally:
        node.move_pen(node.PEN_UP)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    