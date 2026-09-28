"""
Nodo checkpoint_detector - Desafio Stage ROS2.

Escucha /fiducials y reporta cuando el robot pasa CERCA de verdad de la
baliza (marker id = CHECKPOINT_ID), no solo cuando el sensor la detecta
a la distancia maxima. El chequeo se reinicia cada vez que llega un
objetivo nuevo por /goal_pose, para que sea verificable en cada corrida.
"""

import math

import rclpy
from rclpy.node import Node
from marker_msgs.msg import MarkerDetection
from geometry_msgs.msg import PoseStamped

CHECKPOINT_ID = 2
PROXIMITY_METERS = 0.5  # distancia real al marcador para contarlo como "pasado"


class CheckpointDetector(Node):

    def __init__(self):
        super().__init__('checkpoint_detector')
        self.checkpoint_reached = False

        self.create_subscription(
            MarkerDetection, '/fiducials', self.detection_callback, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.get_logger().info('Checkpoint detector iniciado.')

    def goal_callback(self, msg: PoseStamped):
        if self.checkpoint_reached:
            self.get_logger().info('Nuevo objetivo: reinicio la busqueda del checkpoint.')
        self.checkpoint_reached = False

    def detection_callback(self, msg: MarkerDetection):
        if self.checkpoint_reached:
            return

        for marker in msg.markers:
            if CHECKPOINT_ID not in marker.ids:
                continue
            distance = math.hypot(
                marker.pose.position.x, marker.pose.position.y)
            if distance <= PROXIMITY_METERS:
                self.checkpoint_reached = True
                self.get_logger().info(
                    f'Checkpoint (id={CHECKPOINT_ID}) alcanzado a {distance:.2f} m.')
                break


def main(args=None):
    rclpy.init(args=args)
    node = CheckpointDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
