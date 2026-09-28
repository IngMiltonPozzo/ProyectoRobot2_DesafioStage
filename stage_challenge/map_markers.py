"""
Nodo map_markers - Desafio Stage ROS2.

Publica como visualization_msgs/MarkerArray la geometria estatica del
mapa (paredes, obstaculo, baliza) definida en worlds/desafio_stage.world,
en el frame "world" (coincide 1:1 con las coordenadas de ese archivo,
y con lo que se ve en la ventana de Stage), para poder verla en RViz
junto con el laser, el robot y el objetivo.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

WALL_HEIGHT = 0.6

# (nombre, centro_x, centro_y, tamano_x, tamano_y, color (r,g,b))
WALLS = [
    ('perimetro_norte', 0.0, 6.0, 12.2, 0.2, (0.5, 0.5, 0.5)),
    ('perimetro_sur', 0.0, -6.0, 12.2, 0.2, (0.5, 0.5, 0.5)),
    ('perimetro_este', 6.0, 0.0, 0.2, 12.2, (0.5, 0.5, 0.5)),
    ('perimetro_oeste', -6.0, 0.0, 0.2, 12.2, (0.5, 0.5, 0.5)),
    ('divisoria', -3.0, 0.0, 6.0, 0.2, (0.5, 0.5, 0.5)),
    ('obstaculo', 3.0, -2.0, 1.0, 1.0, (0.27, 0.51, 0.71)),
    ('baliza', 0.0, 1.0, 0.3, 0.3, (1.0, 1.0, 0.0)),
]


class MapMarkers(Node):

    def __init__(self):
        super().__init__('map_markers')
        self.publisher_ = self.create_publisher(MarkerArray, '/map_markers', 10)
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(
            'Map markers iniciado. En RViz: Add -> By topic -> /map_markers -> MarkerArray.')

    def publish_markers(self):
        array = MarkerArray()
        for i, (name, wx, wy, sx, sy, color) in enumerate(WALLS):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'desafio_stage_map'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = WALL_HEIGHT / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = sx
            m.scale.y = sy
            m.scale.z = WALL_HEIGHT
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 0.85
            array.markers.append(m)
        self.publisher_.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = MapMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
