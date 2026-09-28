"""
Nodo navigator - Desafio Stage ROS2. Ver DEVLOG.md para arquitectura y decisiones.

Maquina de 4 estados secuencial: ADVANCE / TURN / HOLD / SEARCH.

Dos umbrales: clearance (0.6m, comodo) y min_body_clearance (0.3m,
piso fisico real, radio del pioneer2dx de su centro a su esquina).

Mecanismo 2 -- correccion lateral durante TURN/HOLD-pivote/SEARCH:
importante distincion entre GIRO y AVANCE. El giro tiene direccion
FISICAMENTE VERIFICADA (confirmado con la geometria real: rotar aleja
lo que esta del lado hacia el que se gira). El avance NO tiene esa
garantia -- "para adelante" no dice nada sobre si esa direccion aleja
o acerca al lado que viola el piso, porque depende de la geometria
del contacto en ese instante, que el sistema no puede verificar.
Por eso: mientras la correccion esta activa (algun lado ya cruzo el
piso fisico), el avance se apaga del todo -- solo se confia en el
giro, que si tiene direccion probada. El boost de avance (para
despegarse trasladando, que un pivote puro no logra) sigue funcionando
igual que antes, pero SOLO en la zona preventiva: ajustado, pero
todavia sin haber cruzado el piso. Bug real encontrado: antes el
avance se sumaba SIEMPRE que el costado estaba ajustado, sin importar
si ya habia una correccion activa empujando para el otro lado -- en
un caso real confirmado, el avance aportaba 0.18 m/s exactamente
mientras el sistema seguia sin poder alejarse de un costado que
llevaba mas de un segundo violando el piso.

El disparo de la rama corregida en HOLD (esquina real / choque
lateral) usa <= en vez de < para el chequeo de hold_body -- se
encontro, con datos reales del log, un agujero de un solo ciclo: justo
en el instante exacto en que un costado TOCABA el piso fisico (valor
igual a min_body_clearance, ni mas ni menos), la comparacion estricta
lo dejaba pasar un ciclo por la regulacion recta sin corregir, antes
de recien activarse el ciclo siguiente cuando ya habia cruzado.

Mecanismo 1 -- HOLD libera solo si near Y mid estan los dos, a la vez,
por encima del margen.

Mecanismo 3 -- ADVANCE mira el lado de escape antes de comprometerse a
TURN.

Mecanismo 4 -- empujon minimo en ADVANCE avanzando derecho (mid/far
tambien vigilados, no solo near).

Mecanismo 5 -- decision de giro reutilizada entre ADVANCE y el pivote
de esquina de HOLD (_decide_front_reaction), para que HOLD siempre
reconsidere con datos frescos en vez de seguir atado al lado con el
que arranco a seguir una pared.

Watchdog UNIFICADO (posicion Y orientacion juntas, siempre).
search_dir se recalcula con datos frescos en las tres entradas a
SEARCH.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def compose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    x = ax + math.cos(ath) * bx - math.sin(ath) * by
    y = ay + math.sin(ath) * bx + math.cos(ath) * by
    return (x, y, normalize_angle(ath + bth))


def invert(a):
    ax, ay, ath = a
    cx = -(math.cos(-ath) * ax - math.sin(-ath) * ay)
    cy = -(math.sin(-ath) * ax + math.cos(-ath) * ay)
    return (cx, cy, normalize_angle(-ath))


def sector_room(near, mid, far):
    return min(near, mid, far)


class Navigator(Node):

    def __init__(self):
        super().__init__(
            'navigator',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.declare_parameter('clearance', 0.6)
        self.declare_parameter('front_half_angle_deg', 25.0)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('seek_rotate_in_place_deg', 40.0)
        self.declare_parameter('release_margin', 1.4)
        self.declare_parameter('hold_min_seconds', 0.3)
        self.declare_parameter('turn_max_seconds', 8.0)
        self.declare_parameter('min_body_clearance', 0.3)
        self.declare_parameter('turn_min_angular_factor', 0.2)
        self.declare_parameter('turn_max_forward_factor', 0.6)
        self.declare_parameter('lateral_correction_gain', 1.5)
        self.declare_parameter('hold_target_factor', 0.75)
        self.declare_parameter('hold_gain', 1.5)
        self.declare_parameter('hold_deadband', 0.05)
        self.declare_parameter('hold_max_turn_factor', 0.5)
        self.declare_parameter('stall_check_seconds', 2.0)
        self.declare_parameter('stall_min_displacement', 0.04)
        self.declare_parameter('yaw_stall_min_change_deg', 5.0)
        self.declare_parameter('search_max_seconds', 15.0)
        self.declare_parameter('advance_nudge', 0.15)

        self.clearance = self.get_parameter('clearance').value
        self.hold_target_dist = self.clearance * self.get_parameter(
            'hold_target_factor').value
        self.front_half_angle = math.radians(
            self.get_parameter('front_half_angle_deg').value)
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.hold_max_turn = self.angular_speed * self.get_parameter(
            'hold_max_turn_factor').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.seek_rotate_in_place = math.radians(
            self.get_parameter('seek_rotate_in_place_deg').value)
        self.release_margin = self.get_parameter('release_margin').value
        self.hold_min_seconds = self.get_parameter('hold_min_seconds').value
        self.turn_max_seconds = self.get_parameter('turn_max_seconds').value
        self.min_body_clearance = self.get_parameter('min_body_clearance').value
        self.turn_min_angular_factor = self.get_parameter(
            'turn_min_angular_factor').value
        self.turn_max_forward_factor = self.get_parameter(
            'turn_max_forward_factor').value
        self.lateral_correction_gain = self.get_parameter(
            'lateral_correction_gain').value
        self.hold_gain = self.get_parameter('hold_gain').value
        self.hold_deadband = self.get_parameter('hold_deadband').value
        self.stall_check_seconds = self.get_parameter('stall_check_seconds').value
        self.stall_min_displacement = self.get_parameter(
            'stall_min_displacement').value
        self.yaw_stall_min_change = math.radians(
            self.get_parameter('yaw_stall_min_change_deg').value)
        self.search_max_seconds = self.get_parameter('search_max_seconds').value
        self.advance_nudge = self.get_parameter('advance_nudge').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_pose = None
        self.goal_x = None
        self.goal_y = None
        self.mission_done = False
        self.latest_scan = None

        self.state = 'ADVANCE'
        self.hold_side = None
        self.turn_since = None
        self.hold_since = None
        self.search_since = None
        self.search_dir = 1
        self.motion_anchor = None
        self.gave_up = False

        self._last_trace = None

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(LaserScan, '/base_scan', self.scan_callback, 10)
        self.create_subscription(
            Odometry, '/ground_truth', self.ground_truth_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.2, self.broadcast_tf_correction)

        self.get_logger().info(
            'Navigator iniciado. Esperando objetivo (2D Goal Pose en RViz)...')

    def ground_truth_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def odom_callback(self, msg: Odometry):
        self.odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation))

    def broadcast_tf_correction(self):
        if self.odom_pose is None:
            return
        gt_pose = (self.current_x, self.current_y, self.current_yaw)
        world_to_odom = compose(gt_pose, invert(self.odom_pose))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        t.transform.translation.x = world_to_odom[0]
        t.transform.translation.y = world_to_odom[1]
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(world_to_odom[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.mission_done = False
        self.state = 'ADVANCE'
        self.hold_side = None
        self.turn_since = None
        self.hold_since = None
        self.search_since = None
        self.motion_anchor = None
        self.gave_up = False
        self.get_logger().info(
            f'Objetivo recibido en "{msg.header.frame_id}": '
            f'({self.goal_x:.2f}, {self.goal_y:.2f})')
        self._publish_goal_marker()

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def control_loop(self):
        if self.goal_x is None or self.mission_done:
            return
        if self.latest_scan is None:
            return

        dist_to_goal = math.hypot(
            self.goal_x - self.current_x, self.goal_y - self.current_y)
        if dist_to_goal < self.goal_tolerance:
            self.stop_robot()
            self.mission_done = True
            self.motion_anchor = None
            self.get_logger().info(
                f'Objetivo alcanzado en ({self.current_x:.2f}, {self.current_y:.2f}). '
                'Mision cumplida.')
            return

        front_min, l_near, l_mid, l_far, r_near, r_mid, r_far = \
            self._scan_sectors(self.latest_scan)
        now = time.monotonic()
        self._debug_trace(now, front_min, l_near, l_mid, l_far, r_near, r_mid, r_far)

        if self.gave_up:
            self.stop_robot()
            return

        if self._is_stalled(now):
            if self.state == 'SEARCH':
                self.gave_up = True
                self.get_logger().error(
                    'Buscando salida pero ni la posicion ni el rumbo '
                    'cambian (bloqueo fisico confirmado): me detengo. '
                    'Reinicia la simulacion, o envia un nuevo objetivo '
                    'para reintentar.')
                self.stop_robot()
                return
            self.state = 'SEARCH'
            self.search_since = now
            self.motion_anchor = None
            left_room = sector_room(l_near, l_mid, l_far)
            right_room = sector_room(r_near, r_mid, r_far)
            self.search_dir = 1 if left_room >= right_room else -1
            self.get_logger().warn(
                'Sin progreso real (ni posicion ni rumbo cambian): '
                'paso a busqueda.')
            return

        angle_to_goal = math.atan2(
            self.goal_y - self.current_y, self.goal_x - self.current_x)
        heading_error = normalize_angle(angle_to_goal - self.current_yaw)

        if self.state == 'TURN':
            if now - self.turn_since > self.turn_max_seconds:
                self.state = 'SEARCH'
                self.search_since = now
                left_room = sector_room(l_near, l_mid, l_far)
                right_room = sector_room(r_near, r_mid, r_far)
                self.search_dir = 1 if left_room >= right_room else -1
                self.get_logger().warn(
                    'El giro tardo demasiado sin alinear: paso a busqueda.')
                return
            hold_mid = l_mid if self.hold_side == 'left' else r_mid
            hold_near = l_near if self.hold_side == 'left' else r_near
            hold_body = sector_room(
                *((l_near, l_mid, l_far) if self.hold_side == 'left'
                  else (r_near, r_mid, r_far)))
            if (hold_mid < self.clearance and hold_near >= self.min_body_clearance
                    and hold_body >= self.min_body_clearance):
                self.state = 'HOLD'
                self.hold_since = now
                self.get_logger().info(
                    f'Alineado con el obstaculo ({self.hold_side} mid='
                    f'{hold_mid:.2f} near={hold_near:.2f}): '
                    'avanzo sosteniendolo.')
                return
            angular_sign = -1.0 if self.hold_side == 'left' else 1.0
            self.publisher_.publish(self._arc_cmd(
                angular_sign, front_min, l_near, l_mid, l_far, r_near, r_mid, r_far))
            return

        if self.state == 'HOLD':
            hold_mid = l_mid if self.hold_side == 'left' else r_mid
            hold_near = l_near if self.hold_side == 'left' else r_near
            hold_body = sector_room(
                *((l_near, l_mid, l_far) if self.hold_side == 'left'
                  else (r_near, r_mid, r_far)))
            held_long_enough = (now - self.hold_since) > self.hold_min_seconds
            if (held_long_enough
                    and hold_near >= self.clearance * self.release_margin
                    and hold_mid >= self.clearance * self.release_margin):
                self.get_logger().info(
                    f'Obstaculo liberado ({self.hold_side} near={hold_near:.2f} '
                    f'mid={hold_mid:.2f}): retomo rumbo al objetivo.')
                self.state = 'ADVANCE'
                self.hold_side = None
                return
            if front_min < self.clearance or hold_body <= self.min_body_clearance:
                action, side = self._decide_front_reaction(
                    heading_error, l_near, l_mid, l_far, r_near, r_mid, r_far)
                if action == 'advance':
                    self.get_logger().info(
                        'En esta esquina el objetivo ya esta accesible '
                        'directo: suelto la pared, no espero a que el '
                        'sensor la libere.')
                    self.state = 'ADVANCE'
                    self.hold_side = None
                    return
                if action == 'search':
                    self.state = 'SEARCH'
                    self.search_since = now
                    left_room = sector_room(l_near, l_mid, l_far)
                    right_room = sector_room(r_near, r_mid, r_far)
                    self.search_dir = 1 if left_room >= right_room else -1
                    self.get_logger().warn(
                        'Esquina y el lado de escape tambien critico: '
                        'voy directo a busqueda.')
                    return
                self.hold_side = side
                angular_sign = -1.0 if self.hold_side == 'left' else 1.0
                self.publisher_.publish(self._arc_cmd(
                    angular_sign, front_min, l_near, l_mid, l_far,
                    r_near, r_mid, r_far))
            else:
                side_near = hold_near
                error = self.hold_target_dist - side_near
                if abs(error) < self.hold_deadband:
                    turn = 0.0
                else:
                    turn = max(-self.hold_max_turn, min(
                        self.hold_max_turn, self.hold_gain * error))
                cmd = Twist()
                cmd.angular.z = -turn if self.hold_side == 'left' else turn
                turn_fraction = abs(turn) / self.hold_max_turn \
                    if self.hold_max_turn > 0 else 0.0
                cmd.linear.x = self.linear_speed * (1.0 - 0.4 * turn_fraction)
                self.publisher_.publish(cmd)
            return

        if self.state == 'SEARCH':
            if now - self.search_since > self.search_max_seconds:
                self.gave_up = True
                self.get_logger().error(
                    f'No encontre salida en {self.search_max_seconds:.0f}s '
                    'buscando: me detengo. El robot puede estar fisicamente '
                    'trabado -- reinicia la simulacion, o envia un nuevo '
                    'objetivo para reintentar.')
                self.stop_robot()
                return
            if (front_min >= self.clearance
                    and sector_room(l_near, l_mid, l_far) >= self.clearance
                    and sector_room(r_near, r_mid, r_far) >= self.clearance):
                self.state = 'ADVANCE'
                self.get_logger().info(
                    'Direccion despejada buscando: retomo avance.')
                return
            self.publisher_.publish(self._arc_cmd(
                float(self.search_dir), front_min,
                l_near, l_mid, l_far, r_near, r_mid, r_far))
            return

        # state == 'ADVANCE'
        if front_min < self.clearance or l_near < self.clearance \
                or r_near < self.clearance:
            goal_side = 'left' if heading_error >= 0 else 'right'
            escape_side = 'right' if goal_side == 'left' else 'left'
            left_room = sector_room(l_near, l_mid, l_far)
            right_room = sector_room(r_near, r_mid, r_far)
            escape_room = left_room if escape_side == 'left' else right_room

            if escape_room < self.min_body_clearance:
                self.state = 'SEARCH'
                self.search_since = now
                self.search_dir = 1 if left_room >= right_room else -1
                self.get_logger().warn(
                    f'Objetivo bloqueado por {goal_side} y el lado de '
                    f'escape ({escape_side}) tambien critico '
                    f'({escape_room:.2f}m): voy directo a busqueda.')
                return

            side_near = l_near if goal_side == 'left' else r_near
            side_mid = l_mid if goal_side == 'left' else r_mid
            if side_near < self.clearance or side_mid < self.clearance:
                self.hold_side = goal_side
                self.state = 'TURN'
                self.turn_since = now
                self.get_logger().info(
                    f'Obstaculo bloquea el lado del objetivo ({goal_side}, '
                    f'near={side_near:.2f} mid={side_mid:.2f}, escape '
                    f'{escape_side} con {escape_room:.2f}m): giro hacia '
                    'el otro lado.')
                angular_sign = -1.0 if goal_side == 'left' else 1.0
                self.publisher_.publish(self._arc_cmd(
                    angular_sign, front_min, l_near, l_mid, l_far,
                    r_near, r_mid, r_far))
                return

        left_room = sector_room(l_near, l_mid, l_far)
        right_room = sector_room(r_near, r_mid, r_far)
        self.publisher_.publish(self._seek_cmd(
            heading_error, front_min, left_room, right_room))

    def _decide_front_reaction(self, heading_error, l_near, l_mid, l_far,
                                r_near, r_mid, r_far):
        goal_side = 'left' if heading_error >= 0 else 'right'
        escape_side = 'right' if goal_side == 'left' else 'left'
        left_room = sector_room(l_near, l_mid, l_far)
        right_room = sector_room(r_near, r_mid, r_far)
        escape_room = left_room if escape_side == 'left' else right_room
        side_near = l_near if goal_side == 'left' else r_near
        side_mid = l_mid if goal_side == 'left' else r_mid

        if side_near >= self.clearance and side_mid >= self.clearance:
            return 'advance', None
        if escape_room < self.min_body_clearance:
            return 'search', None
        return 'turn', goal_side

    def _arc_cmd(self, angular_sign, front_min, l_near, l_mid, l_far,
                 r_near, r_mid, r_far):
        left_room = sector_room(l_near, l_mid, l_far)
        right_room = sector_room(r_near, r_mid, r_far)
        side_tightest = min(left_room, right_room)
        overall_tightest = min(front_min, side_tightest)
        span = max(1e-6, self.clearance - self.min_body_clearance)

        t_side = max(0.0, min(1.0, (side_tightest - self.min_body_clearance) / span))
        t_overall = max(0.0, min(1.0, (overall_tightest - self.min_body_clearance) / span))
        t_front = max(0.0, min(1.0, (front_min - self.min_body_clearance) / span))

        angular_factor = self.turn_min_angular_factor + \
            (1.0 - self.turn_min_angular_factor) * t_overall

        base_angular = angular_sign * self.angular_speed * angular_factor
        lateral = self._lateral_correction(left_room, right_room)
        total_angular = max(-self.angular_speed, min(
            self.angular_speed, base_angular + lateral))

        # El giro tiene direccion verificada. El avance NO -- mientras la
        # correccion esta activa (algun lado ya cruzo el piso fisico), el
        # avance se apaga del todo, se confia solo en el giro. El boost
        # de avance preventivo sigue intacto en la zona ajustada-pero-
        # todavia-no-violada.
        if lateral != 0.0:
            forward_factor = 0.0
        else:
            forward_factor = self.turn_max_forward_factor * (1.0 - t_side) * t_front

        cmd = Twist()
        cmd.angular.z = total_angular
        cmd.linear.x = self.linear_speed * forward_factor
        return cmd

    def _lateral_correction(self, left_room, right_room):
        correction = 0.0
        if left_room < self.min_body_clearance:
            deficit = self.min_body_clearance - left_room
            available = max(0.0, right_room - self.min_body_clearance)
            correction -= min(deficit, available) * self.lateral_correction_gain
        if right_room < self.min_body_clearance:
            deficit = self.min_body_clearance - right_room
            available = max(0.0, left_room - self.min_body_clearance)
            correction += min(deficit, available) * self.lateral_correction_gain
        return correction

    def _seek_cmd(self, heading_error, front_min, left_room, right_room):
        cmd = Twist()
        angular = 2.0 * heading_error
        if left_room < self.min_body_clearance:
            angular -= self.advance_nudge
        if right_room < self.min_body_clearance:
            angular += self.advance_nudge
        cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, angular))
        if front_min < self.clearance or abs(heading_error) > self.seek_rotate_in_place:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.linear_speed * (
                1.0 - abs(heading_error) / self.seek_rotate_in_place)
        return cmd

    def _is_stalled(self, now):
        if self.motion_anchor is None:
            self.motion_anchor = (now, self.current_x, self.current_y, self.current_yaw)
            return False
        t0, x0, y0, yaw0 = self.motion_anchor
        if now - t0 < self.stall_check_seconds:
            return False
        displacement = math.hypot(self.current_x - x0, self.current_y - y0)
        yaw_change = abs(normalize_angle(self.current_yaw - yaw0))
        if displacement < self.stall_min_displacement \
                and yaw_change < self.yaw_stall_min_change:
            return True
        self.motion_anchor = (now, self.current_x, self.current_y, self.current_yaw)
        return False

    def stop_robot(self):
        self.publisher_.publish(Twist())

    def _scan_sectors(self, msg: LaserScan):
        front_vals = []
        left_bins = [[], [], []]
        right_bins = [[], [], []]

        left_span = max(1e-6, msg.angle_max - self.front_half_angle)
        right_span = max(1e-6, (-self.front_half_angle) - msg.angle_min)
        left_bin_w = left_span / 3.0
        right_bin_w = right_span / 3.0

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                if -self.front_half_angle <= angle <= self.front_half_angle:
                    front_vals.append(r)
                elif angle > self.front_half_angle:
                    idx = int((angle - self.front_half_angle) / left_bin_w)
                    left_bins[min(2, max(0, idx))].append(r)
                else:
                    idx = int(((-self.front_half_angle) - angle) / right_bin_w)
                    right_bins[min(2, max(0, idx))].append(r)
            angle += msg.angle_increment

        def m(vals):
            return min(vals) if vals else msg.range_max

        front_min = m(front_vals)
        l_near, l_mid, l_far = m(left_bins[0]), m(left_bins[1]), m(left_bins[2])
        r_near, r_mid, r_far = m(right_bins[0]), m(right_bins[1]), m(right_bins[2])
        return front_min, l_near, l_mid, l_far, r_near, r_mid, r_far

    def _debug_trace(self, now, front_min, l_near, l_mid, l_far, r_near, r_mid, r_far):
        if self._last_trace is None or now - self._last_trace > 0.5:
            self._last_trace = now
            self.get_logger().info(
                f'[trace] state={self.state} hold={self.hold_side} '
                f'pos=({self.current_x:.2f},{self.current_y:.2f}) '
                f'yaw={math.degrees(self.current_yaw):.0f} front={front_min:.2f} '
                f'lN={l_near:.2f} lM={l_mid:.2f} lF={l_far:.2f} '
                f'rN={r_near:.2f} rM={r_mid:.2f} rF={r_far:.2f}')

    def _publish_goal_marker(self):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'goal'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = self.goal_x
        m.pose.position.y = self.goal_y
        m.pose.position.z = 0.2
        m.pose.orientation.w = 1.0
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 0.9
        self.goal_marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
