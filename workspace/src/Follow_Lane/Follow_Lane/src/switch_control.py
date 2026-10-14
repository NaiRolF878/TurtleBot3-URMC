import math
from enum import Enum

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan


class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Stop = 3
    Intersection = 4
    Align = 5
    Tunnel = 6


class switch_control(Node):
    """Zustandsmaschine fuer /control/mode -- einziger Publisher dieses Topics.

    Lane <-> Obstacle: per LiDAR, sobald etwas im Pruefkegel naeher als
    obstacle_trigger_distance liegt (mit Entprellung zurueck).

    -> Tunnel: per Schild auf /detect/sign. Der Detektor dafuer fehlt noch,
    manuell testbar mit
      ros2 topic pub /detect/sign std_msgs/msg/UInt8 "{data: 6}"
    Zurueck, sobald der LiDAR freien Raum sieht; wiederhergestellt wird der
    Modus von vor dem Tunnel. Bewusst ohne Kamera: in der Enclosure ist sie laut
    Reglement verboten.
    """

    def __init__(self):
        super().__init__('switch_control')

        self.get_logger().info('Started switch_control Node.')

        def frange(lo, hi):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

        # --- Obstacle-Erkennung (Lane <-> Obstacle) ---
        self.declare_parameter('obstacle_check_angle_deg', 30.0, frange(1.0, 90.0))
        self.declare_parameter('obstacle_trigger_distance', 0.4, frange(0.05, 3.0))
        self.declare_parameter('obstacle_clear_debounce', 1.0, frange(0.0, 10.0))

        # --- Tunnel-Austritt (Tunnel -> vorheriger Modus) ---
        self.declare_parameter('tunnel_exit_open_distance', 1.5, frange(0.1, 10.0))
        self.declare_parameter('tunnel_exit_open_ratio', 0.8, frange(0.0, 1.0))
        self.declare_parameter('tunnel_exit_debounce', 1.0, frange(0.0, 10.0))

        self._param_names = [
            'obstacle_check_angle_deg', 'obstacle_trigger_distance', 'obstacle_clear_debounce',
            'tunnel_exit_open_distance', 'tunnel_exit_open_ratio',
            'tunnel_exit_debounce',
        ]

        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cbScan, qos_profile_sensor_data)
        # Publisher folgt mit der Schild-Detektor-Node
        self.sub_sign = self.create_subscription(UInt8, '/detect/sign', self.cbSign, 1)

        self.pub_control = self.create_publisher(UInt8, '/control/mode', 1)

        # Zyklisch senden, damit auch spaeter startende Nodes den Mode bekommen.
        self.timer = self.create_timer(0.5, self.cbPubControlMode)

        self.current_control_mode = ControlType.Lane
        self._mode_before_tunnel = None
        self._obstacle_clear_since = None
        self._tunnel_exit_since = None

    def _read_params(self):
        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

    def cbPubControlMode(self):
        msg = UInt8()
        msg.data = self.current_control_mode.value
        self.pub_control.publish(msg)

    def cbSign(self, msg):
        try:
            sign = ControlType(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unbekanntes Schild-Signal: {msg.data}')
            return

        if sign == ControlType.Tunnel:
            if self.current_control_mode != ControlType.Tunnel:
                self._mode_before_tunnel = self.current_control_mode
                self.current_control_mode = ControlType.Tunnel
                self._tunnel_exit_since = None
                self.get_logger().info('Tunnel-Schild erkannt -> wechsle zu Tunnel-Modus.')
        else:
            self.get_logger().debug(
                f'Schild {sign.name} erkannt, dafuer ist noch keine Aktion definiert.')

    def cbScan(self, scan):
        self._read_params()

        if self.current_control_mode == ControlType.Tunnel:
            self._update_tunnel_exit(scan)
        elif self.current_control_mode in (ControlType.Lane, ControlType.Obstacle):
            self._update_obstacle_mode(scan)
        # Andere Modi (Stop/Intersection/Align): hier (noch) keine automatische Logik.

    def _update_obstacle_mode(self, scan):
        obstacle_present = self._forward_obstacle_present(scan)

        if obstacle_present:
            self._obstacle_clear_since = None
            if self.current_control_mode == ControlType.Lane:
                self.current_control_mode = ControlType.Obstacle
                self.get_logger().info('Hindernis in der Fahrbahn erkannt -> wechsle zu Obstacle-Modus.')
        elif self.current_control_mode == ControlType.Obstacle:
            now = self.get_clock().now()
            if self._obstacle_clear_since is None:
                self._obstacle_clear_since = now
            elif (now - self._obstacle_clear_since).nanoseconds / 1e9 >= self.obstacle_clear_debounce:
                self.current_control_mode = ControlType.Lane
                self._obstacle_clear_since = None
                self.get_logger().info('Kein Hindernis mehr erkannt -> zurueck zu Lane-Modus.')

    def _forward_obstacle_present(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        if ranges.size == 0:
            return False

        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        # Auf [-pi, pi] normalisieren, damit 0 rad wirklich geradeaus ist
        angles = np.arctan2(np.sin(angles), np.cos(angles))

        # Halbwinkel: der Kegel ist doppelt so breit wie der Parameterwert.
        half_angle = math.radians(self.obstacle_check_angle_deg)
        in_cone = np.abs(angles) <= half_angle
        valid = np.isfinite(ranges) & (ranges > scan.range_min)
        close = ranges < self.obstacle_trigger_distance

        return bool(np.any(in_cone & valid & close))

    def _update_tunnel_exit(self, scan):
        scan_open = self._is_open_space(scan)

        if scan_open:
            now = self.get_clock().now()
            if self._tunnel_exit_since is None:
                self._tunnel_exit_since = now
            elif (now - self._tunnel_exit_since).nanoseconds / 1e9 >= self.tunnel_exit_debounce:
                self._leave_tunnel()
        else:
            self._tunnel_exit_since = None

    def _is_open_space(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        if ranges.size == 0:
            return False

        # inf = kein Echo = frei. Nicht als ungueltig verwerfen, sonst bleiben nur
        # die kurzen Messwerte uebrig und der Tunnel liesse sich nie verlassen.
        no_echo = ~np.isfinite(ranges)
        far = np.isfinite(ranges) & (ranges > self.tunnel_exit_open_distance)
        # Zu kurze Werte sind Artefakte, nicht "frei"
        too_close = np.isfinite(ranges) & (ranges <= scan.range_min)

        open_ratio = np.count_nonzero(no_echo | far) / ranges.size
        if np.count_nonzero(too_close) == ranges.size:
            return False
        return open_ratio >= self.tunnel_exit_open_ratio

    def _leave_tunnel(self):
        restored = self._mode_before_tunnel or ControlType.Lane
        self.get_logger().info(f'Tunnel verlassen -> zurueck zu {restored.name}.')
        self.current_control_mode = restored
        self._mode_before_tunnel = None
        self._tunnel_exit_since = None


def main(args=None):
    rclpy.init(args=args)
    node = switch_control()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:

        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
