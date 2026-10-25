"""Zustandsmaschine fuer den Fahrmodus, einziger Publisher von /control/mode.

Ein:  /scan, /avoid/zones (AvoidZones), /detect/sign, /control/tunnel_request
Aus:  /control/mode (UInt8, ControlType), /control/tunnel_source (String)
Schwellwerte und Entprellung fuer jeden Moduswechsel liegen nur hier.
"""
import math

import numpy as np

import rclpy
from follow_lane_msg.msg import AvoidZones
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src.common import (
    ControlType, SignType, frange, pose2d_from_transform, read_params, run_node, scan_points)

# Zyklisch senden, damit auch spaeter startende Nodes den Modus bekommen.
# control_driving haelt an, wenn er laenger als mode_timeout nichts hoert.
MODE_PUBLISH_PERIOD_S = 0.5

TUNNEL_MODES = (ControlType.TunnelEntry, ControlType.Tunnel)


class switch_control(Node):
    """Lane -> Obstacle: die Frontzone von avoid_node ist naeher als takeover_distance.
    Bis dahin weicht der Bot im Lane-Modus ueber den Offset auf den Spurfehler aus.

    Obstacle -> Lane: die Frontzone ist takeover_release_debounce lang weiter als
    takeover_release_distance. Ohne diese Hysterese springt der Modus an der Schwelle im
    Sekundentakt zwischen PID und Ausweichmanoever.

    -> TunnelEntry per Schild (/detect/sign = SignType.Tunnel, Quelle "auto") oder
    Button im Debugbild (/control/tunnel_request True, Quelle "manuell"). Der Button
    ist noetig, weil jede Challenge einzeln laeuft und es vor Ort sonst keinen
    Ausloeser gibt. Der Bot steht dabei vor dem Tunnel und faehrt mit Kamera auf die
    Oeffnung zu; die Einfahrt ist laut Reglement noch erlaubt.

    TunnelEntry -> Tunnel, bevor der Bot drin ist: der LiDAR sieht links und rechts
    voraus Waende naeher als tunnel_side_distance (die Tunnelwaende neben der
    Oeffnung), oder der Bot ist seit dem Umschalten tunnel_entry_max_distance
    gefahren. Ab dann keine Kamera mehr.

    Tunnel -> Modus von vor dem Tunnel, sobald der LiDAR rundum freien Raum sieht.
    Diese Pruefung laeuft erst ab Tunnel, nicht schon vor der Oeffnung, wo es meist
    auch frei ist. Beides jederzeit von Hand abbrechbar (False).
    """

    def __init__(self):
        super().__init__('switch_control')

        # --- Parameter ---
        self.declare_parameter('takeover_distance', 0.20, frange(0.05, 3.0))
        self.declare_parameter('takeover_release_distance', 0.45, frange(0.05, 5.0))
        self.declare_parameter('takeover_release_debounce', 1.0, frange(0.0, 10.0))
        # Seitensektoren (Grad ab geradeaus, links positiv, rechts gespiegelt), in denen
        # die Tunnelwaende neben der Oeffnung gesucht werden. Ab 30 Grad: Hindernisse
        # direkt voraus zaehlen nicht als Wand.
        self.declare_parameter('tunnel_side_angle_min_deg', 30.0, frange(0.0, 90.0))
        self.declare_parameter('tunnel_side_angle_max_deg', 100.0, frange(0.0, 180.0))
        # Haengt an der Tunnelbreite: halbe Breite plus Reserve. Kleiner = es wird
        # spaeter umgeschaltet, also naeher an der Oeffnung.
        self.declare_parameter('tunnel_side_distance', 0.35, frange(0.05, 2.0))
        self.declare_parameter('tunnel_entry_debounce', 0.2, frange(0.0, 5.0))
        # Sicherheitsnetz, falls die Waende nie erkannt werden: spaetestens nach dieser
        # Strecke (m) ab dem Umschalten gilt der Bot als drin und die Kamera geht aus.
        self.declare_parameter('tunnel_entry_max_distance', 0.8, frange(0.05, 5.0))
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('tunnel_exit_open_distance', 1.5, frange(0.1, 10.0))
        self.declare_parameter('tunnel_exit_open_ratio', 0.8, frange(0.0, 1.0))
        self.declare_parameter('tunnel_exit_debounce', 1.0, frange(0.0, 10.0))
        self._param_names = [
            'takeover_distance', 'takeover_release_distance', 'takeover_release_debounce',
            'tunnel_side_angle_min_deg', 'tunnel_side_angle_max_deg', 'tunnel_side_distance',
            'tunnel_entry_debounce', 'tunnel_entry_max_distance', 'odom_frame', 'robot_frame',
            'tunnel_exit_open_distance', 'tunnel_exit_open_ratio', 'tunnel_exit_debounce',
        ]
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.sub_avoid_zones = self.create_subscription(
            AvoidZones, '/avoid/zones', self.cb_avoid_zones, 1)
        # Noch ohne Publisher: der Schild-Detektor ist nicht gebaut.
        self.sub_sign = self.create_subscription(UInt8, '/detect/sign', self.cb_sign, 1)
        self.sub_tunnel_request = self.create_subscription(
            Bool, '/control/tunnel_request', self.cb_tunnel_request, 1)

        # --- Publisher ---
        self.pub_control_mode = self.create_publisher(UInt8, '/control/mode', 1)
        self.pub_tunnel_source = self.create_publisher(String, '/control/tunnel_source', 1)

        # --- Timer ---
        self.timer = self.create_timer(MODE_PUBLISH_PERIOD_S, self.cb_timer)

        # --- Zustand ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = ControlType.Lane
        self._mode_before_tunnel = None
        self._tunnel_source = ''          # 'auto' (Schild) oder 'manuell' (Button)
        self._obstacle_clear_since = None
        self._tunnel_entry_since = None
        self._tunnel_entry_start = None   # (x, y) im odom_frame beim Umschalten
        self._tunnel_exit_since = None

        self.get_logger().info('Started switch_control Node.')

    # --- Callbacks ---

    def cb_timer(self):
        self._publish_mode()

    def cb_avoid_zones(self, msg):
        read_params(self, self._param_names)

        if self.control_mode == ControlType.Lane:
            if msg.front < self.takeover_distance:
                self._set_mode(ControlType.Obstacle, f'Frontzone blockiert ({msg.front:.2f} m)')
        elif self.control_mode == ControlType.Obstacle:
            self._update_obstacle_exit(msg.front)

    def cb_sign(self, msg):
        try:
            sign = SignType(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unbekanntes Schild-Signal: {msg.data}')
            return

        if sign == SignType.Tunnel:
            self._enter_tunnel('auto')

    def cb_tunnel_request(self, msg):
        if msg.data:
            self._enter_tunnel('manuell')
        elif self.control_mode in TUNNEL_MODES:
            self.get_logger().info('Tunnel von Hand abgebrochen.')
            self._leave_tunnel()

    def cb_scan(self, scan):
        read_params(self, self._param_names)

        if self.control_mode == ControlType.TunnelEntry:
            self._update_tunnel_entry(scan)
        elif self.control_mode == ControlType.Tunnel:
            self._update_tunnel_exit(scan)

    # --- Hilfsmethoden ---

    def _set_mode(self, new_mode, reason):
        if new_mode == self.control_mode:
            return
        self.get_logger().info(f'{reason} -> {self.control_mode.name} => {new_mode.name}.')
        self.control_mode = new_mode
        # Entprell-Zeitstempel gelten nur fuer den Modus, in dem sie gesetzt wurden.
        self._obstacle_clear_since = None
        self._tunnel_entry_since = None
        self._tunnel_exit_since = None
        # Sofort senden, nicht erst mit dem Timer: control_driving schaltet sonst
        # bis zu MODE_PUBLISH_PERIOD_S spaeter um.
        self._publish_mode()

    def _publish_mode(self):
        self.pub_control_mode.publish(UInt8(data=self.control_mode.value))
        self.pub_tunnel_source.publish(String(data=self._tunnel_source))

    def _enter_tunnel(self, source):
        if self.control_mode in TUNNEL_MODES:
            return
        self._mode_before_tunnel = self.control_mode
        self._tunnel_source = source
        self._tunnel_entry_start = self._robot_xy()
        self._set_mode(ControlType.TunnelEntry, f'Tunnel ausgeloest ({source})')

    def _update_tunnel_entry(self, scan):
        if self._side_walls(scan):
            now = self.get_clock().now()
            if self._tunnel_entry_since is None:
                self._tunnel_entry_since = now
            elif (now - self._tunnel_entry_since).nanoseconds / 1e9 >= self.tunnel_entry_debounce:
                self._set_mode(ControlType.Tunnel, 'Tunnelwaende links und rechts')
                return
        else:
            self._tunnel_entry_since = None

        if self._tunnel_entry_start is None:
            self._tunnel_entry_start = self._robot_xy()
            return
        position = self._robot_xy()
        if position is not None and math.dist(position, self._tunnel_entry_start) \
                >= self.tunnel_entry_max_distance:
            self._set_mode(ControlType.Tunnel, 'Einfahrtsstrecke erreicht, Tunnelwaende nicht erkannt')

    def _side_walls(self, scan):
        """Sieht der LiDAR links und rechts voraus eine Wand naeher als tunnel_side_distance?"""
        angles, ranges, _ = scan_points(scan)
        near = ranges < self.tunnel_side_distance
        lo = math.radians(self.tunnel_side_angle_min_deg)
        hi = math.radians(self.tunnel_side_angle_max_deg)
        left = near & (angles >= lo) & (angles <= hi)
        right = near & (angles <= -lo) & (angles >= -hi)
        return bool(left.any() and right.any())

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Keine Roboterpose fuer die Tunnel-Einfahrt: {e}',
                                   throttle_duration_sec=5.0)
            return None
        x, y, _ = pose2d_from_transform(tf)
        return (x, y)

    def _leave_tunnel(self):
        restored = self._mode_before_tunnel or ControlType.Lane
        self._mode_before_tunnel = None
        self._tunnel_source = ''
        self._tunnel_entry_start = None
        self._set_mode(restored, 'Tunnel verlassen')

    def _update_obstacle_exit(self, front):
        if front <= self.takeover_release_distance:
            self._obstacle_clear_since = None
            return

        now = self.get_clock().now()
        if self._obstacle_clear_since is None:
            self._obstacle_clear_since = now
        elif (now - self._obstacle_clear_since).nanoseconds / 1e9 >= self.takeover_release_debounce:
            self._set_mode(ControlType.Lane, f'Front wieder frei ({front:.2f} m)')

    def _update_tunnel_exit(self, scan):
        if not self._is_open_space(scan):
            self._tunnel_exit_since = None
            return

        now = self.get_clock().now()
        if self._tunnel_exit_since is None:
            self._tunnel_exit_since = now
        elif (now - self._tunnel_exit_since).nanoseconds / 1e9 >= self.tunnel_exit_debounce:
            self._leave_tunnel()

    def _is_open_space(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        if ranges.size == 0:
            return False

        # inf = kein Echo = frei. Nicht als ungueltig verwerfen, sonst bleiben nur
        # die kurzen Messwerte uebrig und der Tunnel liesse sich nie verlassen.
        no_echo = ~np.isfinite(ranges)
        far = np.isfinite(ranges) & (ranges > self.tunnel_exit_open_distance)
        # Werte unter range_min sind Artefakte, nicht "frei".
        too_close = np.isfinite(ranges) & (ranges <= scan.range_min)
        if np.count_nonzero(too_close) == ranges.size:
            return False

        open_ratio = np.count_nonzero(no_echo | far) / ranges.size
        return open_ratio >= self.tunnel_exit_open_ratio

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down switch_control Node.')


def main(args=None):
    run_node(switch_control, args)


if __name__ == '__main__':
    main()
