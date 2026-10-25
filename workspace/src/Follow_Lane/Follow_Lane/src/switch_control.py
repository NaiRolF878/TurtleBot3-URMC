"""Zustandsmaschine fuer den Fahrmodus, einziger Publisher von /control/mode.

Ein:  /scan, /detect/lane_obstacles (Spurgedaechtnis), /detect/sign, /control/tunnel_request
Aus:  /control/mode (UInt8, ControlType), /control/tunnel_source (String)
Schwellwerte und Entprellung fuer jeden Moduswechsel liegen nur hier.
"""
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src.common import (
    ControlType, SignType, cloud_xy, frange, pose2d_from_transform, read_params, run_node,
    scan_points, transform_points)

# Zyklisch senden, damit auch spaeter startende Nodes den Modus bekommen.
# control_driving haelt an, wenn er laenger als mode_timeout nichts hoert.
MODE_PUBLISH_PERIOD_S = 0.5

TUNNEL_MODES = (ControlType.TunnelEntry, ControlType.Tunnel)


class switch_control(Node):
    """Lane -> Obstacle: ein LiDAR-Punkt im Kegel +/-obstacle_check_angle_deg naeher als
    obstacle_trigger_distance, und zwischen Bot und Punkt liegt keine Spurlinie. Ein
    Objekt hinter einer Linie gehoert nicht zur eigenen Fahrbahn.

    Obstacle -> Lane: obstacle_clear_debounce lang kein solcher Punkt naeher als
    obstacle_clear_distance im Bereich +/-obstacle_clear_angle_deg. Der Keil direkt
    hinter dem Bot bleibt aussen vor: aus dieser Richtung kam er gerade.

    Die Linien kommen aus dem Spurgedaechtnis von lane_costmap_node, kennen also auch
    Linien neben und hinter dem Bot, die die Kamera gerade nicht sieht. Ist es aelter
    als lane_max_age, zaehlt jeder Punkt (lieber zu oft Obstacle als ein Hindernis
    uebersehen). Entfernungen und Winkel gelten ab dem LiDAR, die Linienpruefung ab
    der Bot-Mitte (base_footprint).

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
        # Halbwinkel: der Kegel ist doppelt so breit.
        self.declare_parameter('obstacle_check_angle_deg', 30.0, frange(1.0, 90.0))
        self.declare_parameter('obstacle_trigger_distance', 0.4, frange(0.05, 3.0))
        # Halbwinkel des Austrittsbereichs: 135 = 270 Grad ohne den Keil hinten.
        self.declare_parameter('obstacle_clear_angle_deg', 135.0, frange(1.0, 180.0))
        self.declare_parameter('obstacle_clear_distance', 0.2, frange(0.05, 3.0))
        self.declare_parameter('obstacle_clear_debounce', 1.0, frange(0.0, 10.0))
        # Ein Linienpunkt naeher als line_block_tolerance an der Strecke Bot -> Hindernis
        # trennt beide. Linienpunkte bis line_obstacle_margin vor dem Hindernis zaehlen
        # nicht: steht es auf der Linie, ragt es in die Fahrbahn.
        self.declare_parameter('line_block_tolerance', 0.02, frange(0.0, 0.2))
        self.declare_parameter('line_obstacle_margin', 0.05, frange(0.0, 0.5))
        self.declare_parameter('lane_max_age', 0.5, frange(0.05, 5.0))
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
            'obstacle_check_angle_deg', 'obstacle_trigger_distance',
            'obstacle_clear_angle_deg', 'obstacle_clear_distance', 'obstacle_clear_debounce',
            'line_block_tolerance', 'line_obstacle_margin', 'lane_max_age',
            'tunnel_side_angle_min_deg', 'tunnel_side_angle_max_deg', 'tunnel_side_distance',
            'tunnel_entry_debounce', 'tunnel_entry_max_distance', 'odom_frame', 'robot_frame',
            'tunnel_exit_open_distance', 'tunnel_exit_open_ratio', 'tunnel_exit_debounce',
        ]
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.sub_lane_obstacles = self.create_subscription(
            PointCloud2, '/detect/lane_obstacles', self.cb_lane_obstacles, 1)
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
        self._lane_points = None          # (N, 2) im Frame der Wolke
        self._lane_frame = None
        self._lane_time = None
        self._scanner_poses = {}          # (Ziel-Frame, Scanner-Frame) -> (x, y, yaw)

        self.get_logger().info('Started switch_control Node.')

    # --- Callbacks ---

    def cb_timer(self):
        self._publish_mode()

    def cb_lane_obstacles(self, cloud):
        try:
            self._lane_points = cloud_xy(cloud)
        except ValueError as e:
            self.get_logger().warn(f'/detect/lane_obstacles nicht lesbar: {e}',
                                   throttle_duration_sec=10.0)
            return
        self._lane_frame = cloud.header.frame_id
        self._lane_time = time.monotonic()

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
        elif self.control_mode == ControlType.Lane:
            if self._obstacle_in_lane(scan, self.obstacle_check_angle_deg,
                                      self.obstacle_trigger_distance):
                self._set_mode(ControlType.Obstacle, 'Hindernis in der Fahrbahn')
        elif self.control_mode == ControlType.Obstacle:
            self._update_obstacle_exit(scan)

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

    def _update_obstacle_exit(self, scan):
        if self._obstacle_in_lane(scan, self.obstacle_clear_angle_deg,
                                  self.obstacle_clear_distance):
            self._obstacle_clear_since = None
            return

        now = self.get_clock().now()
        if self._obstacle_clear_since is None:
            self._obstacle_clear_since = now
        elif (now - self._obstacle_clear_since).nanoseconds / 1e9 >= self.obstacle_clear_debounce:
            self._set_mode(ControlType.Lane, 'Kein Hindernis mehr in der Naehe')

    def _obstacle_in_lane(self, scan, half_angle_deg, distance):
        """Liegt ein LiDAR-Punkt im Bereich +/-half_angle_deg naeher als distance, der
        nicht hinter einer Spurlinie liegt?"""
        angles, ranges, points = scan_points(scan)
        near = (np.abs(angles) <= math.radians(half_angle_deg)) & (ranges < distance)
        if not near.any():
            return False

        lanes = self._fresh_lane_points()
        scanner = self._scanner_pose(self._lane_frame, scan.header.frame_id) \
            if lanes is not None else None
        if scanner is None:
            return True
        candidates = transform_points(scanner, points[near])
        return bool((~self._behind_line(candidates, lanes)).any())

    def _fresh_lane_points(self):
        if self._lane_time is None or self._lane_points is None:
            return None
        if time.monotonic() - self._lane_time > self.lane_max_age:
            return None
        return self._lane_points

    def _behind_line(self, points, lanes):
        """Fuer jeden Punkt (M, 2): liegt ein Linienpunkt auf der Strecke vom Ursprung
        (Bot-Mitte) zum Punkt, mehr als line_obstacle_margin vor ihm?"""
        if lanes.size == 0:
            return np.zeros(len(points), dtype=bool)
        dist = np.hypot(points[:, 0], points[:, 1])
        unit = points / np.maximum(dist, 1e-9)[:, None]
        # Linienpunkte jenseits des fernsten Kandidaten koennen nichts verdecken.
        lanes = lanes[np.hypot(lanes[:, 0], lanes[:, 1]) < dist.max()]
        if lanes.size == 0:
            return np.zeros(len(points), dtype=bool)
        along = lanes @ unit.T                                    # (N, M)
        across = np.abs(lanes[:, :1] * unit[:, 1] - lanes[:, 1:] * unit[:, 0])
        between = (along > 0.0) & (along < dist - self.line_obstacle_margin)
        return ((across <= self.line_block_tolerance) & between).any(axis=0)

    def _scanner_pose(self, target_frame, scanner_frame):
        """Lage des Scanners im target_frame. Fest montiert, daher einmal nachschlagen."""
        key = (target_frame, scanner_frame)
        if key not in self._scanner_poses:
            try:
                tf = self.tf_buffer.lookup_transform(target_frame, scanner_frame, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(
                    f'Keine TF {target_frame} <- {scanner_frame}, Linienfilter aus: {e}',
                    throttle_duration_sec=5.0)
                return None
            self._scanner_poses[key] = pose2d_from_transform(tf)
        return self._scanner_poses[key]

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
