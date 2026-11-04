"""Zustandsmaschine fuer den Fahrmodus, einziger Publisher von /control/mode.

Ein:  /scan, /detect/lane_obstacles (Spurgedaechtnis), /detect/sign, /control/tunnel_request,
      /detect/lines_lost, /tunnel/outside
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
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src.common import (
    ControlType, SignType, cloud_xy, frange, pose2d_from_transform, read_params, require_params,
    run_node, scan_points, transform_points)

# Zyklisch senden, damit auch spaeter startende Nodes den Modus bekommen.
# control_driving haelt an, wenn er laenger als mode_timeout nichts hoert.
MODE_PUBLISH_PERIOD_S = 0.5
# So lange (s) muessen beide Spurlinien fehlen, bis die Kamera ausgeht: mehrere Bilder,
# aber die Bot-Nase ist dann noch vor dem Linienende (Suchzeile endet ~10 cm vor der Achse).
LINES_LOST_S = 0.2

TUNNEL_MODES = (ControlType.TunnelEntry, ControlType.Tunnel)

# Halbwinkel des Austrittsbereichs: 135 = 270 Grad ohne den Keil hinten.
OBSTACLE_CLEAR_ANGLE_DEG = 135.0
OBSTACLE_CLEAR_DEBOUNCE_S = 1.0
# Ein Linienpunkt naeher als LINE_BLOCK_TOLERANCE an der Strecke Bot -> Hindernis trennt beide.
# Linienpunkte bis LINE_OBSTACLE_MARGIN vor dem Hindernis zaehlen nicht: steht es auf der
# Linie, ragt es in die Fahrbahn.
LINE_BLOCK_TOLERANCE = 0.02
LINE_OBSTACLE_MARGIN = 0.05
LANE_MAX_AGE_S = 0.5


class switch_control(Node):
    """Lane -> Obstacle: ein LiDAR-Punkt im Kegel +/-obstacle_check_angle_deg naeher als
    obstacle_trigger_distance, und zwischen Bot und Punkt liegt keine Spurlinie. Ein
    Objekt hinter einer Linie gehoert nicht zur eigenen Fahrbahn.

    Obstacle -> Lane: OBSTACLE_CLEAR_DEBOUNCE_S lang kein solcher Punkt naeher als
    obstacle_clear_distance im Bereich +/-OBSTACLE_CLEAR_ANGLE_DEG. Der Keil direkt
    hinter dem Bot bleibt aussen vor: aus dieser Richtung kam er gerade.

    Die Linien kommen aus dem Spurgedaechtnis von lane_costmap_node, kennen also auch
    Linien neben und hinter dem Bot, die die Kamera gerade nicht sieht. Es enthaelt
    nur die gelbe und die weisse Spurlinie: eine Querlinie vor dem Hindernis trennt nicht. Ist es aelter
    als LANE_MAX_AGE_S, zaehlt jeder Punkt (lieber zu oft Obstacle als ein Hindernis
    uebersehen). Entfernungen und Winkel gelten ab dem LiDAR, die Linienpruefung ab
    der Bot-Mitte (base_footprint).

    -> TunnelEntry per Schild (/detect/sign = SignType.Tunnel, Quelle "auto") oder
    Button im Debugbild (/control/tunnel_request True, Quelle "manuell"). Der Button
    ist noetig, weil jede Challenge einzeln laeuft und es vor Ort sonst keinen
    Ausloeser gibt. Der Bot steht dabei vor dem Tunnel und faehrt mit Kamera auf die
    Oeffnung zu; die Einfahrt ist laut Reglement noch erlaubt.

    TunnelEntry -> Tunnel, bevor der Bot drin ist: beide Spurlinien enden kurz vor dem
    Bot, denn sie reichen nur bis zur Oeffnung. Ab dann keine Kamera mehr. Durch die Tuer
    faehrt er nur mit LiDAR (tunnel_explorer_node).

    Tunnel -> Modus von vor dem Tunnel, sobald tunnel_explorer_node meldet, dass der Bot
    ausserhalb der Wand-Huelle ist. Beides jederzeit von Hand abbrechbar (False).
    """

    def __init__(self):
        super().__init__('switch_control')

        # --- Parameter ---
        # Vor Ort einstellen, Werte nur in Switch_Control.yaml. Halbwinkel: der Kegel ist
        # doppelt so breit.
        self.declare_parameter('obstacle_check_angle_deg', Parameter.Type.DOUBLE, frange(1.0, 90.0))
        self.declare_parameter('obstacle_trigger_distance', Parameter.Type.DOUBLE, frange(0.05, 3.0))
        self.declare_parameter('obstacle_clear_distance', Parameter.Type.DOUBLE, frange(0.05, 3.0))
        self._param_names = ['obstacle_check_angle_deg', 'obstacle_trigger_distance',
                             'obstacle_clear_distance']
        require_params(self, self._param_names)
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
        self.sub_lines_lost = self.create_subscription(
            Bool, '/detect/lines_lost', self.cb_lines_lost, 1)
        self.sub_tunnel_outside = self.create_subscription(
            Bool, '/tunnel/outside', self.cb_tunnel_outside, 1)

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
        self._lines_seen_time = None      # zuletzt eine Spurlinie gesehen (TunnelEntry)
        self._lane_points = None          # (N, 2) im Frame der Wolke
        self._lane_frame = None
        self._lane_time = None
        self._scanner_poses = {}          # (Ziel-Frame, Scanner-Frame) -> (x, y, yaw)

        self.get_logger().info('Started switch_control Node.')

    # --- Callbacks ---

    def cb_timer(self):
        self._update_tunnel_entry()
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

    def cb_lines_lost(self, msg):
        if self.control_mode == ControlType.TunnelEntry and not msg.data:
            self._lines_seen_time = time.monotonic()
        self._update_tunnel_entry()

    def cb_tunnel_outside(self, msg):
        if msg.data and self.control_mode == ControlType.Tunnel:
            self.get_logger().info('Bot ist ausserhalb der Wand-Huelle.')
            self._leave_tunnel()

    def cb_scan(self, scan):
        read_params(self, self._param_names)

        self._update_tunnel_entry()
        if self.control_mode == ControlType.Lane:
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
        self._lines_seen_time = time.monotonic()
        self._set_mode(ControlType.TunnelEntry, f'Tunnel ausgeloest ({source})')

    def _update_tunnel_entry(self):
        """Ohne neue Bilder gilt die Spur ebenfalls als zu Ende."""
        if self.control_mode == ControlType.TunnelEntry \
                and time.monotonic() - self._lines_seen_time >= LINES_LOST_S:
            self._set_mode(ControlType.Tunnel, 'Spurlinien enden')

    def _leave_tunnel(self):
        restored = self._mode_before_tunnel or ControlType.Lane
        self._mode_before_tunnel = None
        self._tunnel_source = ''
        self._set_mode(restored, 'Tunnel verlassen')

    def _update_obstacle_exit(self, scan):
        if self._obstacle_in_lane(scan, OBSTACLE_CLEAR_ANGLE_DEG,
                                  self.obstacle_clear_distance):
            self._obstacle_clear_since = None
            return

        now = self.get_clock().now()
        if self._obstacle_clear_since is None:
            self._obstacle_clear_since = now
        elif (now - self._obstacle_clear_since).nanoseconds / 1e9 >= OBSTACLE_CLEAR_DEBOUNCE_S:
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
        if time.monotonic() - self._lane_time > LANE_MAX_AGE_S:
            return None
        return self._lane_points

    def _behind_line(self, points, lanes):
        """Fuer jeden Punkt (M, 2): liegt ein Linienpunkt auf der Strecke vom Ursprung
        (Bot-Mitte) zum Punkt, mehr als LINE_OBSTACLE_MARGIN vor ihm?"""
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
        between = (along > 0.0) & (along < dist - LINE_OBSTACLE_MARGIN)
        return ((across <= LINE_BLOCK_TOLERANCE) & between).any(axis=0)

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

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down switch_control Node.')


def main(args=None):
    run_node(switch_control, args)


if __name__ == '__main__':
    main()
