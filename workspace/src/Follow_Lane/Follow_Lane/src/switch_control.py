"""Zustandsmaschine fuer den Fahrmodus, einziger Publisher von /control/mode.

Ein:  /scan_corrected, /detect/lane_obstacles (Spurgedaechtnis), /detect/sign,
      /control/tunnel_request, /detect/lines_lost, /tunnel/outside, /detect/barrier,
      /intersection/active, dazu die Eingaenge der Storage-Aufgabe (storage_sequence.py)
Aus:  /control/mode (UInt8, ControlType), /control/tunnel_source (String),
      /control/obstacle_enabled (Bool, fuers Debugbild), /control/align_yaw (Float64, Align),
      dazu die Ausgaenge der Storage-Aufgabe (Zustand, Grundbewegungen, Magnet)
Schwellwerte und Entprellung fuer jeden Moduswechsel liegen nur hier. Ablaeufe aus
Grundbewegungen in Obstacle und Tunnel (Engstelle, Freifahren, Weg genau abfahren) steuert
ManeuverSequence (maneuver_sequence.py), die der Storage-Aufgabe StorageSequence.
"""
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float64, String, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src.common import (
    ControlType, SignType, cloud_xy, frange, pose2d_from_transform, read_params, require_params,
    run_node, scan_points, transform_points)
from Follow_Lane.src.maneuver_sequence import SEQUENCE_MODES, ManeuverSequence
from Follow_Lane.src.storage_sequence import StorageSequence

# Zyklisch senden, damit auch spaeter startende Nodes den Modus bekommen.
# control_driving haelt an, wenn er laenger als MODE_TIMEOUT nichts hoert.
MODE_PUBLISH_PERIOD_S = 0.5
# So viele Bilder in Folge muessen beide Spurlinien fehlen, bis die Kamera ausgeht. Bilder
# statt Zeit: ein einzelnes langsames Bild darf nicht reichen. Ein Bild mit Linie setzt zurueck.
LINES_LOST_FRAMES = 5
# Kommt so lange (s) gar kein Bild (detect_lane steht), gilt die Spur ebenfalls als zu Ende.
LINES_NO_IMAGE_S = 1.0

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
# Schranke: so viele Meldungen in Folge (Rohbilder, ~30/s) bis Stop, so lange (s) frei bis
# zurueck zu Lane.
BARRIER_CONFIRM = 3
BARRIER_CLEAR_DEBOUNCE_S = 1.0

# Ausrichten nach Obstacle (Modus Align): Spurrichtung = Hauptachse der gemerkten Linienpunkte
# bis ALIGN_RADIUS_M um den Bot, nur bei eindeutig laenglicher Wolke (Eigenwert-Verhaeltnis).
# Weicht der Bot mehr als ALIGN_MIN_DEG ab, dreht control_driving auf der Stelle, bis
# ALIGN_DONE_DEG erreicht ist oder ALIGN_TIMEOUT_S um sind; dann Lane. Anlass 12.11.: nach
# Obstacle stand der Bot 40 Grad schraeg, der PID fand die Spur nicht mehr.
ALIGN_RADIUS_M = 0.5
ALIGN_MIN_POINTS = 30
ALIGN_ELONGATION = 4.0
ALIGN_MIN_DEG = 15.0
ALIGN_DONE_DEG = 5.0
ALIGN_TIMEOUT_S = 5.0


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
    Button im Debugbild (/control/tunnel_request True, Quelle "manuell"), um die
    Challenge einzeln zu fahren. Der Bot faehrt mit Kamera auf die Oeffnung zu; vor dem
    Tunnel ist die Kamera erlaubt.

    TunnelEntry -> Tunnel, bevor der Bot drin ist: beide Spurlinien enden kurz vor dem
    Bot, denn sie reichen nur bis zur Oeffnung. Ab dann keine Kamera mehr. Durch die Tuer
    faehrt er nur mit LiDAR (tunnel_explorer_node).

    Tunnel -> Modus von vor dem Tunnel, sobald tunnel_explorer_node meldet, dass der Bot
    ausserhalb der Wand-Huelle ist. Beides jederzeit von Hand abbrechbar (False).

    Storage (Palette, Button im Debugbild auf /control/storage_request): den Ablauf
    fuehrt StorageSequence, sie setzt die Modi Lane, Stop und Maneuver. Solange
    sie laeuft, gibt es kein Obstacle (der LiDAR spielt dort keine Rolle; Schilder-
    pfosten oder Stuetzen loesten ihn sonst aus) und keinen Tunnel. Nach Abbruch gilt
    initial_mode: Lane fuer die Strecke, Stop fuer den Start mit storage.launch.py.

    In Obstacle und Tunnel faehrt ManeuverSequence auf Anfrage (tunnel_explorer_node bzw.
    nav2_goal_publisher) Grundbewegungen: Engstelle ausgerichtet durchfahren, freifahren. Der
    Modus bleibt dabei, im Tunnel also die Kamera aus.

    Lane -> Stop, wenn detect_lane BARRIER_CONFIRM Mal in Folge eine Schranke meldet
    (/detect/barrier); Stop -> Lane, wenn sie BARRIER_CLEAR_DEBOUNCE_S lang weg ist
    (Schranke offen). In Stop sendet control_driving Null.

    Lane -> Intersection, solange intersection_node /intersection/active True meldet
    (Haltelinie erreicht: halten, Vorfahrt pruefen, queren); False -> zurueck zu Lane.
    """

    def __init__(self):
        super().__init__('switch_control')

        # --- Parameter ---
        # Vor Ort einstellen, Werte nur in Switch_Control.yaml. Halbwinkel: der Kegel ist
        # doppelt so breit.
        self.declare_parameter('obstacle_check_angle_deg', Parameter.Type.DOUBLE, frange(1.0, 90.0))
        self.declare_parameter('obstacle_trigger_distance', Parameter.Type.DOUBLE, frange(0.05, 3.0))
        self.declare_parameter('obstacle_clear_distance', Parameter.Type.DOUBLE, frange(0.05, 3.0))
        # Obstacle ueberhaupt erlauben (rqt-Haekchen, Taste O im Not-Aus). follow_lane.sh setzt
        # false: dort laeuft kein Nav2, in Obstacle bliebe der Bot nur stehen. Abschalten in
        # Obstacle -> Lane.
        self.declare_parameter('obstacle_enabled', Parameter.Type.BOOL)
        self._param_names = ['obstacle_check_angle_deg', 'obstacle_trigger_distance',
                             'obstacle_clear_distance', 'obstacle_enabled']
        # Modus beim Start und nach einem Storage-Abbruch: Lane oder Stop.
        self.declare_parameter('initial_mode', Parameter.Type.STRING)
        require_params(self, self._param_names + ['initial_mode'])
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan_corrected', self.cb_scan, qos_profile_sensor_data)
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
        self.sub_barrier = self.create_subscription(
            Bool, '/detect/barrier', self.cb_barrier, 1)
        self.sub_intersection_active = self.create_subscription(
            Bool, '/intersection/active', self.cb_intersection_active, 1)

        # --- Publisher ---
        self.pub_control_mode = self.create_publisher(UInt8, '/control/mode', 1)
        self.pub_tunnel_source = self.create_publisher(String, '/control/tunnel_source', 1)
        # Zielkurs (rad, odom) im Modus Align, fuer control_driving.
        self.pub_align_yaw = self.create_publisher(Float64, '/control/align_yaw', 1)
        self.pub_obstacle_enabled = self.create_publisher(Bool, '/control/obstacle_enabled', 1)

        # --- Timer ---
        self.timer = self.create_timer(MODE_PUBLISH_PERIOD_S, self.cb_timer)

        # --- Zustand ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = self._initial_mode()
        self._mode_before_tunnel = None
        self._tunnel_source = ''          # 'auto' (Schild) oder 'manuell' (Button)
        self._obstacle_clear_since = None
        self._lines_lost_count = 0        # Bilder in Folge ohne Spurlinie (TunnelEntry)
        self._lines_msg_time = None       # letztes /detect/lines_lost (TunnelEntry)
        self._lane_points = None          # (N, 2) im Frame der Wolke
        self._align_yaw = None            # Zielkurs in odom (Modus Align)
        self._align_start = None          # Beginn des Ausrichtens (monotonic)
        self._lane_frame = None
        self._lane_time = None
        self._scanner_poses = {}          # (Ziel-Frame, Scanner-Frame) -> (x, y, yaw)
        # Brauchen tf_buffer, daher zuletzt.
        self.storage = StorageSequence(self)
        self.maneuvers = ManeuverSequence(self)
        self._barrier_hits = 0            # Schranken-Meldungen in Folge
        self._barrier_clear_since = None  # seit wann keine Schranke mehr (Stop)

        self.get_logger().info(f'Started switch_control Node ({self.control_mode.name}).')

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
        if self.control_mode != ControlType.TunnelEntry:
            return
        self._lines_msg_time = time.monotonic()
        self._lines_lost_count = self._lines_lost_count + 1 if msg.data else 0
        self._update_tunnel_entry()

    def cb_tunnel_outside(self, msg):
        if msg.data and self.control_mode == ControlType.Tunnel:
            self.get_logger().info('Bot ist aus dem Tunnelkasten hinausgefahren.')
            self._leave_tunnel(ControlType.Lane)

    def cb_barrier(self, msg):
        # Storage steuert die Modi selbst (auch Stop beim Lesen/Greifen/Ablegen, Lane beim
        # Erkunden); die Schilder sind blau wie die Schranke. "Schranke offen" wuerde sonst
        # mitten in Storage auf Lane schalten.
        if self.storage.active:
            self._barrier_hits = 0
            self._barrier_clear_since = None
            return
        self._barrier_hits = self._barrier_hits + 1 if msg.data else 0
        if self.control_mode == ControlType.Lane and self._barrier_hits >= BARRIER_CONFIRM:
            self._set_mode(ControlType.Stop, 'Schranke voraus')
        elif self.control_mode == ControlType.Stop:
            if msg.data:
                self._barrier_clear_since = None
            elif self._barrier_clear_since is None:
                self._barrier_clear_since = time.monotonic()
            elif time.monotonic() - self._barrier_clear_since >= BARRIER_CLEAR_DEBOUNCE_S:
                self._set_mode(ControlType.Lane, 'Schranke offen')

    def cb_intersection_active(self, msg):
        if self.storage.active:
            return
        if msg.data and self.control_mode == ControlType.Lane:
            self._set_mode(ControlType.Intersection, 'Haltelinie erreicht')
        elif not msg.data and self.control_mode == ControlType.Intersection:
            self._set_mode(ControlType.Lane, 'Kreuzung gequert')

    def cb_scan(self, scan):
        read_params(self, self._param_names)

        self._update_tunnel_entry()
        if self.control_mode == ControlType.Obstacle and not self.obstacle_enabled:
            self._set_mode(ControlType.Lane, 'Obstacle abgeschaltet (obstacle_enabled)')
        elif self.control_mode == ControlType.Lane and self.obstacle_enabled \
                and not self.storage.active:
            # Erst mit Spurgedaechtnis: ohne zaehlt jeder Punkt im Kegel, auch eine Wand
            # neben der Spur -- direkt nach dem Start loeste das Obstacle grundlos aus.
            if self._lane_time is None:
                return
            hit = self._obstacle_in_lane(scan, self.obstacle_check_angle_deg,
                                         self.obstacle_trigger_distance)
            if hit is not None:
                self._set_mode(ControlType.Obstacle,
                               f'Hindernis in der Fahrbahn ({hit[0]:+.0f} Grad, '
                               f'{hit[1] * 100:.0f} cm)')
        elif self.control_mode == ControlType.Obstacle:
            self._update_obstacle_exit(scan)
        elif self.control_mode == ControlType.Align:
            self._update_align()

    # --- Hilfsmethoden ---

    def _set_mode(self, new_mode, reason):
        if new_mode == self.control_mode:
            return
        self.get_logger().info(f'{reason} -> {self.control_mode.name} => {new_mode.name}.')
        if self.control_mode in SEQUENCE_MODES and new_mode not in SEQUENCE_MODES:
            self.maneuvers.cancel()
        self.control_mode = new_mode
        # Entprell-Zeitstempel gelten nur fuer den Modus, in dem sie gesetzt wurden.
        self._obstacle_clear_since = None
        self._barrier_clear_since = None
        # Sofort senden, nicht erst mit dem Timer: control_driving schaltet sonst
        # bis zu MODE_PUBLISH_PERIOD_S spaeter um.
        self._publish_mode()

    def _publish_mode(self):
        self.pub_control_mode.publish(UInt8(data=self.control_mode.value))
        self.pub_tunnel_source.publish(String(data=self._tunnel_source))
        if self.control_mode == ControlType.Align and self._align_yaw is not None:
            self.pub_align_yaw.publish(Float64(data=float(self._align_yaw)))
        # Frisch gelesen: Taste O / rqt wirken sonst erst mit dem naechsten Scan.
        self.pub_obstacle_enabled.publish(
            Bool(data=bool(self.get_parameter('obstacle_enabled').value)))

    def set_initial_mode(self, reason):
        """Zurueck auf initial_mode (nach einem Storage-Abbruch)."""
        self._set_mode(self._initial_mode(), reason)

    def _initial_mode(self):
        name = self.get_parameter('initial_mode').value
        if name not in ('Lane', 'Stop'):
            self.get_logger().warn(f'initial_mode "{name}" unbekannt -- nehme Lane.')
            return ControlType.Lane
        return ControlType[name]

    def _enter_tunnel(self, source):
        if self.control_mode in TUNNEL_MODES:
            return
        if self.storage.active:
            self.get_logger().warn(f'Tunnel ({source}) ignoriert: Storage laeuft.')
            return
        # Ausrichten ist nur ein Uebergang: danach gilt Lane.
        self._mode_before_tunnel = ControlType.Lane if self.control_mode == ControlType.Align \
            else self.control_mode
        self._tunnel_source = source
        self._lines_lost_count = 0
        self._lines_msg_time = time.monotonic()
        self._set_mode(ControlType.TunnelEntry, f'Tunnel ausgeloest ({source})')

    def _update_tunnel_entry(self):
        """LINES_LOST_FRAMES Bilder in Folge ohne Spurlinie -> Tunnel. Ohne neue Bilder
        (LINES_NO_IMAGE_S) gilt die Spur ebenfalls als zu Ende."""
        if self.control_mode != ControlType.TunnelEntry:
            return
        if self._lines_lost_count >= LINES_LOST_FRAMES:
            self._set_mode(ControlType.Tunnel,
                           f'Spurlinien enden ({self._lines_lost_count} Bilder ohne Linie)')
        elif time.monotonic() - self._lines_msg_time >= LINES_NO_IMAGE_S:
            self._set_mode(ControlType.Tunnel,
                           f'Spurlinien enden (seit {LINES_NO_IMAGE_S:.1f} s kein Bild)')

    def _leave_tunnel(self, mode=None):
        """Abbruch von Hand: zurueck in den Modus von vor dem Tunnel. Hinausgefahren (mode=Lane):
        immer Follow Lane, auch wenn der Tunnel aus Obstacle heraus ausgeloest wurde -- Obstacle
        faehrt ohne Spurmitte nur geradeaus; ein Hindernis in der Fahrbahn schaltet aus Lane
        ohnehin wieder auf Obstacle."""
        restored = mode or self._mode_before_tunnel or ControlType.Lane
        self._mode_before_tunnel = None
        self._tunnel_source = ''
        self._set_mode(restored, 'Tunnel verlassen')

    def _update_obstacle_exit(self, scan):
        hit = self._obstacle_in_lane(scan, OBSTACLE_CLEAR_ANGLE_DEG, self.obstacle_clear_distance)
        if hit is not None:
            self._obstacle_clear_since = None
            self.get_logger().info(
                f'Obstacle bleibt: Punkt bei {hit[0]:+.0f} Grad, {hit[1] * 100:.0f} cm',
                throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()
        if self._obstacle_clear_since is None:
            self._obstacle_clear_since = now
        elif (now - self._obstacle_clear_since).nanoseconds / 1e9 >= OBSTACLE_CLEAR_DEBOUNCE_S:
            self._leave_obstacle()

    def _leave_obstacle(self):
        """Obstacle -> Lane, vorher auf die Spurrichtung drehen, falls der Bot schraeg steht."""
        robot = self._robot_pose()
        lane_dir = self._lane_direction()
        if robot is None or lane_dir is None:
            self._set_mode(ControlType.Lane, 'Kein Hindernis mehr in der Naehe (Spurrichtung '
                                             'unbekannt, ohne Ausrichten)')
            return
        if abs(math.degrees(lane_dir)) <= ALIGN_MIN_DEG:
            self._set_mode(ControlType.Lane, 'Kein Hindernis mehr in der Naehe')
            return
        self._align_yaw = robot[2] + lane_dir
        self._align_start = time.monotonic()
        self._set_mode(ControlType.Align, f'Kein Hindernis mehr, Bot {math.degrees(lane_dir):+.0f} '
                                          'Grad zur Spur -- erst ausrichten')

    def _update_align(self):
        robot = self._robot_pose()
        waited = time.monotonic() - self._align_start
        if robot is not None:
            err = math.degrees(math.atan2(math.sin(self._align_yaw - robot[2]),
                                          math.cos(self._align_yaw - robot[2])))
            if abs(err) <= ALIGN_DONE_DEG:
                self._set_mode(ControlType.Lane, f'Ausgerichtet (Rest {err:+.1f} Grad)')
                return
        if waited >= ALIGN_TIMEOUT_S:
            self._set_mode(ControlType.Lane, f'Ausrichten nach {ALIGN_TIMEOUT_S:.0f} s abgebrochen')

    def _lane_direction(self):
        """Winkel (rad) der Spur zur Fahrtrichtung des Bots, -90..+90 Grad: Hauptachse der
        gemerkten Linienpunkte (base_footprint) bis ALIGN_RADIUS_M. None ohne frische Punkte
        oder wenn die Wolke nicht eindeutig laenglich ist (Kreuzung, Querlinien)."""
        lanes = self._fresh_lane_points()
        if lanes is None or self._lane_frame != 'base_footprint':
            return None
        near = lanes[np.hypot(lanes[:, 0], lanes[:, 1]) <= ALIGN_RADIUS_M]
        if len(near) < ALIGN_MIN_POINTS:
            return None
        c = near - near.mean(axis=0)
        values, vectors = np.linalg.eigh(c.T @ c)
        if values[1] < ALIGN_ELONGATION * max(values[0], 1e-12):
            self.get_logger().info('Spurrichtung nicht eindeutig (Linien nicht parallel) -- '
                                   'kein Ausrichten.')
            return None
        axis = vectors[:, -1]
        if axis[0] < 0.0:
            axis = -axis
        return math.atan2(axis[1], axis[0])

    def _robot_pose(self):
        """(x, y, yaw) von base_footprint in odom, None ohne TF."""
        try:
            tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Keine TF odom <- base_footprint: {e}',
                                   throttle_duration_sec=5.0)
            return None
        return pose2d_from_transform(tf)

    def _obstacle_in_lane(self, scan, half_angle_deg, distance):
        """Naechster LiDAR-Punkt im Bereich +/-half_angle_deg naeher als distance, der
        nicht hinter einer Spurlinie liegt, als (Winkel in Grad, Abstand in m); None ohne."""
        angles, ranges, points = scan_points(scan)
        near = (np.abs(angles) <= math.radians(half_angle_deg)) & (ranges < distance)
        if not near.any():
            return None

        lanes = self._fresh_lane_points()
        scanner = self._scanner_pose(self._lane_frame, scan.header.frame_id) \
            if lanes is not None else None
        hit = np.flatnonzero(near)
        if scanner is not None:
            candidates = transform_points(scanner, points[near])
            hit = hit[~self._behind_line(candidates, lanes)]
            if hit.size == 0:
                return None
        i = hit[np.argmin(ranges[hit])]
        return math.degrees(angles[i]), float(ranges[i])

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
        self.get_logger().info('Shutting down switch_control Node -- Magnet aus.')
        self.storage.shut_down()


def main(args=None):
    run_node(switch_control, args)


if __name__ == '__main__':
    main()
