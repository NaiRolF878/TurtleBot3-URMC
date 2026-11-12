"""Gemeinsame Bausteine der Follow_Lane-Nodes.

Fahrmodi (ControlType), Parameter-Helfer, Geometrie (Gierwinkel, BEV-Pixel ->
Meter) und der einheitliche Ablauf von main(): starten, spinnen, bei
SIGINT/SIGTERM shut_down().
"""
import math
import signal
import threading
from enum import Enum

import numpy as np
import rclpy
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.exceptions import ParameterUninitializedException
from rclpy.signals import SignalHandlerOptions

# Wartezeit von spin_once; so schnell reagiert ein Node auf Strg+C.
SPIN_TIMEOUT_S = 0.1
# Kostenwert der OccupancyGrid-Skala: ab hier plant NavFn nicht mehr (253 -> 99).
COSTMAP_BLOCKED = 99
# Groesser als robot_radius + footprint_padding der global_costmap (0,10 m): so weit reicht
# der Sperrsaum der Wand, an der der Bot steht.
PATH_CHECK_SKIP_M = 0.125


class ControlType(Enum):
    """Fahrmodus auf /control/mode (std_msgs/UInt8), gesetzt nur von switch_control."""
    Lane = 1
    Obstacle = 2
    Stop = 3            # Schranke (switch_control)
    Intersection = 4    # Halt an der Haltelinie + Queren (intersection_node)
    Align = 5           # noch ohne Ausloeser
    Tunnel = 6          # im Tunnel: Kamera verboten (Reglement)
    TunnelEntry = 7     # Einfahrt bis kurz vor die Oeffnung, Kamera noch erlaubt


class SignType(Enum):
    """Erkanntes Schild auf /detect/sign (std_msgs/UInt8), unabhaengig von ControlType.

    Welcher Modus aus einem Schild folgt, entscheidet switch_control.
    """
    Tunnel = 6


def frange(lo, hi):
    """Wertebereich eines Fliesskomma-Parameters (Schieberegler in rqt).

    step=0.0: rclpy prueft sonst (wert - from) % step == 0, was mit Fliesskomma
    zufaellig fehlschlaegt.
    """
    return ParameterDescriptor(floating_point_range=[
        FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])


def irange(lo, hi):
    """Wertebereich eines Ganzzahl-Parameters."""
    return ParameterDescriptor(integer_range=[
        IntegerRange(from_value=lo, to_value=hi, step=1)])


def read_params(node, names):
    """Parameter als node.<name> bereitstellen.

    declare_parameter legt keine Attribute an. Bei jedem Aufruf neu gelesen,
    wirken Aenderungen aus rqt sofort.
    """
    for name in names:
        setattr(node, name, node.get_parameter(name).value)


def require_params(node, names):
    """Beendet den Node mit klarer Meldung, wenn ein Parameter ohne Code-Default keinen Wert
    aus der YAML bekommen hat. So gibt es jeden Wert nur an einer Stelle."""
    missing = []
    for name in names:
        try:
            node.get_parameter(name)
        except ParameterUninitializedException:
            missing.append(name)
    if missing:
        node.get_logger().fatal(
            f'Ohne Wert: {", ".join(missing)} -- YAML nicht geladen oder unvollstaendig. '
            f'{node.get_name()} startet nicht.')
        raise SystemExit(1)


def control_mode_from_msg(node, msg):
    """UInt8 von /control/mode -> ControlType, None bei unbekanntem Wert."""
    try:
        return ControlType(msg.data)
    except ValueError:
        node.get_logger().warn(f'Unbekannter Control-Mode empfangen: {msg.data}',
                               throttle_duration_sec=5.0)
        return None


def yaw_from_quaternion(q):
    """Gierwinkel (rad, positiv = nach links) aus einem geometry_msgs-Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    """Winkel (rad) auf [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def pose2d_from_transform(tf):
    """(x, y, yaw) aus einem geometry_msgs/TransformStamped; z, Roll und Nick entfallen."""
    t = tf.transform.translation
    return (t.x, t.y, yaw_from_quaternion(tf.transform.rotation))


def transform_points(pose, xy):
    """Punkte (N, 2) aus dem Kind- in den Eltern-Frame von pose = (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x + c * xy[:, 0] - s * xy[:, 1], y + s * xy[:, 0] + c * xy[:, 1]])


def inverse_transform_points(pose, xy):
    """Gegenstueck zu transform_points: Punkte aus dem Eltern- in den Kind-Frame."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = xy[:, 0] - x, xy[:, 1] - y
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


def scan_points(scan):
    """Gueltige LiDAR-Messungen als (Winkel [-pi, pi], Entfernung, Punkte (N, 2)) im
    Frame des Scanners. inf/nan (kein Echo) und Werte unter range_min fallen weg."""
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    # Auf [-pi, pi] normalisieren, damit 0 rad wirklich geradeaus ist.
    angles = np.arctan2(np.sin(angles), np.cos(angles))
    valid = np.isfinite(ranges) & (ranges > scan.range_min)
    angles, ranges = angles[valid], ranges[valid]
    return angles, ranges, np.column_stack([ranges * np.cos(angles), ranges * np.sin(angles)])


def path_blocked(points, robot_xy, grid, resolution, origin_xy):
    """Liegt eine gesperrte Zelle auf dem Pfad (N, 2) vor dem Bot?

    grid: OccupancyGrid-Werte (H, W), origin_xy im selben Frame wie points. Der Abschnitt bis
    PATH_CHECK_SKIP_M um den Bot zaehlt nicht: steht er nah an einer Wand, beginnt der Pfad
    im Saum, und die Pruefung sagte sonst dauernd "blockiert".
    """
    d = np.hypot(points[:, 0] - robot_xy[0], points[:, 1] - robot_xy[1])
    ahead = points[int(np.argmin(d)):]
    ahead = ahead[np.hypot(ahead[:, 0] - robot_xy[0], ahead[:, 1] - robot_xy[1]) > PATH_CHECK_SKIP_M]
    cells = np.floor((ahead - np.asarray(origin_xy)) / resolution).astype(np.int64)
    h, w = grid.shape
    valid = (cells[:, 0] >= 0) & (cells[:, 0] < w) & (cells[:, 1] >= 0) & (cells[:, 1] < h)
    return bool((grid[cells[valid, 1], cells[valid, 0]] >= COSTMAP_BLOCKED).any())


def cloud_xy(cloud):
    """x/y einer mit point_cloud2.create_cloud_xyz32 erzeugten PointCloud2 als (N, 2)."""
    if cloud.point_step != 12 or [f.name for f in cloud.fields] != ['x', 'y', 'z']:
        raise ValueError('PointCloud2 ist nicht xyz32')
    pts = np.frombuffer(bytes(cloud.data), dtype=np.float32).reshape(-1, 3)
    return pts[:, :2].astype(np.float64)


def bev_pixels_to_ground(bev, us, vs):
    """BEV-Pixel (Spalte u, Zeile v) -> Boden in Metern, base_footprint.

    bev aus bev_geometry.load_bev_spec. Gibt (X vorwaerts ab Radachse, Y nach
    links) zurueck, gemessen an der Pixelmitte wie in Parameter/bev.yaml.
    """
    us = np.asarray(us, dtype=np.float64)
    vs = np.asarray(vs, dtype=np.float64)
    return bev['x_max'] - (vs + 0.5) * bev['res'], bev['y_half'] - (us + 0.5) * bev['res']


def stop_on_signals():
    """threading.Event, das SIGINT und SIGTERM setzen.

    rclpy.init laeuft dazu mit SignalHandlerOptions.NO: so ist der ROS-Context
    beim Beenden noch gueltig und shut_down() kann z.B. den Nullbefehl senden.
    SIGTERM kommt von ros2 launch, wenn SIGINT nicht reicht.
    """
    stop = threading.Event()

    def handler(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    return stop


def spin_until(node, stop):
    # spin_once mit Timeout statt spin(): spin() blockiert in C bis zur naechsten
    # Nachricht, ein gesetztes Stop-Event kaeme ohne Daten nie an.
    while rclpy.ok() and not stop.is_set():
        rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)


def run_node(node_class, args=None):
    """main() aller Nodes ausser notaus: spinnen bis SIGINT/SIGTERM, dann
    node.shut_down() und aufraeumen."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = stop_on_signals()
    node = node_class()
    try:
        spin_until(node, stop)
    finally:
        node.shut_down()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
