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
from rclpy.signals import SignalHandlerOptions

# Wartezeit von spin_once; so schnell reagiert ein Node auf Strg+C.
SPIN_TIMEOUT_S = 0.1


class ControlType(Enum):
    """Fahrmodus auf /control/mode (std_msgs/UInt8), gesetzt nur von switch_control."""
    Lane = 1
    Obstacle = 2
    Stop = 3            # noch ohne Ausloeser
    Intersection = 4    # noch ohne Ausloeser
    Align = 5           # noch ohne Ausloeser
    Tunnel = 6          # im Tunnel: Kamera verboten (Reglement)
    TunnelEntry = 7     # Einfahrt bis kurz vor die Oeffnung, Kamera noch erlaubt


class SignType(Enum):
    """Erkanntes Schild auf /detect/sign (std_msgs/UInt8), unabhaengig von ControlType.

    Welcher Modus aus einem Schild folgt, entscheidet switch_control. Weitere
    Schilder hier eintragen, sobald bekannt ist, wie sie aussehen. Tunnel = 6, damit
    der Testbefehl "ros2 topic pub /detect/sign std_msgs/msg/UInt8 '{data: 6}'"
    gueltig bleibt.
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
