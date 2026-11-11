"""Gemeinsame Bausteine der Follow_Lane-Nodes.

Fahrmodi (ControlType), Storage-Zustaende (StorageState), Parameter-Helfer (auch
Parameter eines anderen Nodes live mitlesen: SharedParams), CLAHE,
Geometrie (Gierwinkel, BEV-Pixel -> Meter) und der einheitliche Ablauf von main():
starten, spinnen, bei SIGINT/SIGTERM shut_down().
"""
import math
import signal
import threading
from enum import Enum

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import (
    FloatingPointRange, IntegerRange, ParameterDescriptor, ParameterEvent)
from rcl_interfaces.srv import GetParameters
from rclpy.exceptions import ParameterUninitializedException
from rclpy.parameter import parameter_value_to_python
from rclpy.signals import SignalHandlerOptions

# Wartezeit von spin_once; so schnell reagiert ein Node auf Strg+C.
SPIN_TIMEOUT_S = 0.1
# Kostenwert der OccupancyGrid-Skala: ab hier plant NavFn nicht mehr (253 -> 99).
COSTMAP_BLOCKED = 99
# Groesser als robot_radius + footprint_padding der global_costmap (0,10 m): so weit reicht
# der Sperrsaum der Wand, an der der Bot steht.
PATH_CHECK_SKIP_M = 0.125
# Eigener Bot vorn im Bild (Magnet, Halter), wie er in der BEV erscheint: bis SELF_MASK_X_M vor
# der Radachse und +-SELF_MASK_Y_M nie Linie oder Palette (detect_lane, storage_zone_node). In der
# BEV wirkt der Magnet weiter vorn, als er ist (er sitzt 7,5 cm hoch, die BEV gilt fuer den Boden).
SELF_MASK_X_M = 0.11
SELF_MASK_Y_M = 0.05
# Magnet (gemessen): Radachse bis Magnetende 7,5 cm, Magnetmitte 7,5 cm ueber dem Boden.
MAGNET_FRONT_M = 0.075
MAGNET_HEIGHT_M = 0.075
# Breite des Burger ueber die Raeder (m), wie der Footprint in nav2_params.yaml; gemeinsam
# fuer Tunnel (Durchfahrt) und Storage (kleinste moegliche Zone).
BOT_WIDTH = 0.178


class ControlType(Enum):
    """Fahrmodus auf /control/mode (std_msgs/UInt8), gesetzt nur von switch_control."""
    Lane = 1
    Obstacle = 2
    Stop = 3            # Nullbefehl; Storage beim Lesen, Greifen, Ablegen
    Intersection = 4    # noch ohne Ausloeser
    Align = 5           # noch ohne Ausloeser
    Tunnel = 6          # im Tunnel: Kamera verboten (Reglement)
    TunnelEntry = 7     # Einfahrt bis kurz vor die Oeffnung, Kamera noch erlaubt
    # 8 war Dock (Storage-Feinanfahrt im Kamerabild), ersetzt durch Maneuver geradeaus
    Maneuver = 9        # Storage: Grundbewegung in odom (/control/maneuver)


class StorageState(Enum):
    """Unterzustand der Storage-Aufgabe auf /control/storage_state (std_msgs/String,
    Name des Werts), gesetzt nur von switch_control. Die Detektoren arbeiten nur in
    den Zustaenden, die sie brauchen."""
    IDLE = 0
    ERKUNDEN = 1             # an der Spur entlang, Zonengrenzen sammeln
    ZUR_ZONE_DREHEN = 2      # zur Zonenmitte auf der Spur, dann zur Zone drehen
    SCHILD_LESEN = 3         # stehen, Schild, Palette und Zonentiefe abstimmen
    ZURUECKDREHEN = 4        # zurueck in Fahrtrichtung der Spur
    ZUR_PALETTENZONE = 5     # auf der Spur vor die Palettenzone, zur Zone drehen
    POSITIONIERT = 6         # anhalten vor der Palettenzone (Wertung), Palette vermessen
    ANDOCKEN = 7             # rechtwinklig und gerade bis an die Palette
    GREIFEN = 8              # Magnet an, warten
    GRIFF_PRUEFEN = 9        # kurz zuruecksetzen, kommt die Palette mit?
    NEU_ANSETZEN = 10        # zurueck auf die Spur, erneut ausrichten
    HERAUSFAHREN = 11        # mit Palette rueckwaerts auf die Spur
    TRANSPORT = 12           # auf der Spur vor die Zielzone, zur Zone drehen
    ABLAGE_ANFAHREN = 13     # bis zur Zonenmitte
    ABLEGEN = 14             # Magnet aus, warten
    VERLASSEN = 15           # rueckwaerts auf die Spur
    FERTIG = 16
    FEHLER = 17
    AUSRICHTEN = 18          # rechtwinklig vor die Palette (Vor-Andock-Pose)


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


# Farben gibt es nur einmal: detect_lane haelt sie (Follow_Lane_ROI_LAB.yaml, rqt), andere
# Nodes lesen sie mit SharedParams mit.
COLOR_SOURCE = 'detect_lane'
WHITE_PARAMS = ['lab_l_white_l', 'lab_l_white_h', 'lab_a_white_l', 'lab_a_white_h',
                'lab_b_white_l', 'lab_b_white_h']
CLAHE_PARAMS = ['clahe_clip_limit', 'clahe_tile_grid_size']


class SharedParams:
    """Parameter eines anderen Nodes live mitlesen: einmal per get_parameters holen (bis es
    klappt, alle RETRY_S), danach jede Aenderung aus /parameter_events uebernehmen. So gibt
    es jeden Wert nur an einer Stelle; was in rqt beim Quell-Node eingestellt wird, gilt
    sofort auch hier. Werte: shared.values[name]; erst verwenden, wenn shared.ready."""

    RETRY_S = 1.0

    def __init__(self, node, source, names):
        self.node = node
        self.source = source.strip('/')
        self.names = list(names)
        self.values = {}
        self._future = None
        self._client = node.create_client(GetParameters, f'/{self.source}/get_parameters')
        node.create_subscription(ParameterEvent, '/parameter_events', self._on_event, 10)
        self._timer = node.create_timer(self.RETRY_S, self._request)
        self._request()

    @property
    def ready(self):
        return all(n in self.values for n in self.names)

    def __getitem__(self, name):
        return self.values[name]

    def _request(self):
        if self.ready:
            self._timer.cancel()
            return
        if self._future is not None and not self._future.done():
            return
        if not self._client.service_is_ready():
            self.node.get_logger().warn(f'Warte auf die Parameter von /{self.source} ...',
                                        throttle_duration_sec=5.0)
            return
        self._future = self._client.call_async(GetParameters.Request(names=self.names))
        self._future.add_done_callback(self._on_response)

    def _on_response(self, future):
        for name, value in zip(self.names, future.result().values):
            value = parameter_value_to_python(value)
            if value is not None:
                self.values[name] = value
        if self.ready:
            self.node.get_logger().info(f'Parameter von /{self.source} uebernommen: '
                                        + ', '.join(f'{n}={self.values[n]}' for n in self.names))

    def _on_event(self, event):
        if event.node.strip('/') != self.source:
            return
        was_ready = self.ready
        for p in list(event.new_parameters) + list(event.changed_parameters):
            if p.name in self.names:
                self.values[p.name] = parameter_value_to_python(p.value)
                if was_ready:   # beim Start des Quell-Nodes kommen alle einzeln -- nicht melden
                    self.node.get_logger().info(f'{p.name}={self.values[p.name]} von /{self.source}')
        if not was_ready and self.ready:
            self.node.get_logger().info(f'Parameter von /{self.source} uebernommen.')


def lab_clahe(image, clip_limit, tile_grid_size):
    """BGR -> LAB, CLAHE auf dem L-Kanal (gleicht ueber- und unterbelichtete Bereiche an)."""
    l_channel, a_channel, b_channel = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2LAB))
    tile = max(1, int(tile_grid_size))
    clahe = cv2.createCLAHE(clipLimit=float(clip_limit), tileGridSize=(tile, tile))
    return cv2.merge((clahe.apply(l_channel), a_channel, b_channel))


def white_in(lab, shared):
    """Weiss-Maske aus einem LAB-Bild mit den gemeinsamen Weiss-Werten (SharedParams)."""
    return cv2.inRange(lab, tuple(int(shared[n]) for n in WHITE_PARAMS[0::2]),
                       tuple(int(shared[n]) for n in WHITE_PARAMS[1::2]))


def control_mode_from_msg(node, msg):
    """UInt8 von /control/mode -> ControlType, None bei unbekanntem Wert."""
    try:
        return ControlType(msg.data)
    except ValueError:
        node.get_logger().warn(f'Unbekannter Control-Mode empfangen: {msg.data}',
                               throttle_duration_sec=5.0)
        return None


def storage_state_from_msg(msg):
    """String von /control/storage_state -> StorageState, IDLE bei unbekanntem Namen."""
    return StorageState.__members__.get(msg.data, StorageState.IDLE)


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


def ground_to_bev_pixels(bev, xs, ys):
    """Gegenstueck zu bev_pixels_to_ground: Boden (base_footprint) -> (u, v) als float."""
    xs = np.asarray(xs, dtype=np.float64)
    ys = np.asarray(ys, dtype=np.float64)
    return (bev['y_half'] - ys) / bev['res'] - 0.5, (bev['x_max'] - xs) / bev['res'] - 0.5


# Zonen im Debugbild: Kasten ab side_near_m (Abstand Spur -> Zonen) so tief in die Zone.
ZONE_BOX_DEPTH_M = 0.25
# Gesehene Querlinien eines Bildes: naeher als das (m, entlang der Fahrtrichtung) = dieselbe Linie
# (beide Kanten eines Klebestreifens, zerrissene Segmente).
SEEN_LINE_MERGE_M = 0.05
SEEN_COLOR = (0, 200, 0)        # BGR: aus dem Bild gesehene Zone
MODEL_COLOR = (0, 140, 255)     # BGR: Zonenmodell (Umriss)


def _line_at(center, direction, lateral):
    """Punkt der Querlinie (Mitte, Richtung) mit seitlichem Abstand lateral (Y, Bot-Frame)."""
    (x, y), (dx, dy) = center, direction
    if abs(dy) < 1e-6:
        return x, lateral
    t = (lateral - y) / dy
    return x + t * dx, lateral


def seen_zone_boxes(lines, near_m, side_sign, depth_m=ZONE_BOX_DEPTH_M):
    """Zonen, wie sie in EINEM Bild zu sehen sind: Kasten zwischen je zwei benachbarten
    Querlinien, ab near_m neben der Spur depth_m tief. lines: ((x, y), (dx, dy)) im Bot-Frame
    (base_footprint) zum Bildzeitpunkt. Keine Odometrie, kein Modell. side_sign: Y-Vorzeichen
    der Zonenseite. Liste von Ecken (4, 2) in m, nach x sortiert."""
    merged = []
    for (x, y), (dx, dy) in sorted(lines, key=lambda l: l[0][0]):
        n = math.hypot(dx, dy) or 1.0
        if merged and x - merged[-1][-1][0][0] <= SEEN_LINE_MERGE_M:
            merged[-1].append(((x, y), (dx / n, dy / n)))
        else:
            merged.append([((x, y), (dx / n, dy / n))])
    mids = []
    for group in merged:
        cx = sum(p[0][0] for p in group) / len(group)
        cy = sum(p[0][1] for p in group) / len(group)
        dx = sum(p[1][0] for p in group)
        dy = sum(p[1][1] for p in group)
        mids.append(((cx, cy), (dx, dy)))
    near, far = side_sign * near_m, side_sign * (near_m + depth_m)
    boxes = []
    for a, b in zip(mids, mids[1:]):
        boxes.append(np.array([_line_at(*a, near), _line_at(*a, far),
                               _line_at(*b, far), _line_at(*b, near)]))
    return boxes


def zone_digit_at(zones, pose, xy):
    """Ziffer der Modellzone, in der der Punkt xy (Bot-Frame) liegt; 0 ohne Modell/Treffer.
    Die Modelllinien liegen auf der Spur, entlang der Spurrichtung wird verglichen."""
    if zones is None or len(zones.lines) < 2 or pose is None:
        return 0
    p = transform_points(pose, np.array([xy]))[0]
    pts = np.array([(l.position.x, l.position.y) for l in zones.lines])
    u = pts[-1] - pts[0]
    norm = np.linalg.norm(u)
    if norm < 1e-6:
        return 0
    u /= norm
    s = (pts - pts[0]) @ u
    sp = float((p - pts[0]) @ u)
    for k in range(len(s) - 1):
        if s[k] <= sp < s[k + 1]:
            return int(zones.digits[k]) if k < len(zones.digits) else 0
    return 0


def zone_boxes(zones, pose, near_m, depth_m=ZONE_BOX_DEPTH_M):
    """Zonenmodell (StorageZones, odom) als Vierecke im Bot-Frame (base_footprint).
    Je Zone: (Ecken (4, 2) in m, Mitte (x, y), Ziffer (0 = unbekannt), beide Linien gemessen)."""
    if zones is None or len(zones.lines) < 2 or pose is None:
        return []
    pts = inverse_transform_points(
        pose, np.array([(p.position.x, p.position.y) for p in zones.lines]))
    yaws = [yaw_from_quaternion(p.orientation) - pose[2] for p in zones.lines]
    boxes = []
    for k in range(len(pts) - 1):
        corners = np.array([
            (pts[i][0] + d * math.cos(yaws[i]), pts[i][1] + d * math.sin(yaws[i]))
            for i, d in ((k, near_m), (k, near_m + depth_m),
                         (k + 1, near_m + depth_m), (k + 1, near_m))])
        digit = zones.digits[k] if k < len(zones.digits) else 0
        measured = bool(zones.measured[k] and zones.measured[k + 1])
        boxes.append((corners, corners.mean(axis=0), int(digit), measured))
    return boxes


def _bev_poly(bev, corners):
    u, v = ground_to_bev_pixels(bev, corners[:, 0], corners[:, 1])
    return np.column_stack([u, v]).round().astype(np.int32)


def _label(view, bev, center, text, scale, thickness):
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    u, v = ground_to_bev_pixels(bev, center[0], center[1])
    org = (int(round(float(u))) - tw // 2, int(round(float(v))) + th // 2)
    cv2.putText(view, text, org, font, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
    cv2.putText(view, text, org, font, scale, (255, 255, 255), thickness, cv2.LINE_AA)


def draw_seen_zones(view, bev, boxes):
    """Aus dem Bild gesehene Zonen: gruener, halbdurchsichtiger Kasten, Ziffer in der Mitte.
    boxes: (Ecken (4, 2) m, Ziffer (0 = ?))."""
    if not boxes:
        return
    overlay = view.copy()
    polys = [_bev_poly(bev, corners) for corners, _ in boxes]
    for poly in polys:
        cv2.fillPoly(overlay, [poly], SEEN_COLOR)
    cv2.addWeighted(overlay, 0.3, view, 0.7, 0, dst=view)
    for poly, (corners, digit) in zip(polys, boxes):
        cv2.polylines(view, [poly], True, SEEN_COLOR, 2, cv2.LINE_AA)
        _label(view, bev, corners.mean(axis=0), str(digit) if digit else '?', 1.4, 3)


def draw_model_zones(view, bev, boxes):
    """Zonenmodell nur als duenner Umriss (orange), kleine Ziffer -- zum Vergleich mit den
    gesehenen Zonen. boxes aus zone_boxes()."""
    for corners, center, digit, _ in boxes:
        cv2.polylines(view, [_bev_poly(bev, corners)], True, MODEL_COLOR, 1, cv2.LINE_AA)
        _label(view, bev, center, str(digit) if digit else '?', 0.5, 1)


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
