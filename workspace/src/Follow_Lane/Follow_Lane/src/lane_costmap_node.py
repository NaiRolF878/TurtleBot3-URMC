"""Spurgedaechtnis: Spurlinien als virtuelle Waende fuer Nav2, Spurmitte als Pfad.

Ein:  /detect/lane_mask (LaneMask, BEV-Pixel), /lokal/bev_view (Sichtmaske), /scan,
      /control/mode, /control/clear_maps (Empty, Taste L im Not-Aus)
Aus:  /detect/lane_obstacles (PointCloud2 aller gemerkten Linienzellen, fuer die
      lane_layer der Costmaps und den Linienfilter von switch_control),
      /lane/center_path (Path im odom-Frame, nur in Obstacle)
Im Tunnel nichts ausser einer leeren Wolke beim Eintritt: die Kamera ist dort laut
Reglement verboten.
"""
import math
import os
import time
from collections import deque

import cv2
import numpy as np
import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, Header, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import LaneMask
from Follow_Lane.src import bev_geometry
from Follow_Lane.src.common import (
    ControlType, bev_pixels_to_ground, control_mode_from_msg, frange, inverse_transform_points,
    irange, pose2d_from_transform, read_params, require_params, run_node, scan_points, transform_points)

EXTRINSICS = os.path.join('camera_info', 'extrinsics_800x600.yaml')
ROBOT_FRAME = 'base_footprint'
ODOM_FRAME = 'odom'
LANE_OBSTACLES_TOPIC = '/detect/lane_obstacles'
CENTER_PATH_TOPIC = '/lane/center_path'
PIXEL_STRIDE = 2
# Zellgroesse (m), = resolution beider Costmaps.
MEMORY_RESOLUTION = 0.01
# Weiter entfernte Zellen bleiben gemerkt, werden aber nicht gesendet; = obstacle_max_range
# der lane_layer in nav2_params.yaml.
MEMORY_RADIUS = 2.0
# Jeder LiDAR-Punkt verdeckt fuer die Kamera einen Keil dieser halben Breite (m); Zellen bis
# SHADOW_MARGIN vor dem Punkt zaehlen schon zum Hindernis.
SHADOW_RADIUS = 0.03
SHADOW_MARGIN = 0.05
# Linienverfolgung: Schrittweite (m), groesste Drehung je Schritt (Grad, 35 Grad auf 3 cm folgt
# Kurven bis r ~5 cm), groesste ueberbrueckte Luecke (m). Knickt eine Linie gegen die andere um
# mehr als TRACE_PARALLEL_DEG ab, endet sie dort (Querlinie in L-Form).
TRACE_STEP = 0.03
TRACE_MAX_TURN_DEG = 35.0
TRACE_MAX_GAP = 0.09
TRACE_PARALLEL_DEG = 60.0
# Tiefe eines Streifens der Spurmitte (m), zugleich Abstand der Posen im Pfad.
CENTER_BIN_SIZE = 0.05
# Mindestzahl Mittelpunkte, sonst wird kein Pfad gesendet.
CENTER_MIN_POINTS = 3
# Ueber so viele Bilder wird die gemessene Spurbreite (Linienmitte zu Linienmitte) gemittelt
# (Median); sie gilt, wenn ein Bild nur eine Linie sieht. Kurz genug, dass der Wechsel von der
# 20-cm-Spur in den breiteren Hindernisbereich nach wenigen Bildern nachgezogen ist.
CENTER_WIDTH_HISTORY = 20
CENTER_PATH_MODES = (ControlType.Obstacle,)
# LiDAR-Scan aelter als das (s) taugt nicht mehr fuer den Sichtschatten.
SCAN_MAX_AGE_S = 0.5
# So lange (s) nach der leeren Wolke werden die Costmaps geleert: bis dahin ist sie in
# beiden lane_layer angekommen.
COSTMAP_CLEAR_DELAY_S = 0.3

# Linienverfolgung (m): Startpunkt im bot-nahen Streifen dieser Tiefe; Gruppen darin
# trennt eine Luecke ueber SEED_GAP. Breiter als SEED_MAX_WIDTH ist keine Linie
# allein (Querlinie haengt daran): dann zaehlt nur das aeussere Ende.
SEED_DEPTH = 0.08
SEED_GAP = 0.02
SEED_MAX_WIDTH = 0.08
# Startpunkt des letzten Bilds: Gruppen bis so weit davon werden bevorzugt.
SEED_HINT_TOL = 0.04
SEED_HINT_MAX_AGE_S = 0.5
# Verfolgt wird auf einem ausgeduennten Raster (schneller, Median bleibt gleich).
TRACE_THIN_CELL = 0.01
TRACE_MIN_POINTS = 5
# Anfangsrichtung aus den Linienpunkten in diesem Umkreis (m) um den Startpunkt.
TRACE_START_RADIUS = 0.10
# ... aber nur, wenn die Punkte dort so viel laenger als breit sind (Verhaeltnis der
# Eigenwerte); sonst geradeaus.
TRACE_START_ELONGATION = 4.0
TRACE_MAX_LENGTH = 1.5
NEAR_RES = 0.005
# Winkelraster (rad) der Sichtschatten-Tabelle: 0,25 Grad, an der Sichtgrenze (0,6 m)
# ~2,6 mm -- weit unter memory_resolution.
SHADOW_BIN = math.radians(0.25)


def _bin_medians(pts, x_min, x_max, bin_size):
    """Median-Punkt der Linienpixel pro Streifen [x, x+bin_size) -> (x, y)-Arrays.

    Der Median liegt auf der Mitte des Linienstreifens. Eine Kante (Min/Max)
    waere bei schraeger Linie verzerrt, weil die Linie innerhalb eines Streifens
    seitlich wandert; einzelne Fehlpixel verschieben den Median kaum.
    """
    xs, ys = [], []
    edges = np.arange(x_min, x_max + 1e-9, bin_size)
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (pts[0] >= lo) & (pts[0] < hi)
        if m.any():
            xs.append(np.median(pts[0][m]))
            ys.append(np.median(pts[1][m]))
    return np.array(xs), np.array(ys)


def _fit(xs, ys):
    """Polynom Y(X) durch die Linienpunkte: 2. Grad ab 4 Punkten, sonst Gerade."""
    if len(xs) < 2:
        return None
    return np.polyfit(xs, ys, 2 if len(xs) >= 4 else 1)


def _slope(coeff, x):
    return 0.0 if coeff is None else float(np.polyval(np.polyder(coeff), x))


def lane_center_points(yellow, white, x_min, x_max, bin_size, half_width):
    """Punkte auf der Spurmitte, in base_footprint.

    yellow/white: (forward, lateral)-Arrays. Gelb liegt links, Weiss rechts.
    Jede Linie wird fuer sich durch ein Polynom angenaehert (wie bei ROBOTIS
    turtlebot3_autorace), dann wird jeder ihrer Punkte senkrecht zur Linie um
    die halbe Spurbreite zur Mitte versetzt. Senkrecht, nicht entlang Y: in
    Kurven waere der Abstand entlang Y groesser als die echte Breite.

    Die Spurbreite wird gemessen, nicht vorgegeben: von Linienmitte zu
    Linienmitte, ueberall dort, wo beide Linien zu sehen sind. Die Linienbreite
    faellt dabei heraus. Sieht dieses Bild nur eine Linie (Luecke in Weiss,
    enge Kurve, Hindernis verdeckt), gilt half_width aus den vorigen Bildern;
    ist die noch unbekannt (None), gibt es keine Mittelpunkte.

    Weiss links der gelben Linie gehoert nicht zu dieser Spur und wird ignoriert.
    Gibt (x, y, breiten) zurueck; breiten sind die in diesem Bild gemessenen.
    """
    empty = (np.empty(0), np.empty(0), [])

    yx, yy = _bin_medians(yellow, x_min, x_max, bin_size)
    y_fit = _fit(yx, yy)
    if y_fit is not None and white[0].size:
        keep = white[1] < np.polyval(y_fit, white[0])
        white = (white[0][keep], white[1][keep])
    wx, wy = _bin_medians(white, x_min, x_max, bin_size)
    w_fit = _fit(wx, wy)

    widths = []
    if y_fit is not None and w_fit is not None:
        for x in np.arange(max(yx.min(), wx.min()), min(yx.max(), wx.max()) + 1e-9, bin_size):
            k = (_slope(y_fit, x) + _slope(w_fit, x)) / 2.0
            width = (np.polyval(y_fit, x) - np.polyval(w_fit, x)) / np.hypot(1.0, k)
            if width > 0:
                widths.append(float(width))

    hw = float(np.median(widths)) / 2.0 if widths else half_width
    if hw is None:
        return empty

    cx, cy = [], []
    for x, y in zip(yx, yy):          # Normale nach rechts: (k, -1) / |(1, k)|
        k = _slope(y_fit, x)
        n = np.hypot(1.0, k)
        cx.append(x + k * hw / n)
        cy.append(y - hw / n)
    for x, y in zip(wx, wy):          # Normale nach links: (-k, 1) / |(1, k)|
        k = _slope(w_fit, x)
        n = np.hypot(1.0, k)
        cx.append(x - k * hw / n)
        cy.append(y + hw / n)
    return np.array(cx), np.array(cy), widths


def fit_center_line(xs, ys, step):
    """Glatte Mittellinie durch die Mittelpunkte, in base_footprint.

    Polynom zweiten Grades Y(X) ab 4 Punkten, darunter eine Gerade. Abgetastet
    wird nur zwischen dem naechsten und dem fernsten Mittelpunkt -- keine
    Extrapolation. Gibt (x, y, yaw) zurueck oder None, wenn weniger als zwei
    Posen herauskaemen.
    """
    if len(xs) < 2:
        return None
    deg = 2 if len(xs) >= 4 else 1
    coeff = np.polyfit(xs, ys, deg)
    px = np.arange(xs.min(), xs.max() + 1e-9, step)
    if px.size < 2:
        return None
    py = np.polyval(coeff, px)
    yaw = np.arctan(np.polyval(np.polyder(coeff), px))
    return px, py, yaw


def thin_points(pts, cell):
    """Je Rasterzelle (cell) ein Punkt in der Zellmitte."""
    if len(pts) == 0:
        return pts
    idx = np.floor(pts / cell).astype(np.int64)
    _, first = np.unique(idx[:, 0] * 1000003 + idx[:, 1], return_index=True)
    return (idx[first] + 0.5) * cell


def _groups_1d(values, gap):
    """Werte in (min, max)-Gruppen trennen, wo die Luecke groesser als gap ist."""
    order = np.sort(values)
    cuts = np.nonzero(np.diff(order) > gap)[0]
    return list(zip(order[np.r_[0, cuts + 1]], order[np.r_[cuts, order.size - 1]]))


def line_seed(pts, outer_sign, inner_limit=None, hint_y=None):
    """Startpunkt einer Linie (robot_frame) im bot-nahen Streifen, sonst None.

    outer_sign +1: Linie liegt links (Gelb), -1: rechts (Weiss). inner_limit: Y, auf
    dessen Aussenseite die Linie liegen muss (Weiss rechts von Gelb); dann gilt die
    innerste Gruppe, sonst die groesste. hint_y: Y des Startpunkts im letzten Bild.
    """
    if len(pts) == 0:
        return None
    strip = pts[pts[:, 0] < pts[:, 0].min() + SEED_DEPTH]
    if inner_limit is not None:
        strip = strip[outer_sign * (strip[:, 1] - inner_limit) > 0]
        if len(strip) == 0:
            return None
    groups = []
    for lo, hi in _groups_1d(strip[:, 1], SEED_GAP):
        if hi - lo > SEED_MAX_WIDTH:
            lo, hi = (hi - SEED_MAX_WIDTH, hi) if outer_sign > 0 else (lo, lo + SEED_MAX_WIDTH)
        groups.append(strip[(strip[:, 1] >= lo) & (strip[:, 1] <= hi)])
    if hint_y is not None:
        near = [g for g in groups if abs(np.median(g[:, 1]) - hint_y) <= SEED_HINT_TOL]
        groups = near or groups
    if inner_limit is not None:
        group = min(groups, key=lambda g: outer_sign * np.median(g[:, 1]))
    else:
        group = max(groups, key=len)
    return np.median(group, axis=0)


def _start_direction(pts, seed):
    """Anfangsrichtung der Verfolgung: Hauptachse der Linienpunkte bis TRACE_START_RADIUS um
    den Startpunkt, nach vorn (+X) zeigend. Fest geradeaus brach die Verfolgung ab, wenn der
    Bot schraeg zur Spur stand (Obstacle, Linie 47 Grad zur Fahrtrichtung: 3 cm statt der
    ganzen Linie, Aufnahme 12.11.). Zu wenig Punkte: geradeaus wie bisher."""
    near = pts[np.hypot(pts[:, 0] - seed[0], pts[:, 1] - seed[1]) < TRACE_START_RADIUS]
    if len(near) < TRACE_MIN_POINTS:
        return np.array([1.0, 0.0])
    c = near - near.mean(axis=0)
    values, vectors = np.linalg.eigh(c.T @ c)
    # Nur bei deutlich laenglicher Punktwolke: haengt eine Querlinie am Startpunkt (L-Form an
    # der Kreuzung), ist die Achse unklar -- dann geradeaus wie bisher.
    if values[1] < TRACE_START_ELONGATION * max(values[0], 1e-12):
        return np.array([1.0, 0.0])
    axis = vectors[:, -1]
    return axis if axis[0] >= 0.0 else -axis


def trace_line(pts, seed, step, max_turn, max_gap):
    """Linienmitte ab seed in beide Richtungen verfolgen.

    Je Schritt: Median der Punkte im Kreis (Radius 4/3 step) um den naechsten Punkt
    voraus, die Richtung dreht hoechstens um max_turn (rad) zum Median hin. So folgt
    sie engen Kurven, eine seitlich anschliessende Querlinie verschiebt den Median
    aber kaum. Luecken bis max_gap (m) werden geradeaus ueberbrueckt.
    Gibt (Polylinie (K, 2), Index des Startpunkts) zurueck.
    """
    radius = step * 4.0 / 3.0
    max_gap_steps = int(round(max_gap / step))
    start = _start_direction(pts, seed)
    halves = []
    for direction in (start, -start):
        p, d, path, missed = seed, direction, [], 0
        for _ in range(int(TRACE_MAX_LENGTH / step)):
            q = p + step * d
            near = pts[np.hypot(pts[:, 0] - q[0], pts[:, 1] - q[1]) < radius]
            nd = np.median(near, axis=0) - p if len(near) >= TRACE_MIN_POINTS else None
            # Linienende ragt nur noch knapp ins Fenster: wie eine Luecke behandeln.
            if nd is None or nd @ d < 0.5 * step:
                missed += 1
                if missed > max_gap_steps:
                    break
                p = q
                continue
            if missed:
                path.append(p)
                missed = 0
            nd /= math.hypot(nd[0], nd[1])
            turn = math.atan2(d[0] * nd[1] - d[1] * nd[0], d @ nd)
            turn = max(-max_turn, min(max_turn, turn))
            c, s = math.cos(turn), math.sin(turn)
            d = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
            p = p + step * d
            path.append(p)
        halves.append(path)
    return np.array(halves[1][::-1] + [seed] + halves[0]), len(halves[1])


def cut_unparallel(line, seed_index, other, max_angle):
    """line ab dem Startpunkt nach aussen dort abschneiden, wo sie mit der
    naechstgelegenen Strecke von other mehr als max_angle (rad) einschliesst: in einer
    Kurve biegen beide Linien ab, an einer Querlinie in L-Form nur eine."""
    if other is None or len(other) < 2 or len(line) < 2:
        return line
    other_dir = np.diff(other, axis=0)
    other_dir /= np.maximum(np.hypot(other_dir[:, 0], other_dir[:, 1]), 1e-12)[:, None]
    other_mid = (other[:-1] + other[1:]) / 2.0
    cos_max = math.cos(max_angle)

    def bends_away(i):
        d = line[i + 1] - line[i]
        n = math.hypot(d[0], d[1])
        if n < 1e-9:
            return False
        m = (line[i] + line[i + 1]) / 2.0
        k = np.argmin(np.hypot(other_mid[:, 0] - m[0], other_mid[:, 1] - m[1]))
        return abs(d @ other_dir[k]) / n < cos_max

    hi = next((i for i in range(seed_index, len(line) - 1) if bends_away(i)), len(line) - 1)
    lo = next((i + 1 for i in range(seed_index - 1, -1, -1) if bends_away(i)), 0)
    return line[lo:hi + 1]


def near_polyline(pts, line, margin):
    """Maske: Punkte hoechstens margin (auf NEAR_RES genau) von der Polylinie entfernt.
    Die Polylinie wird mit Breite 2*margin in ein Raster gezeichnet."""
    if len(line) < 2 or len(pts) == 0:
        return np.zeros(len(pts), dtype=bool)
    origin = pts.min(axis=0) - margin
    size = np.ceil((pts.max(axis=0) + margin - origin) / NEAR_RES).astype(int) + 1
    grid = np.zeros((size[1], size[0]), np.uint8)
    poly = np.round((line - origin) / NEAR_RES).astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(grid, [poly], False, 1, thickness=max(1, int(round(2 * margin / NEAR_RES))))
    idx = np.floor((pts - origin) / NEAR_RES).astype(int)
    return grid[idx[:, 1], idx[:, 0]] > 0



class lane_costmap_node(Node):
    """Spurgedaechtnis: ein Raster im odom-Frame (MEMORY_RESOLUTION), das sich die
    Spurlinien merkt. Eine Zelle wechselt zwischen "Linie" und "frei" erst, wenn
    line_confirm_frames bzw. free_confirm_frames Kamerabilder hintereinander das
    Gegenteil zeigen. So bleiben Linien neben und hinter dem Bot erhalten, die die
    Kamera nicht mehr sieht, und einzelne Fehldetektionen kommen nicht hinein.

    Wand werden nur die gelbe und die weisse Spurlinie (_lane_line_points): jede Linie
    wird vom bot-nahen Ende aus verfolgt, nur Pixel bis line_margin um ihre Mitte
    zaehlen. Querlinien, Papier oder helle Flaechen neben der Strecke werden so keine
    Wand. Pixel am Hindernis und in seinem LiDAR-Schatten fallen vorher weg: die BEV
    zieht hohe helle Gegenstaende lang.

    Abgestimmt wird nur, was die Kamera in diesem Bild sieht (Sichtmaske von
    ingress_node). Zellen im Sichtschatten eines LiDAR-Hindernisses bekommen keine
    "frei"-Stimme: dort verdeckt das Hindernis die Linie, sie ist nicht weg.

    Gesendet wird jedes Mal das ganze Gedaechtnis im Umkreis MEMORY_RADIUS, im
    robot_frame zum Bildzeitpunkt. Die lane_layer raeumen per Strahl vom Bot aus
    frei und tragen alle Punkte wieder ein; gemerkte Linien bleiben so stehen,
    vergessene verschwinden.

    In jedem Modus ausser Tunnel, damit der Korridor beim Umschalten auf Obstacle
    schon in der Karte steht. Vor der ersten /control/mode nichts, damit ein Neustart
    im Tunnel keine Kameradaten durchlaesst.

    Beim Tunnel-Eintritt wird das Gedaechtnis verworfen und eine leere Wolke gesendet,
    COSTMAP_CLEAR_DELAY_S spaeter werden beide Costmaps geleert. Die Reihenfolge zaehlt:
    ein lane_layer behaelt die letzte Wolke im Puffer und traegt sie nach dem Leeren
    wieder ein; eine leere Wolke allein loescht nichts aus der Karte. Dasselbe auf
    /control/clear_maps, wenn die Karten nicht mehr zur Odometrie passen (Bot angehoben).

    Die Spurmitte (nur Obstacle) liegt im odom-Frame zum Bildzeitpunkt und ist
    das Ziel, zu dem nav2_goal_publisher den Bahnplaner rechnen laesst.

    Pixel -> Meter ueber Parameter/bev.yaml: X vorwaerts ab Radachse, Y nach links, am
    Boden, also base_footprint (TurtleBot3-URDF).
    """

    def __init__(self):
        super().__init__('lane_costmap_node')

        # --- Parameter ---
        # Werte nur in Lane_Costmap.yaml. Bilder in Folge, bis eine Zelle zur Linie bzw. wieder
        # frei wird. free groesser als line: ein Hindernis oder der eigene Schatten kann eine
        # Linie kurz verdecken, ohne dass die LiDAR-Schattenpruefung greift.
        self.declare_parameter('line_confirm_frames', Parameter.Type.INTEGER, irange(1, 100))
        self.declare_parameter('free_confirm_frames', Parameter.Type.INTEGER, irange(1, 500))
        # Pixel bis so weit (m) um die Linienmitte werden Wand; 0: nur Pixel direkt auf der
        # verfolgten Mittellinie (ein Rasterpunkt NEAR_RES breit).
        self.declare_parameter('line_margin', Parameter.Type.DOUBLE, frange(0.0, 0.2))
        self._param_names = ['line_confirm_frames', 'free_confirm_frames', 'line_margin']
        require_params(self, self._param_names)
        read_params(self, self._param_names)

        # --- Subscriber ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_clear_maps = self.create_subscription(
            Empty, '/control/clear_maps', self.cb_clear_maps, 1)
        self.sub_lane_mask = self.create_subscription(
            LaneMask, '/detect/lane_mask', self.cb_lane_mask, 1)
        self.sub_bev_view = self.create_subscription(
            Image, '/lokal/bev_view', self.cb_bev_view, latched)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)

        # --- Publisher ---
        self.pub_lane_obstacles = self.create_publisher(
            PointCloud2, LANE_OBSTACLES_TOPIC, 1)
        self.pub_center_path = self.create_publisher(
            Path, CENTER_PATH_TOPIC, 1)
        self.clear_clients = {
            'local_costmap': (self.create_client(
                ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'),
                'controller_server'),
            'global_costmap': (self.create_client(
                ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'),
                'planner_server'),
        }

        # --- Timer ---
        self.clear_timer = self.create_timer(COSTMAP_CLEAR_DELAY_S, self.cb_clear_timer)
        self.clear_timer.cancel()

        # --- Zustand ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = None
        self.memory = {}               # (ix, iy) im odom-Raster -> [ist_linie, gegenstimmen]
        self._widths = deque(maxlen=CENTER_WIDTH_HISTORY)
        self._size_error_logged = False
        self._last_cloud_stamp = None
        self._seed_hints = {}           # Farbe -> (Y des Startpunkts, time.monotonic())
        self._view_points = None       # sichtbare BEV-Stuetzpunkte (K, 2) in robot_frame
        self._scan = None
        self._scan_time = None
        self._scanner_pose = None      # Scanner im robot_frame (x, y, yaw)
        self._bev = self._load_bev()
        self._camera_xy = self._load_camera_position()

        self.get_logger().info('Started lane_costmap_node Node.')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None:
            return
        entering_tunnel = new_mode == ControlType.Tunnel and self.control_mode != ControlType.Tunnel
        self.control_mode = new_mode
        if entering_tunnel:
            self._forget_everything()

    def cb_clear_maps(self, _msg):
        self.get_logger().info('Spurgedaechtnis und Costmaps werden geleert (/control/clear_maps).')
        self._forget_everything()

    def cb_bev_view(self, msg):
        if self._bev is None:
            return
        if (msg.width, msg.height) != (self._bev['width'], self._bev['height']):
            self.get_logger().error(
                f'/lokal/bev_view ist {msg.width}x{msg.height}, bev.yaml beschreibt '
                f'{self._bev["width"]}x{self._bev["height"]} -- Gedaechtnis vergisst nichts.')
            return
        view = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
        # Stuetzpunkte im halben Zellabstand, damit jede sichtbare Zelle getroffen wird.
        step = max(1, int(MEMORY_RESOLUTION / 2.0 / self._bev['res']))
        vs, us = np.nonzero(view[::step, :msg.width:step] > 0)
        self._view_points = np.column_stack(
            bev_pixels_to_ground(self._bev, us * step, vs * step))

    def cb_clear_timer(self):
        self.clear_timer.cancel()
        self._clear_costmaps()

    def cb_scan(self, scan):
        self._scan = scan
        self._scan_time = time.monotonic()

    def cb_lane_mask(self, lane_mask):
        if self.control_mode is None or self.control_mode == ControlType.Tunnel:
            return
        if self._bev is None:
            return

        size = (self._bev['width'], self._bev['height'])
        if (lane_mask.image_width, lane_mask.image_height) != size:
            if not self._size_error_logged:
                self.get_logger().error(
                    f'LaneMask kommt mit {lane_mask.image_width}x{lane_mask.image_height}, '
                    f'{bev_geometry.BEV_FILE} beschreibt {size[0]}x{size[1]} '
                    '-- es werden keine Spurwaende gesendet.')
                self._size_error_logged = True
            return

        read_params(self, self._param_names)
        stride = max(1, int(PIXEL_STRIDE))
        yellow = self._pixels_to_ground(lane_mask.yellow_x, lane_mask.yellow_y, stride)
        white = self._pixels_to_ground(lane_mask.white_x, lane_mask.white_y, stride)

        # Pose des Bots zum Bildzeitpunkt, nicht jetzt: sonst laegen Linien und Pfad
        # um die seitdem gefahrene Strecke versetzt.
        stamp = lane_mask.header.stamp
        try:
            tf = self.tf_buffer.lookup_transform(
                ODOM_FRAME, ROBOT_FRAME, Time.from_msg(stamp))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'Keine TF {ODOM_FRAME} <- {ROBOT_FRAME} zum Bildzeitpunkt: {e}',
                throttle_duration_sec=2.0)
            return
        robot_pose = pose2d_from_transform(tf)

        yellow, white = self._lane_line_points(np.column_stack(yellow), np.column_stack(white))
        self._update_memory(robot_pose, np.vstack([yellow, white]))
        self._publish_memory(robot_pose, stamp)
        self._update_center_path((yellow[:, 0], yellow[:, 1]), (white[:, 0], white[:, 1]),
                                 robot_pose, stamp)

    # --- Hilfsmethoden ---

    def _load_bev(self):
        param_dir = os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter')
        try:
            bev = bev_geometry.load_bev_spec(param_dir)
        except Exception as e:  # noqa: BLE001 -- jeder Ladefehler heisst: keine Spurwaende
            self.get_logger().error(
                f'{bev_geometry.BEV_FILE} nicht ladbar ({e}) -- es werden keine Spurwaende gesendet.')
            return None
        self.get_logger().info(
            f'BEV aus {bev_geometry.BEV_FILE}: {bev["width"]}x{bev["height"]} px, '
            f'{bev["res"] * 1000:.2f} mm/px, X {bev["x_min"]:.2f}-{bev["x_max"]:.2f} m.')
        return bev

    def _load_camera_position(self):
        """Linse in robot_frame (x, y) aus der Extrinsik; Ursprung des Sichtschattens."""
        path = os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter', EXTRINSICS)
        try:
            with open(path) as f:
                ext = yaml.safe_load(f)
            return (float(ext.get('forward_offset_m', 0.0)), float(ext.get('lateral_offset_m', 0.0)))
        except Exception as e:  # noqa: BLE001 -- Radachse ist eine brauchbare Naeherung
            self.get_logger().warn(f'Extrinsik nicht ladbar ({e}) -- Kamera bei x=0 angenommen.')
            return (0.0, 0.0)

    def _forget_everything(self):
        self.memory.clear()
        self._seed_hints.clear()
        self._publish_empty_cloud()
        self.clear_timer.reset()

    def _clear_costmaps(self):
        # Leert alle Layer; LiDAR und Eingangssperre fuellen sich im naechsten Zyklus neu.
        for costmap, (client, server) in self.clear_clients.items():
            if not client.service_is_ready():
                self.get_logger().error(
                    f'{costmap}-Clear-Service nicht erreichbar -- alte Spurwaende bleiben '
                    f'in der Karte. Laeuft {server}?')
                continue
            client.call_async(ClearEntireCostmap.Request())

    def _publish_empty_cloud(self):
        # Stempel der letzten Wolke: zu dem hat die Costmap sicher eine TF. Ohne
        # bisherige Wolke steht auch nichts im Puffer.
        if self._last_cloud_stamp is None:
            return
        header = Header()
        header.stamp = self._last_cloud_stamp
        header.frame_id = ROBOT_FRAME
        self.pub_lane_obstacles.publish(point_cloud2.create_cloud_xyz32(header, []))

    def _pixels_to_ground(self, xs, ys, stride):
        """Linienpixel -> (forward, lateral) in Metern, base_footprint."""
        empty = (np.empty(0), np.empty(0))
        if not len(xs):
            return empty
        xs = np.asarray(xs)
        ys = np.asarray(ys)
        keep = (xs % stride == 0) & (ys % stride == 0)
        if not keep.any():
            return empty
        return bev_pixels_to_ground(self._bev, xs[keep], ys[keep])

    def _cells(self, points_odom):
        if len(points_odom) == 0:
            return set()
        idx = np.unique(np.floor(points_odom / MEMORY_RESOLUTION).astype(np.int64), axis=0)
        return set(map(tuple, idx.tolist()))

    def _lane_line_points(self, yellow, white):
        """Nur die Pixel (robot_frame, (N, 2)) der gelben und der weissen Spurlinie."""
        now = time.monotonic()
        max_turn = math.radians(TRACE_MAX_TURN_DEG)
        lines, seeds = {}, {}
        for name, side in (('Gelb', 1), ('Weiss', -1)):
            pts = yellow if side > 0 else white
            if len(pts):
                pts = pts[~self._shadowed(pts, unknown=False)]
            hint = self._seed_hints.get(name)
            hint_y = hint[0] if hint and now - hint[1] <= SEED_HINT_MAX_AGE_S else None
            inner = seeds['Gelb'][1] if side < 0 and 'Gelb' in seeds else None
            thin = thin_points(pts, TRACE_THIN_CELL)
            seed = line_seed(thin, side, inner_limit=inner, hint_y=hint_y)
            if seed is None:
                lines[name] = (pts, None, 0)
                continue
            seeds[name] = seed
            self._seed_hints[name] = (float(seed[1]), now)
            line, seed_index = trace_line(thin, seed, TRACE_STEP, max_turn, TRACE_MAX_GAP)
            lines[name] = (pts, line, seed_index)

        max_angle = math.radians(TRACE_PARALLEL_DEG)
        kept = []
        for name, other in (('Gelb', 'Weiss'), ('Weiss', 'Gelb')):
            pts, line, seed_index = lines[name]
            if line is None:
                kept.append(np.empty((0, 2)))
                continue
            line = cut_unparallel(line, seed_index, lines[other][1], max_angle)
            kept.append(pts[near_polyline(pts, line, self.line_margin)])
        return kept[0], kept[1]

    def _update_memory(self, robot_pose, line_points):
        """Eine Stimme pro sichtbarer Zelle: Linie, wenn ein Linienpixel darin liegt,
        sonst frei. Der Zustand kippt nach genug Gegenstimmen in Folge."""
        line_cells = self._cells(transform_points(robot_pose, line_points))
        observed = set(line_cells)
        if self._view_points is not None:
            free_candidates = self._view_points[~self._shadowed(self._view_points)]
            observed |= self._cells(transform_points(robot_pose, free_candidates))
        else:
            self.get_logger().warn(
                'Noch keine Sichtmaske /lokal/bev_view -- Linien werden nur gemerkt, '
                'nicht vergessen.', throttle_duration_sec=10.0)

        for cell in observed:
            seen_line = cell in line_cells
            state = self.memory.get(cell)
            if state is None:
                if not seen_line:
                    continue
                state = self.memory[cell] = [False, 0]
            if seen_line == state[0]:
                state[1] = 0
            else:
                state[1] += 1
                needed = self.line_confirm_frames if seen_line else self.free_confirm_frames
                if state[1] >= needed:
                    state[0], state[1] = seen_line, 0
            # Freie Zellen ohne Gegenstimme muss sich niemand merken.
            if not state[0] and state[1] == 0:
                del self.memory[cell]
        # Entfernte Zellen werden nicht vergessen, nur nicht gesendet (_publish_memory):
        # ein Odometrie-Sprung setzte den Bot sonst fuer ein Bild weit weg und loeschte
        # das ganze Gedaechtnis. Geleert wird nur beim Tunnel-Eintritt und mit Taste L.

    def _shadowed(self, points, unknown=True):
        """Welche Punkte (robot_frame) verdeckt ein LiDAR-Hindernis fuer die Kamera,
        einschliesslich des Hindernisses selbst? Ohne aktuellen Scan gilt unknown fuer
        alle: True beim Vergessen (dann wird nichts vergessen), False beim Verwerfen
        von Linienpixeln (dann wird nichts verworfen)."""
        if self._scan is None or time.monotonic() - self._scan_time > SCAN_MAX_AGE_S:
            return np.full(len(points), unknown, dtype=bool)
        if self._scanner_pose is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    ROBOT_FRAME, self._scan.header.frame_id, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                return np.full(len(points), unknown, dtype=bool)
            self._scanner_pose = pose2d_from_transform(tf)

        _, _, hits = scan_points(self._scan)
        hits = transform_points(self._scanner_pose, hits) - self._camera_xy
        hits = hits[hits[:, 0] > 0.0]
        if hits.size == 0:
            return np.zeros(len(points), dtype=bool)
        # Tabelle ueber den Winkel: je Raster die naechste Schattengrenze aller Keile, die
        # es ueberdecken. Statt jeden Punkt gegen jeden Treffer zu pruefen (bei 1-cm-Raster
        # ~17000 x 180, 40 ms je Bild) nur ein Nachschlagen je Punkt.
        ang_h, dist_h = np.arctan2(hits[:, 1], hits[:, 0]), np.hypot(hits[:, 0], hits[:, 1])
        half = np.arctan2(SHADOW_RADIUS, dist_h)
        n_bins = int(math.ceil(2.0 * math.pi / SHADOW_BIN))
        start = np.floor((ang_h - half + math.pi) / SHADOW_BIN).astype(int)
        stop = np.floor((ang_h + half + math.pi) / SHADOW_BIN).astype(int)
        edge = np.full(n_bins, np.inf)
        for b0, b1, d in zip(start, stop, dist_h - SHADOW_MARGIN):
            idx = np.arange(b0, b1 + 1) % n_bins
            edge[idx] = np.minimum(edge[idx], d)

        rel = points - self._camera_xy
        ang_p, dist_p = np.arctan2(rel[:, 1], rel[:, 0]), np.hypot(rel[:, 0], rel[:, 1])
        bins = np.floor((ang_p + math.pi) / SHADOW_BIN).astype(int) % n_bins
        return dist_p >= edge[bins]

    def _publish_memory(self, robot_pose, stamp):
        lines = [c for c, state in self.memory.items() if state[0]]
        if lines:
            centers = (np.asarray(lines, dtype=np.float64) + 0.5) * MEMORY_RESOLUTION
            local = inverse_transform_points(robot_pose, centers)
            local = local[np.hypot(local[:, 0], local[:, 1]) <= MEMORY_RADIUS]
            points = [(float(x), float(y), 0.0) for x, y in local]
        else:
            points = []
        header = Header()
        header.stamp = stamp
        header.frame_id = ROBOT_FRAME
        self.pub_lane_obstacles.publish(point_cloud2.create_cloud_xyz32(header, points))
        self._last_cloud_stamp = stamp

    def _update_center_path(self, yellow, white, robot_pose, stamp):
        half_width = float(np.median(self._widths)) / 2.0 if self._widths else None
        xs, ys, widths = lane_center_points(
            yellow, white, self._bev['x_min'], self._bev['x_max'],
            CENTER_BIN_SIZE, half_width)
        # Die Breite auch in den anderen Modi messen: beim Umschalten ist
        # sonst noch keine bekannt, falls dann nur eine Linie zu sehen ist.
        if widths:
            self._widths.append(float(np.median(widths)))
        # Nur in diesen Modi plant jemand damit; sonst keine Planungsdaten senden.
        if self.control_mode not in CENTER_PATH_MODES or len(xs) < CENTER_MIN_POINTS:
            return

        line = fit_center_line(xs, ys, CENTER_BIN_SIZE)
        if line is None:
            return
        px, py, pyaw = line
        poses_odom = transform_points(robot_pose, np.column_stack([px, py]))
        robot_yaw = robot_pose[2]

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = ODOM_FRAME
        for (x, y), yaw in zip(poses_odom, pyaw + robot_yaw):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        self.pub_center_path.publish(path)

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down lane_costmap_node Node.')


def main(args=None):
    run_node(lane_costmap_node, args)


if __name__ == '__main__':
    main()
