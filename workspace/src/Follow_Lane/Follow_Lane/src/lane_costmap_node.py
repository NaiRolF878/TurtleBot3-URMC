"""Spurgedaechtnis: Spurlinien als virtuelle Waende fuer Nav2, Spurmitte als Pfad.

Ein:  /detect/lane_mask (LaneMask, BEV-Pixel), /lokal/bev_view (Sichtmaske), /scan,
      /control/mode
Aus:  /detect/lane_obstacles (PointCloud2 aller gemerkten Linienzellen, fuer die
      lane_layer der Costmaps und den Linienfilter von switch_control),
      /lane/center_path (Path im odom-Frame, nur in Obstacle und TunnelEntry)
Im Tunnel nichts: die Kamera ist dort laut Reglement verboten.
"""
import math
import os
import time
from collections import deque

import numpy as np
import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import LaneMask
from Follow_Lane.src import bev_geometry
from Follow_Lane.src.common import (
    ControlType, bev_pixels_to_ground, control_mode_from_msg, frange, inverse_transform_points,
    irange, pose2d_from_transform, read_params, run_node, scan_points, transform_points)

EXTRINSICS = os.path.join('camera_info', 'extrinsics_800x600.yaml')
CENTER_PATH_MODES = (ControlType.Obstacle, ControlType.TunnelEntry)
# LiDAR-Scan aelter als das (s) taugt nicht mehr fuer den Sichtschatten.
SCAN_MAX_AGE_S = 0.5


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



class lane_costmap_node(Node):
    """Spurgedaechtnis: ein Raster im odom-Frame (memory_resolution), das sich die
    Spurlinien merkt. Eine Zelle wechselt zwischen "Linie" und "frei" erst, wenn
    line_confirm_frames bzw. free_confirm_frames Kamerabilder hintereinander das
    Gegenteil zeigen. So bleiben Linien neben und hinter dem Bot erhalten, die die
    Kamera nicht mehr sieht, und einzelne Fehldetektionen kommen nicht hinein.

    Abgestimmt wird nur, was die Kamera in diesem Bild sieht (Sichtmaske von
    ingress_node). Zellen im Sichtschatten eines LiDAR-Hindernisses bekommen keine
    "frei"-Stimme: dort verdeckt das Hindernis die Linie, sie ist nicht weg.

    Gesendet wird jedes Mal das ganze Gedaechtnis im Umkreis memory_radius, im
    robot_frame zum Bildzeitpunkt. Die lane_layer raeumen per Strahl vom Bot aus
    frei und tragen alle Punkte wieder ein; gemerkte Linien bleiben so stehen,
    vergessene verschwinden.

    In jedem Modus ausser Tunnel, damit der Korridor beim Umschalten auf Obstacle
    schon in der Karte steht. Vor der ersten /control/mode nichts, damit ein Neustart
    im Tunnel keine Kameradaten durchlaesst. Beim Tunnel-Eintritt wird das Gedaechtnis
    verworfen und die local_costmap geleert.

    Die Spurmitte (nur Obstacle und TunnelEntry) liegt im odom-Frame zum Bildzeitpunkt und ist
    das Ziel, zu dem nav2_goal_publisher den Bahnplaner rechnen laesst.

    Pixel -> Meter ueber Parameter/bev.yaml: X vorwaerts ab Radachse, Y nach links, am
    Boden. Das ist genau base_footprint: in der TurtleBot3-URDF (burger) sitzen die
    Radgelenke bei x=0 relativ zu base_link, base_link liegt 0.010 m ueber
    base_footprint, die Achse also 0.010+0.023 = 0.033 m hoch = Radradius.
    """

    def __init__(self):
        super().__init__('lane_costmap_node')

        # --- Parameter ---
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('output_topic', '/detect/lane_obstacles')
        self.declare_parameter('pixel_stride', 2, irange(1, 20))
        # Zellgroesse (m), gleich der resolution der Costmaps.
        self.declare_parameter('memory_resolution', 0.025, frange(0.005, 0.2))
        # Weiter entfernte Zellen werden vergessen; nicht ueber die Reichweite der
        # lane_layer (obstacle_max_range in nav2_params.yaml) hinaus.
        self.declare_parameter('memory_radius', 2.0, frange(0.5, 10.0))
        self.declare_parameter('line_confirm_frames', 3, irange(1, 100))
        # Groesser als line_confirm_frames: ein Hindernis oder der eigene Schatten kann
        # eine Linie kurz verdecken, ohne dass die LiDAR-Schattenpruefung greift.
        self.declare_parameter('free_confirm_frames', 10, irange(1, 500))
        # Jeder LiDAR-Punkt verdeckt fuer die Kamera einen Keil dieser halben Breite
        # (m); Zellen bis shadow_margin vor dem Punkt zaehlen schon zum Hindernis.
        self.declare_parameter('shadow_radius', 0.03, frange(0.0, 0.5))
        self.declare_parameter('shadow_margin', 0.05, frange(0.0, 0.5))
        self.declare_parameter('center_path_topic', '/lane/center_path')
        # Tiefe eines Streifens (m), zugleich Abstand der Posen im Pfad.
        self.declare_parameter('center_bin_size', 0.05, frange(0.01, 0.5))
        # Mindestzahl Mittelpunkte, sonst wird kein Pfad gesendet.
        self.declare_parameter('center_min_points', 3, irange(2, 50))
        # Ueber so viele Bilder wird die gemessene Spurbreite (Linienmitte zu
        # Linienmitte) gemittelt (Median); sie gilt, wenn ein Bild nur eine Linie
        # sieht. Kurz genug, dass der Wechsel von der 20-cm-Spur in den breiteren
        # Hindernisbereich nach wenigen Bildern nachgezogen ist.
        self.declare_parameter('center_width_history', 20, irange(1, 500))
        self._param_names = [
            'robot_frame', 'odom_frame', 'pixel_stride', 'memory_resolution', 'memory_radius',
            'line_confirm_frames', 'free_confirm_frames', 'shadow_radius', 'shadow_margin',
            'center_bin_size', 'center_min_points',
        ]
        read_params(self, self._param_names)

        # --- Subscriber ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_lane_mask = self.create_subscription(
            LaneMask, '/detect/lane_mask', self.cb_lane_mask, 1)
        self.sub_bev_view = self.create_subscription(
            Image, '/lokal/bev_view', self.cb_bev_view, latched)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)

        # --- Publisher ---
        self.pub_lane_obstacles = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 1)
        self.pub_center_path = self.create_publisher(
            Path, self.get_parameter('center_path_topic').value, 1)
        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # --- Zustand ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = None
        self.memory = {}               # (ix, iy) im odom-Raster -> [ist_linie, gegenstimmen]
        self._widths = deque(maxlen=self.get_parameter('center_width_history').value)
        self._size_error_logged = False
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
            self.memory.clear()
            self._clear_local_costmap()

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
        step = max(1, int(self.memory_resolution / 2.0 / self._bev['res']))
        vs, us = np.nonzero(view[::step, :msg.width:step] > 0)
        self._view_points = np.column_stack(
            bev_pixels_to_ground(self._bev, us * step, vs * step))

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
        stride = max(1, int(self.pixel_stride))
        yellow = self._pixels_to_ground(lane_mask.yellow_x, lane_mask.yellow_y, stride)
        white = self._pixels_to_ground(lane_mask.white_x, lane_mask.white_y, stride)

        # Pose des Bots zum Bildzeitpunkt, nicht jetzt: sonst laegen Linien und Pfad
        # um die seitdem gefahrene Strecke versetzt.
        stamp = lane_mask.header.stamp
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.robot_frame, Time.from_msg(stamp))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'Keine TF {self.odom_frame} <- {self.robot_frame} zum Bildzeitpunkt: {e}',
                throttle_duration_sec=2.0)
            return
        robot_pose = pose2d_from_transform(tf)

        line_points = np.vstack([np.column_stack(yellow), np.column_stack(white)])
        self._update_memory(robot_pose, line_points)
        self._publish_memory(robot_pose, stamp)
        self._update_center_path(yellow, white, robot_pose, stamp)

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

    def _clear_local_costmap(self):
        # Leert alle Layer; LiDAR und Eingangssperre fuellen sich im naechsten Zyklus neu.
        if not self.clear_local_costmap.service_is_ready():
            self.get_logger().error(
                'local_costmap-Clear-Service nicht erreichbar -- alte Spurwaende bleiben '
                'in der Karte. Laeuft controller_server?')
            return
        self.clear_local_costmap.call_async(ClearEntireCostmap.Request())

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
        idx = np.unique(np.floor(points_odom / self.memory_resolution).astype(np.int64), axis=0)
        return set(map(tuple, idx.tolist()))

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

        rx, ry, _ = robot_pose
        limit = (self.memory_radius / self.memory_resolution) ** 2
        cx, cy = rx / self.memory_resolution, ry / self.memory_resolution
        for cell in [c for c in self.memory if (c[0] + 0.5 - cx) ** 2 + (c[1] + 0.5 - cy) ** 2 > limit]:
            del self.memory[cell]

    def _shadowed(self, points):
        """Welche Punkte (robot_frame) verdeckt ein LiDAR-Hindernis fuer die Kamera?
        Ohne aktuellen Scan gelten alle als verdeckt: dann wird nichts vergessen."""
        if self._scan is None or time.monotonic() - self._scan_time > SCAN_MAX_AGE_S:
            return np.ones(len(points), dtype=bool)
        if self._scanner_pose is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.robot_frame, self._scan.header.frame_id, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                return np.ones(len(points), dtype=bool)
            self._scanner_pose = pose2d_from_transform(tf)

        _, _, hits = scan_points(self._scan)
        hits = transform_points(self._scanner_pose, hits) - self._camera_xy
        hits = hits[hits[:, 0] > 0.0]
        if hits.size == 0:
            return np.zeros(len(points), dtype=bool)
        rel = points - self._camera_xy
        ang_p, dist_p = np.arctan2(rel[:, 1], rel[:, 0]), np.hypot(rel[:, 0], rel[:, 1])
        ang_h, dist_h = np.arctan2(hits[:, 1], hits[:, 0]), np.hypot(hits[:, 0], hits[:, 1])
        d_ang = np.abs(np.arctan2(np.sin(ang_p[:, None] - ang_h), np.cos(ang_p[:, None] - ang_h)))
        in_wedge = d_ang <= np.arctan2(self.shadow_radius, dist_h)
        behind = dist_p[:, None] >= dist_h - self.shadow_margin
        return (in_wedge & behind).any(axis=1)

    def _publish_memory(self, robot_pose, stamp):
        lines = [c for c, state in self.memory.items() if state[0]]
        if lines:
            centers = (np.asarray(lines, dtype=np.float64) + 0.5) * self.memory_resolution
            local = inverse_transform_points(robot_pose, centers)
            points = [(float(x), float(y), 0.0) for x, y in local]
        else:
            points = []
        header = Header()
        header.stamp = stamp
        header.frame_id = self.robot_frame
        self.pub_lane_obstacles.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _update_center_path(self, yellow, white, robot_pose, stamp):
        half_width = float(np.median(self._widths)) / 2.0 if self._widths else None
        xs, ys, widths = lane_center_points(
            yellow, white, self._bev['x_min'], self._bev['x_max'],
            self.center_bin_size, half_width)
        # Die Breite auch in den anderen Modi messen: beim Umschalten ist
        # sonst noch keine bekannt, falls dann nur eine Linie zu sehen ist.
        if widths:
            self._widths.append(float(np.median(widths)))
        # Nur in diesen Modi plant jemand damit; sonst keine Planungsdaten senden.
        if self.control_mode not in CENTER_PATH_MODES or len(xs) < self.center_min_points:
            return

        line = fit_center_line(xs, ys, self.center_bin_size)
        if line is None:
            return
        px, py, pyaw = line
        poses_odom = transform_points(robot_pose, np.column_stack([px, py]))
        robot_yaw = robot_pose[2]

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.odom_frame
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
