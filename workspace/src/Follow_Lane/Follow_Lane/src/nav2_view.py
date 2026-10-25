"""Draufsicht fuer den Nav2-Zweig: was Nav2 sieht und plant, als Bild.

Ein:  /local_costmap/costmap bzw. im Tunnel /global_costmap/costmap, Footprint, Pfade,
      Eingangssperre, /scan, /detect/lane_mask, /control/mode, /nav2/status, /rosout
Aus:  /nav2/view (sensor_msgs/Image, z.B. in rqt_image_view)
"""
import math
import os

import cv2
import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import LaneMask
from Follow_Lane.src import bev_geometry
from Follow_Lane.src.common import (
    ControlType, bev_pixels_to_ground, control_mode_from_msg, frange, irange, run_node,
    yaw_from_quaternion)

# Farben (BGR): befahrbar gruen, nicht befahrbar rot, unbekannt grau.
UNKNOWN = (128, 128, 128)
FREE = (60, 200, 60)          # Kosten 0
NEAR = (20, 90, 30)           # Kosten 98: noch befahrbar, aber dicht am Hindernis
BLOCKED = (40, 40, 220)       # 99 inscribed / 100 lethal: Bot-Umriss wuerde kollidieren
LANE_YELLOW = (0, 230, 255)
LANE_WHITE = (255, 255, 255)
ROBOT = (255, 60, 0)
HEADING = (200, 0, 200)
PATH = (255, 255, 0)
WALL = (0, 140, 255)
SCAN = (255, 190, 120)

# Spurlinien aelter als das werden nicht mehr gezeichnet.
LANE_FRESH_S = 1.0

HEADER_H = 52
FOOTER_H = 26

# Wie lange eine Warnung aus /rosout rot angezeigt wird, danach grau.
REASON_FRESH_S = 10.0


def cost_lut():
    """OccupancyGrid-Wert (-1..100) -> Farbe, Index = Wert + 1.

    costmap_2d uebersetzt Kosten so: 0 frei, 1..98 Inflation, 99 inscribed,
    100 lethal, -1 unbekannt. Befahrbar fuer die Bot-Mitte ist alles < 99 -- gruen,
    mit steigenden Kosten dunkler. 99/100 rot.
    """
    lut = np.zeros((102, 3), np.uint8)
    lut[0] = UNKNOWN
    for v in range(0, 99):
        t = v / 98.0
        lut[v + 1] = [int(round((1 - t) * FREE[c] + t * NEAR[c])) for c in range(3)]
    lut[100] = BLOCKED
    lut[101] = BLOCKED
    return lut


class nav2_view(Node):
    """Im Tunnel die global_costmap (Tunnelkarte), sonst die local_costmap mit den
    Spurwaenden:

        gruen  befahrbar (frei; dunkler = naeher am Hindernis, Nav2 meidet es, darf aber)
        rot    nicht befahrbar (Hindernis, Spurwand, Sicherheitsabstand fuer den Bot-Umriss)
        grau   noch nicht gescannt / unbekannt
        gelb / weiss  erkannte Spurlinien (/detect/lane_mask), nicht im Tunnel
        blau   Umriss des Bots (published_footprint der local_costmap), magenta Fahrtrichtung
        cyan   Pfad, dem Nav2 gerade folgt, mit Ziel (X); nur waehrend einer Fahrt
        orange Eingangssperre     hellblau LiDAR-Punkte

    Oben eine Statuszeile: Modus, was nav2_goal_publisher gerade tut, und die letzte
    Warnung der Nav2-Knoten aus /rosout. Das ist der Abbruchgrund: das
    FollowPath-Ergebnis meldet in Humble nur "abgebrochen", den Grund schreibt
    controller_server nur ins Log.

    Feste Ausrichtung: odom-X nach rechts, odom-Y nach oben. Die Karte liegt ohnehin
    in odom; so stehen Waende still und nur der Bot bewegt sich.
    """

    def __init__(self):
        super().__init__('nav2_view')

        # --- Parameter ---
        self.declare_parameter('view_rate', 5.0, frange(0.5, 30.0))
        # Groesse der laengeren Kartenkante im Bild (px).
        self.declare_parameter('canvas_size', 600, irange(200, 2000))
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        # Ausschnitt ausserhalb des Tunnels: Quadrat um den Bot, 1.2 m je Richtung.
        # Deckt die MPPI-Vorausschau (~1 m) mit Rand. Zentriert, nicht in
        # Fahrtrichtung verschoben, sonst springt das Bild bei jedem Lenkausschlag.
        self.declare_parameter('local_view_size_m', 2.4, frange(0.5, 20.0))
        # Im Tunnel: nur die schon gesehene Flaeche plus dieser Rand, damit das Bild
        # anfangs gross ist und mit der Karte herauszoomt.
        self.declare_parameter('tunnel_view_margin_m', 0.3, frange(0.0, 5.0))
        # Nur Warnungen dieser Knoten zaehlen als Abbruchgrund. nav2_goal_publisher
        # fehlt: sein "Rolling-Goal beendet" wuerde den eigentlichen Grund von
        # controller_server ueberschreiben; was er tut, kommt ueber /nav2/status.
        self.declare_parameter('reason_nodes', [
            'controller_server', 'planner_server', 'local_costmap', 'global_costmap',
            'tunnel_explorer_node', 'lane_costmap_node', 'keepout_zone_node'])

        # --- Subscriber ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._grids = {}
        for name in ('local_costmap', 'global_costmap'):
            self.create_subscription(OccupancyGrid, f'/{name}/costmap',
                                     lambda msg, n=name: self._grids.__setitem__(n, msg), latched)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                 self.cb_footprint, 1)
        self.create_subscription(Path, '/nav2/active_path', self.cb_active_path,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointCloud2, '/detect/tunnel_entrance_obstacle',
                                 self.cb_entrance, 1)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/control/mode', self.cb_control_mode, 1)
        self.create_subscription(String, '/control/tunnel_source', self.cb_tunnel_source, 1)
        self.create_subscription(String, '/nav2/status', self.cb_nav2_status, 1)
        self.create_subscription(Log, '/rosout', self.cb_rosout, 50)
        self.create_subscription(LaneMask, '/detect/lane_mask', self.cb_lane_mask, 1)

        # --- Publisher ---
        # Image, nicht CompressedImage auf .../compressed: rqt_image_view abonniert das
        # Topic teils als Image, teils als CompressedImage und bricht dann mit
        # "incompatible type" ab. nav2_view und rqt laufen beide auf dem Laptop, das
        # Bild geht nicht uebers Netz.
        self.pub_view = self.create_publisher(Image, '/nav2/view', 1)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / self.get_parameter('view_rate').value, self.cb_timer)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._lut = cost_lut()
        self.footprint = None
        self.active_path = None
        self.entrance = None
        self.scan = None
        self.control_mode = None
        self.tunnel_source = ''
        self.nav2_status = ''
        self.reason = None          # (Zeit, Knoten, Text)
        self.lane = None            # (Empfangszeit, gelb (N,2), weiss (N,2)) in base_footprint [m]
        try:
            self._bev = bev_geometry.load_bev_spec(
                os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter'))
        except Exception as e:  # noqa: BLE001 -- ohne bev.yaml nur keine Spurlinien
            self._bev = None
            self.get_logger().warn(f'bev.yaml nicht ladbar ({e}) -- keine Spurlinien in der Ansicht.')

        self.get_logger().info('Started nav2_view Node.')

    # --- Callbacks ---

    def cb_lane_mask(self, msg):
        # Im Tunnel keine Kameradaten, auch nicht zur Anzeige (Reglement).
        if self.control_mode == ControlType.Tunnel:
            return
        b = self._bev
        if b is None or (msg.image_width, msg.image_height) != (b['width'], b['height']):
            return

        def to_m(us, vs):
            return np.column_stack(bev_pixels_to_ground(b, us[::3], vs[::3]))
        self.lane = (self.get_clock().now(), to_m(msg.yellow_x, msg.yellow_y),
                     to_m(msg.white_x, msg.white_y))

    def cb_footprint(self, msg):
        self.footprint = msg

    def cb_active_path(self, msg):
        self.active_path = msg

    def cb_entrance(self, cloud):
        pts = [(p[0], p[1]) for p in point_cloud2.read_points(
            cloud, field_names=('x', 'y'), skip_nans=True)]
        self.entrance = (cloud.header.frame_id, pts)

    def cb_scan(self, msg):
        self.scan = msg

    def cb_control_mode(self, msg):
        self.control_mode = control_mode_from_msg(self, msg)
        if self.control_mode == ControlType.Tunnel:
            self.lane = None

    def cb_tunnel_source(self, msg):
        self.tunnel_source = msg.data

    def cb_nav2_status(self, msg):
        self.nav2_status = msg.data

    def cb_rosout(self, log):
        # Log.WARN ist in Humble ein bytes-Wert (b'\x1e'), log.level dagegen ein int --
        # direkt verglichen wirft das TypeError und der Knoten stuerzt ab.
        warn = Log.WARN[0] if isinstance(Log.WARN, bytes) else Log.WARN
        if log.level < warn:
            return
        # Costmap-Knoten loggen z.B. als "local_costmap.local_costmap"
        node = log.name.split('.')[0]
        if node in self.get_parameter('reason_nodes').value:
            self.reason = (self.get_clock().now(), node, log.msg)

    def cb_timer(self):
        if self.pub_view.get_subscription_count() == 0:
            return

        in_tunnel = self.control_mode == ControlType.Tunnel
        grid_name = 'global_costmap' if in_tunnel else 'local_costmap'
        grid = self._grids.get(grid_name)

        canvas_size = int(self.get_parameter('canvas_size').value)
        if grid is None or grid.info.width == 0:
            body = np.full((canvas_size, canvas_size, 3), UNKNOWN, np.uint8)
            cv2.putText(body, f'keine Karte von /{grid_name}/costmap', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            self.publish(self.compose(body, grid_name, None))
            return

        info = grid.info
        if grid.header.frame_id != self.get_parameter('odom_frame').value:
            self.get_logger().warn(
                f'/{grid_name}/costmap liegt in {grid.header.frame_id}, erwartet '
                f'{self.get_parameter("odom_frame").value} -- Zeichnung kann verschoben sein.',
                throttle_duration_sec=10.0)

        cells = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
        r0, r1, c0, c1 = self.view_window(cells, info, in_tunnel)
        cells = cells[r0:r1, c0:c1]
        h, w = cells.shape
        res = info.resolution
        ox = info.origin.position.x + c0 * res
        oy = info.origin.position.y + r0 * res

        img = self._lut[np.clip(cells, -1, 100) + 1]
        # Zeile 0 der Karte liegt bei origin.y (unten) -- im Bild nach unten.
        img = np.flipud(img)
        scale = max(1, canvas_size // max(w, h))
        body = cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)

        def px(x, y):
            u = (x - ox) / res * scale
            v = (h - (y - oy) / res) * scale
            return (int(round(u)), int(round(v)))

        self.draw_lane(body, px)
        self.draw_scan(body, px)
        self.draw_entrance(body, px)
        # Lane-Modus: nav2_goal_publisher hat den Pfad beim Verlassen geleert.
        self.draw_path(body, px, self.active_path)
        self.draw_robot(body, px)

        self.publish(self.compose(body, grid_name, (res, scale)))

    # --- Hilfsmethoden ---

    def view_window(self, cells, info, in_tunnel):
        """Zeilen/Spalten (r0, r1, c0, c1) des gezeigten Ausschnitts der Karte."""
        full = (0, info.height, 0, info.width)
        res = info.resolution

        if in_tunnel:
            known = np.argwhere(cells >= 0)
            if known.size == 0:
                return full
            m = int(round(self.get_parameter('tunnel_view_margin_m').value / res))
            (r0, c0), (r1, c1) = known.min(axis=0) - m, known.max(axis=0) + 1 + m
        else:
            pose = self.lookup(self.get_parameter('robot_frame').value)
            if pose is None:
                return full
            half = int(round(self.get_parameter('local_view_size_m').value / 2.0 / res))
            rc = int((pose[1] - info.origin.position.y) / res)
            cc = int((pose[0] - info.origin.position.x) / res)
            r0, r1, c0, c1 = rc - half, rc + half, cc - half, cc + half

        r0, c0 = max(0, r0), max(0, c0)
        r1, c1 = min(info.height, r1), min(info.width, c1)
        if r1 - r0 < 2 or c1 - c0 < 2:
            return full
        return int(r0), int(r1), int(c0), int(c1)

    def lookup(self, frame):
        """2D-Pose (x, y, yaw) von frame im odom-Frame, neueste TF, oder None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter('odom_frame').value, frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quaternion(tf.transform.rotation)

    def draw_scan(self, body, px):
        if self.scan is None:
            return
        pose = self.lookup(self.scan.header.frame_id)
        if pose is None:
            return
        sx, sy, syaw = pose
        r = np.asarray(self.scan.ranges, dtype=np.float64)
        a = self.scan.angle_min + np.arange(r.size) * self.scan.angle_increment
        ok = np.isfinite(r) & (r > self.scan.range_min) & (r < self.scan.range_max)
        for ri, ai in zip(r[ok], a[ok]):
            x = sx + ri * math.cos(syaw + ai)
            y = sy + ri * math.sin(syaw + ai)
            cv2.circle(body, px(x, y), 2, SCAN, -1)

    def draw_lane(self, body, px):
        """Erkannte Spurlinien in ihrer Farbe ueber die Karte legen."""
        if self.lane is None or self.control_mode == ControlType.Tunnel:
            return
        stamp, yellow, white = self.lane
        if (self.get_clock().now() - stamp).nanoseconds / 1e9 > LANE_FRESH_S:
            return
        pose = self.lookup(self.get_parameter('robot_frame').value)
        if pose is None:
            return
        rx, ry, ryaw = pose
        c, s_ = math.cos(ryaw), math.sin(ryaw)
        h, w = body.shape[:2]
        for pts, col in ((yellow, LANE_YELLOW), (white, LANE_WHITE)):
            for X, Y in pts:
                u, v = px(rx + X * c - Y * s_, ry + X * s_ + Y * c)
                if 0 <= u < w and 0 <= v < h:
                    cv2.circle(body, (u, v), 2, col, -1)

    def draw_entrance(self, body, px):
        if self.control_mode != ControlType.Tunnel or not self.entrance:
            return
        frame, pts = self.entrance
        if len(pts) < 2 or frame != self.get_parameter('odom_frame').value:
            return
        line = np.array([px(x, y) for x, y in pts], np.int32)
        cv2.polylines(body, [line], False, WALL, 3, cv2.LINE_AA)

    def draw_path(self, body, px, path):
        if path is None or len(path.poses) < 2:
            return
        if path.header.frame_id != self.get_parameter('odom_frame').value:
            return
        line = np.array([px(p.pose.position.x, p.pose.position.y) for p in path.poses], np.int32)
        cv2.polylines(body, [line], False, PATH, 2, cv2.LINE_AA)
        gx, gy = line[-1]
        cv2.drawMarker(body, (int(gx), int(gy)), PATH, cv2.MARKER_TILTED_CROSS, 18, 2)

    def draw_robot(self, body, px):
        pose = self.lookup(self.get_parameter('robot_frame').value)
        fp = self.footprint
        if fp is not None and fp.header.frame_id != self.get_parameter('odom_frame').value:
            fp = None
        if fp is not None and len(fp.polygon.points) >= 3:
            poly = np.array([px(p.x, p.y) for p in fp.polygon.points], np.int32)
            cv2.polylines(body, [poly], True, ROBOT, 2, cv2.LINE_AA)
        elif pose is not None:
            # Ohne published_footprint wenigstens die Position zeigen.
            cv2.circle(body, px(pose[0], pose[1]), 6, ROBOT, 2, cv2.LINE_AA)
        if pose is not None:
            x, y, yaw = pose
            tip = (x + 0.08 * math.cos(yaw), y + 0.08 * math.sin(yaw))
            cv2.arrowedLine(body, px(x, y), px(*tip), HEADING, 2, cv2.LINE_AA, tipLength=0.4)

    def compose(self, body, grid_name, scale_info):
        """Statuszeile oben, Kartenname und Massstab unten."""
        w = body.shape[1]
        header = np.full((HEADER_H, w, 3), (40, 40, 40), np.uint8)

        mode = self.control_mode.name if self.control_mode is not None else 'kein Mode'
        if self.control_mode in (ControlType.TunnelEntry, ControlType.Tunnel) and self.tunnel_source:
            mode += f' ({self.tunnel_source})'
        line1 = f'{mode} | Nav2: {self.nav2_status or "-"}'
        cv2.putText(header, self.fit(line1, w), (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        if self.reason is None:
            line2, color = 'keine Warnung', (150, 150, 150)
        else:
            stamp, node, text = self.reason
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            line2 = f'vor {age:.0f}s {node}: {text}'
            color = (80, 80, 255) if age < REASON_FRESH_S else (150, 150, 150)
        cv2.putText(header, self.fit(line2, w), (8, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        footer = np.full((FOOTER_H, w, 3), (40, 40, 40), np.uint8)
        cv2.putText(footer, f'/{grid_name}/costmap', (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        if scale_info is not None:
            res, scale = scale_info
            bar = int(round(0.5 / res * scale))        # 0,5 m
            x1 = w - 12
            cv2.line(footer, (x1 - bar, 14), (x1, 14), (255, 255, 255), 2)
            cv2.putText(footer, '0,5 m', (x1 - bar - 52, 19),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        legend = np.full((FOOTER_H, w, 3), (40, 40, 40), np.uint8)
        x = 8
        for col, text in ((FREE, 'befahrbar'), (BLOCKED, 'blockiert'), (UNKNOWN, 'unbekannt'),
                          (LANE_YELLOW, 'gelb'), (LANE_WHITE, 'weiss'), (PATH, 'Pfad'),
                          (ROBOT, 'Bot')):
            cv2.rectangle(legend, (x, 7), (x + 12, 19), col, -1)
            cv2.putText(legend, text, (x + 16, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (220, 220, 220), 1, cv2.LINE_AA)
            x += 22 + cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)[0][0]
            if x > w - 40:
                break
        return np.vstack([header, body, footer, legend])

    @staticmethod
    def fit(text, width, scale=0.55):
        """Text kuerzen, bis er in die Bildbreite passt."""
        while text and cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] > width - 16:
            text = text[:-2]
        return text

    def publish(self, img):
        msg = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(img), encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('odom_frame').value
        self.pub_view.publish(msg)

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down nav2_view Node.')


def main(args=None):
    run_node(nav2_view, args)


if __name__ == '__main__':
    main()
