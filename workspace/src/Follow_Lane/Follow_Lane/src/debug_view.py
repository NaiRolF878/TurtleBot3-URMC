"""Debugbild mit Spurerkennung, Fahrmodus und Tunnel-Button.

Ein:  /detect/lane_debug, /control/mode, /control/tunnel_source, /control/cmd_vel, /odom,
      /battery_state, /detect/lane_obstacles, /detect/timing, /control/obstacle_enabled,
      Klicks aus rqt_image_view
Aus:  /lokal/debug_view (Image), /control/tunnel_request (Bool, Button)
"""
import datetime as dt
import json
import math
import time
from collections import defaultdict
import os

import cv2
import numpy as np

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PolygonStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage, Image, PointCloud2
from std_msgs.msg import Bool, Int32, String, UInt8
from vision_msgs.msg import Detection2DArray

from follow_lane_msg.msg import FollowLaneDebug
from Follow_Lane.src import bev_geometry
from Follow_Lane.src.common import ControlType, control_mode_from_msg, run_node
from Follow_Lane.src.ingress_node_nav2 import CALIBRATION, EXTRINSICS, FISHEYE_BALANCE

BATT_FULL_V = 12.6    # 4.2 V/Zelle
BATT_EMPTY_V = 11.0

# Kopfzeile ueber dem Kamerabild und Breite der beiden Panels daneben; daraus
# ergibt sich, wo der Tunnel-Button im fertigen Bild liegt.
HEAD_H = 40
PANEL_W = 230
# Tunnel-Button im Mode-Panel (Panel-Koordinaten: x0, y0, x1, y1).
TUNNEL_BUTTON = (8, 290, PANEL_W - 10, 345)

# Groesse (h, w) der Kameraflaeche, solange noch kein Bild kam: /lokal/bev, 640x440.
DEFAULT_CAMERA_SIZE = (440, 640)
# Ohne neues Spurbild (Tunnel, detect_lane steht) baut der Timer das Debugbild mit
# Platzhalter neu auf, damit Modus und Tunnel-Button aktuell bleiben. Ab
# LANE_DEBUG_STALE_S (s) gilt das letzte Spurbild als veraltet.
REDRAW_PERIOD_S = 0.5
LANE_DEBUG_STALE_S = 1.0

# Farbe pro Fahrmodus (BGR), damit auf einen Blick erkennbar ist, wer gerade faehrt.
MODE_COLOR = {
    ControlType.Lane:         (135, 250, 159),   # gruen
    ControlType.Obstacle:     (0, 165, 255),     # orange
    ControlType.TunnelEntry:  (255, 200, 0),     # cyan
    ControlType.Tunnel:       (255, 200, 0),
    ControlType.Stop:         (60, 60, 255),     # rot
    ControlType.Intersection: (255, 0, 255),     # magenta
    ControlType.Align:        (200, 200, 200),
}

TUNNEL_MODES = (ControlType.TunnelEntry, ControlType.Tunnel)

# YOLO-Boxen nur zeichnen, wenn sie hoechstens so viel (s) vom Rohbild abweichen.
OBJECTS_MAX_DT_S = 0.3
# Farbe (BGR) je YOLO-Klasse, unbekannte grau.
OBJECT_COLOR = {
    'tb':         (255, 80, 0),      # blau
    'park':       (255, 200, 0),
    'tunnel':     (200, 120, 255),
    'stop':       (40, 40, 230),     # rot
    'left':       (0, 200, 255),
    'right':      (0, 140, 255),
    'work':       (0, 255, 255),     # gelb
    'start_line': (60, 255, 180),
    'no_entry':   (120, 0, 200),
}

# Zeitmessung von detect_lane (/detect/timing): Grenzen (ms) gruen/gelb, darueber rot.
# Alter = Kamera-Stempel bis Ankunft (braucht synchrone Bot-Uhr), Luecke = groesster Abstand
# zweier Bilder (Ruckler). Aelter als TIMING_STALE_S -> Anzeige grau.
TIMING_AGE_MS = (150.0, 300.0)
TIMING_GAP_MS = (80.0, 200.0)
TIMING_CALC_MS = (25.0, 45.0)
TIMING_STALE_S = 3.0
# Eigene Messung (Anzeige, Punktversatz) jede so-vielte Sekunde zusammengefasst ins Log.
VIEW_LOG_EVERY = 5
# Ab so viel (ms) Alter laeuft eher die Bot-Uhr falsch als das Netz.
CLOCK_SUSPECT_MS = 5000.0

# Klickposition aus rqt_image_view ("Publish click location"): <Bildtopic>_mouse_left.
CLICK_TOPIC = '/lokal/debug_view_mouse_left'

# Welche Quelle control_driving je Modus auf /control/cmd_vel durchreicht; in allen
# anderen Modi sendet er Null.
MODE_SOURCE = {
    ControlType.Lane:     'PID',
    ControlType.Obstacle: 'Nav2/MPPI',
    ControlType.TunnelEntry: 'PID',
    ControlType.Tunnel:   'Nav2/MPPI',
    ControlType.Intersection: 'Kreuzung',
}


class debug_view(Node):
    """Im Tunnel kein Kamerabild (Reglement): detect_lane pausiert dort, und ein
    zwischengespeichertes Bild wird beim Eintritt verworfen.

    Der Tunnel-Button kommt als Klickposition von rqt_image_view ("Publish click
    location" anhaken, Topic <Bildtopic>_mouse_left).
    """

    def __init__(self):
        super().__init__('debug_view')

        # --- Subscriber ---
        self.sub_lane_debug = self.create_subscription(
            FollowLaneDebug, '/detect/lane_debug', self.cb_lane_debug, 1)
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_tunnel_source = self.create_subscription(
            String, '/control/tunnel_source', self.cb_tunnel_source, 1)
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/control/cmd_vel', self.cb_cmd_vel, 1)
        self.sub_odom = self.create_subscription(Odometry, '/odom_filtered', self.cb_odom, 1)
        self.sub_battery_state = self.create_subscription(
            BatteryState, '/battery_state', self.cb_battery_state, qos_profile_sensor_data)
        # Nur zum Mitzaehlen: bestaetigt im Debugbild, dass die Kette
        # detect_lane -> lane_costmap_node -> Costmap Daten liefert.
        self.sub_lane_obstacles = self.create_subscription(
            PointCloud2, '/detect/lane_obstacles', self.cb_lane_obstacles, 1)
        self.sub_click = self.create_subscription(
            Point, CLICK_TOPIC, self.cb_click, 1)
        # Schranke: Zahl der blauen Pixel im Suchbereich und ob die Schwelle erreicht ist.
        self.sub_barrier_pixels = self.create_subscription(
            Int32, '/detect/barrier_pixels', self.cb_barrier_pixels, 1)
        self.sub_barrier = self.create_subscription(Bool, '/detect/barrier', self.cb_barrier, 1)
        # Rohbild-Ansicht: Schranken-Debugbild (Rohbild mit blauer Maske) als Grundlage, dazu
        # die Kreuzungserkennung aus der BEV ins Rohbild projiziert.
        self.sub_barrier_debug = self.create_subscription(
            CompressedImage, '/detect/barrier_debug/compressed', self.cb_barrier_debug, 1)
        self.sub_intersection = self.create_subscription(
            PolygonStamped, '/detect/intersection', self.cb_intersection, 1)
        # YOLO-Boxen (Pixel im Rohbild) fuer die Rohbild-Ansicht.
        self.sub_objects = self.create_subscription(
            Detection2DArray, '/detect/objects', self.cb_objects, 1)
        # Phase von intersection_node (Anfahrt/Halt/Queren) fuers Mode-Panel.
        self.sub_intersection_state = self.create_subscription(
            String, '/intersection/state', self.cb_intersection_state, 1)
        # Bildalter, Raten, Rechenzeiten (detect_lane) als Tabelle im Kamerabild.
        self.sub_timing = self.create_subscription(String, '/detect/timing', self.cb_timing, 1)
        # Obstacle erlaubt (Taste O / rqt) -- Schild in der Kopfzeile.
        self.sub_obstacle_enabled = self.create_subscription(
            Bool, '/control/obstacle_enabled', self.cb_obstacle_enabled, 1)

        # --- Publisher ---
        self.pub_debug_image = self.create_publisher(Image, '/lokal/debug_view', 1)
        self.pub_debug_raw = self.create_publisher(Image, '/lokal/debug_view_raw', 1)
        self.pub_tunnel_request = self.create_publisher(Bool, '/control/tunnel_request', 1)

        # --- Timer ---
        self.timer = self.create_timer(REDRAW_PERIOD_S, self.cb_timer)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.follow_lane_debug = None
        self.lane_debug_stamp = None
        self.camera_size = DEFAULT_CAMERA_SIZE
        self.control_mode = None
        self.tunnel_source = ''
        self.target_v = None
        self.actual_v = None
        self.battery_level = None
        self.lane_obstacle_points = 0
        self.lane_obstacle_stamp = None
        self.barrier_pixels = None
        self.barrier = False
        self.intersection = None          # bestaetigtes Kreuzungsfeld (N, 2) in BEV-Pixeln
        self.intersection_state = ''      # Phase von intersection_node
        self.timing = None                # letztes /detect/timing (dict)
        self.obstacle_enabled = None      # None = von switch_control noch nichts gehoert
        self._timing_time = None          # monotonic beim Empfang
        # Eigene Messung je Ansicht ('view' = BEV-Debugbild, 'rawview' = Rohbild-Debug):
        # Alter des gezeigten Bilds beim Senden, Abstand der Sendungen, im Rohbild zusaetzlich
        # der Versatz der Spurpunkte (BEV-Bild) zum gezeigten Rohbild. 1x/s zusammengefasst.
        self._view_samples = defaultdict(list)
        self._view_last = {}
        self.view_stats = {}
        self.timer_view_stats = self.create_timer(1.0, self.cb_view_stats)
        self._view_log = defaultdict(list)   # dasselbe ueber VIEW_LOG_EVERY Sekunden
        self._view_count = 0
        self.objects = None               # letztes Detection2DArray (YOLO)
        self._bev_to_raw = None           # 3x3 BEV-Pixel -> Rohbild-Pixel (entzerrt)
        self._bev_to_raw_size = None
        try:
            self._bev_spec = bev_geometry.load_bev_spec(
                os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter'))
        except Exception:  # noqa: BLE001 -- ohne bev.yaml kein Abstand zur Kreuzung
            self._bev_spec = None
        # Lage des Tunnel-Buttons im zuletzt gesendeten Bild (x0, y0, x1, y1).
        self._tunnel_button = None
        self.battery_icons = self._load_battery_icons()

        self.get_logger().info('Started debug_view Node.')

    # --- Callbacks ---

    def cb_lane_debug(self, msg):
        if self.control_mode == ControlType.Tunnel:
            return
        self.follow_lane_debug = msg
        self.lane_debug_stamp = self.get_clock().now()
        self.build_debug_image()

    def cb_control_mode(self, msg):
        self.control_mode = control_mode_from_msg(self, msg)
        if self.control_mode == ControlType.Tunnel:
            self.follow_lane_debug = None

    def cb_tunnel_source(self, msg):
        self.tunnel_source = msg.data

    def cb_cmd_vel(self, twist):
        self.target_v = twist.linear.x

    def cb_odom(self, odom):
        self.actual_v = odom.twist.twist.linear.x

    def cb_barrier_pixels(self, msg):
        self.barrier_pixels = msg.data

    def cb_barrier(self, msg):
        self.barrier = msg.data

    def cb_obstacle_enabled(self, msg):
        self.obstacle_enabled = msg.data

    def cb_timing(self, msg):
        try:
            self.timing = json.loads(msg.data)
        except ValueError:
            return
        self._timing_time = time.monotonic()

    def cb_intersection_state(self, msg):
        self.intersection_state = msg.data

    def cb_intersection(self, msg):
        pts = [(p.x, p.y) for p in msg.polygon.points]
        self.intersection = np.array(pts, np.float64) if len(pts) >= 3 else None

    def cb_objects(self, msg):
        self.objects = msg

    def cb_barrier_debug(self, msg):
        if self.pub_debug_raw.get_subscription_count() == 0:
            return
        try:
            raw = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001 -- kaputtes Einzelbild nur melden
            self.get_logger().error(f'Rohbild-Debug nicht dekodierbar: {e}',
                                    throttle_duration_sec=5.0)
            return
        self.draw_intersection_raw(raw)
        self.draw_objects_raw(raw, msg.header.stamp)
        self.note_view('rawview', msg.header.stamp)
        d = self.follow_lane_debug
        if d is not None and d.raw_image.header.stamp.sec \
                and (self.lane_debug_age() or 1e9) <= LANE_DEBUG_STALE_S:
            # > 0: Punkte stammen aus einem aelteren Bild als das gezeigte Rohbild.
            self._view_samples['offset_ms'].append(
                (Time.from_msg(msg.header.stamp) - Time.from_msg(d.raw_image.header.stamp))
                .nanoseconds / 1e6)
        out = self.bridge.cv2_to_imgmsg(self.compose(raw, 'rawview'), encoding='bgr8')
        out.header = msg.header
        self.pub_debug_raw.publish(out)

    def cb_battery_state(self, state):
        self.battery_level = self.battery_percent(state)

    def cb_lane_obstacles(self, cloud):
        # width * height = Punktanzahl, ohne die Punkte dekodieren zu muessen.
        self.lane_obstacle_points = cloud.width * cloud.height
        self.lane_obstacle_stamp = self.get_clock().now()

    def cb_click(self, point):
        if self._tunnel_button is None or self.control_mode is None:
            return
        x0, y0, x1, y1 = self._tunnel_button
        if not (x0 <= point.x <= x1 and y0 <= point.y <= y1):
            return
        # Ausdruecklich Start oder Abbruch, kein Umschalten: ein doppelt gezaehlter
        # Klick darf den Tunnel nicht sofort wieder beenden.
        req = Bool(data=self.control_mode not in TUNNEL_MODES)
        self.pub_tunnel_request.publish(req)
        self.get_logger().info('Tunnel-Button: ' + ('Start' if req.data else 'Abbruch'))

    def cb_timer(self):
        age = self.lane_debug_age()
        if age is None or age > LANE_DEBUG_STALE_S:
            self.build_debug_image()

    # --- Hilfsmethoden ---

    def lane_debug_age(self):
        """Sekunden seit dem letzten Spurbild, None ohne Bild."""
        if self.lane_debug_stamp is None or self.follow_lane_debug is None:
            return None
        return (self.get_clock().now() - self.lane_debug_stamp).nanoseconds / 1e9

    def build_debug_image(self):
        camera = self.camera_view()
        if camera is None:
            return
        fresh = (self.lane_debug_age() or 1e9) <= LANE_DEBUG_STALE_S
        if fresh and self.follow_lane_debug.raw_image.header.stamp.sec:
            self.note_view('view', self.follow_lane_debug.raw_image.header.stamp)
        final_debug_image = self.compose(camera, 'view')
        if self.control_mode is not None:
            bx0, by0, bx1, by1 = TUNNEL_BUTTON
            off_x, off_y = camera.shape[1] + PANEL_W, HEAD_H
            self._tunnel_button = (off_x + bx0, off_y + by0, off_x + bx1, off_y + by1)

        msg = self.bridge.cv2_to_imgmsg(final_debug_image, encoding='bgr8')
        age = self.lane_debug_age()
        if age is not None and age <= LANE_DEBUG_STALE_S:
            msg.header = self.follow_lane_debug.raw_image.header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_debug_image.publish(msg)

    def compose(self, camera, view):
        """Kopfzeile (Datum, Akku) ueber Kamerabild und den beiden Seitenpanels.
        view: 'view' (BEV-Debugbild) oder 'rawview' (Rohbild-Debug), fuer die Zeittabelle."""
        cam_h, cam_w = camera.shape[:2]
        self.draw_timing(camera, view)
        panel_generic = self.build_generic_panel(cam_h)
        panel_mode = self.build_mode_panel(cam_h, PANEL_W)

        total_w = cam_w + PANEL_W * 2
        head_panel = np.full((HEAD_H, total_w, 3), (220, 220, 220), dtype=np.uint8)
        now = dt.datetime.now()
        cv2.putText(head_panel, now.strftime('%d.%m.%Y'), (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(head_panel, now.strftime('%H:%M'), (200, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
        self.draw_battery(head_panel, total_w)
        self.draw_obstacle_badge(head_panel)
        return np.vstack([head_panel, np.hstack([camera, panel_generic, panel_mode])])

    def note_view(self, name, stamp):
        """Alter des gezeigten Bilds (Kamera-Stempel) beim Senden und Abstand der Sendungen."""
        now = time.monotonic()
        if name in self._view_last:
            self._view_samples[name + '_gap_ms'].append((now - self._view_last[name]) * 1000.0)
        self._view_last[name] = now
        self._view_samples[name + '_age_ms'].append(
            (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e6)

    @staticmethod
    def view_stats_of(samples, period):
        stats = {k: [round(float(np.mean(v)), 1), round(float(np.max(v)), 1)]
                 for k, v in samples.items() if v}
        for name in ('view', 'rawview'):
            stats[name + '_hz'] = len(samples.get(name + '_age_ms', ())) / period
        return stats

    def cb_view_stats(self):
        self.view_stats = self.view_stats_of(self._view_samples, 1.0)
        for key, values in self._view_samples.items():
            self._view_log[key].extend(values)
        self._view_samples.clear()
        self._view_count += 1
        if self._view_count % VIEW_LOG_EVERY:
            return
        st = self.view_stats_of(self._view_log, float(VIEW_LOG_EVERY))
        self._view_log.clear()
        parts = []
        for name, label in (('view', 'Debugbild'), ('rawview', 'Rohbild-Debug')):
            age, gap = st.get(name + '_age_ms'), st.get(name + '_gap_ms')
            if age:
                parts.append(f'{label} {st[name + "_hz"]:.0f} Hz Alter {age[0]:.0f}/{age[1]:.0f}'
                             + (f' Luecke max {gap[1]:.0f}' if gap else ''))
        if 'offset_ms' in st:
            parts.append(f'Punkte aelter als Rohbild {st["offset_ms"][0]:.0f}/'
                         f'{st["offset_ms"][1]:.0f}')
        if parts:
            self.get_logger().info('Anzeige (ms, Mittel/max): ' + ' | '.join(parts))

    def draw_timing(self, img, view):
        """Tabelle unten links, je Stufe der Kette Alter (Mittel/max), Rate, groesste Luecke
        und Rechenzeit: Kamera (Rohbild in detect_lane), Spur (BEV), PID (Spurfehler
        gesendet), YOLO (Boxen gesendet) aus /detect/timing; Anzeige = dieses Debugbild.
        Im Rohbild zusaetzlich, wie viel aelter die Spurpunkte als das gezeigte Bild sind."""
        if self.timing is None:
            return
        stale = time.monotonic() - self._timing_time > TIMING_STALE_S
        t = dict(self.timing, **self.view_stats)

        def cell(key, limits, idx=0):
            v = t.get(key)
            if v is None:
                return '--', (150, 150, 150)
            v = v[idx]
            if stale:
                return f'{v:.0f}', (150, 150, 150)
            color = (0, 220, 0) if v <= limits[0] else (0, 220, 255) if v <= limits[1] \
                else (60, 60, 255)
            return f'{v:.0f}', color

        rows = [('Kamera', 'raw', 'raw_hz', None), ('Spur', 'bev', 'bev_hz', 'lane_ms'),
                ('PID', 'pid', 'pid_hz', None), ('YOLO', 'yolo', 'yolo_hz', 'yolo_ms'),
                ('Anzeige', view, view + '_hz', None)]
        font, scale = cv2.FONT_HERSHEY_SIMPLEX, 0.45
        cols = [8, 74, 154, 222, 262, 350]          # Name, Alter, max, Hz, Luecke, Rechnen
        line_h = 18
        h = line_h * (len(rows) + 1) + 10
        y0 = img.shape[0] - h - 4
        box = img[y0:y0 + h, 4:4 + 440]
        box[:] = (box * 0.35).astype(np.uint8)
        head = ['', 'Alter ms', 'max ms', 'Hz', 'Luecke ms', 'Rechnen ms']
        for x, text in zip(cols, head):
            cv2.putText(img, text, (4 + x, y0 + 15), font, scale, (200, 200, 200), 1, cv2.LINE_AA)
        clock_bad = False
        for i, (name, src, hz_key, calc_key) in enumerate(rows):
            y = y0 + 15 + line_h * (i + 1)
            age_key = 'yolo_age_ms' if src == 'yolo' else f'{src}_age_ms'
            if (t.get(age_key) or [0])[0] > CLOCK_SUSPECT_MS:
                clock_bad = True
            hz = t.get(hz_key)
            cells = [(name, (230, 230, 230)), cell(age_key, TIMING_AGE_MS),
                     cell(age_key, TIMING_AGE_MS, 1),
                     ('--' if hz is None else f'{hz:.0f}', (230, 230, 230)),
                     cell(f'{src}_gap_ms', TIMING_GAP_MS, 1) if src != 'yolo' else ('', None),
                     cell(calc_key, TIMING_CALC_MS) if calc_key else ('', None)]
            for x, (text, color) in zip(cols, cells):
                if text:
                    cv2.putText(img, text, (4 + x, y), font, scale, color, 1, cv2.LINE_AA)
        if view == 'rawview' and 'offset_ms' in t:
            text, color = cell('offset_ms', TIMING_GAP_MS)
            text_max, _ = cell('offset_ms', TIMING_GAP_MS, 1)
            cv2.putText(img, f'Punkte {text} ms (max {text_max}) aelter als Bild', (8, y0 - 6),
                        font, 0.5, color, 1, cv2.LINE_AA)
            y0 -= 20
        if clock_bad:
            cv2.putText(img, 'Bot-Uhr falsch? scripts/tb/sync-time.sh', (8, y0 - 6), font, 0.5,
                        (60, 60, 255), 1, cv2.LINE_AA)

    def bev_to_raw(self, shape):
        """3x3 BEV-Pixel -> Pixel im entzerrten Rohbild dieser Groesse; None ohne
        Kalibrierung. Dieselbe Rechnung wie ingress_node_nav2 (gleiche Dateien, balance)."""
        h, w = shape[:2]
        if self._bev_to_raw_size == (w, h):
            return self._bev_to_raw
        self._bev_to_raw_size = (w, h)
        self._bev_to_raw = None
        pdir = os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter')
        try:
            self._bev_to_raw, self._bev_spec = bev_geometry.load_bev_to_image(
                pdir, CALIBRATION, EXTRINSICS, FISHEYE_BALANCE, (w, h))
        except Exception as e:  # noqa: BLE001 -- ohne Kalibrierung keine Projektion
            self.get_logger().warn(f'Keine Kalibrierung fuers Rohbild-Debug ({e}).')
            return None
        return self._bev_to_raw

    def draw_intersection_raw(self, raw):
        """Kreuzungserkennung (Laengs-/Querkanten, Haltelinie, Ecken, Felder) und die
        verfolgten Spurpunkte aus der BEV ins Rohbild zeichnen; bestaetigte Kreuzung mit
        Rahmen und Abstand."""
        M = self.bev_to_raw(raw.shape)
        d = self.follow_lane_debug
        if M is None or d is None or (self.lane_debug_age() or 1e9) > LANE_DEBUG_STALE_S:
            return

        def proj(pts):
            pts = np.asarray(pts, np.float64).reshape(-1, 1, 2)
            return cv2.perspectiveTransform(pts, M).reshape(-1, 2).astype(np.int32)

        for poly in d.polygons:
            pts = proj([[p.x, p.y] for p in poly.points])
            cv2.polylines(raw, [pts], True, (0, 0, 255), 2, cv2.LINE_AA)
        for lines, color in ((d.horizontal_lines, (144, 238, 144)),
                             (d.vertical_lines, (240, 16, 255))):
            for line in lines:
                a, b = proj([[line.points[0].x, line.points[0].y],
                             [line.points[1].x, line.points[1].y]])
                cv2.line(raw, tuple(map(int, a)), tuple(map(int, b)), color, 3, cv2.LINE_AA)
        for (x, y), color in (((d.center_yellow_x, d.center_yellow_y), (0, 255, 255)),
                              ((d.center_white_x, d.center_white_y), (255, 255, 255))):
            p = proj([[x, y]])[0]
            cv2.circle(raw, (int(p[0]), int(p[1])), 7, color, -1, cv2.LINE_AA)

        if self.intersection is not None:
            box = proj(self.intersection)
            x, y, bw, bh = cv2.boundingRect(box)
            cv2.rectangle(raw, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
            dist = self.intersection_distance()
            label = 'KREUZUNG' + ('' if dist is None else f' {dist * 100:.0f} cm')
            cv2.putText(raw, label, (x, max(25, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0), 2, cv2.LINE_AA)

    def draw_objects_raw(self, raw, stamp):
        """YOLO-Boxen mit Klasse und Konfidenz; nur wenn sie zum Rohbild passen (Zeit)."""
        objs = self.objects
        if objs is None:
            return
        dt = abs((objs.header.stamp.sec - stamp.sec) + (objs.header.stamp.nanosec - stamp.nanosec) * 1e-9)
        if dt > OBJECTS_MAX_DT_S:
            return
        h, w = raw.shape[:2]
        for det in objs.detections:
            if not det.results:
                continue
            name = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            x0 = int(max(0, cx - det.bbox.size_x / 2))
            y0 = int(max(0, cy - det.bbox.size_y / 2))
            x1 = int(min(w - 1, cx + det.bbox.size_x / 2))
            y1 = int(min(h - 1, cy + det.bbox.size_y / 2))
            color = OBJECT_COLOR.get(name, (160, 160, 160))
            cv2.rectangle(raw, (x0, y0), (x1, y1), color, 2)
            label = f'{name} {score:.2f}'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            ty = y0 - 4 if y0 - th - 6 >= 0 else y0 + th + 4
            cv2.rectangle(raw, (x0, ty - th - 3), (x0 + tw + 4, ty + 3), color, -1)
            cv2.putText(raw, label, (x0 + 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1,
                        cv2.LINE_AA)

    def intersection_distance(self):
        """Abstand (m) der Haltelinie (Unterkante des Kreuzungsfelds) ab Radachse."""
        if self.intersection is None or self._bev_spec is None:
            return None
        v = float(self.intersection[:, 1].max())
        return self._bev_spec['x_max'] - (v + 0.5) * self._bev_spec['res']

    def camera_view(self):
        """Kamerabild mit Spurmarkierungen; im Tunnel, ohne Bild oder bei veraltetem
        Bild ein Platzhalter. None, wenn das Bild nicht dekodiert werden kann."""
        age = self.lane_debug_age()
        if self.control_mode == ControlType.Tunnel:
            text = 'Kamera aus (Tunnel)'
        elif age is None:
            text = 'kein Kamerabild'
        elif age > LANE_DEBUG_STALE_S:
            text = f'Spurbild veraltet ({age:.0f} s)'
        else:
            text = None
        if text is not None:
            placeholder = np.zeros((*self.camera_size, 3), dtype=np.uint8)
            cv2.putText(placeholder, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (200, 200, 200), 2, cv2.LINE_AA)
            return placeholder

        d = self.follow_lane_debug
        if not d.raw_image.data:
            self.get_logger().warn('Debug-Message enthaelt kein Bild.', throttle_duration_sec=5.0)
            return None
        try:
            raw = self.bridge.compressed_imgmsg_to_cv2(d.raw_image, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001 -- kaputtes Einzelbild nur melden
            self.get_logger().error(f'Debug-Bild konnte nicht dekodiert werden: {e}')
            return None
        raw_h, raw_w = raw.shape[:2]
        self.camera_size = (raw_h, raw_w)

        # Das Bild ist die Vogelperspektive (/lokal/bev), die Punkte liegen direkt in
        # seinen Pixeln. Fahrtlinie (Bildmitte = echte Fahrtrichtung) und Suchzeilen:
        cv2.line(raw, (raw_w // 2, 0), (raw_w // 2, raw_h - 1), (0, 200, 0), 1, cv2.LINE_AA)
        for y, col in ((d.center_yellow_y, (0, 255, 255)), (d.center_white_y, (255, 255, 255))):
            cv2.line(raw, (0, int(y)), (raw_w - 1, int(y)), col, 1, cv2.LINE_AA)
        # Abweichung: von der Fahrtlinie zur Spurmitte
        y_c = int((d.center_yellow_y + d.center_white_y) / 2)
        cv2.arrowedLine(raw, (raw_w // 2, y_c), (int(d.lane_center), y_c), (0, 0, 255), 2,
                        cv2.LINE_AA, tipLength=0.15)

        cv2.circle(raw, (int(d.center_yellow_x), int(d.center_yellow_y)), 6, (0, 255, 255), -1,
                   cv2.LINE_AA)
        cv2.circle(raw, (int(d.center_white_x), int(d.center_white_y)), 6, (255, 255, 255), -1,
                   cv2.LINE_AA)
        cv2.circle(raw, (int(d.lane_center), y_c), 6, (255, 255, 255), -1, cv2.LINE_AA)

        # Polygone aus detect_lane liegen schon in den Koordinaten von raw; OpenCV
        # braucht ein int32-Array der Form (N, 2).
        for poly in d.polygons:
            pts = np.array([[pt.x, pt.y] for pt in poly.points], dtype=np.int32)
            cv2.polylines(raw, [pts], isClosed=True, color=(0, 0, 255), thickness=2,
                          lineType=cv2.LINE_AA)

        # Waagerechte Kanten hellgruen, senkrechte neonpink (BGR)
        for lines, color in ((d.horizontal_lines, (144, 238, 144)),
                             (d.vertical_lines,   (240, 16, 255))):
            for line in lines:
                a, b = line.points[0], line.points[1]
                cv2.line(raw, (int(a.x), int(a.y)), (int(b.x), int(b.y)), color, 3, cv2.LINE_AA)
        return raw

    def build_generic_panel(self, height):
        def speed(v):
            return '--' if v is None else f'{v:.2f} m/s'

        panel = np.zeros((height, PANEL_W, 3), dtype=np.uint8)
        cv2.line(panel, (PANEL_W - 2, 5), (PANEL_W - 2, height - 5), thickness=1,
                 color=(90, 90, 90), lineType=cv2.LINE_AA)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(panel, 'TurtleBot', (8, 40), font, 1, (135, 250, 159), 2, cv2.LINE_AA)
        cv2.putText(panel, 'Speed', (8, 80), font, 1, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(panel, 'Target:', (8, 120), font, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(panel, speed(self.target_v), (8, 155), font, 1, (135, 250, 159), 2, cv2.LINE_AA)
        cv2.putText(panel, 'Actual:', (8, 195), font, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(panel, speed(self.actual_v), (8, 230), font, 1, (250, 135, 223), 2, cv2.LINE_AA)
        cv2.putText(panel, 'Lane error:', (8, 280), font, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        lane_error = '--' if self.follow_lane_debug is None \
            else f'{self.follow_lane_debug.lane_error:+.3f}'
        cv2.putText(panel, lane_error, (8, 315), font, 1, (80, 80, 255), 2, cv2.LINE_AA)
        cv2.putText(panel, 'Blaue Pixel:', (8, 365), font, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        blue = '--' if self.barrier_pixels is None else str(self.barrier_pixels)
        blue_color = (80, 80, 255) if self.barrier else (255, 160, 60)
        cv2.putText(panel, blue, (8, 400), font, 1, blue_color, 2, cv2.LINE_AA)
        return panel

    def build_mode_panel(self, height, width):
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.line(panel, (1, 5), (1, height - 5), thickness=1, color=(90, 90, 90),
                 lineType=cv2.LINE_AA)
        cv2.putText(panel, 'Mode', (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 1,
                    cv2.LINE_AA)

        if self.control_mode is None:
            cv2.putText(panel, '-- kein Mode --', (8, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1, cv2.LINE_AA)
            return panel

        color = MODE_COLOR.get(self.control_mode, (200, 200, 200))
        cv2.putText(panel, self.control_mode.name, (8, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 2, cv2.LINE_AA)
        if self.control_mode in TUNNEL_MODES and self.tunnel_source:
            label = self.tunnel_source
            cv2.putText(panel, f'({label})', (8, 106),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

        source = MODE_SOURCE.get(self.control_mode, 'Stopp (Null)')
        cv2.putText(panel, 'Quelle:', (8, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(panel, source, (8, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

        self.draw_lane_obstacle_status(panel)
        self.draw_tunnel_button(panel)
        cv2.putText(panel, 'Kreuzung:', (8, 375),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        dist = self.intersection_distance()
        text, ink = ('--', (120, 120, 120)) if dist is None else (f'{dist * 100:.0f} cm', (0, 255, 0))
        cv2.putText(panel, text, (8, 410), cv2.FONT_HERSHEY_SIMPLEX, 1, ink, 2, cv2.LINE_AA)
        if self.intersection_state and self.intersection_state != 'Frei':
            cv2.putText(panel, self.intersection_state, (8, 432), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, MODE_COLOR[ControlType.Intersection], 1, cv2.LINE_AA)
        return panel

    def draw_tunnel_button(self, panel):
        """Button zum Starten/Abbrechen des Tunnelmodus. Zeigt nur den Modus, springt
        nach dem automatischen Austritt also von selbst zurueck."""
        x0, y0, x1, y1 = TUNNEL_BUTTON
        in_tunnel = self.control_mode in TUNNEL_MODES
        fill = (60, 60, 200) if in_tunnel else MODE_COLOR[ControlType.Tunnel]
        ink = (255, 255, 255) if in_tunnel else (0, 0, 0)
        text = 'TUNNEL ABBRECHEN' if in_tunnel else 'TUNNEL STARTEN'

        cv2.rectangle(panel, (x0, y0), (x1, y1), fill, -1)
        cv2.rectangle(panel, (x0, y0), (x1, y1), (230, 230, 230), 1)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.putText(panel, text, (x0 + (x1 - x0 - tw) // 2, y0 + (y1 - y0 + th) // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, ink, 2, cv2.LINE_AA)

    def draw_lane_obstacle_status(self, panel):
        cv2.putText(panel, 'Spur -> Costmap:', (8, 215),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        if self.lane_obstacle_stamp is None:
            text, color = 'nie empfangen', (120, 120, 120)
        else:
            age = (self.get_clock().now() - self.lane_obstacle_stamp).nanoseconds / 1e9
            if age > 2.0:
                text, color = f'still ({age:.0f}s)', (60, 60, 255)
            else:
                text, color = f'{self.lane_obstacle_points} Pkt', (135, 250, 159)
        cv2.putText(panel, text, (8, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

    def battery_percent(self, state):
        if not math.isnan(state.percentage) and state.percentage > 0.0:
            pct = state.percentage
            return int(round(pct if pct > 1.0 else pct * 100))
        if math.isnan(state.voltage) or state.voltage <= 0.0:
            return None
        ratio = (state.voltage - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V)
        return int(round(max(0.0, min(1.0, ratio)) * 100))

    def draw_obstacle_badge(self, head_panel):
        """Schild 'OBSTACLE AN/AUS' rechts neben der Uhrzeit (Taste O / rqt)."""
        if self.obstacle_enabled is None:
            return
        text = 'OBSTACLE AN' if self.obstacle_enabled else 'OBSTACLE AUS'
        fill = MODE_COLOR[ControlType.Obstacle] if self.obstacle_enabled else (120, 120, 120)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        x0, y0 = 320, 6
        cv2.rectangle(head_panel, (x0, y0), (x0 + tw + 16, HEAD_H - 6), fill, -1)
        cv2.putText(head_panel, text, (x0 + 8, y0 + (HEAD_H - 12 + th) // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0) if self.obstacle_enabled
                    else (255, 255, 255), 2, cv2.LINE_AA)

    def draw_battery(self, head_panel, total_w):
        text = '--%' if self.battery_level is None else f'{self.battery_level}%'
        (text_w, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
        text_x = max(0, total_w - text_w - 10)
        cv2.putText(head_panel, text, (text_x, 33), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2,
                    cv2.LINE_AA)

        icon = self.battery_icon()
        if icon is None:
            return
        icon_h, icon_w = icon.shape[:2]
        x = text_x - icon_w - 8
        y = (head_panel.shape[0] - icon_h) // 2
        if x < 0 or y < 0 or icon_h > head_panel.shape[0]:
            return

        if icon.shape[2] == 4:
            alpha = icon[:, :, 3:4].astype(np.float32) / 255.0
            head_panel[y:y + icon_h, x:x + icon_w] = (
                icon[:, :, :3] * alpha
                + head_panel[y:y + icon_h, x:x + icon_w] * (1.0 - alpha)
            ).astype(np.uint8)
        else:
            head_panel[y:y + icon_h, x:x + icon_w] = icon[:, :, :3]

    def battery_icon(self):
        if self.battery_level is None:
            return None
        if self.battery_level >= 75:
            return self.battery_icons[3]
        if self.battery_level >= 50:
            return self.battery_icons[2]
        if self.battery_level >= 25:
            return self.battery_icons[1]
        return self.battery_icons[0]

    def _load_battery_icons(self):
        media_dir = os.path.join(get_package_share_directory('Follow_Lane'), 'media', 'Battery')
        icons = []
        for name in ('battery_0.png', 'battery_1.png', 'battery_2.png', 'battery_3.png'):
            path = os.path.join(media_dir, name)
            icon = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if icon is None:
                self.get_logger().warn(f'Batterie-Icon nicht gefunden: {path}')
            icons.append(icon)
        return icons

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down debug_view Node.')


def main(args=None):
    run_node(debug_view, args)


if __name__ == '__main__':
    main()
