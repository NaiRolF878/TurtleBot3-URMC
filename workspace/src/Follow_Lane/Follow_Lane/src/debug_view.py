"""Debugbild mit Spurerkennung, Fahrmodus und Tunnel-Button.

Ein:  /detect/lane_debug, /control/mode, /control/tunnel_source, /control/cmd_vel, /odom,
      /battery_state, /detect/lane_obstacles, Klicks aus rqt_image_view
Aus:  /lokal/debug_view (Image), /control/tunnel_request (Bool, Button)
"""
import datetime as dt
import math
import os

import cv2
import numpy as np

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Image, PointCloud2
from std_msgs.msg import Bool, String, UInt8

from follow_lane_msg.msg import FollowLaneDebug
from Follow_Lane.src.common import ControlType, control_mode_from_msg, run_node

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
# Ohne neues Kamerabild (Tunnel, detect_lane steht) baut der Timer das Debugbild mit
# Platzhalter neu auf, damit Modus und Tunnel-Button aktuell bleiben.
REDRAW_PERIOD_S = 0.5
LANE_DEBUG_STALE_S = 1.0

# Farbe pro Fahrmodus (BGR), damit auf einen Blick erkennbar ist, wer gerade faehrt.
MODE_COLOR = {
    ControlType.Lane:         (135, 250, 159),   # gruen
    ControlType.Obstacle:     (0, 165, 255),     # orange
    ControlType.TunnelEntry:  (255, 200, 0),     # cyan
    ControlType.Tunnel:       (255, 200, 0),
    ControlType.Stop:         (60, 60, 255),     # rot
    ControlType.Intersection: (200, 200, 200),
    ControlType.Align:        (200, 200, 200),
}

TUNNEL_MODES = (ControlType.TunnelEntry, ControlType.Tunnel)

# Welche Quelle control_driving je Modus auf /control/cmd_vel durchreicht; in allen
# anderen Modi sendet er Null.
MODE_SOURCE = {
    ControlType.Lane:     'PID',
    ControlType.Obstacle: 'Nav2/MPPI',
    ControlType.TunnelEntry: 'Nav2/MPPI',
    ControlType.Tunnel:   'Nav2/MPPI',
}


class debug_view(Node):
    """Im Tunnel kein Kamerabild (Reglement): detect_lane pausiert dort, und ein
    zwischengespeichertes Bild wird beim Eintritt verworfen.

    Der Tunnel-Button kommt als Klickposition von rqt_image_view ("Publish click
    location" anhaken, Topic <Bildtopic>_mouse_left).
    """

    def __init__(self):
        super().__init__('debug_view')

        # --- Parameter ---
        self.declare_parameter('click_topic', '/lokal/debug_view_mouse_left')

        # --- Subscriber ---
        self.sub_lane_debug = self.create_subscription(
            FollowLaneDebug, '/detect/lane_debug', self.cb_lane_debug, 1)
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_tunnel_source = self.create_subscription(
            String, '/control/tunnel_source', self.cb_tunnel_source, 1)
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/control/cmd_vel', self.cb_cmd_vel, 1)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, 1)
        self.sub_battery_state = self.create_subscription(
            BatteryState, '/battery_state', self.cb_battery_state, qos_profile_sensor_data)
        # Nur zum Mitzaehlen: bestaetigt im Debugbild, dass die Kette
        # detect_lane -> lane_costmap_node -> Costmap Daten liefert.
        self.sub_lane_obstacles = self.create_subscription(
            PointCloud2, '/detect/lane_obstacles', self.cb_lane_obstacles, 1)
        self.sub_click = self.create_subscription(
            Point, self.get_parameter('click_topic').value, self.cb_click, 1)

        # --- Publisher ---
        self.pub_debug_image = self.create_publisher(Image, '/lokal/debug_view', 1)
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
        if self.lane_debug_stamp is None or self.follow_lane_debug is None:
            self.build_debug_image()
            return
        age = (self.get_clock().now() - self.lane_debug_stamp).nanoseconds / 1e9
        if age > LANE_DEBUG_STALE_S:
            self.build_debug_image()

    # --- Hilfsmethoden ---

    def build_debug_image(self):
        camera = self.camera_view()
        if camera is None:
            return
        cam_h, cam_w = camera.shape[:2]

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

        final_debug_image = np.vstack([head_panel, np.hstack([camera, panel_generic, panel_mode])])

        if self.control_mode is not None:
            bx0, by0, bx1, by1 = TUNNEL_BUTTON
            off_x, off_y = cam_w + PANEL_W, HEAD_H
            self._tunnel_button = (off_x + bx0, off_y + by0, off_x + bx1, off_y + by1)

        msg = self.bridge.cv2_to_imgmsg(final_debug_image, encoding='bgr8')
        if self.follow_lane_debug is not None:
            msg.header = self.follow_lane_debug.raw_image.header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_debug_image.publish(msg)

    def camera_view(self):
        """Kamerabild mit Spurmarkierungen, im Tunnel oder ohne Bild ein Platzhalter.
        None, wenn das Bild nicht dekodiert werden kann."""
        if self.control_mode == ControlType.Tunnel or self.follow_lane_debug is None:
            text = 'Kamera aus (Tunnel)' if self.control_mode == ControlType.Tunnel \
                else 'kein Kamerabild'
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
