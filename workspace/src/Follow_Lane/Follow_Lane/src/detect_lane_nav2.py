"""Spurerkennung auf der Vogelperspektive: Spurfehler fuer den PID und Linienpixel fuer Nav2.

Ein:  /lokal/bev, /control/mode
Aus:  /detect/lane (Float64), /detect/lane_debug, /detect/mask_debug/compressed,
      /detect/lane_mask (LaneMask fuer lane_costmap_node)
Kopie von detect_lane fuer den Nav2-Zweig; im Tunnel pausiert (Reglement).
"""
import array

import cv2
import numpy as np

from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, Polygon
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64, UInt8

from follow_lane_msg.msg import FollowLaneDebug, LaneMask
from Follow_Lane.src.common import (
    ControlType, control_mode_from_msg, frange, irange, read_params, run_node)

# Kantenlaenge des Erosionskerns (px), entfernt Einzelpixel aus den Farbmasken.
ERODE_KERNEL = 5
# Ersatzspalte als Anteil der Bildbreite, wenn eine Linie nicht gefunden wird.
WHITE_ALTERNATIVE = 0.65
YELLOW_ALTERNATIVE = 0.35
# Kreuzungserkennung: Mindestlaenge einer Kante (px), Konturen kleiner als
# MIN_CONTOUR_AREA (px^2) sind Rauschen, Toleranzen in Grad.
MIN_EDGE_LEN = 30
MIN_CONTOUR_AREA = 50
POLY_EPSILON = 3
PARALLEL_TOL_DEG = 4.0
CORNER_TOL_DEG = 10.0
MIN_BISECTOR_DOT = 0.7


class detect_lane(Node):
    """Vorzeichen: lane_error > 0 heisst Spurmitte links der Bildmitte (gelb links,
    weiss rechts), Wertebereich etwa -1..1.

    Im Tunnel und vor der ersten /control/mode wird kein Bild verarbeitet: in der
    Enclosure ist die Kamera laut Reglement verboten, und ein Neustart dort soll
    nichts durchlassen.
    """

    def __init__(self):
        super().__init__('detect_lane')

        # --- Parameter ---
        byte = irange(0, 255)
        # L = Helligkeit, A = gruen(<128)..rot(>128), B = blau(<128)..gelb(>128); 128 = farblos.
        # Startwerte gemessen auf BEV-Bildern nach CLAHE (docs/bev-tests/README.md):
        #   Gelb       L 170-244  A 115-127  B 152-210   -> erkennbar am hohen B
        #   Weiss      L 210-255  A 120-128  B 118-157   -> erkennbar am hohen L, A/B nahe 128
        #   Matte      L  35-185  A 124-132  B 100-133
        self.declare_parameter('lab_l_white_l', 200, byte)
        self.declare_parameter('lab_l_white_h', 255, byte)
        self.declare_parameter('lab_a_white_l', 112, byte)
        self.declare_parameter('lab_a_white_h', 140, byte)
        self.declare_parameter('lab_b_white_l', 110, byte)
        self.declare_parameter('lab_b_white_h', 145, byte)
        self.declare_parameter('lab_l_yellow_l', 140, byte)
        self.declare_parameter('lab_l_yellow_h', 255, byte)
        self.declare_parameter('lab_a_yellow_l', 100, byte)
        self.declare_parameter('lab_a_yellow_h', 140, byte)
        self.declare_parameter('lab_b_yellow_l', 150, byte)
        self.declare_parameter('lab_b_yellow_h', 255, byte)
        # Suchzeile als Anteil der Bildhoehe (0 = oben, fern).
        self.declare_parameter('detection_row_factor', 0.75, frange(0.5, 0.95))
        # Mindestabstand gelb -> weiss in BEV-Pixeln (1,25 mm/px: 80 px = 10 cm).
        self.declare_parameter('min_lane_width', 80, irange(0, 300))
        # CLAHE gleicht ueber-/unterbelichtete Bildbereiche vor der LAB-Farbfilterung
        # an, damit wechselndes Licht die festen Schwellwerte nicht verfaelscht.
        self.declare_parameter('clahe_clip_limit', 2.0, frange(0.1, 40.0))
        self.declare_parameter('clahe_tile_grid_size', 8, irange(1, 64))
        # Rasterweite fuers Ausduennen der auf /detect/lane_mask gesendeten Punkte.
        self.declare_parameter('lane_mask_stride', 2, irange(1, 20))
        self._param_names = [
            'lab_l_white_l', 'lab_l_white_h', 'lab_a_white_l', 'lab_a_white_h',
            'lab_b_white_l', 'lab_b_white_h',
            'lab_l_yellow_l', 'lab_l_yellow_h', 'lab_a_yellow_l', 'lab_a_yellow_h',
            'lab_b_yellow_l', 'lab_b_yellow_h',
            'detection_row_factor', 'min_lane_width',
            'clahe_clip_limit', 'clahe_tile_grid_size', 'lane_mask_stride',
        ]

        # --- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        # Tiefe 1: die Kamera liefert 30 Bilder/s, die Verarbeitung hier ist langsamer.
        # Mit tieferer Queue stapeln sich Frames und der Regler lenkt nach Bildern,
        # die bis zu ~330 ms alt sind. Das neueste Bild ist immer das beste.
        self.sub_bev = self.create_subscription(Image, '/lokal/bev', self.cb_bev, 1)

        # --- Publisher ---
        self.pub_lane_error = self.create_publisher(Float64, '/detect/lane', 1)
        self.pub_debug_image = self.create_publisher(FollowLaneDebug, '/detect/lane_debug', 1)
        # Muss auf /compressed enden: image_transport deutet das letzte Namenssegment als Transport
        self.pub_mask_debug = self.create_publisher(CompressedImage, '/detect/mask_debug/compressed', 1)
        # Verlustfreie Pixelkoordinaten der Linienpunkte fuer lane_costmap_node, getrennt
        # vom komprimierten Debug-Bild oben, das nur zum Anschauen gedacht ist.
        self.pub_lane_mask = self.create_publisher(LaneMask, '/detect/lane_mask', 1)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.control_mode = None
        self.is_running = False
        self.img_height = None   # Groesse des Bilds, pro Frame gesetzt
        self.img_width = None

        self.get_logger().info('Started detect_lane Node (Nav2-Zweig).')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None or new_mode == self.control_mode:
            return
        if new_mode == ControlType.Tunnel:
            self.get_logger().info('Tunnel -- Spurerkennung pausiert.')
        elif self.control_mode == ControlType.Tunnel:
            self.get_logger().info('Tunnel verlassen -- Spurerkennung laeuft wieder.')
        self.control_mode = new_mode

    def cb_bev(self, img):
        if self.control_mode is None or self.control_mode == ControlType.Tunnel:
            return
        if self.is_running:
            return
        self.is_running = True
        # Alles im try: sonst bliebe das Flag nach einer Exception dauerhaft True und
        # der Node verarbeitete still keine Bilder mehr.
        try:
            self.find_lane(img)
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'Spurerkennung fehlgeschlagen: {e}')
        finally:
            self.is_running = False

    # --- Hilfsmethoden ---

    def find_lane(self, img):
        read_params(self, self._param_names)

        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        self.img_height, self.img_width = cv_image.shape[:2]

        mask_yellow, mask_white = self.color_masks(cv_image)
        kernel = np.ones((ERODE_KERNEL, ERODE_KERNEL), np.uint8)
        mask_yellow = cv2.erode(mask_yellow, kernel, iterations=1)
        mask_white = cv2.erode(mask_white, kernel, iterations=1)

        width = self.img_width
        white_alternative = int(width * WHITE_ALTERNATIVE)
        yellow_alternative = int(width * YELLOW_ALTERNATIVE)

        # Gelb fest am konfigurierten detection_row_factor
        center_yellow_y = int(self.img_height * self.detection_row_factor)
        center_yellow_x = self.get_x_for_driving(
            mask_yellow, center_yellow_y, yellow_alternative, left_line=True)

        # Weisse Maske: alles links von gelb + min_lane_width ausblenden
        mask_white_filtered = mask_white.copy()
        corridor_start = max(0, int(center_yellow_x) + self.min_lane_width)
        mask_white_filtered[:, :corridor_start] = 0

        # Weiss sucht weiter unten, falls nicht gefunden
        detection_row_factor = self.detection_row_factor
        center_white_y = int(self.img_height * detection_row_factor)
        center_white_x = self.get_x_for_driving(
            mask_white_filtered, center_white_y, white_alternative, left_line=False)
        while center_white_x == white_alternative and detection_row_factor <= 0.95:
            detection_row_factor += 0.05
            center_white_y = int(self.img_height * detection_row_factor)
            center_white_x = self.get_x_for_driving(
                mask_white_filtered, center_white_y, white_alternative, left_line=False)

        if center_white_x <= center_yellow_x:
            if center_white_x > int(width * 0.4):
                center_yellow_x = yellow_alternative
            else:
                center_white_x = white_alternative

        lane_center = (center_white_x + center_yellow_x) / 2
        lane_error = 1 - (lane_center / width * 2)

        polies, horizontal, vertical, corners, intersection_corners = \
            self.detect_intersection(mask_white, mask_yellow)

        self.pub_lane_error.publish(Float64(data=lane_error))
        self.pub_debug_follow_lane(center_yellow_x, center_yellow_y, center_white_x, center_white_y,
                                   lane_error, int(lane_center), cv_image, polies, horizontal,
                                   vertical)
        self.publish_mask_debug(mask_yellow, mask_white,
                                center_yellow_x, center_yellow_y, center_white_x, center_white_y)
        # Ungefilterte Masken: die links der gelben Linie ausgeblendeten weissen Pixel
        # braucht nur der PID; fuer die Costmap trennt lane_costmap_node selbst.
        self.publish_lane_mask(mask_yellow, mask_white, img.header.stamp)

    def publish_mask_debug(self, mask_yellow, mask_white,
                           center_yellow_x, center_yellow_y, center_white_x, center_white_y):
        if self.pub_mask_debug.get_subscription_count() == 0:
            return

        overlay = np.zeros((mask_yellow.shape[0], mask_yellow.shape[1], 3), np.uint8)
        both = (mask_yellow > 0) & (mask_white > 0)
        overlay[mask_white > 0] = (255, 255, 255)
        overlay[mask_yellow > 0] = (0, 255, 255)
        overlay[both] = (0, 0, 255)

        cv2.circle(overlay, (center_yellow_x, center_yellow_y), 6, (0, 255, 0), -1, cv2.LINE_AA)
        cv2.circle(overlay, (center_white_x, center_white_y), 6, (255, 0, 255), -1, cv2.LINE_AA)

        success, encoded = cv2.imencode('.jpg', overlay)
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        # array('B') statt bytes: rclpy prueft bytes sonst Byte fuer Byte in Python (~0,7 s/MB)
        msg.data = array.array('B', encoded.tobytes())
        self.pub_mask_debug.publish(msg)

    def publish_lane_mask(self, mask_yellow, mask_white, source_stamp):
        """Linienpixel der BEV-Masken fuer lane_costmap_node (/detect/lane_mask).

        Die Koordinaten sind Pixel in /lokal/bev; lane_costmap_node rechnet sie ueber
        Parameter/bev.yaml in Meter um. source_stamp ist der Stempel des Kamerabilds:
        Nav2s obstacle_layer transformiert die Spurwolke per TF auf genau diesen
        Zeitpunkt, ein spaeterer (now()) verschoebe die Waende gegen die Roboterpose.
        """
        # Nur berechnen, wenn jemand zuschaut.
        if self.pub_lane_mask.get_subscription_count() == 0:
            return

        # Schon hier ausduennen: lane_costmap_node verwirft ohnehin die meisten Punkte
        # wieder, in voller Aufloesung waeren es zehntausende int32 pro Frame.
        stride = max(1, int(self.lane_mask_stride))
        yellow_ys, yellow_xs = np.where(mask_yellow[::stride, ::stride] > 0)
        white_ys, white_xs = np.where(mask_white[::stride, ::stride] > 0)

        msg = LaneMask()
        msg.header.stamp = source_stamp
        msg.image_height, msg.image_width = mask_yellow.shape[:2]
        msg.yellow_x = (yellow_xs * stride).astype(np.int32).tolist()
        msg.yellow_y = (yellow_ys * stride).astype(np.int32).tolist()
        msg.white_x = (white_xs * stride).astype(np.int32).tolist()
        msg.white_y = (white_ys * stride).astype(np.int32).tolist()
        self.pub_lane_mask.publish(msg)

    def pub_debug_follow_lane(self, center_yellow_x, center_yellow_y, center_white_x,
                              center_white_y, lane_error, lane_center, img, polies,
                              horizontal, vertical):
        info = FollowLaneDebug()
        info.center_white_x = center_white_x
        info.center_white_y = center_white_y
        info.center_yellow_x = center_yellow_x
        info.center_yellow_y = center_yellow_y
        info.lane_center = lane_center
        info.lane_error = lane_error
        # 0 = das Bild ist schon die BEV; die RoI-Felder der Nachricht bleiben leer.
        info.crop_im_size = 0

        success, encoded = cv2.imencode('.jpg', img)
        if success:
            info.raw_image.format = 'jpeg'
            # array('B') statt bytes: rclpy prueft bytes sonst Byte fuer Byte in Python (~0,7 s/MB)
            info.raw_image.data = array.array('B', encoded.tobytes())

        for poly in polies or []:
            p = Polygon()
            p.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in poly.reshape(-1, 2)]
            info.polygons.append(p)

        for lines, field in ((horizontal, info.horizontal_lines),
                             (vertical, info.vertical_lines)):
            for p1, p2 in lines or []:
                field.append(Polygon(points=[Point32(x=float(p1[0]), y=float(p1[1]), z=0.0),
                                             Point32(x=float(p2[0]), y=float(p2[1]), z=0.0)]))

        self.pub_debug_image.publish(info)

    def lines_to_poly(self, img, y_offset):
        polies = []
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE,
                                       offset=(0, y_offset))
        for contour in contours:
            if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
                continue
            polies.append(cv2.approxPolyDP(contour, epsilon=POLY_EPSILON, closed=True))
        return polies

    def get_angle_of_line(self, p1, p2):
        d = p2 - p1
        ang = abs(np.degrees(np.arctan2(d[1], d[0])))
        return min(ang, 180 - ang)

    def line_dir(self, p1, p2):
        d = np.asarray(p2, float) - np.asarray(p1, float)
        return np.degrees(np.arctan2(d[1], d[0])) % 180.0

    def angle_diff(self, a, b):
        d = abs(a - b) % 180.0
        return min(d, 180.0 - d)

    def corner_bisector(self, p_prev, p, p_next):
        u1 = (p_prev - p) / np.linalg.norm(p_prev - p)
        u2 = (p_next - p) / np.linalg.norm(p_next - p)
        b = u1 + u2
        n = np.linalg.norm(b)
        return None if n < 1e-6 else b / n

    def _on_border(self, p1, p2, y0):
        return (p1[0] == 0 and p2[0] == 0) \
            or (p1[0] == self.img_width - 1 and p2[0] == self.img_width - 1) \
            or (p1[1] == y0 and p2[1] == y0) \
            or (p1[1] == self.img_height - 1 and p2[1] == self.img_height - 1)

    def detect_intersection(self, img_white, img_yellow):
        """Kanten und Kreuzungsecken der Fahrbahnlinien, fuer das Debugbild."""
        full_img = cv2.bitwise_or(img_white, img_yellow)
        y0 = int(self.img_height * 0.30)
        polies = self.lines_to_poly(full_img[y0:, :], y0)

        vertical = []
        horizontal = []
        corners_per_poly = []      # pro Fahrbahnlinie: [(eckpunkt, halbierende), ...]
        for poly in polies:
            pts = poly.reshape(-1, 2).astype(float)
            len_pts = len(pts)
            if len_pts < 4:
                continue

            # Kanten sammeln (ohne Rand-Kanten und ohne kurze Fransen)
            edge_dirs = []
            for i in range(len_pts):
                p1, p2 = pts[i], pts[(i + 1) % len_pts]
                if self._on_border(p1, p2, y0):
                    continue
                if np.linalg.norm(p2 - p1) > MIN_EDGE_LEN:
                    edge_dirs.append(self.line_dir(p1, p2))
                    angle = self.get_angle_of_line(p1, p2)
                    if angle < 10:
                        horizontal.append((p1, p2))
                    elif angle >= 80:
                        vertical.append((p1, p2))

            has_parallel = any(self.angle_diff(edge_dirs[a], edge_dirs[b]) <= PARALLEL_TOL_DEG
                               for a in range(len(edge_dirs)) for b in range(a + 1, len(edge_dirs)))
            if not has_parallel:
                continue

            corners = []
            for i in range(len_pts):
                p_prev, p, p_next = pts[i - 1], pts[i], pts[(i + 1) % len_pts]
                if min(np.linalg.norm(p_prev - p), np.linalg.norm(p_next - p)) < MIN_EDGE_LEN:
                    continue
                if self._on_border(p_prev, p, y0) or self._on_border(p, p_next, y0):
                    continue
                if abs(self.angle_diff(self.line_dir(p_prev, p),
                                       self.line_dir(p, p_next)) - 90) > CORNER_TOL_DEG:
                    continue
                b = self.corner_bisector(p_prev, p, p_next)
                if b is not None:
                    corners.append((p, b))
            if corners:
                corners_per_poly.append((poly, corners))

        # Kreuzungsecke = Innen- und Aussenkante desselben Streifens knicken parallel ab,
        # d.h. zwei Ecken im selben Polygon mit gleich gerichteter Halbierender.
        # Pro Paar bleibt die konvexe Ecke (Halbierende zeigt ins Polygon), das ist die
        # Ecke an der Fahrbahnseite.
        intersection_corners = []
        for poly, corners in corners_per_poly:
            used = set()
            for i in range(len(corners)):
                for j in range(i + 1, len(corners)):
                    if i in used or j in used:
                        continue
                    (c1, b1), (c2, b2) = corners[i], corners[j]
                    if np.dot(b1, b2) <= MIN_BISECTOR_DOT:
                        continue
                    used.update((i, j))
                    for c, b in ((c1, b1), (c2, b2)):
                        probe = c + 5.0 * b
                        if cv2.pointPolygonTest(poly, (float(probe[0]), float(probe[1])), False) > 0:
                            intersection_corners.append((c, b))
                            break

        corners = [c for _, cs in corners_per_poly for c in cs]
        return polies, horizontal, vertical, corners, intersection_corners

    def color_masks(self, img):
        """Gelb- und Weiss-Maske aus dem BGR-Bild: CLAHE auf L, dann Schwellen in LAB."""
        lab = self.apply_clahe(img)
        mask_yellow = cv2.inRange(lab,
            (self.lab_l_yellow_l, self.lab_a_yellow_l, self.lab_b_yellow_l),
            (self.lab_l_yellow_h, self.lab_a_yellow_h, self.lab_b_yellow_h))
        mask_white = cv2.inRange(lab,
            (self.lab_l_white_l, self.lab_a_white_l, self.lab_b_white_l),
            (self.lab_l_white_h, self.lab_a_white_h, self.lab_b_white_h))
        return mask_yellow, mask_white

    def apply_clahe(self, img):
        """BGR -> LAB mit CLAHE auf dem L-Kanal."""
        l_channel, a_channel, b_channel = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2LAB))
        tile_size = max(1, int(self.clahe_tile_grid_size))
        clahe = cv2.createCLAHE(clipLimit=float(self.clahe_clip_limit),
                                tileGridSize=(tile_size, tile_size))
        return cv2.merge((clahe.apply(l_channel), a_channel, b_channel))

    def get_x_for_driving(self, mask, distance, no_lane_value, left_line):
        """Spalte der Linienkante um Zeile distance (+/-50 Zeilen, Median), sonst
        no_lane_value. left_line: rechte Kante der Linie (zur Spur hin), sonst linke."""
        grad = cv2.Sobel(mask, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0,
                         borderType=cv2.BORDER_DEFAULT)
        # Betrag, sonst enthaelt th1 nur linke Kanten (rechte sind negativ) und [-1]/[0] liefern dasselbe
        abs_grad = cv2.convertScaleAbs(grad)
        _, th1 = cv2.threshold(abs_grad, 127, 255, cv2.THRESH_BINARY)

        a = []
        for row in range(max(0, distance - 50), min(len(mask), distance + 50)):
            hits = np.where(th1[row] == 255)[0]
            if hits.size == 0:
                continue
            a.append(hits[-1] if left_line else hits[0])

        if len(a) > 10:
            return int(np.median(a))
        return no_lane_value

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down detect_lane Node (Nav2-Zweig).')


def main(args=None):
    run_node(detect_lane, args)


if __name__ == '__main__':
    main()
