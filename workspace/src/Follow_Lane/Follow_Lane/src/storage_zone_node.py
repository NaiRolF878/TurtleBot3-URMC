"""Zonenlinien der Storage-Aufgabe in der Vogelperspektive.

Ein:  /lokal/bev, /control/storage_state, /control/storage_zones (Zonenmodell, nur fuer
      die Anzeige), TF odom <- base_footprint
Aus:  /detect/zone_boundaries (PoseArray, odom): Querlinien neben der Spur, also die
      Grenzen der Zonen; nur im Zustand ERKUNDEN.
      /detect/zone_dash_start, /detect/zone_dash_end (PoseArray, odom): Beginn und Ende
      der gestrichelten Spurlinie vor den Zonen; nur im Zustand ERKUNDEN.
      /detect/zone_cross_lines (PoseArray, odom): Querlinien vor dem Bot, also Vorder-
      und Rueckkante der Zone, vor der er steht; nur im Zustand SCHILD_LESEN.
      /detect/zone_debug/compressed (nur mit Zuschauer)
"""
import array
import math
import os

import cv2
import numpy as np

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import StorageZones
from Follow_Lane.src import bev_geometry
from Follow_Lane.src.common import (
    StorageState, bev_pixels_to_ground, frange, ground_to_bev_pixels, inverse_transform_points,
    irange, pose2d_from_transform, read_params, require_params, run_node,
    storage_state_from_msg, transform_points, yaw_from_quaternion)

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'
ERODE_KERNEL = 3
CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_GRID_SIZE = 8
# Reglement: Zonen rechts der Spur.
ZONES_RIGHT = True
# SCHILD_LESEN: Querlinien in +-CROSS_HALF_M um die Bildmitte.
CROSS_HALF_M = 0.08
# HoughLinesP: Mindestlaenge, groesste ueberbrueckte Luecke (m), Stimmen.
MIN_LINE_M = 0.05
MAX_GAP_M = 0.03
HOUGH_THRESHOLD = 30
# Strichel-Streifen: Abstand von der Bot-Mitte (m), Spur 20 cm, Linie 2 cm.
DASH_NEAR_M = 0.06
DASH_FAR_M = 0.16
DASH_MIN_GAPS = 2
# Eine BEV-Zeile gilt als Linie, wenn im Strichel-Streifen so viele Pixel weiss sind.
DASH_MIN_PX = 4
# Kuerzere Luecken (m) sind Rauschen in der Maske, keine Strichluecke.
DASH_NOISE_M = 0.005
# Zonenmodell im Debugbild: Linien so lang (m) ab side_near_m, Ziffer so tief in der Zone.
ZONE_DRAW_M = 0.25
ZONE_LABEL_M = 0.12


class storage_zone_node(Node):
    """Weisse Linien (LAB nach CLAHE) werden mit HoughLinesP in einem Streifen der BEV
    gesucht. Es zaehlen nur Linien, die hoechstens max_angle_deg von der Querrichtung
    abweichen; Spurlinien und die Rueckkante der Zonen laufen laengs und fallen weg.

    ERKUNDEN: Streifen neben der Spur auf der Seite der Zonen (side_near_m bis zum
    BEV-Rand). Jede Linie geht als Pose (Mitte, Richtung von der Spur weg) nach odom.
    Dazu die Spurlinie auf dieser Seite (DASH_NEAR_M bis DASH_FAR_M): Zeile fuer Zeile
    weiss oder nicht, nah nach fern. Durchgehend ueber solid_min_m und danach mindestens
    DASH_MIN_GAPS Luecken bis dash_gap_max_m ist der Beginn der Strichelung, umgekehrt ihr
    Ende.
    SCHILD_LESEN: Streifen +-CROSS_HALF_M um die Bildmitte; der Bot schaut in die Zone.

    Mit Zuschauer laeuft die Erkennung auch in den anderen Zustaenden (wie ERKUNDEN), damit
    sich die Werte einstellen lassen; gesendet wird dann nichts.
    """

    def __init__(self):
        super().__init__('storage_zone_node')

        # --- Parameter ---
        byte = irange(0, 255)
        self.declare_parameter('lab_l_white_l', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_l_white_h', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_a_white_l', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_a_white_h', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_b_white_l', Parameter.Type.INTEGER, byte)
        self.declare_parameter('lab_b_white_h', Parameter.Type.INTEGER, byte)
        self.declare_parameter('side_near_m', Parameter.Type.DOUBLE, frange(0.0, 0.4))
        self.declare_parameter('max_angle_deg', Parameter.Type.DOUBLE, frange(1.0, 89.0))
        self.declare_parameter('solid_min_m', Parameter.Type.DOUBLE, frange(0.01, 0.5))
        self.declare_parameter('dash_gap_max_m', Parameter.Type.DOUBLE, frange(0.005, 0.3))
        self._param_names = [
            'lab_l_white_l', 'lab_l_white_h', 'lab_a_white_l', 'lab_a_white_h',
            'lab_b_white_l', 'lab_b_white_h', 'side_near_m', 'max_angle_deg', 'solid_min_m',
            'dash_gap_max_m',
        ]
        require_params(self, self._param_names)
        read_params(self, self._param_names)

        # --- Subscriber ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_state = self.create_subscription(
            String, '/control/storage_state', self.cb_state, 1)
        self.sub_zones = self.create_subscription(
            StorageZones, '/control/storage_zones', self.cb_zones, latched)
        self.sub_bev = self.create_subscription(Image, '/lokal/bev', self.cb_bev, 1)

        # --- Publisher ---
        self.pub_boundaries = self.create_publisher(PoseArray, '/detect/zone_boundaries', 5)
        self.pub_dash_start = self.create_publisher(PoseArray, '/detect/zone_dash_start', 5)
        self.pub_dash_end = self.create_publisher(PoseArray, '/detect/zone_dash_end', 5)
        self.pub_cross_lines = self.create_publisher(PoseArray, '/detect/zone_cross_lines', 5)
        self.pub_debug = self.create_publisher(CompressedImage, '/detect/zone_debug/compressed', 1)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state = StorageState.IDLE
        self.zones = None
        self.bev = bev_geometry.load_bev_spec(
            os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter'))

        self.get_logger().info('Started storage_zone_node.')

    # --- Callbacks ---

    def cb_state(self, msg):
        self.state = storage_state_from_msg(msg)

    def cb_zones(self, msg):
        self.zones = msg

    def cb_bev(self, img):
        active = self.state in (StorageState.ERKUNDEN, StorageState.SCHILD_LESEN)
        if not active and self.pub_debug.get_subscription_count() == 0:
            return
        read_params(self, self._param_names)
        try:
            self.find_lines(img, active)
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'Zonenlinien fehlgeschlagen: {e}')

    # --- Hilfsmethoden ---

    def find_lines(self, img, active):
        image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        w = image.shape[1]
        res = self.bev['res']
        mask = self.white_mask(image)

        exploring = self.state != StorageState.SCHILD_LESEN
        if exploring:
            u0, u1 = self.side_columns(self.side_near_m, self.bev['y_half'])
        else:
            half = int(round(CROSS_HALF_M / res))
            u0, u1 = max(0, w // 2 - half), min(w, w // 2 + half)
        band = np.zeros_like(mask)
        band[:, u0:u1] = mask[:, u0:u1]

        segments = cv2.HoughLinesP(
            band, 1, np.pi / 180, int(HOUGH_THRESHOLD),
            minLineLength=MIN_LINE_M / res, maxLineGap=MAX_GAP_M / res)
        segments = [] if segments is None else segments.reshape(-1, 4)

        lines = []          # (Mitte (X, Y), Richtung (dX, dY)) in base_footprint
        for x1, y1, x2, y2 in segments:
            gx, gy = bev_pixels_to_ground(self.bev, [x1, x2], [y1, y2])
            dx, dy = gx[1] - gx[0], gy[1] - gy[0]
            # Querlinie: Richtung nahe der Y-Achse.
            if math.degrees(math.atan2(abs(dx), abs(dy))) > self.max_angle_deg:
                continue
            # Richtung zur Seite der Zonen, also von der Spur weg (bei SCHILD_LESEN egal).
            if (dy > 0) == ZONES_RIGHT:
                dx, dy = -dx, -dy
            lines.append(((float(np.mean(gx)), float(np.mean(gy))), (dx, dy)))

        dash_cols = self.side_columns(DASH_NEAR_M, DASH_FAR_M)
        starts, ends = self.find_dashes(mask, dash_cols) if exploring else ([], [])

        need_pose = (active and (lines or starts or ends)) or \
            (self.zones is not None and self.pub_debug.get_subscription_count() > 0)
        pose = self.robot_pose(img.header.stamp) if need_pose else None
        self.publish_debug(image, mask, (u0, u1), dash_cols, segments, lines, starts, ends, pose)
        if not active or pose is None:
            return

        if lines:
            out = self.pose_array(img.header.stamp)
            mids = transform_points(pose, np.array([m for m, _ in lines]))
            for (x, y), (_, (dx, dy)) in zip(mids, lines):
                out.poses.append(self.make_pose(x, y, pose[2] + math.atan2(dy, dx)))
            (self.pub_boundaries if exploring else self.pub_cross_lines).publish(out)
        y_dash = self.side_sign() * (DASH_NEAR_M + DASH_FAR_M) / 2.0
        for xs, pub in ((starts, self.pub_dash_start), (ends, self.pub_dash_end)):
            if xs:
                out = self.pose_array(img.header.stamp)
                for x, y in transform_points(pose, np.array([(x, y_dash) for x in xs])):
                    out.poses.append(self.make_pose(x, y, pose[2]))
                pub.publish(out)

    def find_dashes(self, mask, cols):
        """X (m) aller Beginne und Enden der Strichelung im Bild."""
        h = mask.shape[0]
        res = self.bev['res']
        white = np.count_nonzero(mask[:, cols[0]:cols[1]], axis=1) >= DASH_MIN_PX
        white = white[::-1]                         # Index 0 = naechste Zeile
        x_of = bev_pixels_to_ground(self.bev, 0, np.arange(h)[::-1])[0]

        def runs(values):
            cut = np.flatnonzero(np.diff(values.astype(np.int8))) + 1
            first = np.concatenate(([0], cut))
            last = np.concatenate((cut, [len(values)]))
            return list(zip(values[first], first, last))

        noise = max(1, int(round(DASH_NOISE_M / res)))
        for val, a, b in runs(white):
            if not val and b - a < noise and a > 0 and b < len(white):
                white[a:b] = True

        kinds = []                                  # (Art, X nah, X fern)
        for val, a, b in runs(white):
            length = (b - a) * res
            if val:
                kind = 'solid' if length >= self.solid_min_m else 'dash'
            else:
                kind = 'gap' if length <= self.dash_gap_max_m else 'none'
            kinds.append((kind, float(x_of[a]), float(x_of[b - 1])))

        def gaps_from(i, step):
            n = 0
            i += step
            while 0 <= i < len(kinds) and kinds[i][0] in ('gap', 'dash'):
                n += kinds[i][0] == 'gap'
                i += step
            return n

        starts = [far for i, (k, _, far) in enumerate(kinds)
                  if k == 'solid' and gaps_from(i, 1) >= DASH_MIN_GAPS]
        ends = [near for i, (k, near, _) in enumerate(kinds)
                if k == 'solid' and gaps_from(i, -1) >= DASH_MIN_GAPS]
        return starts, ends

    def side_columns(self, near_m, far_m):
        """BEV-Spalten (u0, u1) fuer den Abstand near_m..far_m auf der Seite der Zonen."""
        y_near, y_far = self.side_sign() * near_m, self.side_sign() * far_m
        u = ground_to_bev_pixels(self.bev, [0.0, 0.0], [y_near, y_far])[0]
        u0, u1 = int(round(min(u))), int(round(max(u))) + 1
        return max(0, u0), min(self.bev['width'], u1)

    def side_sign(self):
        """Y-Vorzeichen der Zonenseite (Y nach links)."""
        return -1.0 if ZONES_RIGHT else 1.0

    def pose_array(self, stamp):
        out = PoseArray()
        out.header.frame_id = ODOM_FRAME
        out.header.stamp = stamp
        return out

    @staticmethod
    def make_pose(x, y, yaw):
        p = Pose()
        p.position.x, p.position.y = float(x), float(y)
        p.orientation.z, p.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return p

    def white_mask(self, image):
        l_channel, a_channel, b_channel = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2LAB))
        clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT,
                                tileGridSize=(CLAHE_TILE_GRID_SIZE, CLAHE_TILE_GRID_SIZE))
        lab = cv2.merge((clahe.apply(l_channel), a_channel, b_channel))
        mask = cv2.inRange(lab,
            (self.lab_l_white_l, self.lab_a_white_l, self.lab_b_white_l),
            (self.lab_l_white_h, self.lab_a_white_h, self.lab_b_white_h))
        return cv2.erode(mask, np.ones((ERODE_KERNEL, ERODE_KERNEL), np.uint8))

    def robot_pose(self, stamp):
        """Pose zum Bildzeitpunkt; geht das nicht (Uhren Bot/Rechner), die neueste."""
        for t in (Time.from_msg(stamp), Time()):
            try:
                tf = self.tf_buffer.lookup_transform(ODOM_FRAME, ROBOT_FRAME, t)
                return pose2d_from_transform(tf)
            except ExtrapolationException:
                continue
            except (LookupException, ConnectivityException) as e:
                self.get_logger().warn(f'Keine Roboterpose: {e}', throttle_duration_sec=5.0)
                return None
        self.get_logger().warn('Keine Roboterpose zum Bildzeitpunkt.', throttle_duration_sec=5.0)
        return None

    def publish_debug(self, image, mask, band, dash_cols, segments, lines, starts, ends, pose):
        if self.pub_debug.get_subscription_count() == 0:
            return
        view = image.copy()
        view[mask > 0] = (255, 255, 255)
        bottom = view.shape[0] - 1
        cv2.rectangle(view, (band[0], 0), (band[1] - 1, bottom), (255, 200, 0), 2)
        cv2.rectangle(view, (dash_cols[0], 0), (dash_cols[1] - 1, bottom), (200, 0, 200), 1)
        for x1, y1, x2, y2 in segments:
            cv2.line(view, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2, cv2.LINE_AA)
        for xs, color in ((starts, (255, 255, 0)), (ends, (255, 0, 255))):
            for x in xs:
                v = int(round(ground_to_bev_pixels(self.bev, x, 0.0)[1]))
                cv2.line(view, (dash_cols[0], v), (dash_cols[1], v), color, 3)
        if pose is not None:
            self.draw_zones(view, pose)
        cv2.putText(view, f'{self.state.name}: {len(lines)} Querlinien, '
                    f'Strichel {len(starts)} Beginn / {len(ends)} Ende', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode('.jpg', view)
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = array.array('B', encoded.tobytes())
        self.pub_debug.publish(msg)

    def draw_zones(self, view, pose):
        """Zonenmodell von switch_control: gemessene Linien gruen, ergaenzte orange, Ziffern."""
        z = self.zones
        if z is None or not z.lines:
            return
        pts = inverse_transform_points(
            pose, np.array([(p.position.x, p.position.y) for p in z.lines]))
        yaws = [yaw_from_quaternion(p.orientation) - pose[2] for p in z.lines]

        def px(x, y):
            u, v = ground_to_bev_pixels(self.bev, x, y)
            return int(round(float(u))), int(round(float(v)))

        for (x, y), yaw, measured in zip(pts, yaws, z.measured):
            c, s = math.cos(yaw), math.sin(yaw)
            a = px(x + self.side_near_m * c, y + self.side_near_m * s)
            b = px(x + (self.side_near_m + ZONE_DRAW_M) * c, y + (self.side_near_m + ZONE_DRAW_M) * s)
            cv2.line(view, a, b, (0, 220, 0) if measured else (0, 165, 255),
                     3 if measured else 2, cv2.LINE_AA)
        for k, digit in enumerate(z.digits):
            if k + 1 >= len(pts):
                break
            x, y = (pts[k] + pts[k + 1]) / 2.0
            yaw = yaws[k]
            d = self.side_near_m + ZONE_LABEL_M
            cv2.putText(view, str(digit) if digit else '?',
                        px(x + d * math.cos(yaw), y + d * math.sin(yaw)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 220, 0), 3, cv2.LINE_AA)

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down storage_zone_node.')


def main(args=None):
    run_node(storage_zone_node, args)


if __name__ == '__main__':
    main()
