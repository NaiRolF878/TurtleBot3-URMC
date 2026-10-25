"""Zonenlinien der Storage-Aufgabe in der Vogelperspektive.

Ein:  /lokal/bev, /control/storage_state, TF odom <- base_footprint
Aus:  /detect/zone_boundaries (PoseArray, odom): Querlinien neben der Spur, also die
      Grenzen der Zonen; nur im Zustand ERKUNDEN.
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
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src import bev_geometry
from Follow_Lane.src.common import (
    StorageState, bev_pixels_to_ground, frange, irange, pose2d_from_transform, read_params,
    run_node, storage_state_from_msg, transform_points)

ERODE_KERNEL = 3


class storage_zone_node(Node):
    """Weisse Linien (LAB nach CLAHE, Startwerte wie detect_lane) werden mit
    HoughLinesP in einem Streifen der BEV gesucht. Es zaehlen nur Linien, die hoechstens
    max_angle_deg schraeg zur Querrichtung liegen; Spurlinien und die Rueckkante der
    Zonen beim Vorbeifahren laufen laengs und fallen weg.

    ERKUNDEN: Streifen neben der Spur auf der Seite der Zonen (side_near_m bis zum
    BEV-Rand). Jede Linie geht als Pose (Mitte, Richtung von der Spur weg) nach odom.
    SCHILD_LESEN: Streifen +-cross_half_m um die Bildmitte; der Bot schaut in die Zone.
    """

    def __init__(self):
        super().__init__('storage_zone_node')

        # --- Parameter ---
        byte = irange(0, 255)
        self.declare_parameter('lab_l_white_l', 200, byte)
        self.declare_parameter('lab_l_white_h', 255, byte)
        self.declare_parameter('lab_a_white_l', 112, byte)
        self.declare_parameter('lab_a_white_h', 140, byte)
        self.declare_parameter('lab_b_white_l', 110, byte)
        self.declare_parameter('lab_b_white_h', 145, byte)
        self.declare_parameter('clahe_clip_limit', 2.0, frange(0.1, 40.0))
        self.declare_parameter('clahe_tile_grid_size', 8, irange(1, 64))
        self.declare_parameter('zones_right', True)
        # Abstand (m) von der Bot-Mitte, ab dem neben der Spur gesucht wird: rechts von
        # der gestrichelten Linie (halbe Spur 0,10 m + Linie + Reserve).
        self.declare_parameter('side_near_m', 0.14, frange(0.0, 0.4))
        self.declare_parameter('cross_half_m', 0.08, frange(0.01, 0.4))
        self.declare_parameter('min_line_m', 0.05, frange(0.01, 0.5))
        self.declare_parameter('max_gap_m', 0.03, frange(0.0, 0.2))
        self.declare_parameter('hough_threshold', 30, irange(1, 500))
        self.declare_parameter('max_angle_deg', 30.0, frange(1.0, 89.0))
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self._param_names = [
            'lab_l_white_l', 'lab_l_white_h', 'lab_a_white_l', 'lab_a_white_h',
            'lab_b_white_l', 'lab_b_white_h', 'clahe_clip_limit', 'clahe_tile_grid_size',
            'zones_right', 'side_near_m', 'cross_half_m', 'min_line_m', 'max_gap_m',
            'hough_threshold', 'max_angle_deg', 'odom_frame', 'robot_frame',
        ]
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_state = self.create_subscription(
            String, '/control/storage_state', self.cb_state, 1)
        self.sub_bev = self.create_subscription(Image, '/lokal/bev', self.cb_bev, 1)

        # --- Publisher ---
        self.pub_boundaries = self.create_publisher(PoseArray, '/detect/zone_boundaries', 5)
        self.pub_cross_lines = self.create_publisher(PoseArray, '/detect/zone_cross_lines', 5)
        self.pub_debug = self.create_publisher(CompressedImage, '/detect/zone_debug/compressed', 1)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state = StorageState.IDLE
        self.bev = bev_geometry.load_bev_spec(
            os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter'))

        self.get_logger().info('Started storage_zone_node.')

    # --- Callbacks ---

    def cb_state(self, msg):
        self.state = storage_state_from_msg(msg)

    def cb_bev(self, img):
        if self.state not in (StorageState.ERKUNDEN, StorageState.SCHILD_LESEN):
            return
        read_params(self, self._param_names)
        try:
            self.find_lines(img)
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'Zonenlinien fehlgeschlagen: {e}')

    # --- Hilfsmethoden ---

    def find_lines(self, img):
        image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        h, w = image.shape[:2]
        res = self.bev['res']
        mask = self.white_mask(image)

        exploring = self.state == StorageState.ERKUNDEN
        # Spalte u <-> Y: Y = y_half - (u + 0.5) * res (Y nach links).
        if exploring:
            near = int(round((self.bev['y_half'] + self.side_near_m) / res))
            u0, u1 = (near, w) if self.zones_right else (0, w - near)
        else:
            half = int(round(self.cross_half_m / res))
            u0, u1 = w // 2 - half, w // 2 + half
        u0, u1 = max(0, u0), min(w, u1)
        band = np.zeros_like(mask)
        band[:, u0:u1] = mask[:, u0:u1]

        segments = cv2.HoughLinesP(
            band, 1, np.pi / 180, int(self.hough_threshold),
            minLineLength=self.min_line_m / res, maxLineGap=self.max_gap_m / res)
        segments = [] if segments is None else segments.reshape(-1, 4)

        lines = []          # (Mitte (X, Y), Richtung (dX, dY)) in base_footprint
        for x1, y1, x2, y2 in segments:
            gx, gy = bev_pixels_to_ground(self.bev, [x1, x2], [y1, y2])
            dx, dy = gx[1] - gx[0], gy[1] - gy[0]
            # Querlinie: Richtung nahe der Y-Achse.
            if math.degrees(math.atan2(abs(dx), abs(dy))) > self.max_angle_deg:
                continue
            # Richtung zur Seite der Zonen, also von der Spur weg (bei SCHILD_LESEN egal).
            if (dy > 0) == self.zones_right:
                dx, dy = -dx, -dy
            lines.append(((float(np.mean(gx)), float(np.mean(gy))), (dx, dy)))

        self.publish_debug(image, mask, (u0, u1), segments, lines)
        if not lines:
            return

        pose = self.robot_pose(img.header.stamp)
        if pose is None:
            return
        out = PoseArray()
        out.header.frame_id = self.odom_frame
        out.header.stamp = img.header.stamp
        mids = transform_points(pose, np.array([m for m, _ in lines]))
        for (x, y), (_, (dx, dy)) in zip(mids, lines):
            yaw = pose[2] + math.atan2(dy, dx)
            p = Pose()
            p.position.x, p.position.y = float(x), float(y)
            p.orientation.z, p.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            out.poses.append(p)
        (self.pub_boundaries if exploring else self.pub_cross_lines).publish(out)

    def white_mask(self, image):
        l_channel, a_channel, b_channel = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2LAB))
        tile = max(1, int(self.clahe_tile_grid_size))
        clahe = cv2.createCLAHE(clipLimit=float(self.clahe_clip_limit), tileGridSize=(tile, tile))
        lab = cv2.merge((clahe.apply(l_channel), a_channel, b_channel))
        mask = cv2.inRange(lab,
            (self.lab_l_white_l, self.lab_a_white_l, self.lab_b_white_l),
            (self.lab_l_white_h, self.lab_a_white_h, self.lab_b_white_h))
        return cv2.erode(mask, np.ones((ERODE_KERNEL, ERODE_KERNEL), np.uint8))

    def robot_pose(self, stamp):
        """Pose zum Bildzeitpunkt; geht das nicht (Uhren Bot/Rechner), die neueste."""
        for t in (Time.from_msg(stamp), Time()):
            try:
                tf = self.tf_buffer.lookup_transform(self.odom_frame, self.robot_frame, t)
                return pose2d_from_transform(tf)
            except ExtrapolationException:
                continue
            except (LookupException, ConnectivityException) as e:
                self.get_logger().warn(f'Keine Roboterpose: {e}', throttle_duration_sec=5.0)
                return None
        self.get_logger().warn('Keine Roboterpose zum Bildzeitpunkt.', throttle_duration_sec=5.0)
        return None

    def publish_debug(self, image, mask, band, segments, lines):
        if self.pub_debug.get_subscription_count() == 0:
            return
        view = image.copy()
        view[mask > 0] = (255, 255, 255)
        cv2.rectangle(view, (band[0], 0), (band[1] - 1, view.shape[0] - 1), (255, 200, 0), 2)
        for x1, y1, x2, y2 in segments:
            cv2.line(view, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(view, f'{self.state.name}: {len(lines)} Querlinien', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode('.jpg', view)
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = array.array('B', encoded.tobytes())
        self.pub_debug.publish(msg)

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down storage_zone_node.')


def main(args=None):
    run_node(storage_zone_node, args)


if __name__ == '__main__':
    main()
