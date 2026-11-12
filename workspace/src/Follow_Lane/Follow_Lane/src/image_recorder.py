"""Trainingsbilder fuer YOLO waehrend der Fahrt: alle RECORD_PERIOD_S ein Bild,
abwechselnd das Original-Kamerabild und das intrinsisch entzerrte.

Ein:  /camera/image_raw/compressed (Original), /lokal/raw_image (entzerrt, ingress_node),
      /odom_filtered (nur speichern, solange der Bot faehrt), /control/mode
Aus:  dataset/fahrt_<datum>_<zeit>/fahrt_<datum>_<zeit>_<nr>_{roh,entzerrt}.jpg

Das Original wird als JPEG-Bytes vom Bot unveraendert gespeichert (keine zweite
Kompression), das entzerrte mit JPEG_QUALITY. Im Tunnel nichts (Kamera laut Reglement
verboten). Start: scripts/tb/launchers/follow_lane_record.sh.
"""
import os
from datetime import datetime

import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import UInt8

from Follow_Lane.src.common import ControlType, control_mode_from_msg, run_node

# Abstand (s) zwischen zwei gespeicherten Bildern (roh und entzerrt im Wechsel).
RECORD_PERIOD_S = 1.0
# Nur speichern, wenn der Bot schneller als das faehrt (m/s), sonst entstehen im Stand
# lauter gleiche Bilder.
MIN_SPEED_MPS = 0.02
# Aelter (s) darf ein Bild beim Speichern nicht sein.
MAX_AGE_S = 0.5
JPEG_QUALITY = 95


def repo_root():
    """Repo-Ordner: vom Ort dieser Datei (auch im install/) aufwaerts bis .git."""
    d = os.path.dirname(os.path.abspath(__file__))
    while d != os.path.dirname(d):
        if os.path.isdir(os.path.join(d, '.git')):
            return d
        d = os.path.dirname(d)
    return os.path.expanduser('~')


class image_recorder(Node):

    def __init__(self):
        super().__init__('image_recorder')

        # --- Subscriber ---
        self.sub_orig = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.cb_orig,
            qos_profile_sensor_data)
        self.sub_undist = self.create_subscription(
            Image, '/lokal/raw_image', self.cb_undist, 1)
        self.sub_odom = self.create_subscription(Odometry, '/odom_filtered', self.cb_odom, 1)
        self.sub_mode = self.create_subscription(UInt8, '/control/mode', self.cb_mode, 1)

        # --- Timer ---
        self.timer = self.create_timer(RECORD_PERIOD_S, self.cb_timer)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.orig = None
        self.undist = None
        self.speed = 0.0
        self.control_mode = None
        self.next_is_orig = True
        self.count = 0
        session = 'fahrt_' + datetime.now().strftime('%Y%m%d_%H%M%S')
        self.prefix = session
        self.out_dir = os.path.join(repo_root(), 'dataset', session)
        os.makedirs(self.out_dir, exist_ok=True)

        self.get_logger().info(f'Started image_recorder: alle {RECORD_PERIOD_S} s ein Bild '
                               f'(roh/entzerrt im Wechsel) nach {self.out_dir}')

    # --- Callbacks ---

    def cb_orig(self, msg):
        self.orig = msg

    def cb_undist(self, msg):
        self.undist = msg

    def cb_odom(self, msg):
        self.speed = abs(msg.twist.twist.linear.x)

    def cb_mode(self, msg):
        mode = control_mode_from_msg(self, msg)
        if mode is not None:
            self.control_mode = mode

    def cb_timer(self):
        if self.control_mode in (None, ControlType.Tunnel) or self.speed < MIN_SPEED_MPS:
            return
        msg = self.orig if self.next_is_orig else self.undist
        if msg is None or self.age(msg) > MAX_AGE_S:
            # Bild dieser Sorte fehlt (z.B. ingress_node noch nicht da): dann die andere.
            self.next_is_orig = not self.next_is_orig
            msg = self.orig if self.next_is_orig else self.undist
            if msg is None or self.age(msg) > MAX_AGE_S:
                return
        try:
            self.save(msg, self.next_is_orig)
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'Bild nicht gespeichert: {e}', throttle_duration_sec=5.0)
            return
        self.next_is_orig = not self.next_is_orig

    # --- Hilfsmethoden ---

    def age(self, msg):
        return (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9

    def save(self, msg, is_orig):
        self.count += 1
        kind = 'roh' if is_orig else 'entzerrt'
        path = os.path.join(self.out_dir, f'{self.prefix}_{self.count:04d}_{kind}.jpg')
        if is_orig:
            with open(path, 'wb') as f:
                f.write(bytes(msg.data))
        else:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if self.count % 20 == 0:
            self.get_logger().info(f'{self.count} Bilder gespeichert ({self.out_dir}).')

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info(f'Shutting down image_recorder ({self.count} Bilder in '
                               f'{self.out_dir}).')


def main(args=None):
    run_node(image_recorder, args)


if __name__ == '__main__':
    main()
