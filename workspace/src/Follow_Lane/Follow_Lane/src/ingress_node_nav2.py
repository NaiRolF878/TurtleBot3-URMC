"""Kamerabild fuer die Pipeline: entzerrt und als metrische Vogelperspektive.

Ein:  /camera/image_raw/compressed, /control/mode
Aus:  /lokal/raw_image (entzerrt), /lokal/bev (BEV, Bereich/Massstab aus Parameter/bev.yaml),
      /lokal/bev_view (mono8, latched: 255 = BEV-Pixel liegt im Kamerabild)
Kopie von ingress_node fuer den Nav2-Zweig; das Original bleibt fuer die
Kalibrierwerkzeuge (extrinsik_view.sh, dataset_capture.sh), die ohne /control/mode laufen.
"""
import os

import cv2
import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import UInt8

from Follow_Lane.src import bev_geometry
from Follow_Lane.src.common import ControlType, control_mode_from_msg, run_node

CALIBRATION = 'imx219_800x600.yaml'
EXTRINSICS = 'extrinsics_800x600.yaml'


class ingress_node(Node):
    """Im Tunnel und vor der ersten /control/mode wird kein Kamerabild verarbeitet:
    in der Enclosure ist die Kamera laut Reglement verboten, und ein Neustart dort
    soll nichts durchlassen. Der Kameratreiber auf dem Bot sendet trotzdem weiter.
    """

    def __init__(self):
        super().__init__('ingress_node')

        # --- Parameter ---
        self.declare_parameter('undistort', True)
        # Nur fuer Fisheye-Kalibrierungen (distortion_model: equidistant): wie viel vom
        # weiten Bildwinkel nach dem Entzerren bleibt. 0.0 = auf gueltige Pixel
        # zuschneiden (keine schwarzen Raender, Rand geht verloren), 1.0 = alles
        # behalten (schwarze Ecken).
        self.declare_parameter('balance', 0.0)
        # Direkt aus dem Rohbild in einem Remap, unabhaengig von 'undistort'.
        self.declare_parameter('publish_bev', True)

        # --- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_raw_image = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.cb_raw_image,
            qos_profile_sensor_data)

        # --- Publisher ---
        self.pub_raw_image = self.create_publisher(Image, '/lokal/raw_image', 1)
        self.pub_bev = self.create_publisher(Image, '/lokal/bev', 1)
        # Die Ecken der BEV liegen teils ausserhalb des Kamerabilds und sind schwarz.
        # lane_costmap_node darf dort nicht "keine Linie gesehen" folgern.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_bev_view = self.create_publisher(Image, '/lokal/bev_view', latched)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.control_mode = None
        self.K = self.D = None
        self.fisheye = False
        self._maps = None
        self._map_size = None
        self.bev_spec = self.ext = None
        self._bev_maps = None
        self._bev_size = None
        self._load_calibration()
        self._load_bev()

        self.get_logger().info('Started ingress_node (Nav2-Zweig).')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None or new_mode == self.control_mode:
            return
        if new_mode == ControlType.Tunnel:
            self.get_logger().info('Tunnel -- Kamerabilder werden nicht mehr verarbeitet.')
        elif self.control_mode == ControlType.Tunnel:
            self.get_logger().info('Tunnel verlassen -- Kamerabilder werden wieder verarbeitet.')
        self.control_mode = new_mode

    def cb_raw_image(self, img):
        if self.control_mode is None or self.control_mode == ControlType.Tunnel:
            return
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(img, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001 -- kaputtes Einzelbild nur melden
            self.get_logger().error(f'Bild konnte nicht dekodiert werden: {e}')
            return

        if self.K is not None and self.bev_spec is not None \
                and self.get_parameter('publish_bev').value:
            bev_msg = self.bridge.cv2_to_imgmsg(self._bev(cv_image), encoding='bgr8')
            bev_msg.header.stamp = img.header.stamp
            bev_msg.header.frame_id = 'bev'
            self.pub_bev.publish(bev_msg)

        if self.K is not None and self.get_parameter('undistort').value:
            cv_image = self._undistort(cv_image)

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = img.header
        self.pub_raw_image.publish(out_msg)

    # --- Hilfsmethoden ---

    def _load_calibration(self):
        path = os.path.join(get_package_share_directory('Follow_Lane'),
                            'Parameter', 'camera_info', CALIBRATION)
        try:
            with open(path) as f:
                d = yaml.safe_load(f)
            self.K = np.array(d['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
            self.D = np.array(d['distortion_coefficients']['data'], dtype=np.float64).reshape(1, -1)
            self.fisheye = d.get('distortion_model', 'plumb_bob') in ('equidistant', 'fisheye')
            if self.fisheye:
                self.D = self.D.reshape(4, 1)
            self.cal_w, self.cal_h = d['image_width'], d['image_height']
            self.get_logger().info(
                f'Kalibrierung geladen ({self.cal_w}x{self.cal_h}, '
                f'{"Fisheye" if self.fisheye else "plumb_bob"}): '
                f'fx={self.K[0, 0]:.1f} cx={self.K[0, 2]:.1f}')
        except Exception as e:  # noqa: BLE001 -- ohne Kalibrierung laeuft der Node unentzerrt
            self.get_logger().warn(f'Keine Kalibrierung ({e}) -- Entzerrung bleibt aus.')

    def _load_bev(self):
        pdir = os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter')
        try:
            self.bev_spec = bev_geometry.load_bev_spec(pdir)
            with open(os.path.join(pdir, 'camera_info', EXTRINSICS)) as f:
                self.ext = yaml.safe_load(f)
            s, e = self.bev_spec, self.ext
            self.get_logger().info(
                f'BEV: {s["x_min"]*100:.0f}-{s["x_max"]*100:.0f} cm voraus, +-{s["y_half"]*100:.0f} cm, '
                f'{s["res"]*1000:.1f} mm/px -> {s["width"]}x{s["height"]} px auf /lokal/bev '
                f'(Hoehe {e["camera_height_m"]*100:.1f} cm, Neigung {e["camera_pitch_deg"]:.1f}, '
                f'Roll {e["camera_roll_deg"]:.1f}, Yaw {e.get("camera_yaw_deg", 0.0):.1f}, '
                f'forward {e.get("forward_offset_m", 0.0)*100:.1f} cm)')
        except Exception as ex:  # noqa: BLE001 -- ohne BEV bleibt nur /lokal/bev leer
            self.bev_spec = None
            self.get_logger().warn(f'Keine BEV ({ex}) -- /lokal/bev bleibt leer.')

    def _bev(self, raw):
        """Rohbild -> metrische Vogelperspektive (eine Remap-Tabelle je Bildgroesse)."""
        h, w = raw.shape[:2]
        balance = float(self.get_parameter('balance').value)
        if self._bev_size != (w, h, balance):
            K_raw = self.K.copy()
            K_raw[0] *= w / self.cal_w
            K_raw[1] *= h / self.cal_h
            if self.fisheye:
                K_und = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    K_raw, self.D, (w, h), np.eye(3), balance=balance)
                ext_bal = float(self.ext.get('balance', 0.0))
                if abs(ext_bal - balance) > 1e-6:
                    self.get_logger().warn(
                        f'Extrinsik wurde mit balance={ext_bal} gemessen, ingress_node laeuft mit '
                        f'{balance} -- BEV stimmt dann nicht.')
            else:
                K_und = K_raw
            self._bev_maps = bev_geometry.build_bev_maps(
                self.bev_spec, self.ext, K_raw, self.D, self.fisheye, K_und)
            self._bev_size = (w, h, balance)
            view = cv2.remap(np.full((h, w), 255, np.uint8), self._bev_maps[0],
                             self._bev_maps[1], cv2.INTER_NEAREST,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            view_msg = self.bridge.cv2_to_imgmsg(view, encoding='mono8')
            view_msg.header.frame_id = 'bev'
            self.pub_bev_view.publish(view_msg)
        return cv2.remap(raw, self._bev_maps[0], self._bev_maps[1], cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)

    def _undistort(self, img):
        h, w = img.shape[:2]
        balance = float(self.get_parameter('balance').value)
        if self._map_size != (w, h, balance):
            # K gilt fuer die Kalibrieraufloesung; bei anderer Bildgroesse mitskalieren.
            K = self.K.copy()
            K[0] *= w / self.cal_w
            K[1] *= h / self.cal_h
            if (w, h) != (self.cal_w, self.cal_h):
                self.get_logger().warn(
                    f'Bild {w}x{h} weicht von der Kalibrierung {self.cal_w}x{self.cal_h} ab '
                    '-- skaliert, aber besser neu kalibrieren.')
            # Einmal vorberechnen: cv2.undistort baut diese Tabellen pro Bild neu auf.
            if self.fisheye:
                newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    K, self.D, (w, h), np.eye(3), balance=balance)
                self._maps = cv2.fisheye.initUndistortRectifyMap(
                    K, self.D, np.eye(3), newK, (w, h), cv2.CV_16SC2)
                self.get_logger().info(f'Fisheye-Entzerrung, balance={balance:.2f}: '
                                       f'fx {K[0, 0]:.0f} -> {newK[0, 0]:.0f}')
            else:
                self._maps = cv2.initUndistortRectifyMap(K, self.D, None, K, (w, h), cv2.CV_16SC2)
            self._map_size = (w, h, balance)
        return cv2.remap(img, self._maps[0], self._maps[1], cv2.INTER_LINEAR)

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down ingress_node (Nav2-Zweig).')


def main(args=None):
    run_node(ingress_node, args)


if __name__ == '__main__':
    main()
