import os

import cv2
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge

from Follow_Lane.src import bev_geometry

CALIBRATION = 'imx219_800x600.yaml'
EXTRINSICS = 'extrinsics_800x600.yaml'


class ingress_node(Node):
    def __init__(self):
        super().__init__('ingress_node')

        self.get_logger().info('Started ingress_node.')

        self.declare_parameter('undistort', True)
        # Nur fuer Fisheye-Kalibrierungen (distortion_model: equidistant): wie viel vom weiten
        # Bildwinkel nach dem Entzerren erhalten bleibt. 0.0 = auf gueltige Pixel zuschneiden
        # (keine schwarzen Raender, aber Rand geht verloren), 1.0 = alles behalten (schwarze Ecken).
        self.declare_parameter('balance', 0.0)
        # Metrische Vogelperspektive auf /lokal/bev (Bereich/Massstab: Parameter/bev.yaml).
        # Direkt aus dem Rohbild in einem Remap -- unabhaengig von 'undistort'.
        self.declare_parameter('publish_bev', True)

        self.bridge = CvBridge()

        self.K = self.D = None
        self.fisheye = False
        self._maps = None
        self._map_size = None
        self.load_calibration()

        self.bev_spec = self.ext = None
        self._bev_maps = None
        self._bev_size = None
        self.load_bev()

        #--- Subscriber ---
        self.sub_raw_image = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.cbPubLokalRawImage, qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cbPubLokalLIDARScan, qos_profile_sensor_data)

        #--- Publisher ---
        self.pub_lokal_raw_image = self.create_publisher(Image, '/lokal/raw_image',1)
        self.pub_bev = self.create_publisher(Image, '/lokal/bev', 1)
        self.pub_lokal_scan = self.create_publisher(LaserScan, '/lokal/scan', 1)

    def load_calibration(self):
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
                f'fx={self.K[0,0]:.1f} cx={self.K[0,2]:.1f}')
        except Exception as e:
            self.get_logger().warn(f'Keine Kalibrierung ({e}) -- Entzerrung bleibt aus.')

    def load_bev(self):
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
        except Exception as ex:
            self.bev_spec = None
            self.get_logger().warn(f'Keine BEV ({ex}) -- /lokal/bev bleibt leer.')

    def bev(self, raw):
        """Rohbild -> metrische Vogelperspektive (eine Remap-Tabelle, einmal pro Bildgroesse)."""
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
        return cv2.remap(raw, self._bev_maps[0], self._bev_maps[1], cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)

    def undistort(self, img):
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
            # Einmal vorberechnen: cv2.undistort wuerde diese Tabellen pro Bild neu aufbauen.
            if self.fisheye:
                newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    K, self.D, (w, h), np.eye(3), balance=balance)
                self._maps = cv2.fisheye.initUndistortRectifyMap(
                    K, self.D, np.eye(3), newK, (w, h), cv2.CV_16SC2)
                self.get_logger().info(f'Fisheye-Entzerrung, balance={balance:.2f}: '
                                       f'fx {K[0,0]:.0f} -> {newK[0,0]:.0f}')
            else:
                self._maps = cv2.initUndistortRectifyMap(K, self.D, None, K, (w, h), cv2.CV_16SC2)
            self._map_size = (w, h, balance)
        return cv2.remap(img, self._maps[0], self._maps[1], cv2.INTER_LINEAR)

    def cbPubLokalRawImage(self, img):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(img, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Bild konnte nicht dekodiert werden: {e}')
            return

        if self.K is not None and self.bev_spec is not None \
                and self.get_parameter('publish_bev').value:
            bev_msg = self.bridge.cv2_to_imgmsg(self.bev(cv_image), encoding='bgr8')
            bev_msg.header.stamp = img.header.stamp      # eigener Header, frame_id nur fuer die BEV
            bev_msg.header.frame_id = 'bev'
            self.pub_bev.publish(bev_msg)

        if self.K is not None and self.get_parameter('undistort').value:
            cv_image = self.undistort(cv_image)

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = img.header
        self.pub_lokal_raw_image.publish(out_msg)

    def cbPubLokalLIDARScan(self, scan):
        self.pub_lokal_scan.publish(scan)


def main(args=None):
    # SIGINT selbst behandeln (KeyboardInterrupt), damit der Context beim Aufraeumen noch
    # gilt -- sonst bei Strg+C ExternalShutdownException-/"context is invalid"-Tracebacks.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ingress_node()
    try:
        # spin_once mit Timeout statt spin(): spin() wartet in C auf die naechste Nachricht,
        # Strg+C kaeme ohne Daten nie an (Knoten haengt beim Beenden).
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:

        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
