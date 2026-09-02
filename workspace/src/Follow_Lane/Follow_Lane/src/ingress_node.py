import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge

CALIBRATION = 'imx219_800x600.yaml'


class ingress_node(Node):
    def __init__(self):
        super().__init__('ingress_node')

        self.get_logger().info('Started ingress_node.')

        self.declare_parameter('undistort', True)

        self.bridge = CvBridge()

        self.K = self.D = None
        self._maps = None
        self._map_size = None
        self.load_calibration()

        #--- Subscriber ---
        self.sub_raw_image = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.cbPubLokalRawImage, qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cbPubLokalLIDARScan, qos_profile_sensor_data)

        #--- Publisher ---
        self.pub_lokal_raw_image = self.create_publisher(Image, '/lokal/raw_image',1)
        self.pub_lokal_scan = self.create_publisher(LaserScan, '/lokal/scan', 1)

    def load_calibration(self):
        path = os.path.join(get_package_share_directory('Follow_Lane'),
                            'Parameter', 'camera_info', CALIBRATION)
        try:
            with open(path) as f:
                d = yaml.safe_load(f)
            self.K = np.array(d['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
            self.D = np.array(d['distortion_coefficients']['data'], dtype=np.float64).reshape(1, -1)
            self.cal_w, self.cal_h = d['image_width'], d['image_height']
            self.get_logger().info(
                f'Kalibrierung geladen ({self.cal_w}x{self.cal_h}): '
                f'fx={self.K[0,0]:.1f} cx={self.K[0,2]:.1f}')
        except Exception as e:
            self.get_logger().warn(f'Keine Kalibrierung ({e}) -- Entzerrung bleibt aus.')

    def undistort(self, img):
        h, w = img.shape[:2]
        if self._map_size != (w, h):
            # K gilt fuer die Kalibrieraufloesung; bei anderer Bildgroesse mitskalieren.
            K = self.K.copy()
            K[0] *= w / self.cal_w
            K[1] *= h / self.cal_h
            if (w, h) != (self.cal_w, self.cal_h):
                self.get_logger().warn(
                    f'Bild {w}x{h} weicht von der Kalibrierung {self.cal_w}x{self.cal_h} ab '
                    '-- skaliert, aber besser neu kalibrieren.')
            # Einmal vorberechnen: cv2.undistort wuerde diese Tabellen pro Bild neu aufbauen.
            self._maps = cv2.initUndistortRectifyMap(K, self.D, None, K, (w, h), cv2.CV_16SC2)
            self._map_size = (w, h)
        return cv2.remap(img, self._maps[0], self._maps[1], cv2.INTER_LINEAR)

    def cbPubLokalRawImage(self, img):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(img, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Bild konnte nicht dekodiert werden: {e}')
            return

        if self.K is not None and self.get_parameter('undistort').value:
            cv_image = self.undistort(cv_image)

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = img.header
        self.pub_lokal_raw_image.publish(out_msg)

    def cbPubLokalLIDARScan(self, scan):
        self.pub_lokal_scan.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = ingress_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:

        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
