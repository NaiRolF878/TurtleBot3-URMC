#!/usr/bin/env python3
"""Live-Vogelperspektive des GANZEN Kamerabilds als Topic (ohne Overlays).
Mit --grid stattdessen die Ansicht von camera-extrinsics.py --preview (Kamerabild mit
Gitter + BEV nebeneinander).

Aufruf (Container, ingress_node muss mit undistort:=true laufen):
    python3 scripts/tb/bev-live.py                    # -> /lokal/bev_view
    ros2 run rqt_image_view rqt_image_view /lokal/bev_view

Kameralage, Ausschnitt und entzerrtes K kommen aus extrinsics_800x600.yaml.
Die Werte lassen sich zur Laufzeit verstellen (z.B. gegen einen Zollstock):
    ros2 param set /bev_live forward_offset_m 0.05
Festschreiben danach mit camera-extrinsics.py --recompute --forward <wert>.
"""
import argparse
import importlib.util
import os
import sys

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _camera_geometry as cg  # noqa: E402

# camera-extrinsics.py hat einen Bindestrich im Namen -> per importlib laden
_spec = importlib.util.spec_from_file_location('camera_extrinsics', os.path.join(HERE, 'camera-extrinsics.py'))
ce = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ce)

OUT_TOPIC = '/lokal/bev_view'


class BevLive(Node):
    def __init__(self, args):
        super().__init__('bev_live')
        with open(ce.OUTFILE) as f:
            self.ext = yaml.safe_load(f)
        e = self.ext
        self.K = np.array(e['camera_matrix_undistorted'], float).reshape(3, 3)
        self.declare_parameter('camera_height_m', float(e['camera_height_m']))
        self.declare_parameter('pitch_deg', float(e['camera_pitch_deg']))
        self.declare_parameter('roll_deg', float(e['camera_roll_deg']))
        self.declare_parameter('forward_offset_m', float(e.get('forward_offset_m', 0.0)))
        self.declare_parameter('lateral_offset_m', float(e.get('lateral_offset_m', 0.0)))
        self.declare_parameter('yaw_deg', float(e.get('camera_yaw_deg', 0.0)))
        self.declare_parameter('max_dist_m', float(args.max_dist))
        self.declare_parameter('rate_hz', float(args.rate))

        self.grid = args.grid
        self.br = CvBridge()
        self.latest = None
        self.n = 0
        self.create_subscription(Image, ce.RAW_TOPIC, self.cb, 1)
        self.pub = self.create_publisher(Image, OUT_TOPIC, 1)
        self.timer = self.create_timer(1.0 / max(0.5, args.rate), self.tick)
        self.create_timer(5.0, self.watchdog)
        self.get_logger().info(
            f'Kameralage aus {os.path.basename(ce.OUTFILE)}: Hoehe {e["camera_height_m"] * 100:.1f} cm, '
            f'Neigung {e["camera_pitch_deg"]:.1f} Grad, Roll {e["camera_roll_deg"]:.1f} Grad '
            f'-> {OUT_TOPIC}')

    def cb(self, msg):
        self.latest = msg

    def tick(self):
        if self.latest is None:
            return
        msg, self.latest = self.latest, None
        img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = img.shape[:2]
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        # K gilt fuer die Kalibrieraufloesung -- bei anderer Bildgroesse mitskalieren
        K = self.K.copy()
        K[0] *= w / self.ext['image_width']
        K[1] *= h / self.ext['image_height']
        H_g2i = cg.ground_homography(K, p('camera_height_m'), p('pitch_deg'), p('roll_deg'),
                                     p('forward_offset_m'), p('lateral_offset_m'), p('yaw_deg'))
        e = self.ext
        if self.grid:
            A = ce.output_affine(e['ground_x_min_m'], e['ground_x_max_m'], e['ground_y_half_m'], e['output_size_px'])
            combo, _ = ce.build_preview(img, H_g2i, A, e['ground_x_min_m'], e['ground_x_max_m'],
                                        e['ground_y_half_m'], e['output_size_px'], max_dist=p('max_dist_m'))
        else:
            combo = self.full_bev(img, H_g2i, p('max_dist_m'))
        if combo is None:
            return
        out = self.br.cv2_to_imgmsg(combo, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)
        self.n += 1

    def full_bev(self, img, H_g2i, max_dist):
        """Das GANZE Kamerabild in die Vogelperspektive, ohne Overlays. Ausgabe 600 px hoch,
        gleicher Massstab in beide Richtungen."""
        h, w = img.shape[:2]
        H_i2g = np.linalg.inv(H_g2i)
        us, vs = np.meshgrid(np.linspace(0, w - 1, 41), np.linspace(0, h - 1, 61))
        g = H_i2g @ np.stack([us.ravel(), vs.ravel(), np.ones(us.size)])
        X, Y = g[0] / g[2], g[1] / g[2]
        depth = (H_g2i @ np.stack([X, Y, np.ones_like(X)]))[2]
        ok = (depth > 0) & (X > 0) & (X <= max_dist)
        if ok.sum() < 10:
            return None
        x_min, x_max = float(X[ok].min()), float(min(max_dist, X[ok].max()))
        y_half = float(min(np.abs(Y[ok]).max(), 1.0))
        ppm = 600.0 / (x_max - x_min)
        bw = min(1200, int(round(2 * y_half * ppm)))
        y_half = bw / (2 * ppm)
        A = np.array([[0.0, -1.0 / ppm, x_max],
                      [-1.0 / ppm, 0.0, y_half],
                      [0.0, 0.0, 1.0]])
        M = np.linalg.inv(H_g2i @ A)
        M /= M[2, 2]
        return cv2.warpPerspective(img, M, (bw, 600))

    def watchdog(self):
        if self.n == 0:
            self.get_logger().warn(f'Noch kein Bild ausgegeben -- kommt etwas auf {ce.RAW_TOPIC}? '
                                   '(ingress_node mit undistort:=true starten)')
        else:
            self.get_logger().info(f'{self.n} Bilder -> {OUT_TOPIC}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rate', type=float, default=10.0, help='Ausgaberate in Hz (Default 10)')
    ap.add_argument('--max-dist', type=float, default=1.0, help='BEV bis so weit voraus [m]')
    ap.add_argument('--grid', action='store_true',
                    help='statt reiner BEV: Kamerabild mit Gitter + BEV nebeneinander (wie --preview)')
    args = ap.parse_args()
    rclpy.init()
    node = BevLive(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
