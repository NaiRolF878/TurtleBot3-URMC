#!/usr/bin/env python3
"""Faehrt GERADE RUECKWAERTS (vom Brett weg) und stoppt ueber die KAMERA: sobald die Brettmitte
um --dist weiter weg ist. Unabhaengig vom Radschlupf (Odometrie auf der Matte ~50 % daneben).

Stoppt ausserdem bei: Brett > 1 s nicht erkannt, Zeitlimit, Odometrie > --odom-max, Strg+C.
Aufruf: python3 docs/bev-tests/drive-until-board.py [--dist 0.10] [--speed 0.03]
"""
import argparse
import importlib.util
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('board_position', os.path.join(HERE, 'board-position.py'))
bp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bp)
cg = bp.cg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=float, default=0.10, help='Soll-Strecke laut Kamera [m]')
    ap.add_argument('--speed', type=float, default=0.03, help='[m/s], max 0.05')
    ap.add_argument('--odom-max', type=float, default=0.35, help='Sicherheitsgrenze laut Odometrie [m]')
    ap.add_argument('--timeout', type=float, default=15.0)
    args = ap.parse_args()
    speed = -min(abs(args.speed), 0.05)

    ipath = os.path.join(bp.PARAM, 'imx219_800x600.yaml')
    K0, D0, cw, ch = cg.load_intrinsics(ipath)
    model = cg.intrinsics_model(ipath)
    K_und = cg.undistorted_camera_matrix(K0, D0, cw, ch, model, 0.0)
    dictionary, board = cg.charuco_board()

    rclpy.init()
    node = rclpy.create_node('drive_until_board')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    br = CvBridge()
    st = {'img': None, 'odom': None}
    node.create_subscription(Image, '/lokal/raw_image', lambda m: st.__setitem__('img', m), 1)
    node.create_subscription(Odometry, '/odom', lambda m: st.__setitem__('odom', m), 10)

    def board_cf():
        msg, st['img'] = st['img'], None
        if msg is None:
            return None
        img = br.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]
        K = K_und.copy(); K[0] *= w / cw; K[1] *= h / ch
        if w < 1200:                       # bei 800x600 hochskalieren (flaches Brett)
            img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
            K[:2] *= 2
        m = bp.measure(img, K, dictionary, board)
        return None if m is None else m['cf']

    def stop():
        for _ in range(10):
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)

    # Startlage: Median aus 5 Erkennungen
    starts, t_end = [], time.time() + 10
    while len(starts) < 5 and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        cf = board_cf()
        if cf is not None:
            starts.append(cf)
    if len(starts) < 5 or st['odom'] is None:
        print('Brett oder /odom fehlt -- nichts gefahren.')
        node.destroy_node(); rclpy.shutdown(); return 1
    cf0 = float(np.median(starts))
    o = st['odom'].pose.pose.position
    x0, y0 = o.x, o.y
    print(f'Start: Brettmitte {cf0*100:.2f} cm vor der Linse. Fahre rueckwaerts bis +{args.dist*100:.0f} cm ...')

    cmd = Twist(); cmd.linear.x = speed
    t0, last_seen, reason, cf = time.time(), time.time(), 'Zeitlimit', cf0
    try:
        while time.time() - t0 < args.timeout:
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.01)
            o = st['odom'].pose.pose.position
            if math.hypot(o.x - x0, o.y - y0) > args.odom_max:
                reason = 'Odometrie-Grenze'; break
            c = board_cf()
            if c is not None:
                cf, last_seen = c, time.time()
                if cf - cf0 >= args.dist:
                    reason = 'Strecke erreicht'; break
            elif time.time() - last_seen > 1.0:
                reason = 'Brett nicht mehr erkannt'; break
    finally:
        stop()
    o = st['odom'].pose.pose.position
    print(f'Stopp ({reason}): laut Kamera {(cf - cf0)*100:.1f} cm, '
          f'laut Odometrie {math.hypot(o.x - x0, o.y - y0)*100:.1f} cm')
    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
