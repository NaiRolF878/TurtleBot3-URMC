#!/usr/bin/env python3
"""Misst die Lage des ChArUco-Kachelrasters am Boden relativ zur LINSE (Median aus N Bildern).

Ausgabe: Kamerahoehe/Neigung/Roll, Abstand der NAECHSTEN Rasterkante vor der Linse,
seitlicher Versatz der Rastermitte, Drehung des Bretts. Mit dem per Zollstock gemessenen
Abstand Radachse -> naechste Rasterkante ergibt sich forward = gemessen - Abstand_ab_Linse.

Aufruf (ingress_node mit undistort:=true muss laufen):
    python3 docs/bev-tests/board-position.py [--zollstock-cm 21.5] [--n 20]
"""
import argparse
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'scripts', 'tb'))
import _camera_geometry as cg  # noqa: E402

PARAM = os.path.join(REPO, 'workspace/src/Follow_Lane/Follow_Lane/Parameter/camera_info')


def measure(img, K, dictionary, board):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, rvec, tvec, n, cc, ci = cg.detect_board_pose(gray, K, np.zeros((1, 5)), dictionary, board)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    pose = cg.pose_from_board(R, tvec)
    tvec = tvec.reshape(3)
    C = -R.T @ tvec
    axis = R.T @ np.array([0.0, 0.0, 1.0])
    fwd = np.array([axis[0], axis[1]]); fwd /= np.linalg.norm(fwd)
    left = np.array([-fwd[1], fwd[0]])
    sx, sy = cg.SQUARES_X * cg.SQUARE_M, cg.SQUARES_Y * cg.SQUARE_M
    corners = np.array([[0, 0], [sx, 0], [0, sy], [sx, sy]], float)
    rel = corners - C[:2]
    f, l = rel @ fwd, rel @ left
    # naechste Kante = Mittel der beiden naechsten Ecken (Brett gerade -> beide gleich weit)
    near = np.sort(f)[:2]
    yaw = pose['board_yaw_deg']
    rms = cg.reprojection_rms(board, cc, ci, rvec, tvec, K, np.zeros((1, 5)))
    return dict(h=pose['height_m'], pitch=pose['pitch_deg'], roll=pose['roll_deg'],
                near=float(near.mean()), near_spread=float(abs(near[1] - near[0])),
                lat=float(l.mean()), cf=float(f.mean()), yaw=float(yaw), n=n, rms=rms)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--zollstock-cm', type=float, default=None,
                    help='gemessener Abstand Radachse -> naechste Rasterkante')
    ap.add_argument('--n', type=int, default=20)
    ap.add_argument('--balance', type=float, default=0.0)
    ap.add_argument('--upscale', type=float, default=2.0,
                    help='Bild vor der Erkennung vergroessern (Default 2)')
    ap.add_argument('--save', metavar='JSON', default=None,
                    help='Median + Streuung als JSON ablegen (fuer yaw-drive-test.py)')
    args = ap.parse_args()

    ipath = os.path.join(PARAM, 'imx219_800x600.yaml')
    K0, D0, cw, ch = cg.load_intrinsics(ipath)
    model = cg.intrinsics_model(ipath)
    K_und = cg.undistorted_camera_matrix(K0, D0, cw, ch, model, args.balance) if cg.is_fisheye(model) else K0
    dictionary, board = cg.charuco_board()

    rclpy.init()
    node = rclpy.create_node('board_position')
    br = CvBridge()
    res = []

    def cb(msg):
        if len(res) >= args.n:
            return
        img = br.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]
        K = K_und.copy(); K[0] *= w / cw; K[1] *= h / ch
        if args.upscale != 1.0:
            # Brett liegt sehr flach im Bild -> Marker zu niedrig fuer die Erkennung.
            # Hochskalieren hilft der Detektion; K mitskalieren, Pose bleibt gleich.
            img = cv2.resize(img, None, fx=args.upscale, fy=args.upscale, interpolation=cv2.INTER_CUBIC)
            K = K.copy(); K[:2] *= args.upscale
        m = measure(img, K, dictionary, board)
        if m:
            res.append(m)
            print(f'  {len(res):2d}: Kante {m["near"]*100:6.2f} cm ab Linse, seitlich {m["lat"]*100:+5.2f} cm, '
                  f'h {m["h"]*100:5.2f} cm, Neigung {m["pitch"]:5.2f}, Roll {m["roll"]:+5.2f}, '
                  f'{m["n"]} Ecken, RMS {m["rms"]:.2f} px')

    node.create_subscription(Image, '/lokal/raw_image', cb, 1)
    end = time.time() + 60
    while rclpy.ok() and len(res) < args.n and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node(); rclpy.shutdown()
    if len(res) < 5:
        print(f'Nur {len(res)} Messungen -- Brett erkannt?')
        return 1

    med = {k: float(np.median([r[k] for r in res])) for k in res[0]}
    std = {k: float(np.std([r[k] for r in res])) for k in res[0]}
    print(f'\n--- Median aus {len(res)} Messungen ---')
    print(f'  Kamerahoehe        {med["h"]*100:6.2f} cm   (+-{std["h"]*100:.2f})')
    print(f'  Neigung            {med["pitch"]:6.2f} Grad (+-{std["pitch"]:.2f})')
    print(f'  Roll               {med["roll"]:+6.2f} Grad (+-{std["roll"]:.2f})')
    print(f'  naechste Kante     {med["near"]*100:6.2f} cm vor der Linse (+-{std["near"]*100:.2f}), '
          f'Kante schief um {med["near_spread"]*100:.2f} cm')
    side = 'links' if med['lat'] > 0 else 'rechts'
    print(f'  Rastermitte        {abs(med["lat"])*100:6.2f} cm {side} der Bildmitte-Achse')
    print(f'  Rastermitte        {med["cf"]*100:6.2f} cm vor der Linse')
    print(f'  Brett gedreht um   {med["yaw"]:+6.2f} Grad gegen die Kamera-Blickrichtung (+-{std["yaw"]:.2f})')
    print(f'  Mittlerer RMS      {med["rms"]:.2f} px')
    if args.save:
        import json
        with open(args.save, 'w') as fh:
            json.dump({'median': med, 'std': std, 'n': len(res), 'time': time.time()}, fh, indent=1)
        print(f'  gespeichert: {args.save}')
    if args.zollstock_cm is not None:
        fwd = args.zollstock_cm / 100 - med['near']
        print(f'\n  Zollstock {args.zollstock_cm:.1f} cm ab Radachse -> Linse liegt '
              f'{fwd*100:.2f} cm VOR der Radachse')
        print(f'  -> --forward {fwd:.4f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
