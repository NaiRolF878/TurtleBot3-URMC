#!/usr/bin/env python3
"""Bestimmt Hoehe, Neigung und Rollwinkel der Kamera gegenueber der Fahrbahn.

Aufruf (Container, ingress_node muss laufen):
    python3 scripts/tb/camera-extrinsics.py

Vorgehen: Das ChArUco-Brett FLACH auf den Boden vor den Bot legen, moeglichst
formatfuellend im Bild. Das Skript sammelt mehrere Messungen und mittelt sie.

Warum optisch statt mit dem Zollstock: Der Neigungswinkel laesst sich von Hand kaum
auf besser als 2-3 Grad messen, und genau der bestimmt, wohin ferne Bildzeilen auf dem
Boden abgebildet werden. solvePnP liefert ihn aus der bekannten Brettgeometrie.

Ergebnis geht nach Parameter/camera_info/extrinsics_800x600.yaml und ist die Eingangs-
groesse fuer die Bodenebenen-Homographie (Schritt 2).
"""
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PARAM_DIR = os.path.join(REPO, 'workspace/src/Follow_Lane/Follow_Lane/Parameter/camera_info')
INTRINSICS = os.path.join(PARAM_DIR, 'imx219_800x600.yaml')
OUTFILE = os.path.join(PARAM_DIR, 'extrinsics_800x600.yaml')

# Muss zum Ausdruck passen (steht dort aufgedruckt).
SQUARES_X, SQUARES_Y = 7, 5
SQUARE_M, MARKER_M = 0.030, 0.022
ARUCO_DICT = cv2.aruco.DICT_5X5_250

N_SAMPLES = 20


def ground_homography(K, height, pitch_deg, roll_deg, dx, dy):
    """Homographie Bodenebene (X vorwaerts, Y links, Z=0) -> Bildpixel.

    Kamerasystem nach OpenCV-Konvention: x rechts, y unten, z optische Achse.
    Robotersystem nach ROS: x vorwaerts, y links, z oben.
    """
    th, ph = math.radians(pitch_deg), math.radians(roll_deg)

    # Grunddrehung ohne Neigung: robot-x -> cam-z, robot-y -> -cam-x, robot-z -> -cam-y
    R0 = np.array([[0., -1., 0.],
                   [0., 0., -1.],
                   [1., 0., 0.]])
    # Nicken nach unten um die Kamera-x-Achse. Vorzeichen so, dass die Fahrtrichtung
    # ueber die Bildmitte wandert (-y = oben in OpenCV) -- andersherum schaut sie hoch.
    Rx = np.array([[1., 0., 0.],
                   [0., math.cos(th), -math.sin(th)],
                   [0., math.sin(th), math.cos(th)]])
    # Rollen um die optische Achse.
    Rz = np.array([[math.cos(ph), -math.sin(ph), 0.],
                   [math.sin(ph), math.cos(ph), 0.],
                   [0., 0., 1.]])

    R_rc = Rz @ Rx @ R0
    t = -R_rc @ np.array([dx, dy, height])
    # Z=0 einsetzen laesst die dritte Spalte von R_rc entfallen.
    return K @ np.column_stack([R_rc[:, 0], R_rc[:, 1], t])


def output_affine(x_min, x_max, y_half, size):
    """Zielbildpixel (u,v) -> Bodenkoordinate (X,Y).

    Oben im Zielbild = weit weg, unten = dicht vor dem Bot, links = links.
    """
    return np.array([
        [0.0,              -(x_max - x_min) / size, x_max],
        [-(2 * y_half) / size, 0.0,                 y_half],
        [0.0,               0.0,                    1.0],
    ])


def load_intrinsics(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], dtype=np.float64).reshape(1, -1)
    return K, D, d['image_width'], d['image_height']


class Extrinsics(Node):
    def __init__(self, K, D):
        super().__init__('camera_extrinsics')
        self.K, self.D = K, D
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT)
        self.board = cv2.aruco.CharucoBoard_create(
            SQUARES_X, SQUARES_Y, SQUARE_M, MARKER_M, self.dictionary)
        self.results = []
        self.create_subscription(Image, '/lokal/raw_image', self.cb, 1)

    def cb(self, msg):
        if len(self.results) >= N_SAMPLES:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        if ids is None or len(ids) < 6:
            return

        n, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board)
        # Unter ~8 Ecken wird die Lageschaetzung unzuverlaessig.
        if n is None or n < 8:
            return

        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            ch_corners, ch_ids, self.board, self.K, self.D, None, None)
        if not ok:
            return

        R, _ = cv2.Rodrigues(rvec)
        # Kameraposition im Brettsystem. Das Brett liegt flach, also ist z die Hoehe.
        C = (-R.T @ tvec).ravel()
        # Optische Achse (Kamera-z) im Brettsystem ausgedrueckt.
        axis = (R.T @ np.array([[0.0], [0.0], [1.0]])).ravel()

        height = abs(C[2])
        # Negatives z der Achse = Blick nach unten. Winkel gegen die Waagerechte.
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -axis[2] * np.sign(C[2])))))
        # Kamera-x im Brettsystem: dessen z-Anteil kippt den Horizont.
        right = (R.T @ np.array([[1.0], [0.0], [0.0]])).ravel()
        roll = math.degrees(math.asin(max(-1.0, min(1.0, right[2] * np.sign(C[2])))))

        self.results.append((height, pitch, roll, n))
        print(f'  Messung {len(self.results):2d}/{N_SAMPLES}: '
              f'Hoehe {height*100:6.1f} cm   Neigung {pitch:5.1f}°   Roll {roll:5.1f}°   '
              f'({n} Ecken)')


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--forward', type=float, default=0.0,
                    help='Linse nach vorn von der Radachse, in Metern')
    ap.add_argument('--lateral', type=float, default=0.0,
                    help='seitlicher Versatz der Linse, positiv = links, in Metern')
    ap.add_argument('--x-min', type=float, default=0.15, help='naeheste Bodenzeile [m]')
    ap.add_argument('--x-max', type=float, default=0.75, help='fernste Bodenzeile [m]')
    ap.add_argument('--y-half', type=float, default=0.30, help='halbe Breite des Ausschnitts [m]')
    ap.add_argument('--size', type=int, default=400, help='Kantenlaenge des Zielbilds [px]')
    ap.add_argument('--recompute', action='store_true',
                    help='nicht neu messen, nur Homographie aus der gespeicherten Lage rechnen')
    args = ap.parse_args()

    if not os.path.isfile(INTRINSICS):
        print(f'Intrinsische Kalibrierung fehlt: {INTRINSICS}')
        return 1

    K, D, w, h = load_intrinsics(INTRINSICS)
    print(f'Intrinsik geladen ({w}x{h}): fx={K[0,0]:.1f} fy={K[1,1]:.1f} '
          f'cx={K[0,2]:.1f} cy={K[1,2]:.1f}\n')

    if args.recompute:
        if not os.path.isfile(OUTFILE):
            print(f'Keine gespeicherte Lage vorhanden: {OUTFILE}')
            return 1
        with open(OUTFILE) as f:
            prev = yaml.safe_load(f)
        med = np.array([prev['camera_height_m'], prev['camera_pitch_deg'], prev['camera_roll_deg']])
        std = np.array([prev.get('std_height_m', 0.0), prev.get('std_pitch_deg', 0.0), 0.0])
        n_used = prev.get('samples', 0)
        print('Nur Neuberechnung -- gespeicherte Kameralage wird uebernommen.\n')
        node = None
    else:
        print('Brett FLACH auf den Boden vor den Bot legen, formatfuellend im Bild.')
        print(f'Sammle {N_SAMPLES} Messungen ...\n')

        rclpy.init()
        node = Extrinsics(K, D)
        end = time.time() + 120
        while rclpy.ok() and len(node.results) < N_SAMPLES and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

        if len(node.results) < 5:
            print(f'\nNur {len(node.results)} Messungen -- Brett sichtbar? Beleuchtung?')
            node.destroy_node(); rclpy.shutdown(); return 1

        arr = np.array([(r[0], r[1], r[2]) for r in node.results])
        med = np.median(arr, axis=0)
        std = np.std(arr, axis=0)
        n_used = len(node.results)

    print(f'--- Kameralage ({n_used} Messungen, Median) ---')
    print(f'  Hoehe ueber Boden : {med[0]*100:6.2f} cm   (Streuung {std[0]*100:.2f} cm)')
    print(f'  Neigung nach unten: {med[1]:6.2f}°    (Streuung {std[1]:.2f}°)')
    print(f'  Rollwinkel        : {med[2]:6.2f}°    (Streuung {std[2]:.2f}°)')
    if std[1] > 1.0:
        print('\n  WARNUNG: Streuung der Neigung ueber 1° -- Brett lag evtl. nicht plan')
        print('  oder war zu klein im Bild. Fuer die Homographie besser wiederholen.')

    # --- Bodenebenen-Homographie ---
    H_g2i = ground_homography(K, med[0], med[1], med[2], args.forward, args.lateral)
    A = output_affine(args.x_min, args.x_max, args.y_half, args.size)
    H_out2src = H_g2i @ A
    M = np.linalg.inv(H_out2src)          # src -> dst, wie crop_img es erwartet
    M /= M[2, 2]

    print(f'\n--- Bodenausschnitt {args.x_min:.2f}..{args.x_max:.2f} m voraus, '
          f'+/-{args.y_half:.2f} m seitlich -> {args.size}x{args.size} px ---')
    inside = 0
    for name, (X, Y) in [('nah  links', (args.x_min, args.y_half)),
                         ('nah  rechts', (args.x_min, -args.y_half)),
                         ('fern links', (args.x_max, args.y_half)),
                         ('fern rechts', (args.x_max, -args.y_half))]:
        p = H_g2i @ np.array([X, Y, 1.0])
        u, v = p[0] / p[2], p[1] / p[2]
        ok = 0 <= u < w and 0 <= v < h
        inside += ok
        print(f'  Ecke {name:12s} -> Bildpixel ({u:7.1f}, {v:7.1f}) '
              f'{"im Bild" if ok else "AUSSERHALB"}')
    if inside < 4:
        print('\n  Nicht alle Ecken liegen im Bild. Der Ausschnitt ist zu gross oder')
        print('  zu weit vorn -- mit --x-max kleiner bzw. --y-half kleiner neu rechnen.')
        print('  (Die Messung selbst bleibt gueltig, nur der Ausschnitt passt nicht.)')

    with open(OUTFILE, 'w') as f:
        yaml.dump({
            'image_width': int(w), 'image_height': int(h),
            'camera_height_m': float(med[0]),
            'camera_pitch_deg': float(med[1]),
            'camera_roll_deg': float(med[2]),
            'forward_offset_m': float(args.forward),
            'lateral_offset_m': float(args.lateral),
            'ground_x_min_m': float(args.x_min),
            'ground_x_max_m': float(args.x_max),
            'ground_y_half_m': float(args.y_half),
            'output_size_px': int(args.size),
            # Direkt fuer cv2.warpPerspective auf dem ENTZERRTEN Bild verwendbar.
            'warp_matrix': [float(x) for x in M.ravel()],
            'samples': int(n_used),
            'std_height_m': float(std[0]),
            'std_pitch_deg': float(std[1]),
        }, f, default_flow_style=False, sort_keys=True)
    print(f'\nGeschrieben: {OUTFILE}')
    print('Hinweis: warp_matrix gilt fuer das ENTZERRTE Bild -- vorher cv2.undistort(K, D).')

    if node is not None:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
