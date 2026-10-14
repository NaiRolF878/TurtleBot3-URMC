#!/usr/bin/env python3
"""Bestimmt Hoehe, Neigung und Rollwinkel der Kamera gegenueber der Fahrbahn.

Aufruf (Container, ingress_node muss laufen):
    python3 scripts/tb/camera-extrinsics.py                 # messen + Homographie
    python3 scripts/tb/camera-extrinsics.py --recompute     # nur Ausschnitt neu rechnen
    python3 scripts/tb/camera-extrinsics.py --recompute --preview   # Sichtpruefung
    python3 scripts/tb/camera-extrinsics.py --force-roll 0           # symmetrisch erzwingen

Vorgehen: Das ChArUco-Brett FLACH auf den Boden vor den Bot legen, moeglichst
formatfuellend im Bild. Das Skript sammelt mehrere Messungen und mittelt sie.

Warum optisch statt mit dem Zollstock: Der Neigungswinkel laesst sich von Hand kaum
auf besser als 2-3 Grad messen, und genau der bestimmt, wohin ferne Bildzeilen auf dem
Boden abgebildet werden. solvePnP liefert ihn aus der bekannten Brettgeometrie.

Ergebnis geht nach Parameter/camera_info/extrinsics_800x600.yaml und ist die Eingangs-
groesse fuer die Bodenebenen-Homographie (Schritt 2).

WICHTIG: Diese Homographie ist nur so gut wie die INTRINSIK
(Parameter/camera_info/imx219_800x600.yaml). Ein daneben liegender Hauptpunkt (cx/cy)
oder grosse tangentiale Verzeichnung kippen die Vogelperspektive seitlich -- dann hilft
keine noch so saubere Extrinsik. Das Skript prueft die Intrinsik beim Start und warnt.
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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _camera_geometry as cg  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PARAM_DIR = os.path.join(REPO, 'workspace/src/Follow_Lane/Follow_Lane/Parameter/camera_info')
INTRINSICS = os.path.join(PARAM_DIR, 'imx219_800x600.yaml')
OUTFILE = os.path.join(PARAM_DIR, 'extrinsics_800x600.yaml')
PREVIEW_OUT = os.path.join(REPO, 'logs', 'vogelperspektive.png')

RAW_TOPIC = '/lokal/raw_image'

N_SAMPLES = 20


def ground_homography(K, height, pitch_deg, roll_deg, dx, dy):
    """Duenner Wrapper -- die Rechnung steht in _camera_geometry (eine Quelle)."""
    return cg.ground_homography(K, height, pitch_deg, roll_deg, dx, dy)


def output_affine(x_min, x_max, y_half, size):
    """Zielbildpixel (u,v) -> Bodenkoordinate (X,Y).

    Oben im Zielbild = weit weg, unten = dicht vor dem Bot, links = links.
    """
    return np.array([
        [0.0,                  -(x_max - x_min) / size, x_max],
        [-(2 * y_half) / size,  0.0,                    y_half],
        [0.0,                   0.0,                    1.0],
    ])


def grab_frame(timeout=15.0):
    """Ein einzelnes Bild von RAW_TOPIC holen (bereits entzerrt, wenn ingress_node
    mit undistort:=true laeuft -- so gilt es auch die warp_matrix)."""
    from cv_bridge import CvBridge
    br = CvBridge()
    holder = {}
    own_context = not rclpy.ok()
    if own_context:
        rclpy.init()
    node = rclpy.create_node('extrinsics_preview_grab')

    def cb(msg):
        holder['img'] = br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    node.create_subscription(Image, RAW_TOPIC, cb, 1)
    end = time.time() + timeout
    while rclpy.ok() and 'img' not in holder and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if own_context and rclpy.ok():
        rclpy.shutdown()
    return holder.get('img')


def render_preview(src_bgr, H_g2i, A, x_min, x_max, y_half, size, outpath):
    """Links: entzerrtes Kamerabild mit eingezeichnetem Bodenausschnitt.
    Rechts: Vogelperspektive mit metrischem Gitter (0.1 m), Y=0 gruen.
    """
    h, w = src_bgr.shape[:2]
    M = np.linalg.inv(H_g2i @ A)
    M /= M[2, 2]
    bev = cv2.warpPerspective(src_bgr, M, (size, size))

    Ainv = np.linalg.inv(A)

    def g2o(X, Y):
        p = Ainv @ np.array([X, Y, 1.0])
        return int(round(p[0] / p[2])), int(round(p[1] / p[2]))

    def g2i(X, Y):
        p = H_g2i @ np.array([X, Y, 1.0])
        return p[0] / p[2], p[1] / p[2]

    # --- Gitter in die Vogelperspektive ---
    x0 = math.ceil(x_min * 10) / 10
    xv = x0
    while xv <= x_max + 1e-9:
        cv2.line(bev, g2o(xv, y_half), g2o(xv, -y_half), (0, 200, 200), 1, cv2.LINE_AA)
        u, v = g2o(xv, y_half)
        cv2.putText(bev, f'{xv:.1f}', (3, max(12, v + 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 220), 1, cv2.LINE_AA)
        xv = round(xv + 0.1, 2)

    yv = -round(math.floor(y_half * 10) / 10, 2)
    while yv <= y_half + 1e-9:
        green = abs(yv) < 1e-6
        col = (0, 255, 0) if green else (200, 0, 200)
        cv2.line(bev, g2o(x_min, yv), g2o(x_max, yv), col, 2 if green else 1, cv2.LINE_AA)
        u, v = g2o((x_min + x_max) / 2, yv)
        cv2.putText(bev, f'{yv:+.1f}', (max(2, u - 12), size - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
        yv = round(yv + 0.1, 2)

    # --- Bodenausschnitt in das Kamerabild ---
    src = src_bgr.copy()
    quad = np.array([g2i(x_max, y_half), g2i(x_max, -y_half),
                     g2i(x_min, -y_half), g2i(x_min, y_half)], np.int32)
    cv2.polylines(src, [quad], True, (0, 255, 255), 2, cv2.LINE_AA)
    cx_i, cy_i = g2i(x_min, 0.0)
    cx_f, cy_f = g2i(x_max, 0.0)
    cv2.line(src, (int(cx_i), int(cy_i)), (int(cx_f), int(cy_f)), (0, 255, 0), 2, cv2.LINE_AA)

    # --- nebeneinander, gleiche Hoehe ---
    scale = h / size
    bev_r = cv2.resize(bev, (int(size * scale), h))
    combo = np.hstack([src, bev_r])
    cv2.putText(combo, 'Kamera entzerrt + Bodenausschnitt', (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(combo, 'Vogelperspektive, Gitter 0.1 m, Y=0 gruen', (w + 10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(outpath, combo)
    print(f'Vorschau geschrieben: {outpath}')


class Extrinsics(Node):
    def __init__(self, K, D):
        super().__init__('camera_extrinsics')
        self.K, self.D = K, D
        self.bridge = CvBridge()
        self.dictionary, self.board = cg.charuco_board()
        self.results = []
        self.create_subscription(Image, RAW_TOPIC, self.cb, 1)

    def cb(self, msg):
        if len(self.results) >= N_SAMPLES:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ok, rvec, tvec, n, cc, ci = cg.detect_board_pose(
            gray, self.K, self.D, self.dictionary, self.board)
        if not ok:
            return

        R, _ = cv2.Rodrigues(rvec)
        pose = cg.pose_from_board(R, tvec)
        rms = cg.reprojection_rms(self.board, cc, ci, rvec, tvec, self.K, self.D)

        self.results.append((pose['height_m'], pose['pitch_deg'], pose['roll_deg'], n, rms))
        print(f'  Messung {len(self.results):2d}/{N_SAMPLES}: '
              f'Hoehe {pose["height_m"]*100:6.1f} cm   Neigung {pose["pitch_deg"]:5.1f} deg   '
              f'Roll {pose["roll_deg"]:5.1f} deg   ({n} Ecken, RMS {rms:.2f} px)')


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--forward', type=float, default=0.0,
                    help='Linse nach vorn von der Radachse, in Metern')
    ap.add_argument('--lateral', type=float, default=0.0,
                    help='seitlicher Versatz der Linse, positiv = links, in Metern')
    ap.add_argument('--x-min', type=float, default=0.20, help='naeheste Bodenzeile [m]')
    ap.add_argument('--x-max', type=float, default=0.60, help='fernste Bodenzeile [m]')
    ap.add_argument('--y-half', type=float, default=0.20, help='halbe Breite des Ausschnitts [m]')
    ap.add_argument('--size', type=int, default=400, help='Kantenlaenge des Zielbilds [px]')
    ap.add_argument('--force-roll', type=float, default=None, metavar='DEG',
                    help='Rollwinkel [Grad] fest vorgeben statt messen. --force-roll 0 '
                         'erzwingt eine links/rechts-symmetrische Vogelperspektive -- nur '
                         'sinnvoll, wenn die Kamera nachweislich waagerecht montiert ist.')
    ap.add_argument('--distorted', action='store_true',
                    help=f'{RAW_TOPIC} ist NICHT entzerrt (ingress_node mit undistort:=false). '
                         'Default: Bild ist entzerrt, PnP rechnet daher mit D = 0.')
    ap.add_argument('--recompute', action='store_true',
                    help='nicht neu messen, nur Homographie aus der gespeicherten Lage rechnen')
    ap.add_argument('--preview', action='store_true',
                    help='ein Kamerabild von %s holen und die Vogelperspektive mit '
                         'metrischem Gitter nach logs/vogelperspektive.png schreiben. '
                         'ingress_node muss mit undistort:=true laufen.' % RAW_TOPIC)
    args = ap.parse_args()

    if not os.path.isfile(INTRINSICS):
        print(f'Intrinsische Kalibrierung fehlt: {INTRINSICS}')
        return 1

    K, D, w, h = cg.load_intrinsics(INTRINSICS)
    print(f'Intrinsik geladen ({w}x{h}): fx={K[0,0]:.1f} fy={K[1,1]:.1f} '
          f'cx={K[0,2]:.1f} cy={K[1,2]:.1f}')
    ok_intr, lines = cg.check_intrinsics(K, D, w, h)
    print('  ' + lines[0])
    for line in lines[1:]:
        print('  ! ' + line)
    if not ok_intr:
        print('  -> Extrinsik wird trotzdem gerechnet, erbt aber diesen Fehler.')

    # ENTSCHEIDEND: /lokal/raw_image ist bereits entzerrt. Wird D hier nochmal
    # uebergeben, entzerrt solvePnP die Ecken ein zweites Mal -> Lage systematisch
    # falsch (gemessen: 0.4 deg Neigung, 0.3 deg Roll, 2 mm Hoehe).
    D_pnp = D if args.distorted else np.zeros((1, 5))
    print(f'  PnP rechnet mit '
          f'{"echtem D (Bild verzeichnet)" if args.distorted else "D = 0 (Bild entzerrt)"}')
    print()

    patch_l, patch_w = args.x_max - args.x_min, 2 * args.y_half
    if abs(patch_l - patch_w) / max(patch_l, patch_w) > 0.1:
        print(f'  Hinweis: Bodenausschnitt {patch_l:.2f} m lang x {patch_w:.2f} m breit ist '
              f'nicht quadratisch.\n  Bei {args.size}x{args.size} px Ausgabe werden die Pixel '
              f'dann nicht-quadratisch und gerade Linien\n  wirken gedreht/geschert. Fuer die '
              f'reine Sichtpruefung x-Spanne = y-Spanne waehlen.\n')

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
        node = Extrinsics(K, D_pnp)
        end = time.time() + 120
        while rclpy.ok() and len(node.results) < N_SAMPLES and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

        if len(node.results) < 5:
            print(f'\nNur {len(node.results)} Messungen -- Brett sichtbar? Beleuchtung?')
            node.destroy_node(); rclpy.shutdown(); return 1

        arr = np.array([(r[0], r[1], r[2]) for r in node.results])
        mean_rms = float(np.mean([r[4] for r in node.results]))
        med = np.median(arr, axis=0)
        std = np.std(arr, axis=0)
        n_used = len(node.results)

    if not args.recompute:
        print(f'\n  Mittlerer Reprojektionsfehler: {mean_rms:.2f} px '
              f'({"gut" if mean_rms < 0.6 else "grenzwertig -- Intrinsik pruefen"})')
    print(f'--- Kameralage ({n_used} Messungen, Median) ---')
    print(f'  Hoehe ueber Boden : {med[0]*100:6.2f} cm   (Streuung {std[0]*100:.2f} cm)')
    print(f'  Neigung nach unten: {med[1]:6.2f}°    (Streuung {std[1]:.2f}°)')
    print(f'  Rollwinkel        : {med[2]:6.2f}°    (Streuung {std[2]:.2f}°)')
    if std[1] > 1.0:
        print('\n  WARNUNG: Streuung der Neigung ueber 1° -- Brett lag evtl. nicht plan')
        print('  oder war zu klein im Bild. Fuer die Homographie besser wiederholen.')

    roll_forced = args.force_roll is not None
    if roll_forced:
        print(f'\n  Rollwinkel per --force-roll auf {args.force_roll:.2f}° gesetzt '
              f'(gemessen: {med[2]:.2f}°).')
        med[2] = args.force_roll

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
        print('\n  Nicht alle Ecken liegen im Kamerabild -- die betroffenen Ecken der')
        print('  Vogelperspektive bleiben schwarz. Fuer Spurverfolgung ist das ok (die')
        print('  handgetunte RoI macht das auch). Stoert es, mit --y-half / --x-max')
        print('  kleiner rechnen. Die Messung selbst bleibt gueltig.')

    with open(OUTFILE, 'w') as f:
        yaml.dump({
            'image_width': int(w), 'image_height': int(h),
            'camera_height_m': float(med[0]),
            'camera_pitch_deg': float(med[1]),
            'camera_roll_deg': float(med[2]),
            'roll_forced': bool(roll_forced),
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
            'std_roll_deg': float(std[2]),
        }, f, default_flow_style=False, sort_keys=True)
    print(f'\nGeschrieben: {OUTFILE}')
    print('Hinweis: warp_matrix gilt fuer das ENTZERRTE Bild -- vorher cv2.undistort(K, D).')

    if args.preview:
        print(f'\nHole ein Bild von {RAW_TOPIC} fuer die Vorschau ...')
        frame = grab_frame()
        if frame is None:
            print(f'  Kein Bild auf {RAW_TOPIC} -- laeuft ingress_node?')
        else:
            render_preview(frame, H_g2i, A, args.x_min, args.x_max, args.y_half,
                           args.size, PREVIEW_OUT)

    if node is not None:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
