#!/usr/bin/env python3
"""Abnahmetest fuer die Entzerrung: bleiben gerade Linien gerade?

Der einzige Test, der ohne jede Kenntnis von Pose, Abstand oder Position auskommt:
Eine Gerade in der Welt muss im entzerrten Bild eine Gerade sein. Krummt sie sich,
steckt Restverzeichnung drin -- egal was die Reprojektionsfehler der Kalibrierung sagen.

Genau damit wurde am 2026-09-09 nachgewiesen, dass die damalige Kalibrierung das Bild
SCHLECHTER machte als gar keine Entzerrung (3.29 % gegen 1.79 %).

Ziel: ein Blatt mit Raster (Flipchart-Papier) flach auf den Boden, formatfuellend,
moeglichst bis in die Bildecken. Dann:

    python3 scripts/tb/undistort-check.py --grab          # Rohbild holen
    python3 scripts/tb/undistort-check.py                 # auswerten

Gemessen wird die Kruemmung der Rasterlinien als Prozent eines Rasterquadrats --
skalenfrei, also unabhaengig von Aufloesung und Rasterweite.

Bewertung:
    entzerrt deutlich kleiner als roh   -> Kalibrierung taugt
    entzerrt ~ roh                      -> Entzerrung bringt nichts (aber schadet nicht)
    entzerrt GROESSER als roh           -> Kalibrierung ist kaputt, nicht verwenden

ACHTUNG beim Interpretieren: Papier, das nicht plan liegt, erzeugt einen Rauschboden
(gemessen ~1.4 %). Unterschiede darunter sind nicht aussagekraeftig.
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _camera_geometry as cg  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PARAM_DIR = os.path.join(REPO, 'workspace/src/Follow_Lane/Follow_Lane/Parameter/camera_info')
INTRINSICS = os.path.join(PARAM_DIR, 'imx219_800x600.yaml')
FRAME = os.path.join(REPO, 'logs', 'raster_roh.png')

RAW_TOPIC = '/camera/image_raw/compressed'


def dark_minima(row, min_depth=6):
    """Subpixelgenaue Lage dunkler Linien in einem Helligkeitsprofil."""
    r = cv2.GaussianBlur(row.astype(np.float32).reshape(1, -1), (9, 1), 0).ravel()
    bg = cv2.GaussianBlur(r.reshape(1, -1), (61, 1), 0).ravel()
    d = bg - r
    out = []
    for i in range(2, len(r) - 2):
        if d[i] > min_depth and d[i] >= d[i - 1] and d[i] > d[i + 1]:
            a, b, c = d[i - 1], d[i], d[i + 1]
            den = a - 2 * b + c
            out.append(i + (0.5 * (a - c) / den if abs(den) > 1e-6 else 0.0))
    return out


def trace_lines(gray, rows, tol=7.0, min_len=30):
    """Etwa senkrechte dunkle Linien ueber die Bildzeilen hinweg verfolgen."""
    tracks = []
    step = rows[1] - rows[0] if len(rows) > 1 else 1
    for y in rows:
        for x in dark_minima(gray[y]):
            best, bd = None, tol
            for t in tracks:
                if t['last_y'] == y:
                    continue
                pts = t['pts']
                if len(pts) >= 3:
                    ys = np.array([p[1] for p in pts[-6:]])
                    xs = np.array([p[0] for p in pts[-6:]])
                    pred = np.polyval(np.polyfit(ys, xs, 1), y)
                else:
                    pred = pts[-1][0]
                if abs(y - t['last_y']) <= step * 2 and abs(x - pred) < bd:
                    bd, best = abs(x - pred), t
            if best is None:
                tracks.append({'pts': [(x, y)], 'last_y': y})
            else:
                best['pts'].append((x, y))
                best['last_y'] = y
    return [np.array(t['pts']) for t in tracks if len(t['pts']) >= min_len]


def curvature(img, band=(0.30, 0.97)):
    """Mittlere Kruemmung der Rasterlinien in % eines Rasterquadrats."""
    big = cv2.resize(img, (2400, 1800))          # hochskalieren fuers Subpixel-Tracking
    gray = cv2.cvtColor(big, cv2.COLOR_BGR2GRAY)
    y0, y1 = int(1800 * band[0]), int(1800 * band[1])
    keep = [P for P in trace_lines(gray, list(range(y0, y1, 4)))
            if (P[:, 1].max() - P[:, 1].min()) > (y1 - y0) * 0.5]
    if len(keep) < 4:
        return None
    ym = (y0 + y1) / 2
    rows = sorted((np.polyval(np.polyfit(P[:, 1], P[:, 0], 1), ym),
                   float(np.abs((P - P.mean(0)) @ np.linalg.svd(P - P.mean(0))[2][1]).max()))
                  for P in keep)
    xs = np.array([r[0] for r in rows])
    sp = np.median(np.diff(xs))
    pct = np.array([100 * r[1] / sp for r in rows])
    rel = np.abs((xs - 1200) / 1200)
    return dict(n=len(rows), gesamt=float(pct.mean()),
                mitte=float(pct[rel <= 0.35].mean()) if (rel <= 0.35).any() else float('nan'),
                aussen=float(pct[rel > 0.35].mean()) if (rel > 0.35).any() else float('nan'))


def grab(path):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    br, holder = CvBridge(), {}
    rclpy.init()
    node = rclpy.create_node('undistort_check_grab')
    node.create_subscription(
        CompressedImage, RAW_TOPIC,
        lambda m: holder.setdefault('i', br.compressed_imgmsg_to_cv2(m, 'bgr8')),
        qos_profile_sensor_data)
    import time
    end = time.time() + 20
    while rclpy.ok() and 'i' not in holder and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if 'i' not in holder:
        print(f'Kein Bild auf {RAW_TOPIC}. Laeuft die Kamera?')
        return None
    cv2.imwrite(path, holder['i'])
    print(f'Rohbild geschrieben: {path} ({holder["i"].shape[1]}x{holder["i"].shape[0]})')
    return holder['i']


def compare_models(raw):
    """Alle Modelle aus dem Tarball rechnen und am selben Rasterbild vergleichen.
    Das Rasterbild ist der eigentliche Schiedsrichter -- der Reprojektionsfehler
    einer Kalibrierung sagt nichts darueber, ob sie gerade Linien gerade laesst."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'camint', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'camera-intrinsics.py'))
    ci = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ci)

    if not os.path.isfile(ci.TARBALL):
        print(f'Keine Aufnahmen: {ci.TARBALL}')
        return 1
    images = ci.load_images()
    print(f'{len(images)} Kalibrieraufnahmen\n')

    base = curvature(raw)
    if base is None:
        print('Zu wenige Rasterlinien im Bild erkannt.')
        return 1
    cw, ch = raw.shape[1], raw.shape[0]

    print(f"{'Modell':40s} {'RMS':>6} {'cx':>7} {'Mitte':>7} {'aussen':>7} {'gesamt':>7}")
    print(f"  {'(gar keine Entzerrung)':38s} {'-':>6} {'-':>7} "
          f"{base['mitte']:6.2f}% {base['aussen']:6.2f}% {base['gesamt']:6.2f}%")
    best, best_c = None, base['gesamt']
    for key in ci.MODELS:
        r = ci.fit(images, key)
        if r is None:
            continue
        K, D = r['K'], r['D'].reshape(1, -1)
        m = cv2.initUndistortRectifyMap(K, D, None, K, (cw, ch), cv2.CV_16SC2)
        c = curvature(cv2.remap(raw, m[0], m[1], cv2.INTER_LINEAR))
        if c is None:
            continue
        print(f"  {key:38s} {r['rms']:6.3f} {K[0,2]:7.1f} "
              f"{c['mitte']:6.2f}% {c['aussen']:6.2f}% {c['gesamt']:6.2f}%")
        if c['gesamt'] < best_c:
            best, best_c = key, c['gesamt']
    print()
    if best is None:
        print('  Kein Modell macht das Bild gerader als das Rohbild. Neu aufnehmen.')
    else:
        print(f'  BESTES MODELL: {best}  ({best_c:.2f} % gegen {base["gesamt"]:.2f} % roh)')
        print(f'  Uebernehmen:  python3 scripts/tb/camera-intrinsics.py --calibrate '
              f'--write {best}  &&  colcon build')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--grab', action='store_true', help=f'Rohbild von {RAW_TOPIC} holen')
    ap.add_argument('--models', action='store_true',
                    help='ALLE Kalibriermodelle aus logs/intrinsik_aufnahmen.tar.gz '
                         'rechnen und am Rasterbild gegeneinander testen. Beantwortet '
                         '"welches Modell nehme ich?" ohne jedesmal colcon build.')
    ap.add_argument('--image', default=FRAME, help='auszuwertendes ROHBILD (unverzerrt korrigiert)')
    args = ap.parse_args()

    if args.grab:
        return 0 if grab(FRAME) is not None else 1

    raw = cv2.imread(args.image)
    if raw is None:
        print(f'Bild nicht lesbar: {args.image}\nErst aufnehmen: --grab')
        return 1

    if args.models:
        return compare_models(raw)

    K, D, cw, ch = cg.load_intrinsics(INTRINSICS)
    if raw.shape[1] != cw or raw.shape[0] != ch:
        print(f'Bild {raw.shape[1]}x{raw.shape[0]} wird auf die Kalibriergroesse '
              f'{cw}x{ch} skaliert.')
        raw = cv2.resize(raw, (cw, ch))

    print(f'\nKalibrierung: fx {K[0,0]:.1f}  cx {K[0,2]:.1f} (Bildmitte {cw/2:.0f})  '
          f'p1 {D.ravel()[2]:+.4f}  p2 {D.ravel()[3]:+.4f}\n')

    m = cv2.initUndistortRectifyMap(K, D, None, K, (cw, ch), cv2.CV_16SC2)
    und = cv2.remap(raw, m[0], m[1], cv2.INTER_LINEAR)

    print(f"{'':34s} {'Mitte':>8} {'aussen':>8} {'gesamt':>8}")
    out = {}
    for tag, img in (('Rohbild (keine Entzerrung)', raw),
                     ('mit dieser Kalibrierung entzerrt', und)):
        c = curvature(img)
        if c is None:
            print(f'  {tag:32s}  zu wenige Rasterlinien erkannt')
            return 1
        out[tag] = c
        print(f'  {tag:32s} {c["mitte"]:7.2f}% {c["aussen"]:7.2f}% {c["gesamt"]:7.2f}%'
              f'   ({c["n"]} Linien)')

    roh = out['Rohbild (keine Entzerrung)']['gesamt']
    ent = out['mit dieser Kalibrierung entzerrt']['gesamt']
    print()
    if ent > roh * 1.15:
        print(f'  URTEIL: KAPUTT. Die Entzerrung macht das Bild krummer ({ent:.2f} % gegen '
              f'{roh:.2f} %).\n          Nicht verwenden -- neu kalibrieren.')
    elif ent < roh * 0.85:
        print(f'  URTEIL: GUT. Die Entzerrung macht das Bild gerader ({ent:.2f} % gegen '
              f'{roh:.2f} %).')
    else:
        print(f'  URTEIL: WIRKUNGSLOS ({ent:.2f} % gegen {roh:.2f} %). Sie schadet nicht,')
        print('          bringt aber auch nichts. Moeglich: das Objektiv verzeichnet kaum,')
        print('          oder das Papier liegt nicht plan (Rauschboden ~1.4 %).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
