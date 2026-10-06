#!/usr/bin/env python3
"""Bodenkalibrierung: Bild <-> Boden direkt an abgemessene Punkte fitten.

Zweck: zuverlaessig Entfernungen und Breiten auf dem Boden schaetzen. Es wird KEINE
Zerlegung in cx / Neigung / Roll / Yaw gemacht -- stattdessen wird die Homographie
Bild <-> Bodenebene als Ganzes an echte Zollstock-Maesse angepasst. Damit ist es egal,
ob ein Restfehler im Hauptpunkt, in der Neigung oder in der Restverzeichnung steckt:
er wird mitgefittet. Am Ende steht eine belegte Genauigkeit in Millimetern.

Koordinaten: X vorwaerts ab RADACHSE, Y nach LINKS, beides in cm bei der Eingabe.

--- Ablauf ---

1) Marker auslegen. Klebestreifen oder Muenzen auf abgemessene Stellen am Boden.
   Mindestens 4, sinnvoll 9-15, ueber den ganzen Bereich verteilt, den du spaeter
   nutzen willst -- auch seitlich, NICHT alle auf einer Linie.

   Empfohlenes Muster (X,Y in cm), gegen das reale Sichtfeld geprueft:
       20,0 20,5 20,-5 30,0 30,10 30,-10 40,0 40,10 40,-10
       55,0 55,15 55,-15 70,0 70,20 70,-20

   ACHTUNG, die Kamera sieht nach vorn hin nur einen schmalen Streifen. Bei 12,3 cm
   Hoehe und 62 Grad Objektiv ist seitlich hoechstens sichtbar:
       X = 20 cm -> +/- 9 cm       X = 40 cm -> +/- 17 cm
       X = 30 cm -> +/- 13 cm      X = 55 cm -> +/- 24 cm
       X = 70 cm -> +/- 30 cm
   Naeher als ~18 cm liegt gar nichts mehr im Bild. Marker weiter aussen zu legen
   bringt nichts -- sie sind schlicht nicht drauf.

2) Bild aufnehmen (ingress_node muss laufen):
       python3 scripts/tb/ground-calib.py --grab

3) Punkte anklicken und fitten:
       python3 scripts/tb/ground-calib.py --layout "20,0 20,5 20,-5 30,0 30,10 30,-10 40,0 40,10 40,-10 55,0 55,15 55,-15 70,0 70,20 70,-20"
   Es oeffnet sich ein Fenster. Klicke die Marker in der Reihenfolge der Liste an.
   Rueckgaengig: Backspace.  Abbrechen: q.  (Vorher auf dem Host: xhost +local:root)

   Ohne GUI geht es auch mit vorbereiteten Pixelkoordinaten:
       python3 scripts/tb/ground-calib.py --layout "..." --pixels "412,533 250,528 ..."

4) Ergebnis pruefen:
       python3 scripts/tb/extrinsics-view.py --ground   # Gitter aus DIESEM Fit

Ergebnis: Parameter/camera_info/ground_homography_800x600.yaml

--- Wie genau ist es? ---
Das Skript meldet pro Punkt den Restfehler in mm und zusaetzlich eine
Kreuzvalidierung (jeder Punkt einmal weggelassen). Der Kreuzvalidierungswert ist der
ehrliche Wert -- der reine Fit-Restfehler ist immer optimistisch.
"""
import argparse
import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _camera_geometry as cg  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PARAM_DIR = os.path.join(REPO, 'workspace/src/Follow_Lane/Follow_Lane/Parameter/camera_info')
INTRINSICS = os.path.join(PARAM_DIR, 'imx219_800x600.yaml')
OUTFILE = os.path.join(PARAM_DIR, 'ground_homography_800x600.yaml')
FRAME = os.path.join(REPO, 'logs', 'ground_calib_frame.png')

RAW_TOPIC = '/lokal/raw_image'


def grab_frame(path, timeout=20.0):
    """Ein Bild von RAW_TOPIC holen und ablegen."""
    import time

    import rclpy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    br = CvBridge()
    holder = {}
    rclpy.init()
    node = rclpy.create_node('ground_calib_grab')
    node.create_subscription(
        Image, RAW_TOPIC,
        lambda m: holder.setdefault('img', br.imgmsg_to_cv2(m, desired_encoding='bgr8')), 1)
    end = time.time() + timeout
    while rclpy.ok() and 'img' not in holder and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if 'img' not in holder:
        print(f'Kein Bild auf {RAW_TOPIC}. Laeuft ingress_node?')
        return None
    cv2.imwrite(path, holder['img'])
    print(f'Bild geschrieben: {path}  ({holder["img"].shape[1]}x{holder["img"].shape[0]})')
    return holder['img']


def parse_layout(text):
    """'20,0 20,15 35,-20' -> [(0.20, 0.00), (0.20, 0.15), (0.35, -0.20)] in Metern."""
    pts = []
    for tok in text.replace(';', ' ').split():
        try:
            x, y = tok.split(',')
            pts.append((float(x) / 100.0, float(y) / 100.0))
        except ValueError:
            raise SystemExit(f'Layout-Eintrag "{tok}" nicht lesbar -- erwartet X,Y in cm.')
    return pts


def click_points(img, ground_pts):
    """Marker der Reihe nach anklicken. Backspace = zurueck, q = abbrechen."""
    picked = []
    win = 'Bodenkalibrierung -- Marker anklicken'
    disp = [img.copy()]

    def redraw():
        d = img.copy()
        for i, (u, v) in enumerate(picked):
            cv2.circle(d, (int(u), int(v)), 6, (0, 255, 0), -1, cv2.LINE_AA)
            cv2.putText(d, f'{i + 1}', (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        if len(picked) < len(ground_pts):
            X, Y = ground_pts[len(picked)]
            txt = (f'Punkt {len(picked) + 1}/{len(ground_pts)}:  '
                   f'X = {X * 100:.0f} cm voraus, Y = {Y * 100:+.0f} cm '
                   f'({"links" if Y > 0 else "rechts" if Y < 0 else "mittig"})')
        else:
            txt = 'Alle Punkte gesetzt -- ENTER zum Fitten, Backspace zum Korrigieren'
        cv2.rectangle(d, (0, 0), (d.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(d, txt, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)
        disp[0] = d
        cv2.imshow(win, d)

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN and len(picked) < len(ground_pts):
            picked.append((float(x), float(y)))
            redraw()

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)
    redraw()
    while True:
        key = cv2.waitKey(20) & 0xFF
        if key in (ord('q'), 27):
            cv2.destroyAllWindows()
            return None
        if key == 8 and picked:          # Backspace
            picked.pop()
            redraw()
        if key in (13, 10) and len(picked) == len(ground_pts):
            break
    cv2.destroyAllWindows()
    return picked


def fit_and_report(img_pts, ground_pts):
    """Homographie Bild -> Boden fitten und die Genauigkeit ehrlich beziffern."""
    src = np.array(img_pts, dtype=np.float64).reshape(-1, 1, 2)
    dst = np.array(ground_pts, dtype=np.float64).reshape(-1, 1, 2)
    H, _ = cv2.findHomography(src, dst, method=0)     # 0 = alle Punkte, kein RANSAC
    if H is None:
        raise SystemExit('Homographie liess sich nicht loesen -- liegen die Punkte auf '
                         'einer Geraden? Marker breiter verteilen.')

    proj = cv2.perspectiveTransform(src, H).reshape(-1, 2)
    gt = np.array(ground_pts)
    err = np.linalg.norm(proj - gt, axis=1) * 1000.0     # mm

    print('\n--- Fit-Restfehler pro Punkt ---')
    for (X, Y), e, p in zip(ground_pts, err, proj):
        print(f'  Soll X {X * 100:6.1f} Y {Y * 100:+6.1f} cm  ->  '
              f'Fit X {p[0] * 100:6.1f} Y {p[1] * 100:+6.1f} cm   Fehler {e:5.1f} mm')
    print(f'  RMS {np.sqrt(np.mean(err ** 2)):.1f} mm, max {err.max():.1f} mm')

    # Kreuzvalidierung: jeder Punkt einmal raus. Das ist der belastbare Wert --
    # der Fit-Restfehler oben ist zwangslaeufig optimistisch, weil derselbe Punkt
    # die Loesung mitbestimmt hat.
    cv_err = []
    if len(ground_pts) >= 5:
        for i in range(len(ground_pts)):
            keep = [j for j in range(len(ground_pts)) if j != i]
            Hi, _ = cv2.findHomography(src[keep], dst[keep], method=0)
            if Hi is None:
                continue
            pi = cv2.perspectiveTransform(src[i:i + 1], Hi).reshape(2)
            cv_err.append(np.linalg.norm(pi - gt[i]) * 1000.0)
        cv_err = np.array(cv_err)
        print(f'\n  Kreuzvalidierung (jeder Punkt einmal weggelassen):')
        print(f'  RMS {np.sqrt(np.mean(cv_err ** 2)):.1f} mm, max {cv_err.max():.1f} mm'
              f'   <-- das ist die ehrliche Genauigkeit')
    else:
        print('\n  Weniger als 5 Punkte -- keine Kreuzvalidierung moeglich. '
              'Mehr Marker geben eine belastbarere Aussage.')
    return H, err, (np.array(cv_err) if len(cv_err) else None)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--grab', action='store_true',
                    help=f'ein Bild von {RAW_TOPIC} holen und nach logs/ ablegen, dann Ende')
    ap.add_argument('--image', default=FRAME,
                    help='Bild, auf dem geklickt wird (Default: das zuletzt gegrabbte)')
    ap.add_argument('--layout', default=None,
                    help='Soll-Positionen der Marker, "X,Y X,Y ..." in cm '
                         '(X vorwaerts ab Radachse, Y links positiv)')
    ap.add_argument('--pixels', default=None,
                    help='Pixelkoordinaten statt Klicken, "u,v u,v ..." in derselben '
                         'Reihenfolge wie --layout (fuer den Betrieb ohne GUI)')
    args = ap.parse_args()

    if args.grab:
        return 0 if grab_frame(FRAME) is not None else 1

    if not args.layout:
        ap.error('--layout wird gebraucht. Beispiel: '
                 '--layout "20,0 20,15 20,-15 35,0 35,20 35,-20"')

    ground_pts = parse_layout(args.layout)
    if len(ground_pts) < 4:
        raise SystemExit('Mindestens 4 Punkte noetig, sinnvoll 6-10.')

    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f'Bild nicht lesbar: {args.image}\n'
                         f'Erst aufnehmen mit: python3 {sys.argv[0]} --grab')
    h, w = img.shape[:2]
    print(f'Bild {args.image}  ({w}x{h}), {len(ground_pts)} Marker erwartet.')

    if args.pixels:
        img_pts = [tuple(float(v) for v in tok.split(',')) for tok in args.pixels.split()]
        if len(img_pts) != len(ground_pts):
            raise SystemExit(f'{len(img_pts)} Pixel- aber {len(ground_pts)} Bodenpunkte.')
    else:
        img_pts = click_points(img, ground_pts)
        if img_pts is None:
            print('Abgebrochen.')
            return 1

    H, err, cv_err = fit_and_report(img_pts, ground_pts)

    # Gegenprobe gegen die bisherige, aus cx/Neigung/Roll gerechnete Projektion.
    if os.path.isfile(INTRINSICS):
        K, D, cw, ch = cg.load_intrinsics(INTRINSICS)
        print(f'\n  Zum Vergleich: die bisherige Modell-Projektion setzt "geradeaus" auf '
              f'Spalte u = cx = {K[0, 2]:.1f} (Bildmitte {w / 2:.0f}).')
        gy = cv2.perspectiveTransform(
            np.array([[[w / 2.0, h * 0.85]]], np.float64), H).reshape(2)
        print(f'  Dieser Fit liest an Bildmitte/unten: X {gy[0] * 100:.1f} cm, '
              f'Y {gy[1] * 100:+.1f} cm.')

    # Vorzeichen so drehen, dass die Tiefe fuer reale Bodenpunkte positiv ist --
    # sonst verwirft jeder "vor der Kamera"-Test spaeter alle Punkte.
    ref = np.mean(np.array(ground_pts), axis=0)
    Hinv = cg.normalize_ground_homography(np.linalg.inv(H), ref)
    with open(OUTFILE, 'w') as f:
        yaml.dump({
            'image_width': int(w), 'image_height': int(h),
            'note': 'Direkt an abgemessene Bodenpunkte gefittet. Gilt fuer das ENTZERRTE '
                    'Bild (ingress_node mit undistort:=true) bei genau dieser Aufloesung.',
            'frame': 'X vorwaerts ab Radachse, Y links, in Metern',
            'H_image_to_ground': [float(x) for x in H.ravel()],
            'H_ground_to_image': [float(x) for x in Hinv.ravel()],
            'markers_ground_m': [[float(a), float(b)] for a, b in ground_pts],
            'markers_pixel': [[float(a), float(b)] for a, b in img_pts],
            'fit_rms_mm': float(np.sqrt(np.mean(err ** 2))),
            'fit_max_mm': float(err.max()),
            'crossval_rms_mm': (float(np.sqrt(np.mean(cv_err ** 2)))
                                if cv_err is not None else None),
            'crossval_max_mm': float(cv_err.max()) if cv_err is not None else None,
        }, f, default_flow_style=False, sort_keys=True)
    print(f'\nGeschrieben: {OUTFILE}')
    print('Pruefen mit: python3 scripts/tb/extrinsics-view.py --ground')
    return 0


if __name__ == '__main__':
    sys.exit(main())
