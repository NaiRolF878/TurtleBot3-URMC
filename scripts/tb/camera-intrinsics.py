#!/usr/bin/env python3
"""Intrinsische Kalibrierung -- Treiber um camera_calibration.MonoCalibrator.

Nutzt bewusst die Bibliothek des offiziellen Werkzeugs (Erkennung, Auswahl neuer
Ansichten, Fortschrittsbalken, Fit, Tarball) und ersetzt nur die GUI. Denn nicht die
Bibliothek war das Problem, sondern drei Dinge drumherum -- siehe docs/MEMORY.md:

  1. Die GUI stuerzte bei entarteten Ansichten ab (`p_size = 0.000`), ausgerechnet bei
     den stark gekippten und randnahen. Wer die Abstuerze vermeidet, sammelt grosse,
     mittige, frontale Ansichten -- und aus DENEN sind cx, cy, p1, p2 mathematisch
     nicht bestimmbar. Hier werden solche Ansichten vor dem Fit AUSSORTIERT.
  2. Die Rohbilder landeten in /tmp und waren nach dem Container-Neustart weg. Hier
     gehen sie nach logs/ -- damit ist ein Fit ohne neue Aufnahmesession wiederholbar.
  3. Es gab kein Modell mit festgehaltenem Hauptpunkt. Hier werden drei Varianten
     aus DENSELBEN Bildern gerechnet und gegenuebergestellt.

Ergebnis der alten Kalibrierung zum Vergleich: cx 76 px daneben, p2 15-fach zu gross,
Entzerrung nachweislich schlechter als gar keine (3.29 % gegen 1.79 % Linienkruemmung).

--- Ablauf ---

1) Aufnehmen (Bot laeuft, Kamera liefert; ingress_node wird NICHT gebraucht):
       python3 scripts/tb/camera-intrinsics.py --capture

   Es geht ein Fenster auf (dafuer auf dem HOST einmal: xhost +local:root).
   Parallel liegt dasselbe Bild auf /lokal/intrinsik_view, falls das Fenster klemmt:
       ros2 run rqt_image_view rqt_image_view /lokal/intrinsik_view

   Die Live-Ansicht zeigt die Fortschrittsbalken X / Y / Size / Skew wie im Original.
   Alle vier muessen voll werden. Strg+C speichert den Tarball.
   WICHTIG: beim starken Kippen NAEHER rangehen, sonst wird das Brett zu klein.

2) Rechnen -- drei Modelle aus denselben Bildern:
       python3 scripts/tb/camera-intrinsics.py --calibrate

3) Uebernehmen und pruefen:
       python3 scripts/tb/camera-intrinsics.py --calibrate --write fixed
       colcon build
       python3 scripts/tb/undistort-check.py --grab && python3 scripts/tb/undistort-check.py

Abnahme: cx/cy ~ Bildmitte +/-15, |p1|,|p2| < 0.005, fx ~ fy, fx ~ 663, RMS < 0.5 px.
"""
import argparse
import io
import os
import sys
import tarfile
import time

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from camera_calibration.calibrator import (  # noqa: E402
    ChessboardInfo, MonoCalibrator, Patterns)

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PARAM_DIR = os.path.join(REPO, 'workspace/src/Follow_Lane/Follow_Lane/Parameter/camera_info')
TARBALL = os.path.join(REPO, 'logs', 'intrinsik_aufnahmen.tar.gz')
OUTFILE = os.path.join(PARAM_DIR, 'imx219_800x600.yaml')

RAW_TOPIC = '/camera/image_raw/compressed'
PREVIEW_TOPIC = '/lokal/intrinsik_view'

# Brett wie aufgedruckt. Achtung: -s erwartet die KACHELN (7x5), nicht die inneren Ecken.
BOARD = dict(n_cols=7, n_rows=5, dim=0.030, marker_size=0.022, aruco_dict='5x5_250')

HFOV_DEG = 62.2         # Datenblatt IMX219, fuer Startwert und Abnahme
MIN_PSIZE = 0.18        # darunter entartet -- genau daran ist die alte Sitzung gestorben

MODELS = {
    'fixed':     (cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_ZERO_TANGENT_DIST
                  | cv2.CALIB_FIX_K3,
                  'cx,cy fest + p1=p2=0 + k3=0   (EMPFOHLEN)'),
    'k3frei':    (cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_ZERO_TANGENT_DIST,
                  'wie oben, aber k3 frei'),
    'notangent': (cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3,
                  'p1=p2=k3=0, Hauptpunkt frei'),
    'frei':      (0,
                  'alles frei -- so entstand die kaputte alte Kalibrierung'),
}


def make_calibrator(flags=0, max_speed=-1.0):
    """max_speed > 0 verwirft Bilder, in denen sich das Brett zu schnell bewegt --
    verwackelte Aufnahmen treiben sonst den Reprojektionsfehler hoch. Beim Rechnen
    aus dem Tarball irrelevant (dort auf -1 lassen)."""
    board = ChessboardInfo(pattern='charuco', **BOARD)
    return MonoCalibrator([board], flags=flags, pattern=Patterns.ChArUco, name='imx219',
                          max_chessboard_speed=max_speed)


def draw_bars(scrib, params, n_db, n_use=0, cur_size=None):
    """Fortschrittsbalken X / Y / Size / Skew, wie im Original-Werkzeug."""
    w = scrib.shape[1]
    bar = np.zeros((92, w, 3), np.uint8)
    cv2.putText(bar, f'{n_db} Ansichten, davon {n_use} BRAUCHBAR', (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255) if n_use >= 25 else (0, 165, 255), 2, cv2.LINE_AA)
    if cur_size is not None and cur_size < MIN_PSIZE:
        cv2.putText(bar, f'BRETT ZU KLEIN ({cur_size:.2f} < {MIN_PSIZE}) -- NAEHER RAN!',
                    (330, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 80, 255), 2, cv2.LINE_AA)
    if not params:
        cv2.putText(bar, 'Brett zeigen ...', (150, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 200, 255), 1, cv2.LINE_AA)
        return np.vstack([bar, scrib])

    x0, bw = 8, min(300, w - 120)
    for i, (name, lo, hi, prog) in enumerate(params):
        y = 40 + i * 13
        full = prog >= 1.0
        cv2.rectangle(bar, (x0 + 46, y - 8), (x0 + 46 + bw, y), (60, 60, 60), -1)
        cv2.rectangle(bar, (x0 + 46, y - 8), (x0 + 46 + int(bw * prog), y),
                      (0, 200, 0) if full else (0, 160, 255), -1)
        cv2.putText(bar, name, (x0, y - 1), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                    (200, 255, 200) if full else (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(bar, f'{prog * 100:3.0f}%', (x0 + 52 + bw, y - 1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1, cv2.LINE_AA)
    if all(p >= 1.0 for (_, _, _, p) in params) or n_db >= 40:
        cv2.putText(bar, 'GENUG -- Strg+C, dann --calibrate', (x0 + 60 + bw, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    return np.vstack([bar, scrib])


def capture(window=True, topic=True, max_speed=4.0):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image

    mc = make_calibrator(max_speed=max_speed)
    br = CvBridge()
    st = {'prev': 0.0, 'vis': None, 'seen': 0, 'warned': 0.0, 'err': None}
    WIN = 'Intrinsik-Kalibrierung  (q = beenden und speichern)'

    rclpy.init()
    node = rclpy.create_node('camera_intrinsics_capture')
    pub = node.create_publisher(Image, PREVIEW_TOPIC, 1) if topic else None

    def cb(msg):
        st['seen'] += 1
        try:
            # MonoCalibrator.mkgray() erwartet ein sensor_msgs/Image (nutzt msg.encoding),
            # deshalb das komprimierte Bild dekodieren und neu verpacken.
            img = br.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            n_before = len(mc.db)
            drawable = mc.handle_msg(br.cv2_to_imgmsg(img, encoding='bgr8'))
            if len(mc.db) > n_before:
                p = mc.db[-1][0]
                print(f'[{len(mc.db):3d}] angenommen   X {p[0]:.2f}  Y {p[1]:.2f}  '
                      f'size {p[2]:.2f}  skew {p[3]:.2f}'
                      + ('   <- klein, wird vor dem Fit verworfen' if p[2] < MIN_PSIZE else ''))
            n_use = sum(1 for (pp, _) in mc.db if pp[2] >= MIN_PSIZE)
            cur = mc.db[-1][0][2] if len(mc.db) > n_before else st.get('cur')
            st['cur'] = cur
            vis = draw_bars(drawable.scrib, drawable.params, len(mc.db), n_use, cur)
        except Exception as e:                    # nie den Aufnahmelauf abbrechen
            if st['err'] != str(e):
                st['err'] = str(e)
                print(f'  Bildverarbeitung fehlgeschlagen: {e}')
            return
        st['vis'] = vis
        if pub is not None and time.time() - st['prev'] > 0.1:
            st['prev'] = time.time()
            pub.publish(br.cv2_to_imgmsg(vis, encoding='bgr8'))

    node.create_subscription(CompressedImage, RAW_TOPIC, cb, qos_profile_sensor_data)

    print(f'Aufnahme von {RAW_TOPIC}')
    if window:
        print('Fenster geht gleich auf. Fehlt es, auf dem HOST einmal: xhost +local:root')
    if topic:
        print(f'Zusaetzlich als Topic: rqt_image_view {PREVIEW_TOPIC}')
    print(f'Brett DEUTLICH naeher als beim letzten Mal -- p_size muss ueber {MIN_PSIZE}')
    print(f'kommen, sonst ist die Ansicht wertlos. Nach jeder Position kurz STILLHALTEN')
    print(f'(verwackelte Bilder werden ab {max_speed} px Bewegung verworfen).')
    print('Alle vier Balken (X / Y / Size / Skew) vollmachen.')
    print('q im Fenster oder Strg+C beendet und speichert.\n')

    can_window = window
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            if st['vis'] is None:
                # Ohne Kamerabild trotzdem ein Fenster zeigen -- sonst sieht der
                # Nutzer gar nichts und haelt das Skript fuer kaputt.
                ph = np.zeros((200, 640, 3), np.uint8)
                cv2.putText(ph, 'Warte auf Kamerabild ...', (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2, cv2.LINE_AA)
                cv2.putText(ph, RAW_TOPIC, (20, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
                cv2.putText(ph, 'Laeuft die Kamera auf dem Bot?  (botup)', (20, 140),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA)
                cv2.putText(ph, f'{st["seen"]} Bilder empfangen', (20, 175),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
                st['vis'] = ph

            if can_window:
                try:
                    cv2.imshow(WIN, st['vis'])
                    if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                        break
                except cv2.error as e:
                    can_window = False
                    print(f'\n  Fenster nicht moeglich ({str(e).splitlines()[-1][:60]}).')
                    print('  Auf dem HOST "xhost +local:root" ausfuehren, oder stattdessen:')
                    print(f'  ros2 run rqt_image_view rqt_image_view {PREVIEW_TOPIC}\n')

            # Ohne Bilder passiert gar nichts -- das aber bitte laut sagen.
            if st['seen'] == 0 and time.time() - st['warned'] > 5.0:
                st['warned'] = time.time()
                print(f'  Noch kein Bild auf {RAW_TOPIC}. Laeuft die Kamera auf dem Bot?')
                print('  Pruefen: ros2 topic hz /camera/image_raw/compressed')
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # ExternalShutdownException (SIGTERM) u.ae. -- die Aufnahmen sind zu wertvoll,
        # um sie an einer Abbruch-Exception zu verlieren.
        print(f'\nAbbruch ({type(e).__name__}) -- speichere trotzdem, was da ist.')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if len(mc.db) < 8:
        print(f'\nNur {len(mc.db)} Ansichten -- zu wenig, nichts gespeichert.')
        return 1
    os.makedirs(os.path.dirname(TARBALL), exist_ok=True)
    # NICHT mc.do_tarfile_save() -- das schreibt auch ost.yaml und braucht dafuer
    # self.distortion, also eine bereits gelaufene Kalibrierung. Nur die Bilder.
    with tarfile.open(TARBALL, 'w:gz') as tf:
        for i, (_params, im) in enumerate(mc.db):
            buf = cv2.imencode('.png', im)[1].tobytes()
            ti = tarfile.TarInfo(f'left-{i:04d}.png')
            ti.size = len(buf)
            tf.addfile(ti, io.BytesIO(buf))
    n_use = sum(1 for (pp, _) in mc.db if pp[2] >= MIN_PSIZE)
    print(f'\n{len(mc.db)} Ansichten -> {TARBALL}')
    print(f'Davon BRAUCHBAR (p_size >= {MIN_PSIZE}): {n_use}')
    if n_use < 25:
        print(f'  ZU WENIG. Das Brett war fast immer zu klein im Bild.')
        print(f'  Beim naechsten Mal DEUTLICH naeher ran -- es soll einen guten Teil')
        print(f'  des Bildes ausfuellen, nicht nur ein Fleckchen sein.')
    print('Weiter mit: python3 scripts/tb/camera-intrinsics.py --calibrate')
    return 0


def load_images():
    from camera_calibration.calibrator import image_from_archive
    with tarfile.open(TARBALL, 'r') as ar:
        names = [f for f in ar.getnames()
                 if f.startswith('left') and f.endswith(('.png', '.pgm'))]
        return [image_from_archive(ar, n) for n in sorted(names)]


def fit(images, key):
    """Ein Modell rechnen. Entartete Ansichten vorher aussortieren -- daran ist die
    alte Sitzung gestorben (cvFindExtrinsicCameraParams2 stirbt an ihnen)."""
    flags, desc = MODELS[key]
    mc = make_calibrator(flags)
    board = mc._boards[0]
    size = (images[0].shape[1], images[0].shape[0])

    # Ansichten einmal vermessen, dann nach p_size sortiert filtern.
    scored = []
    for img in images:
        try:
            # wirft CalibrationException, wenn gar keine Ecken gefunden werden
            got = mc.collect_corners([img])
        except Exception:
            continue
        if not got:
            continue
        corners, ids, b = got[0]
        p = mc.get_parameters(corners, ids, b, size)
        scored.append((p[2], corners, ids, b))
    scored.sort(key=lambda t: t[0])

    w, h = size
    fx0 = (w / 2) / np.tan(np.radians(HFOV_DEG / 2))
    K0 = np.array([[fx0, 0, w / 2.], [0, fx0, h / 2.], [0, 0, 1.]])

    # NICHT mc.cal_fromcorners() benutzen! Deren ChArUco-Zweig ruft
    # cv2.aruco.calibrateCameraCharuco OHNE flags= auf -- self.calib_flags wird dort
    # stillschweigend ignoriert (der Chessboard-Zweig direkt darunter uebergibt sie).
    # Genau deshalb war CALIB_FIX_PRINCIPAL_POINT ueber das offizielle Werkzeug mit
    # einem ChArUco-Brett gar nicht erreichbar. Also selbst aufrufen.
    #
    # Schwelle notfalls anheben: entartete Ansichten (p_size ~ 0) bringen
    # cvFindExtrinsicCameraParams2 zum Absturz -- daran ist die GUI dreimal gestorben.
    rms = K = D = None
    for thr in (MIN_PSIZE, 0.22, 0.26, 0.30, 0.35):
        good = [(c, i, b) for (ps, c, i, b) in scored if ps >= thr]
        if len(good) < 6:
            continue
        try:
            rms, K, D, _, _ = cv2.aruco.calibrateCameraCharuco(
                [g[0] for g in good], [g[1] for g in good], board.charuco_board,
                size, K0, np.zeros((5, 1)),
                flags=flags | cv2.CALIB_USE_INTRINSIC_GUESS)
        except Exception:
            continue
        dropped = len(scored) - len(good)
        used_thr = thr
        break
    if K is None:
        print(f'\n  --- {desc} ---\n    FEHLGESCHLAGEN: kein Fit moeglich, auch nicht '
              f'nach Aussortieren. Zu wenige brauchbare Ansichten?')
        return None

    K, D = np.asarray(K), np.asarray(D).ravel()
    rms = float(rms)
    hfov = 2 * np.degrees(np.arctan((w / 2) / K[0, 0]))
    ok = (abs(K[0, 2] - w / 2) < 15 and abs(K[1, 2] - h / 2) < 15
          and abs(D[2]) < 0.005 and abs(D[3]) < 0.005
          and abs(K[0, 0] - K[1, 1]) / K[0, 0] < 0.05 and rms < 0.5)

    print(f'\n  --- {desc} ---')
    print(f'    {len(good)} Ansichten benutzt, {dropped} verworfen '
          f'(p_size < {used_thr:.2f})')
    print(f'    RMS {rms:.3f} px      {"OK" if rms < 0.5 else "ZU HOCH (< 0.5 erwartet)"}')
    print(f'    fx {K[0,0]:7.2f}  fy {K[1,1]:7.2f}   (erwartet ~{fx0:.0f}, HFOV {hfov:.1f} deg)')
    print(f'    cx {K[0,2]:7.2f}  cy {K[1,2]:7.2f}   (Bildmitte {w/2:.0f} / {h/2:.0f})')
    print(f'    k1 {D[0]:+.5f}  k2 {D[1]:+.5f}  k3 {D[4]:+.5f}')
    print(f'    p1 {D[2]:+.5f}  p2 {D[3]:+.5f}')
    print(f'    Abnahme: {"BESTANDEN" if ok else "durchgefallen"}')
    return dict(K=K, D=D, ok=ok, desc=desc, size=size, n=len(good), rms=rms)


def write_yaml(res):
    w, h = res['size']
    K, D = res['K'], res['D']
    with open(OUTFILE, 'w') as f:
        f.write(f"# scripts/tb/camera-intrinsics.py -- Modell: {res['desc']}\n")
        f.write(f"# {res['n']} Ansichten. Rohbilder: logs/intrinsik_aufnahmen.tar.gz\n")
        yaml.dump({
            'image_width': int(w), 'image_height': int(h), 'camera_name': 'camera',
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': [float(x) for x in K.ravel()]},
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {'rows': 1, 'cols': 5,
                                        'data': [float(x) for x in D[:5]]},
            'rectification_matrix': {'rows': 3, 'cols': 3,
                                     'data': [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
            'projection_matrix': {'rows': 3, 'cols': 4,
                                  'data': [float(K[0, 0]), 0., float(K[0, 2]), 0.,
                                           0., float(K[1, 1]), float(K[1, 2]), 0.,
                                           0., 0., 1., 0.]},
        }, f, default_flow_style=False, sort_keys=False)
    print(f'\nGeschrieben: {OUTFILE}')
    print('!! JETZT "colcon build" -- build/ haelt eine KOPIE der YAML, ingress_node')
    print('   laedt sonst weiter die alte Datei (siehe docs/MEMORY.md).')
    print('!! Danach die Bodenkalibrierung NEU machen (ground-calib.py) -- das entzerrte')
    print('   Bild hat sich geaendert, der alte Fit gilt nicht mehr.')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--capture', action='store_true', help='Ansichten aufnehmen')
    ap.add_argument('--calibrate', action='store_true', help='aus dem Tarball rechnen')
    ap.add_argument('--no-window', action='store_true',
                    help='kein eigenes Fenster (nur das Topic)')
    ap.add_argument('--max-speed', type=float, default=4.0,
                    help='max. Brettbewegung in px zwischen zwei Bildern; verwirft '
                         'verwackelte Aufnahmen (0 = Pruefung aus, Default 4)')
    ap.add_argument('--no-topic', action='store_true',
                    help=f'{PREVIEW_TOPIC} nicht publizieren')
    ap.add_argument('--write', metavar='MODELL', default=None,
                    help=f'Ergebnis speichern: {" | ".join(MODELS)}')
    args = ap.parse_args()

    if args.capture:
        return capture(window=not args.no_window, topic=not args.no_topic,
                       max_speed=args.max_speed if args.max_speed > 0 else -1.0)
    if not (args.calibrate or args.write):
        ap.error('--capture oder --calibrate angeben.')
    if not os.path.isfile(TARBALL):
        print(f'Keine Aufnahmen: {TARBALL}\nErst: --capture')
        return 1

    images = load_images()
    print(f'{len(images)} Aufnahmen aus {os.path.basename(TARBALL)}')
    res = {k: r for k in MODELS for r in [fit(images, k)] if r}

    if args.write:
        if args.write not in res:
            print(f'\nModell "{args.write}" nicht verfuegbar. Vorhanden: {list(res)}')
            return 1
        write_yaml(res[args.write])
    else:
        print('\nNichts geschrieben. Zum Uebernehmen z.B.:')
        print('  python3 scripts/tb/camera-intrinsics.py --calibrate --write fixed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
