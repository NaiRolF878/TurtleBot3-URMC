#!/usr/bin/env python3
"""Festen Weissabgleich (ColourGains) fuer die Kamera messen.

Hintergrund: Der automatische Weissabgleich regelt bei wechselndem Licht nach, dadurch
springen die Farbtoene -- und die HSV-Schwellen der Spurerkennung passen nicht mehr.
Mit festem Weissabgleich bleibt dieselbe Farbe dieselbe Farbe.

Ablauf (Bot laeuft mit botup, Licht so wie beim Fahren):
    1. Ein WEISSES oder GRAUES Blatt (matt, kein Glanz) mittig vor die Kamera legen,
       es muss das gruene Kaestchen im Bild ganz ausfuellen.
    2. python3 scripts/tb/awb-calib.py            # misst, stellt live um, zeigt Ergebnis
       python3 scripts/tb/awb-calib.py --write    # ... und traegt es in camera.conf ein
    3. Ab dem naechsten botup laeuft die Kamera mit diesen Werten.

Funktionsweise: schaltet am laufenden camera_node den Weissabgleich ab (AwbEnable=false)
und regelt ColourGains (rot, blau) iterativ, bis das Blatt im Bild neutral grau ist
(R = G = B). Iterativ, weil die ISP nach den Gains noch Farbmatrix und Gamma anwendet --
eine einzelne Rechnung trifft deshalb nicht genau.

Die Werte gelten fuer genau dieses Licht. In einer anderen Halle (Wettbewerb!) neu messen.
Zurueck zum automatischen Weissabgleich: CAM_COLOUR_GAINS="" in camera.conf, dann botup.
"""
import argparse
import os
import re
import sys
import time

import cv2
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CAMERA_CONF = os.path.join(SCRIPT_DIR, 'camera.conf')
TOPIC = '/camera/image_raw/compressed'
NODE = '/camera'


def write_conf(r, b):
    with open(CAMERA_CONF) as f:
        text = f.read()
    new, n = re.subn(r'^CAM_COLOUR_GAINS=.*$', f'CAM_COLOUR_GAINS="{r:.3f},{b:.3f}"',
                     text, flags=re.M)
    if n != 1:
        print(f'CAM_COLOUR_GAINS nicht in {CAMERA_CONF} gefunden -- bitte von Hand eintragen.')
        return
    with open(CAMERA_CONF, 'w') as f:
        f.write(new)
    print(f'Eingetragen in {os.path.relpath(CAMERA_CONF)}. Wirkt ab dem naechsten botup.')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--write', action='store_true', help='Ergebnis in camera.conf eintragen')
    ap.add_argument('--roi', type=float, default=0.25,
                    help='Kantenlaenge des Messfelds relativ zur Bildbreite (Standard 0.25)')
    ap.add_argument('--start', default='1.6,1.6', help='Start-Gains "rot,blau"')
    ap.add_argument('--tol', type=float, default=0.015, help='erlaubte Abweichung von R/G und B/G')
    ap.add_argument('--max-iter', type=int, default=12)
    args = ap.parse_args()

    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    rclpy.init()
    node = rclpy.create_node('awb_calib')
    st = {'img': None, 'n': 0}

    def cb(msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            st['img'], st['n'] = img, st['n'] + 1

    # QoS an den Publisher anpassen (lane = best_effort, cam-mode full = reliable).
    qos = qos_profile_sensor_data
    infos = []
    end = time.time() + 5.0
    while time.time() < end and not infos:
        infos = node.get_publishers_info_by_topic(TOPIC)
        rclpy.spin_once(node, timeout_sec=0.2)
    if not infos:
        print(f'Kein Publisher auf {TOPIC}. Laeuft der Bot (botup)? Stimmt ROS_DOMAIN_ID?')
        return 1
    if infos[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE:
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(CompressedImage, TOPIC, cb, qos)

    cli = node.create_client(SetParameters, f'{NODE}/set_parameters')
    if not cli.wait_for_service(timeout_sec=5.0):
        print(f'Parameterdienst {NODE}/set_parameters nicht erreichbar.')
        return 1

    def set_params(**kv):
        req = SetParameters.Request()
        for name, val in kv.items():
            if isinstance(val, bool):
                pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=val)
            else:
                pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                    double_array_value=[float(v) for v in val])
            req.parameters.append(Parameter(name=name, value=pv))
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        res = fut.result()
        bad = [r.reason for r in (res.results if res else []) if not r.successful]
        if res is None or bad:
            raise RuntimeError(f'Parameter abgelehnt: {bad or "keine Antwort"}')

    def fresh_frame(skip=6):
        """Ein paar Bilder abwarten, damit die neuen Gains sicher wirken."""
        n0 = st['n']
        end = time.time() + 10.0
        while st['n'] < n0 + skip and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        return st['img'] if st['n'] >= n0 + skip else None

    def measure(img):
        h, w = img.shape[:2]
        s = int(w * args.roi)
        x0, y0 = (w - s) // 2, (h - s) // 2
        roi = img[y0:y0 + s, x0:x0 + s].reshape(-1, 3).astype(np.float64)
        ok = (roi.max(axis=1) < 245) & (roi.min(axis=1) > 15)  # Ueber-/Unterbelichtung raus
        frac = ok.mean()
        b, g, r = roi[ok].mean(axis=0) if ok.any() else (0, 0, 0)
        vis = img.copy()
        cv2.rectangle(vis, (x0, y0), (x0 + s, y0 + s), (0, 255, 0), 2)
        return r, g, b, frac, vis

    def show(vis, lines):
        for i, t in enumerate(lines):
            cv2.putText(vis, t, (10, 25 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(vis, t, (10, 25 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        try:
            cv2.imshow('Weissabgleich', vis)
            cv2.waitKey(1)
        except cv2.error:
            pass

    try:
        gr, gb = (float(v) for v in args.start.split(','))
        print('Schalte automatischen Weissabgleich ab ...')
        set_params(AwbEnable=False, ColourGains=[gr, gb])

        for it in range(1, args.max_iter + 1):
            img = fresh_frame()
            if img is None:
                print('Keine Bilder mehr -- Abbruch.')
                return 1
            r, g, b, frac, vis = measure(img)
            if frac < 0.5:
                print(f'  Messfeld zu {100 * (1 - frac):.0f} % ueber-/unterbelichtet. Blatt richtig '
                      'platziert? Glanz/Spiegelung vermeiden.')
                show(vis, ['Messfeld ueber-/unterbelichtet!'])
                continue
            rg, bg = r / g, b / g
            print(f'  [{it:2d}] Gains rot {gr:.3f} blau {gb:.3f}  ->  R/G {rg:.3f}  B/G {bg:.3f}')
            show(vis, [f'Gains rot {gr:.3f} blau {gb:.3f}', f'R/G {rg:.3f}  B/G {bg:.3f}'])
            if abs(rg - 1) < args.tol and abs(bg - 1) < args.tol:
                break
            # Multiplikativ nachregeln, gedaempft gegen Ueberschwingen durch Gamma/Farbmatrix.
            gr = float(np.clip(gr * (1 / rg) ** 0.8, 0.3, 8.0))
            gb = float(np.clip(gb * (1 / bg) ** 0.8, 0.3, 8.0))
            set_params(ColourGains=[gr, gb])
        else:
            print('Nicht ganz konvergiert -- Werte sind trotzdem brauchbar, Licht/Blatt pruefen.')

        print(f'\nErgebnis:  CAM_COLOUR_GAINS="{gr:.3f},{gb:.3f}"')
        print('Die Kamera laeuft jetzt schon mit diesen Werten (bis zum naechsten Neustart).')
        if args.write:
            write_conf(gr, gb)
        else:
            print('Dauerhaft: mit --write erneut ausfuehren oder in scripts/tb/camera.conf eintragen.')
        time.sleep(1.5)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f'\n{e}')
        print('Unterstuetzt eure camera_ros-Version AwbEnable/ColourGains zur Laufzeit?')
        print('Pruefen: ros2 param list /camera | grep -i -E "awb|colour"')
        return 1
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
