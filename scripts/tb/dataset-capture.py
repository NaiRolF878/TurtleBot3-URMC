#!/usr/bin/env python3
"""Bilder fuer einen Vision-Datensatz aufnehmen: Livebild anzeigen, p speichert ein Foto.

Live am Bot (Kamera muss laufen, botup):
    python3 scripts/tb/dataset-capture.py
    python3 scripts/tb/dataset-capture.py --label schranke       # Unterordner/Praefix
    python3 scripts/tb/dataset-capture.py --undistorted          # entzerrt (ingress_node noetig)

Tasten (im Fenster oder, falls kein Fenster geht, im Terminal):
    p        Foto speichern
    q / Esc  beenden

Ueber den Launcher laeuft die Kamera dabei in hoher Aufloesung mit vollem Bildwinkel
(Cam 2: 1600x1200, Cam 3: 1920x1080, s. cam-mode.sh).

Standardmaessig wird das ROHE Kamerabild gespeichert, und zwar die JPEG-Bytes vom
Bot unveraendert (keine zweite Kompression, kein Qualitaetsverlust). Grund: die
Intrinsik ist noch nicht verlaesslich (s. docs/MEMORY.md) -- entzerren laesst sich
ein Rohbild spaeter immer noch, umgekehrt nicht.

Ablage: dataset/<label>/<label>_<datum>_<zeit>_<nr>.jpg  (dataset/ ist in .gitignore)
"""
import argparse
import os
import select
import sys
import termios
import time
import tty
from datetime import datetime

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATASET_DIR = os.path.join(REPO, 'dataset')

RAW_TOPIC = '/camera/image_raw/compressed'
UNDIST_TOPIC = '/lokal/raw_image'
WIN = 'Datensatz-Aufnahme  (p = Foto, q = beenden)'


def overlay(img, lines, flash=False):
    """Statuszeile oben links; nach dem Speichern kurz gruener Rahmen."""
    vis = img.copy()
    for i, t in enumerate(lines):
        y = 24 + 22 * i
        cv2.putText(vis, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1, cv2.LINE_AA)
    if flash:
        cv2.rectangle(vis, (0, 0), (vis.shape[1] - 1, vis.shape[0] - 1), (0, 255, 0), 8)
    return vis


def placeholder(topic, seen):
    ph = np.zeros((200, 640, 3), np.uint8)
    cv2.putText(ph, 'Warte auf Kamerabild ...', (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2, cv2.LINE_AA)
    cv2.putText(ph, topic, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(ph, 'Laeuft die Kamera auf dem Bot?  (botup)', (20, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA)
    cv2.putText(ph, f'{seen} Bilder empfangen', (20, 175),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
    return ph


class TerminalKeys:
    """Einzelne Tasten aus dem Terminal lesen -- Rueckfall, wenn kein Fenster geht."""

    def __init__(self):
        self.fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self.old = None

    def __enter__(self):
        if self.fd is not None:
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read(self):
        if self.fd is None:
            return None
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if r else None


def capture(args):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image

    label = args.label
    out_dir = os.path.join(args.out, label) if label else args.out
    os.makedirs(out_dir, exist_ok=True)
    prefix = label or 'img'
    n_existing = len([f for f in os.listdir(out_dir) if f.lower().endswith(('.jpg', '.png'))])

    br = CvBridge()
    # frame_no zaehlt empfangene Bilder; saved_frame verhindert, dass dasselbe Bild
    # bei gedrueckt gehaltenem p mehrfach gespeichert wird.
    st = {'img': None, 'jpeg': None, 'frame_no': 0, 'saved_frame': -1,
          'count': 0, 'flash_until': 0.0, 'warned': 0.0}

    rclpy.init()
    node = rclpy.create_node('dataset_capture')

    if args.undistorted:
        topic = UNDIST_TOPIC

        def cb(msg):
            st['img'] = br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            st['jpeg'] = None
            st['frame_no'] += 1

        node.create_subscription(Image, topic, cb, qos_profile_sensor_data)
    else:
        topic = RAW_TOPIC

        def cb(msg):
            data = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if img is None:
                return
            st['img'] = img
            st['jpeg'] = bytes(msg.data) if 'jpeg' in msg.format.lower() or 'jpg' in msg.format.lower() else None
            st['frame_no'] += 1

        # QoS an den Publisher anpassen: cam-mode.sh full sendet RELIABLE, lane BEST_EFFORT.
        # Ein reliable-Leser bekommt von einem best_effort-Sender gar nichts, ein
        # best_effort-Leser von einem reliable-Sender keine nachgesendeten Stuecke --
        # und ohne die kommt ein Vollbild nie komplett an.
        qos = qos_profile_sensor_data
        end = time.time() + 5.0
        while time.time() < end:
            infos = node.get_publishers_info_by_topic(topic)
            if infos:
                if infos[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE:
                    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                     reliability=ReliabilityPolicy.RELIABLE)
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        else:
            print(f'  Kein Publisher auf {topic} gefunden -- abonniere best_effort.')
        print(f'QoS: {"reliable" if qos is not qos_profile_sensor_data else "best_effort"}')
        node.create_subscription(CompressedImage, topic, cb, qos)

    def save():
        if st['img'] is None:
            print('  Noch kein Kamerabild -- nichts gespeichert.')
            return
        if st['frame_no'] == st['saved_frame']:
            return                                  # gleiches Bild schon gespeichert
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        nr = n_existing + st['count'] + 1
        if st['jpeg'] is not None:
            path = os.path.join(out_dir, f'{prefix}_{stamp}_{nr:04d}.jpg')
            with open(path, 'wb') as f:
                f.write(st['jpeg'])
        else:
            path = os.path.join(out_dir, f'{prefix}_{stamp}_{nr:04d}.png')
            cv2.imwrite(path, st['img'])
        st['saved_frame'] = st['frame_no']
        st['count'] += 1
        st['flash_until'] = time.time() + 0.25
        print(f'  [{st["count"]:4d}] {os.path.relpath(path, REPO)}')

    print(f'Aufnahme von {topic}')
    print(f'Speichere nach {os.path.relpath(out_dir, REPO)}/  ({n_existing} Bilder schon vorhanden)')
    print('p = Foto speichern, q / Esc = beenden.')
    print('Fenster geht gleich auf. Fehlt es, auf dem HOST einmal: xhost +local:root')
    print('(Ohne Fenster funktionieren p und q auch hier im Terminal.)\n')

    can_window = not args.no_window
    view, view_frame = None, -1
    try:
        with TerminalKeys() as term:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.03)
                key = None

                if can_window:
                    if st['img'] is None:
                        vis = placeholder(topic, st['frame_no'])
                    else:
                        # Die hohe Aufloesung ist fuer die meisten Bildschirme zu gross -- nur die ANZEIGE
                        # verkleinern (einmal pro neuem Bild), gespeichert wird das Original.
                        if view_frame != st['frame_no']:
                            view_frame = st['frame_no']
                            scale = min(1.0, args.view_width / st['img'].shape[1])
                            view = st['img'] if scale == 1.0 else cv2.resize(
                                st['img'], None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                        h, w = st['img'].shape[:2]
                        vis = overlay(view,
                                      [f'{prefix}: {st["count"]} neu / {n_existing + st["count"]} gesamt',
                                       f'{w}x{h}   p = Foto   q = Ende'],
                                      flash=time.time() < st['flash_until'])
                    try:
                        cv2.imshow(WIN, vis)
                        k = cv2.waitKey(1) & 0xFF
                        if k != 255:
                            key = chr(k) if k != 27 else 'esc'
                    except cv2.error as e:
                        can_window = False
                        print(f'\n  Fenster nicht moeglich ({str(e).splitlines()[-1][:60]}).')
                        print('  Auf dem HOST "xhost +local:root" ausfuehren -- oder einfach')
                        print('  hier im Terminal p / q druecken.\n')

                t = term.read()
                if t is not None:
                    key = 'esc' if t == '\x1b' else t

                if key in ('p', 'P'):
                    save()
                elif key in ('q', 'Q', 'esc'):
                    break

                if st['frame_no'] == 0 and time.time() - st['warned'] > 5.0:
                    st['warned'] = time.time()
                    print(f'  Noch kein Bild auf {topic}. Laeuft die Kamera auf dem Bot?')
                    print(f'  Pruefen: ros2 topic hz {topic}')
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f'\n{st["count"]} Bilder gespeichert in {os.path.relpath(out_dir, REPO)}/')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--label', default='',
                    help='Klasse/Szene, z.B. schranke -> dataset/schranke/schranke_....jpg')
    ap.add_argument('--out', default=DATASET_DIR, help=f'Zielordner (Standard: {DATASET_DIR})')
    ap.add_argument('--undistorted', action='store_true',
                    help=f'entzerrtes Bild von {UNDIST_TOPIC} speichern (PNG, ingress_node noetig)')
    ap.add_argument('--no-window', action='store_true',
                    help='kein Fenster, Tasten nur im Terminal')
    ap.add_argument('--view-width', type=int, default=1024,
                    help='maximale Breite der Anzeige in px (Speichern immer in Originalgroesse)')
    capture(ap.parse_args())


if __name__ == '__main__':
    main()
