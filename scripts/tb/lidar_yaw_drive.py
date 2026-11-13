#!/usr/bin/env python3
"""Verdrehung des LiDAR-Kopfs messen: geradeaus an einer Flasche vorbeifahren.

Aufruf (NUR bot-up, Pipeline aus -- das Skript faehrt selbst):
    source scripts/tb/tb-env.sh
    python3 scripts/tb/lidar_yaw_drive.py [Strecke in cm, Standard 40]

Aufbau: eine Flasche 15-40 cm seitlich neben/vor den Bot stellen (links ODER rechts), davor
mindestens Strecke + 15 cm frei. Wie der Bot zur Flasche steht, ist egal.

Prinzip: der Bot faehrt langsam geradeaus (nur linear.x, kein Lenken). Die Flasche steht still,
im LiDAR wandert sie also genau entgegen der Fahrtrichtung. Sitzt der Kopf gerade, laeuft ihre
Spur parallel zur x-Achse des Scans; ist er um d verdreht, laeuft sie um d schraeg. Die Mitte der
Flasche kommt aus einem Kreis durch ihre Scanpunkte (die sichtbare Seite wechselt beim Vorbei-
fahren, die Kreismitte nicht). Dreht sich der Bot doch etwas, misst das die IMU und wird
herausgerechnet; ueber MAX_TURN_DEG gilt die Fahrt als ungueltig.

Mehrmals messen, mit Flasche links und rechts: stimmen die Winkel ueberein, ist es der Kopf.
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

SPEED = 0.05                # m/s
FRONT_STOP_M = 0.15         # Hindernis voraus naeher -> sofort Halt
FRONT_HALF_DEG = 20.0
BOTTLE_SEARCH = (0.0, 0.45, 0.12, 0.45)   # Suchbereich: x von..bis, |y| von..bis (m, Scan)
TRACK_GATE_M = 0.06         # so weit darf die Flasche von Scan zu Scan springen
MIN_POINTS = 5
MAX_TURN_DEG = 3.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def points(scan):
    r = np.asarray(scan.ranges, dtype=np.float64)
    a = scan.angle_min + np.arange(r.size) * scan.angle_increment
    a = np.arctan2(np.sin(a), np.cos(a))
    ok = np.isfinite(r) & (r > scan.range_min)
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]), a[ok], r[ok]


def circle_center(p):
    """Kreis durch die Punkte (algebraisch, Kasa), Mitte (x, y) und Radius."""
    A = np.column_stack([p[:, 0], p[:, 1], np.ones(len(p))])
    b = -(p[:, 0] ** 2 + p[:, 1] ** 2)
    (D, E, F), *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = -D / 2.0, -E / 2.0
    return np.array([cx, cy]), math.sqrt(max(cx * cx + cy * cy - F, 0.0))


def find_bottle(p):
    x0, x1, y0, y1 = BOTTLE_SEARCH
    cand = p[(p[:, 0] >= x0) & (p[:, 0] <= x1) & (np.abs(p[:, 1]) >= y0) & (np.abs(p[:, 1]) <= y1)]
    if len(cand) < MIN_POINTS:
        return None
    # Naechster Punkt und alles im Umkreis von 6 cm darum = die Flasche.
    seed = cand[np.argmin(np.hypot(cand[:, 0], cand[:, 1]))]
    near = cand[np.hypot(*(cand - seed).T) < 0.06]
    return near if len(near) >= MIN_POINTS else None


def track_direction(centers, yaws):
    """Verdrehung (Grad) aus der Spur der Flaschenmitte im Scan, Drehung des Bots herausgerechnet.
    Positiv = LiDAR schaut nach links."""
    c = np.asarray(centers)
    dpsi = np.asarray(yaws) - yaws[0]
    # Jede Mitte in die Lage des Scanners beim Start zurueckdrehen.
    cs, sn = np.cos(dpsi), np.sin(dpsi)
    c0 = np.column_stack([cs * c[:, 0] - sn * c[:, 1], sn * c[:, 0] + cs * c[:, 1]])
    v = np.polyfit(np.arange(len(c0)), c0, 1)[0]        # Steigung je Scan (vx, vy)
    # Bei einer leichten Kurve zeigt die mittlere Fahrtrichtung um die halbe Drehung schraeg.
    yaw = math.atan2(v[1], -v[0]) + dpsi[-1] / 2.0
    return math.degrees(yaw), float(np.hypot(*(c0[-1] - c0[0])))


def main():
    distance = (float(sys.argv[1].replace(',', '.')) / 100.0) if len(sys.argv) > 1 else 0.40
    rclpy.init()
    node = Node('lidar_yaw_drive')
    scans, imu = [], []
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    node.create_subscription(Imu, '/imu', lambda m: imu.append(yaw_of(m.orientation)),
                             qos_profile_sensor_data)
    pub = node.create_publisher(Twist, '/cmd_vel', 1)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def stop():
        for _ in range(10):
            pub.publish(Twist())
            spin(0.05)

    spin(2.0)
    others = node.count_publishers('/cmd_vel') - 1
    if others > 0:
        print(f'Abbruch: {others} anderer Sender auf /cmd_vel (Pipeline/notaus?) -- bitte nur bot-up.')
        sys.exit(1)
    if not scans or not imu:
        print('Abbruch: kein /scan oder /imu -- laeuft bot-up, stimmt ROS_DOMAIN_ID?')
        sys.exit(1)
    p, _, _ = points(scans[-1])
    bottle = find_bottle(p)
    if bottle is None:
        print('Abbruch: keine Flasche 15-45 cm seitlich gefunden.')
        sys.exit(1)
    center, radius = circle_center(bottle)
    print(f'Flasche gefunden: Mitte x={center[0]:.3f} y={center[1]:.3f} m, Radius {radius * 100:.1f} cm '
          f'({"links" if center[1] > 0 else "rechts"}). Fahre {distance * 100:.0f} cm geradeaus ...')

    centers, yaws = [], []
    last = len(scans)
    twist = Twist()
    twist.linear.x = SPEED
    t_end = time.time() + distance / SPEED
    try:
        while time.time() < t_end:
            pub.publish(twist)
            spin(0.05)
            for s in scans[last:]:
                p, a, r = points(s)
                front = (np.abs(np.degrees(a)) < FRONT_HALF_DEG) & (r < FRONT_STOP_M)
                if front.any():
                    print('Halt: Hindernis voraus.')
                    t_end = 0.0
                    break
                near = p[np.hypot(*(p - center).T) < TRACK_GATE_M + 0.05]
                if len(near) >= MIN_POINTS:
                    c, _ = circle_center(near)
                    if np.hypot(*(c - center)) < TRACK_GATE_M:
                        center = c
                        centers.append(c)
                        yaws.append(imu[-1])
            last = len(scans)
    finally:
        stop()
    rclpy.shutdown()

    if len(centers) < 10:
        print(f'Zu wenig Messungen ({len(centers)}) -- Flasche verloren?')
        sys.exit(1)
    turn = math.degrees(math.remainder(yaws[-1] - yaws[0], 2 * math.pi))
    yaw_deg, travelled = track_direction(centers, np.unwrap(yaws))
    print(f'Messungen: {len(centers)}, Flasche im Scan {travelled * 100:.1f} cm gewandert, '
          f'Bot hat sich dabei um {turn:+.1f} Grad gedreht (IMU).')
    if abs(turn) > MAX_TURN_DEG:
        print(f'Ungueltig: mehr als {MAX_TURN_DEG:.0f} Grad gedreht -- bitte wiederholen.')
        sys.exit(1)
    side = 'links' if yaw_deg > 0 else 'rechts'
    print(f'==> LiDAR schaut {abs(yaw_deg):.1f} Grad nach {side} (verdreht gegenueber der Fahrtrichtung).')


if __name__ == '__main__':
    main()
