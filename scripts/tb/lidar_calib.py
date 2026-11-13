#!/usr/bin/env python3
"""LiDAR-Kalibrierung an einer geraden Wand: Verdrehung (Grad) und Entfernungsfehler (m).

Aufruf (Pipeline oder bot-up muss laufen, ROS-Umgebung wie bei tb):
    python3 scripts/tb/lidar_calib.py <Abstand Bot-Vorderkante -> Wand in cm>

Aufbau: Bot gerade vor eine gerade, glatte Wand stellen (z.B. Tunnelwand), 20-40 cm davor,
Vorderkante parallel zur Wand (Geodreieck an Wand und Bot-Vorderkante). Abstand Vorderkante ->
Wand mit dem Lineal messen. Den Bot waehrend der Messung nicht beruehren.

Gemessen wird /scan (roh, ohne scan_correct): die Strahlen +/- SECTOR_DEG um die Fahrtrichtung,
eine Gerade durch die Wandpunkte. Steht die Wand im Scan schraeg, ist der LiDAR um diesen Winkel
verdreht (Annahme: der Bot steht rechtwinklig). Der senkrechte Abstand Scanner -> Wand verglichen
mit dem Linealwert (+ 6,9 cm Scanner -> Vorderkante, laut URDF/FOOTPRINT) ergibt den
Entfernungsfehler. Ergebnis zum Eintragen in Follow_Lane/Parameter/Scan_Correct.yaml.
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SCANNER_X = -0.032          # base_scan in base_footprint (TF)
FRONT_X = 0.037             # Vorderkante in base_footprint (robot_geometry.FOOTPRINT)
SECTOR_DEG = 30.0
SCANS = 30
INLIER_M = 0.01


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    front_to_wall = float(sys.argv[1].replace(',', '.')) / 100.0
    truth = front_to_wall + (FRONT_X - SCANNER_X)

    rclpy.init()
    node = Node('lidar_calib')
    scans = []
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    start = time.time()
    while len(scans) < SCANS and time.time() - start < 20.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if len(scans) < 5:
        print('Zu wenige Scans auf /scan -- laeuft der Bot (bot-up) und stimmt ROS_DOMAIN_ID?')
        sys.exit(1)

    points = []
    for s in scans:
        r = np.asarray(s.ranges, dtype=np.float64)
        a = s.angle_min + np.arange(r.size) * s.angle_increment
        a = np.arctan2(np.sin(a), np.cos(a))
        ok = np.isfinite(r) & (r > s.range_min) & (np.abs(a) <= math.radians(SECTOR_DEG))
        points.append(np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]))
    pts = np.vstack(points)
    if len(pts) < 50:
        print('Zu wenige Wandpunkte vor dem Bot -- steht die Wand 20-40 cm davor?')
        sys.exit(1)

    # Gerade x = c + m*y (Wand quer vor dem Bot), robust: zweimal Ausreisser verwerfen.
    keep = np.ones(len(pts), bool)
    for _ in range(3):
        m, c = np.polyfit(pts[keep, 1], pts[keep, 0], 1)
        resid = np.abs(pts[:, 0] - (c + m * pts[:, 1])) / math.hypot(1.0, m)
        keep = resid < max(INLIER_M, 3 * np.median(resid[keep]))
    yaw_deg = math.degrees(math.atan(m))          # Wand schraeg im Scan = LiDAR verdreht
    dist = abs(c) / math.hypot(1.0, m)            # senkrechter Abstand Scanner -> Wand
    rms = float(np.sqrt(np.mean(resid[keep] ** 2)))

    print(f'Scans: {len(scans)}, Wandpunkte: {keep.sum()} von {len(pts)}, Streuung {rms * 1000:.1f} mm')
    print(f'Wand im Scan: {dist * 100:.1f} cm vom Scanner, Lineal + 6,9 cm: {truth * 100:.1f} cm')
    print(f'  -> Entfernungsfehler: Scan misst {(truth - dist) * 100:+.1f} cm zu kurz '
          f'(range_offset_m: {truth - dist:.3f})')
    side = 'links' if yaw_deg > 0 else 'rechts'
    print(f'  -> Verdrehung: LiDAR schaut {abs(yaw_deg):.1f} Grad nach {side} '
          f'(Wand steht im Scan um diesen Winkel schraeg; gilt, wenn der Bot rechtwinklig steht)')
    print('Zur Kontrolle den Bot um 180 Grad umdrehen ist nicht moeglich -- daher lieber zweimal '
          'messen (Bot neu ausrichten) und vergleichen.')


if __name__ == '__main__':
    main()
