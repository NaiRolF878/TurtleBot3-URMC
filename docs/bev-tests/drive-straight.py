#!/usr/bin/env python3
"""Faehrt den Bot eine kurze Strecke GERADE (angular.z = 0) und stoppt ueber die Odometrie.

Aufruf:  python3 docs/bev-tests/drive-straight.py [--dist 0.15] [--speed 0.03] [--forward]
Default: 15 cm RUECKWAERTS mit 3 cm/s. Abbruch: Strecke erreicht, Zeitlimit, Strg+C oder
fehlende Odometrie -- in jedem Fall wird mehrfach Stillstand gesendet.
Gibt gefahrene Strecke und Drehung laut Odometrie aus (Drehung muss ~0 sein).
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=float, default=0.15, help='Strecke in m (max 0.4)')
    ap.add_argument('--speed', type=float, default=0.03, help='Geschwindigkeit in m/s (max 0.08)')
    ap.add_argument('--forward', action='store_true', help='vorwaerts statt rueckwaerts')
    args = ap.parse_args()
    dist = min(abs(args.dist), 0.4)
    speed = min(abs(args.speed), 0.08) * (1 if args.forward else -1)

    rclpy.init()
    node = rclpy.create_node('drive_straight_test')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    st = {}
    node.create_subscription(Odometry, '/odom', lambda m: st.__setitem__('odom', m), 10)

    def stop():
        for _ in range(10):
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)

    t_wait = time.time() + 5
    while 'odom' not in st and time.time() < t_wait:
        rclpy.spin_once(node, timeout_sec=0.1)
    if 'odom' not in st:
        print('Keine /odom -- laeuft turtlebot3_node? Nichts gefahren.')
        node.destroy_node(); rclpy.shutdown(); return 1

    p0 = st['odom'].pose.pose
    x0, y0, a0 = p0.position.x, p0.position.y, yaw_of(p0.orientation)
    limit = time.time() + dist / abs(speed) * 1.5 + 2
    cmd = Twist(); cmd.linear.x = speed
    moved, ok = 0.0, False
    try:
        while time.time() < limit:
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)
            p = st['odom'].pose.pose
            moved = math.hypot(p.position.x - x0, p.position.y - y0)
            if moved >= dist:
                ok = True
                break
    finally:
        stop()
    t_end = time.time() + 1.0                        # auslaufen lassen, Endlage lesen
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    p = st['odom'].pose.pose
    moved = math.hypot(p.position.x - x0, p.position.y - y0)
    turn = math.degrees(yaw_of(p.orientation) - a0)
    print(f'{"Fertig" if ok else "ZEITLIMIT"}: {moved*100:.1f} cm gefahren '
          f'({"vor" if speed > 0 else "zurueck"}), Drehung laut Odometrie {turn:+.2f} Grad')
    node.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
