"""LiDAR-Scan korrigiert weiterreichen: gemessener Entfernungsfehler und Bewegung waehrend des Umlaufs.

Ein:  /scan, /odom_filtered, TF base_footprint <- Scanner
Aus:  /scan_corrected (LaserScan, Frame und Stempel wie /scan)

Der Scanner misst nah zu kurz (range_offset_m, gemessen). Dreht der Bot waehrend eines Umlaufs,
misst jeder Strahl aus einer anderen Pose: die Punkte werden auf die Pose beim ersten Strahl
(Stempel des Scans) bezogen. Ohne time_increment oder ohne Odometrie nur der Entfernungsfehler.
Alle Nodes und beide Costmaps lesen /scan_corrected.
"""
from collections import deque

import numpy as np

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src.common import (
    frange, pose2d_from_transform, read_params, require_params, run_node, wrap_angle,
    yaw_from_quaternion)
from Follow_Lane.src.robot_geometry import correct_scan

ROBOT_FRAME = 'base_footprint'
# Odometrie-Verlauf (Eintraege): reicht bei ~30 Hz fuer mehrere Umlaeufe.
ODOM_HISTORY = 100


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class scan_correct(Node):

    def __init__(self):
        super().__init__('scan_correct')

        # --- Parameter ---
        # Werte nur in Scan_Correct.yaml. So viel (m) misst der LiDAR zu kurz.
        self.declare_parameter('range_offset_m', Parameter.Type.DOUBLE, frange(-0.1, 0.1))
        self._param_names = ['range_offset_m']
        require_params(self, self._param_names)
        read_params(self, self._param_names)

        # --- Subscriber / Publisher ---
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(Odometry, '/odom_filtered', self.cb_odom, 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan_corrected', qos_profile_sensor_data)

        # --- Zustand ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._scanner = None            # Scanner in base_footprint (x, y, yaw)
        self._odom = deque(maxlen=ODOM_HISTORY)   # (Stempel s, x, y, yaw)

        self.get_logger().info('Started scan_correct Node.')

    # --- Callbacks ---

    def cb_odom(self, msg):
        p = msg.pose.pose
        self._odom.append((stamp_s(msg.header.stamp), p.position.x, p.position.y,
                           yaw_from_quaternion(p.orientation)))

    def cb_scan(self, scan):
        read_params(self, self._param_names)
        scanner = self._scanner_pose(scan.header.frame_id)
        # Ohne Lage des Scanners nur der Entfernungsfehler: lieber unkorrigiert als gar kein Scan.
        rel = self._relative_poses(scan) if scanner is not None else None
        ranges = correct_scan(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
                              self.range_offset_m, scanner or (0.0, 0.0, 0.0), rel)
        out = LaserScan()
        out.header = scan.header
        out.angle_min, out.angle_max = scan.angle_min, scan.angle_max
        out.angle_increment = scan.angle_increment
        # Alle Strahlen gelten jetzt zum Stempel des Scans.
        out.time_increment = 0.0 if rel is not None else scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min, out.range_max = scan.range_min, scan.range_max
        out.ranges = ranges.astype(np.float32).tolist()
        out.intensities = scan.intensities
        self.pub_scan.publish(out)

    # --- Hilfsmethoden ---

    def _relative_poses(self, scan):
        """(N, 3) Pose je Strahl relativ zur Pose beim ersten Strahl, None ohne Zeitangaben
        oder ohne Odometrie fuer den ganzen Umlauf."""
        n = len(scan.ranges)
        if scan.time_increment <= 0.0 or n < 2 or len(self._odom) < 2:
            return None
        t0 = stamp_s(scan.header.stamp)
        times = t0 + np.arange(n) * scan.time_increment
        hist = list(self._odom)
        stamps = [h[0] for h in hist]
        if times[0] < stamps[0] or times[-1] > stamps[-1] + 0.1:
            return None
        xs = np.interp(times, stamps, [h[1] for h in hist])
        ys = np.interp(times, stamps, [h[2] for h in hist])
        yaws = np.interp(times, stamps, np.unwrap([h[3] for h in hist]))
        dx, dy = xs - xs[0], ys - ys[0]
        c, s = np.cos(yaws[0]), np.sin(yaws[0])
        rel_yaw = np.array([wrap_angle(a - yaws[0]) for a in yaws])
        return np.column_stack([c * dx + s * dy, -s * dx + c * dy, rel_yaw])

    def _scanner_pose(self, frame):
        if self._scanner is None:
            try:
                tf = self.tf_buffer.lookup_transform(ROBOT_FRAME, frame, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f'Keine TF {ROBOT_FRAME} <- {frame}: {e}',
                                       throttle_duration_sec=5.0)
                return None
            self._scanner = pose2d_from_transform(tf)
        return self._scanner

    def shut_down(self):
        self.get_logger().info('Shutting down scan_correct Node.')


def main(args=None):
    run_node(scan_correct, args)


if __name__ == '__main__':
    main()
