"""Sperrt im Obstacle-Modus die Strasse hinter dem Startpunkt, damit NavFn nicht rueckwaerts
plant, und leert beim Verlassen von Obstacle und Tunnel beide Costmaps (Sperren raeumen sich
nicht selbst). Den Tunnel-Eingang sperrt tunnel_explorer_node (/nav2/blocked).

Ein:  /control/mode, /detect/lane_obstacles (Spurgedaechtnis: Breite der Startsperre), TF
Aus:  /detect/tunnel_entrance_obstacle (PointCloud2, odom; Startsperre, entrance_layer)
"""
import math
import time

import numpy as np

import rclpy
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src.common import (
    ControlType, cloud_xy, control_mode_from_msg, pose2d_from_transform, run_node)

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'
PUBLISH_PERIOD_S = 0.2
# Abstand der Wandpunkte (m): = kleinste resolution der Costmaps (global und local 0.01),
# sonst bleiben Luecken.
WALL_POINT_STEP_M = 0.01

# Startsperre im Obstacle-Modus (12.11.: NavFn plante wieder rueckwaerts durch den Startpunkt;
# max_path_backward in nav2_goal_publisher verwirft solche Plaene nur, verhindert sie nicht).
# Quer zur Fahrtrichtung beim Wechsel Lane -> Obstacle, so weit hinter der Radachse, dass sie
# sofort frei hinter dem Heck liegt (Heck 0.101 + Reserve). Je Seite reicht sie bis zur naechsten
# gemerkten Spurlinie (Punkte bis OBSTACLE_WALL_BAND_M neben der Wandlinie) plus
# OBSTACLE_WALL_OVERLAP_M; ohne Linie auf einer Seite (oder ohne frisches Spurgedaechtnis,
# LANE_MAX_AGE_S) bis OBSTACLE_WALL_SEARCH_M. Keine feste Spurbreite.
OBSTACLE_WALL_BACK_M = 0.20
OBSTACLE_WALL_BAND_M = 0.05
OBSTACLE_WALL_OVERLAP_M = 0.05
OBSTACLE_WALL_SEARCH_M = 1.0
LANE_MAX_AGE_S = 0.5
# Beim Verlassen von Obstacle und Tunnel: erst leere Wolke, nach so vielen s beide Costmaps leeren
# (sonst bleibt die Wand eingetragen; Linien und LiDAR kommen im naechsten Zyklus wieder).
COSTMAP_CLEAR_DELAY_S = 0.3


class keepout_zone_node(Node):
    """Obstacle: Startsperre beim Eintritt (siehe OBSTACLE_WALL_*), im odom-Frame ortsfest.
    Beim Verlassen von Obstacle eine leere Wolke, beim Verlassen von Obstacle und Tunnel
    COSTMAP_CLEAR_DELAY_S spaeter beide Costmaps leeren: die entrance_layer raeumen nicht selbst
    (clearing: false), Startsperre, Tunnel-Eingang und gesperrte Engstellen blieben sonst stehen.
    """

    def __init__(self):
        super().__init__('keepout_zone_node')

        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_lane = self.create_subscription(
            PointCloud2, '/detect/lane_obstacles', self.cb_lane_obstacles, 1)
        self.pub_entrance = self.create_publisher(PointCloud2, '/detect/tunnel_entrance_obstacle', 1)
        self.timer = self.create_timer(PUBLISH_PERIOD_S, self.cb_timer)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = None
        self._start_wall = None         # Obstacle: (kante a, kante b) in odom
        self._lane = None               # (monotonic, frame, (N, 2)) Spurgedaechtnis

        self.clear_clients = {
            'local_costmap': self.create_client(
                ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'),
            'global_costmap': self.create_client(
                ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'),
        }
        self.clear_timer = self.create_timer(COSTMAP_CLEAR_DELAY_S, self.cb_clear_timer)
        self.clear_timer.cancel()

        self.get_logger().info('Started keepout_zone_node Node.')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None:
            return
        if self.control_mode == ControlType.Tunnel and new_mode != ControlType.Tunnel:
            # Tunnel-Eingang und gesperrte Engstellen (/nav2/blocked, tunnel_explorer_node sendet
            # beim Verlassen eine leere Wolke) stehen sonst weiter in den Karten.
            self.clear_timer.reset()
        if new_mode == ControlType.Obstacle and self.control_mode != ControlType.Obstacle:
            self._start_wall = self._start_wall_at_robot()
        elif self.control_mode == ControlType.Obstacle and new_mode != ControlType.Obstacle:
            self._remove_start_wall()
        self.control_mode = new_mode

    def cb_clear_timer(self):
        self.clear_timer.cancel()
        for name, client in self.clear_clients.items():
            if not client.service_is_ready():
                self.get_logger().error(f'{name}-Clear-Service nicht erreichbar -- Sperren '
                                        'bleiben in der Karte.')
                continue
            client.call_async(ClearEntireCostmap.Request())

    def cb_lane_obstacles(self, cloud):
        try:
            self._lane = (time.monotonic(), cloud.header.frame_id, cloud_xy(cloud))
        except ValueError:
            self._lane = None

    def _start_wall_at_robot(self):
        """Startsperre: Linie quer zur Fahrtrichtung, OBSTACLE_WALL_BACK_M hinter der Radachse,
        links und rechts bis zur Spurlinie (_wall_reach)."""
        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn('Keine Pose beim Wechsel in Obstacle -- keine Startsperre.')
            return None
        x, y, yaw = pose
        normal = np.array([math.cos(yaw), math.sin(yaw)])
        center = np.array([x, y]) - normal * OBSTACLE_WALL_BACK_M
        left = np.array([-normal[1], normal[0]])
        reach_left, reach_right = self._wall_reach()
        self.get_logger().info(f'Obstacle: Startsperre {OBSTACLE_WALL_BACK_M * 100:.0f} cm hinter '
                               f'dem Bot bei x={center[0]:.2f} y={center[1]:.2f} (odom), '
                               f'{reach_left:.2f} m links, {reach_right:.2f} m rechts.')
        return (center - left * reach_right, center + left * reach_left)

    def _wall_reach(self):
        """(links, rechts) in m von der Wandmitte bis zur naechsten Spurlinie an der Wandlinie
        plus OBSTACLE_WALL_OVERLAP_M; ohne Linie OBSTACLE_WALL_SEARCH_M. Spurgedaechtnis im
        Bot-Frame zum Bildzeitpunkt: die Fahrt seit dem Bild (ein Bild, wenige cm) verschiebt
        die Punkte vor allem laengs der Wand-Normalen, nicht quer."""
        reach = [OBSTACLE_WALL_SEARCH_M, OBSTACLE_WALL_SEARCH_M]
        lane = self._lane
        if lane is None or lane[1] != ROBOT_FRAME \
                or time.monotonic() - lane[0] > LANE_MAX_AGE_S:
            return tuple(reach)
        pts = lane[2]
        near = pts[np.abs(pts[:, 0] + OBSTACLE_WALL_BACK_M) <= OBSTACLE_WALL_BAND_M]
        for i, side in enumerate((near[:, 1], -near[:, 1])):
            hits = side[side > 0.0]
            if hits.size:
                reach[i] = min(OBSTACLE_WALL_SEARCH_M, float(hits.min()) + OBSTACLE_WALL_OVERLAP_M)
        return tuple(reach)

    def _remove_start_wall(self):
        if self._start_wall is None:
            return
        self._start_wall = None
        self._publish_empty()
        self.clear_timer.reset()
        self.get_logger().info('Obstacle beendet -- Startsperre entfernt.')

    def cb_timer(self):
        if self.control_mode == ControlType.Obstacle and self._start_wall is not None:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = ODOM_FRAME
            self.pub_entrance.publish(point_cloud2.create_cloud_xyz32(
                header, self._line_points(*self._start_wall)))

    # --- Hilfsmethoden ---

    def _publish_empty(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = ODOM_FRAME
        self.pub_entrance.publish(point_cloud2.create_cloud_xyz32(header, []))

    def _robot_pose(self):
        """(x, y, yaw) in odom oder None."""
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF-Lookup fuer Roboterpose fehlgeschlagen: {e}', throttle_duration_sec=2.0)
            return None
        return pose2d_from_transform(tf)

    @staticmethod
    def _line_points(a, b):
        count = max(2, int(math.ceil(np.hypot(*(b - a)) / WALL_POINT_STEP_M)) + 1)
        return [(float(p[0]), float(p[1]), 0.0) for p in np.linspace(a, b, count)]

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down keepout_zone_node Node.')


def main(args=None):
    run_node(keepout_zone_node, args)


if __name__ == '__main__':
    main()
