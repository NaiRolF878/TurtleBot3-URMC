"""Sperrt im Tunnel den Eingang, damit der Bot dort nicht wieder hinausfaehrt.

Ein:  /control/mode, /tunnel/door (PolygonStamped: Tuerkanten, Punkt innen), TF
Aus:  /detect/tunnel_entrance_obstacle (PointCloud2, odom)
"""
import math

import numpy as np

import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src.common import ControlType, control_mode_from_msg, run_node

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'
PUBLISH_PERIOD_S = 0.2
# Heck 0.101 + footprint_padding 0.01 + 0.05 Reserve: erst dann liegt keine Wandzelle
# mehr unter dem Bot.
WALL_DELAY_M = 0.16
# Abstand der Wandpunkte (m): = resolution der global_costmap, sonst bleiben Luecken.
WALL_POINT_STEP_M = 0.025


class keepout_zone_node(Node):
    """Sendet eine virtuelle Wand zwischen die von tunnel_explorer_node gemessenen Tuerkanten,
    im odom-Frame, damit sie ortsfest bleibt. Ohne Tuer (Start im Tunnel) gibt es keine Wand.

    Gesendet wird sie erst, wenn der Bot WALL_DELAY_M hinter der Tuer ist. Sonst stuende er
    in ihr und MPPI saehe jede Bewegung als Kollision.
    """

    def __init__(self):
        super().__init__('keepout_zone_node')

        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_door = self.create_subscription(PolygonStamped, '/tunnel/door', self.cb_door, 1)
        self.pub_entrance = self.create_publisher(PointCloud2, '/detect/tunnel_entrance_obstacle', 1)
        self.timer = self.create_timer(PUBLISH_PERIOD_S, self.cb_timer)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = None
        self._door = None               # (kante a, kante b, mitte, normale nach innen)
        self._wall_active = False

        self.get_logger().info('Started keepout_zone_node Node.')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None:
            return
        if (self.control_mode == ControlType.Tunnel) != (new_mode == ControlType.Tunnel):
            self._door = None
            self._wall_active = False
        self.control_mode = new_mode

    def cb_door(self, msg):
        a, b, inside = (np.array([p.x, p.y], dtype=np.float64) for p in msg.polygon.points)
        center = (a + b) / 2.0
        normal = (inside - center) / max(np.hypot(*(inside - center)), 1e-9)
        self._door = (a, b, center, normal)
        self._wall_active = False
        self.get_logger().info(f'Tunnel-Eingang bei x={center[0]:.2f} y={center[1]:.2f} (odom).')

    def cb_timer(self):
        if self.control_mode != ControlType.Tunnel or self._door is None:
            return
        if not self._wall_active:
            if self._distance_past_door() < WALL_DELAY_M:
                return
            self._wall_active = True
            self.get_logger().info('Bot ist vom Eingang weg -- Eingangssperre aktiv.')
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = ODOM_FRAME
        self.pub_entrance.publish(point_cloud2.create_cloud_xyz32(header, self._wall_points()))

    # --- Hilfsmethoden ---

    def _distance_past_door(self):
        """Wie weit die Radachse hinter der Tuerlinie steht (m); 0 ohne Pose."""
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF-Lookup fuer Roboterpose fehlgeschlagen: {e}', throttle_duration_sec=2.0)
            return 0.0
        t = tf.transform.translation
        _, _, center, normal = self._door
        return float((np.array([t.x, t.y]) - center) @ normal)

    def _wall_points(self):
        a, b, _, _ = self._door
        count = max(2, int(math.ceil(np.hypot(*(b - a)) / WALL_POINT_STEP_M)) + 1)
        return [(float(p[0]), float(p[1]), 0.0) for p in np.linspace(a, b, count)]

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down keepout_zone_node Node.')


def main(args=None):
    run_node(keepout_zone_node, args)


if __name__ == '__main__':
    main()
