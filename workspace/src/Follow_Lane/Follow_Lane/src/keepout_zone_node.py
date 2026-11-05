"""Sperrt im Tunnel den Eingang, damit der Bot dort nicht wieder hinausfaehrt.

Ein:  /control/mode, /control/clear_maps (Taste L: Sperre neu an der aktuellen Pose), TF
Aus:  /detect/tunnel_entrance_obstacle (PointCloud2, odom)
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, Header, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src.common import (
    ControlType, control_mode_from_msg, pose2d_from_transform, run_node)

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'
PUBLISH_PERIOD_S = 0.2
# Heck 0.101 + footprint_padding 0.01 + 0.05 Reserve: erst dann liegt keine Wandzelle
# mehr unter dem Bot.
WALL_DELAY_M = 0.16
# Laenge der Sperre (m), mittig auf dem Bot quer zur Fahrtrichtung. Eingang max. 30 cm breit
# (Nutzer, 2026-11-05) + je 5 cm, weil der Bot nicht genau mittig im Eingang stehen muss; der
# Ueberstand liegt auf der Tunnelwand und stoert nicht.
WALL_WIDTH_M = 0.4
# Abstand der Wandpunkte (m): = kleinste resolution der Costmaps (global und local 0.01),
# sonst bleiben Luecken.
WALL_POINT_STEP_M = 0.01


class keepout_zone_node(Node):
    """Sendet eine virtuelle Wand an der Stelle, an der switch_control von TunnelEntry nach
    Tunnel wechselt (Spurlinien enden = Eingang): quer zur Fahrtrichtung durch die Radachse,
    WALL_WIDTH_M lang, im odom-Frame, damit sie ortsfest bleibt. Vorher kam sie von der
    Tuererkennung im Scan (/tunnel/door) und lag oft falsch (2026-11-05). Ohne TunnelEntry
    davor (Start im Tunnel) gibt es keine Wand.

    Gesendet wird sie erst, wenn der Bot WALL_DELAY_M hinter der Tuer ist. Sonst stuende er
    in ihr und MPPI saehe jede Bewegung als Kollision.
    """

    def __init__(self):
        super().__init__('keepout_zone_node')

        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_clear_maps = self.create_subscription(
            Empty, '/control/clear_maps', self.cb_clear_maps, 1)
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
        if new_mode == ControlType.Tunnel and self.control_mode == ControlType.TunnelEntry:
            self._door = self._door_at_robot('Wechsel TunnelEntry -> Tunnel')
        self.control_mode = new_mode

    def cb_clear_maps(self, _msg):
        """Taste L im Tunnel: Neustart vor dem Tunnel. Die alte Sperre verschwindet, die neue
        liegt an der Pose beim Druck auf L (Bot also erst vor den Eingang stellen) und wird wie
        beim Moduswechsel erst WALL_DELAY_M hinter ihr aktiv."""
        if self.control_mode != ControlType.Tunnel:
            return
        # Leere Wolke zuerst: entrance_layer behaelt sonst die alte Wand im Puffer und traegt sie
        # nach dem Leeren der Costmaps (lane_costmap_node, 0,3 s spaeter) wieder ein.
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = ODOM_FRAME
        self.pub_entrance.publish(point_cloud2.create_cloud_xyz32(header, []))
        self._wall_active = False
        self._door = self._door_at_robot('Taste L')

    def _door_at_robot(self, reason):
        """Eingang an der aktuellen Pose: Linie quer zur Fahrtrichtung durch die Radachse,
        Normale = Fahrtrichtung (nach innen). None ohne Pose."""
        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn('Keine Pose beim Wechsel in den Tunnel -- keine Eingangssperre.')
            return None
        x, y, yaw = pose
        center = np.array([x, y])
        normal = np.array([math.cos(yaw), math.sin(yaw)])
        side = np.array([-normal[1], normal[0]]) * WALL_WIDTH_M / 2.0
        self.get_logger().info(f'Tunnel-Eingang bei x={x:.2f} y={y:.2f} (odom, {reason}).')
        return (center - side, center + side, center, normal)

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

    def _robot_pose(self):
        """(x, y, yaw) in odom oder None."""
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF-Lookup fuer Roboterpose fehlgeschlagen: {e}', throttle_duration_sec=2.0)
            return None
        return pose2d_from_transform(tf)

    def _distance_past_door(self):
        """Wie weit die Radachse hinter der Tuerlinie steht (m); 0 ohne Pose."""
        pose = self._robot_pose()
        if pose is None:
            return 0.0
        _, _, center, normal = self._door
        return float((np.array(pose[:2]) - center) @ normal)

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
