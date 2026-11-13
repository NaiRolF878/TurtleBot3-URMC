"""Sperrt im Tunnel den Eingang, damit der Bot dort nicht wieder hinausfaehrt, und im
Obstacle-Modus die Strasse hinter dem Startpunkt, damit NavFn nicht rueckwaerts plant.

Ein:  /control/mode, /control/clear_maps (Taste L: Sperre neu an der aktuellen Pose), TF
Aus:  /detect/tunnel_entrance_obstacle (PointCloud2, odom; beide Sperren, entrance_layer)
"""
import math

import numpy as np

import rclpy
from nav2_msgs.srv import ClearEntireCostmap
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
# Laenge der Sperre (m), mittig quer zur Fahrtrichtung: Eingang max. 30 cm + je 5 cm, weil der
# Bot nicht genau mittig steht. Der Ueberstand liegt auf der Tunnelwand.
WALL_WIDTH_M = 0.4
# Abstand der Wandpunkte (m): = kleinste resolution der Costmaps (global und local 0.01),
# sonst bleiben Luecken.
WALL_POINT_STEP_M = 0.01

# Startsperre im Obstacle-Modus (12.11.: NavFn plante wieder rueckwaerts durch den Startpunkt;
# max_path_backward in nav2_goal_publisher verwirft solche Plaene nur, verhindert sie nicht).
# Quer zur Fahrtrichtung beim Wechsel Lane -> Obstacle, so weit hinter der Radachse, dass sie
# sofort frei hinter dem Heck liegt (>= WALL_DELAY_M), breit genug fuer beide Spuren samt Linien.
OBSTACLE_WALL_BACK_M = 0.20
OBSTACLE_WALL_WIDTH_M = 0.6
# Beim Verlassen von Obstacle: erst leere Wolke, nach so vielen s beide Costmaps leeren (sonst
# bleibt die Wand eingetragen; Linien und LiDAR kommen im naechsten Zyklus wieder).
COSTMAP_CLEAR_DELAY_S = 0.3


class keepout_zone_node(Node):
    """Sendet eine virtuelle Wand an der Stelle, an der switch_control von TunnelEntry nach
    Tunnel wechselt (Spurlinien enden = Eingang): quer zur Fahrtrichtung durch die Radachse,
    WALL_WIDTH_M lang, im odom-Frame, damit sie ortsfest bleibt. Ohne TunnelEntry davor
    (Start im Tunnel) gibt es keine Wand.

    Gesendet wird sie erst, wenn der Bot WALL_DELAY_M hinter der Tuer ist. Sonst stuende er
    in ihr und Regler und Waechter saehen jede Bewegung als Kollision. Beim Verlassen des Tunnels
    geht eine leere Wolke hinaus, sonst traegt die entrance_layer die Sperre weiter ein.
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
        self._start_wall = None         # Obstacle: (kante a, kante b) in odom

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
        if (self.control_mode == ControlType.Tunnel) != (new_mode == ControlType.Tunnel):
            if self._wall_active:
                self._publish_empty()
            self._door = None
            self._wall_active = False
        if new_mode == ControlType.Tunnel and self.control_mode == ControlType.TunnelEntry:
            self._door = self._door_at_robot('Wechsel TunnelEntry -> Tunnel')
        if new_mode == ControlType.Obstacle and self.control_mode != ControlType.Obstacle:
            self._start_wall = self._start_wall_at_robot()
        elif self.control_mode == ControlType.Obstacle and new_mode != ControlType.Obstacle:
            self._remove_start_wall()
        self.control_mode = new_mode

    def cb_clear_maps(self, _msg):
        """Taste L im Tunnel: Neustart vor dem Tunnel. Die alte Sperre verschwindet, die neue
        liegt an der Pose beim Druck auf L (Bot also erst vor den Eingang stellen) und wird wie
        beim Moduswechsel erst WALL_DELAY_M hinter ihr aktiv."""
        if self.control_mode != ControlType.Tunnel:
            return
        # Leere Wolke zuerst: entrance_layer behaelt sonst die alte Wand im Puffer und traegt sie
        # nach dem Leeren der Costmaps (lane_costmap_node, 0,3 s spaeter) wieder ein.
        self._publish_empty()
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

    def cb_clear_timer(self):
        self.clear_timer.cancel()
        for name, client in self.clear_clients.items():
            if not client.service_is_ready():
                self.get_logger().error(f'{name}-Clear-Service nicht erreichbar -- Startsperre '
                                        'bleibt in der Karte.')
                continue
            client.call_async(ClearEntireCostmap.Request())

    def _start_wall_at_robot(self):
        """Startsperre: Linie quer zur Fahrtrichtung, OBSTACLE_WALL_BACK_M hinter der Radachse."""
        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn('Keine Pose beim Wechsel in Obstacle -- keine Startsperre.')
            return None
        x, y, yaw = pose
        normal = np.array([math.cos(yaw), math.sin(yaw)])
        center = np.array([x, y]) - normal * OBSTACLE_WALL_BACK_M
        side = np.array([-normal[1], normal[0]]) * OBSTACLE_WALL_WIDTH_M / 2.0
        self.get_logger().info(f'Obstacle: Startsperre {OBSTACLE_WALL_BACK_M * 100:.0f} cm hinter '
                               f'dem Bot bei x={center[0]:.2f} y={center[1]:.2f} (odom).')
        return (center - side, center + side)

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
            return
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

    def _distance_past_door(self):
        """Wie weit die Radachse hinter der Tuerlinie steht (m); 0 ohne Pose."""
        pose = self._robot_pose()
        if pose is None:
            return 0.0
        _, _, center, normal = self._door
        return float((np.array(pose[:2]) - center) @ normal)

    def _wall_points(self):
        a, b, _, _ = self._door
        return self._line_points(a, b)

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
