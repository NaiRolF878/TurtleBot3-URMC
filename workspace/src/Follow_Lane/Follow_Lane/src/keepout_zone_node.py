"""Sperrt im Tunnel den Eingang, damit der Bot dort nicht wieder hinausfaehrt.

Ein:  /control/mode, TF odom <- base_footprint
Aus:  /detect/tunnel_entrance_obstacle (PointCloud2, odom), /tunnel/entrance_pose
      (PoseStamped); leert beim Tunnel-Eintritt die global_costmap.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src.common import (
    ControlType, control_mode_from_msg, frange, irange, read_params, run_node,
    yaw_from_quaternion)


class keepout_zone_node(Node):
    """Merkt sich beim Wechsel in den Tunnel einmalig die Roboterpose, um
    entrance_wall_offset in Fahrtrichtung vorgeschoben, und sendet dort laufend eine
    kurze virtuelle Wand quer zur Einfahrtrichtung. Im odom-Frame, damit sie ortsfest
    bleibt und wieder in die mitfahrende Costmap rutscht, sobald sich der Bot dem
    Eingang naehert.

    Der Versatz, weil switch_control schon kurz vor der Oeffnung auf Tunnel schaltet
    (Kamera rechtzeitig aus); die Wand gehoert aber an die Oeffnung, sonst bliebe
    zwischen ihr und dem Tunnel ein Streifen, durch den der Bot seitlich hinaus koennte.

    Gesendet wird die Wand erst, wenn der Bot entrance_wall_delay weit in
    Einfahrtrichtung von ihr weg ist. Sonst stuende er im ersten Moment selbst in ihr
    und MPPI saehe jede Bewegung als Kollision. Die Wand nach hinten zu verschieben
    waere keine Loesung: dann entstuende eine Luecke, durch die er seitlich hinaus
    koennte.
    """

    def __init__(self):
        super().__init__('keepout_zone_node')

        # --- Parameter ---
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('output_topic', '/detect/tunnel_entrance_obstacle')
        self.declare_parameter('entrance_wall_width', 0.6, frange(0.1, 3.0))
        self.declare_parameter('entrance_wall_points', 9, irange(2, 50))
        self.declare_parameter('publish_rate', 5.0, frange(0.5, 20.0))
        # Heck des Footprints 0.101 m hinter der Radachse + footprint_padding 0.01
        # + 0.05 Reserve (mindestens eine Zelle): erst dann liegt keine Wandzelle
        # mehr unter dem Bot.
        self.declare_parameter('entrance_wall_delay', 0.16, frange(0.0, 2.0))
        # So weit (m) steht die Radachse beim Umschalten etwa vor der Oeffnung; folgt aus
        # switch_control.tunnel_side_distance und der Tunnelbreite.
        self.declare_parameter('entrance_wall_offset', 0.2, frange(0.0, 2.0))
        self._param_names = ['odom_frame', 'robot_frame', 'entrance_wall_width',
                             'entrance_wall_points', 'entrance_wall_delay', 'entrance_wall_offset']

        # --- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)

        # --- Publisher ---
        self.pub_entrance = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 1)
        self.pub_entrance_pose = self.create_publisher(PoseStamped, '/tunnel/entrance_pose', 1)
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')

        # --- Timer ---
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value, self.cb_timer)

        # --- Zustand ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = None
        self.entrance_pose = None       # (x, y, yaw) im odom-Frame
        self._wall_active = False

        self.get_logger().info('Started keepout_zone_node Node.')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None:
            return
        was_tunnel = self.control_mode == ControlType.Tunnel
        is_tunnel = new_mode == ControlType.Tunnel
        self.control_mode = new_mode

        if was_tunnel != is_tunnel:
            self.entrance_pose = None
            self._wall_active = False
        if is_tunnel and not was_tunnel:
            self._clear_global_costmap()

    def cb_timer(self):
        if self.control_mode != ControlType.Tunnel:
            return
        read_params(self, self._param_names)

        if self.entrance_pose is None:
            robot = self._lookup_robot_pose()
            if robot is None:
                return
            rx, ry, yaw = robot
            self.entrance_pose = (rx + self.entrance_wall_offset * math.cos(yaw),
                                  ry + self.entrance_wall_offset * math.sin(yaw), yaw)
            x, y, _ = self.entrance_pose
            self.get_logger().info(f'Tunnel-Eingang gemerkt bei x={x:.2f} y={y:.2f} (odom).')

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.odom_frame

        # tunnel_explorer_node braucht die Eingangspose sofort, nicht erst mit der Wand.
        self._publish_entrance_pose(header)

        if not self._wall_active:
            if self._distance_past_entrance() < self.entrance_wall_delay:
                return
            self._wall_active = True
            self.get_logger().info('Bot ist vom Eingang weg -- Eingangssperre aktiv.')

        self.pub_entrance.publish(point_cloud2.create_cloud_xyz32(header, self._wall_points()))

    # --- Hilfsmethoden ---

    def _clear_global_costmap(self):
        if not self.clear_global_costmap.service_is_ready():
            self.get_logger().warn(
                'global_costmap-Clear-Service nicht erreichbar -- laeuft planner_server?',
                throttle_duration_sec=5.0)
            return
        self.clear_global_costmap.call_async(ClearEntireCostmap.Request())

    def _distance_past_entrance(self):
        """Wie weit der Bot in Einfahrtrichtung vor der Eintrittslinie steht (m)."""
        robot = self._lookup_robot_pose()
        if robot is None:
            return 0.0
        x, y, yaw = self.entrance_pose
        return (robot[0] - x) * math.cos(yaw) + (robot[1] - y) * math.sin(yaw)

    def _publish_entrance_pose(self, header):
        x, y, yaw = self.entrance_pose
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_entrance_pose.publish(pose)

    def _lookup_robot_pose(self):
        """Aktuelle Pose (x, y, yaw) im odom_frame, oder None ohne TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF-Lookup fuer Roboterpose fehlgeschlagen: {e}', throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        return (t.x, t.y, yaw_from_quaternion(tf.transform.rotation))

    def _wall_points(self):
        x, y, yaw = self.entrance_pose
        perp = yaw + math.pi / 2.0
        dx, dy = math.cos(perp), math.sin(perp)
        half_width = self.entrance_wall_width / 2.0
        n = max(2, int(self.entrance_wall_points))
        return [(x + t * dx, y + t * dy, 0.0)
                for t in (-half_width + self.entrance_wall_width * i / (n - 1) for i in range(n))]

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down keepout_zone_node Node.')


def main(args=None):
    run_node(keepout_zone_node, args)


if __name__ == '__main__':
    main()
