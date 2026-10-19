import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from std_msgs.msg import UInt8, Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from nav2_msgs.srv import ClearEntireCostmap

from Follow_Lane.src.switch_control import ControlType


class keepout_zone_node(Node):
    """Sperrt den Tunnel-Eingang, damit der Bot nicht dort wieder rausfaehrt.

    Merkt sich beim Wechsel in den Tunnel-Modus einmalig die Roboterpose und
    veroeffentlicht dort laufend eine kurze virtuelle Wand als PointCloud2 --
    im odom-Frame, damit sie ortsfest bleibt und wieder in die mitfahrende
    Costmap rutscht, sobald sich der Bot dem Eingang naehert.

    Die Wand bleibt an der gemerkten Stelle, wird aber erst gesendet, wenn der
    Bot sich entrance_wall_delay weit in Einfahrtrichtung davon entfernt hat.
    Sonst stuende er im ersten Moment selbst in ihr, und MPPI saehe jede
    Bewegung als Kollision. Die Wand nach hinten zu verschieben waere keine
    Loesung: dann entstuende eine Luecke, durch die er seitlich wieder hinaus
    koennte.

    Bei jedem neuen Tunnel wird die Pose ueberschrieben und global_costmap
    geleert. Ausserhalb des Tunnel-Modus passiert nichts.
    """

    def __init__(self):
        super().__init__('keepout_zone_node')

        self.get_logger().info('Started keepout_zone_node Node.')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('output_topic', '/detect/tunnel_entrance_obstacle')
        self.declare_parameter('entrance_wall_width', 0.6,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.1, to_value=3.0, step=0.0)]))
        self.declare_parameter('entrance_wall_points', 9,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=2, to_value=50, step=1)]))
        self.declare_parameter('publish_rate', 5.0,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.5, to_value=20.0, step=0.0)]))
        # Heck des Footprints 0.101 m hinter der Radachse + footprint_padding 0.01
        # + eine Zelle 0.05: erst dann liegt keine Wandzelle mehr unter dem Bot.
        self.declare_parameter('entrance_wall_delay', 0.16,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.0, to_value=2.0, step=0.0)]))

        self._param_names = ['frame_id', 'robot_frame', 'entrance_wall_width',
                             'entrance_wall_points', 'entrance_wall_delay']

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cbGetControlMode, 1)

        output_topic = self.get_parameter('output_topic').value
        self.pub_entrance = self.create_publisher(PointCloud2, output_topic, 1)
        self.pub_entrance_pose = self.create_publisher(PoseStamped, '/tunnel/entrance_pose', 1)

        # Standard-Nav2-Service, leert die Tunnelkarte bei jedem neuen Tunnel.
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')

        self.current_control_mode = ControlType.Lane
        self.entrance_pose = None  # (x, y, yaw) im odom-Frame, oder None
        self._wall_active = False

        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.cbPublish)

    def cbGetControlMode(self, msg):
        try:
            new_mode = ControlType(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unbekannter Control-Mode empfangen: {msg.data}')
            return

        entering_tunnel = (new_mode == ControlType.Tunnel
                            and self.current_control_mode != ControlType.Tunnel)
        leaving_tunnel = (new_mode != ControlType.Tunnel
                           and self.current_control_mode == ControlType.Tunnel)

        self.current_control_mode = new_mode

        if entering_tunnel:
            self.entrance_pose = None
            self._wall_active = False
            self._clear_global_costmap()
        elif leaving_tunnel:
            self.entrance_pose = None
            self._wall_active = False

    def _clear_global_costmap(self):
        if not self.clear_global_costmap.service_is_ready():
            self.get_logger().warn(
                'global_costmap-Clear-Service nicht erreichbar -- laeuft planner_server?',
                throttle_duration_sec=5.0)
            return
        self.clear_global_costmap.call_async(ClearEntireCostmap.Request())

    def cbPublish(self):
        if self.current_control_mode != ControlType.Tunnel:
            return

        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

        if self.entrance_pose is None:
            self.entrance_pose = self._lookup_robot_pose()
            if self.entrance_pose is None:
                return  # TF noch nicht verfuegbar -- naechster Timer-Tick versucht's erneut
            x, y, _ = self.entrance_pose
            self.get_logger().info(f'Tunnel-Eingang gemerkt bei x={x:.2f} y={y:.2f} (odom).')

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        # tunnel_explorer_node braucht die Eingangspose sofort, nicht erst mit der Wand.
        self._publish_entrance_pose(header)

        if not self._wall_active:
            if self._distance_past_entrance() < self.entrance_wall_delay:
                return
            self._wall_active = True
            self.get_logger().info('Bot ist vom Eingang weg -- Eingangssperre aktiv.')

        cloud = point_cloud2.create_cloud_xyz32(header, self._wall_points())
        self.pub_entrance.publish(cloud)

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
        """Aktuelle Pose (x, y, yaw) im frame_id, oder None ohne TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, self.robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF-Lookup fuer Roboterpose fehlgeschlagen: {e}', throttle_duration_sec=2.0)
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (x, y, yaw)

    def _wall_points(self):
        x, y, yaw = self.entrance_pose
        # Wand quer zur Einfahrtrichtung
        perp = yaw + math.pi / 2.0
        dx, dy = math.cos(perp), math.sin(perp)

        half_width = self.entrance_wall_width / 2.0
        n = max(2, int(self.entrance_wall_points))

        points = []
        for i in range(n):
            t = -half_width + self.entrance_wall_width * i / (n - 1)
            points.append((x + t * dx, y + t * dy, 0.0))
        return points


def main(args=None):
    rclpy.init(args=args)
    node = keepout_zone_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
