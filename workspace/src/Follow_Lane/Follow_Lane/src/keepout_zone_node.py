import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from std_msgs.msg import UInt8, Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from Follow_Lane.src.switch_control import ControlType


class keepout_zone_node(Node):
    """
    Merkt sich die Roboterpose im odom-Frame in dem Moment, in dem der
    Tunnel-Modus beginnt ("wo bin ich gerade in die Sackgasse reingefahren"),
    und veroeffentlicht ab da laufend eine kurze virtuelle Wand an genau
    dieser Weltposition -- quer zur damaligen Blickrichtung, als PointCloud2,
    nach demselben Muster wie lane_costmap_node fuer die Spurlinien.

    Wichtig: die Wand wird bewusst im odom-Frame (nicht relativ zum Roboter)
    veroeffentlicht, weil sie an einer festen Stelle der Welt bleiben muss,
    unabhaengig davon, wohin sich der Roboter danach bewegt. Die Local
    Costmap ist zwar nur ein mitfahrendes Fenster, aber sobald der Roboter
    sich der gemerkten Position wieder naehert (z.B. Versuch, durch den
    Eingang wieder rauszufahren), rutscht die Wand automatisch wieder ins
    Fenster und wird wie ein normales Hindernis behandelt.

    Es wird immer nur EINE Pose gespeichert. Bei jedem erneuten Eintritt in
    den Tunnel-Modus (z.B. ein zweiter Tunnel weiter auf der Strecke) wird
    sie ueberschrieben -- die alte Wand verschwindet dadurch automatisch aus
    zukuenftigen Veroeffentlichungen. Beim Verlassen des Tunnel-Modus wird
    gar nichts mehr veroeffentlicht.
    """

    def __init__(self):
        super().__init__('keepout_zone_node')

        self.get_logger().info('Started keepout_zone_node Node.')

        # ---------------------------------
        # Parameter:
        # ---------------------------------
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

        self._param_names = ['frame_id', 'robot_frame', 'entrance_wall_width', 'entrance_wall_points']

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #--- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cbGetControlMode, 1)

        #--- Publisher ---
        output_topic = self.get_parameter('output_topic').value
        self.pub_entrance = self.create_publisher(PointCloud2, output_topic, 1)

        #--- Zustand ---
        self.current_control_mode = ControlType.Lane
        self.entrance_pose = None  # (x, y, yaw) im odom-Frame, oder None

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
            # Ueberschreibt eine evtl. noch gemerkte Pose eines frueheren Tunnels.
            self.entrance_pose = None
        elif leaving_tunnel:
            self.entrance_pose = None

    def cbPublish(self):
        if self.current_control_mode != ControlType.Tunnel:
            return

        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

        if self.entrance_pose is None:
            self.entrance_pose = self._capture_entrance_pose()
            if self.entrance_pose is None:
                return  # TF noch nicht verfuegbar -- naechster Timer-Tick versucht's erneut

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        cloud = point_cloud2.create_cloud_xyz32(header, self._wall_points())
        self.pub_entrance.publish(cloud)

    def _capture_entrance_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, self.robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF-Lookup fuer Eingangs-Pose fehlgeschlagen: {e}', throttle_duration_sec=2.0)
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.get_logger().info(f'Tunnel-Eingang gemerkt bei x={x:.2f} y={y:.2f} (odom).')
        return (x, y, yaw)

    def _wall_points(self):
        x, y, yaw = self.entrance_pose
        # Wand quer zur Blickrichtung beim Einfahren -> Richtungsvektor um 90 Grad gedreht.
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
