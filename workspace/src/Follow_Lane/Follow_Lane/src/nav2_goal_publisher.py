import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from std_msgs.msg import UInt8
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus

from Follow_Lane.src.switch_control import ControlType

# Modes, in denen dieser Node ueberhaupt FollowPath-Ziele senden soll.
_ACTIVE_MODES = (ControlType.Obstacle, ControlType.Tunnel)

# Klartext fuer die Statuscodes aus action_msgs/GoalStatus, nur fuer die Logausgabe.
_GOAL_STATUS_TEXT = {
    GoalStatus.STATUS_SUCCEEDED: 'erreicht',
    GoalStatus.STATUS_CANCELED: 'abgebrochen (Moduswechsel)',
    GoalStatus.STATUS_ABORTED: 'abgebrochen durch controller_server '
                                '(z.B. progress_checker/goal_checker oder Kollisionsgefahr)',
    GoalStatus.STATUS_UNKNOWN: 'unbekannt',
}


class nav2_goal_publisher(Node):
    """Einziger FollowPath-Action-Client. Pfadquelle je nach Modus:

    - Obstacle: Spurmitte von lane_costmap_node (center_path_topic), damit MPPI
      auch in Kurven der Spur folgt. Ist sie aelter als center_path_max_age --
      Linien seit einer Weile nicht gesehen --, die Gerade "goal_distance
      geradeaus". MPPI weicht in beiden Faellen anhand der Local Costmap ab.
    - Tunnel: Pfad von tunnel_explorer_node auf /tunnel/explore_path.

    Jeder Moduswechsel bricht ein laufendes Ziel ab.
    """

    def __init__(self):
        super().__init__('nav2_goal_publisher')

        self.get_logger().info('Started nav2_goal_publisher Node.')

        self.declare_parameter('goal_distance', 1.0,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.2, to_value=5.0, step=0.0)]))
        self.declare_parameter('goal_frame_id', 'base_footprint')
        self.declare_parameter('num_waypoints', 5,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=2, to_value=50, step=1)]))
        self.declare_parameter('center_path_topic', '/lane/center_path')
        # Gemessen an der Empfangszeit, nicht am Bildstempel -- die Uhren von Bot
        # und Rechner muessen dafuer nicht synchron laufen.
        self.declare_parameter('center_path_max_age', 1.0,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.1, to_value=10.0, step=0.0)]))
        # Nur Wiederanlauf-Pruefung; neue Ziele kommen ereignisgetrieben aus cb_goal_result.
        self.declare_parameter('watchdog_rate', 1.0,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.2, to_value=10.0, step=0.0)]))

        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cbGetControlMode, 1)
        self.sub_explore_path = self.create_subscription(
            Path, '/tunnel/explore_path', self.cbExplorePath, 1)
        self.sub_center_path = self.create_subscription(
            Path, self.get_parameter('center_path_topic').value, self.cbCenterPath, 1)

        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self._goal_handle = None
        self._goal_active = False
        self._latest_explore_path = None
        self._latest_center_path = None
        self._center_path_received = None

        self.current_control_mode = ControlType.Lane

        watchdog_rate = self.get_parameter('watchdog_rate').value
        self.timer = self.create_timer(1.0 / watchdog_rate, self.cbWatchdog)

    def cbGetControlMode(self, msg):
        try:
            new_mode = ControlType(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unbekannter Control-Mode empfangen: {msg.data}')
            return

        if new_mode == self.current_control_mode:
            return

        was_active = self.current_control_mode in _ACTIVE_MODES
        is_active = new_mode in _ACTIVE_MODES
        self.current_control_mode = new_mode

        # Auch Obstacle <-> Tunnel abbrechen, sonst laeuft die alte Fahrt weiter.
        if was_active:
            self.cancel_goal()
            self._latest_explore_path = None

        if is_active and not self._goal_active:
            self.try_start_chain()

    def cbExplorePath(self, path):
        self._latest_explore_path = path

    def cbCenterPath(self, path):
        self._latest_center_path = path
        self._center_path_received = self.get_clock().now()

    def cbWatchdog(self):
        if self.current_control_mode in _ACTIVE_MODES and not self._goal_active:
            self.try_start_chain()

    def try_start_chain(self):
        if not self._action_client.server_is_ready():
            self.get_logger().warn(
                'follow_path Action-Server nicht erreichbar -- laeuft/aktiviert '
                'der Nav2 controller_server?', throttle_duration_sec=5.0)
            return

        path = self.build_path()
        if path is None:
            # Tunnel-Modus ohne Pfad vom Explorer -- warten
            return

        self.send_rolling_goal(path)

    def build_path(self):
        if self.current_control_mode == ControlType.Tunnel:
            return self._latest_explore_path

        if self._center_path_received is not None:
            age = (self.get_clock().now() - self._center_path_received).nanoseconds / 1e9
            if age <= self.get_parameter('center_path_max_age').value:
                return self._latest_center_path
        self.get_logger().info(
            'Keine aktuelle Spurmitte -- Ziel ist die Gerade geradeaus.',
            throttle_duration_sec=2.0)

        goal_distance = self.get_parameter('goal_distance').value
        frame_id = self.get_parameter('goal_frame_id').value
        num_waypoints = self.get_parameter('num_waypoints').value

        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(num_waypoints):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = goal_distance * i / (num_waypoints - 1)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def send_rolling_goal(self, path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path

        self._goal_active = True
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.cb_goal_response)

    def cb_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(
                'Rolling-Goal wurde vom controller_server abgelehnt.',
                throttle_duration_sec=5.0)
            self._goal_active = False
            return

        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.cb_goal_result)

    def cb_goal_result(self, future):
        self._goal_active = False

        result = future.result()
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN
        status_text = _GOAL_STATUS_TEXT.get(status, f'Statuscode {status}')

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().debug(f'Rolling-Goal {status_text}.')
        else:
            # Abbrueche sind beim rollierenden Ziel normal, daher nur warn()
            self.get_logger().warn(
                f'Rolling-Goal beendet: {status_text}.', throttle_duration_sec=2.0)

        # Naechstes Teilstueck nachschieben -- so entsteht das rollierende Ziel.
        if self.current_control_mode in _ACTIVE_MODES:
            self.try_start_chain()

    def cancel_goal(self):
        if self._goal_handle is not None and self._goal_active:
            self._goal_handle.cancel_goal_async()
        self._goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = nav2_goal_publisher()
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
