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

# Klartext fuer die Statuscodes aus action_msgs/GoalStatus, nur fuer die Logausgabe.
_GOAL_STATUS_TEXT = {
    GoalStatus.STATUS_SUCCEEDED: 'erreicht',
    GoalStatus.STATUS_CANCELED: 'abgebrochen (Moduswechsel)',
    GoalStatus.STATUS_ABORTED: 'abgebrochen durch controller_server '
                                '(z.B. progress_checker/goal_checker oder Kollisionsgefahr)',
    GoalStatus.STATUS_UNKNOWN: 'unbekannt',
}


class nav2_goal_publisher(Node):
    """
    Ersetzt den fehlenden globalen Planer: wir haben kein festes Navigationsziel,
    also gibt dieser Node dem Nav2 controller_server per FollowPath-Action
    staendig ein kurzes "fahr geradeaus"-Ziel relativ zum Roboter vor. Der
    Local Controller (MPPI) weicht davon dann anhand der Local Costmap
    (LiDAR + virtuelle Spurlinien-Waende) ab, so entsteht effektiv reaktives
    Fahren im freien Korridor.

    Aktiv nur im Control-Mode ControlType.Obstacle. Beim Verlassen dieses
    Modes wird ein evtl. laufendes Goal abgebrochen, damit controller_server
    keine veralteten Fahrbefehle mehr sendet.
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
        # Niedrige Frequenz: dient nur als Wiederanlauf-Pruefung, falls z.B. beim
        # Umschalten in den Obstacle-Modus der Action-Server noch nicht bereit war.
        # Das eigentliche "Nachschieben" neuer Ziele passiert ereignisgetrieben in
        # cb_goal_result, sobald das vorherige Ziel erreicht/beendet wurde.
        self.declare_parameter('watchdog_rate', 1.0,
            ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=0.2, to_value=10.0, step=0.0)]))

        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cbGetControlMode, 1)

        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self._goal_handle = None
        self._goal_active = False

        # Sicherer Default, passend zum Default von switch_control: erst nach
        # einer expliziten Obstacle-Mode-Nachricht wird ueberhaupt ein Ziel gesendet.
        self.current_control_mode = ControlType.Lane

        watchdog_rate = self.get_parameter('watchdog_rate').value
        self.timer = self.create_timer(1.0 / watchdog_rate, self.cbWatchdog)

    def cbGetControlMode(self, msg):
        try:
            new_mode = ControlType(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unbekannter Control-Mode empfangen: {msg.data}')
            return

        became_active = (new_mode == ControlType.Obstacle
                          and self.current_control_mode != ControlType.Obstacle)
        became_inactive = (new_mode != ControlType.Obstacle
                            and self.current_control_mode == ControlType.Obstacle)
        self.current_control_mode = new_mode

        if became_inactive:
            self.cancel_goal()
        elif became_active and not self._goal_active:
            self.try_start_chain()

    def cbWatchdog(self):
        if self.current_control_mode == ControlType.Obstacle and not self._goal_active:
            self.try_start_chain()

    def try_start_chain(self):
        if not self._action_client.server_is_ready():
            self.get_logger().warn(
                'follow_path Action-Server nicht erreichbar -- laeuft/aktiviert '
                'der Nav2 controller_server?', throttle_duration_sec=5.0)
            return
        self.send_rolling_goal()

    def build_path(self):
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

    def send_rolling_goal(self):
        goal_msg = FollowPath.Goal()
        goal_msg.path = self.build_path()

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
            # ABORTED/CANCELED etc. sind im Dauerbetrieb (rollierendes Ziel) normal,
            # z.B. beim Umschalten des Control-Mode -- daher warn() statt error().
            self.get_logger().warn(
                f'Rolling-Goal beendet: {status_text}.', throttle_duration_sec=2.0)

        # Sofort das naechste Teilstueck nachschieben, solange der Obstacle-Modus
        # noch aktiv ist -- dadurch entsteht die "rollende" Zielverfolgung.
        if self.current_control_mode == ControlType.Obstacle:
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
