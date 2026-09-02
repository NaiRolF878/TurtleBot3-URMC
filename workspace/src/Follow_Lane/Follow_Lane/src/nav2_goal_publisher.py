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
    """
    Einziger FollowPath-Action-Client fuers Projekt -- die Pfad-Quelle haengt
    vom Control-Mode ab:

    - Obstacle: kein globaler Planer vorhanden, also gibt dieser Node dem
      controller_server staendig ein kurzes "fahr geradeaus"-Ziel relativ zum
      Roboter vor. Der Local Controller (MPPI) weicht davon anhand der Local
      Costmap (LiDAR + virtuelle Spurlinien-Waende) ab -> reaktives Fahren im
      freien Korridor.
    - Tunnel: der Pfad kommt stattdessen von tunnel_explorer_node ueber
      /tunnel/explore_path (echte Planung durch bereits bekanntes Gebiet,
      siehe dortiger Docstring). Dieser Node schickt hier nur den jeweils
      zuletzt empfangenen Pfad als FollowPath-Ziel; ist noch keiner da, wird
      gewartet, statt selbst einen Pfad zu erfinden.

    Aktiv in beiden Faellen nur im jeweiligen Control-Mode. Beim Verlassen
    wird ein evtl. laufendes Goal abgebrochen, damit controller_server keine
    veralteten Fahrbefehle mehr sendet.
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
        self.sub_explore_path = self.create_subscription(
            Path, '/tunnel/explore_path', self.cbExplorePath, 1)

        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self._goal_handle = None
        self._goal_active = False
        self._latest_explore_path = None

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

        if new_mode == self.current_control_mode:
            return

        was_active = self.current_control_mode in _ACTIVE_MODES
        is_active = new_mode in _ACTIVE_MODES
        self.current_control_mode = new_mode

        # Jeder Moduswechsel bricht ein laufendes Ziel ab -- auch Obstacle <-> Tunnel,
        # sonst laeuft die Geradeaus-Fahrt im Tunnel einfach weiter.
        if was_active:
            self.cancel_goal()
            self._latest_explore_path = None

        if is_active and not self._goal_active:
            self.try_start_chain()

    def cbExplorePath(self, path):
        self._latest_explore_path = path

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
            # Tunnel-Modus, aber tunnel_explorer_node hat noch keinen Pfad
            # geliefert -- warten statt selbst einen zu erfinden.
            return

        self.send_rolling_goal(path)

    def build_path(self):
        if self.current_control_mode == ControlType.Tunnel:
            return self._latest_explore_path

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
            # ABORTED/CANCELED etc. sind im Dauerbetrieb (rollierendes Ziel) normal,
            # z.B. beim Umschalten des Control-Mode -- daher warn() statt error().
            self.get_logger().warn(
                f'Rolling-Goal beendet: {status_text}.', throttle_duration_sec=2.0)

        # Sofort das naechste Teilstueck nachschieben, solange der Modus noch
        # aktiv ist -- dadurch entsteht die "rollende" Zielverfolgung. Im
        # Tunnel-Modus liefert build_path() dabei entweder den naechsten,
        # zwischenzeitlich neu eingetroffenen Explorations-Pfad, oder (falls
        # noch keiner da ist) None -- try_start_chain() wartet dann einfach.
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
