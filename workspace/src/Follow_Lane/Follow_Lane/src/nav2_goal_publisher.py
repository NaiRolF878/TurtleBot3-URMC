"""Gibt Nav2 im TunnelEntry- und Tunnel-Modus die Fahrt vor; in Lane und Obstacle ruht er.

Ein:  /control/mode, /lane/center_path (TunnelEntry), /tunnel/explore_path (Tunnel), TF
Aus:  ComputePathToPose an planner_server (TunnelEntry), FollowPath an controller_server,
      /nav2/active_path (Path, latched: der Pfad, dem Nav2 gerade folgt; leer, wenn keiner),
      /nav2/status (String, fuer nav2_view)
"""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from Follow_Lane.src.common import (
    ControlType, control_mode_from_msg, frange, pose2d_from_transform, run_node,
    yaw_from_quaternion)

# Modi, in denen nav2_goal_publisher selbst ein Ziel setzt und planen laesst.
PLAN_MODES = (ControlType.TunnelEntry,)
ACTIVE_MODES = PLAN_MODES + (ControlType.Tunnel,)

# Antwortet planner_server so lange nicht, gilt die Anfrage als verloren.
PLAN_TIMEOUT_S = 5.0
# lane_costmap_node sendet die Spurmitte erst ab dem Umschalten, also ab dem naechsten
# Kamerabild. So lange (s) wartet die erste Planung darauf, bevor sie geradeaus zielt.
CENTER_PATH_WAIT_S = 0.5

# Klartext der Statuscodes aus action_msgs/GoalStatus, nur fuer das Log.
GOAL_STATUS_TEXT = {
    GoalStatus.STATUS_SUCCEEDED: 'erreicht',
    GoalStatus.STATUS_CANCELED: 'abgebrochen (Moduswechsel)',
    GoalStatus.STATUS_ABORTED: 'abgebrochen durch controller_server '
                               '(z.B. progress_checker/goal_checker, Kollisionsgefahr '
                               'oder von einem neueren Ziel verdraengt)',
    GoalStatus.STATUS_UNKNOWN: 'unbekannt',
}


class nav2_goal_publisher(Node):
    """TunnelEntry: Ziel ist ein Punkt goal_distance voraus auf der Spurmitte von
    lane_costmap_node (ueber ihr Ende hinaus in ihrer Richtung verlaengert). Ist sie
    aelter als center_path_max_age, liegt das Ziel geradeaus. planner_server (NavFn)
    rechnet auf der global_costmap (LiDAR + Spurgedaechtnis) einen Weg darum herum,
    controller_server (MPPI) faehrt ihn ab. Alle replan_period wird neu geplant, damit
    neu gesehene Hindernisse und Linien einfliessen.

    In TunnelEntry fuehrt die Spurmitte zur Tunneloeffnung; NavFn plant durch sie
    hindurch, soweit der LiDAR schon hineinsieht.

    Tunnel: der Pfad von tunnel_explorer_node.

    Jedes gesendete Ziel bekommt eine Generationsnummer; ein Moduswechsel erhoeht sie
    ebenfalls. Antworten und Ergebnisse aelterer Generationen werden ignoriert,
    verspaetet angenommene alte Ziele sofort abgebrochen. Sonst startet das Ergebnis
    eines verdraengten Ziels (controller_server bricht es beim neuen Ziel ab) eine
    zweite Zielkette, und beide verdraengen sich gegenseitig. Planergebnisse gelten nur
    fuer den Modus, in dem sie angefragt wurden.

    Den Grund eines Abbruchs kennt nur controller_server: das Humble-FollowPath-
    Ergebnis meldet nur "abgebrochen". nav2_view zeigt ihn aus /rosout.
    """

    def __init__(self):
        super().__init__('nav2_goal_publisher')

        # --- Parameter ---
        # Abstand des Planziels vom Bot (m). Muss hinter dem Hindernis liegen, sonst
        # endet der Weg davor; MPPI schaut ~1 m voraus (nav2_params.yaml).
        self.declare_parameter('goal_distance', 1.0, frange(0.2, 5.0))
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('center_path_topic', '/lane/center_path')
        # Gemessen an der Empfangszeit, nicht am Bildstempel: die Uhren von Bot
        # und Rechner muessen dafuer nicht synchron laufen.
        self.declare_parameter('center_path_max_age', 1.0, frange(0.1, 10.0))
        self.declare_parameter('replan_period', 1.0, frange(0.2, 10.0))
        self.declare_parameter('watchdog_rate', 2.0, frange(0.2, 10.0))

        # --- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_explore_path = self.create_subscription(
            Path, '/tunnel/explore_path', self.cb_explore_path, 1)
        self.sub_center_path = self.create_subscription(
            Path, self.get_parameter('center_path_topic').value, self.cb_center_path, 1)

        # --- Publisher ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_active_path = self.create_publisher(Path, '/nav2/active_path', latched)
        self.pub_status = self.create_publisher(String, '/nav2/status', 1)
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # --- Timer ---
        self.timer = self.create_timer(
            1.0 / self.get_parameter('watchdog_rate').value, self.cb_watchdog)

        # --- Zustand ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = None
        self._mode_generation = 0
        self._mode_since = time.monotonic()
        self._goal_generation = 0
        self._goal_handle = None
        self._goal_active = False
        self._plan_requested = None     # time.monotonic() der laufenden Planung, sonst None
        self._last_plan_time = None
        self._latest_explore_path = None
        self._latest_center_path = None
        self._center_path_received = None
        self._status = ''
        self._path_source = ''

        self.get_logger().info('Started nav2_goal_publisher Node.')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None or new_mode == self.control_mode:
            return

        was_active = self.control_mode in ACTIVE_MODES
        self.control_mode = new_mode
        self._mode_generation += 1
        self._mode_since = time.monotonic()
        self._last_plan_time = None
        # Pfade gehoeren zu genau einem Abschnitt (Tunneleinfahrt bzw. Tunnel).
        self._latest_explore_path = None
        self._latest_center_path = None
        self._center_path_received = None

        # Auch TunnelEntry -> Tunnel abbrechen, sonst laeuft die alte Fahrt weiter.
        if was_active:
            self._cancel_goal()
        if new_mode in ACTIVE_MODES:
            self._advance()

    def cb_explore_path(self, path):
        self._latest_explore_path = path

    def cb_center_path(self, path):
        first = self._center_path_received is None
        self._latest_center_path = path
        self._center_path_received = self.get_clock().now()
        # Die erste Spurmitte nach dem Umschalten sofort nutzen, nicht erst im Watchdog.
        if first and self.control_mode in PLAN_MODES and not self._goal_active:
            self._advance()

    def cb_watchdog(self):
        if self.control_mode not in ACTIVE_MODES:
            name = self.control_mode.name if self.control_mode else 'kein Modus'
            self._set_status(f'ruht ({name})')
        else:
            self._advance()
        # Zyklisch, damit auch spaeter startende Anzeigen den Stand bekommen.
        self._publish_status()

    def cb_plan_response(self, future, generation):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._plan_requested = None
            self.get_logger().warn('Planungsanfrage vom planner_server abgelehnt.',
                                   throttle_duration_sec=5.0)
            return
        goal_handle.get_result_async().add_done_callback(
            lambda f, g=generation: self.cb_plan_result(f, g))

    def cb_plan_result(self, future, generation):
        self._plan_requested = None
        if generation != self._mode_generation or self.control_mode not in PLAN_MODES:
            return
        result = future.result()
        path = result.result.path if result is not None else None
        if path is None or not path.poses:
            self._set_status('kein Weg gefunden')
            self.get_logger().warn(
                'planner_server findet keinen Weg um das Hindernis (Linien oder Hindernisse '
                'sperren den Korridor?).', throttle_duration_sec=2.0)
            return
        self._send_goal(path, f'Bahnplaner ({self._path_source})')

    def cb_goal_response(self, future, generation):
        goal_handle = future.result()
        stale = generation != self._goal_generation
        if goal_handle is None or not goal_handle.accepted:
            if not stale:
                self.get_logger().warn(
                    'Ziel wurde vom controller_server abgelehnt.', throttle_duration_sec=5.0)
                self._set_status('Ziel abgelehnt')
                self._goal_active = False
            return

        if stale:
            goal_handle.cancel_goal_async()
            return

        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda f, g=generation: self.cb_goal_result(f, g))

    def cb_goal_result(self, future, generation):
        if generation != self._goal_generation:
            return
        self._goal_active = False
        self._goal_handle = None

        result = future.result()
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN
        status_text = GOAL_STATUS_TEXT.get(status, f'Statuscode {status}')
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().debug(f'Ziel {status_text}.')
        else:
            # Abbrueche sind beim rollierenden Ziel normal, daher nur warn().
            self.get_logger().warn(f'Ziel beendet: {status_text}.', throttle_duration_sec=2.0)

        # Naechstes Teilstueck nachschieben -- so entsteht das rollierende Ziel.
        if self.control_mode in ACTIVE_MODES:
            self._last_plan_time = None
            self._advance()

    # --- Hilfsmethoden ---

    def _advance(self):
        """Naechster Schritt im aktiven Modus: planen bzw. Explorer-Pfad abfahren."""
        if not self.follow_client.server_is_ready():
            self._set_status('controller_server nicht erreichbar')
            self.get_logger().warn(
                'follow_path Action-Server nicht erreichbar -- laeuft/aktiviert '
                'der Nav2 controller_server?', throttle_duration_sec=5.0)
            return

        if self.control_mode == ControlType.Tunnel:
            if self._goal_active:
                return
            if self._latest_explore_path is None:
                self._set_status('wartet auf Explorer-Pfad')
                return
            self._send_goal(self._latest_explore_path, 'Explorer-Pfad')
            return

        now = time.monotonic()
        if self._plan_requested is not None:
            if now - self._plan_requested < PLAN_TIMEOUT_S:
                return
            self.get_logger().warn('planner_server antwortet nicht -- neue Anfrage.')
        due = self._last_plan_time is None \
            or now - self._last_plan_time >= self.get_parameter('replan_period').value
        if not self._goal_active or due:
            self._request_plan()

    def _request_plan(self):
        if not self.plan_client.server_is_ready():
            self._set_status('planner_server nicht erreichbar')
            self.get_logger().warn(
                'compute_path_to_pose Action-Server nicht erreichbar -- laeuft planner_server?',
                throttle_duration_sec=5.0)
            return
        goal = self._obstacle_goal()
        if goal is None:
            return

        request = ComputePathToPose.Goal()
        request.goal = goal
        request.use_start = False
        self._plan_requested = self._last_plan_time = time.monotonic()
        if not self._goal_active:
            self._set_status(f'plant Weg ({self._path_source})')
        generation = self._mode_generation
        self.plan_client.send_goal_async(request).add_done_callback(
            lambda f, g=generation: self.cb_plan_response(f, g))

    def _obstacle_goal(self):
        """Planziel im odom-Frame, oder None ohne Roboterpose."""
        odom_frame = self.get_parameter('odom_frame').value
        try:
            tf = self.tf_buffer.lookup_transform(
                odom_frame, self.get_parameter('robot_frame').value, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self._set_status('keine Roboterpose (TF)')
            self.get_logger().warn(f'Keine Roboterpose fuer das Planziel: {e}',
                                   throttle_duration_sec=2.0)
            return None
        rx, ry, ryaw = pose2d_from_transform(tf)
        distance = self.get_parameter('goal_distance').value

        center = self._fresh_center_path()
        if center is None and time.monotonic() - self._mode_since < CENTER_PATH_WAIT_S:
            self._set_status('wartet auf Spurmitte')
            return None
        if center is not None:
            self._path_source = 'Spurmitte'
            end = center.poses[-1].pose
            ex, ey = end.position.x, end.position.y
            yaw = yaw_from_quaternion(end.orientation)
            # Die Spurmitte reicht nur so weit, wie die Kamera sieht (~0,6 m); in ihrer
            # Richtung bis goal_distance verlaengern.
            rest = max(0.0, distance - math.hypot(ex - rx, ey - ry))
            gx, gy = ex + rest * math.cos(yaw), ey + rest * math.sin(yaw)
        else:
            self._path_source = 'geradeaus'
            self.get_logger().info('Keine aktuelle Spurmitte -- Ziel liegt geradeaus.',
                                   throttle_duration_sec=2.0)
            yaw = ryaw
            gx, gy = rx + distance * math.cos(yaw), ry + distance * math.sin(yaw)

        goal = PoseStamped()
        goal.header.frame_id = odom_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def _fresh_center_path(self):
        if self._center_path_received is None or len(self._latest_center_path.poses) < 2:
            return None
        age = (self.get_clock().now() - self._center_path_received).nanoseconds / 1e9
        if age > self.get_parameter('center_path_max_age').value:
            return None
        return self._latest_center_path

    def _send_goal(self, path, source):
        self._goal_generation += 1
        generation = self._goal_generation
        self._goal_active = True
        self._set_status(f'faehrt: {source}')
        self.pub_active_path.publish(path)

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        self.follow_client.send_goal_async(goal_msg).add_done_callback(
            lambda f, g=generation: self.cb_goal_response(f, g))

    def _cancel_goal(self):
        # Neue Generation: eine noch ausstehende Antwort bricht ihr Ziel selbst ab.
        self._goal_generation += 1
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._goal_active = False
        empty = Path()
        empty.header.frame_id = self.get_parameter('odom_frame').value
        empty.header.stamp = self.get_clock().now().to_msg()
        self.pub_active_path.publish(empty)

    def _set_status(self, text):
        if text != self._status:
            self._status = text
            self._publish_status()

    def _publish_status(self):
        self.pub_status.publish(String(data=self._status))

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down nav2_goal_publisher Node.')
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


def main(args=None):
    run_node(nav2_goal_publisher, args)


if __name__ == '__main__':
    main()
