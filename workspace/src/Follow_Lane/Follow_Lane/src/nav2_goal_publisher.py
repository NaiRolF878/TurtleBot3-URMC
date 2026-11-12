"""Gibt Nav2 im Obstacle- und Tunnel-Modus die Fahrt vor; sonst ruht er.

Ein:  /control/mode, /lane/center_path (Obstacle), /tunnel/explore_path (Tunnel),
      /global_costmap/costmap (Pfad frei? Engstellen), /control/guard (Waechter haelt an),
      /control/maneuver_sequence (Grundbewegungen fertig), /rosout (Abbruchgrund), TF
Aus:  ComputePathToPose an planner_server (Obstacle), FollowPath an controller_server,
      /tunnel/path_result (String, Tunnel: erreicht / kollision / abgebrochen),
      /control/maneuver_request (Obstacle: Engstelle, Freifahren), /nav2/blocked (Obstacle:
      gesperrte Engstellen), /nav2/active_path (Path, latched: der Pfad, dem Nav2 gerade folgt;
      leer, wenn keiner), /nav2/status (String, fuer nav2_view)
"""
import math
import time
from collections import deque

import numpy as np

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Empty, Header, String, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import ManeuverRequest, ManeuverStatus
from Follow_Lane.src import passage_geometry as pg
from Follow_Lane.src.common import (
    ControlType, control_mode_from_msg, frange, path_blocked, pose2d_from_transform,
    require_params, run_node, yaw_from_quaternion)

ROBOT_FRAME = 'base_footprint'
ODOM_FRAME = 'odom'
CENTER_PATH_TOPIC = '/lane/center_path'
# Gemessen an der Empfangszeit, nicht am Bildstempel: die Uhren von Bot und Rechner muessen
# dafuer nicht synchron laufen.
CENTER_PATH_MAX_AGE_S = 1.0
WATCHDOG_PERIOD_S = 0.5

# Modi, in denen nav2_goal_publisher selbst ein Ziel setzt und planen laesst.
PLAN_MODES = (ControlType.Obstacle,)
ACTIVE_MODES = PLAN_MODES + (ControlType.Tunnel,)

# Antwortet planner_server so lange nicht, gilt die Anfrage als verloren.
PLAN_TIMEOUT_S = 5.0
# lane_costmap_node sendet die Spurmitte erst ab dem Umschalten, also ab dem naechsten
# Kamerabild. So lange (s) wartet die erste Planung darauf, bevor sie geradeaus zielt.
CENTER_PATH_WAIT_S = 0.5
# Obstacle: Neuplanung im Takt (s); der neue Pfad gilt nur bei gleicher Richtung
# (passage_geometry.same_direction) oder wenn der alte blockiert ist.
REPLAN_PERIOD_S = 1.0
# Haelt der Waechter so lange (s) an, wird das Ziel abgebrochen wie bei einer Kollision.
GUARD_ABORT_S = 1.0
# Abbruchgrund: Meldungen von controller_server so lange (s) vor dem Ergebnis zaehlen.
ABORT_REASON_WINDOW_S = 2.0
# Liegt der Start einer Engstelle so nah (m), beginnen die Grundbewegungen sofort.
PASSAGE_NEAR_M = 0.10
# Engstellen naeher als das (m) sind dieselbe.
SAME_GAP_M = 0.05

# Klartext der Statuscodes aus action_msgs/GoalStatus, nur fuer das Log.
GOAL_STATUS_TEXT = {
    GoalStatus.STATUS_SUCCEEDED: 'erreicht',
    GoalStatus.STATUS_CANCELED: 'abgebrochen (Moduswechsel)',
    GoalStatus.STATUS_ABORTED: 'abgebrochen durch controller_server '
                               '(z.B. progress_checker/goal_checker, Kollisionsgefahr '
                               'oder von einem neueren Ziel verdraengt)',
    GoalStatus.STATUS_UNKNOWN: 'unbekannt',
}

COLLISION = 'kollision'
ABORTED = 'abgebrochen'
REACHED = 'erreicht'


class nav2_goal_publisher(Node):
    """Obstacle: Ziel ist ein Punkt goal_distance voraus auf der Spurmitte von
    lane_costmap_node (ueber ihr Ende hinaus in ihrer Richtung verlaengert). Ist sie
    aelter als CENTER_PATH_MAX_AGE_S, liegt das Ziel geradeaus. planner_server (NavFn)
    rechnet auf der global_costmap (LiDAR + Spurgedaechtnis) einen Weg darum herum, RPP
    faehrt ihn ab. Neu geplant wird, wenn eine gesperrte Zelle auf dem Pfad liegt, wenn er bis
    auf goal_distance/2 abgefahren ist, wenn die Fahrt endet, und im Takt REPLAN_PERIOD_S (dann
    nur bei gleicher Richtung).

    Jeder neue Pfad wird vor dem Losfahren mit dem Umriss geprueft (passage_geometry):
    passt der Bot an einer Engstelle nur ausgerichtet, faehrt RPP bis zum Start der Geraden,
    dann Grundbewegungen (switch_control) hindurch, danach wird neu geplant. Passt er gar
    nicht oder ist die Durchfahrt dort schon gescheitert, wird die Engstelle gesperrt
    (/nav2/blocked, bis Obstacle endet).

    Bricht das Ziel wegen Kollisionsgefahr ab (Meldung von controller_server oder Waechter
    GUARD_ABORT_S lang), faehrt der Bot frei (Grundbewegung, geprueft), dann neue Planung.

    Tunnel: der Pfad von tunnel_explorer_node, jeder genau einmal und sofort, auch mitten in
    einer Fahrt. Das Ergebnis samt Grund (erreicht / kollision / abgebrochen) geht auf
    /tunnel/path_result; was dann geschieht, entscheidet tunnel_explorer_node.

    Jedes gesendete Ziel bekommt eine Generationsnummer; ein Moduswechsel erhoeht sie
    ebenfalls. Antworten und Ergebnisse aelterer Generationen werden ignoriert,
    verspaetet angenommene alte Ziele sofort abgebrochen. Sonst startet das Ergebnis
    eines verdraengten Ziels (controller_server bricht es beim neuen Ziel ab) eine
    zweite Zielkette, und beide verdraengen sich gegenseitig. Planergebnisse gelten nur
    fuer den Modus, in dem sie angefragt wurden.

    Den Grund eines Abbruchs meldet das Humble-FollowPath-Ergebnis nicht; er kommt aus /rosout.
    """

    def __init__(self):
        super().__init__('nav2_goal_publisher')

        # --- Parameter ---
        # Werte nur in Lane_Costmap.yaml. Abstand des Planziels vom Bot (m): muss hinter dem
        # Hindernis liegen, sonst endet der Weg davor.
        self.declare_parameter('goal_distance', Parameter.Type.DOUBLE, frange(0.2, 5.0))
        # Ein Planweg mit einem Punkt mehr als so weit (m) hinter der Radachse wird verworfen:
        # NavFn kennt keine Fahrtrichtung und fuehrt sonst hinter dem Bot aussen um die
        # Spurlinien herum, wo die Kamera nie hingesehen hat. Etwas mehr als das Heck (0.101),
        # damit Pfade aus der eigenen Zelle heraus durchgehen.
        self.declare_parameter('max_path_backward', Parameter.Type.DOUBLE, frange(0.0, 2.0))
        require_params(self, ['goal_distance', 'max_path_backward'])

        # --- Subscriber ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_explore_path = self.create_subscription(
            Path, '/tunnel/explore_path', self.cb_explore_path, 1)
        self.sub_center_path = self.create_subscription(
            Path, CENTER_PATH_TOPIC, self.cb_center_path, 1)
        self.sub_costmap = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.cb_costmap, 1)
        self.sub_clear_maps = self.create_subscription(
            Empty, '/control/clear_maps', self.cb_clear_maps, 1)
        self.sub_guard = self.create_subscription(Bool, '/control/guard', self.cb_guard, latched)
        self.sub_sequence = self.create_subscription(
            ManeuverStatus, '/control/maneuver_sequence', self.cb_sequence, latched)
        self.sub_rosout = self.create_subscription(Log, '/rosout', self.cb_rosout, 50)

        # --- Publisher ---
        self.pub_active_path = self.create_publisher(Path, '/nav2/active_path', latched)
        self.pub_status = self.create_publisher(String, '/nav2/status', 1)
        self.pub_path_result = self.create_publisher(String, '/tunnel/path_result', 1)
        self.pub_request = self.create_publisher(ManeuverRequest, '/control/maneuver_request', 1)
        self.pub_blocked = self.create_publisher(PointCloud2, '/nav2/blocked', 1)
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # --- Timer ---
        self.timer = self.create_timer(
            WATCHDOG_PERIOD_S, self.cb_watchdog)

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
        self._last_plan = 0.0           # time.monotonic() der letzten Planungsanfrage
        self._active_points = None      # (N, 2) des aktiven Obstacle-Pfads, odom
        self._costmap = None
        self._latest_explore_path = None
        self._latest_center_path = None
        self._center_path_received = None
        self._status = ''
        self._path_source = ''
        self._controller_log = deque(maxlen=20)   # (monotonic, Meldung von controller_server)
        self._guard_since = None        # Waechter haelt seit (monotonic)
        self._waiting_for = None        # Obstacle: laufende Grundbewegungen (ManeuverRequest-Typ)
        self._pending_passage = None    # Obstacle: Engstelle, zu deren Start RPP gerade faehrt
        self._failed_gaps = []          # Obstacle: Mitten gescheiterter Durchfahrten (odom)
        self._blocked = []              # Obstacle: Sperrlinien (N, 2) in odom

        self.get_logger().info('Started nav2_goal_publisher Node.')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None or new_mode == self.control_mode:
            return

        was_active = self.control_mode in ACTIVE_MODES
        was_planning = self.control_mode in PLAN_MODES
        self.control_mode = new_mode
        self._mode_generation += 1
        self._mode_since = time.monotonic()
        self._active_points = None
        # Pfade gehoeren zu genau einem Abschnitt (Hindernisbereich bzw. Tunnel).
        self._latest_explore_path = None
        self._latest_center_path = None
        self._center_path_received = None
        self._waiting_for = None
        self._pending_passage = None
        self._failed_gaps = []
        if was_planning or self._blocked:
            # Leere Wolke: die entrance_layer tragen sonst die alten Sperren weiter ein.
            self._blocked = []
            self._publish_blocked()

        # Beim Verlassen eines aktiven Modus abbrechen, sonst laeuft die alte Fahrt weiter.
        if was_active:
            self._cancel_goal()
        if new_mode in ACTIVE_MODES:
            self._advance()

    def cb_explore_path(self, path):
        self._latest_explore_path = path
        if self.control_mode == ControlType.Tunnel:
            self._advance()

    def cb_costmap(self, grid):
        self._costmap = grid

    def cb_clear_maps(self, _msg):
        # Taste L im Tunnel (Neustart vor dem Tunnel): der alte Explorer-Pfad fuehrte vom
        # alten Standort weg, also abbrechen; den naechsten liefert tunnel_explorer_node.
        if self.control_mode != ControlType.Tunnel:
            return
        self._latest_explore_path = None
        self._cancel_goal()
        self._set_status('Tunnel zurueckgesetzt (Taste L)')

    def cb_center_path(self, path):
        first = self._center_path_received is None
        self._latest_center_path = path
        self._center_path_received = self.get_clock().now()
        # Die erste Spurmitte nach dem Umschalten sofort nutzen, nicht erst im Watchdog.
        if first and self.control_mode in PLAN_MODES and not self._goal_active:
            self._advance()

    def cb_guard(self, msg):
        self._guard_since = (self._guard_since or time.monotonic()) if msg.data else None

    def cb_rosout(self, log):
        # Log.WARN ist in Humble ein bytes-Wert, log.level ein int (siehe nav2_view).
        warn = Log.WARN[0] if isinstance(Log.WARN, bytes) else Log.WARN
        if log.level >= warn and log.name.split('.')[0] == 'controller_server':
            self._controller_log.append((time.monotonic(), log.msg))

    def cb_sequence(self, msg):
        if msg.active or self._waiting_for is None or self.control_mode not in PLAN_MODES:
            return
        kind, self._waiting_for = self._waiting_for, None
        if kind == ManeuverRequest.PASSAGE:
            passage, self._pending_passage = self._pending_passage, None
            if not msg.success and passage is not None:
                self.get_logger().warn(f'Durchfahrt gescheitert ({msg.reason}) -- Engstelle '
                                       'gesperrt.')
                self._failed_gaps.append(passage[2])
                self._block(passage[2], passage[3], passage[4])
        elif not msg.success:
            self._set_status(f'festgefahren: {msg.reason}')
        self._active_points = None
        self._advance()

    def cb_watchdog(self):
        if self.control_mode not in ACTIVE_MODES:
            name = self.control_mode.name if self.control_mode else 'kein Modus'
            self._set_status(f'ruht ({name})')
        else:
            if self._goal_active and self._guard_since is not None \
                    and time.monotonic() - self._guard_since >= GUARD_ABORT_S:
                self.get_logger().warn('Waechter haelt an -- Ziel abgebrochen.')
                self._cancel_goal()
                self._on_aborted(COLLISION)
            self._advance()
        if self.control_mode in PLAN_MODES:
            self._publish_blocked()
        # Zyklisch, damit auch spaeter startende Anzeigen den Stand bekommen.
        self._publish_status()

    def cb_plan_response(self, future, generation, periodic):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._plan_requested = None
            self.get_logger().warn('Planungsanfrage vom planner_server abgelehnt.',
                                   throttle_duration_sec=5.0)
            return
        goal_handle.get_result_async().add_done_callback(
            lambda f, g=generation, p=periodic: self.cb_plan_result(f, g, p))

    def cb_plan_result(self, future, generation, periodic):
        self._plan_requested = None
        if generation != self._mode_generation or self.control_mode not in PLAN_MODES \
                or self._waiting_for is not None:
            return
        result = future.result()
        path = result.result.path if result is not None else None
        if path is None or not path.poses:
            if not periodic:
                self._set_status('kein Weg gefunden')
                self.get_logger().warn(
                    'planner_server findet keinen Weg um das Hindernis (Linien oder Hindernisse '
                    'sperren den Korridor?).', throttle_duration_sec=2.0)
            return
        robot = self._robot_pose()
        if robot is None:
            return
        backward = self._backward_extent(path, robot)
        if backward > self.get_parameter('max_path_backward').value:
            # Ein laufendes Ziel nach vorn bleibt aktiv; ohne eins steht der Bot.
            self._set_status('Weg fuehrt nach hinten -- verworfen')
            self.get_logger().warn(
                f'Planweg fuehrt {backward:.2f} m hinter den Bot -- verworfen (nur Wege '
                'nach vorn).', throttle_duration_sec=2.0)
            return
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
        if periodic and self._active_points is not None \
                and not pg.same_direction(self._active_points, points, robot[:2]):
            return
        self._drive(path, points, robot)

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
            self._on_reached()
        else:
            # Abbrueche sind beim rollierenden Ziel normal, daher nur warn().
            self.get_logger().warn(f'Ziel beendet: {status_text}.', throttle_duration_sec=2.0)
            self._on_aborted(self._abort_reason())

    # --- Ablauf ---

    def _on_reached(self):
        if self.control_mode == ControlType.Tunnel:
            self.pub_path_result.publish(String(data=REACHED))
            return
        if self.control_mode not in PLAN_MODES:
            return
        if self._pending_passage is not None:
            self._request(ManeuverRequest.PASSAGE, self._pending_passage[:2])
            return
        # Naechstes Teilstueck nachschieben -- so entsteht das rollierende Ziel.
        self._active_points = None
        self._advance()

    def _on_aborted(self, reason):
        if self.control_mode == ControlType.Tunnel:
            # Was als Naechstes kommt, entscheidet tunnel_explorer_node.
            self.pub_path_result.publish(String(data=reason))
            return
        if self.control_mode not in PLAN_MODES:
            return
        self._pending_passage = None
        self._active_points = None
        if reason == COLLISION:
            self._set_status('Kollisionsgefahr -- faehrt frei')
            self._request(ManeuverRequest.UNWEDGE)
            return
        self._advance()

    def _drive(self, path, points, robot):
        """Pfad pruefen (Engstellen), dann abfahren, bis zum Start einer Engstelle fahren oder
        die Engstelle sperren."""
        verdict, info = pg.FREE, None
        if self._costmap is not None:
            grid, res, origin = self._grid()
            verdict, info = pg.check_path(points, grid, res, origin, robot[:2])
        if verdict == pg.ALIGN and self._near(info[2], self._failed_gaps):
            verdict, info = pg.BLOCK, info[2:]
        if verdict == pg.BLOCK:
            center, axis, width = info
            self.get_logger().warn(f'Engstelle {width:.2f} m: Bot passt nicht hindurch -- '
                                   'gesperrt.', throttle_duration_sec=2.0)
            self._block(center, axis, width)
            self._cancel_goal()
            self._set_status('Engstelle gesperrt, plant neu')
            return
        if verdict == pg.ALIGN:
            start = info[0]
            self._pending_passage = info
            if math.hypot(start[0] - robot[0], start[1] - robot[1]) < PASSAGE_NEAR_M:
                self._cancel_goal()
                self._request(ManeuverRequest.PASSAGE, info[:2])
                return
            points = pg.truncate_at(points, start)
            path = self._path_msg(points)
            self._active_points = points
            self._send_goal(path, f'zur Engstelle ({self._path_source})')
            return
        self._pending_passage = None
        self._active_points = points
        self._send_goal(path, f'Bahnplaner ({self._path_source})')

    def _request(self, kind, line=None):
        """Grundbewegungen bei switch_control anfordern; weiter geht es in cb_sequence."""
        msg = ManeuverRequest(type=kind)
        if line is not None:
            msg.path = self._path_msg(line)
        self._waiting_for = kind
        self.pub_request.publish(msg)
        if kind == ManeuverRequest.PASSAGE:
            self._set_status('Engstelle: ausrichten und durchfahren')

    def _block(self, center, axis, width):
        if self._near(center, [line.mean(axis=0) for line in self._blocked]):
            return
        self._blocked.append(pg.block_points(center, axis, width))
        self._publish_blocked()

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
            if self._latest_explore_path is not None:
                path, self._latest_explore_path = self._latest_explore_path, None
                self._send_goal(path, 'Explorer-Pfad')
            elif not self._goal_active:
                self._set_status('wartet auf Explorer-Pfad')
            return

        if self._waiting_for is not None:
            return
        now = time.monotonic()
        if self._plan_requested is not None:
            if now - self._plan_requested < PLAN_TIMEOUT_S:
                return
            self.get_logger().warn('planner_server antwortet nicht -- neue Anfrage.')
        if not self._goal_active or self._path_needs_replan():
            self._request_plan(periodic=False)
        elif self._pending_passage is None and now - self._last_plan >= REPLAN_PERIOD_S:
            self._request_plan(periodic=True)

    def _path_needs_replan(self):
        """Pfad blockiert oder fast abgefahren? Ohne Pose oder Karte bleibt er bestehen. Auf
        dem Weg zum Start einer Engstelle zaehlt nur blockiert: der Pfad ist dort kurz."""
        points, grid = self._active_points, self._costmap
        if points is None or grid is None:
            return False
        robot = self._robot_pose()
        if robot is None:
            return False
        rx, ry, _ = robot
        ahead = points[int(np.argmin(np.hypot(points[:, 0] - rx, points[:, 1] - ry))):]
        left = float(np.hypot(*np.diff(ahead, axis=0).T).sum()) if len(ahead) > 1 else 0.0
        if self._pending_passage is None \
                and left < self.get_parameter('goal_distance').value / 2.0:
            return True
        cells, res, origin = self._grid()
        return path_blocked(points, (rx, ry), cells, res, origin)

    def _request_plan(self, periodic):
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
        self._plan_requested = self._last_plan = time.monotonic()
        if not self._goal_active:
            self._set_status(f'plant Weg ({self._path_source})')
        generation = self._mode_generation
        self.plan_client.send_goal_async(request).add_done_callback(
            lambda f, g=generation, p=periodic: self.cb_plan_response(f, g, p))

    def _obstacle_goal(self):
        """Planziel im odom-Frame, oder None ohne Roboterpose."""
        robot = self._robot_pose()
        if robot is None:
            self._set_status('keine Roboterpose (TF)')
            return None
        rx, ry, ryaw = robot
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
        goal.header.frame_id = ODOM_FRAME
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    @staticmethod
    def _backward_extent(path, robot):
        """Wie weit (m) der Pfad hinter die Radachse reicht (0 = nie dahinter). Der Pfad liegt
        im odom-Frame (global_frame des Planers) wie robot."""
        rx, ry, ryaw = robot
        c, s = math.cos(ryaw), math.sin(ryaw)
        ahead = min(c * (p.pose.position.x - rx) + s * (p.pose.position.y - ry)
                    for p in path.poses)
        return max(0.0, -ahead)

    def _abort_reason(self):
        """COLLISION, wenn controller_server kurz vor dem Abbruch Kollisionsgefahr gemeldet hat
        (RPP: "detected collision ahead"), sonst ABORTED."""
        now = time.monotonic()
        recent = [m for t, m in self._controller_log if now - t <= ABORT_REASON_WINDOW_S]
        return COLLISION if any('collision' in m.lower() for m in recent) else ABORTED

    def _fresh_center_path(self):
        if self._center_path_received is None or len(self._latest_center_path.poses) < 2:
            return None
        age = (self.get_clock().now() - self._center_path_received).nanoseconds / 1e9
        if age > CENTER_PATH_MAX_AGE_S:
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
        self._active_points = None
        empty = Path()
        empty.header.frame_id = ODOM_FRAME
        empty.header.stamp = self.get_clock().now().to_msg()
        self.pub_active_path.publish(empty)

    def _path_msg(self, points):
        msg = Path()
        msg.header.frame_id = ODOM_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _publish_blocked(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = ODOM_FRAME
        points = [(float(x), float(y), 0.0) for line in self._blocked for x, y in line]
        self.pub_blocked.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _grid(self):
        info = self._costmap.info
        cells = np.asarray(self._costmap.data, dtype=np.int16).reshape(info.height, info.width)
        return cells, info.resolution, (info.origin.position.x, info.origin.position.y)

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Keine Roboterpose: {e}', throttle_duration_sec=2.0)
            return None
        return pose2d_from_transform(tf)

    @staticmethod
    def _near(point, others):
        return any(math.hypot(point[0] - o[0], point[1] - o[1]) < SAME_GAP_M for o in others)

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
