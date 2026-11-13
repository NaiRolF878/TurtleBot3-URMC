"""Gibt Nav2 im Obstacle- und Tunnel-Modus die Fahrt vor; sonst ruht er.

Ein:  /control/mode, /lane/center_path (Obstacle), /tunnel/explore_path (Tunnel),
      /global_costmap/costmap (Pfad frei? Engstellen), /control/guard (Waechter haelt an),
      /control/maneuver_sequence (Grundbewegungen fertig), /rosout (Abbruchgrund), TF
Aus:  ComputePathToPose an planner_server (Obstacle), FollowPath an controller_server,
      /tunnel/path_result (String, Tunnel: erreicht / kollision / abgebrochen),
      /control/maneuver_request (Obstacle: Engstelle, Freifahren), /nav2/blocked (Obstacle:
      gesperrte Engstellen), /nav2/active_path (Path, latched: der Pfad, dem Nav2 gerade folgt;
      leer, wenn keiner), /nav2/status (String, fuer nav2_view)
Tagebuch: je Obstacle-Abschnitt ein Fahrtprotokoll in $OBSTACLE_LOG_DIR/<Zeit>/ (events, poses,
      scans als jsonl, Kartenbilder der global_costmap -- der Karte, auf der NavFn plant).
"""
import math
import os
import time
from collections import deque

import cv2
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
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Empty, Header, String, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import ManeuverRequest, ManeuverStatus
from Follow_Lane.src import passage_geometry as pg
from Follow_Lane.src.mission_log import MissionLog, scan_record
from Follow_Lane.src.robot_geometry import FOOTPRINT, lethal_cells, transform
from Follow_Lane.src.common import (
    ControlType, cloud_xy, control_mode_from_msg, frange, path_blocked, pose2d_from_transform,
    require_params, run_node, yaw_from_quaternion)

ROBOT_FRAME = 'base_footprint'
# Tagebuch: Verzeichnis aus dieser Umgebungsvariable (launchers/nav2_obstacle.sh).
LOG_DIR_ENV = 'OBSTACLE_LOG_DIR'
# Richtung, in der das Planziel ueber das Ende der Spurmitte hinaus verlaengert wird: die
# Richtung der gemerkten Spurlinien (/detect/lane_obstacles) bis LANE_DIR_RADIUS_M um den Bot --
# der Winkel, bei dem sich die Punkte quer dazu am schaerfsten zu schmalen Streifen buendeln
# (parallele Linien = scharfe Spitzen), gesucht bis LANE_DIR_SEARCH_DEG um die Spurmitte.
# Gilt nur, wenn die Buendelung LANE_DIR_SHARPNESS-mal schaerfer ist als quer dazu (sonst
# Kreuzung, Kurve oder zu wenig Linie). Das letzte Stueck der Spurmitte allein knickt an
# Hindernissen ab (13.11.: -8 statt -29 Grad, Ziel 20 cm neben der Spur).
LANE_DIR_RADIUS_M = 0.8
LANE_DIR_MIN_POINTS = 30
LANE_DIR_SEARCH_DEG = 45.0
LANE_DIR_SHARPNESS = 2.0
LANE_DIR_BIN_M = 0.01
LANE_POINTS_MAX_AGE_S = 1.0
# Liegt das Planziel in Zellen ab GOAL_MAX_COST (OccupancyGrid 0..100; 99 = Bot-Mitte gesperrt),
# wandert es zur naechsten guenstigeren Zelle im Umkreis GOAL_SNAP_M.
GOAL_MAX_COST = 50
GOAL_SNAP_M = 0.25
# Bricht RPP mit Kollisionsgefahr ab, obwohl die Pfadpruefung "frei" sagte und der Umriss nichts
# beruehrt (enge Stelle, RPP schneidet die Kurve), wird das naechste Stueck des Wegs (m) mit
# Grundbewegungen genau abgefahren (FOLLOW) statt freizufahren.
FOLLOW_LENGTH_M = 0.5
# Obstacle: den NavFn-Weg in die Mitte schieben (center_path). Punkte alle CENTER_STEP_M; jeder
# wandert quer zum Weg (bis CENTER_REACH_M) bergauf: in die Richtung, in der der Abstand zum
# naechsten Hindernis-Feld (Dose oder Spurlinie, global_costmap, in alle Richtungen) waechst,
# solange er waechst -- gedeckelt bei CENTER_KEEP_M. Zwischen zwei Hindernissen landet er auf
# ihrer Mittellinie (durch die Mitte der kuerzesten Verbindung, auch wenn sie in Fahrtrichtung
# versetzt sind); neben einem einzelnen auf CENTER_KEEP_M Abstand, nicht weiter. Nie ueber ein
# Hindernis oder Unbekanntes hinweg (13.11.: global gesucht sprang er hinter eine Flasche).
# Am Bot geht der Weg ueber CENTER_RAMP_M in den verschobenen ueber -- kuerzer als RPPs
# Vorausblick (0,15-0,35 m), denn es wird jede Sekunde neu geplant. Danach gleitender
# Mittelwert ueber CENTER_SMOOTH Punkte. Gesperrte Punkte (Bot-Mitte darf nicht hinein, und
# schlechter als der NavFn-Punkt) bleiben auf dem NavFn-Weg.
CENTER_STEP_M = 0.02
CENTER_REACH_M = 0.40
CENTER_KEEP_M = 0.20
CENTER_RAMP_M = 0.10
CENTER_SMOOTH = 5
# Obstacle: Tore (gate_path, vor dem Mittig-Schieben). Hindernis-Felder der global_costmap werden
# zu Hindernissen zusammengefasst (Luecken bis GATE_JOIN_M). Ein Tor ist die kuerzeste Verbindung
# zweier Hindernisse, wenn kein drittes im Kreis darueber liegt (wirklich frei), sie fuer die
# Bot-Mitte breit genug ist (> 2 * GATE_MIN_HALF_M) und hoechstens GATE_MAX_M lang. Der NavFn-Weg
# bestimmt, durch welche Tore es geht; der neue Weg verbindet Bot -> Tormitte -> ... -> Ziel
# gerade (13.11.: die Mitte je Wegpunkt glich gegen das jeweils naechste Hindernis aus, bei
# versetzten Flaschen eine schraeg hinter dem Bot -- der Weg klebte an der gemeinsamen Flasche).
# Gesperrte gerade Stuecke (Kosten >= 99) bleiben auf dem NavFn-Weg. Nur bis GATE_RANGE_M voraus.
GATE_JOIN_M = 0.02
GATE_MIN_HALF_M = 0.10
GATE_MAX_M = 0.70
GATE_RANGE_M = 1.2
# Ecken an den Tormitten abrunden: gleitender Mittelwert ueber so viele Wegpunkte (2 cm Abstand,
# 9 = +/-8 cm). Die Tormitte verfehlt er dabei nur um 1-2 cm; gerundete Punkte, die gesperrt
# waeren (Kosten >= 99, schlechter als vorher), bleiben auf der Ecke (13.11.: an den Knicken
# lenkte "Weg genau abfahren" scharf ein, +25 und -30 Grad auf 64 cm).
GATE_ROUND_POINTS = 9
# Kartenbild im Tagebuch: beim ersten "kein Weg" bzw. Sperren sofort, dann hoechstens so oft (s).
PICTURE_PERIOD_S = 5.0
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
        # Nur fuers Tagebuch: die Karte, auf der RPP und Waechter pruefen.
        self.sub_local_costmap = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.cb_local_costmap, 1)
        self.sub_clear_maps = self.create_subscription(
            Empty, '/control/clear_maps', self.cb_clear_maps, 1)
        self.sub_guard = self.create_subscription(Bool, '/control/guard', self.cb_guard, latched)
        self.sub_sequence = self.create_subscription(
            ManeuverStatus, '/control/maneuver_sequence', self.cb_sequence, latched)
        self.sub_rosout = self.create_subscription(Log, '/rosout', self.cb_rosout, 50)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan_corrected', self.cb_scan, qos_profile_sensor_data)
        self.sub_lane_points = self.create_subscription(
            PointCloud2, '/detect/lane_obstacles', self.cb_lane_points, 1)

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
        self._last_drive_points = None  # (N, 2) des zuletzt an RPP gegebenen Pfads (Freifahren)
        self._last_verdict = None       # Ergebnis der letzten Pfadpruefung (pg.FREE ...)
        self._raw_points = None         # (N, 2) NavFn-Weg vor dem Mittig-Schieben (Tagebuch)
        self._follow_failed = False     # letztes FOLLOW gescheitert -> naechstes Mal freifahren
        self._costmap = None
        self._local_costmap = None
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
        self._lane_points = None        # (Zeit monotonic, frame, (N, 2)) Spurgedaechtnis
        self._log = None                # Tagebuch des laufenden Obstacle-Abschnitts
        self._goal_xy = None            # letztes Planziel (odom), fuers Kartenbild
        self._pictures = 0
        self._last_picture = 0.0        # time.monotonic() des letzten Kartenbilds
        self._no_path = False           # letzte Planung ohne Weg (Bild nur beim Wechsel sofort)

        self.get_logger().info('Started nav2_goal_publisher Node.')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None or new_mode == self.control_mode:
            return

        was_active = self.control_mode in ACTIVE_MODES
        was_planning = self.control_mode in PLAN_MODES
        if was_planning and new_mode not in PLAN_MODES:
            self._close_log(f'Modus {new_mode.name}')
        elif new_mode in PLAN_MODES and not was_planning:
            self._open_log(self.control_mode)
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
        self._follow_failed = False
        self._raw_points = None
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

    def cb_local_costmap(self, grid):
        self._local_costmap = grid

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

    def cb_scan(self, scan):
        if self._log is not None:
            self._log.write('scans', scan_record(scan))

    def cb_lane_points(self, cloud):
        try:
            self._lane_points = (time.monotonic(), cloud.header.frame_id, cloud_xy(cloud))
        except ValueError:
            self._lane_points = None

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
        self._event('maneuver_done', maneuver=int(kind), success=msg.success, reason=msg.reason)
        if kind == ManeuverRequest.PASSAGE:
            passage, self._pending_passage = self._pending_passage, None
            if not msg.success and passage is not None:
                self.get_logger().warn(f'Durchfahrt gescheitert ({msg.reason}) -- Engstelle '
                                       'gesperrt.')
                self._failed_gaps.append(passage[2])
                self._block(passage[2], passage[3], passage[4])
        elif kind == ManeuverRequest.UNWEDGE and not msg.success:
            # Freifahren ging nicht: beim naechsten Abbruch wieder genau abfahren versuchen,
            # sonst wechseln sich Abbruch und "kein freier Ausweg" endlos ab.
            self._follow_failed = False
            self._set_status(f'festgefahren: {msg.reason}')
        elif kind == ManeuverRequest.FOLLOW:
            self._follow_failed = not msg.success
            if not msg.success:
                self.get_logger().warn(f'Weg genau abfahren gescheitert ({msg.reason}) -- '
                                       'beim naechsten Abbruch freifahren.')
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
                self._event('guard_abort')
                self._snapshot('abbruch_waechter', force=True, local=True)
                self._cancel_goal()
                self._on_aborted(COLLISION)
            self._advance()
        if self.control_mode in PLAN_MODES:
            self._publish_blocked()
            if self._log is not None:
                robot = self._robot_pose()
                if robot is not None:
                    self._log.write('poses', {'t': time.monotonic(), 'pose': list(robot),
                                              'status': self._status})
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
            self._event('plan', found=False, periodic=periodic)
            if not self._no_path or self._picture_due():
                self._snapshot('kein_weg')
            self._no_path = True
            if not periodic:
                self._set_status('kein Weg gefunden')
                self.get_logger().warn(
                    'planner_server findet keinen Weg um das Hindernis (Linien oder Hindernisse '
                    'sperren den Korridor?).', throttle_duration_sec=2.0)
            return
        self._no_path = False
        robot = self._robot_pose()
        if robot is None:
            return
        backward = self._backward_extent(path, robot)
        self._event('plan', found=True, periodic=periodic, points=len(path.poses),
                    backward=round(backward, 3))
        if backward > self.get_parameter('max_path_backward').value:
            # Ein laufendes Ziel nach vorn bleibt aktiv; ohne eins steht der Bot.
            self._set_status('Weg fuehrt nach hinten -- verworfen')
            self.get_logger().warn(
                f'Planweg fuehrt {backward:.2f} m hinter den Bot -- verworfen (nur Wege '
                'nach vorn).', throttle_duration_sec=2.0)
            return
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
        gated = self._gate_path(points)
        centered = gated if gated is not None else self._center_path(points)
        if centered is not None:
            self._raw_points = points
            points = centered
            path = self._path_msg(points)
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
            self._event('drive_result', result=REACHED)
            self._on_reached()
        else:
            # Abbrueche sind beim rollierenden Ziel normal, daher nur warn().
            self.get_logger().warn(f'Ziel beendet: {status_text}.', throttle_duration_sec=2.0)
            reason = self._abort_reason()
            now = time.monotonic()
            messages = sorted({m for t, m in self._controller_log
                               if now - t <= ABORT_REASON_WINDOW_S})
            self._event('drive_result', result=reason, status=status_text,
                        controller=messages, guard=self._guard_since is not None)
            self._snapshot(f'abbruch_{reason}', force=True, local=True)
            self._on_aborted(reason)

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
        self._follow_failed = False
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
        if reason == COLLISION and self._follow_instead():
            return
        if reason == COLLISION:
            self._set_status('Kollisionsgefahr -- faehrt frei')
            # Der zuletzt gefahrene Weg: Freifahren dreht sich zuerst in seine Richtung.
            line = self._last_drive_points
            self._request(ManeuverRequest.UNWEDGE, line if line is not None and len(line) > 1
                          else None)
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
        self._event('path_check', verdict=verdict, points=len(points))
        self._last_verdict = verdict
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

    def _gate_path(self, points):
        """NavFn-Weg (N, 2) durch die Mitten der Tore, die er kreuzt, gerade verbunden; None
        ohne Karte oder ohne Tor."""
        if self._costmap is None or len(points) < 3:
            return None
        cells, res, origin = self._grid()
        h, w = cells.shape
        seg = np.hypot(*np.diff(points, axis=0).T)
        along = np.concatenate([[0.0], np.cumsum(seg)])

        # Ausschnitt um den Weg bis GATE_RANGE_M (plus Torlaenge).
        near = points[along <= GATE_RANGE_M]
        lo = np.floor((near.min(axis=0) - GATE_MAX_M - origin) / res).astype(int)
        hi = np.ceil((near.max(axis=0) + GATE_MAX_M - origin) / res).astype(int)
        c0, r0 = max(0, lo[0]), max(0, lo[1])
        c1, r1 = min(w, hi[0]), min(h, hi[1])
        if c1 - c0 < 3 or r1 - r0 < 3:
            return None
        lethal = (cells[r0:r1, c0:c1] >= 100).astype(np.uint8)
        k = 2 * int(round(GATE_JOIN_M / res)) + 1
        n_lab, labels = cv2.connectedComponents(cv2.dilate(lethal, np.ones((k, k), np.uint8)))
        labels[lethal == 0] = 0
        clusters = []
        for lab in range(1, n_lab):
            rr, cc = np.nonzero(labels == lab)
            if rr.size == 0:
                continue
            xy = np.column_stack([origin[0] + (cc + c0 + 0.5) * res, origin[1] + (rr + r0 + 0.5) * res])
            clusters.append(xy[::2] if len(xy) > 400 else xy)
        if len(clusters) < 2:
            return None
        all_xy = np.vstack(clusters)
        owner = np.concatenate([np.full(len(c), i) for i, c in enumerate(clusters)])

        def crossing(a, b):
            """Weglaenge, bei der der Weg die Strecke a-b kreuzt, sonst None."""
            p, q = points[:-1], points[1:]
            d1, d2 = q - p, b - a
            den = d1[:, 0] * d2[1] - d1[:, 1] * d2[0]
            with np.errstate(divide='ignore', invalid='ignore'):
                t = ((a[0] - p[:, 0]) * d2[1] - (a[1] - p[:, 1]) * d2[0]) / den
                u = ((a[0] - p[:, 0]) * d1[:, 1] - (a[1] - p[:, 1]) * d1[:, 0]) / den
            hit = np.flatnonzero((np.abs(den) > 1e-12) & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1))
            if hit.size == 0:
                return None
            i = hit[0]
            return along[i] + t[i] * seg[i]

        gates = []
        for i in range(len(clusters)):
            for j in range(i + 1, len(clusters)):
                A, B = clusters[i], clusters[j]
                d = np.hypot(A[:, None, 0] - B[None, :, 0], A[:, None, 1] - B[None, :, 1])
                ia, ib = np.unravel_index(np.argmin(d), d.shape)
                width = d[ia, ib]
                if width <= 2 * GATE_MIN_HALF_M or width > GATE_MAX_M:
                    continue
                a, b = A[ia], B[ib]
                mid = (a + b) / 2.0
                others = (owner != i) & (owner != j)
                if np.any(np.hypot(*(all_xy[others] - mid).T) < width / 2.0 - res):
                    continue                    # ein drittes Hindernis im Tor
                s_cross = crossing(a, b)
                if s_cross is None or s_cross > GATE_RANGE_M:
                    continue
                gates.append((s_cross, mid, width))
        if not gates:
            return None
        gates.sort(key=lambda g: g[0])
        # Fast gleichzeitig gekreuzte Tore: das engere zaehlt.
        kept = []
        for g in gates:
            if kept and g[0] - kept[-1][0] < 0.05:
                if g[2] < kept[-1][2]:
                    kept[-1] = g
                continue
            kept.append(g)

        # Gerade Stuecke Bot -> Tormitten -> Ziel; gesperrte Stuecke bleiben auf dem NavFn-Weg.
        def blocked(a, b):
            n = max(2, int(np.hypot(*(b - a)) / res) + 1)
            xy = a + (b - a) * np.linspace(0.0, 1.0, n)[:, None]
            c = np.floor((xy - origin) / res).astype(int)
            ok = (c[:, 0] >= 0) & (c[:, 0] < w) & (c[:, 1] >= 0) & (c[:, 1] < h)
            return (not ok.all()) or bool(np.any(cells[c[ok, 1], c[ok, 0]] >= 99))

        anchors = [(0.0, points[0])] + [(g[0], g[1]) for g in kept] + [(along[-1], points[-1])]
        out = [points[:1]]
        used = 0
        for (sa, pa), (sb, pb) in zip(anchors[:-1], anchors[1:]):
            if not blocked(pa, pb):
                n = max(2, int(np.hypot(*(pb - pa)) / CENTER_STEP_M) + 1)
                out.append((pa + (pb - pa) * np.linspace(0.0, 1.0, n)[:, None])[1:])
                used += 1
            else:
                part = points[(along > sa) & (along < sb)]
                out.append(np.vstack([part, pb[None, :]]) if len(part) else pb[None, :])
        result = np.vstack(out)
        if len(result) > GATE_ROUND_POINTS:
            k = GATE_ROUND_POINTS // 2
            padded = np.vstack([np.repeat(result[:1], k, axis=0), result,
                                np.repeat(result[-1:], k, axis=0)])
            kernel = np.ones(GATE_ROUND_POINTS) / GATE_ROUND_POINTS
            rounded = np.column_stack([np.convolve(padded[:, i], kernel, mode='valid')
                                       for i in (0, 1)])
            rounded[0], rounded[-1] = result[0], result[-1]

            def cost_at(xy):
                c = np.floor((xy - origin) / res).astype(int)
                c[:, 0] = np.clip(c[:, 0], 0, w - 1)
                c[:, 1] = np.clip(c[:, 1], 0, h - 1)
                return cells[c[:, 1], c[:, 0]]
            new_cost, old_cost = cost_at(rounded), cost_at(result)
            worse = (new_cost >= 99) & (new_cost > old_cost)
            rounded[worse] = result[worse]
            result = rounded
        self._event('gates', gates=[{'s': round(float(g[0]), 2), 'mid': [round(float(v), 3) for v in g[1]],
                                     'width': round(float(g[2]), 3)} for g in kept],
                    straight=used, pieces=len(anchors) - 1)
        if used == 0:
            return None
        return result

    def _center_path(self, points):
        """NavFn-Weg (N, 2) in die Mitte zwischen den Hindernissen links und rechts geschoben,
        als (M, 2); None ohne Karte oder bei zu kurzem Weg."""
        if self._costmap is None or len(points) < 3:
            return None
        cells, res, origin = self._grid()
        h, w = cells.shape
        # Gleichmaessig CENTER_STEP_M.
        seg = np.hypot(*np.diff(points, axis=0).T)
        along = np.concatenate([[0.0], np.cumsum(seg)])
        if along[-1] < 2 * CENTER_STEP_M:
            return None
        s = np.arange(0.0, along[-1] + 1e-9, CENTER_STEP_M)
        pts = np.column_stack([np.interp(s, along, points[:, 0]), np.interp(s, along, points[:, 1])])
        tangent = np.gradient(pts, axis=0)
        tangent /= np.maximum(np.hypot(tangent[:, 0], tangent[:, 1]), 1e-9)[:, None]
        normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])     # links

        def cost(xy):
            c = np.floor((xy - origin) / res).astype(np.int64)
            ok = (c[..., 0] >= 0) & (c[..., 0] < w) & (c[..., 1] >= 0) & (c[..., 1] < h)
            out = np.full(c.shape[:-1], -1, np.int16)
            out[ok] = cells[c[ok][:, 1], c[ok][:, 0]]
            return out

        # Abstand jeder Zelle zum naechsten Hindernis-Feld (m).
        clear = cv2.distanceTransform((cells < 100).astype(np.uint8), cv2.DIST_L2, 5) * res

        offsets = np.arange(-CENTER_REACH_M, CENTER_REACH_M + 1e-9, res)
        zero = int(np.argmin(np.abs(offsets)))
        cand = pts[:, None, :] + normal[:, None, :] * offsets[None, :, None]       # (N, K, 2)
        cc = np.floor((cand - origin) / res).astype(np.int64)
        inside = (cc[..., 0] >= 0) & (cc[..., 0] < w) & (cc[..., 1] >= 0) & (cc[..., 1] < h)
        col, row = np.clip(cc[..., 0], 0, w - 1), np.clip(cc[..., 1], 0, h - 1)
        c_cost = np.where(inside, cells[row, col], -1)
        usable = inside & (c_cost >= 0) & (c_cost < 99)
        score = np.where(usable, np.minimum(clear[row, col], CENTER_KEEP_M), -1.0)

        shift = np.zeros(len(pts))
        for i in range(len(pts)):
            if not usable[i, zero]:
                continue
            # Bergauf vom NavFn-Punkt: in die Richtung, in der der Abstand waechst, solange er
            # waechst (bis CENTER_KEEP_M). So landet der Punkt auf der naechsten Mittellinie und
            # nie hinter einem Hindernis, wo quer zum Weg noch mehr Platz waere.
            k = zero
            for step in (1, -1):
                j = zero
                while 0 <= j + step < len(offsets) and usable[i, j + step] \
                        and score[i, j + step] > score[i, j] + 1e-6:
                    j += step
                if score[i, j] > score[i, k] + 1e-6:
                    k = j
            shift[i] = offsets[k]
        shift *= np.clip(s / CENTER_RAMP_M, 0.0, 1.0)          # am Bot fest
        moved = pts + normal * shift[:, None]
        if len(moved) >= CENTER_SMOOTH:
            k = CENTER_SMOOTH // 2
            padded = np.vstack([np.repeat(moved[:1], k, axis=0), moved, np.repeat(moved[-1:], k, axis=0)])
            kernel = np.ones(CENTER_SMOOTH) / CENTER_SMOOTH
            smooth = np.column_stack([np.convolve(padded[:, i], kernel, mode='valid') for i in (0, 1)])
            smooth[0] = moved[0]
            moved = smooth
        # Gesperrte Mitte nur zuruecknehmen, wenn sie schlechter ist als der NavFn-Punkt.
        new_cost, old_cost = cost(moved), cost(pts)
        blocked = (new_cost >= 99) & (new_cost > old_cost)
        moved[blocked] = pts[blocked]
        moved[0] = points[0]
        offset = np.hypot(*(moved - pts).T)
        self._event('center', max_shift=round(float(offset.max()), 3),
                    mean_shift=round(float(offset.mean()), 3), points=len(moved),
                    blocked=int(blocked.sum()))
        return moved

    def _follow_instead(self):
        """Statt freizufahren das naechste Stueck des Wegs genau abfahren (FOLLOW)? Nur wenn die
        letzte Pfadpruefung "frei" sagte, der Umriss in der local_costmap nichts beruehrt und
        das letzte FOLLOW nicht gescheitert ist. True, wenn angefordert."""
        points = self._last_drive_points
        if self._last_verdict != pg.FREE or self._follow_failed or points is None \
                or len(points) < 2 or self._local_costmap is None:
            return False
        robot = self._robot_pose()
        if robot is None:
            return False
        info = self._local_costmap.info
        grid = np.asarray(self._local_costmap.data, dtype=np.int16).reshape(info.height, info.width)
        if lethal_cells(grid, info.resolution,
                        (info.origin.position.x, info.origin.position.y), robot):
            return False
        i = int(np.argmin(np.hypot(points[:, 0] - robot[0], points[:, 1] - robot[1])))
        ahead = points[i:]
        walked = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(ahead, axis=0).T))])
        piece = ahead[walked <= FOLLOW_LENGTH_M]
        if len(piece) < 2:
            return False
        self._set_status('enge Stelle -- faehrt Weg genau ab')
        self.get_logger().info(f'RPP meldet Kollision, Weg ist frei -- {walked[len(piece) - 1]:.2f} m '
                               'mit Grundbewegungen genau abfahren.')
        self._request(ManeuverRequest.FOLLOW, piece)
        return True

    def _request(self, kind, line=None):
        """Grundbewegungen bei switch_control anfordern; weiter geht es in cb_sequence."""
        msg = ManeuverRequest(type=kind)
        self._event('maneuver', maneuver=int(kind))
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
        self._event('block', center=[float(center[0]), float(center[1])], width=float(width))
        self._snapshot('engstelle_gesperrt', force=True)

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
            pts = np.array([[p.pose.position.x, p.pose.position.y] for p in center.poses])
            yaw, direction = self._extension_yaw(robot, pts, yaw_from_quaternion(end.orientation))
            # Die Spurmitte reicht nur so weit, wie die Kamera sieht (~0,6 m); in Spurrichtung
            # bis goal_distance verlaengern.
            rest = max(0.0, distance - math.hypot(ex - rx, ey - ry))
            gx, gy = ex + rest * math.cos(yaw), ey + rest * math.sin(yaw)
        else:
            self._path_source = 'geradeaus'
            self.get_logger().info('Keine aktuelle Spurmitte -- Ziel liegt geradeaus.',
                                   throttle_duration_sec=2.0)
            yaw, direction = self._extension_yaw(robot, None, ryaw)
            gx, gy = rx + distance * math.cos(yaw), ry + distance * math.sin(yaw)

        raw = (gx, gy)
        gx, gy = self._snap_goal(gx, gy)
        self._goal_xy = (gx, gy)
        self._event('goal', xy=[gx, gy], source=self._path_source, direction=direction,
                    yaw=yaw, raw=list(raw), robot=[rx, ry, ryaw])
        goal = PoseStamped()
        goal.header.frame_id = ODOM_FRAME
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def _extension_yaw(self, robot, center_pts, fallback_yaw):
        """(Richtung in odom, Quelle) fuer die Verlaengerung: Hauptachse der gemerkten
        Spurlinien um den Bot, sonst Gesamtrichtung der Spurmitte, sonst fallback_yaw.
        Ausgerichtet nach vorn: wie die Spurmitte bzw. die Blickrichtung des Bots."""
        rx, ry, ryaw = robot
        forward = np.array([math.cos(ryaw), math.sin(ryaw)])
        if center_pts is not None and len(center_pts) >= 2:
            chord = center_pts[-1] - center_pts[0]
            if np.hypot(*chord) > 0.05:
                forward = chord / np.hypot(*chord)

        lanes = self._lane_points
        if lanes is not None and time.monotonic() - lanes[0] <= LANE_POINTS_MAX_AGE_S \
                and lanes[1] == ROBOT_FRAME:
            pts = lanes[2]
            near = pts[np.hypot(pts[:, 0], pts[:, 1]) <= LANE_DIR_RADIUS_M]
            if len(near) >= LANE_DIR_MIN_POINTS:
                # Punkte (base_footprint) nach odom drehen, Suche um die Vorwaertsrichtung.
                cy, sy = math.cos(ryaw), math.sin(ryaw)
                world = near @ np.array([[cy, sy], [-sy, cy]])
                ahead = math.atan2(forward[1], forward[0])
                best = self._sharpest_direction(world, ahead)
                if best is not None:
                    return best, 'Spurlinien'

        if center_pts is not None and len(center_pts) >= 3:
            c = center_pts - center_pts.mean(axis=0)
            _, vectors = np.linalg.eigh(c.T @ c)
            axis = vectors[:, -1]
            if axis @ forward < 0.0:
                axis = -axis
            return math.atan2(axis[1], axis[0]), 'Spurmitte gesamt'
        return fallback_yaw, 'letztes Stueck' if center_pts is not None else 'Blickrichtung'

    @staticmethod
    def _sharpest_direction(points, around):
        """Richtung (rad) um around (+/- LANE_DIR_SEARCH_DEG), quer zu der sich die Punkte am
        schaerfsten buendeln; None, wenn das nicht deutlich schaerfer ist als senkrecht dazu."""
        def sharpness(angle):
            across = points @ np.array([-math.sin(angle), math.cos(angle)])
            h = np.bincount(np.floor((across - across.min()) / LANE_DIR_BIN_M).astype(np.int64))
            return float(np.sum(h.astype(np.float64) ** 2))
        search = math.radians(LANE_DIR_SEARCH_DEG)
        coarse = around + np.radians(np.arange(-LANE_DIR_SEARCH_DEG, LANE_DIR_SEARCH_DEG + 0.1, 1.0))
        best = max(coarse, key=sharpness)
        fine = best + np.radians(np.arange(-1.0, 1.01, 0.1))
        best = max(fine, key=sharpness)
        if abs(math.remainder(best - around, 2 * math.pi)) > search:
            return None
        if sharpness(best) < LANE_DIR_SHARPNESS * sharpness(best + math.pi / 2):
            return None
        return float(math.remainder(best, 2 * math.pi))

    def _snap_goal(self, gx, gy):
        """Liegt das Ziel im teuren Saum oder gesperrt, die naechste Zelle unter GOAL_MAX_COST
        im Umkreis GOAL_SNAP_M; sonst bzw. ohne solche Zelle das Ziel selbst."""
        if self._costmap is None:
            return gx, gy
        cells, res, origin = self._grid()
        c, r = int((gx - origin[0]) / res), int((gy - origin[1]) / res)
        h, w = cells.shape
        if not (0 <= r < h and 0 <= c < w) or 0 <= cells[r, c] < GOAL_MAX_COST:
            return gx, gy
        k = int(math.ceil(GOAL_SNAP_M / res))
        r0, r1, c0, c1 = max(0, r - k), min(h, r + k + 1), max(0, c - k), min(w, c + k + 1)
        window = cells[r0:r1, c0:c1]
        rows, cols = np.nonzero((window >= 0) & (window < GOAL_MAX_COST))
        if rows.size == 0:
            return gx, gy
        d = np.hypot(rows + r0 - r, cols + c0 - c)
        i = int(np.argmin(d))
        if d[i] * res > GOAL_SNAP_M:
            return gx, gy
        return (origin[0] + (cols[i] + c0 + 0.5) * res, origin[1] + (rows[i] + r0 + 0.5) * res)

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
        self._last_drive_points = np.array([[p.pose.position.x, p.pose.position.y]
                                            for p in path.poses]) if path.poses else None
        if self.control_mode in PLAN_MODES:
            self._event('drive', source=source, points=len(path.poses))
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

    # --- Tagebuch ---

    def _open_log(self, previous_mode):
        self._close_log('neuer Abschnitt')
        self._pictures = 0
        self._last_picture = 0.0
        self._no_path = False
        self._goal_xy = None
        root = os.environ.get(LOG_DIR_ENV)
        if not root:
            self.get_logger().warn(f'{LOG_DIR_ENV} nicht gesetzt -- kein Obstacle-Tagebuch.',
                                   throttle_duration_sec=60.0)
            return
        self._log = MissionLog(os.path.join(root, time.strftime('%Y%m%d-%H%M%S')),
                               self.get_logger())
        self._event('start', from_mode=previous_mode.name if previous_mode else None)

    def _close_log(self, why):
        if self._log is None:
            return
        self._event('end', why=why)
        self._log.close(self._picture())
        self._log = None

    def _event(self, kind, **data):
        if self._log is not None:
            self._log.write('events', {'t': time.monotonic(), 'kind': kind, **data})

    def _picture_due(self):
        return time.monotonic() - self._last_picture >= PICTURE_PERIOD_S

    def _snapshot(self, why, force=False, local=False):
        """Kartenbild ins Tagebuch: hoechstens alle PICTURE_PERIOD_S, das erste und force
        (gesperrte Engstelle, Fahrtabbruch) sofort. local: local_costmap (RPP, Waechter)
        statt global_costmap (NavFn)."""
        if self._log is None or (self._pictures and not force and not self._picture_due()):
            return
        self._pictures += 1
        self._last_picture = time.monotonic()
        image = self._picture(self._local_costmap if local else None)
        name = self._log.picture(f'{self._pictures:03d}_{why}', image)
        if name is not None:
            self._event('picture', file=name, why=why)

    def _picture(self, costmap=None):
        """Karte (Standard global_costmap, wie NavFn sie sieht): weiss frei, hellblau Saum
        (befahrbar, teuer), rot gesperrt (Bot-Mitte darf nicht hinein; in der local_costmap
        nur der Inkreis des Umrisses), dunkelrot Hindernis selbst (lethal), grau unbekannt.
        Darauf Spurmitte (gelb), NavFn-Weg vor dem Mittig-Schieben (grau), aktiver Pfad
        (magenta), Planziel (X), Sperren (schwarz), Bot-Umriss (blau)."""
        costmap = costmap or self._costmap
        if costmap is None or costmap.header.frame_id not in ('', ODOM_FRAME):
            return None
        info = costmap.info
        grid = np.asarray(costmap.data, dtype=np.int16).reshape(info.height, info.width)
        res, origin = info.resolution, (info.origin.position.x, info.origin.position.y)
        img = np.full(grid.shape + (3,), 255, np.uint8)
        img[grid == -1] = (160, 160, 160)
        img[(grid > 0) & (grid < 99)] = (240, 215, 170)
        img[grid >= 99] = (60, 60, 220)
        img[grid >= 100] = (20, 20, 140)

        def px(xy):
            c = np.floor((np.asarray(xy, dtype=np.float64) - origin) / res).astype(np.int32)
            return tuple(int(v) for v in c)

        center = self._latest_center_path
        if center is not None and len(center.poses) > 1:
            line = np.array([px((p.pose.position.x, p.pose.position.y)) for p in center.poses])
            cv2.polylines(img, [line.astype(np.int32)], False, (0, 200, 255), 1)
        if self._raw_points is not None and len(self._raw_points) > 1:
            line = np.array([px(p) for p in self._raw_points], np.int32)
            cv2.polylines(img, [line], False, (120, 120, 120), 1)
        if self._active_points is not None and len(self._active_points) > 1:
            line = np.array([px(p) for p in self._active_points], np.int32)
            cv2.polylines(img, [line], False, (255, 0, 255), 1)
        for line in self._blocked:
            cv2.line(img, px(line[0]), px(line[-1]), (0, 0, 0), 2)
        if self._goal_xy is not None:
            cv2.drawMarker(img, px(self._goal_xy), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 9, 2)
        robot = self._robot_pose()
        if robot is not None:
            outline = np.array([px(p) for p in transform(robot, FOOTPRINT)], np.int32)
            cv2.polylines(img, [outline], True, (200, 0, 0), 1)
            nose = (robot[0] + 0.08 * math.cos(robot[2]), robot[1] + 0.08 * math.sin(robot[2]))
            cv2.line(img, px(robot[:2]), px(nose), (0, 0, 0), 1)
        return cv2.flip(img, 0)          # Bild: y nach oben

    # --- Beenden ---

    def shut_down(self):
        self._close_log('Node beendet')
        self.get_logger().info('Shutting down nav2_goal_publisher Node.')
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


def main(args=None):
    run_node(nav2_goal_publisher, args)


if __name__ == '__main__':
    main()
