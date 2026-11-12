"""Ablauf der Storage-Aufgabe (URMRC 2026, 4.2.2) als Teil von switch_control.

Kein eigener Node: switch_control legt StorageSequence an, damit es der einzige
Entscheider ueber /control/mode bleibt. Konzept und Zustandsdiagramm:
docs/Storage_Retrieval_Konzept.md.

Ein:  /control/storage_request (Bool, Button), /detect/zone_boundaries,
      und /detect/zone_cross_lines
      (PoseArray, odom), /detect/storage_sign (StorageSign), /detect/pallet
      (PalletDetection), /detect/pallet_grip (Float32), /control/maneuver_status (UInt32), TF
Aus:  /control/storage_state (String, Name aus StorageState), /control/storage_info
      (String, Klartext fuer debug_view), /control/storage_zones (StorageZones, Anzeige
      in storage_zone_node), /control/maneuver (Maneuver), /lokal/magnet_control (Bool)
"""
import math
import time
from collections import Counter

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, String, UInt32
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import Maneuver, PalletDetection, StorageSign, StorageZones
from Follow_Lane.src.common import (
    MAGNET_FRONT_M, ControlType, StorageState, frange, irange, pose2d_from_transform, require_params,
    wrap_angle, yaw_from_quaternion)

S = StorageState

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'
TICK_PERIOD_S = 0.1
STATE_PUBLISH_PERIOD_S = 0.5
# Zustaende, die eine Folge von Grundbewegungen abfahren und dann weiterschalten.
MANEUVER_STATES = (S.ZUR_ZONE_DREHEN, S.ZURUECKDREHEN, S.ZUR_PALETTENZONE, S.AUSRICHTEN,
                   S.ANDOCKEN, S.GRIFF_PRUEFEN, S.NEU_ANSETZEN, S.HERAUSFAHREN, S.TRANSPORT,
                   S.ABLAGE_ANFAHREN, S.VERLASSEN, S.ZURUECK_ZUM_START)
# Mit Palette am Magneten: verschwindet sie aus dem Bild, ist sie verloren.
INACTIVE_STATES = (S.IDLE, S.FERTIG, S.FEHLER)
# Zwei Querlinien naeher als das (m) zaehlen bei der Zonentiefe als eine.
CROSS_LINE_MERGE_M = 0.03
CROSS_LINE_MAX_M = 1.0
# So alt (s) darf die letzte Griffmeldung beim Griff-Pruefen hoechstens sein.
PALLET_FRESH_S = 0.5
MAGNET_TOPIC = '/lokal/magnet_control'

# Reglement: drei Zonen rechts der Spur.
ZONE_COUNT = 3
ZONES_RIGHT = True
# So oft darf die Erkundung nach falscher Zuordnung der Zonen neu beginnen.
MAX_RESTARTS = 2
# Treffer naeher als BOUNDARY_MERGE_M (entlang der Spur) sind dieselbe Linie, bestaetigt
# nach BOUNDARY_CONFIRM Bildern.
BOUNDARY_MERGE_M = 0.05
BOUNDARY_CONFIRM = 3
# Zonenmodell: Abweichung einer Linie vom Raster als Anteil der Zonenbreite.
ZONE_WIDTH_TOL = 0.25
# Schmaler als der Bot kann eine Zone nicht sein (er faehrt hinein); schliesst Raster aus
# Schein-Linien im Abstand weniger Zentimeter aus.
ZONE_MIN_DEPTH = 0.08
# Weicht die gemessene Linienrichtung mehr ab, gilt quer zur Spur.
YAW_IN_MAX_DEG = 30.0
# Spurrichtung aus der ersten gemessenen Zonenrichtung (Zonenkanten liegen parallel zur Spur):
# einmal gesetzt, danach fest. Bis dahin gilt die Startrichtung. Mehr als TRACK_MAX_DEV_DEG davon weg
# gilt als Messfehler (Aufnahme 11.11.: Start 26 Grad schraeg, mit Zonenrichtung parallel).
TRACK_MAX_DEV_DEG = 40.0
# Richtung in die Zone beim Schild-Lesen messen: Querlinien vor dem Bot (Vorder-/Hinterkante der
# Zone, parallel zur Spur) -- mindestens so viele Treffer, sonst bleibt die Modellrichtung.
YAW_IN_MIN_HITS = 5
# Schild lesen: SIGN_SETTLE_S warten, dann SIGN_READ_S abstimmen, ohne Ergebnis
# spaetestens nach SIGN_READ_TIMEOUT_S weiter.
SIGN_SETTLE_S = 0.3
SIGN_READ_S = 1.5
SIGN_READ_TIMEOUT_S = 4.0
SIGN_MIN_VOTES = 3
SIGN_MIN_SHARE = 0.6
# Ein Bild zeigt eine Palette, wenn so viel der Zonenflaeche weder Boden noch Linie ist;
# die Zone ist belegt, wenn das fuer PALLET_MIN_RATIO der Bilder gilt.
FILL_MIN = 0.10
PALLET_MIN_RATIO = 0.5
# Halt vor der Palettenzone (Wertung: Bot positioniert); dabei mindestens so viele
# Messungen der Palettenvorderkante.
POSITION_HOLD_S = 1.0
MIN_FRONT_FRAMES = 3
# Der Metallstreifen ist buendig mit der Stuetze: liegt der Magnet an, ist die Stuetzenkante
# (am Boden, die BEV misst sie richtig) am Magnetende, MAGNET_FRONT_M vor der Radachse.
MAGNET_EDGE_M = MAGNET_FRONT_M
# Ausrichten: seitlicher Versatz zur Palette ab ALIGN_MIN_M wird entlang der Spur ausgeglichen
# (drehen auf der Stelle, parallel zur Spur fahren, zurueckdrehen). Der Kurs in die Zone kommt
# aus den Zonenlinien bzw. der Spur -- die Palette liegt laut Reglement gerade.
ALIGN_MIN_M = 0.015
# Andocken nur vorwaerts geradeaus (Kurs gehalten): zuegig bis PREDOCK_M vor dem erwarteten
# Kontakt, dann mit DOCK_VEL bis hoechstens DOCK_OVERSHOOT_M darueber hinaus.
PREDOCK_M = 0.10
DOCK_VEL = 0.03
DOCK_OVERSHOOT_M = 0.04
# Kontakt kann erst gelten, wenn der Bot bis auf so viel (m) an den erwarteten Kontakt heran ist.
DOCK_CHECK_M = 0.05
# Kontakt: das Bild ueber dem Magneten steht still (pallet_node, /detect/pallet_still), obwohl
# der Bot vorfaehrt -- er schiebt die Palette. So viele Bilder in Folge ab CONTACT_STILL_MIN.
CONTACT_STILL_MIN = 0.95
CONTACT_FRAMES = 6
DOCK_TIMEOUT_S = 20.0
MAX_ATTEMPTS = 3
MAGNET_ON_WAIT_S = 1.0
MAGNET_OFF_WAIT_S = 1.0
# Griff pruefen: so weit zuruecksetzen (m).
GRIP_CHECK_M = 0.03
# Palette haengt am Magneten, wenn das Bild ueber dem Magneten nach dem Zuruecksetzen mindestens
# so aehnlich ist wie beim Greifen (Korrelation von pallet_node, 0..1). Nur dort geprueft: einmal
# gegriffen, verliert der Magnet die Palette nicht. Beim Tragen gab der Vergleich Fehlalarm
# (Spiegelung im Metallstreifen aendert sich beim Drehen, Lauf 11.11.).
GRIP_MIN = 0.8
STEP_TIMEOUT_S = 30.0


class Zone:
    """Eine bei der Erkundung besuchte Zone. index: Platz im Zonenmodell (0 = erste in
    Fahrtrichtung), None beim Rueckfall. track_xy/yaw_in: Pose auf der Spur vor der
    Zonenmitte, Blick in die Zone. depth: Abstand von dort bis zur Zonenmitte (m)."""

    def __init__(self, index, s_center, center_xy, yaw_in, fallback):
        self.index = index
        self.s_center = s_center
        self.center_xy = center_xy
        self.yaw_in = yaw_in
        self.fallback = fallback
        self.track_xy = None
        self.digit = None
        self.occupied = 0.0         # Anteil der Bilder mit Palette
        self.depth = None

    def name(self):
        if self.digit:
            return str(self.digit)
        return f'Platz {self.index + 1}' if self.index is not None else '?'

    def label(self):
        return (str(self.digit) if self.digit else '?') + ('P' if self.occupied else '')


class StorageSequence:
    """Erkundung: Spurfolge (Lane). Querlinien neben der Spur werden ueber die Strecke s
    entlang der Startrichtung gebuendelt. Daraus das Zonenmodell (_zone_model): ZONE_COUNT
    gleich breite Zonen, also ZONE_COUNT + 1 Linien im gleichen Abstand, das Raster, das die
    meisten Querlinien erklaert. Fehlende Linien werden ergaenzt.

    In der Mitte von Zone k (beide Linien gemessen, mindestens drei Treffer im Modell oder
    Bot schon hinter der Zone) faehrt er genau hin, dreht zur Zone, liest Schild, Palette
    und Zonentiefe und dreht zurueck. Ohne Modell dreht er fallback_step hinter der letzten
    Grenze zur Seite und liest nur das Schild.

    Belegt ist eine Zone, wenn genug Bilder genug Flaeche zeigen, die weder Boden noch Linie
    ist (fill von storage_zone_node).

    Danach: per odom vor die Palettenzone, halten (Wertung) und dabei seitlichen Versatz und
    Abstand der Palette messen. Versatz entlang der Spur ausgleichen (auf der Stelle drehen,
    parallel zur Spur fahren, zurueckdrehen; quer zur Spur nie rueckwaerts), Kurs in die Zone
    aus den Zonenlinien. Nur vorwaerts geradeaus andocken, bis der Magnet anliegt (das Bild
    ueber ihm steht still, obwohl der Bot vorfaehrt), greifen, Griff pruefen (nach dem
    Zuruecksetzen Korrelation von pallet_node ab GRIP_MIN), gerade zurueck auf die Spur, an der
    Spur entlang vor die Zielzone, so tief hinein wie beim Aufnehmen, ablegen, rueckwaerts
    zurueck auf die Spur.

    Ziffer einer Zone: zone_order (Ziffern in Fahrtrichtung) nach ihrem Platz im Modell,
    das Schild kontrolliert nur; leer = Schild. Palettenzone: wo die Palette gesehen wurde
    (sonst Abbruch, nicht raten). Zielzone: Ziffer target_zone.
    """

    def __init__(self, node):
        self.node = node

        # --- Parameter ---
        p = node.declare_parameter
        p('target_zone', Parameter.Type.INTEGER, irange(1, 3))
        p('zone_order', Parameter.Type.STRING)
        p('fallback_step', Parameter.Type.DOUBLE, frange(0.0, 3.0))
        p('explore_max_distance', Parameter.Type.DOUBLE, frange(0.2, 10.0))
        p('carry_max_vel', Parameter.Type.DOUBLE, frange(0.01, 0.22))
        p('zone_width', Parameter.Type.DOUBLE, frange(0.10, 1.0))
        p('zone_depth', Parameter.Type.DOUBLE, frange(0.05, 1.0))
        p('line_width', Parameter.Type.DOUBLE, frange(0.0, 0.10))
        self._param_names = [
            'target_zone', 'zone_order', 'fallback_step', 'explore_max_distance',
            'carry_max_vel', 'zone_width', 'zone_depth', 'line_width',
        ]
        require_params(node, self._param_names)
        self._read_params()

        # --- Subscriber ---
        node.create_subscription(Bool, '/control/storage_request', self.cb_request, 1)
        node.create_subscription(PoseArray, '/detect/zone_boundaries', self.cb_boundaries, 5)
        node.create_subscription(PoseArray, '/detect/zone_cross_lines', self.cb_cross_lines, 5)
        node.create_subscription(StorageSign, '/detect/storage_sign', self.cb_sign, 10)
        node.create_subscription(PalletDetection, '/detect/pallet', self.cb_pallet, 1)
        node.create_subscription(Float32, '/detect/pallet_grip', self.cb_grip, 1)
        node.create_subscription(Float32, '/detect/pallet_still', self.cb_still, 1)
        node.create_subscription(UInt32, '/control/maneuver_status', self.cb_maneuver_status, 1)

        # --- Publisher ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = node.create_publisher(String, '/control/storage_state', latched)
        self.pub_info = node.create_publisher(String, '/control/storage_info', latched)
        self.pub_zones = node.create_publisher(StorageZones, '/control/storage_zones', latched)
        self.pub_maneuver = node.create_publisher(Maneuver, '/control/maneuver', 1)
        self.pub_magnet = node.create_publisher(Bool, MAGNET_TOPIC, 1)

        # --- Timer ---
        node.create_timer(TICK_PERIOD_S, self.cb_tick)
        node.create_timer(STATE_PUBLISH_PERIOD_S, self._publish_state)

        # --- Zustand ---
        self.state = S.IDLE
        self._since = time.monotonic()
        self._reason = ''
        self._magnet = False
        self._maneuver_id = 0
        self._queue = []                # noch abzufahrende Grundbewegungen
        self._reset_run()

    # --- Oeffentlich fuer switch_control ---

    @property
    def active(self):
        return self.state not in INACTIVE_STATES

    def shut_down(self):
        self._set_magnet(False, force=True)

    # --- Callbacks ---

    def cb_request(self, msg):
        if msg.data:
            self._start()
        elif self.state != S.IDLE:
            self.node.get_logger().info('Storage von Hand abgebrochen.')
            self._abort()

    def cb_boundaries(self, msg):
        if self.state != S.ERKUNDEN:
            return
        # Eine breite Linie liefert mehrere Segmente; je Bild zaehlt jede Grenze einmal.
        seen = set()
        for pose in msg.poses:
            p = (pose.position.x, pose.position.y)
            b = self._add_hit(self._boundaries, self._s_of(p), seen,
                              yaw_from_quaternion(pose.orientation), xy=p)
            if b is not None:
                # Die Grenze selbst, nicht die Bot-Position: die BEV sieht bis 0,6 m voraus.
                self._last_boundary_s = max(self._last_boundary_s, b['s'])
                self.node.get_logger().info(f'Zonengrenze bestaetigt bei s={b["s"]:.2f} m.')

    def cb_cross_lines(self, msg):
        if self.state != S.SCHILD_LESEN or not self._reading():
            return
        robot = self._robot_pose()
        if robot is None:
            return
        c, s = math.cos(robot[2]), math.sin(robot[2])
        for pose in msg.poses:
            f = (pose.position.x - robot[0]) * c + (pose.position.y - robot[1]) * s
            if 0.0 < f < CROSS_LINE_MAX_M:
                self._cross_lines.append(f)
                self._cross_yaws.append(yaw_from_quaternion(pose.orientation))

    def cb_sign(self, msg):
        if self.state != S.SCHILD_LESEN or not self._reading():
            return
        if msg.digit > 0:
            self._sign_votes[msg.digit] += 1

    def cb_pallet(self, msg):
        if self.state == S.SCHILD_LESEN and self._reading() and msg.fill >= 0.0:
            self._pallet_frames += 1
            self._pallet_found += int(msg.fill >= FILL_MIN)
        elif self.state == S.POSITIONIERT and msg.found:
            self._fronts.append((msg.distance, msg.lateral, msg.yaw))

    def cb_still(self, msg):
        """Kontakt: Bild ueber dem Magneten steht CONTACT_FRAMES lang still, und der Bot ist
        (Odometrie) bis auf DOCK_CHECK_M an den erwarteten Kontakt heran."""
        if self.state != S.ANDOCKEN:
            return
        robot = self._robot_xy()
        near = robot is not None and self._dock_start is not None \
            and math.dist(robot, self._dock_start) >= self._approach - DOCK_CHECK_M
        log = self._dock_log
        log['frames'] += 1
        log['max'] = max(log['max'], msg.data)
        if near:
            log['near_frames'] += 1
            log['max_near'] = max(log['max_near'], msg.data)
        if near and msg.data >= CONTACT_STILL_MIN:
            self._contact += 1
            log['best_run'] = max(log['best_run'], self._contact)
            if self._contact >= CONTACT_FRAMES:
                self._log_dock(f'Magnet liegt an (Bild steht still, {msg.data:.2f})')
                self._enter(S.GREIFEN)
        else:
            self._contact = 0

    def _log_dock(self, outcome):
        """Andock-Versuch fuers Log: wie weit gefahren, wie still das Bild war."""
        log, robot = self._dock_log, self._robot_xy()
        driven = math.dist(robot, self._dock_start) if robot and self._dock_start else float('nan')
        def f(v):
            return f'{v:.2f}' if v >= 0 else '-'
        self.node.get_logger().info(
            f'Andocken: {outcome}. Gefahren {driven:.3f} m von erwartet {self._approach:.3f} m '
            f'(Kontakt zaehlt ab {self._approach - DOCK_CHECK_M:.3f} m). Stillstand: '
            f'{log["frames"]} Bilder, max {f(log["max"])}; im Kontaktbereich {log["near_frames"]} '
            f'Bilder, max {f(log["max_near"])}, laengste Folge >= {CONTACT_STILL_MIN} '
            f'{log["best_run"]}/{CONTACT_FRAMES}.')

    def cb_grip(self, msg):
        self._grip_score = msg.data
        self._attached_time = time.monotonic()

    def cb_maneuver_status(self, msg):
        if self.state not in MANEUVER_STATES or not self._queue:
            return
        if msg.data != self._queue[0].id:
            return
        self._queue.pop(0)
        if self._queue:
            self._publish_maneuver()
        else:
            self._maneuvers_done()

    def cb_tick(self):
        self._read_params()
        if self.state in MANEUVER_STATES and self._queue:
            # Zyklisch, falls control_driving eine Nachricht verpasst hat.
            self._publish_maneuver()
        elapsed = time.monotonic() - self._since

        if self.state == S.ERKUNDEN:
            self._tick_explore()
        elif self.state == S.SCHILD_LESEN:
            self._tick_read(elapsed)
        elif self.state == S.POSITIONIERT and elapsed >= POSITION_HOLD_S:
            self._plan_dock()
        elif self.state == S.ANDOCKEN and elapsed >= DOCK_TIMEOUT_S:
            self._log_dock('Zeit abgelaufen')
            self._retry('Andocken dauert zu lange')
        elif self.state == S.GREIFEN and elapsed >= MAGNET_ON_WAIT_S:
            self._enter(S.GRIFF_PRUEFEN)
        elif self.state == S.ABLEGEN and elapsed >= MAGNET_OFF_WAIT_S:
            self._report('Palette abgelegt')
            self._enter(S.VERLASSEN)

        if self.state in MANEUVER_STATES and elapsed >= STEP_TIMEOUT_S:
            self._fail(f'{self.state.name} dauert laenger als {STEP_TIMEOUT_S:.0f} s')

    # --- Ablauf ---

    def _start(self):
        if self.active:
            return
        if self.node.control_mode in (ControlType.TunnelEntry, ControlType.Tunnel,
                                      ControlType.Obstacle):
            self.node.get_logger().warn(
                f'Storage nicht gestartet: Modus {self.node.control_mode.name} laeuft.')
            return
        pose = self._robot_pose()
        if pose is None:
            self.node.get_logger().warn('Storage nicht gestartet: keine Roboterpose (TF).')
            return
        self._reset_run()
        if self._magnet:
            self.node.get_logger().info('Neuer Start: Magnet vom letzten Lauf noch an -- aus.')
            self._set_magnet(False, force=True)
        self._origin = self._start_xy = (pose[0], pose[1])
        self._track_yaw = self._start_yaw = pose[2]
        self.node.get_logger().info(
            f'Storage gestartet, Ziel Zone {self.target_zone}'
            + (f', Ziffern in Fahrtrichtung {self.zone_order}' if self.zone_order else '') + '.')
        self._enter(S.ERKUNDEN)

    def _abort(self):
        self._release_unless_gripped()
        self._queue = []
        self._enter(S.IDLE)
        self.node.set_initial_mode('Storage abgebrochen')

    def _fail(self, reason):
        self.node.get_logger().error(f'Storage FEHLER: {reason}')
        self._reason = reason
        self._enter(S.FEHLER)

    def _retry(self, reason):
        self._attempts += 1
        self.node.get_logger().warn(f'{reason} -- Versuch {self._attempts}/{MAX_ATTEMPTS}.')
        if self._attempts >= MAX_ATTEMPTS:
            self._fail(f'{reason}, Versuche aufgebraucht')
        else:
            self._enter(S.NEU_ANSETZEN)

    def _enter(self, state):
        self.node.get_logger().info(f'Storage: {self.state.name} => {state.name}')
        self.state = state
        self._since = time.monotonic()
        self._queue = []
        mode = ControlType.Stop

        if state == S.IDLE:
            mode = None
        elif state == S.ERKUNDEN:
            mode = ControlType.Lane
        elif state == S.ZUR_ZONE_DREHEN:
            z = self._zones[-1]
            self._queue = [self._path_to(z.center_xy), self._rotate(z.yaw_in)]
        elif state == S.SCHILD_LESEN:
            self._sign_votes = Counter()
            self._pallet_frames = self._pallet_found = 0
            self._cross_lines = []
            self._cross_yaws = []
        elif state == S.ZURUECKDREHEN:
            self._queue = [self._rotate(self._track_yaw)]
        elif state == S.ZUR_PALETTENZONE:
            z = self._pallet_zone
            self._queue = [self._path_to(z.track_xy), self._rotate(z.yaw_in)]
        elif state == S.POSITIONIERT:
            self._fronts = []
        elif state == S.AUSRICHTEN:
            self._queue = list(self._align_queue)
        elif state == S.ANDOCKEN:
            self._contact = 0
            self._dock_start = self._robot_xy()
            self._dock_log = {'frames': 0, 'max': -1.0, 'near_frames': 0, 'max_near': -1.0,
                              'best_run': 0}
            # Magnet schon beim Anfahren an: beruehrt er die Palette, haelt er sie sofort, sie
            # bewegt sich mit dem Bot und das Bild steht sicher still (Lauf 11.11.: Kontakt nie
            # erkannt). Haelt er die Palette nicht (Griffpruefung), schaltet NEU_ANSETZEN ihn aus.
            self._set_magnet(True)
            fast = self._approach - PREDOCK_M
            slow = min(self._approach, PREDOCK_M) + DOCK_OVERSHOOT_M
            self._queue = ([self._straight(fast)] if fast > 0.01 else []) + \
                [self._straight(slow, max_vel=DOCK_VEL)]
        elif state == S.GREIFEN:
            self._contact_xy = self._robot_xy()
            self._set_magnet(True)
        elif state == S.GRIFF_PRUEFEN:
            self._queue = [self._straight(-GRIP_CHECK_M)]
        elif state == S.NEU_ANSETZEN:
            # Gerade zurueck auf den Spurpunkt, von dem aus angedockt wurde -- nur bis zur Spur.
            self._set_magnet(False)
            self._queue = [self._path_to(self._dock_lane_xy(), reverse=True),
                           self._rotate(self._pallet_zone.yaw_in)]
        elif state == S.HERAUSFAHREN:
            self._queue = [self._path_to(self._dock_lane_xy(), reverse=True,
                                         max_vel=self.carry_max_vel)]
        elif state == S.TRANSPORT:
            self._report(f'lege Palette in die Zone {self._target_zone.name()}')
            src, dst = self._dock_lane_xy(), self._target_zone.track_xy
            heading = math.atan2(dst[1] - src[1], dst[0] - src[0])
            self._queue = [self._rotate(heading, max_vel=self.carry_max_vel),
                           self._path_to(dst, reverse=False, max_vel=self.carry_max_vel),
                           self._rotate(self._target_zone.yaw_in, max_vel=self.carry_max_vel)]
        elif state == S.ABLAGE_ANFAHREN:
            self._queue = [self._path_to(self._drop_point(), reverse=False,
                                         max_vel=self.carry_max_vel)]
        elif state == S.ABLEGEN:
            self._gripped = False
            self._set_magnet(False)
        elif state == S.VERLASSEN:
            self._queue = [self._path_to(self._target_zone.track_xy, reverse=True),
                           self._rotate(self._track_yaw)]
        elif state == S.FERTIG:
            mode = ControlType.Lane
        elif state == S.ZURUECK_ZUM_START:
            self._queue = [self._path_to(self._start_xy, reverse=True),
                           self._rotate(self._track_yaw)]
        elif state == S.FEHLER:
            self._release_unless_gripped()

        if self._queue:
            mode = ControlType.Maneuver
        if mode is not None:
            self.node._set_mode(mode, f'Storage {state.name}')
        if self._queue:
            self._publish_maneuver()
        self._publish_state()

    def _maneuvers_done(self):
        if self.state == S.ZUR_ZONE_DREHEN:
            self._enter(S.SCHILD_LESEN)
        elif self.state == S.ZURUECKDREHEN:
            robot = self._robot_pose()
            if robot:
                self._last_stop_xy = (robot[0], robot[1])
                self._last_stop_s = self._s_of(robot)
            if self._plan():
                self._enter(S.ZUR_PALETTENZONE)
            elif sum(1 for z in self._zones if not z.fallback) >= \
                    ZONE_COUNT - (self._target_index() is not None):
                self._finish_exploration()
            else:
                self._enter(S.ERKUNDEN)
        elif self.state == S.ZUR_PALETTENZONE:
            self._enter(S.POSITIONIERT)
        elif self.state == S.AUSRICHTEN:
            self._enter(S.ANDOCKEN)
        elif self.state == S.ANDOCKEN:
            # Strecke bis zum erwarteten Kontakt + DOCK_OVERSHOOT_M abgefahren, Magnet ist an.
            # Der Stillstand kam im ersten Versuch nie (die Palette rutscht beim Schieben, Lauf
            # 11.11.); ob sie haengt, entscheidet die Griffpruefung nach dem Zuruecksetzen.
            self._log_dock('Strecke abgefahren, Stillstand nicht erkannt -- greife trotzdem')
            self._enter(S.GREIFEN)
        elif self.state == S.GRIFF_PRUEFEN:
            self._check_grip()
        elif self.state == S.NEU_ANSETZEN:
            self._enter(S.POSITIONIERT)
        elif self.state == S.HERAUSFAHREN:
            self._enter(S.TRANSPORT)
        elif self.state == S.TRANSPORT:
            self._enter(S.ABLAGE_ANFAHREN)
        elif self.state == S.ABLAGE_ANFAHREN:
            self._enter(S.ABLEGEN)
        elif self.state == S.VERLASSEN:
            self.node.get_logger().info('Storage fertig.')
            self._enter(S.FERTIG)
        elif self.state == S.ZURUECK_ZUM_START:
            self._enter(S.ERKUNDEN)

    # --- Erkundung ---

    def _tick_explore(self):
        robot = self._robot_pose()
        if robot is None:
            return
        s = self._s_of(robot)
        self._model = self._zone_model()
        self._log_model()
        if self._model_moved():
            self._restart_exploration()
            return

        nxt = self._next_model_zone(s)
        if nxt is not None:
            k, s_center = nxt
            # Punkt auf der Spur auf Hoehe der Zonenmitte (seitliche Lage des Bots bleibt).
            d = s_center - s
            xy = (robot[0] + d * math.cos(self._track_yaw), robot[1] + d * math.sin(self._track_yaw))
            self._zones.append(Zone(k, s_center, xy, self._model['yaw_in'], fallback=False))
            self._enter(S.ZUR_ZONE_DREHEN)
            return

        if self._model is None and self.fallback_step > 0.0 \
                and s - max(self._last_stop_s, self._last_boundary_s) >= self.fallback_step:
            self.node.get_logger().info('Rueckfall: keine Zonengrenze erkannt, drehe zur Seite.')
            self._zones.append(Zone(None, s, (robot[0], robot[1]), self._side_yaw(), fallback=True))
            self._enter(S.ZUR_ZONE_DREHEN)
            return

        if s >= self.explore_max_distance:
            self._finish_exploration()

    def _add_hit(self, clusters, s, seen, yaw=None, xy=None):
        """Treffer bei s einem Buendel zuordnen; gibt das Buendel zurueck, wenn es damit
        gerade bestaetigt ist."""
        for c in clusters:
            if abs(c['s'] - s) <= BOUNDARY_MERGE_M:
                if id(c) in seen:
                    return None
                seen.add(id(c))
                n = c['n']
                c['s'] = (c['s'] * n + s) / (n + 1)
                if xy is not None and c.get('xy') is not None:
                    c['xy'] = ((c['xy'][0] * n + xy[0]) / (n + 1), (c['xy'][1] * n + xy[1]) / (n + 1))
                if yaw is not None:
                    # Richtung als Einheitsvektor mitteln, sonst springt der Mittelwert bei +-pi.
                    c['dir'] = (c['dir'][0] + math.cos(yaw), c['dir'][1] + math.sin(yaw))
                c['n'] = n + 1
                return c if c['n'] == BOUNDARY_CONFIRM else None
        c = {'s': s, 'n': 1, 'xy': xy,
             'dir': (0.0, 0.0) if yaw is None else (math.cos(yaw), math.sin(yaw))}
        seen.add(id(c))
        clusters.append(c)
        return None

    def _zone_model(self):
        """ZONE_COUNT + 1 Linien im Abstand zone_width + line_width (Linienmitte zu
        Linienmitte) aus den bestaetigten Querlinien, oder None. Dict: s (Linien), measured,
        hits (Anzahl gemessener Linien), width, yaw_in.

        Jede Querlinie darf jede Grenzlinie sein; es gewinnt die Lage, die die meisten Linien
        erklaert. Linien innerhalb des Rasters, die nicht passen, und fehlende Linien, wo schon
        hingesehen wurde, zaehlen dagegen. Schon eine Linie legt alle Zonen fest (sie gilt dann
        als Grenze vor Zone 1); spaetere pruefen gegen und duerfen die Lage verschieben. Von der
        Palette verdeckte Linien schaden nicht."""
        lines = [c for c in self._boundaries if c['n'] >= BOUNDARY_CONFIRM]
        marks = sorted(c['s'] for c in lines)
        pitch = self.zone_width + self.line_width
        best = None
        for a in marks:
            for i in range(ZONE_COUNT + 1):
                fit = self._fit_grid(a - i * pitch, pitch, marks)
                if best is None or self._score(fit) > self._score(best):
                    best = fit
        if best is None:
            return None

        dx = sum(c['dir'][0] for c in lines)
        dy = sum(c['dir'][1] for c in lines)
        yaw_in = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else self._side_yaw()
        if abs(wrap_angle(yaw_in - self._side_yaw())) > math.radians(YAW_IN_MAX_DEG):
            yaw_in = self._side_yaw()
        best['yaw_in'] = yaw_in
        return best

    @staticmethod
    def _fit_grid(a, width, marks):
        """Raster a + i * width (i = 0..ZONE_COUNT): Treffer zuordnen, Lage ausgleichen."""
        tol = ZONE_WIDTH_TOL * width
        hits = {}
        extra = 0           # Linien im Raster, die zu keiner Rasterlinie passen
        for m in marks:
            i = int(round((m - a) / width))
            if 0 <= i <= ZONE_COUNT and abs(m - (a + i * width)) <= tol:
                hits.setdefault(i, []).append(m)
            elif a - tol < m < a + ZONE_COUNT * width + tol:
                extra += 1
        if hits:
            offsets = [m - i * width for i, ms in hits.items() for m in ms]
            a = sum(offsets) / len(offsets)
        err = sum((m - (a + i * width)) ** 2 for i, ms in hits.items() for m in ms)
        lines = [a + i * width for i in range(ZONE_COUNT + 1)]
        measured = [i in hits for i in range(ZONE_COUNT + 1)]
        # Ergaenzte Linien, wo schon gemessen wurde: dort haette man sie sehen muessen.
        misses = sum(1 for s, ok in zip(lines, measured) if not ok and s < marks[-1])
        return {'s': lines, 'measured': measured, 'hits': len(hits), 'misses': misses,
                'extra': extra, 'err': err, 'width': width}

    @staticmethod
    def _score(fit):
        return fit['hits'] - fit['misses'] - fit['extra'], fit['hits'], -round(fit['err'], 9)

    def _next_model_zone(self, s_robot):
        """(Platz k, s der Mitte) der naechsten nicht besuchten Zone, deren Mitte der Bot
        erreicht hat und die sicher genug ist; sonst None."""
        model = self._model
        if model is None:
            return None
        s, measured = model['s'], model['measured']
        target = self._target_index()
        for k in range(ZONE_COUNT):
            # Zielzone ist laut Aufgabe immer frei: nicht anfahren, Lage aus dem Modell.
            if k == target or any(z.index == k for z in self._zones):
                continue
            s_center = (s[k] + s[k + 1]) / 2.0
            if s_robot < s_center:
                return None
            sure = (measured[k] and measured[k + 1]) or model['hits'] >= 3
            # Schon hinter der Zone: weiter kommt nichts mehr dazu.
            return (k, s_center) if sure or s_robot >= s[k + 1] else None
        return None

    def _model_moved(self):
        """True, wenn eine besuchte Zone im Modell mehr als eine halbe Zonenbreite weiter
        liegt als beim Besuch: die erste Linie war keine Grenze, Schild und Palette gehoeren
        zu einer anderen Zone."""
        m = self._model
        if m is None:
            return False
        half = (self.zone_width + self.line_width) / 2.0
        return any(abs((m['s'][z.index] + m['s'][z.index + 1]) / 2.0 - z.s_center) > half
                   for z in self._zones if z.index is not None)

    def _restart_exploration(self):
        """Rueckwaerts zum Start und neu erkunden; die Linien bleiben, das Modell ist also
        schon korrigiert. Hoechstens MAX_RESTARTS-mal."""
        if self._restarts >= MAX_RESTARTS:
            self._fail(f'Zonen nach {MAX_RESTARTS} Neustarts nicht zuzuordnen')
            return
        self._restarts += 1
        self._report(f'Zonen falsch zugeordnet -- zurueck zum Start ({self._restarts}/'
                     f'{MAX_RESTARTS})')
        self._zones = []
        self._pallet_zone = self._target_zone = None
        self._last_stop_xy = None
        self._last_stop_s = 0.0
        self._enter(S.ZURUECK_ZUM_START)

    def _log_model(self):
        """Zonenmodell ins Log, wenn es sich merklich aendert."""
        m = self._model
        key = None if m is None else (tuple(round(v, 2) for v in m['s']), tuple(m['measured']))
        if key == self._model_key:
            return
        self._model_key = key
        if m is not None:
            lines = ' / '.join(f'{v:.2f}' + ('' if ok else '*')
                               for v, ok in zip(m['s'], m['measured']))
            self.node.get_logger().info(
                f'Zonenmodell: Linien {lines} m (* = ergaenzt), Breite {m["width"]:.3f} m.')

    def _tick_read(self, elapsed):
        if elapsed < SIGN_SETTLE_S + SIGN_READ_S and elapsed < SIGN_READ_TIMEOUT_S:
            return
        z = self._zones[-1]
        order = self._zone_order()
        by_hand = order[z.index] if z.index is not None and z.index < len(order) else None
        digit = self._vote_digit()
        if digit is None and by_hand is None and elapsed < SIGN_READ_TIMEOUT_S:
            return

        robot = self._robot_pose()
        if robot is not None:
            z.track_xy = (robot[0], robot[1])
            z.yaw_in = robot[2]
            measured = self._measured_yaw_in()
            if measured is not None:
                self.node.get_logger().info(
                    f'Richtung in die Zone gemessen ({len(self._cross_yaws)} Linien): '
                    f'{math.degrees(wrap_angle(measured - robot[2])):+.1f} Grad gegenueber der '
                    'Blickrichtung.')
                z.yaw_in = measured
                if not self._track_from_zone:
                    self._set_track_from_zone(measured)
        z.digit = by_hand if by_hand is not None else digit
        if by_hand is not None and digit is not None and digit != by_hand:
            self.node.get_logger().warn(
                f'Zone {by_hand}: Schild zeigt {digit}, es gilt die Reihenfolge (zone_order).')
        if self._pallet_frames > 0:
            ratio = self._pallet_found / self._pallet_frames
            z.occupied = ratio if ratio >= PALLET_MIN_RATIO else 0.0
        z.depth = self._zone_depth()
        self.node.get_logger().info(
            f'Zone {z.name()}: Schild-Stimmen {dict(self._sign_votes)}, Palette '
            f'{self._pallet_found}/{self._pallet_frames} Bilder, '
            f'Tiefe {"?" if z.depth is None else f"{z.depth:.2f} m"}.')
        if z.occupied:
            self._report(f'Palette in Zone {z.name()} gefunden')

        if z.fallback and z.digit is None:
            self.node.get_logger().info('Rueckfall: kein Schild gelesen, Stelle verworfen.')
            self._zones.pop()
        elif z.fallback and any(o.digit == z.digit for o in self._zones[:-1]):
            self.node.get_logger().info(f'Rueckfall: Schild {z.digit} schon bekannt, verworfen.')
            self._zones.pop()
        self._enter(S.ZURUECKDREHEN)

    def _set_track_from_zone(self, yaw_in):
        """Spurrichtung = gemessene Richtung in die Zone -/+ 90 Grad (einmal). Ursprung bleibt
        der Startpunkt; alle bisherigen Strecken (Zonengrenzen, letzter Halt) werden aus ihren
        Bodenpunkten neu berechnet, das Zonenmodell liegt danach parallel zur Spur."""
        track = wrap_angle(yaw_in + (math.pi / 2 if ZONES_RIGHT else -math.pi / 2))
        dev = math.degrees(wrap_angle(track - self._start_yaw))
        if abs(dev) > TRACK_MAX_DEV_DEG:
            self.node.get_logger().warn(
                f'Spurrichtung aus der Zonenrichtung {dev:+.1f} Grad neben der Startrichtung -- '
                'verworfen.')
            return
        self._track_yaw = track
        self._track_from_zone = True
        for b in self._boundaries:
            if b.get('xy') is not None:
                b['s'] = self._s_of(b['xy'])
        confirmed = [b['s'] for b in self._boundaries if b['n'] >= BOUNDARY_CONFIRM]
        self._last_boundary_s = max(confirmed, default=0.0)
        if self._last_stop_xy is not None:
            self._last_stop_s = self._s_of(self._last_stop_xy)
        for z in self._zones:
            z.s_center = self._s_of(z.center_xy)
        self.node.get_logger().info(
            f'Spurrichtung aus der Zonenrichtung: {dev:+.1f} Grad gegenueber der Startrichtung '
            '(gilt ab jetzt).')

    def _measured_yaw_in(self):
        """Richtung in die Zone aus den Querlinien vor dem Bot (Vorder-/Hinterkante der Zone,
        parallel zur Spur): senkrecht dazu, auf der Seite der Zonen. None bei zu wenig Treffern
        oder mehr als YAW_IN_MAX_DEG neben der Querrichtung zur Spur."""
        if len(self._cross_yaws) < YAW_IN_MIN_HITS:
            return None
        # Linien haben keine Richtung: Winkel verdoppeln, mitteln, halbieren.
        cx = sum(math.cos(2.0 * a) for a in self._cross_yaws)
        cy = sum(math.sin(2.0 * a) for a in self._cross_yaws)
        line = 0.5 * math.atan2(cy, cx)
        side = self._side_yaw()
        yaw_in = min((wrap_angle(line + math.pi / 2), wrap_angle(line - math.pi / 2)),
                     key=lambda a: abs(wrap_angle(a - side)))
        if abs(wrap_angle(yaw_in - side)) > math.radians(TRACK_MAX_DEV_DEG):
            return None
        return yaw_in

    def _vote_digit(self):
        total = sum(self._sign_votes.values())
        if total == 0:
            return None
        digit, votes = self._sign_votes.most_common(1)[0]
        if votes >= SIGN_MIN_VOTES and votes / total >= SIGN_MIN_SHARE:
            return digit
        return None

    def _zone_depth(self):
        """Abstand Bot -> Zonenmitte aus vorderster und hinterster Querlinie."""
        if not self._cross_lines:
            return None
        clusters = []
        for f in sorted(self._cross_lines):
            if clusters and f - clusters[-1][-1] <= CROSS_LINE_MERGE_M:
                clusters[-1].append(f)
            else:
                clusters.append([f])
        lines = [sum(c) / len(c) for c in clusters if len(c) >= 2]
        if len(lines) < 2 or lines[-1] - lines[0] < ZONE_MIN_DEPTH:
            return None
        return (lines[0] + lines[-1]) / 2.0

    def _plan(self, final=False):
        """Paletten- und Zielzone bestimmen; True, wenn beide bekannt und verschieden.
        Ohne gesehene Palette meldet final 'keine Palette gefunden'."""
        known = [z for z in self._zones if z.track_xy is not None]
        occupied = [z for z in known if z.occupied]
        if occupied:
            self._pallet_zone = max(occupied, key=lambda z: z.occupied)
        elif final:
            self._report('keine Palette gefunden')
            self._pallet_zone = None
        else:
            self._pallet_zone = None
        self._target_zone = next((z for z in known if z.digit == self.target_zone), None) \
            or self._target_from_model()
        ok = self._pallet_zone is not None and self._target_zone is not None \
            and self._pallet_zone is not self._target_zone
        if ok:
            self.node.get_logger().info(
                f'Palette in Zone {self._pallet_zone.label()}, Ziel Zone {self._target_zone.label()}.')
        return ok

    def _target_index(self):
        """Platz der Zielzone im Modell aus zone_order, None ohne Reihenfolge (dann Schild)."""
        order = self._zone_order()
        return order.index(self.target_zone) if self.target_zone in order else None

    def _target_from_model(self):
        """Zielzone ohne Besuch (sie ist immer frei): Spurpunkt auf Hoehe ihrer Modellmitte,
        auf der Linie des Spurpunkts einer besuchten Zone (Palettenzone bevorzugt), Richtung in
        die Zone wie dort gemessen (Zonen liegen parallel). None ohne Modell oder Bezug."""
        k, m = self._target_index(), self._model
        ref = self._pallet_zone or next((z for z in self._zones if z.track_xy is not None), None)
        if k is None or m is None or ref is None:
            return None
        s_center = (m['s'][k] + m['s'][k + 1]) / 2.0
        d = s_center - self._s_of(ref.track_xy)
        xy = (ref.track_xy[0] + d * math.cos(self._track_yaw),
              ref.track_xy[1] + d * math.sin(self._track_yaw))
        z = Zone(k, s_center, xy, ref.yaw_in, fallback=False)
        z.track_xy = xy
        z.digit = self.target_zone
        self.node.get_logger().info(
            f'Zielzone {self.target_zone} nicht angefahren (immer frei): Mitte aus dem Modell bei '
            f's={s_center:.2f} m, {d:+.2f} m von Zone {ref.name()}.')
        return z

    def _finish_exploration(self):
        if self._plan(final=True):
            self._enter(S.ZUR_PALETTENZONE)
            return
        if self._pallet_zone is None:
            self._fail('Palette in keiner Zone gesehen')
        elif self._target_zone is None:
            self._fail(f'Zielzone {self.target_zone} nicht gefunden')
        else:
            self._fail(f'Palette liegt schon in der Zielzone {self.target_zone}')

    # --- Andocken und Tragen ---

    def _plan_dock(self):
        """Aus den Messungen beim Halt: seitlicher Versatz zur Palette und Abstand bis zum
        Kontakt. Der Versatz wird entlang der Spur ausgeglichen (auf der Stelle parallel zur
        Spur drehen, vor oder zurueck, zurueck zur Zone drehen), sonst nur der Kurs in die Zone
        (Zonenlinien bzw. Spur) gesetzt. Den Winkel der Palette braucht es nicht."""
        robot = self._robot_pose()
        if len(self._fronts) < MIN_FRONT_FRAMES or robot is None:
            self._retry('Palette beim Positionieren nicht gefunden')
            return
        dist, lat = (sorted(v)[len(v) // 2] for v in list(zip(*self._fronts))[:2])
        # Links vom Bot (Blick in die Zone) auf die Spurrichtung umrechnen.
        track = (math.cos(self._track_yaw), math.sin(self._track_yaw))
        shift = lat * (-math.sin(robot[2]) * track[0] + math.cos(robot[2]) * track[1])
        if abs(shift) <= ALIGN_MIN_M:
            shift = 0.0
        self._lane_xy = (robot[0] + shift * track[0], robot[1] + shift * track[1])
        self._approach = max(0.0, dist - MAGNET_EDGE_M)
        yaw_in = self._pallet_zone.yaw_in
        if abs(shift) > ALIGN_MIN_M:
            self._align_queue = [self._rotate(self._track_yaw), self._straight(shift),
                                 self._rotate(yaw_in)]
        else:
            self._align_queue = [self._rotate(yaw_in)]
        self.node.get_logger().info(
            f'Palette vermessen: {dist:.3f} m voraus, seitlich {lat:+.3f} m -- '
            + (f'entlang der Spur {shift:+.3f} m, ' if abs(shift) > ALIGN_MIN_M else '')
            + f'dann {self._approach:.3f} m geradeaus bis zum Kontakt.')
        self._enter(S.AUSRICHTEN)

    def _dock_lane_xy(self):
        """Spurpunkt, von dem aus angedockt wird (nach dem Ausgleich), sonst der der Zone."""
        return self._lane_xy if self._lane_xy is not None else self._pallet_zone.track_xy

    def _check_grip(self):
        fresh = time.monotonic() - self._attached_time <= PALLET_FRESH_S
        score = f'{self._grip_score:.2f}' if fresh else 'keine aktuelle Messung'
        self.node.get_logger().info(f'Griff: Korrelation {score} (mindestens {GRIP_MIN}).')
        if fresh and self._grip_score >= GRIP_MIN:
            self._gripped = True
            self._report('Palette erfolgreich aufgenommen')
            self._enter(S.HERAUSFAHREN)
        else:
            self._retry('Palette kommt beim Zuruecksetzen nicht mit')

    def _release_unless_gripped(self):
        """Magnet aus -- ausser die Palette haengt dran: dann fiele sie mitten auf die Spur.
        Ausschalten dann von Hand (Topic, siehe Log) oder mit botdown."""
        if self._gripped:
            self.node.get_logger().warn(
                'Palette haengt am Magneten -- Magnet bleibt AN. Ablegen von Hand: ros2 topic pub '
                f'--once {MAGNET_TOPIC} std_msgs/msg/Bool "{{data: false}}"')
        else:
            self._set_magnet(False, force=True)

    def _drop_point(self):
        """Ablagepunkt der Radachse: so tief in die Zielzone wie beim Aufnehmen in die
        Palettenzone (Spurpunkt -> Kontakt). Die Palette lag dort in der Zonenmitte, abgelegt
        wird auch in der Mitte -- dafuer braucht es keine Palettenmasse. Sind beide Zonentiefen
        gemessen, um deren Unterschied verschoben."""
        z, src = self._target_zone, self._pallet_zone
        dist = math.dist(self._dock_lane_xy(), self._contact_xy) \
            if self._contact_xy is not None else 0.0
        if z.depth is not None and src.depth is not None:
            dist += z.depth - src.depth
        self.node.get_logger().info(f'Ablage {dist:.2f} m tief in Zone {z.name()}.')
        dist = max(0.0, dist)
        return (z.track_xy[0] + dist * math.cos(z.yaw_in), z.track_xy[1] + dist * math.sin(z.yaw_in))

    # --- Grundbewegungen ---

    def _new_maneuver(self, kind, max_vel):
        self._maneuver_id += 1
        m = Maneuver()
        m.header.frame_id = ODOM_FRAME
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.id = self._maneuver_id
        m.type = kind
        m.max_vel = float(max_vel)
        return m

    def _rotate(self, yaw, max_vel=0.0):
        m = self._new_maneuver(Maneuver.ROTATE, max_vel)
        m.yaw = float(yaw)
        return m

    def _straight(self, distance, max_vel=0.0):
        m = self._new_maneuver(Maneuver.STRAIGHT, max_vel)
        m.distance = float(distance)
        return m

    def _path_to(self, xy, reverse=None, max_vel=0.0):
        """Zum Punkt xy (odom). reverse=None: rueckwaerts, wenn er hinter dem Bot liegt."""
        m = self._new_maneuver(Maneuver.PATH, max_vel)
        robot = self._robot_pose()
        if reverse is None:
            reverse = robot is not None and \
                (xy[0] - robot[0]) * math.cos(robot[2]) + (xy[1] - robot[1]) * math.sin(robot[2]) < 0.0
        m.reverse = bool(reverse)
        m.path.header = m.header
        pose = PoseStamped()
        pose.header = m.header
        pose.pose.position.x, pose.pose.position.y = float(xy[0]), float(xy[1])
        pose.pose.orientation.w = 1.0
        m.path.poses = [pose]
        return m

    def _publish_maneuver(self):
        self.pub_maneuver.publish(self._queue[0])

    # --- Hilfsmethoden ---

    def _read_params(self):
        for name in self._param_names:
            setattr(self, name, self.node.get_parameter(name).value)

    def _reset_run(self):
        self._origin = self._start_xy = (0.0, 0.0)
        self._track_yaw = self._start_yaw = 0.0
        self._last_stop_xy = None
        self._track_from_zone = False   # Spurrichtung schon aus einer Zone gemessen
        self._cross_yaws = []
        self._boundaries = []
        self._model = None
        self._model_key = None
        self._zones = []
        self._last_stop_s = 0.0
        self._last_boundary_s = 0.0
        self._pallet_zone = self._target_zone = None
        self._attempts = 0
        self._restarts = 0
        self._contact = 0
        self._contact_xy = None
        self._lane_xy = None
        self._approach = 0.0
        self._align_queue = []
        self._dock_start = None
        self._fronts = []
        self._grip_score = 0.0
        self._attached_time = 0.0
        self._gripped = False           # Griff bestaetigt, bis ABLEGEN
        self._dock_log = {'frames': 0, 'max': -1.0, 'near_frames': 0, 'max_near': -1.0,
                          'best_run': 0}
        self._sign_votes = Counter()
        self._pallet_frames = self._pallet_found = 0
        self._cross_lines = []
        self._reason = ''
        self._message = ''

    def _reading(self):
        return time.monotonic() - self._since >= SIGN_SETTLE_S

    def _zone_order(self):
        try:
            return [int(t) for t in str(self.zone_order).replace(' ', '').split(',') if t]
        except ValueError:
            self.node.get_logger().warn(f'zone_order "{self.zone_order}" nicht lesbar.',
                                        throttle_duration_sec=10.0)
            return []

    def _side_yaw(self):
        return self._track_yaw + (-math.pi / 2 if ZONES_RIGHT else math.pi / 2)

    def _s_of(self, p):
        """Strecke entlang der Startrichtung vom Startpunkt bis p (m)."""
        return (p[0] - self._origin[0]) * math.cos(self._track_yaw) \
            + (p[1] - self._origin[1]) * math.sin(self._track_yaw)

    def _robot_pose(self):
        try:
            tf = self.node.tf_buffer.lookup_transform(
                ODOM_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().warn(f'Keine Roboterpose fuer Storage: {e}',
                                        throttle_duration_sec=5.0)
            return None
        return pose2d_from_transform(tf)

    def _robot_xy(self):
        pose = self._robot_pose()
        return None if pose is None else (pose[0], pose[1])

    def _set_magnet(self, on, force=False):
        if on == self._magnet and not force:
            return
        self._magnet = on
        self.pub_magnet.publish(Bool(data=on))
        self.node.get_logger().info(f'Magnet {"AN" if on else "AUS"}.')

    def _report(self, text):
        """Rueckmeldung fuer Log und debug_view."""
        self._message = text
        self.node.get_logger().info(text + '.')
        self._publish_state()

    def _publish_state(self):
        self.pub_state.publish(String(data=self.state.name))
        zones = ' '.join(z.label() for z in self._zones) or '-'
        info = f'Zonen {zones} | Ziel {self.target_zone}'
        if self._model is not None:
            info += f' | Modell {self._model["hits"]}/{ZONE_COUNT + 1} Linien, ' \
                    f'Breite {self._model["width"]:.2f} m'
        if self._attempts:
            info += f' | Versuch {self._attempts}'
        if self._message:
            info += f' | {self._message}'
        if self.state == S.FEHLER and self._reason:
            info += f' | {self._reason}'
        self.pub_info.publish(String(data=info))
        self._publish_zones()

    def _publish_zones(self):
        msg = StorageZones()
        msg.header.frame_id = ODOM_FRAME
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.lines, msg.measured, msg.digits = [], [], []
        if self._model is not None:
            c, s = math.cos(self._track_yaw), math.sin(self._track_yaw)
            yaw = self._model['yaw_in']
            for d, measured in zip(self._model['s'], self._model['measured']):
                pose = Pose()
                pose.position.x = self._origin[0] + d * c
                pose.position.y = self._origin[1] + d * s
                pose.orientation.z, pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
                msg.lines.append(pose)
                msg.measured.append(bool(measured))
            order = self._zone_order()
            for k in range(ZONE_COUNT):
                visited = next((z for z in self._zones if z.index == k), None)
                digit = order[k] if k < len(order) else (visited.digit if visited else None)
                msg.digits.append(int(digit or 0))
        self.pub_zones.publish(msg)
