"""Ablauf der Storage-Aufgabe (URMRC 2026, 4.2.2) als Teil von switch_control.

Kein eigener Node: switch_control legt StorageSequence an, damit es der einzige
Entscheider ueber /control/mode bleibt. Konzept und Zustandsdiagramm:
docs/Storage_Retrieval_Konzept.md.

Ein:  /control/storage_request (Bool, Button), /detect/zone_boundaries und
      /detect/zone_cross_lines (PoseArray, odom), /detect/storage_sign (StorageSign),
      /detect/pallet (PalletDetection), /control/maneuver_status (UInt32), TF
Aus:  /control/storage_state (String, Name aus StorageState), /control/storage_info
      (String, Klartext fuer debug_view), /control/maneuver (Maneuver),
      Magnet (Bool, Topic aus Parameter magnet_topic)
"""
import math
import time
from collections import Counter

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String, UInt32
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import Maneuver, PalletDetection, StorageSign
from Follow_Lane.src.common import (
    ControlType, StorageState, frange, irange, pose2d_from_transform, wrap_angle,
    yaw_from_quaternion)

S = StorageState

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'
TICK_PERIOD_S = 0.1
STATE_PUBLISH_PERIOD_S = 0.5
# Zustaende, die eine Folge von Grundbewegungen abfahren und dann weiterschalten.
MANEUVER_STATES = (S.ZUR_ZONE_DREHEN, S.ZURUECKDREHEN, S.ZUR_PALETTENZONE, S.GRIFF_PRUEFEN,
                   S.NEU_ANSETZEN, S.HERAUSFAHREN, S.TRANSPORT, S.ABLAGE_ANFAHREN, S.VERLASSEN)
# Mit Palette am Magneten: verschwindet sie aus dem Bild, ist sie verloren.
CARRY_STATES = (S.HERAUSFAHREN, S.TRANSPORT, S.ABLAGE_ANFAHREN)
INACTIVE_STATES = (S.IDLE, S.FERTIG, S.FEHLER)
# Zwei Querlinien naeher als das (m) zaehlen bei der Zonentiefe als eine.
CROSS_LINE_MERGE_M = 0.03
CROSS_LINE_MAX_M = 1.0
# So alt (s) darf die letzte Palettenerkennung beim Griff-Pruefen hoechstens sein.
PALLET_FRESH_S = 0.5


class Zone:
    """Eine bei der Erkundung besuchte Zone. track_xy/yaw_in: Pose auf der Spur vor der
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

    def label(self):
        text = str(self.digit) if self.digit else '?'
        return text + ('P' if self.occupied else '')


class StorageSequence:
    """Erkundung: an der Spur entlang (Lane). Zonengrenzen (Querlinien neben der Spur)
    werden in odom gesammelt und ueber die Strecke s entlang der Startrichtung
    gebuendelt; zwei bestaetigte Nachbargrenzen ergeben eine Zonenmitte. Erreicht der
    Bot sie, faehrt er genau hin, dreht zur Zone, liest Schild, Palette und Zonentiefe,
    dreht zurueck. Rueckfall, wenn der Bot fallback_step hinter der letzten bestaetigten
    Grenze (bzw. dem Start oder letzten Halt) steht, ohne eine neue gesehen zu haben: an
    der aktuellen Stelle zur Seite drehen und nur das Schild lesen.

    Sind Paletten- und Zielzone bekannt, faehrt er per odom vor die Palettenzone, haelt
    (Wertung), dockt im Bild an, greift, prueft durch kurzes Zuruecksetzen, faehrt
    rueckwaerts auf die Spur, an ihr entlang vor die Zielzone, bis zur Zonenmitte,
    legt ab und faehrt rueckwaerts zurueck auf die Spur.

    Palettenzone: die Zone, in der die Palette gesehen wurde; sonst die mit der Ziffer
    pallet_zone. Zielzone: Ziffer target_zone. zone_order (z.B. "3,2,1", Ziffern in
    Reihenfolge der Erkundung) ersetzt das Lesen der Schilder.
    """

    def __init__(self, node):
        self.node = node

        # --- Parameter ---
        p = node.declare_parameter
        p('target_zone', 2, irange(1, 3))
        p('pallet_zone', 1, irange(1, 3))
        p('zone_order', '')
        p('zone_count', 3, irange(1, 9))
        p('zones_right', True)
        # Zonengrenzen: Erkennungen naeher als boundary_merge_dist (m, entlang der Spur)
        # sind dieselbe Grenze; bestaetigt nach boundary_confirm Erkennungen.
        p('boundary_merge_dist', 0.05, frange(0.005, 0.3))
        p('boundary_confirm', 3, irange(1, 50))
        # Plausibilitaet, kein Streckenmass: die Palette (10 cm) muss hineinpassen.
        p('zone_min_width', 0.12, frange(0.0, 1.0))
        p('zone_min_depth', 0.08, frange(0.0, 1.0))
        # Rueckfall B (Strecke hinter der letzten Grenze); 0 = aus. Etwa eine Zonenbreite,
        # vor Ort einstellen.
        p('fallback_step', 0.35, frange(0.0, 3.0))
        # Sicherheitsnetz: nach so viel Strecke (m) endet die Erkundung.
        p('explore_max_distance', 2.5, frange(0.2, 10.0))
        # Schild lesen: erst settle s warten, dann read_time s abstimmen; ohne Ergebnis
        # spaetestens nach read_timeout weiter.
        p('sign_settle_time', 0.3, frange(0.0, 5.0))
        p('sign_read_time', 1.5, frange(0.1, 10.0))
        p('sign_read_timeout', 4.0, frange(0.1, 30.0))
        p('sign_min_votes', 3, irange(1, 100))
        p('sign_min_share', 0.6, frange(0.0, 1.0))
        p('pallet_min_ratio', 0.5, frange(0.0, 1.0))
        p('position_hold_time', 1.0, frange(0.0, 10.0))
        # Andocken: Kontakt nach contact_frames Bildern mit Luecke <= contact_gap_px
        # (gleich halten mit dock_contact_gap_px von control_driving). Ist die Luecke
        # unter dock_check_gap_px und der Versatz noch ueber dock_lateral_tol: neu ansetzen.
        p('contact_gap_px', 3.0, frange(-50.0, 50.0))
        p('contact_frames', 3, irange(1, 30))
        p('dock_check_gap_px', 30.0, frange(0.0, 500.0))
        p('dock_lateral_tol', 0.15, frange(0.0, 1.0))
        p('dock_timeout', 20.0, frange(1.0, 120.0))
        p('max_attempts', 3, irange(1, 10))
        p('magnet_on_wait', 1.0, frange(0.0, 10.0))
        p('magnet_off_wait', 1.0, frange(0.0, 10.0))
        # Griff pruefen: so weit zuruecksetzen (m); haengt die Palette noch am Magneten
        # (Luecke <= grip_ok_gap_px), hat er sie.
        p('grip_check_distance', 0.03, frange(0.005, 0.2))
        p('grip_ok_gap_px', 10.0, frange(-50.0, 200.0))
        # Beim Tragen: Luecke ueber pallet_lost_gap_px oder keine Palette, laenger als
        # pallet_lost_time -> verloren.
        p('pallet_lost_gap_px', 25.0, frange(0.0, 500.0))
        p('pallet_lost_time', 1.0, frange(0.1, 10.0))
        # Abstand Radachse -> Palettenmitte, wenn sie am Magneten haengt (m). Am Bot messen.
        p('pallet_center_offset', 0.09, frange(0.0, 0.3))
        p('carry_max_vel', 0.05, frange(0.01, 0.22))
        p('step_timeout', 30.0, frange(1.0, 300.0))
        p('magnet_topic', '/lokal/magnet_control')
        self._param_names = [
            'target_zone', 'pallet_zone', 'zone_order', 'zone_count', 'zones_right',
            'boundary_merge_dist', 'boundary_confirm', 'zone_min_width', 'zone_min_depth',
            'fallback_step', 'explore_max_distance', 'sign_settle_time', 'sign_read_time',
            'sign_read_timeout', 'sign_min_votes', 'sign_min_share', 'pallet_min_ratio',
            'position_hold_time', 'contact_gap_px', 'contact_frames', 'dock_check_gap_px',
            'dock_lateral_tol', 'dock_timeout', 'max_attempts', 'magnet_on_wait',
            'magnet_off_wait', 'grip_check_distance', 'grip_ok_gap_px', 'pallet_lost_gap_px',
            'pallet_lost_time', 'pallet_center_offset', 'carry_max_vel', 'step_timeout',
        ]
        self._read_params()

        # --- Subscriber ---
        node.create_subscription(Bool, '/control/storage_request', self.cb_request, 1)
        node.create_subscription(PoseArray, '/detect/zone_boundaries', self.cb_boundaries, 5)
        node.create_subscription(PoseArray, '/detect/zone_cross_lines', self.cb_cross_lines, 5)
        node.create_subscription(StorageSign, '/detect/storage_sign', self.cb_sign, 10)
        node.create_subscription(PalletDetection, '/detect/pallet', self.cb_pallet, 1)
        node.create_subscription(UInt32, '/control/maneuver_status', self.cb_maneuver_status, 1)

        # --- Publisher ---
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = node.create_publisher(String, '/control/storage_state', latched)
        self.pub_info = node.create_publisher(String, '/control/storage_info', latched)
        self.pub_maneuver = node.create_publisher(Maneuver, '/control/maneuver', 1)
        self.pub_magnet = node.create_publisher(
            Bool, node.get_parameter('magnet_topic').value, 1)

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
            self._add_boundary(self._s_of(p), yaw_from_quaternion(pose.orientation), seen)

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

    def cb_sign(self, msg):
        if self.state != S.SCHILD_LESEN or not self._reading():
            return
        if msg.digit > 0:
            self._sign_votes[msg.digit] += 1

    def cb_pallet(self, msg):
        now = time.monotonic()
        self._pallet = msg
        self._pallet_time = now

        if self.state == S.SCHILD_LESEN and self._reading():
            self._pallet_frames += 1
            self._pallet_found += int(msg.found)
        elif self.state == S.ANDOCKEN:
            self._check_dock(msg)
        elif self.state in CARRY_STATES and msg.found and msg.gap_px <= self.pallet_lost_gap_px:
            self._pallet_ok_time = now

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
        elif self.state == S.POSITIONIERT and elapsed >= self.position_hold_time:
            self._enter(S.ANDOCKEN)
        elif self.state == S.ANDOCKEN:
            if elapsed >= self.dock_timeout:
                self._retry('Andocken dauert zu lange')
            elif time.monotonic() - self._pallet_seen_time > self.pallet_lost_time:
                self._retry('Palette beim Andocken nicht im Bild')
        elif self.state == S.GREIFEN and elapsed >= self.magnet_on_wait:
            self._enter(S.GRIFF_PRUEFEN)
        elif self.state == S.ABLEGEN and elapsed >= self.magnet_off_wait:
            self._enter(S.VERLASSEN)

        if self.state in CARRY_STATES \
                and time.monotonic() - self._pallet_ok_time > self.pallet_lost_time:
            self._fail('Palette nicht mehr am Magneten')
        elif self.state in MANEUVER_STATES and elapsed >= self.step_timeout:
            self._fail(f'{self.state.name} dauert laenger als {self.step_timeout:.0f} s')

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
        self._origin = (pose[0], pose[1])
        self._track_yaw = pose[2]
        self.node.get_logger().info(
            f'Storage gestartet, Ziel Zone {self.target_zone}'
            + (f', Zonen von Hand: {self.zone_order}' if self.zone_order else '') + '.')
        self._enter(S.ERKUNDEN)

    def _abort(self):
        self._set_magnet(False, force=True)
        self._queue = []
        self._enter(S.IDLE)
        self.node.set_initial_mode('Storage abgebrochen')

    def _fail(self, reason):
        self.node.get_logger().error(f'Storage FEHLER: {reason}')
        self._reason = reason
        self._enter(S.FEHLER)

    def _retry(self, reason):
        self._attempts += 1
        self.node.get_logger().warn(f'{reason} -- Versuch {self._attempts}/{self.max_attempts}.')
        if self._attempts >= self.max_attempts:
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
        elif state == S.ZURUECKDREHEN:
            self._queue = [self._rotate(self._track_yaw)]
        elif state == S.ZUR_PALETTENZONE:
            z = self._pallet_zone
            self._queue = [self._path_to(z.track_xy), self._rotate(z.yaw_in)]
        elif state == S.ANDOCKEN:
            mode = ControlType.Dock
            self._contact = 0
            self._pallet_seen_time = time.monotonic()
        elif state == S.GREIFEN:
            self._contact_xy = self._robot_xy()
            self._set_magnet(True)
        elif state == S.GRIFF_PRUEFEN:
            self._queue = [self._straight(-self.grip_check_distance)]
        elif state == S.NEU_ANSETZEN:
            self._set_magnet(False)
            z = self._pallet_zone
            self._queue = [self._path_to(z.track_xy), self._rotate(z.yaw_in)]
        elif state == S.HERAUSFAHREN:
            self._pallet_ok_time = time.monotonic()
            self._queue = [self._path_to(self._pallet_zone.track_xy, max_vel=self.carry_max_vel)]
        elif state == S.TRANSPORT:
            src, dst = self._pallet_zone.track_xy, self._target_zone.track_xy
            heading = math.atan2(dst[1] - src[1], dst[0] - src[0])
            self._queue = [self._rotate(heading, max_vel=self.carry_max_vel),
                           self._path_to(dst, reverse=False, max_vel=self.carry_max_vel),
                           self._rotate(self._target_zone.yaw_in, max_vel=self.carry_max_vel)]
        elif state == S.ABLAGE_ANFAHREN:
            self._queue = [self._path_to(self._drop_point(), reverse=False,
                                         max_vel=self.carry_max_vel)]
        elif state == S.ABLEGEN:
            self._set_magnet(False)
        elif state == S.VERLASSEN:
            self._queue = [self._path_to(self._target_zone.track_xy, reverse=True)]
        elif state == S.FEHLER:
            self._set_magnet(False, force=True)

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
            self._last_stop_s = self._s_of(robot) if robot else self._last_stop_s
            if self._plan():
                self._enter(S.ZUR_PALETTENZONE)
            elif sum(1 for z in self._zones if not z.fallback) >= self.zone_count:
                self._finish_exploration()
            else:
                self._enter(S.ERKUNDEN)
        elif self.state == S.ZUR_PALETTENZONE:
            self._enter(S.POSITIONIERT)
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

    # --- Erkundung ---

    def _tick_explore(self):
        robot = self._robot_pose()
        if robot is None:
            return
        s = self._s_of(robot)

        center = self._next_zone_center(s)
        if center is not None:
            s_center, yaw_in = center
            # Punkt auf der Spur auf Hoehe der Zonenmitte (seitliche Lage des Bots bleibt).
            d = s_center - s
            xy = (robot[0] + d * math.cos(self._track_yaw), robot[1] + d * math.sin(self._track_yaw))
            self._zones.append(Zone(len(self._zones), s_center, xy, yaw_in, fallback=False))
            self._enter(S.ZUR_ZONE_DREHEN)
            return

        if self.fallback_step > 0.0 \
                and s - max(self._last_stop_s, self._last_boundary_s) >= self.fallback_step:
            self.node.get_logger().info('Rueckfall: keine Zonengrenze erkannt, drehe zur Seite.')
            self._zones.append(Zone(len(self._zones), s, (robot[0], robot[1]),
                                    self._side_yaw(), fallback=True))
            self._enter(S.ZUR_ZONE_DREHEN)
            return

        if s >= self.explore_max_distance:
            self._finish_exploration()

    def _add_boundary(self, s, yaw, seen):
        for b in self._boundaries:
            if abs(b['s'] - s) <= self.boundary_merge_dist:
                if id(b) in seen:
                    return
                seen.add(id(b))
                n = b['n']
                b['s'] = (b['s'] * n + s) / (n + 1)
                # Richtung als Einheitsvektor mitteln, sonst springt der Mittelwert bei +-pi.
                b['dir'] = (b['dir'][0] + math.cos(yaw), b['dir'][1] + math.sin(yaw))
                b['n'] = n + 1
                if b['n'] == self.boundary_confirm:
                    # Die Grenze selbst, nicht die Bot-Position: die BEV sieht bis 0,6 m voraus.
                    self._last_boundary_s = max(self._last_boundary_s, b['s'])
                    self.node.get_logger().info(f'Zonengrenze bestaetigt bei s={b["s"]:.2f} m.')
                return
        b = {'s': s, 'n': 1, 'dir': (math.cos(yaw), math.sin(yaw))}
        seen.add(id(b))
        self._boundaries.append(b)

    def _next_zone_center(self, s_robot):
        """(s, yaw_in) der naechsten nicht besuchten Zonenmitte, die der Bot erreicht hat."""
        confirmed = sorted((b for b in self._boundaries if b['n'] >= self.boundary_confirm),
                           key=lambda b: b['s'])
        for a, b in zip(confirmed, confirmed[1:]):
            if b['s'] - a['s'] < self.zone_min_width:
                continue
            s_center = (a['s'] + b['s']) / 2.0
            if any(abs(z.s_center - s_center) < self.zone_min_width / 2.0 for z in self._zones):
                continue
            if s_robot < s_center:
                return None
            dx, dy = a['dir'][0] + b['dir'][0], a['dir'][1] + b['dir'][1]
            yaw_in = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else self._side_yaw()
            # Nur Grenzen, die ungefaehr quer zur Spur liegen, geben die Richtung vor.
            if abs(wrap_angle(yaw_in - self._side_yaw())) > math.radians(30.0):
                yaw_in = self._side_yaw()
            return s_center, yaw_in
        return None

    def _tick_read(self, elapsed):
        if elapsed < self.sign_settle_time + self.sign_read_time and elapsed < self.sign_read_timeout:
            return
        z = self._zones[-1]
        order = self._zone_order()
        by_hand = order[z.index] if z.index < len(order) else None
        digit = self._vote_digit()
        if digit is None and by_hand is None and elapsed < self.sign_read_timeout:
            return

        robot = self._robot_pose()
        if robot is not None:
            z.track_xy = (robot[0], robot[1])
            z.yaw_in = robot[2]
        z.digit = by_hand if by_hand is not None else digit
        if self._pallet_frames > 0:
            ratio = self._pallet_found / self._pallet_frames
            z.occupied = ratio if ratio >= self.pallet_min_ratio else 0.0
        z.depth = self._zone_depth()
        self.node.get_logger().info(
            f'Zone {z.index}: Ziffer {z.digit or "?"} (Stimmen {dict(self._sign_votes)}), '
            f'Palette {"ja" if z.occupied else "nein"} ({self._pallet_found}/{self._pallet_frames}), '
            f'Tiefe {"?" if z.depth is None else f"{z.depth:.2f} m"}.')

        if z.fallback and z.digit is None:
            self.node.get_logger().info('Rueckfall: kein Schild gelesen, Stelle verworfen.')
            self._zones.pop()
        elif z.fallback and any(o.digit == z.digit for o in self._zones[:-1]):
            self.node.get_logger().info(f'Rueckfall: Schild {z.digit} schon bekannt, verworfen.')
            self._zones.pop()
        self._enter(S.ZURUECKDREHEN)

    def _vote_digit(self):
        total = sum(self._sign_votes.values())
        if total == 0:
            return None
        digit, votes = self._sign_votes.most_common(1)[0]
        if votes >= self.sign_min_votes and votes / total >= self.sign_min_share:
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
        if len(lines) < 2 or lines[-1] - lines[0] < self.zone_min_depth:
            return None
        return (lines[0] + lines[-1]) / 2.0

    def _plan(self):
        """Paletten- und Zielzone bestimmen; True, wenn beide bekannt und verschieden."""
        known = [z for z in self._zones if z.track_xy is not None]
        occupied = [z for z in known if z.occupied]
        if occupied:
            self._pallet_zone = max(occupied, key=lambda z: z.occupied)
        else:
            self._pallet_zone = next((z for z in known if z.digit == self.pallet_zone), None)
        self._target_zone = next((z for z in known if z.digit == self.target_zone), None)
        ok = self._pallet_zone is not None and self._target_zone is not None \
            and self._pallet_zone is not self._target_zone
        if ok:
            self.node.get_logger().info(
                f'Palette in Zone {self._pallet_zone.label()}, Ziel Zone {self._target_zone.label()}.')
        return ok

    def _finish_exploration(self):
        if self._plan():
            self._enter(S.ZUR_PALETTENZONE)
            return
        if self._pallet_zone is None:
            self._fail('Palette nicht gefunden (weder gesehen noch Schild '
                       f'{self.pallet_zone} gelesen)')
        elif self._target_zone is None:
            self._fail(f'Zielzone {self.target_zone} nicht gefunden')
        else:
            self._fail(f'Palette liegt schon in der Zielzone {self.target_zone}')

    # --- Andocken und Tragen ---

    def _check_dock(self, msg):
        if not msg.found:
            self._contact = 0
            return
        self._pallet_seen_time = time.monotonic()
        if msg.gap_px <= self.contact_gap_px:
            self._contact += 1
            if self._contact >= self.contact_frames:
                self._enter(S.GREIFEN)
            return
        self._contact = 0
        if msg.gap_px <= self.dock_check_gap_px and abs(msg.lateral) > self.dock_lateral_tol:
            self._retry(f'Seitlicher Versatz {msg.lateral:+.2f} kurz vor der Palette')

    def _check_grip(self):
        msg = self._pallet
        fresh = msg is not None and time.monotonic() - self._pallet_time <= PALLET_FRESH_S
        if fresh and msg.found and msg.gap_px <= self.grip_ok_gap_px:
            self._enter(S.HERAUSFAHREN)
        else:
            self._retry('Palette kommt beim Zuruecksetzen nicht mit')

    def _drop_point(self):
        """Ablagepunkt der Radachse: Palettenmitte auf der Zonenmitte der Zielzone. Ohne
        gemessene Tiefe so tief wie beim Aufnehmen (Palette lag in der Mitte der Zone)."""
        z = self._target_zone
        if z.depth is not None:
            dist = z.depth - self.pallet_center_offset
        else:
            dist = math.dist(self._pallet_zone.track_xy, self._contact_xy) \
                if self._contact_xy is not None else 0.0
            self.node.get_logger().warn(
                f'Tiefe der Zielzone unbekannt -- Ablage so tief wie beim Aufnehmen ({dist:.2f} m).')
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
        self._origin = (0.0, 0.0)
        self._track_yaw = 0.0
        self._boundaries = []
        self._zones = []
        self._last_stop_s = 0.0
        self._last_boundary_s = 0.0
        self._pallet_zone = self._target_zone = None
        self._attempts = 0
        self._contact = 0
        self._contact_xy = None
        self._pallet = None
        self._pallet_time = 0.0
        self._pallet_seen_time = 0.0
        self._pallet_ok_time = 0.0
        self._sign_votes = Counter()
        self._pallet_frames = self._pallet_found = 0
        self._cross_lines = []
        self._reason = ''

    def _reading(self):
        return time.monotonic() - self._since >= self.sign_settle_time

    def _zone_order(self):
        try:
            return [int(t) for t in str(self.zone_order).replace(' ', '').split(',') if t]
        except ValueError:
            self.node.get_logger().warn(f'zone_order "{self.zone_order}" nicht lesbar.',
                                        throttle_duration_sec=10.0)
            return []

    def _side_yaw(self):
        return self._track_yaw + (-math.pi / 2 if self.zones_right else math.pi / 2)

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

    def _publish_state(self):
        self.pub_state.publish(String(data=self.state.name))
        zones = ' '.join(z.label() for z in self._zones) or '-'
        info = f'Zonen {zones} | Ziel {self.target_zone}'
        if self._attempts:
            info += f' | Versuch {self._attempts}'
        if self.state == S.FEHLER and self._reason:
            info += f' | {self._reason}'
        self.pub_info.publish(String(data=info))
