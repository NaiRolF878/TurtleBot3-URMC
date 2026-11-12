"""Einzige Quelle fuer Fahrbefehle: waehlt je nach /control/mode den Regler.

Ein:  /control/mode (UInt8), /detect/lane (Float64, Spurfehler), /nav2/cmd_vel (Twist),
      /control/maneuver (Maneuver), /local_costmap/costmap (Waechter), TF odom -> base_footprint
Aus:  /control/cmd_vel (Twist) -> notaus -> /cmd_vel,
      /control/maneuver_status (ManeuverStatus, zuletzt beendete Grundbewegung),
      /control/guard (Bool, latched: Waechter haelt gerade an)
Lane/TunnelEntry: PID auf den Spurfehler. Obstacle/Tunnel: /nav2/cmd_vel.
Obstacle/Tunnel mit laufender Grundbewegung in odom (Engstelle, Freifahren): nur diese.
Alles ausser dem PID wird auf die Radgrenze skaliert, der PID begrenzt selbst.
Align: auf der Stelle zum Kurs /control/align_yaw (switch_control), Ist-Kurs aus
/odom_filtered.
Waechter (Obstacle, Tunnel, Align): jeder Befehl wird GUARD_HORIZON_S voraus mit dem Umriss
gegen die local_costmap geprueft; stoesst er an, geht Null hinaus.
Jeder andere Modus, unbekannter Modus oder veraltete Quelle: Nullbefehl.
"""
import math
import time

import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import Maneuver, ManeuverStatus
from Follow_Lane.src.common import (
    ControlType, control_mode_from_msg, frange, pose2d_from_transform, read_params,
    require_params, run_node, wrap_angle, yaw_from_quaternion)
from Follow_Lane.src.robot_geometry import choose_unwedge, motion_collides, simulate

# TunnelEntry mit PID: auf der 20-cm-Spur vor dem Tunnel passt der Bot fuer Nav2
# nicht zwischen die Spurwaende.
PID_MODES = (ControlType.Lane, ControlType.TunnelEntry)
NAV2_MODES = (ControlType.Obstacle, ControlType.Tunnel)
# Hier hat eine laufende Grundbewegung Vorrang vor Nav2; der Modus bleibt (im Tunnel keine Kamera).
MANEUVER_MODES = (ControlType.Obstacle, ControlType.Tunnel)

# Waechter: so weit (s) voraus wird jeder Befehl geprueft (wie HKA). Nur in diesen Modi: im Lane-
# Modus hielte er an Dingen hinter der Spurlinie. Eine Karte aelter als GUARD_MAP_MAX_AGE_S
# taugt nicht, dann Null. Haelt er eine Grundbewegung GUARD_MANEUVER_FAIL_S lang an, gilt sie
# als abgebrochen.
GUARD_MODES = (ControlType.Obstacle, ControlType.Tunnel, ControlType.Align)
GUARD_HORIZON_S = 0.6
GUARD_MAP_MAX_AGE_S = 1.0
GUARD_MANEUVER_FAIL_S = 1.0

# Align (nach Obstacle): auf der Stelle zum Zielkurs von switch_control (/control/align_yaw,
# odom) drehen, Ist-Kurs aus /odom_filtered. P-Regler, mindestens ALIGN_MIN_VEL (sonst
# bleibt der Burger vor dem Ziel stehen), hoechstens ALIGN_MAX_VEL; ab ALIGN_TOL_RAD Null.
# switch_control schaltet bei 5 Grad Rest auf Lane.
ALIGN_KP = 2.0
ALIGN_MIN_VEL = 0.3
ALIGN_MAX_VEL = 1.2
ALIGN_TOL_RAD = math.radians(3.0)
ALIGN_INPUT_TIMEOUT = 1.0

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'

# Prueft, ob die aktive Quelle noch liefert, und regelt Grundbewegungen; = controller_frequency
# in nav2_params.yaml.
WATCHDOG_RATE_HZ = 20.0

# Feste Abtastzeit fuer D- und I-Anteil: die Bildrate schwankt, kp/ki/kd sind auf diesen Wert
# eingestellt.
PID_SAMPLE_TIME_S = 0.1
INTEGRAL_LIMIT = 1.0
# Begrenzung der PID-Lenkung (rad/s); der Burger schafft ~2,84 rad/s.
MAX_ANGULAR_VEL = 2.0
# Hoechstgeschwindigkeit des Burger; das aeussere Rad (v + |omega| * WHEEL_SEPARATION / 2)
# bleibt darunter.
MAX_WHEEL_VEL = 0.22
WHEEL_SEPARATION = 0.160

# Zeitgrenzen (s): LANE_CMD_TIMEOUT umfasst mehrere Kamerabilder,
# NAV2_CMD_TIMEOUT fuenf Perioden von controller_frequency (20 Hz), MODE_TIMEOUT drei
# Perioden von switch_control (2 Hz).
LANE_CMD_TIMEOUT = 0.3
NAV2_CMD_TIMEOUT = 0.25
MODE_TIMEOUT = 1.5
MODE_SWITCH_HOLD = 0.3

STOP_REPEAT = 10
STOP_REPEAT_PERIOD_S = 0.05

# Grundbewegung: darunter bewegt sich der Burger nicht sicher (m/s, rad/s).
MANEUVER_MIN_VEL = 0.02
MANEUVER_MIN_ANG = 0.3
# Fertig, wenn so nah am Ziel; Zwischenpunkte eines Pfads gelten ab MANEUVER_WAYPOINT_TOL als
# erreicht, ohne dort zu halten.
MANEUVER_DIST_TOL = 0.01
MANEUVER_YAW_TOL_DEG = 2.0
MANEUVER_WAYPOINT_TOL = 0.03
# Liegt der naechste Pfadpunkt weiter als das neben der Fahrtrichtung, erst auf der Stelle drehen.
MANEUVER_ROTATE_FIRST_DEG = 30.0
# Sicherungen (Lauf 11.11.: Bot drehte sich endlos, die Odometrie meldete die Drehung nicht):
# aendert sich die Pose so lange (s) nicht, ist sie veraltet -> anhalten; fehlt sie so lange
# (s) waehrend einer Grundbewegung, gilt diese als abgebrochen ("Pose veraltet"), damit der
# Auftraggeber sofort reagiert statt auf die Zeitgrenze von switch_control zu warten.
MANEUVER_POSE_STALE_S = 0.5
MANEUVER_POSE_FAIL_S = 1.0
# Drehung abbrechen, wenn der Bot so viel (Grad) mehr gedreht haette als geplant
# (aufsummierte Drehbefehle, unabhaengig von der Odometrie).
MANEUVER_ROTATE_EXTRA_DEG = 90.0
# Pfad: liegt das Ziel naeher (m), wird nicht mehr gedreht, nur noch gerade gefahren.
MANEUVER_NEAR_M = 0.05


def clamp(x, limit):
    return max(-limit, min(limit, x))


class control_driving(Node):
    """Vorzeichen: lane_error > 0 heisst Spurmitte links der Bildmitte, angular.z > 0
    heisst nach links drehen (REP 103).

    Nach jedem Wechsel des Reglers zaehlen nur Befehle, die danach eintreffen; der
    Wechsel selbst sendet sofort Null, damit turtlebot3_node nicht den letzten Befehl
    der alten Quelle weiterfaehrt. Lane <-> TunnelEntry behaelt den PID ohne Halt.
    Nach dem Wechsel in den Tunnel bleibt die Nav2-Quelle MODE_SWITCH_HOLD lang gesperrt.

    Grundbewegung (nur in MANEUVER_MODES): switch_control sendet sie zyklisch auf
    /control/maneuver. Eine neue id startet sie ab der aktuellen Pose; solange sie laeuft,
    zaehlt /nav2/cmd_vel nicht. Ist sie fertig oder abgebrochen (Sicherung, Waechter, Pose
    veraltet, kein freier Ausweg), meldet dieser Node das auf /control/maneuver_status, sendet Null und folgt
    wieder Nav2. Ein Moduswechsel verwirft die laufende Bewegung. Freifahren (UNWEDGE) waehlt
    beim Start die freieste kleine Bewegung (robot_geometry.choose_unwedge) und faehrt sie.
    """

    def __init__(self):
        super().__init__('control_driving')

        # --- Parameter ---
        # Ohne Default: nur aus der Control-YAML des Launches, sonst startet der Node nicht.
        self.declare_parameter('kp',                 Parameter.Type.DOUBLE, frange(0.0, 20.0))
        self.declare_parameter('ki',                 Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self.declare_parameter('kd',                 Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self.declare_parameter('MAX_VEL',            Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self.declare_parameter('speed_curve_factor', Parameter.Type.DOUBLE, frange(0.0, 5.0))
        self.declare_parameter('min_speed_factor',   Parameter.Type.DOUBLE, frange(0.0, 10.0))
        # Grundbewegung: Geschwindigkeit proportional zum Rest (k_lin, k_ang), nach oben begrenzt.
        self.declare_parameter('maneuver_max_vel',   Parameter.Type.DOUBLE, frange(0.01, 0.22))
        self.declare_parameter('maneuver_max_ang',   Parameter.Type.DOUBLE, frange(0.1, 2.8))
        self.declare_parameter('maneuver_k_lin',     Parameter.Type.DOUBLE, frange(0.1, 10.0))
        self.declare_parameter('maneuver_k_ang',     Parameter.Type.DOUBLE, frange(0.1, 10.0))
        self._param_names = [
            'kp', 'ki', 'kd', 'MAX_VEL', 'speed_curve_factor', 'min_speed_factor',
            'maneuver_max_vel', 'maneuver_max_ang', 'maneuver_k_lin', 'maneuver_k_ang',
        ]
        require_params(self, self._param_names)
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_lane_error = self.create_subscription(
            Float64, '/detect/lane', self.cb_lane_error, 1)
        self.sub_nav2_cmd_vel = self.create_subscription(
            Twist, '/nav2/cmd_vel', self.cb_nav2_cmd_vel, 1)
        self.sub_align_yaw = self.create_subscription(
            Float64, '/control/align_yaw', self.cb_align_yaw, 1)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom_filtered', self.cb_odom, 1)
        self.sub_maneuver = self.create_subscription(
            Maneuver, '/control/maneuver', self.cb_maneuver, 1)
        self.sub_costmap = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.cb_costmap, 1)

        # --- Publisher ---
        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)
        self.pub_maneuver_status = self.create_publisher(
            ManeuverStatus, '/control/maneuver_status', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_guard = self.create_publisher(Bool, '/control/guard', latched)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / WATCHDOG_RATE_HZ, self.cb_watchdog)

        # --- Zustand ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # None bis zur ersten /control/mode: ohne switch_control wird nicht gefahren.
        self.control_mode = None
        self._mode_time = None
        self._lane_time = None
        self._nav2_time = None
        self._nav2_blocked_until = 0.0
        self._align_yaw = None          # (Zielkurs rad, Empfangszeit)
        self._yaw = None                # (Ist-Kurs rad, Empfangszeit)
        self._maneuver = None           # laufende Grundbewegung (Maneuver-Nachricht)
        self._maneuver_start = None     # (x, y, yaw) in odom beim Start
        self._waypoint = 0              # PATH: Index des naechsten Punkts
        self._last_status = None        # ManeuverStatus der zuletzt beendeten Bewegung
        self._turned = 0.0              # ROTATE: aufsummierte Drehbefehle (rad)
        self._turn_time = None
        self._pose_stamp = None         # letzter Zeitstempel der odom-TF ...
        self._pose_changed = None       # ... und wann (monotonic) er sich zuletzt geaendert hat
        self._pose_lost_since = None    # Grundbewegung ohne gueltige Pose seit (monotonic)
        self._costmap = None            # (Zellen (H, W), Aufloesung, Ursprung, Empfang)
        self._guard_active = None       # zuletzt auf /control/guard gesendet
        self._guard_since = None        # Waechter haelt seit (monotonic)
        self._reset_pid()
        self._set_guard(False)

        self.get_logger().info('Started control_driving Node.')

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None:
            return
        now = time.monotonic()
        self._mode_time = now
        if new_mode == self.control_mode:
            return

        read_params(self, self._param_names)
        old_mode = self.control_mode
        self.control_mode = new_mode
        if old_mode in PID_MODES and new_mode in PID_MODES:
            return
        self._lane_time = None
        self._nav2_time = None
        self._maneuver = None
        self._maneuver_start = None
        if new_mode in PID_MODES:
            self._reset_pid()
        if new_mode == ControlType.Tunnel:
            self._nav2_blocked_until = now + MODE_SWITCH_HOLD
        self._publish_stop()

    def cb_lane_error(self, msg):
        if not self._mode_is(*PID_MODES):
            return
        read_params(self, self._param_names)

        now = time.monotonic()
        # Nach einer Luecke ist lastError Sekunden alt und der D-Anteil gaebe
        # einen Lenkschlag.
        if not self._fresh(self._lane_time, now, LANE_CMD_TIMEOUT):
            self._reset_pid()
        self._lane_time = now
        self.pub_cmd_vel.publish(self._pid(msg.data))

    def cb_nav2_cmd_vel(self, msg):
        if not self._mode_is(*NAV2_MODES):
            return
        now = time.monotonic()
        if now < self._nav2_blocked_until or self._maneuver is not None:
            return
        self._nav2_time = now
        self._publish_guarded(self._limit_wheels(msg))

    def cb_costmap(self, grid):
        info = grid.info
        cells = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
        self._costmap = (cells, info.resolution,
                         (info.origin.position.x, info.origin.position.y), time.monotonic())

    def cb_align_yaw(self, msg):
        self._align_yaw = (msg.data, time.monotonic())

    def cb_odom(self, msg):
        self._yaw = (yaw_from_quaternion(msg.pose.pose.orientation), time.monotonic())

    def _align(self, now):
        """Twist fuer Align oder None, wenn Ziel- oder Ist-Kurs fehlen bzw. veraltet sind."""
        if not (self._align_yaw and self._yaw
                and self._fresh(self._align_yaw[1], now, ALIGN_INPUT_TIMEOUT)
                and self._fresh(self._yaw[1], now, ALIGN_INPUT_TIMEOUT)):
            return None
        err = self._align_yaw[0] - self._yaw[0]
        err = math.atan2(math.sin(err), math.cos(err))
        twist = Twist()
        if abs(err) > ALIGN_TOL_RAD:
            speed = min(ALIGN_MAX_VEL, max(ALIGN_MIN_VEL, ALIGN_KP * abs(err)))
            twist.angular.z = float(math.copysign(speed, err))
        return twist

    def cb_maneuver(self, msg):
        if not self._mode_is(*MANEUVER_MODES):
            return
        # Zyklisch wiederholt: nur eine neue id (oder die erste nach einem
        # Moduswechsel) startet die Bewegung, eine beendete nie erneut.
        if self._last_status is not None and msg.id == self._last_status.id:
            return
        if self._maneuver is not None and msg.id == self._maneuver.id:
            return
        self._maneuver = msg
        self._maneuver_start = None
        self._waypoint = 0
        self._turned = 0.0
        self._turn_time = None
        self._pose_lost_since = None

    def cb_watchdog(self):
        read_params(self, self._param_names)
        # Zyklisch, falls switch_control die Meldung verpasst hat.
        if self._last_status is not None:
            self.pub_maneuver_status.publish(self._last_status)

        now = time.monotonic()
        if self._mode_is(ControlType.Align):
            twist = self._align(now)
            if twist is None:
                self.get_logger().warn('Align ohne Zielkurs oder /odom_filtered -- Null.',
                                       throttle_duration_sec=2.0)
                self._publish_stop()
            else:
                self._publish_guarded(twist)
            return
        if self._maneuver is not None and self._mode_is(*MANEUVER_MODES):
            self._drive_maneuver()
            return
        if self._mode_is(*PID_MODES):
            fresh = self._fresh(self._lane_time, now, LANE_CMD_TIMEOUT)
        elif self._mode_is(*NAV2_MODES):
            fresh = self._fresh(self._nav2_time, now, NAV2_CMD_TIMEOUT)
        else:
            fresh = False
        if not fresh:
            self._publish_stop()
        # Ohne geprueften Befehl haelt der Waechter auch nichts an.
        if not fresh or not self._mode_is(*GUARD_MODES):
            self._set_guard(False)

    # --- Hilfsmethoden ---

    def _mode_is(self, *modes):
        """Aktueller Modus ist einer von modes und /control/mode ist nicht veraltet."""
        if self.control_mode not in modes:
            return False
        if not self._fresh(self._mode_time, time.monotonic(), MODE_TIMEOUT):
            self.get_logger().warn(
                f'Seit {MODE_TIMEOUT:.1f} s kein /control/mode -- laeuft switch_control? '
                'Sende Null.', throttle_duration_sec=5.0)
            return False
        return True

    @staticmethod
    def _fresh(stamp, now, timeout):
        return stamp is not None and now - stamp <= timeout

    def _reset_pid(self):
        self.integral = 0.0
        self.last_error = 0.0

    def _pid(self, error):
        self.integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, self.integral + error))
        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * (error - self.last_error) / PID_SAMPLE_TIME_S
        self.last_error = error

        angular = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, p + i + d))
        linear = self.MAX_VEL * max(self.min_speed_factor, 1 - abs(error) * self.speed_curve_factor)
        # Lenkung hat Vorrang; die Radgrenze gilt auch vor min_speed_factor.
        linear = max(0.0, min(linear, MAX_WHEEL_VEL - abs(angular) * WHEEL_SEPARATION / 2))

        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        return twist

    @staticmethod
    def _limit_wheels(twist):
        """Skaliert v und omega gemeinsam, bis das aeussere Rad MAX_WHEEL_VEL einhaelt.
        Die Kruemmung bleibt, der Bot faehrt die geplante Bahn nur langsamer."""
        outer = abs(twist.linear.x) + abs(twist.angular.z) * WHEEL_SEPARATION / 2
        if outer > MAX_WHEEL_VEL:
            scale = MAX_WHEEL_VEL / outer
            twist.linear.x *= scale
            twist.angular.z *= scale
        return twist

    def _drive_maneuver(self):
        m = self._maneuver
        if m is None:
            self._publish_stop()
            return
        pose = self._robot_pose()
        if pose is None:
            self._publish_stop()
            now = time.monotonic()
            if self._pose_lost_since is None:
                self._pose_lost_since = now
            elif now - self._pose_lost_since >= MANEUVER_POSE_FAIL_S:
                self.get_logger().warn(f'Grundbewegung {m.id} abgebrochen: Pose veraltet.')
                self._finish_maneuver(m, False, 'Pose veraltet (Odometrie?)')
            return
        self._pose_lost_since = None
        if m.type == Maneuver.UNWEDGE:
            m = self._start_unwedge(m, pose)
            if m is None:
                return
        if self._maneuver_start is None:
            self._maneuver_start = pose

        if m.type == Maneuver.ROTATE:
            twist, done = self._rotate(pose, m.yaw)
            planned = abs(wrap_angle(m.yaw - self._maneuver_start[2]))
            if not done and self._rotation_runaway(m, twist, planned):
                return
        elif m.type == Maneuver.STRAIGHT:
            twist, done = self._straight(pose, m)
        elif m.type == Maneuver.PATH:
            twist, done = self._path(pose, m)
            # Ein Pfad dreht hoechstens einmal halb herum.
            if not done and self._rotation_runaway(m, twist, math.pi):
                return
        else:
            self.get_logger().warn(f'Unbekannte Grundbewegung {m.type} -- gilt als fertig.')
            twist, done = Twist(), True

        if done:
            self.get_logger().info(f'Grundbewegung {m.id} fertig.')
            self._finish_maneuver(m, True, '')
            return
        if not self._publish_guarded(self._limit_wheels(twist))                 and time.monotonic() - self._guard_since >= GUARD_MANEUVER_FAIL_S:
            self.get_logger().warn(f'Grundbewegung {m.id} abgebrochen: Waechter haelt an.')
            self._finish_maneuver(m, False, 'Waechter: Kollision voraus')

    def _start_unwedge(self, m, pose):
        """Freifahren: freieste kleine Bewegung als STRAIGHT/ROTATE mit derselben id, None (und
        abgebrochen), wenn es keine gibt."""
        grid = self._fresh_costmap()
        move = choose_unwedge(grid[0], grid[1], grid[2], pose) if grid else None
        if move is None:
            reason = 'kein freier Ausweg' if grid else 'keine aktuelle local_costmap'
            self.get_logger().warn(f'Freifahren nicht moeglich: {reason}.')
            self._finish_maneuver(m, False, reason)
            return None
        kind, amount = move
        m.type = Maneuver.STRAIGHT if kind == 'straight' else Maneuver.ROTATE
        m.distance = float(amount) if kind == 'straight' else 0.0
        m.yaw = float(wrap_angle(pose[2] + amount)) if kind == 'rotate' else 0.0
        self.get_logger().info(f'Freifahren: {"geradeaus" if kind == "straight" else "drehen"} '
                               f'{amount:+.2f}.')
        return m

    def _finish_maneuver(self, m, success, reason):
        self._last_status = ManeuverStatus(id=m.id, active=False, success=success, reason=reason)
        self._maneuver = None
        self.pub_maneuver_status.publish(self._last_status)
        self._publish_stop()

    def _rotation_runaway(self, m, twist, planned):
        """True (und Grundbewegung abgebrochen), wenn die Drehbefehle schon deutlich mehr als
        die geplante Drehung (rad) ergeben: der Bot dreht sich im Kreis."""
        now = time.monotonic()
        if self._turn_time is not None:
            self._turned += abs(twist.angular.z) * min(now - self._turn_time, 0.5)
        self._turn_time = now
        if self._turned <= planned + math.radians(MANEUVER_ROTATE_EXTRA_DEG):
            return False
        self.get_logger().error(
            f'Grundbewegung {m.id} abgebrochen: {math.degrees(self._turned):.0f} Grad gedreht, '
            f'geplant hoechstens {math.degrees(planned):.0f} -- dreht sich im Kreis. Bot steht.')
        self._finish_maneuver(m, False, 'dreht sich im Kreis (Odometrie?)')
        return True

    # --- Waechter ---

    def _publish_guarded(self, twist):
        """Sendet twist, wenn der Umriss GUARD_HORIZON_S voraus nirgends neu anstoesst, sonst
        Null. True, wenn twist hinausging."""
        if not self._mode_is(*GUARD_MODES) or (twist.linear.x == 0.0 and twist.angular.z == 0.0):
            self._set_guard(False)
            self.pub_cmd_vel.publish(twist)
            return True
        grid = self._fresh_costmap()
        pose = self._guard_pose()
        if grid is None or pose is None:
            self.get_logger().warn('Waechter ohne aktuelle local_costmap oder Pose -- Null.',
                                   throttle_duration_sec=2.0)
            blocked = True
        else:
            cells, res, origin, _ = grid
            poses = simulate(pose, twist.linear.x, twist.angular.z, GUARD_HORIZON_S, res)
            blocked = motion_collides(cells, res, origin, poses)
        self._set_guard(blocked)
        if blocked:
            self.get_logger().warn('Waechter: Umriss stiesse an -- Null.', throttle_duration_sec=1.0)
            self._publish_stop()
            return False
        self.pub_cmd_vel.publish(twist)
        return True

    def _set_guard(self, active):
        if active and self._guard_since is None:
            self._guard_since = time.monotonic()
        elif not active:
            self._guard_since = None
        if active != self._guard_active:
            self._guard_active = active
            self.pub_guard.publish(Bool(data=active))

    def _fresh_costmap(self):
        if self._costmap is None or time.monotonic() - self._costmap[3] > GUARD_MAP_MAX_AGE_S:
            return None
        return self._costmap

    def _guard_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        return pose2d_from_transform(tf)

    def _max_vel(self, m):
        return m.max_vel if m.max_vel > 0.0 else self.maneuver_max_vel

    def _turn_rate(self, error):
        """Drehrate fuer einen Winkelfehler (rad), mindestens MANEUVER_MIN_ANG."""
        w = clamp(self.maneuver_k_ang * error, self.maneuver_max_ang)
        return math.copysign(max(abs(w), MANEUVER_MIN_ANG), error)

    def _rotate(self, pose, target_yaw):
        error = wrap_angle(target_yaw - pose[2])
        if abs(error) <= math.radians(MANEUVER_YAW_TOL_DEG):
            return Twist(), True
        twist = Twist()
        twist.angular.z = float(self._turn_rate(error))
        return twist, False

    def _straight(self, pose, m):
        sx, sy, syaw = self._maneuver_start
        along = (pose[0] - sx) * math.cos(syaw) + (pose[1] - sy) * math.sin(syaw)
        remaining = m.distance - along
        # Auch ueber das Ziel hinaus (Vorzeichen gewechselt) ist fertig.
        if abs(remaining) <= MANEUVER_DIST_TOL or remaining * m.distance < 0.0:
            return Twist(), True
        speed = min(self._max_vel(m), max(MANEUVER_MIN_VEL, self.maneuver_k_lin * abs(remaining)))
        twist = Twist()
        twist.linear.x = float(math.copysign(speed, remaining))
        # Richtung vom Start halten
        twist.angular.z = float(clamp(self.maneuver_k_ang * wrap_angle(syaw - pose[2]),
                                      self.maneuver_max_ang))
        return twist, False

    def _path(self, pose, m):
        points = [(p.pose.position.x, p.pose.position.y) for p in m.path.poses]
        if not points:
            return Twist(), True
        x, y, yaw = pose
        travel_yaw = yaw + math.pi if m.reverse else yaw

        # Zwischenpunkte ueberspringen, sobald sie nah genug sind.
        while self._waypoint < len(points) - 1 and math.hypot(
                points[self._waypoint][0] - x, points[self._waypoint][1] - y) \
                < MANEUVER_WAYPOINT_TOL:
            self._waypoint += 1

        gx, gy = points[self._waypoint]
        dist = math.hypot(gx - x, gy - y)
        bearing = wrap_angle(math.atan2(gy - y, gx - x) - travel_yaw)
        last = self._waypoint == len(points) - 1
        if last and (dist <= MANEUVER_DIST_TOL
                     or (abs(bearing) > math.pi / 2 and dist < 3 * MANEUVER_DIST_TOL)):
            # erreicht oder knapp vorbei (Ziel liegt schon hinter der Fahrtrichtung)
            return Twist(), True

        twist = Twist()
        if last and dist < MANEUVER_NEAR_M:
            # Ganz nah: nicht mehr drehen -- die Richtung zu einem Punkt wenige cm entfernt
            # springt mit jedem Odometrie-Wackler, der Bot drehte sich im Kreis (Lauf 11.11.).
            # Nur noch gerade bis auf Hoehe des Ziels; der seitliche Rest bleibt.
            along = dist * math.cos(bearing)
            if abs(along) <= MANEUVER_DIST_TOL:
                return Twist(), True
            speed = max(MANEUVER_MIN_VEL, self.maneuver_k_lin * abs(along))
            speed = math.copysign(min(self._max_vel(m), speed), along)
            twist.linear.x = float(-speed if m.reverse else speed)
            return twist, False
        if abs(bearing) > math.radians(MANEUVER_ROTATE_FIRST_DEG):
            twist.angular.z = float(self._turn_rate(bearing))
            return twist, False

        # Restweg bis zum Ende, damit an Zwischenpunkten nicht gebremst wird.
        rest = dist + sum(math.dist(points[i], points[i + 1])
                          for i in range(self._waypoint, len(points) - 1))
        speed = min(self._max_vel(m), max(MANEUVER_MIN_VEL, self.maneuver_k_lin * rest))
        speed *= math.cos(bearing)
        twist.linear.x = float(-speed if m.reverse else speed)
        twist.angular.z = float(clamp(self.maneuver_k_ang * bearing, self.maneuver_max_ang))
        return twist, False

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Keine Roboterpose fuer die Grundbewegung: {e}', throttle_duration_sec=5.0)
            return None
        # Veraltet? Am lokalen Empfang gemessen, nicht an der Bot-Uhr (die kann abweichen).
        now = time.monotonic()
        stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
        if stamp != self._pose_stamp:
            self._pose_stamp, self._pose_changed = stamp, now
        elif now - self._pose_changed > MANEUVER_POSE_STALE_S:
            self.get_logger().error(
                f'Roboterpose seit {now - self._pose_changed:.1f} s unveraendert (odom_filter, '
                '/joint_states?) -- Bot haelt an.', throttle_duration_sec=2.0)
            return None
        return pose2d_from_transform(tf)

    def _publish_stop(self):
        self.pub_cmd_vel.publish(Twist())

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down control_driving Node -- sende Null.')
        for _ in range(STOP_REPEAT):
            self._publish_stop()
            time.sleep(STOP_REPEAT_PERIOD_S)


def main(args=None):
    run_node(control_driving, args)


if __name__ == '__main__':
    main()
