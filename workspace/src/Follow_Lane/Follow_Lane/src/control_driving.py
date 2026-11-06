"""Einzige Quelle fuer Fahrbefehle: waehlt je nach /control/mode den Regler.

Ein:  /control/mode (UInt8), /detect/lane (Float64, Spurfehler), /nav2/cmd_vel (Twist),
      /detect/pallet (PalletDetection), /control/maneuver (Maneuver), TF odom -> base_footprint
Aus:  /control/cmd_vel (Twist) -> notaus -> /cmd_vel,
      /control/maneuver_status (UInt32, id der zuletzt beendeten Grundbewegung)
Lane/TunnelEntry: PID auf den Spurfehler. Obstacle/Tunnel: /nav2/cmd_vel.
Dock: Feinanfahrt auf die Palette im Kamerabild. Maneuver: Grundbewegung in odom.
Alles ausser dem PID wird auf die Radgrenze skaliert, der PID begrenzt selbst.
Jeder andere Modus, unbekannter Modus oder veraltete Quelle: Nullbefehl.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64, UInt8, UInt32
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

from follow_lane_msg.msg import Maneuver, PalletDetection
from Follow_Lane.src.common import (
    ControlType, control_mode_from_msg, frange, pose2d_from_transform, read_params,
    require_params, run_node, wrap_angle)

# TunnelEntry mit PID: auf der 20-cm-Spur vor dem Tunnel passt der Bot fuer Nav2
# nicht zwischen die Spurwaende.
PID_MODES = (ControlType.Lane, ControlType.TunnelEntry)
NAV2_MODES = (ControlType.Obstacle, ControlType.Tunnel)

ODOM_FRAME = 'odom'
ROBOT_FRAME = 'base_footprint'

# Prueft, ob die aktive Quelle noch liefert, und regelt Maneuver; = controller_frequency
# in nav2_params.yaml.
WATCHDOG_RATE_HZ = 20.0

# Feste Abtastzeit fuer D- und I-Anteil: die Bildrate schwankt, kp/ki/kd sind auf diesen Wert
# eingestellt.
PID_SAMPLE_TIME_S = 0.1
INTEGRAL_LIMIT = 1.0
# Begrenzung der PID-Lenkung (rad/s); der Burger schafft ~2,84 rad/s.
MAX_ANGULAR_VEL = 2.0
# Hoechstgeschwindigkeit des Burger; das aeussere Rad (v + |omega| * WHEEL_SEPARATION / 2)
# bleibt darunter. Gleicher Wert wie vx_max in nav2_params.yaml.
MAX_WHEEL_VEL = 0.22
WHEEL_SEPARATION = 0.160

# Zeitgrenzen (s): LANE_CMD_TIMEOUT und DOCK_CMD_TIMEOUT umfassen mehrere Kamerabilder,
# NAV2_CMD_TIMEOUT fuenf Perioden von controller_frequency (20 Hz), MODE_TIMEOUT drei
# Perioden von switch_control (2 Hz).
LANE_CMD_TIMEOUT = 0.3
DOCK_CMD_TIMEOUT = 0.5
NAV2_CMD_TIMEOUT = 0.25
MODE_TIMEOUT = 1.5
MODE_SWITCH_HOLD = 0.3

STOP_REPEAT = 10
STOP_REPEAT_PERIOD_S = 0.05


def clamp(x, limit):
    return max(-limit, min(limit, x))


class control_driving(Node):
    """Vorzeichen: lane_error > 0 heisst Spurmitte links der Bildmitte, angular.z > 0
    heisst nach links drehen (REP 103).

    Nach jedem Wechsel des Reglers zaehlen nur Befehle, die danach eintreffen; der
    Wechsel selbst sendet sofort Null, damit turtlebot3_node nicht den letzten Befehl
    der alten Quelle weiterfaehrt. Lane <-> TunnelEntry behaelt den PID ohne Halt.
    Nach dem Wechsel in den Tunnel bleibt die Nav2-Quelle MODE_SWITCH_HOLD lang gesperrt.

    Maneuver: switch_control sendet die Grundbewegung zyklisch auf /control/maneuver.
    Eine neue id startet sie ab der aktuellen Pose; ist sie fertig, meldet dieser
    Node die id auf /control/maneuver_status und sendet Null, bis eine neue kommt.
    Ein Moduswechsel verwirft die laufende Bewegung.

    Dock: dreht auf der Stelle, solange die Palette seitlich mehr als dock_align_tol
    neben dem Magneten liegt, sonst faehrt er langsam vor und lenkt nach. Ab
    dock_contact_gap_px steht er; ob gegriffen wird, entscheidet switch_control.
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
        # Maneuver: Geschwindigkeit proportional zum Rest (k_lin, k_ang), begrenzt
        # nach oben und unten; unter min_* bewegt sich der Burger nicht sicher.
        self.declare_parameter('maneuver_max_vel',   Parameter.Type.DOUBLE, frange(0.01, 0.22))
        self.declare_parameter('maneuver_min_vel',   Parameter.Type.DOUBLE, frange(0.0, 0.1))
        self.declare_parameter('maneuver_max_ang',   Parameter.Type.DOUBLE, frange(0.1, 2.8))
        self.declare_parameter('maneuver_min_ang',   Parameter.Type.DOUBLE, frange(0.0, 1.0))
        self.declare_parameter('maneuver_k_lin',     Parameter.Type.DOUBLE, frange(0.1, 10.0))
        self.declare_parameter('maneuver_k_ang',     Parameter.Type.DOUBLE, frange(0.1, 10.0))
        # Fertig, wenn so nah am Ziel (m bzw. Grad). Zwischenpunkte eines Pfads gelten
        # ab maneuver_waypoint_tol als erreicht, ohne dort zu halten.
        self.declare_parameter('maneuver_dist_tol',  Parameter.Type.DOUBLE, frange(0.002, 0.1))
        self.declare_parameter('maneuver_yaw_tol_deg', Parameter.Type.DOUBLE, frange(0.2, 20.0))
        self.declare_parameter('maneuver_waypoint_tol', Parameter.Type.DOUBLE, frange(0.005, 0.3))
        # Liegt der naechste Pfadpunkt weiter als das neben der Fahrtrichtung, erst
        # auf der Stelle drehen.
        self.declare_parameter('maneuver_rotate_first_deg', Parameter.Type.DOUBLE, frange(5.0, 90.0))
        # Dock: Geschwindigkeit, ab dock_slow_gap_px abbremsen bis auf dock_min_factor.
        self.declare_parameter('dock_vel',           Parameter.Type.DOUBLE, frange(0.005, 0.1))
        self.declare_parameter('dock_min_factor',    Parameter.Type.DOUBLE, frange(0.05, 1.0))
        self.declare_parameter('dock_slow_gap_px',   Parameter.Type.DOUBLE, frange(1.0, 500.0))
        self.declare_parameter('dock_contact_gap_px', Parameter.Type.DOUBLE, frange(-50.0, 50.0))
        self.declare_parameter('dock_k_ang',         Parameter.Type.DOUBLE, frange(0.0, 10.0))
        # Seitlicher Versatz (Anteil der halben Bildbreite), ab dem nur gedreht wird.
        self.declare_parameter('dock_align_tol',     Parameter.Type.DOUBLE, frange(0.0, 1.0))
        self._param_names = [
            'kp', 'ki', 'kd', 'MAX_VEL', 'speed_curve_factor', 'min_speed_factor',
            'maneuver_max_vel', 'maneuver_min_vel', 'maneuver_max_ang', 'maneuver_min_ang',
            'maneuver_k_lin', 'maneuver_k_ang', 'maneuver_dist_tol', 'maneuver_yaw_tol_deg',
            'maneuver_waypoint_tol', 'maneuver_rotate_first_deg',
            'dock_vel', 'dock_min_factor', 'dock_slow_gap_px', 'dock_contact_gap_px',
            'dock_k_ang', 'dock_align_tol',
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
        self.sub_pallet = self.create_subscription(
            PalletDetection, '/detect/pallet', self.cb_pallet, 1)
        self.sub_maneuver = self.create_subscription(
            Maneuver, '/control/maneuver', self.cb_maneuver, 1)

        # --- Publisher ---
        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)
        self.pub_maneuver_status = self.create_publisher(UInt32, '/control/maneuver_status', 1)

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
        self._dock_time = None
        self._nav2_blocked_until = 0.0
        self._maneuver = None           # laufende Grundbewegung (Maneuver-Nachricht)
        self._maneuver_start = None     # (x, y, yaw) in odom beim Start
        self._waypoint = 0              # PATH: Index des naechsten Punkts
        self._maneuver_done_id = None
        self._reset_pid()

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
        self._dock_time = None
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
        if now < self._nav2_blocked_until:
            return
        self._nav2_time = now
        self.pub_cmd_vel.publish(self._limit_wheels(msg))

    def cb_pallet(self, msg):
        if not self._mode_is(ControlType.Dock):
            return
        read_params(self, self._param_names)
        self._dock_time = time.monotonic()
        self.pub_cmd_vel.publish(self._limit_wheels(self._dock(msg)))

    def cb_maneuver(self, msg):
        # Zyklisch wiederholt: nur eine neue id (oder die erste nach einem
        # Moduswechsel) startet die Bewegung, eine beendete nie erneut.
        if msg.id == self._maneuver_done_id:
            return
        if self._maneuver is not None and msg.id == self._maneuver.id:
            return
        self._maneuver = msg
        self._maneuver_start = None
        self._waypoint = 0

    def cb_watchdog(self):
        read_params(self, self._param_names)
        if self._maneuver_done_id is not None:
            self.pub_maneuver_status.publish(UInt32(data=self._maneuver_done_id))

        now = time.monotonic()
        if self._mode_is(ControlType.Maneuver):
            self._drive_maneuver()
            return
        if self._mode_is(*PID_MODES):
            fresh = self._fresh(self._lane_time, now, LANE_CMD_TIMEOUT)
        elif self._mode_is(*NAV2_MODES):
            fresh = self._fresh(self._nav2_time, now, NAV2_CMD_TIMEOUT)
        elif self._mode_is(ControlType.Dock):
            fresh = self._fresh(self._dock_time, now, DOCK_CMD_TIMEOUT)
        else:
            fresh = False
        if not fresh:
            self._publish_stop()

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

    def _dock(self, msg):
        twist = Twist()
        if not msg.found or msg.gap_px <= self.dock_contact_gap_px:
            return twist
        turn = clamp(self.dock_k_ang * msg.lateral, self.maneuver_max_ang)
        if abs(msg.lateral) > self.dock_align_tol:
            # Auf der Stelle zur Palette drehen; unter maneuver_min_ang dreht er nicht.
            twist.angular.z = float(math.copysign(max(abs(turn), self.maneuver_min_ang), turn))
            return twist
        factor = max(self.dock_min_factor, min(1.0, msg.gap_px / self.dock_slow_gap_px))
        twist.linear.x = float(self.dock_vel * factor)
        twist.angular.z = float(turn)
        return twist

    def _drive_maneuver(self):
        m = self._maneuver
        if m is None:
            self._publish_stop()
            return
        pose = self._robot_pose()
        if pose is None:
            self._publish_stop()
            return
        if self._maneuver_start is None:
            self._maneuver_start = pose

        if m.type == Maneuver.ROTATE:
            twist, done = self._rotate(pose, m.yaw)
        elif m.type == Maneuver.STRAIGHT:
            twist, done = self._straight(pose, m)
        elif m.type == Maneuver.PATH:
            twist, done = self._path(pose, m)
        else:
            self.get_logger().warn(f'Unbekannte Grundbewegung {m.type} -- gilt als fertig.')
            twist, done = Twist(), True

        if done:
            self.get_logger().info(f'Grundbewegung {m.id} fertig.')
            self._maneuver_done_id = m.id
            self._maneuver = None
            self.pub_maneuver_status.publish(UInt32(data=m.id))
            self._publish_stop()
            return
        self.pub_cmd_vel.publish(self._limit_wheels(twist))

    def _max_vel(self, m):
        return m.max_vel if m.max_vel > 0.0 else self.maneuver_max_vel

    def _turn_rate(self, error):
        """Drehrate fuer einen Winkelfehler (rad), mindestens maneuver_min_ang."""
        w = clamp(self.maneuver_k_ang * error, self.maneuver_max_ang)
        return math.copysign(max(abs(w), self.maneuver_min_ang), error)

    def _rotate(self, pose, target_yaw):
        error = wrap_angle(target_yaw - pose[2])
        if abs(error) <= math.radians(self.maneuver_yaw_tol_deg):
            return Twist(), True
        twist = Twist()
        twist.angular.z = float(self._turn_rate(error))
        return twist, False

    def _straight(self, pose, m):
        sx, sy, syaw = self._maneuver_start
        along = (pose[0] - sx) * math.cos(syaw) + (pose[1] - sy) * math.sin(syaw)
        remaining = m.distance - along
        # Auch ueber das Ziel hinaus (Vorzeichen gewechselt) ist fertig.
        if abs(remaining) <= self.maneuver_dist_tol or remaining * m.distance < 0.0:
            return Twist(), True
        speed = min(self._max_vel(m), max(self.maneuver_min_vel, self.maneuver_k_lin * abs(remaining)))
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
                < self.maneuver_waypoint_tol:
            self._waypoint += 1

        gx, gy = points[self._waypoint]
        dist = math.hypot(gx - x, gy - y)
        bearing = wrap_angle(math.atan2(gy - y, gx - x) - travel_yaw)
        last = self._waypoint == len(points) - 1
        if last and (dist <= self.maneuver_dist_tol
                     or (abs(bearing) > math.pi / 2 and dist < 3 * self.maneuver_dist_tol)):
            # erreicht oder knapp vorbei (Ziel liegt schon hinter der Fahrtrichtung)
            return Twist(), True

        twist = Twist()
        if abs(bearing) > math.radians(self.maneuver_rotate_first_deg):
            twist.angular.z = float(self._turn_rate(bearing))
            return twist, False

        # Restweg bis zum Ende, damit an Zwischenpunkten nicht gebremst wird.
        rest = dist + sum(math.dist(points[i], points[i + 1])
                          for i in range(self._waypoint, len(points) - 1))
        speed = min(self._max_vel(m), max(self.maneuver_min_vel, self.maneuver_k_lin * rest))
        speed *= math.cos(bearing)
        twist.linear.x = float(-speed if m.reverse else speed)
        twist.angular.z = float(clamp(self.maneuver_k_ang * bearing, self.maneuver_max_ang))
        return twist, False

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Keine Roboterpose fuer Maneuver: {e}', throttle_duration_sec=5.0)
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
