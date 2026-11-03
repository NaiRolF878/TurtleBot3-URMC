"""Einzige Quelle fuer Fahrbefehle: waehlt je nach /control/mode den Regler.

Ein:  /control/mode (UInt8), /detect/lane (Float64, Spurfehler), /nav2/cmd_vel (Twist)
Aus:  /control/cmd_vel (Twist) -> notaus -> /cmd_vel
Lane/TunnelEntry: PID auf den Spurfehler. Obstacle/Tunnel: /nav2/cmd_vel, auf die
Radgrenze skaliert.
Jeder andere Modus, unbekannter Modus oder veraltete Quelle: Nullbefehl.
"""
import time

from geometry_msgs.msg import Twist
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64, UInt8

from Follow_Lane.src.common import ControlType, control_mode_from_msg, frange, read_params, run_node

# TunnelEntry mit PID: auf der 20-cm-Spur vor dem Tunnel passt der Bot fuer Nav2
# nicht zwischen die Spurwaende.
PID_MODES = (ControlType.Lane, ControlType.TunnelEntry)
NAV2_MODES = (ControlType.Obstacle, ControlType.Tunnel)

# Prueft, ob die aktive Quelle noch liefert. 20 Hz = controller_frequency in
# nav2_params.yaml, damit nav2_cmd_timeout auf eine Reglerperiode genau greift.
WATCHDOG_RATE_HZ = 20.0

# Feste Abtastzeit fuer den D-Anteil, nicht die gemessene: detect_lane liefert so
# schnell, wie es Bilder schafft, die Rate schwankt. kp/ki/kd sind mit diesem Wert
# eingestellt; ein gemessenes dt wuerde ihre Wirkung veraendern. Der I-Anteil
# summiert aus demselben Grund ohne dt.
PID_SAMPLE_TIME_S = 0.1
INTEGRAL_LIMIT = 1.0
# Begrenzung der PID-Lenkung (rad/s); der Burger schafft ~2,84 rad/s.
MAX_ANGULAR_VEL = 2.0
# Max. Geschwindigkeit der Motoren 0,22 m/s (TurtleBot3-Handbuch, Burger), darum
# bleibt das aeussere Rad (v + |omega| * WHEEL_SEPARATION / 2) darunter.
# Gleicher Wert wie vx_max in nav2_params.yaml.
MAX_WHEEL_VEL = 0.22
WHEEL_SEPARATION = 0.160

STOP_REPEAT = 10
STOP_REPEAT_PERIOD_S = 0.05


class control_driving(Node):
    """Vorzeichen: lane_error > 0 heisst Spurmitte links der Bildmitte, angular.z > 0
    heisst nach links drehen (REP 103).

    Nach jedem Wechsel des Reglers zaehlen nur Befehle, die danach eintreffen; der
    Wechsel selbst sendet sofort Null, damit turtlebot3_node nicht den letzten Befehl
    der alten Quelle weiterfaehrt. Lane <-> TunnelEntry behaelt den PID ohne Halt.
    Nach dem Wechsel in den Tunnel bleibt die Nav2-Quelle mode_switch_hold lang
    gesperrt: bis der Cancel von nav2_goal_publisher greift, faehrt controller_server
    sonst noch ein Obstacle-Ziel aus der Spurmitte, also aus Kameradaten.
    """

    def __init__(self):
        super().__init__('control_driving')

        # --- Parameter ---
        # Ohne Default: nur aus Control_driving_PID.yaml, sonst startet der Node nicht.
        self.declare_parameter('kp',                 Parameter.Type.DOUBLE, frange(0.0, 20.0))
        self.declare_parameter('ki',                 Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self.declare_parameter('kd',                 Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self.declare_parameter('MAX_VEL',            Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self.declare_parameter('speed_curve_factor', Parameter.Type.DOUBLE, frange(0.0, 5.0))
        self.declare_parameter('min_speed_factor',   Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self._require_params(['kp', 'ki', 'kd', 'MAX_VEL', 'speed_curve_factor', 'min_speed_factor'])
        # Zeitgrenzen in s. lane_cmd_timeout umfasst mehrere Kamerabilder,
        # nav2_cmd_timeout fuenf Perioden von controller_frequency (20 Hz),
        # mode_timeout drei Perioden von switch_control (2 Hz).
        self.declare_parameter('lane_cmd_timeout',   0.3,  frange(0.05, 2.0))
        self.declare_parameter('nav2_cmd_timeout',   0.25, frange(0.05, 2.0))
        self.declare_parameter('mode_timeout',       1.5,  frange(0.5, 10.0))
        self.declare_parameter('mode_switch_hold',   0.3,  frange(0.0, 2.0))
        self._param_names = [
            'kp', 'ki', 'kd', 'MAX_VEL', 'speed_curve_factor', 'min_speed_factor',
            'lane_cmd_timeout', 'nav2_cmd_timeout', 'mode_timeout', 'mode_switch_hold',
        ]
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_lane_error = self.create_subscription(
            Float64, '/detect/lane', self.cb_lane_error, 1)
        self.sub_nav2_cmd_vel = self.create_subscription(
            Twist, '/nav2/cmd_vel', self.cb_nav2_cmd_vel, 1)

        # --- Publisher ---
        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / WATCHDOG_RATE_HZ, self.cb_watchdog)

        # --- Zustand ---
        # None bis zur ersten /control/mode: ohne switch_control wird nicht gefahren.
        self.control_mode = None
        self._mode_time = None
        self._lane_time = None
        self._nav2_time = None
        self._nav2_blocked_until = 0.0
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
        if new_mode in PID_MODES:
            self._reset_pid()
        if new_mode == ControlType.Tunnel:
            self._nav2_blocked_until = now + self.mode_switch_hold
        self._publish_stop()

    def cb_lane_error(self, msg):
        if not self._mode_is(*PID_MODES):
            return
        read_params(self, self._param_names)

        now = time.monotonic()
        # Nach einer Luecke ist lastError Sekunden alt und der D-Anteil gaebe
        # einen Lenkschlag.
        if not self._fresh(self._lane_time, now, self.lane_cmd_timeout):
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

    def cb_watchdog(self):
        read_params(self, self._param_names)
        now = time.monotonic()
        if self._mode_is(*PID_MODES):
            fresh = self._fresh(self._lane_time, now, self.lane_cmd_timeout)
        elif self._mode_is(*NAV2_MODES):
            fresh = self._fresh(self._nav2_time, now, self.nav2_cmd_timeout)
        else:
            fresh = False
        if not fresh:
            self._publish_stop()

    # --- Hilfsmethoden ---

    def _require_params(self, names):
        """Beendet den Node mit Fehlermeldung, wenn ein Parameter keinen Wert hat."""
        missing = []
        for name in names:
            try:
                self.get_parameter(name)
            except ParameterUninitializedException:
                missing.append(name)
        if missing:
            self.get_logger().fatal(
                f'Ohne Wert: {", ".join(missing)} -- Control_driving_PID.yaml nicht geladen '
                'oder unvollstaendig. control_driving startet nicht, der Bot faehrt nicht.')
            raise SystemExit(1)

    def _mode_is(self, *modes):
        """Aktueller Modus ist einer von modes und /control/mode ist nicht veraltet."""
        if self.control_mode not in modes:
            return False
        if not self._fresh(self._mode_time, time.monotonic(), self.mode_timeout):
            self.get_logger().warn(
                f'Seit {self.mode_timeout:.1f} s kein /control/mode -- laeuft switch_control? '
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
