"""Einzige Quelle fuer Fahrbefehle: waehlt je nach /control/mode den Regler.

Ein:  /control/mode (UInt8), /detect/lane (Float64, Spurfehler), /avoid/zones (AvoidZones),
      /control/obstacle_enabled (Bool), /nav2/cmd_vel (Twist)
Aus:  /control/cmd_vel (Twist) -> notaus -> /cmd_vel
Lane: PID auf Spurfehler + Ausweich-Offset (nur mit obstacle_enabled). TunnelEntry: PID ohne
Offset. Obstacle:
Ausweichmanoever aus /avoid/zones. Tunnel: /nav2/cmd_vel. Obstacle und Tunnel auf die
Radgrenze skaliert.
Jeder andere Modus, unbekannter Modus oder veraltete Quelle: Nullbefehl.
"""
import time

from follow_lane_msg.msg import AvoidZones
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float64, UInt8

from Follow_Lane.src.common import (
    ControlType, control_mode_from_msg, frange, read_params, require_params, run_node)

# TunnelEntry mit PID: auf der 20-cm-Spur vor dem Tunnel passt der Bot fuer Nav2
# nicht zwischen die Spurwaende.
PID_MODES = (ControlType.Lane, ControlType.TunnelEntry)
NAV2_MODES = (ControlType.Tunnel,)

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

# Zeitgrenzen (s): LANE_CMD_TIMEOUT umfasst mehrere Kamerabilder, AVOID_TIMEOUT mehrere
# LiDAR-Umlaeufe, NAV2_CMD_TIMEOUT fuenf Perioden von controller_frequency (20 Hz),
# MODE_TIMEOUT drei Perioden von switch_control (2 Hz).
LANE_CMD_TIMEOUT = 0.3
AVOID_TIMEOUT = 0.5
NAV2_CMD_TIMEOUT = 0.25
MODE_TIMEOUT = 1.5
MODE_SWITCH_HOLD = 0.3

STOP_REPEAT = 10
STOP_REPEAT_PERIOD_S = 0.05


class control_driving(Node):
    """Vorzeichen: lane_error > 0 heisst Spurmitte links der Bildmitte, angular.z > 0
    heisst nach links drehen (REP 103).

    Nach jedem Wechsel des Reglers zaehlen nur Befehle, die danach eintreffen; der
    Wechsel selbst sendet sofort Null, damit turtlebot3_node nicht den letzten Befehl
    der alten Quelle weiterfaehrt. Lane <-> TunnelEntry behaelt den PID ohne Halt.
    Nach dem Wechsel in den Tunnel bleibt die Nav2-Quelle MODE_SWITCH_HOLD lang gesperrt.
    Hier gibt es vor dem Tunnel kein Nav2-Ziel, die Sperre verzoegert nur den Start.
    """

    def __init__(self):
        super().__init__('control_driving')

        # --- Parameter ---
        # Ohne Default: nur aus Control_driving_PID_Wide.yaml, sonst startet der Node nicht.
        self.declare_parameter('kp',                 Parameter.Type.DOUBLE, frange(0.0, 20.0))
        self.declare_parameter('ki',                 Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self.declare_parameter('kd',                 Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self.declare_parameter('MAX_VEL',            Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self.declare_parameter('speed_curve_factor', Parameter.Type.DOUBLE, frange(0.0, 5.0))
        self.declare_parameter('min_speed_factor',   Parameter.Type.DOUBLE, frange(0.0, 10.0))
        # Ausweichmanoever im Obstacle-Modus: Drehrichtung und "vorwaerts erlaubt"
        # kommen von avoid_node.
        self.declare_parameter('avoid_linear_speed',  Parameter.Type.DOUBLE, frange(0.0, 0.5))
        self.declare_parameter('avoid_angular_speed', Parameter.Type.DOUBLE, frange(0.0, 3.0))
        self._param_names = ['kp', 'ki', 'kd', 'MAX_VEL', 'speed_curve_factor', 'min_speed_factor',
                             'avoid_linear_speed', 'avoid_angular_speed']
        require_params(self, self._param_names)
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_lane_error = self.create_subscription(
            Float64, '/detect/lane', self.cb_lane_error, 1)
        self.sub_avoid_zones = self.create_subscription(
            AvoidZones, '/avoid/zones', self.cb_avoid_zones, 1)
        self.sub_obstacle_enabled = self.create_subscription(
            Bool, '/control/obstacle_enabled', self.cb_obstacle_enabled, 1)
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
        self._zones = None
        self._zones_time = None
        self._avoid_time = None
        # False bis zur ersten Meldung von switch_control: im Zweifel reine Spurfolge.
        self._obstacle_enabled = False
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
            # Der Offset gehoert nicht zum Spurfehler von TunnelEntry; im D-Anteil gaebe er
            # sonst einen Lenkschlag.
            self.last_error -= self._applied_offset
            self._applied_offset = 0.0
            return
        self._lane_time = None
        self._nav2_time = None
        self._avoid_time = None
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
        # Der Offset zieht den Bot innerhalb der Spur am Hindernis vorbei, die
        # Spurerkennung haelt weiter gegen die Linien. Vor dem Tunnel kein Offset.
        offset = self._avoid_offset(now) if self.control_mode == ControlType.Lane else 0.0
        self.pub_cmd_vel.publish(self._pid(msg.data + offset))
        self._applied_offset = offset

    def cb_obstacle_enabled(self, msg):
        self._obstacle_enabled = msg.data

    def cb_avoid_zones(self, msg):
        now = time.monotonic()
        self._zones = msg
        self._zones_time = now
        if not self._mode_is(ControlType.Obstacle):
            return
        read_params(self, self._param_names)
        self._avoid_time = now
        twist = Twist()
        twist.linear.x = float(self.avoid_linear_speed) if msg.forward_allowed else 0.0
        twist.angular.z = float(msg.direction) * float(self.avoid_angular_speed)
        self.pub_cmd_vel.publish(self._limit_wheels(twist))

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
            fresh = self._fresh(self._lane_time, now, LANE_CMD_TIMEOUT)
        elif self._mode_is(ControlType.Obstacle):
            fresh = self._fresh(self._avoid_time, now, AVOID_TIMEOUT)
        elif self._mode_is(*NAV2_MODES):
            fresh = self._fresh(self._nav2_time, now, NAV2_CMD_TIMEOUT)
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

    def _avoid_offset(self, now):
        """Offset von avoid_node, 0.0 wenn veraltet: faellt avoid_node aus, faehrt der
        PID reine Spurfolge statt dauerhaft schief. 0.0 auch mit obstacle_enabled false."""
        if not self._obstacle_enabled or not self._fresh(self._zones_time, now, AVOID_TIMEOUT):
            return 0.0
        return float(self._zones.offset)

    def _reset_pid(self):
        self.integral = 0.0
        self.last_error = 0.0
        self._applied_offset = 0.0

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
