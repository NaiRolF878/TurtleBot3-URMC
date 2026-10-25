"""Einzige Quelle fuer Fahrbefehle: waehlt je nach /control/mode den Regler.

Ein:  /control/mode (UInt8), /detect/lane (Float64, Spurfehler), /avoid/zones (AvoidZones),
      /nav2/cmd_vel (Twist)
Aus:  /control/cmd_vel (Twist) -> notaus -> /cmd_vel
Lane: PID auf Spurfehler + Ausweich-Offset. Obstacle: Ausweichmanoever aus /avoid/zones.
TunnelEntry/Tunnel: /nav2/cmd_vel unveraendert.
Jeder andere Modus, unbekannter Modus oder veraltete Quelle: Nullbefehl.
"""
import time

from follow_lane_msg.msg import AvoidZones
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, UInt8

from Follow_Lane.src.common import ControlType, control_mode_from_msg, frange, read_params, run_node

NAV2_MODES = (ControlType.TunnelEntry, ControlType.Tunnel)

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

STOP_REPEAT = 10
STOP_REPEAT_PERIOD_S = 0.05


class control_driving(Node):
    """Vorzeichen: lane_error > 0 heisst Spurmitte links der Bildmitte, angular.z > 0
    heisst nach links drehen (REP 103).

    Nach jedem Moduswechsel zaehlen nur Befehle, die danach eintreffen; der Wechsel
    selbst sendet sofort Null, damit turtlebot3_node nicht den letzten Befehl der
    alten Quelle weiterfaehrt. Nach dem Wechsel in den Tunnel bleibt die
    Nav2-Quelle mode_switch_hold lang gesperrt: bis der Cancel von
    nav2_goal_publisher greift, faehrt controller_server sonst noch das Einfahrtsziel
    aus der Spurmitte, also aus Kameradaten.
    """

    def __init__(self):
        super().__init__('control_driving')

        # --- Parameter ---
        self.declare_parameter('kp',                 5.5,  frange(0.0, 20.0))
        self.declare_parameter('ki',                 0.15, frange(0.0, 10.0))
        self.declare_parameter('kd',                 2.75, frange(0.0, 10.0))
        self.declare_parameter('MAX_VEL',            0.34, frange(0.0, 10.0))
        self.declare_parameter('speed_curve_factor', 1.0,  frange(0.0, 5.0))
        self.declare_parameter('min_speed_factor',   0.5,  frange(0.0, 10.0))
        # Ausweichmanoever im Obstacle-Modus: Drehrichtung und "vorwaerts erlaubt"
        # kommen von avoid_node.
        self.declare_parameter('avoid_linear_speed',  0.06, frange(0.0, 0.5))
        self.declare_parameter('avoid_angular_speed', 0.7,  frange(0.0, 3.0))
        # Zeitgrenzen in s. lane_cmd_timeout umfasst mehrere Kamerabilder,
        # avoid_timeout mehrere LiDAR-Umlaeufe, nav2_cmd_timeout fuenf Perioden von
        # controller_frequency (20 Hz), mode_timeout drei Perioden von switch_control (2 Hz).
        self.declare_parameter('lane_cmd_timeout',   0.3,  frange(0.05, 2.0))
        self.declare_parameter('avoid_timeout',      0.5,  frange(0.05, 5.0))
        self.declare_parameter('nav2_cmd_timeout',   0.25, frange(0.05, 2.0))
        self.declare_parameter('mode_timeout',       1.5,  frange(0.5, 10.0))
        self.declare_parameter('mode_switch_hold',   0.3,  frange(0.0, 2.0))
        self._param_names = [
            'kp', 'ki', 'kd', 'MAX_VEL', 'speed_curve_factor', 'min_speed_factor',
            'avoid_linear_speed', 'avoid_angular_speed',
            'lane_cmd_timeout', 'avoid_timeout', 'nav2_cmd_timeout', 'mode_timeout',
            'mode_switch_hold',
        ]
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_lane_error = self.create_subscription(
            Float64, '/detect/lane', self.cb_lane_error, 1)
        self.sub_avoid_zones = self.create_subscription(
            AvoidZones, '/avoid/zones', self.cb_avoid_zones, 1)
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
        self.control_mode = new_mode
        self._lane_time = None
        self._nav2_time = None
        self._avoid_time = None
        if new_mode == ControlType.Lane:
            self._reset_pid()
        if new_mode == ControlType.Tunnel:
            self._nav2_blocked_until = now + self.mode_switch_hold
        self._publish_stop()

    def cb_lane_error(self, msg):
        if not self._mode_is(ControlType.Lane):
            return
        read_params(self, self._param_names)

        now = time.monotonic()
        # Nach einer Luecke ist lastError Sekunden alt und der D-Anteil gaebe
        # einen Lenkschlag.
        if not self._fresh(self._lane_time, now, self.lane_cmd_timeout):
            self._reset_pid()
        self._lane_time = now
        # Der Offset zieht den Bot innerhalb der Spur am Hindernis vorbei, die
        # Spurerkennung haelt weiter gegen die Linien.
        self.pub_cmd_vel.publish(self._pid(msg.data + self._avoid_offset(now)))

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
        self.pub_cmd_vel.publish(twist)

    def cb_nav2_cmd_vel(self, msg):
        if not self._mode_is(*NAV2_MODES):
            return
        now = time.monotonic()
        if now < self._nav2_blocked_until:
            return
        self._nav2_time = now
        self.pub_cmd_vel.publish(msg)

    def cb_watchdog(self):
        read_params(self, self._param_names)
        now = time.monotonic()
        if self._mode_is(ControlType.Lane):
            fresh = self._fresh(self._lane_time, now, self.lane_cmd_timeout)
        elif self._mode_is(ControlType.Obstacle):
            fresh = self._fresh(self._avoid_time, now, self.avoid_timeout)
        elif self._mode_is(*NAV2_MODES):
            fresh = self._fresh(self._nav2_time, now, self.nav2_cmd_timeout)
        else:
            fresh = False
        if not fresh:
            self._publish_stop()

    # --- Hilfsmethoden ---

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

    def _avoid_offset(self, now):
        """Offset von avoid_node, 0.0 wenn veraltet: faellt avoid_node aus, faehrt der
        PID reine Spurfolge statt dauerhaft schief."""
        if not self._fresh(self._zones_time, now, self.avoid_timeout):
            return 0.0
        return float(self._zones.offset)

    def _reset_pid(self):
        self.integral = 0.0
        self.last_error = 0.0

    def _pid(self, error):
        self.integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, self.integral + error))
        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * (error - self.last_error) / PID_SAMPLE_TIME_S
        self.last_error = error

        twist = Twist()
        twist.linear.x = float(
            self.MAX_VEL * max(self.min_speed_factor, 1 - abs(error) * self.speed_curve_factor))
        twist.angular.z = float(max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, p + i + d)))
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
