"""Zustandsmaschine fuer den Fahrmodus, einziger Publisher von /control/mode.

Ein:  /avoid/zones (AvoidZones), /detect/sign, /control/tunnel_request, /detect/lines_lost,
      /tunnel/outside
Aus:  /control/mode (UInt8, ControlType), /control/tunnel_source (String)
Schwellwerte und Entprellung fuer jeden Moduswechsel liegen nur hier.
"""
import time

from follow_lane_msg.msg import AvoidZones
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String, UInt8

from Follow_Lane.src.common import (
    ControlType, SignType, frange, read_params, require_params, run_node)

# Zyklisch senden, damit auch spaeter startende Nodes den Modus bekommen.
# control_driving haelt an, wenn er laenger als mode_timeout nichts hoert.
MODE_PUBLISH_PERIOD_S = 0.5
# So lange (s) muessen beide Spurlinien fehlen, bis die Kamera ausgeht: mehrere Bilder,
# aber die Bot-Nase ist dann noch vor dem Linienende (Suchzeile endet ~10 cm vor der Achse).
LINES_LOST_S = 0.2

TUNNEL_MODES = (ControlType.TunnelEntry, ControlType.Tunnel)


class switch_control(Node):
    """Lane -> Obstacle: die Frontzone von avoid_node ist naeher als takeover_distance.
    Bis dahin weicht der Bot im Lane-Modus ueber den Offset auf den Spurfehler aus.

    Obstacle -> Lane: die Frontzone ist takeover_release_debounce lang weiter als
    takeover_release_distance. Ohne diese Hysterese springt der Modus an der Schwelle im
    Sekundentakt zwischen PID und Ausweichmanoever.

    -> TunnelEntry per Schild (/detect/sign = SignType.Tunnel, Quelle "auto") oder
    Button im Debugbild (/control/tunnel_request True, Quelle "manuell"). Der Button
    ist noetig, weil jede Challenge einzeln laeuft und es vor Ort sonst keinen
    Ausloeser gibt. Der Bot steht dabei vor dem Tunnel und faehrt mit Kamera auf die
    Oeffnung zu; die Einfahrt ist laut Reglement noch erlaubt.

    TunnelEntry -> Tunnel, bevor der Bot drin ist: beide Spurlinien enden kurz vor dem
    Bot, denn sie reichen nur bis zur Oeffnung. Ab dann keine Kamera mehr. Durch die Tuer
    faehrt er nur mit LiDAR (tunnel_explorer_node).

    Tunnel -> Modus von vor dem Tunnel, sobald tunnel_explorer_node meldet, dass der Bot
    ausserhalb der Wand-Huelle ist. Beides jederzeit von Hand abbrechbar (False).
    """

    def __init__(self):
        super().__init__('switch_control')

        # --- Parameter ---
        # Vor Ort einstellen, Werte nur in Switch_Control.yaml.
        self.declare_parameter('takeover_distance', Parameter.Type.DOUBLE, frange(0.05, 3.0))
        self.declare_parameter('takeover_release_distance', Parameter.Type.DOUBLE, frange(0.05, 5.0))
        self.declare_parameter('takeover_release_debounce', Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self._param_names = ['takeover_distance', 'takeover_release_distance',
                             'takeover_release_debounce']
        require_params(self, self._param_names)
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_avoid_zones = self.create_subscription(
            AvoidZones, '/avoid/zones', self.cb_avoid_zones, 1)
        # Noch ohne Publisher: der Schild-Detektor ist nicht gebaut.
        self.sub_sign = self.create_subscription(UInt8, '/detect/sign', self.cb_sign, 1)
        self.sub_tunnel_request = self.create_subscription(
            Bool, '/control/tunnel_request', self.cb_tunnel_request, 1)
        self.sub_lines_lost = self.create_subscription(
            Bool, '/detect/lines_lost', self.cb_lines_lost, 1)
        self.sub_tunnel_outside = self.create_subscription(
            Bool, '/tunnel/outside', self.cb_tunnel_outside, 1)

        # --- Publisher ---
        self.pub_control_mode = self.create_publisher(UInt8, '/control/mode', 1)
        self.pub_tunnel_source = self.create_publisher(String, '/control/tunnel_source', 1)

        # --- Timer ---
        self.timer = self.create_timer(MODE_PUBLISH_PERIOD_S, self.cb_timer)

        # --- Zustand ---
        self.control_mode = ControlType.Lane
        self._mode_before_tunnel = None
        self._tunnel_source = ''          # 'auto' (Schild) oder 'manuell' (Button)
        self._obstacle_clear_since = None
        self._lines_seen_time = None      # zuletzt eine Spurlinie gesehen (TunnelEntry)

        self.get_logger().info('Started switch_control Node.')

    # --- Callbacks ---

    def cb_timer(self):
        self._update_tunnel_entry()
        self._publish_mode()

    def cb_avoid_zones(self, msg):
        read_params(self, self._param_names)

        self._update_tunnel_entry()
        if self.control_mode == ControlType.Lane:
            if msg.front < self.takeover_distance:
                self._set_mode(ControlType.Obstacle, f'Frontzone blockiert ({msg.front:.2f} m)')
        elif self.control_mode == ControlType.Obstacle:
            self._update_obstacle_exit(msg.front)

    def cb_sign(self, msg):
        try:
            sign = SignType(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unbekanntes Schild-Signal: {msg.data}')
            return

        if sign == SignType.Tunnel:
            self._enter_tunnel('auto')

    def cb_tunnel_request(self, msg):
        if msg.data:
            self._enter_tunnel('manuell')
        elif self.control_mode in TUNNEL_MODES:
            self.get_logger().info('Tunnel von Hand abgebrochen.')
            self._leave_tunnel()

    def cb_lines_lost(self, msg):
        if self.control_mode == ControlType.TunnelEntry and not msg.data:
            self._lines_seen_time = time.monotonic()
        self._update_tunnel_entry()

    def cb_tunnel_outside(self, msg):
        if msg.data and self.control_mode == ControlType.Tunnel:
            self.get_logger().info('Bot ist ausserhalb der Wand-Huelle.')
            self._leave_tunnel()

    # --- Hilfsmethoden ---

    def _set_mode(self, new_mode, reason):
        if new_mode == self.control_mode:
            return
        self.get_logger().info(f'{reason} -> {self.control_mode.name} => {new_mode.name}.')
        self.control_mode = new_mode
        # Entprell-Zeitstempel gelten nur fuer den Modus, in dem sie gesetzt wurden.
        self._obstacle_clear_since = None
        # Sofort senden, nicht erst mit dem Timer: control_driving schaltet sonst
        # bis zu MODE_PUBLISH_PERIOD_S spaeter um.
        self._publish_mode()

    def _publish_mode(self):
        self.pub_control_mode.publish(UInt8(data=self.control_mode.value))
        self.pub_tunnel_source.publish(String(data=self._tunnel_source))

    def _enter_tunnel(self, source):
        if self.control_mode in TUNNEL_MODES:
            return
        self._mode_before_tunnel = self.control_mode
        self._tunnel_source = source
        self._lines_seen_time = time.monotonic()
        self._set_mode(ControlType.TunnelEntry, f'Tunnel ausgeloest ({source})')

    def _update_tunnel_entry(self):
        """Ohne neue Bilder gilt die Spur ebenfalls als zu Ende."""
        if self.control_mode == ControlType.TunnelEntry \
                and time.monotonic() - self._lines_seen_time >= LINES_LOST_S:
            self._set_mode(ControlType.Tunnel, 'Spurlinien enden')

    def _leave_tunnel(self):
        restored = self._mode_before_tunnel or ControlType.Lane
        self._mode_before_tunnel = None
        self._tunnel_source = ''
        self._set_mode(restored, 'Tunnel verlassen')

    def _update_obstacle_exit(self, front):
        if front <= self.takeover_release_distance:
            self._obstacle_clear_since = None
            return

        now = self.get_clock().now()
        if self._obstacle_clear_since is None:
            self._obstacle_clear_since = now
        elif (now - self._obstacle_clear_since).nanoseconds / 1e9 >= self.takeover_release_debounce:
            self._set_mode(ControlType.Lane, f'Front wieder frei ({front:.2f} m)')

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down switch_control Node.')


def main(args=None):
    run_node(switch_control, args)


if __name__ == '__main__':
    main()
