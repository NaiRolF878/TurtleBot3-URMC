import math
from enum import Enum

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from std_msgs.msg import Bool, String, UInt8
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from follow_lane_msg.msg import LaneMask


class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Stop = 3
    Intersection = 4
    Align = 5
    Tunnel = 6


class switch_control(Node):
    """Zustandsmaschine fuer /control/mode -- einziger Publisher dieses Topics.

    Zusaetzlich (optional, cmd_mux_enabled) ein Prioritaets-Mux fuer die
    Fahrbefehle, siehe _cbMux:
        1. /cmd_safe  -- Not-Aus aus safety_node
        2. /cmd_avoid -- komplettes Ausweichen aus avoid_node
        3. /cmd_lane  -- Spurfolge aus control_driving
    Es gewinnt die hoechste Prioritaet, von der noch aktuelle Nachrichten
    eintreffen; der Ausgang liegt auf /control/cmd_vel und wird im Launch auf
    /cmd_vel gemappt. Standardmaessig ist der Mux AUS, damit die bestehenden
    Launch-Dateien (follow_lane, nav2_obstacle), in denen control_driving direkt
    auf /cmd_vel sendet, unveraendert weiterlaufen.

    Lane <-> Obstacle: per LiDAR, sobald etwas im Pruefkegel naeher als
    obstacle_trigger_distance liegt (mit Entprellung zurueck). Mit
    obstacle_source='avoid' entscheidet stattdessen avoid_node ueber
    /avoid/takeover, wann umgeschaltet wird -- dann laeuft der Wechsel synchron
    zur Uebernahme im Mux statt aus zwei Quellen mit verschiedenen Schwellen.

    -> Tunnel: per Schild auf /detect/sign. Der Detektor dafuer fehlt noch,
    manuell testbar mit
      ros2 topic pub /detect/sign std_msgs/msg/UInt8 "{data: 6}"
    Zurueck erst, wenn LiDAR freien Raum sieht UND die Kamera wieder Spurpixel
    liefert; wiederhergestellt wird der Modus von vor dem Tunnel.
    """

    def __init__(self):
        super().__init__('switch_control')

        self.get_logger().info('Started switch_control Node.')

        def frange(lo, hi):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

        # --- Obstacle-Erkennung (Lane <-> Obstacle) ---
        self.declare_parameter('obstacle_check_angle_deg', 30.0, frange(1.0, 90.0))
        self.declare_parameter('obstacle_trigger_distance', 0.4, frange(0.05, 3.0))
        self.declare_parameter('obstacle_clear_debounce', 1.0, frange(0.0, 10.0))

        # --- Tunnel-Austritt (Tunnel -> vorheriger Modus) ---
        self.declare_parameter('tunnel_exit_open_distance', 1.5, frange(0.1, 10.0))
        self.declare_parameter('tunnel_exit_open_ratio', 0.8, frange(0.0, 1.0))
        self.declare_parameter('tunnel_exit_min_lane_pixels', 20,
            ParameterDescriptor(integer_range=[IntegerRange(from_value=0, to_value=5000, step=1)]))
        self.declare_parameter('tunnel_exit_debounce', 1.0, frange(0.0, 10.0))

        # --- Quelle fuer den Lane<->Obstacle-Wechsel ---
        # 'scan'  = eigener LiDAR-Kegel (Verhalten wie bisher)
        # 'avoid' = avoid_node meldet die Uebernahme auf /avoid/takeover
        self.declare_parameter('obstacle_source', 'scan')

        # --- Fahrbefehl-Mux (Prioritaet 1 safe > 2 avoid > 3 lane) ---
        self.declare_parameter('cmd_mux_enabled', False)
        self.declare_parameter('cmd_mux_rate', 20.0, frange(1.0, 100.0))
        # Aktualitaetsfenster je Quelle: aeltere Nachrichten zaehlen nicht mehr,
        # so faellt der Mux automatisch auf die naechste Prioritaet zurueck,
        # sobald eine Quelle aufhoert zu senden.
        self.declare_parameter('cmd_timeout_safe',  0.5, frange(0.05, 5.0))
        self.declare_parameter('cmd_timeout_avoid', 0.5, frange(0.05, 5.0))
        self.declare_parameter('cmd_timeout_lane',  0.5, frange(0.05, 5.0))

        self._param_names = [
            'obstacle_check_angle_deg', 'obstacle_trigger_distance', 'obstacle_clear_debounce',
            'tunnel_exit_open_distance', 'tunnel_exit_open_ratio',
            'tunnel_exit_min_lane_pixels', 'tunnel_exit_debounce',
            'obstacle_source',
            'cmd_timeout_safe', 'cmd_timeout_avoid', 'cmd_timeout_lane',
        ]

        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cbScan, qos_profile_sensor_data)
        self.sub_lane_mask = self.create_subscription(
            LaneMask, '/detect/lane_mask', self.cbLaneMask, 1)
        # Publisher folgt mit der Schild-Detektor-Node
        self.sub_sign = self.create_subscription(UInt8, '/detect/sign', self.cbSign, 1)
        self.sub_takeover = self.create_subscription(
            Bool, '/avoid/takeover', self.cbAvoidTakeover, 1)

        self.pub_control = self.create_publisher(UInt8, '/control/mode', 1)

        # Zyklisch senden, damit auch spaeter startende Nodes den Mode bekommen.
        self.timer = self.create_timer(0.5, self.cbPubControlMode)

        self.current_control_mode = ControlType.Lane
        self._mode_before_tunnel = None
        self._obstacle_clear_since = None
        self._tunnel_exit_since = None
        self._lane_visible = False
        self._avoid_takeover = False

        self._setup_mux()

    def _read_params(self):
        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

    # ---------------------------------
    # Fahrbefehl-Mux
    # ---------------------------------

    def _setup_mux(self):
        """Subscriber/Publisher des Mux nur anlegen, wenn er gebraucht wird.

        Sonst haette dieser Node in den alten Launch-Dateien einen zweiten
        Publisher auf /control/cmd_vel -- neben control_driving, das dort direkt
        auf /cmd_vel gemappt ist.
        """
        self._mux_enabled = bool(self.get_parameter('cmd_mux_enabled').value)
        if not self._mux_enabled:
            self.mux_timer = None
            self.get_logger().info('Fahrbefehl-Mux ist aus (cmd_mux_enabled=false).')
            return

        # (Name, Topic, Timeout-Parameter) in Prioritaetsreihenfolge.
        self._mux_sources = [
            ('safe',  '/cmd_safe',  'cmd_timeout_safe'),
            ('avoid', '/cmd_avoid', 'cmd_timeout_avoid'),
            ('lane',  '/cmd_lane',  'cmd_timeout_lane'),
        ]
        # {name: (Twist, Empfangszeit)}
        self._mux_cmds = {}
        self._mux_subs = [
            self.create_subscription(
                Twist, topic, self._make_mux_cb(name), 1)
            for name, topic, _ in self._mux_sources
        ]

        # Ausgang bewusst /control/cmd_vel: das Remapping auf /cmd_vel steht im
        # Launch, damit der Node auch in Gazebo o.ae. nutzbar bleibt.
        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)
        self.pub_cmd_source = self.create_publisher(String, '/control/cmd_source', 1)

        self._mux_active_source = None
        rate = self.get_parameter('cmd_mux_rate').value
        self.mux_timer = self.create_timer(1.0 / rate, self.cbMux)
        self.get_logger().info(
            f'Fahrbefehl-Mux aktiv ({rate:.0f} Hz): '
            'safe > avoid > lane -> /control/cmd_vel')

    def _make_mux_cb(self, name):
        def cb(twist):
            self._mux_cmds[name] = (twist, self.get_clock().now())
        return cb

    def cbMux(self):
        """Hoechste Prioritaet mit aktueller Nachricht gewinnt.

        Kommt von keiner Quelle etwas Aktuelles, wird ein Nullbefehl gesendet.
        Das ist der sichere Ausgang: faellt z.B. detect_lane aus, bleibt der
        Roboter stehen, statt mit dem letzten Befehl weiterzufahren.
        """
        self._read_params()
        now = self.get_clock().now()

        chosen_name = None
        chosen_twist = None
        for name, _, timeout_param in self._mux_sources:
            entry = self._mux_cmds.get(name)
            if entry is None:
                continue
            twist, stamp = entry
            age = (now - stamp).nanoseconds / 1e9
            if age <= getattr(self, timeout_param):
                chosen_name, chosen_twist = name, twist
                break

        if chosen_twist is None:
            chosen_name = 'none'
            chosen_twist = Twist()

        self.pub_cmd_vel.publish(chosen_twist)

        if chosen_name != self._mux_active_source:
            self.get_logger().info(
                f'Fahrbefehl-Quelle: {self._mux_active_source} -> {chosen_name}')
            self._mux_active_source = chosen_name

        msg = String()
        msg.data = chosen_name
        self.pub_cmd_source.publish(msg)

    def cbPubControlMode(self):
        msg = UInt8()
        msg.data = self.current_control_mode.value
        self.pub_control.publish(msg)

    def cbSign(self, msg):
        try:
            sign = ControlType(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unbekanntes Schild-Signal: {msg.data}')
            return

        if sign == ControlType.Tunnel:
            if self.current_control_mode != ControlType.Tunnel:
                self._mode_before_tunnel = self.current_control_mode
                self.current_control_mode = ControlType.Tunnel
                self._tunnel_exit_since = None
                self.get_logger().info('Tunnel-Schild erkannt -> wechsle zu Tunnel-Modus.')
        else:
            self.get_logger().debug(
                f'Schild {sign.name} erkannt, dafuer ist noch keine Aktion definiert.')

    def cbLaneMask(self, lane_mask):
        self._read_params()
        total_pixels = len(lane_mask.yellow_x) + len(lane_mask.white_x)
        self._lane_visible = total_pixels >= self.tunnel_exit_min_lane_pixels

    def cbAvoidTakeover(self, msg):
        """Uebernahmemeldung von avoid_node.

        Wirkt nur bei obstacle_source='avoid'; mit 'scan' bleibt der alte
        LiDAR-Pfad allein zustaendig.
        """
        self._avoid_takeover = bool(msg.data)

        if self.get_parameter('obstacle_source').value != 'avoid':
            return
        if self.current_control_mode not in (ControlType.Lane, ControlType.Obstacle):
            # Im Tunnel- oder Stop-Modus nicht dazwischenfunken.
            return

        if self._avoid_takeover and self.current_control_mode == ControlType.Lane:
            self.current_control_mode = ControlType.Obstacle
            self.get_logger().info('avoid_node uebernimmt -> wechsle zu Obstacle-Modus.')
        elif not self._avoid_takeover and self.current_control_mode == ControlType.Obstacle:
            # avoid_node entprellt die Freigabe bereits selbst (Hysterese +
            # takeover_release_debounce), hier ist keine zweite Entprellung noetig.
            self.current_control_mode = ControlType.Lane
            self.get_logger().info('avoid_node gibt ab -> zurueck zu Lane-Modus.')

    def cbScan(self, scan):
        self._read_params()

        if self.current_control_mode == ControlType.Tunnel:
            self._update_tunnel_exit(scan)
        elif self.current_control_mode in (ControlType.Lane, ControlType.Obstacle):
            if self.obstacle_source != 'avoid':
                self._update_obstacle_mode(scan)
        # Andere Modi (Stop/Intersection/Align): hier (noch) keine automatische Logik.

    def _update_obstacle_mode(self, scan):
        obstacle_present = self._forward_obstacle_present(scan)

        if obstacle_present:
            self._obstacle_clear_since = None
            if self.current_control_mode == ControlType.Lane:
                self.current_control_mode = ControlType.Obstacle
                self.get_logger().info('Hindernis in der Fahrbahn erkannt -> wechsle zu Obstacle-Modus.')
        elif self.current_control_mode == ControlType.Obstacle:
            now = self.get_clock().now()
            if self._obstacle_clear_since is None:
                self._obstacle_clear_since = now
            elif (now - self._obstacle_clear_since).nanoseconds / 1e9 >= self.obstacle_clear_debounce:
                self.current_control_mode = ControlType.Lane
                self._obstacle_clear_since = None
                self.get_logger().info('Kein Hindernis mehr erkannt -> zurueck zu Lane-Modus.')

    def _forward_obstacle_present(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        if ranges.size == 0:
            return False

        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        # Auf [-pi, pi] normalisieren, damit 0 rad wirklich geradeaus ist
        angles = np.arctan2(np.sin(angles), np.cos(angles))

        # Halbwinkel: der Kegel ist doppelt so breit wie der Parameterwert.
        half_angle = math.radians(self.obstacle_check_angle_deg)
        in_cone = np.abs(angles) <= half_angle
        valid = np.isfinite(ranges) & (ranges > scan.range_min)
        close = ranges < self.obstacle_trigger_distance

        return bool(np.any(in_cone & valid & close))

    def _update_tunnel_exit(self, scan):
        scan_open = self._is_open_space(scan)

        if scan_open and self._lane_visible:
            now = self.get_clock().now()
            if self._tunnel_exit_since is None:
                self._tunnel_exit_since = now
            elif (now - self._tunnel_exit_since).nanoseconds / 1e9 >= self.tunnel_exit_debounce:
                self._leave_tunnel()
        else:
            self._tunnel_exit_since = None

    def _is_open_space(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        if ranges.size == 0:
            return False

        # inf = kein Echo = frei. Nicht als ungueltig verwerfen, sonst bleiben nur
        # die kurzen Messwerte uebrig und der Tunnel liesse sich nie verlassen.
        no_echo = ~np.isfinite(ranges)
        far = np.isfinite(ranges) & (ranges > self.tunnel_exit_open_distance)
        # Zu kurze Werte sind Artefakte, nicht "frei"
        too_close = np.isfinite(ranges) & (ranges <= scan.range_min)

        open_ratio = np.count_nonzero(no_echo | far) / ranges.size
        if np.count_nonzero(too_close) == ranges.size:
            return False
        return open_ratio >= self.tunnel_exit_open_ratio

    def _leave_tunnel(self):
        restored = self._mode_before_tunnel or ControlType.Lane
        self.get_logger().info(f'Tunnel verlassen -> zurueck zu {restored.name}.')
        self.current_control_mode = restored
        self._mode_before_tunnel = None
        self._tunnel_exit_since = None


def main(args=None):
    rclpy.init(args=args)
    node = switch_control()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Laeuft der Mux, ist dieser Node der einzige Publisher auf /cmd_vel --
        # ohne Nullbefehl faehrt der Roboter mit dem letzten Wert weiter.
        if node._mux_enabled:
            node.pub_cmd_vel.publish(Twist())

        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
