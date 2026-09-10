"""Fahrmodus und Fahrbefehl-Mux -- die Schaltzentrale der Pipeline.

Zwei Aufgaben:

1. Zustandsmaschine fuer /control/mode (einziger Publisher dieses Topics).
   Lane <-> Obstacle folgt dem, was avoid_node auf /avoid/takeover meldet:
   solange der Offset reicht, faehrt der PID (Lane); uebernimmt avoid_node,
   geht es auf Obstacle und control_driving verstummt von selbst.

2. Prioritaets-Mux fuer die Fahrbefehle:
       1. /cmd_safe  -- Not-Aus aus safety_node
       2. /cmd_avoid -- komplettes Ausweichen aus avoid_node
       3. /cmd_lane  -- Spurfolge aus control_driving
   Es gewinnt die hoechste Prioritaet, von der noch aktuelle Nachrichten
   eintreffen. Der Ausgang liegt auf /control/cmd_vel und wird im Launch auf
   /cmd_vel gemappt -- so bleibt der Node auch in Gazebo o.ae. nutzbar.

Die Aktualitaetspruefung ist der Kern: safety_node und avoid_node senden nur,
solange sie zustaendig sind. Hoert eine Quelle auf, faellt der Mux von selbst
auf die naechste zurueck, ohne dass jemand ihn umschalten muesste. Kommt von
gar keiner Quelle etwas, geht ein Nullbefehl raus statt des letzten
Fahrbefehls.
"""

from enum import Enum

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from std_msgs.msg import Bool, String, UInt8
from geometry_msgs.msg import Twist


class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Stop = 3


class switch_control(Node):

    def __init__(self):
        super().__init__('switch_control')

        self.get_logger().info('Started switch_control Node.')

        # step=0.0: rclpy prueft sonst (wert-from) % step == 0, was mit Fliesskomma
        # zufaellig fehlschlaegt.
        def frange(lo, hi):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

        # --- Fahrbefehl-Mux ---
        self.declare_parameter('cmd_mux_rate', 20.0, frange(1.0, 100.0))
        # Aktualitaetsfenster je Quelle: aeltere Nachrichten zaehlen nicht mehr.
        self.declare_parameter('cmd_timeout_safe',  0.5, frange(0.05, 5.0))
        self.declare_parameter('cmd_timeout_avoid', 0.5, frange(0.05, 5.0))
        self.declare_parameter('cmd_timeout_lane',  0.5, frange(0.05, 5.0))

        self._param_names = [
            'cmd_timeout_safe', 'cmd_timeout_avoid', 'cmd_timeout_lane',
        ]

        # --- Subscriber ---
        self.sub_takeover = self.create_subscription(
            Bool, '/avoid/takeover', self.cbAvoidTakeover, 1)

        # --- Publisher ---
        self.pub_control = self.create_publisher(UInt8, '/control/mode', 1)

        # Zyklisch senden, damit auch spaeter startende Nodes den Mode bekommen.
        self.timer = self.create_timer(0.5, self.cbPubControlMode)

        self.current_control_mode = ControlType.Lane
        self._avoid_takeover = False

        self._setup_mux()

    def _read_params(self):
        # declare_parameter legt keine self.<name>-Attribute an -> hier einlesen.
        # Gleichzeitig wirken so zur Laufzeit geaenderte Parameter sofort.
        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

    # ---------------------------------
    # Fahrmodus
    # ---------------------------------

    def cbPubControlMode(self):
        msg = UInt8()
        msg.data = self.current_control_mode.value
        self.pub_control.publish(msg)

    def cbAvoidTakeover(self, msg):
        """Uebernahmemeldung von avoid_node.

        avoid_node entprellt die Freigabe bereits selbst (Hysterese +
        takeover_release_debounce), hier ist keine zweite Entprellung noetig.
        """
        self._avoid_takeover = bool(msg.data)

        if self.current_control_mode == ControlType.Stop:
            # Ein von aussen gesetzter Stopp hat Vorrang.
            return

        if self._avoid_takeover and self.current_control_mode == ControlType.Lane:
            self.current_control_mode = ControlType.Obstacle
            self.get_logger().info('avoid_node uebernimmt -> wechsle zu Obstacle-Modus.')
        elif not self._avoid_takeover and self.current_control_mode == ControlType.Obstacle:
            self.current_control_mode = ControlType.Lane
            self.get_logger().info('avoid_node gibt ab -> zurueck zu Lane-Modus.')

    # ---------------------------------
    # Fahrbefehl-Mux
    # ---------------------------------

    def _setup_mux(self):
        # (Name, Topic, Timeout-Parameter) in Prioritaetsreihenfolge.
        self._mux_sources = [
            ('safe',  '/cmd_safe',  'cmd_timeout_safe'),
            ('avoid', '/cmd_avoid', 'cmd_timeout_avoid'),
            ('lane',  '/cmd_lane',  'cmd_timeout_lane'),
        ]
        # {name: (Twist, Empfangszeit)}
        self._mux_cmds = {}
        self._mux_subs = [
            self.create_subscription(Twist, topic, self._make_mux_cb(name), 1)
            for name, topic, _ in self._mux_sources
        ]

        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)
        self.pub_cmd_source = self.create_publisher(String, '/control/cmd_source', 1)

        self._mux_active_source = None
        rate = self.get_parameter('cmd_mux_rate').value
        self.mux_timer = self.create_timer(1.0 / rate, self.cbMux)
        self.get_logger().info(
            f'Fahrbefehl-Mux aktiv ({rate:.0f} Hz): safe > avoid > lane -> /control/cmd_vel')

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


def main(args=None):
    rclpy.init(args=args)
    node = switch_control()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Dieser Node ist der einzige Publisher auf /cmd_vel -- ohne Nullbefehl
        # faehrt der Roboter mit dem letzten Wert weiter.
        node.pub_cmd_vel.publish(Twist())

        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
