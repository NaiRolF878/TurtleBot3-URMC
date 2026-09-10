"""Zonenbasierte Hindernisvermeidung auf Basis des 360-Grad-LiDAR.

Zwei Betriebsarten, die sich gegenseitig ausschliessen:

1. Normalbetrieb (PID-Offset):
   Der Spur-PID faehrt weiter. Dieser Node meldet nur einen Offset auf
   /avoid/offset, den control_driving auf den Spur-Error addiert. Der Roboter
   weicht dadurch innerhalb der Spur aus, waehrend die Spurerkennung
   weiterlaeuft und verhindert, dass die gelbe Linie ueberfahren wird.

2. Uebernahme (Takeover):
   Ist die Frontzone komplett blockiert (< takeover_distance), reicht ein
   Offset nicht mehr. Dann meldet dieser Node ueber /avoid/takeover, dass er
   uebernimmt, und faehrt selbst ueber /cmd_avoid. switch_control schaltet
   dabei auf den Obstacle-Modus (Prioritaet 2 im Mux), womit control_driving
   automatisch verstummt.

Bewusst wird /cmd_avoid NUR waehrend der Uebernahme bespielt: der Mux in
switch_control entscheidet anhand der Aktualitaet der Nachrichten, wer fahren
darf. Wuerde dieser Node dauerhaft senden, wuerde er dem PID staendig das
Steuer entreissen.

Zonen sind in Grad konfiguriert, gemessen gegen den Uhrzeigersinn ab
Fahrtrichtung (0 Grad = geradeaus, 90 Grad = links). Ein Bereich, dessen
Startwinkel groesser als sein Endwinkel ist, laeuft ueber die 360/0-Grenze
hinweg (z.B. die Frontzone 350..10).
"""

import math
import signal

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String


# Die Namen tauchen als zone_<name> und cost_<name> in den Parametern und in
# den Logmeldungen wieder auf.
ZONE_NAMES = ('front', 'front_left', 'front_right', 'left', 'right')


class avoid_node(Node):
    def __init__(self):
        super().__init__('avoid_node')

        self.get_logger().info('Started avoid_node.')

        # step=0.0: rclpy prueft sonst (wert-from) % step == 0, was mit Fliesskomma
        # zufaellig fehlschlaegt.
        def frange(lo, hi):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

        # ---------------------------------
        # Parameter
        # ---------------------------------

        # --- Zonen: [start_deg, end_deg], start > end laeuft ueber 360 Grad ---
        self.declare_parameter('zone_front',       [350.0, 10.0])
        self.declare_parameter('zone_front_left',  [10.0, 50.0])
        self.declare_parameter('zone_front_right', [310.0, 350.0])
        self.declare_parameter('zone_left',        [50.0, 90.0])
        self.declare_parameter('zone_right',       [270.0, 310.0])

        # --- Kosten pro Zone ---
        # Vorzeichen = Drehrichtung (positiv = links), Betrag = Kosten. Ausgewaehlt
        # wird der freie Sektor mit dem KLEINSTEN BETRAG, deshalb sind -1 und +1
        # gleich teuer und nur die Richtung unterscheidet sich. Ohne den Betrag
        # wuerde der billigste Sektor immer der mit dem negativsten Wert sein,
        # der Roboter also grundsaetzlich nach rechts ausweichen.
        self.declare_parameter('cost_front',        0.0, frange(-10.0, 10.0))
        self.declare_parameter('cost_front_left',   1.0, frange(-10.0, 10.0))
        self.declare_parameter('cost_front_right', -1.0, frange(-10.0, 10.0))
        self.declare_parameter('cost_left',         2.0, frange(-10.0, 10.0))
        self.declare_parameter('cost_right',       -2.0, frange(-10.0, 10.0))

        # --- Schwellwerte ---
        # Ab hier gilt die Frontzone als blockiert und es wird gedreht.
        self.declare_parameter('front_threshold',           0.35, frange(0.05, 3.0))
        # Ein Sektor gilt als "frei", wenn seine kleinste Messung darueber liegt.
        self.declare_parameter('clear_distance',            0.50, frange(0.05, 5.0))
        # Darunter reicht kein Offset mehr -> komplette Uebernahme.
        self.declare_parameter('takeover_distance',         0.20, frange(0.05, 3.0))
        # Hysterese: erst ab dieser Distanz wird die Uebernahme wieder abgegeben.
        self.declare_parameter('takeover_release_distance', 0.45, frange(0.05, 5.0))
        self.declare_parameter('takeover_release_debounce', 1.0,  frange(0.0, 10.0))
        # Messwerte darunter sind Artefakte (der LDS liefert statt "kein Echo"
        # gelegentlich 0.0) -- ohne den Filter meldet jede Zone dauerhaft 0.0 m.
        self.declare_parameter('min_valid_distance',        0.05, frange(0.0, 1.0))

        # --- Offset-Regelung (Normalbetrieb) ---
        # Ab dieser Distanz beginnt der Offset zu wirken ...
        self.declare_parameter('offset_trigger_distance',    0.80, frange(0.05, 5.0))
        # ... und ab dieser wirkt er voll.
        self.declare_parameter('offset_saturation_distance', 0.25, frange(0.02, 5.0))
        self.declare_parameter('offset_gain',                0.6,  frange(0.0, 5.0))
        self.declare_parameter('offset_max',                 0.6,  frange(0.0, 5.0))
        # Vorzeichenkonvention von detect_lane: lane_error = 1 - 2*lane_center/breite,
        # ein POSITIVER Error laesst control_driving nach LINKS lenken. Ein
        # Hindernis rechts braucht also einen positiven Offset. Wer die Konvention
        # in detect_lane dreht, stellt hier auf -1.0.
        self.declare_parameter('offset_direction', 1.0, frange(-1.0, 1.0))

        # --- Geschwindigkeiten waehrend der Uebernahme ---
        self.declare_parameter('avoid_linear_speed',  0.06, frange(0.0, 0.5))
        self.declare_parameter('avoid_angular_speed', 0.7,  frange(0.0, 3.0))
        # Ist die Front so dicht, dass Vorwaertsfahren gefaehrlich ist: auf der
        # Stelle drehen statt schieben.
        self.declare_parameter('avoid_turn_only_distance', 0.15, frange(0.0, 2.0))

        self._param_names = (
            [f'zone_{n}' for n in ZONE_NAMES]
            + [f'cost_{n}' for n in ZONE_NAMES]
            + ['front_threshold', 'clear_distance', 'takeover_distance',
               'takeover_release_distance', 'takeover_release_debounce',
               'min_valid_distance',
               'offset_trigger_distance', 'offset_saturation_distance',
               'offset_gain', 'offset_max', 'offset_direction',
               'avoid_linear_speed', 'avoid_angular_speed',
               'avoid_turn_only_distance']
        )

        # --- Subscriber ---
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cbScan, qos_profile_sensor_data)

        # --- Publisher ---
        self.pub_offset = self.create_publisher(Float64, '/avoid/offset', 1)
        self.pub_cmd_avoid = self.create_publisher(Twist, '/cmd_avoid', 1)
        self.pub_takeover = self.create_publisher(Bool, '/avoid/takeover', 1)
        # Reine Diagnose ("ros2 topic echo /avoid/zones"), kostet nichts.
        self.pub_zone_info = self.create_publisher(String, '/avoid/zones', 1)

        # --- Zustand ---
        self._takeover = False
        self._release_since = None
        # Die Winkeltabelle wird pro Scan-Geometrie einmal gebaut, nicht pro
        # Nachricht -- sonst liefe das bei 5-10 Hz staendig unnoetig mit.
        self._angle_cache_key = None
        self._zone_masks = {}
        self._zone_param_cache = None

    # ---------------------------------
    # Parameter
    # ---------------------------------

    def _read_params(self):
        # declare_parameter legt keine self.<name>-Attribute an -> hier einlesen.
        # Gleichzeitig wirken so zur Laufzeit geaenderte Parameter sofort.
        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

    # ---------------------------------
    # Zonen
    # ---------------------------------

    def _zone_bounds(self):
        """Aktuelle Zonengrenzen als {name: (start_deg, end_deg)}."""
        return {name: tuple(getattr(self, f'zone_{name}')) for name in ZONE_NAMES}

    def _build_zone_masks(self, scan):
        """Boolesche Maske je Zone ueber die Indizes des Scans.

        Wird nur neu gebaut, wenn sich die Scan-Geometrie oder eine Zonengrenze
        aendert.
        """
        n = len(scan.ranges)
        key = (n, scan.angle_min, scan.angle_increment)
        bounds = self._zone_bounds()

        if key == self._angle_cache_key and bounds == self._zone_param_cache:
            return

        angles_deg = np.degrees(scan.angle_min + np.arange(n) * scan.angle_increment) % 360.0

        masks = {}
        for name, (start, end) in bounds.items():
            start %= 360.0
            end %= 360.0
            if start <= end:
                masks[name] = (angles_deg >= start) & (angles_deg <= end)
            else:
                # Bereich laeuft ueber die 360/0-Grenze, z.B. die Frontzone 350..10
                masks[name] = (angles_deg >= start) | (angles_deg <= end)

        self._zone_masks = masks
        self._angle_cache_key = key
        self._zone_param_cache = bounds

    def _zone_min_distances(self, scan):
        """Kleinste gueltige Distanz je Zone; inf, wenn die Zone frei ist."""
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        # inf/nan = kein Echo = frei. Nur zu kurze Werte sind Artefakte.
        floor = max(scan.range_min, float(self.min_valid_distance))
        valid = np.isfinite(ranges) & (ranges > floor)

        distances = {}
        for name, mask in self._zone_masks.items():
            selected = ranges[mask & valid]
            distances[name] = float(np.min(selected)) if selected.size else math.inf
        return distances

    # ---------------------------------
    # Hauptschleife
    # ---------------------------------

    def cbScan(self, scan):
        if not scan.ranges:
            return

        self._read_params()
        self._build_zone_masks(scan)
        distances = self._zone_min_distances(scan)

        self._update_takeover(distances)

        # Der Offset wird IMMER gesendet, auch waehrend der Uebernahme. Ihn dort
        # auf 0 zu zwingen waere ein Sprung im Sollwert: zwischen /avoid/takeover
        # und dem /control/mode-Wechsel liegen ein bis zwei Nachrichten, und in
        # diesem Fenster rechnet control_driving noch. Der D-Anteil (kd/0.1)
        # macht aus einem Sprung von 0.6 sofort einen Lenkausschlag bis in die
        # +/-2.0-Begrenzung. Ueber die Distanz laeuft der Offset dagegen stetig
        # hoch, und im Obstacle-Modus ignoriert control_driving ihn ohnehin.
        self._publish_offset(self._compute_offset(distances))

        if self._takeover:
            self._drive_avoid(distances)

        self._publish_diagnostics(distances)

    def _update_takeover(self, distances):
        """Uebernahme mit Hysterese: rein bei < takeover_distance, raus erst
        wieder, wenn die Front laenger als takeover_release_debounce frei ist.
        Ohne die Hysterese wuerde der Node an der Schwelle im Sekundentakt
        zwischen PID und Eigenfahrt springen."""
        front = distances['front']
        now = self.get_clock().now()

        if not self._takeover:
            if front < self.takeover_distance:
                self._takeover = True
                self._release_since = None
                self.get_logger().warn(
                    f'Frontzone blockiert ({front:.2f} m) -> avoid_node uebernimmt.')
            self._publish_takeover()
            return

        if front > self.takeover_release_distance:
            if self._release_since is None:
                self._release_since = now
            elif (now - self._release_since).nanoseconds / 1e9 >= self.takeover_release_debounce:
                self._takeover = False
                self._release_since = None
                self.get_logger().info(
                    f'Front wieder frei ({front:.2f} m) -> zurueck zur Spurfolge.')
        else:
            self._release_since = None

        self._publish_takeover()

    # ---------------------------------
    # Normalbetrieb: Offset auf den Spur-Error
    # ---------------------------------

    def _proximity(self, distance):
        """0.0 = kein Hindernis in Reichweite, 1.0 = an der Saettigungsgrenze.

        Damit ist der Offset proportional zur Naehe: je dichter das Hindernis,
        desto staerker das Ausweichen.
        """
        trigger = float(self.offset_trigger_distance)
        # Saettigung kann durch freie Parametrierung ueber dem Ausloeseabstand
        # liegen -- dann waere der Nenner unten negativ.
        saturation = min(float(self.offset_saturation_distance), trigger)

        if not math.isfinite(distance) or distance >= trigger:
            return 0.0
        if distance <= saturation or trigger == saturation:
            return 1.0
        return (trigger - distance) / (trigger - saturation)

    def _compute_offset(self, distances):
        """Offset auf den Spur-Error.

        Hindernis rechts (front_right) -> nach links ausweichen,
        Hindernis links  (front_left)  -> nach rechts ausweichen.
        Sind beide Seiten belegt, heben sich die Anteile teilweise auf und der
        Roboter faehrt mittig zwischen den Hindernissen durch.
        """
        p_right = self._proximity(distances['front_right'])
        p_left = self._proximity(distances['front_left'])

        offset = self.offset_gain * (p_right - p_left) * self.offset_direction
        limit = float(self.offset_max)
        return float(max(-limit, min(limit, offset)))

    def _publish_offset(self, value):
        msg = Float64()
        msg.data = float(value)
        self.pub_offset.publish(msg)

    def _publish_takeover(self):
        msg = Bool()
        msg.data = self._takeover
        self.pub_takeover.publish(msg)

    # ---------------------------------
    # Uebernahme: eigener Fahrbefehl
    # ---------------------------------

    def _select_sector(self, distances):
        """Bester Ausweichsektor als (name, kosten, ist_frei).

        Zuerst nur die freien Sektoren (kleinste Distanz >= clear_distance),
        davon der mit den niedrigsten Kosten (Betrag). Bei Gleichstand gewinnt
        der Sektor mit mehr Platz. Ist kein Sektor frei, wird der mit der
        groessten gemessenen Distanz genommen -- lieber der beste Kompromiss
        als stehen bleiben.
        """
        costs = {name: float(getattr(self, f'cost_{name}')) for name in ZONE_NAMES}

        free = [n for n in ZONE_NAMES if distances[n] >= self.clear_distance]
        if free:
            # -distances: bei gleichen Kosten gewinnt die groessere Distanz.
            best = min(free, key=lambda n: (abs(costs[n]), -distances[n]))
            return best, costs[best], True

        best = max(ZONE_NAMES, key=lambda n: distances[n])
        return best, costs[best], False

    def _drive_avoid(self, distances):
        sector, cost, is_free = self._select_sector(distances)

        # Vorzeichen der Kosten = Drehrichtung (positiv = links). Die Frontzone
        # hat Kosten 0 -> geradeaus.
        direction = 0.0
        if cost > 0:
            direction = 1.0
        elif cost < 0:
            direction = -1.0

        twist = Twist()
        too_close = distances['front'] <= self.avoid_turn_only_distance
        # Kein freier Sektor und keine Drehrichtung: nicht blind vorwaerts.
        stuck = (not is_free) and direction == 0.0
        twist.linear.x = 0.0 if (too_close or stuck) else float(self.avoid_linear_speed)
        twist.angular.z = float(direction * self.avoid_angular_speed)

        self.pub_cmd_avoid.publish(twist)

        self.get_logger().debug(
            f'Ausweichen ueber "{sector}" (Kosten {cost:+.1f}, '
            f'{"frei" if is_free else "Notloesung"}, {distances[sector]:.2f} m)')

    # ---------------------------------
    # Diagnose
    # ---------------------------------

    def _publish_diagnostics(self, distances):
        parts = []
        for name in ZONE_NAMES:
            d = distances[name]
            value = 'inf' if not math.isfinite(d) else format(d, '.2f')
            parts.append(f'{name}={value}')
        msg = String()
        msg.data = ('takeover ' if self._takeover else 'offset   ') + ' '.join(parts)
        self.pub_zone_info.publish(msg)

    def shut_down(self):
        """Beim Beenden den Offset zuruecknehmen und die Uebernahme abgeben,
        damit control_driving nicht mit dem letzten Wert weiterrechnet."""
        self.get_logger().info('Shutting down avoid_node.')
        self._takeover = False
        self._publish_offset(0.0)
        self._publish_takeover()
        self.pub_cmd_avoid.publish(Twist())


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = avoid_node()

    stop = False

    def on_signal(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.shut_down()
        node.destroy_node()
        # rclpy raeumt den Context im eigenen SIGINT-Handler ab -- sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
