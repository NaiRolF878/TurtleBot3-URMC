"""Not-Aus per LiDAR -- hoechste Prioritaet im Mux von switch_control.

Sobald irgendeine gueltige Messung naeher als emergency_distance liegt, geht
ein Twist mit lauter Nullen auf /cmd_safe. Der Mux in switch_control bevorzugt
dieses Topic vor allem anderen, solange Nachrichten eintreffen.

Deshalb wird /cmd_safe NUR im ausgeloesten Zustand bespielt: der Mux erkennt am
Ausbleiben der Nachrichten, dass er wieder auf die naechstniedrigere Quelle
zurueckfallen darf. Ein dauerhaft sendender Not-Aus wuerde den Roboter fuer
immer anhalten.

Den Fall "es kommt gar kein Fahrbefehl mehr" deckt dieser Node NICHT ab -- das
erledigt der Mux in switch_control, der auf einen Nullbefehl faellt, sobald von
allen Quellen nichts Aktuelles mehr eintrifft.
"""

import math
import signal

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class safety_node(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.get_logger().info('Started safety_node.')

        # step=0.0: rclpy prueft sonst (wert-from) % step == 0, was mit Fliesskomma
        # zufaellig fehlschlaegt.
        def frange(lo, hi):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

        # ---------------------------------
        # Parameter
        # ---------------------------------

        # Ausloeseschwelle: alles darunter loest den Not-Aus aus.
        self.declare_parameter('emergency_distance', 0.15, frange(0.02, 2.0))
        # Hysterese: erst darueber gilt die Lage wieder als sicher. Muss groesser
        # als emergency_distance sein, sonst flattert der Not-Aus an der Schwelle.
        self.declare_parameter('release_distance',   0.20, frange(0.02, 3.0))
        # Mindestdauer des Stopps, auch wenn der Messwert sofort wieder frei ist
        # (der LDS liefert bei glaenzenden Oberflaechen einzelne Aussetzer).
        self.declare_parameter('min_stop_duration',  0.5,  frange(0.0, 10.0))
        # Messwerte darunter sind Artefakte, kein Hindernis.
        self.declare_parameter('min_valid_distance', 0.05, frange(0.0, 1.0))
        # Halbwinkel des ueberwachten Sektors um die Fahrtrichtung. 180.0 = der
        # komplette Kreis. Kleiner stellen, falls Aufbauten am Roboter (Kamera-
        # halter, Kabel) den Not-Aus faelschlich ausloesen.
        self.declare_parameter('check_angle_deg',    180.0, frange(1.0, 180.0))
        # Mindestzahl an Messpunkten unter der Schwelle. 1 ist empfindlich, aber
        # korrekt; 2-3 unterdrueckt einzelne Ausreisser.
        self.declare_parameter('min_hits',           2,
            ParameterDescriptor(integer_range=[
                IntegerRange(from_value=1, to_value=50, step=1)]))

        self._param_names = [
            'emergency_distance', 'release_distance', 'min_stop_duration',
            'min_valid_distance', 'check_angle_deg', 'min_hits',
        ]

        # --- Subscriber ---
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cbScan, qos_profile_sensor_data)

        # --- Publisher ---
        self.pub_cmd_safe = self.create_publisher(Twist, '/cmd_safe', 1)
        # Damit debug_view/rqt sehen, warum der Roboter steht.
        self.pub_emergency = self.create_publisher(Bool, '/safety/emergency', 1)

        # --- Zustand ---
        self._emergency = False
        self._stop_since = None
        self._band_warned = False

    def _read_params(self):
        for name in self._param_names:
            setattr(self, name, self.get_parameter(name).value)

    def cbScan(self, scan):
        if not scan.ranges:
            return

        self._read_params()
        self._warn_about_detection_band(scan)

        closest = self._closest_in_sector(scan)
        self._update_state(closest)

        if self._emergency:
            # Wiederholt senden, solange der Not-Aus steht: der Mux prueft die
            # Aktualitaet, eine einzelne Nachricht wuerde veralten.
            self.pub_cmd_safe.publish(Twist())

        msg = Bool()
        msg.data = self._emergency
        self.pub_emergency.publish(msg)

    def _warn_about_detection_band(self, scan):
        """Einmalig warnen, wenn zwischen range_min und emergency_distance kaum
        Platz bleibt.

        Der LDS-01 des Burger meldet range_min = 0.12 m. Alles darunter ist
        unbrauchbar und wird verworfen -- bei emergency_distance = 0.15 m bleibt
        also nur ein 3 cm breites Band, in dem der Not-Aus ueberhaupt ausloesen
        kann. Ein Hindernis, das schneller als eine Scan-Umdrehung durch dieses
        Band faehrt, wird nicht gesehen. Das ist keine Fehlfunktion dieses
        Nodes, muss aber beim Einstellen bekannt sein.
        """
        if self._band_warned:
            return
        self._band_warned = True

        floor = max(scan.range_min, float(self.min_valid_distance))
        band = float(self.emergency_distance) - floor

        if band <= 0.0:
            self.get_logger().error(
                f'emergency_distance ({self.emergency_distance:.2f} m) liegt unter der '
                f'kleinsten verwertbaren Messung ({floor:.2f} m) -- der Not-Aus kann '
                'NIE ausloesen. Wert erhoehen!')
        elif band < 0.05:
            self.get_logger().warn(
                f'Nur {band * 100:.0f} cm zwischen kleinster verwertbarer Messung '
                f'({floor:.2f} m) und emergency_distance ({self.emergency_distance:.2f} m). '
                'Fuer verlaessliches Ausloesen emergency_distance erhoehen '
                f'(Vorschlag: >= {floor + 0.08:.2f} m).')

    def _closest_in_sector(self, scan):
        """Kleinste gueltige Messung im ueberwachten Sektor, sonst inf."""
        ranges = np.asarray(scan.ranges, dtype=np.float64)

        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        # Auf [-pi, pi] normalisieren, damit 0 rad wirklich geradeaus ist.
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        in_sector = np.abs(angles) <= math.radians(self.check_angle_deg)

        # inf/nan = kein Echo = frei; zu kurze Werte sind Artefakte.
        floor = max(scan.range_min, float(self.min_valid_distance))
        valid = np.isfinite(ranges) & (ranges > floor)

        selected = ranges[in_sector & valid]
        if selected.size == 0:
            return math.inf

        # Nicht das absolute Minimum nehmen, sondern den min_hits-kleinsten Wert:
        # ein einzelner Ausreisser loest so keinen Not-Aus aus.
        hits = max(1, int(self.min_hits))
        if selected.size < hits:
            return math.inf
        return float(np.partition(selected, hits - 1)[hits - 1])

    def _update_state(self, closest):
        now = self.get_clock().now()

        if not self._emergency:
            if closest < self.emergency_distance:
                self._emergency = True
                self._stop_since = now
                self.get_logger().error(
                    f'NOT-AUS: Hindernis bei {closest:.2f} m '
                    f'(< {self.emergency_distance:.2f} m).')
            return

        # Mindesthaltezeit abwarten, bevor ueberhaupt freigegeben werden darf.
        held = (now - self._stop_since).nanoseconds / 1e9 if self._stop_since else 0.0
        if held < self.min_stop_duration:
            return

        # release_distance liegt normalerweise ueber emergency_distance; ist es
        # falsch parametriert, faellt der Not-Aus auf die Ausloeseschwelle zurueck.
        release = max(float(self.release_distance), float(self.emergency_distance))
        if closest > release:
            self._emergency = False
            self._stop_since = None
            self.get_logger().info(
                f'Not-Aus aufgehoben, naechstes Hindernis bei {closest:.2f} m.')

    def shut_down(self):
        """Beim Beenden noch einmal einen Nullbefehl schicken -- so bleibt der
        Roboter stehen, falls der Prozess mitten im Not-Aus abgeraeumt wird."""
        self.get_logger().info('Shutting down safety_node.')
        for _ in range(5):
            self.pub_cmd_safe.publish(Twist())


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = safety_node()

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
