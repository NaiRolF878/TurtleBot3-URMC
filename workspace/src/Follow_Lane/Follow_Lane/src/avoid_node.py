"""Zonenauswertung des 360-Grad-LiDAR fuer das Ausweichen, faehrt nicht selbst.

Ein:  /scan (LaserScan)
Aus:  /avoid/zones (AvoidZones): Distanz je Zone, Offset fuer den Lane-Modus,
      Ausweichrichtung fuer den Obstacle-Modus
Ob ausgewichen wird, entscheidet switch_control, den Fahrbefehl rechnet control_driving.
"""
import math

import numpy as np
from follow_lane_msg.msg import AvoidZones
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from Follow_Lane.src.common import frange, read_params, run_node

# Die Namen tauchen als zone_<name> und cost_<name> in den Parametern und als
# Felder von AvoidZones wieder auf.
ZONE_NAMES = ('front', 'front_left', 'front_right', 'left', 'right')


class avoid_node(Node):
    """Zonen in Grad gegen den Uhrzeigersinn ab Fahrtrichtung (0 = geradeaus,
    90 = links). Ist der Startwinkel groesser als der Endwinkel, laeuft die Zone
    ueber die 360/0-Grenze (Frontzone 350..10).
    """

    def __init__(self):
        super().__init__('avoid_node')

        # --- Parameter ---
        # Zonen: [start_deg, end_deg]
        self.declare_parameter('zone_front',       [350.0, 10.0])
        self.declare_parameter('zone_front_left',  [10.0, 50.0])
        self.declare_parameter('zone_front_right', [310.0, 350.0])
        self.declare_parameter('zone_left',        [50.0, 90.0])
        self.declare_parameter('zone_right',       [270.0, 310.0])
        # Vorzeichen = Drehrichtung (positiv = links), Betrag = Kosten. Gewaehlt wird
        # der freie Sektor mit dem kleinsten Betrag, -1 und +1 sind also gleich teuer.
        # Ohne den Betrag gewaenne immer der negativste Wert, der Bot wiche
        # grundsaetzlich nach rechts aus.
        self.declare_parameter('cost_front',        0.0, frange(-10.0, 10.0))
        self.declare_parameter('cost_front_left',   1.0, frange(-10.0, 10.0))
        self.declare_parameter('cost_front_right', -1.0, frange(-10.0, 10.0))
        self.declare_parameter('cost_left',         2.0, frange(-10.0, 10.0))
        self.declare_parameter('cost_right',       -2.0, frange(-10.0, 10.0))
        # Ein Sektor gilt als frei, wenn seine kleinste Messung darueber liegt.
        self.declare_parameter('clear_distance',            0.50, frange(0.05, 5.0))
        # Naeher als das voraus: nur noch auf der Stelle drehen.
        self.declare_parameter('avoid_turn_only_distance',  0.15, frange(0.0, 2.0))
        # Der LDS meldet statt "kein Echo" gelegentlich 0.0; ohne den Filter laege
        # jede Zone dauerhaft bei 0 m.
        self.declare_parameter('min_valid_distance',        0.05, frange(0.0, 1.0))
        # Offset: wirkt ab offset_trigger_distance, voll ab offset_saturation_distance.
        self.declare_parameter('offset_trigger_distance',    0.80, frange(0.05, 5.0))
        self.declare_parameter('offset_saturation_distance', 0.25, frange(0.02, 5.0))
        self.declare_parameter('offset_gain',                0.6,  frange(0.0, 5.0))
        self.declare_parameter('offset_max',                 0.6,  frange(0.0, 5.0))
        # 1.0 passt zu detect_lane (positiver Spurfehler = nach links): Hindernis
        # rechts -> positiver Offset. -1.0 nur, wenn detect_lane die Konvention dreht.
        self.declare_parameter('offset_direction', 1.0, frange(-1.0, 1.0))
        self._param_names = (
            [f'zone_{n}' for n in ZONE_NAMES]
            + [f'cost_{n}' for n in ZONE_NAMES]
            + ['clear_distance', 'avoid_turn_only_distance', 'min_valid_distance',
               'offset_trigger_distance', 'offset_saturation_distance',
               'offset_gain', 'offset_max', 'offset_direction']
        )
        read_params(self, self._param_names)

        # --- Subscriber ---
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)

        # --- Publisher ---
        self.pub_zones = self.create_publisher(AvoidZones, '/avoid/zones', 1)

        # --- Zustand ---
        # Zonenmasken nur neu bauen, wenn sich Scan-Geometrie oder Zonengrenzen aendern.
        self._mask_key = None
        self._zone_masks = {}

        self.get_logger().info('Started avoid_node.')

    # --- Callbacks ---

    def cb_scan(self, scan):
        if not scan.ranges:
            return
        read_params(self, self._param_names)
        self._build_zone_masks(scan)
        distances = self._zone_min_distances(scan)
        sector, direction, forward_allowed = self._select_sector(distances)

        msg = AvoidZones()
        msg.header = scan.header
        for name in ZONE_NAMES:
            setattr(msg, name, distances[name])
        msg.offset = self._compute_offset(distances)
        msg.direction = direction
        msg.forward_allowed = forward_allowed
        msg.sector = sector
        self.pub_zones.publish(msg)

    # --- Zonen ---

    def _build_zone_masks(self, scan):
        n = len(scan.ranges)
        bounds = {name: tuple(getattr(self, f'zone_{name}')) for name in ZONE_NAMES}
        key = (n, scan.angle_min, scan.angle_increment, tuple(bounds.items()))
        if key == self._mask_key:
            return

        angles_deg = np.degrees(scan.angle_min + np.arange(n) * scan.angle_increment) % 360.0
        masks = {}
        for name, (start, end) in bounds.items():
            start %= 360.0
            end %= 360.0
            if start <= end:
                masks[name] = (angles_deg >= start) & (angles_deg <= end)
            else:
                masks[name] = (angles_deg >= start) | (angles_deg <= end)
        self._zone_masks = masks
        self._mask_key = key

    def _zone_min_distances(self, scan):
        """Kleinste gueltige Distanz je Zone, inf wenn frei."""
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        # inf/nan = kein Echo = frei. Nur zu kurze Werte sind Artefakte.
        floor = max(scan.range_min, float(self.min_valid_distance))
        valid = np.isfinite(ranges) & (ranges > floor)

        distances = {}
        for name, mask in self._zone_masks.items():
            selected = ranges[mask & valid]
            distances[name] = float(np.min(selected)) if selected.size else math.inf
        return distances

    # --- Lane-Modus: Offset ---

    def _proximity(self, distance):
        """0.0 = nichts in Reichweite, 1.0 = an der Saettigungsgrenze, dazwischen linear."""
        trigger = float(self.offset_trigger_distance)
        # Frei parametriert kann die Saettigung ueber dem Ausloeseabstand liegen.
        saturation = min(float(self.offset_saturation_distance), trigger)

        if not math.isfinite(distance) or distance >= trigger:
            return 0.0
        if distance <= saturation or trigger == saturation:
            return 1.0
        return (trigger - distance) / (trigger - saturation)

    def _compute_offset(self, distances):
        """Hindernis rechts -> nach links, links -> nach rechts. Sind beide Seiten
        belegt, heben sich die Anteile teilweise auf und der Bot faehrt mittig durch."""
        p_right = self._proximity(distances['front_right'])
        p_left = self._proximity(distances['front_left'])
        offset = self.offset_gain * (p_right - p_left) * self.offset_direction
        limit = float(self.offset_max)
        return float(max(-limit, min(limit, offset)))

    # --- Obstacle-Modus: Ausweichrichtung ---

    def _select_sector(self, distances):
        """(Sektor, Richtung, vorwaerts erlaubt).

        Unter den freien Sektoren der mit dem kleinsten Kostenbetrag, bei Gleichstand
        der mit mehr Platz. Ist keiner frei, der mit der groessten Distanz: lieber der
        beste Kompromiss als stehen bleiben.
        """
        costs = {name: float(getattr(self, f'cost_{name}')) for name in ZONE_NAMES}
        free = [n for n in ZONE_NAMES if distances[n] >= self.clear_distance]
        if free:
            sector = min(free, key=lambda n: (abs(costs[n]), -distances[n]))
        else:
            sector = max(ZONE_NAMES, key=lambda n: distances[n])

        cost = costs[sector]
        direction = 1 if cost > 0 else -1 if cost < 0 else 0
        too_close = distances['front'] <= self.avoid_turn_only_distance
        # Kein freier Sektor und keine Drehrichtung: nicht blind vorwaerts.
        stuck = not free and direction == 0
        return sector, direction, not (too_close or stuck)

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down avoid_node.')


def main(args=None):
    run_node(avoid_node, args)


if __name__ == '__main__':
    main()
