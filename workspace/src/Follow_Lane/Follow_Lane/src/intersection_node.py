"""Kreuzung: an der Haltelinie anhalten, Vorfahrt pruefen (rechts vor links), geradeaus queren.

Ein:  /detect/intersection (PolygonStamped, Kreuzungsfeld in BEV-Pixeln, Stempel = Kamerabild),
      /detect/objects (YOLO-Boxen im entzerrten Rohbild), /detect/lane_far (fernes Spurpaar),
      /control/mode, TF odom -> base_footprint (odom_filter)
Aus:  /intersection/active (Bool) -> switch_control schaltet Lane <-> Intersection,
      /intersection/cmd_vel (Twist) -> control_driving reicht ihn im Modus Intersection durch,
      /intersection/state (String) fuers Debugbild

Ablauf (Phasen):
  Frei     Kreuzung von detect_lane bestaetigt, Modus Lane                -> Anfahrt
  Anfahrt  Der PID faehrt weiter (Lane). Die Haltelinie liegt in odom, der Abstand stimmt
           also auch dann noch, wenn sie zu nah ist, um erkannt zu werden.
           Abstand Radachse -> Haltelinie <= stop_distance -> active      -> Halt
  Halt     Null. Mindestens min_stop_time, dann los, sobald CLEAR_DEBOUNCE_S lang kein
           TurtleBot (YOLO 'tb') rechts der Kreuzung oder in ihr steht    -> Queren
  Queren   Geradeaus mit cross_speed. Ist das Spurpaar hinter der Kreuzung zu sehen, wird
           darauf geregelt (far_kp), sonst Kurs gehalten (heading_kp, odom): senkrecht zur
           Haltelinie, also gerade ueber die Kreuzung, auch wenn der Bot schraeg steht.
           Ist das Paar sicher (FAR_CONFIRM Bilder hintereinander hinter der Mitte der
           Querstrasse, Spurfehler stabil) -> active False -> Lane; detect_lane regelt dann
           auf das ferne Paar, bis die Suchzeile Linien hat. Spaetestens nach max_cross_distance.
  Nachlauf COOLDOWN_M lang keine neue Kreuzung annehmen                   -> Frei
"""
import math
from collections import deque
import os
import time

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PolygonStamped, Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Bool, Float64MultiArray, String, UInt8
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from vision_msgs.msg import Detection2DArray

from Follow_Lane.src import bev_geometry
from Follow_Lane.src.common import (
    ControlType, bev_pixels_to_ground, control_mode_from_msg, frange,
    inverse_transform_points, pose2d_from_transform, read_params, require_params, run_node,
    transform_points)
from Follow_Lane.src.ingress_node_nav2 import CALIBRATION, EXTRINSICS, FISHEYE_BALANCE

RATE_HZ = 20.0
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'

# Geometrie der Kreuzung aus der Haltelinie (Ecken = Innenkanten von gelb und weiss):
# Querstrasse = Linie + Spur + Mittellinie + Spur + Linie, Spurbreite W = Laenge der
# Haltelinie (auf LANE_WIDTH_RANGE_M begrenzt, falls eine Ecke daneben liegt).
LINE_WIDTH_M = 0.05
LANE_WIDTH_RANGE_M = (0.12, 0.35)
# So weit rechts der Kreuzung (ab weisser Linie) zaehlt ein Bot als "von rechts".
RIGHT_ARM_M = 0.40
# Fusspunkte bis so weit ausserhalb einer Zone zaehlen noch (Projektion, Boxrand).
ZONE_MARGIN_M = 0.03
# YOLO-Klassen, die Vorfahrt haben (eine Klasse fuer Duckies hat das Modell nicht).
BLOCKING_CLASSES = ('tb',)
# Fusspunkte je Box: so viele Punkte entlang der Unterkante.
FOOT_SAMPLES = 5
# Kommen so lange (s) keine YOLO-Boxen, gilt die Kreuzung als ungeprueft frei (Warnung).
OBJECTS_MAX_AGE_S = 1.0
# So lange (s) muss die Kreuzung frei sein, bevor der Bot losfaehrt.
CLEAR_DEBOUNCE_S = 0.5

# Fernes Spurpaar: hoechstens so alt (s); es zaehlt erst hinter diesem Anteil der
# geschaetzten Querstrassen-Tiefe ab Haltelinie, damit die Eckboegen an der eigenen
# Haltelinie nicht als Ziel genommen werden.
FAR_MAX_AGE_S = 0.3
FAR_PAST_FRACTION = 0.5
# "Sicher": so viele gueltige Paare hintereinander, Spurfehler darin hoechstens so weit
# auseinander -> zurueck an Lane, der normale PID uebernimmt.
FAR_CONFIRM = 3
FAR_MAX_SPREAD = 0.15
MAX_ANGULAR_VEL = 1.0
# Kurs beim Queren = senkrecht zur Haltelinie, sofern das hoechstens so weit (Grad) vom
# eigenen Kurs abweicht; sonst gilt die Haltelinie als schief erkannt.
MAX_LINE_YAW_DIFF_DEG = 10.0
# Nach dem Queren so weit (m) keine neue Kreuzung annehmen. Die Uebergabe an Lane kommt
# jetzt schon am Anfang der Kreuzung, daher die ganze Querstrasse plus Rest abdecken.
COOLDOWN_M = 0.60

FREE, APPROACH, WAIT, CROSS, COOLDOWN = 'Frei', 'Anfahrt', 'Halt', 'Queren', 'Nachlauf'


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class intersection_node(Node):

    def __init__(self):
        super().__init__('intersection_node')

        # --- Parameter ---
        # Vor Ort einstellen, Werte nur in Intersection.yaml.
        # Abstand Radachse -> Haltelinie (m), bei dem angehalten wird.
        self.declare_parameter('stop_distance', Parameter.Type.DOUBLE, frange(0.0, 0.6))
        # So lange (s) steht der Bot mindestens, auch wenn die Kreuzung frei ist.
        self.declare_parameter('min_stop_time', Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self.declare_parameter('cross_speed', Parameter.Type.DOUBLE, frange(0.0, 0.22))
        # Lenkung beim Queren: auf den Gierwinkelfehler (rad) bzw. den lane_error des fernen
        # Spurpaars (-1..1, wie der normale PID).
        self.declare_parameter('heading_kp', Parameter.Type.DOUBLE, frange(0.0, 10.0))
        self.declare_parameter('far_kp', Parameter.Type.DOUBLE, frange(0.0, 10.0))
        # Spaetestens nach so viel Weg (m) ab Halt zurueck zu Lane.
        self.declare_parameter('max_cross_distance', Parameter.Type.DOUBLE, frange(0.1, 3.0))
        self._param_names = ['stop_distance', 'min_stop_time', 'cross_speed', 'heading_kp',
                             'far_kp', 'max_cross_distance']
        require_params(self, self._param_names)
        read_params(self, self._param_names)

        # Rohbild-Pixel -> BEV-Pixel fuer die Fusspunkte der YOLO-Boxen. /lokal/raw_image hat
        # die Groesse der Intrinsik-Datei (800x600), die Boxen liegen in diesen Pixeln.
        pdir = os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter')
        bev_to_img, self._bev = bev_geometry.load_bev_to_image(
            pdir, CALIBRATION, EXTRINSICS, FISHEYE_BALANCE)
        self._img_to_bev = np.linalg.inv(bev_to_img)

        # --- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        self.sub_intersection = self.create_subscription(
            PolygonStamped, '/detect/intersection', self.cb_intersection, 1)
        self.sub_objects = self.create_subscription(
            Detection2DArray, '/detect/objects', self.cb_objects, 1)
        self.sub_lane_far = self.create_subscription(
            Float64MultiArray, '/detect/lane_far', self.cb_lane_far, 1)

        # --- Publisher ---
        self.pub_active = self.create_publisher(Bool, '/intersection/active', 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/intersection/cmd_vel', 1)
        self.pub_state = self.create_publisher(String, '/intersection/state', 1)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / RATE_HZ, self.cb_timer)

        # --- Zustand ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = None
        self.objects = None
        self._objects_time = None
        self._far = None
        self._far_time = None
        self._reset(FREE)

        self.get_logger().info('Started intersection_node.')

    def _reset(self, phase):
        self.phase = phase
        self._line = None           # Haltelinie (2, 2) in odom: Ecke links, Ecke rechts
        self._zones = {}            # Name -> Polygon (N, 2) in odom
        self._depth = None          # Tiefe der Querstrasse (m)
        self._stop_time = None
        self._last_block = None
        self._start_pose = None     # Pose beim Losfahren (Queren) bzw. Ende (Nachlauf)
        self._yaw_ref = None
        self._stats = None          # Messung einer Querung (_start_stats), Log in _finish
        self._far_ok = deque(maxlen=FAR_CONFIRM)   # lane_error der letzten gueltigen Paare
        self._state = phase

    # --- Callbacks ---

    def cb_control_mode(self, msg):
        mode = control_mode_from_msg(self, msg)
        if mode is None:
            return
        self.control_mode = mode
        if mode not in (ControlType.Lane, ControlType.Intersection) \
                and self.phase in (APPROACH, WAIT, CROSS):
            self.get_logger().info(f'Modus {mode.name} -- Kreuzung abgebrochen.')
            self._reset(FREE)

    def cb_objects(self, msg):
        self.objects = msg
        self._objects_time = time.monotonic()

    def cb_lane_far(self, msg):
        self._far = list(msg.data) if len(msg.data) == 3 else None
        self._far_time = time.monotonic()
        if self.phase == CROSS and self._stats is not None:
            self._count_far()

    def cb_intersection(self, msg):
        if len(msg.polygon.points) < 4 or self.phase not in (FREE, APPROACH, WAIT):
            return
        if self.phase == FREE and self.control_mode != ControlType.Lane:
            return
        # Kreuzungsfeld aus detect_lane: [oben links, oben rechts, unten rechts, unten links],
        # unten = Haltelinie.
        pts = np.array([(p.x, p.y) for p in msg.polygon.points], np.float64)
        X, Y = bev_pixels_to_ground(self._bev, pts[[3, 2], 0], pts[[3, 2], 1])
        line_base = np.column_stack([X, Y])
        pose = self._pose(msg.header.stamp)
        if pose is None:
            return
        self._line = transform_points(pose, line_base)
        self._zones, self._depth = self._make_zones(self._line)
        if self.phase == FREE:
            self.phase = APPROACH
            self.get_logger().info(
                f'Kreuzung voraus, Haltelinie {line_base[:, 0].mean() * 100:.0f} cm -- '
                f'Halt bei {self.stop_distance * 100:.0f} cm.')

    def cb_timer(self):
        read_params(self, self._param_names)
        pose = self._pose()
        if self.phase == APPROACH:
            self._approach(pose)
        elif self.phase == WAIT:
            self._wait(pose)
        elif self.phase == CROSS:
            self._cross(pose)
        elif self.phase == COOLDOWN:
            self._state = COOLDOWN
            if pose is not None and math.hypot(pose[0] - self._start_pose[0],
                                               pose[1] - self._start_pose[1]) >= COOLDOWN_M:
                self._reset(FREE)
        self.pub_active.publish(Bool(data=self.phase in (WAIT, CROSS)))
        self.pub_state.publish(String(data=self._state))

    # --- Phasen ---

    def _approach(self, pose):
        if pose is None:
            return
        remaining = self._line_distance(pose)
        self._state = f'{APPROACH} {remaining * 100:.0f} cm'
        if remaining <= self.stop_distance:
            self.get_logger().info(f'Halt, Haltelinie {remaining * 100:.0f} cm voraus.')
            self.phase = WAIT
            self._stop_time = time.monotonic()

    def _wait(self, pose):
        self.pub_cmd_vel.publish(Twist())
        if pose is None:
            return
        now = time.monotonic()
        blocking = self._blocking(pose)
        if blocking:
            self._last_block = now
            self._state = f'{WAIT}: Bot ' + ', '.join(sorted({z for z, _ in blocking}))
            self.get_logger().info(
                'Warte: ' + ', '.join(f'{name} {zone}' for zone, name in blocking),
                throttle_duration_sec=2.0)
            return
        waited = now - self._stop_time
        self._state = f'{WAIT} {waited:.1f} s'
        if waited < self.min_stop_time:
            return
        if self._last_block is not None and now - self._last_block < CLEAR_DEBOUNCE_S:
            return
        self.phase = CROSS
        self._start_pose = pose
        self._yaw_ref = self._crossing_yaw(pose)
        self._start_stats(pose)
        self.get_logger().info(
            f'Kreuzung frei nach {waited:.1f} s -- quere geradeaus '
            f'(Kurs {math.degrees(wrap(self._yaw_ref - pose[2])):+.1f} Grad zur Haltelage).')

    def _cross(self, pose):
        if pose is None:
            self.pub_cmd_vel.publish(Twist())
            return
        travelled = math.hypot(pose[0] - self._start_pose[0], pose[1] - self._start_pose[1])
        far = self._fresh_far(pose)
        twist = Twist()
        twist.linear.x = float(self.cross_speed)
        if far is not None and len(self._far_ok) >= FAR_CONFIRM \
                and max(self._far_ok) - min(self._far_ok) <= FAR_MAX_SPREAD:
            # Paar hinter der Kreuzung sicher: der normale PID uebernimmt (detect_lane regelt
            # auf das ferne Paar, bis die Suchzeile Linien hat).
            if self._stats is not None:
                self._stats['spur_ab_m'] = travelled
            self._finish(pose, f'Spur dahinter sicher nach {travelled * 100:.0f} cm '
                               f'(Paar {far[1] * 100:.0f} cm voraus) -- PID uebernimmt')
            return
        if far is not None:
            angular = self.far_kp * far[0]
            # Ohne Paar im naechsten Bild den Kurs von jetzt halten, nicht den vom Halt.
            self._yaw_ref = pose[2]
            self._state = f'{CROSS} (Spur {far[1] * 100:.0f} cm)'
        else:
            angular = self.heading_kp * wrap(self._yaw_ref - pose[2])
            self._state = f'{CROSS} (Kurs)'
        if travelled >= self.max_cross_distance:
            self.get_logger().warn(
                f'max_cross_distance ({self.max_cross_distance:.2f} m) erreicht, ohne Spur '
                'hinter der Kreuzung -- zurueck zu Lane.')
            self._finish(pose, 'Weggrenze')
            return
        twist.angular.z = float(max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular)))
        self.pub_cmd_vel.publish(twist)

    def _finish(self, pose, reason):
        self.get_logger().info(f'Kreuzung gequert: {reason}.')
        self._log_stats(pose)
        self.pub_cmd_vel.publish(Twist())
        self._reset(COOLDOWN)
        self._start_pose = pose

    # --- Hilfsmethoden ---

    def _pose(self, stamp=None):
        """(x, y, yaw) von base_footprint in odom zum Zeitpunkt stamp (None = neueste)."""
        try:
            t = Time() if stamp is None else Time.from_msg(stamp)
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, BASE_FRAME, t)
        except ExtrapolationException:
            # Stempel ausserhalb des TF-Puffers (Bot-Uhr abweichend): neueste Lage.
            return None if stamp is None else self._pose()
        except (LookupException, ConnectivityException) as e:
            self.get_logger().warn(f'Keine TF {ODOM_FRAME} <- {BASE_FRAME}: {e}',
                                   throttle_duration_sec=5.0)
            return None
        return pose2d_from_transform(tf)

    def _crossing_yaw(self, pose):
        """Sollkurs fuers Queren: senkrecht zur Haltelinie (gerade ueber die Kreuzung, auch
        wenn der Bot schraeg angehalten hat). Weicht das mehr als MAX_LINE_YAW_DIFF_DEG vom
        eigenen Kurs ab, ist eher die Haltelinie schief erkannt -> eigener Kurs."""
        s_l, s_r = self._line
        left = s_l - s_r
        line_yaw = math.atan2(-left[0], left[1])       # Normale nach vorn = links um -90 Grad
        diff = math.degrees(wrap(line_yaw - pose[2]))
        if abs(diff) > MAX_LINE_YAW_DIFF_DEG:
            self.get_logger().warn(f'Haltelinie {diff:+.0f} Grad schraeg -- halte eigenen Kurs.')
            return pose[2]
        return line_yaw

    def _line_distance(self, pose):
        """Abstand (m) Radachse -> Mitte der Haltelinie, in Fahrtrichtung."""
        return float(inverse_transform_points(pose, self._line).mean(axis=0)[0])

    @staticmethod
    def _make_zones(line):
        """Zonen 'rechts' und 'mitte' (Polygone) und Tiefe der Querstrasse aus der
        Haltelinie (Ecke links, Ecke rechts), im selben Frame wie line."""
        s_l, s_r = line
        across = s_l - s_r
        width = float(np.linalg.norm(across))
        left = across / max(width, 1e-6)               # Einheitsvektor nach links
        fwd = np.array([left[1], -left[0]])            # Einheitsvektor nach vorn
        width = min(max(width, LANE_WIDTH_RANGE_M[0]), LANE_WIDTH_RANGE_M[1])
        depth = 2 * width + 3 * LINE_WIDTH_M
        # Mitte: eigene Spur + Mittellinie + Gegenspur, ueber die ganze Querstrasse.
        span = 2 * width + LINE_WIDTH_M
        mitte = np.array([s_r, s_r + depth * fwd, s_r + depth * fwd + span * left,
                          s_r + span * left])
        # Rechts: weisse Linie + RIGHT_ARM_M nach rechts, ebenfalls ueber die ganze Tiefe.
        arm = -(LINE_WIDTH_M + RIGHT_ARM_M) * left
        rechts = np.array([s_r + arm, s_r + arm + depth * fwd, s_r + depth * fwd, s_r])
        return {'rechts': rechts, 'mitte': mitte}, depth

    def _blocking(self, pose):
        """[(Zone, Klasse)] aller Bots mit Vorfahrt. Ohne frische YOLO-Boxen leer (Warnung)."""
        if self._objects_time is None or time.monotonic() - self._objects_time > OBJECTS_MAX_AGE_S:
            self.get_logger().warn('Keine aktuellen YOLO-Boxen -- Vorfahrt NICHT geprueft.',
                                   throttle_duration_sec=5.0)
            return []
        zones = {name: inverse_transform_points(pose, poly).astype(np.float32)
                 for name, poly in self._zones.items()}
        found = []
        for det in self.objects.detections:
            if not det.results or det.results[0].hypothesis.class_id not in BLOCKING_CLASSES:
                continue
            name = det.results[0].hypothesis.class_id
            feet = self._foot_points(det)
            for zone, poly in zones.items():
                if any(cv2.pointPolygonTest(poly, (float(x), float(y)), True) >= -ZONE_MARGIN_M
                       for x, y in feet):
                    found.append((zone, name))
                    break
        return found

    def _foot_points(self, det):
        """Unterkante der Box (dort steht der Bot auf dem Boden) -> Boden in base_footprint (m).
        Punkte ueber dem Horizont (Strahl trifft den Boden nicht) fallen weg."""
        cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
        xs = np.linspace(cx - det.bbox.size_x / 2, cx + det.bbox.size_x / 2, FOOT_SAMPLES)
        y1 = cy + det.bbox.size_y / 2
        p = self._img_to_bev @ np.vstack([xs, np.full_like(xs, y1), np.ones_like(xs)])
        ok = p[2] > 1e-9
        if not ok.any():
            return np.empty((0, 2))
        X, Y = bev_pixels_to_ground(self._bev, p[0, ok] / p[2, ok], p[1, ok] / p[2, ok])
        feet = np.column_stack([X, Y])
        return feet[X > 0]

    def _fresh_far(self, pose):
        """Fernes Spurpaar [lane_error, Abstand, Abstand Suchzeile] oder None, wenn alt
        oder noch vor dem Ende der Querstrasse."""
        if self._far is None or time.monotonic() - self._far_time > FAR_MAX_AGE_S:
            return None
        return self._far if self._far[1] >= self._far_needed(pose) else None

    def _far_needed(self, pose):
        """So weit (m, ab Radachse) muss ein fernes Paar mindestens liegen: hinter der Mitte
        der Querstrasse, also nicht mehr die Eckboegen an der eigenen Haltelinie. Die volle
        geschaetzte Tiefe (2W + 3 Linien) war auf der Strecke zu gross -- alle echten Paare
        wurden verworfen (12.11.)."""
        return self._line_distance(pose) + self._depth * FAR_PAST_FRACTION

    # --- Messung einer Querung (nur Log) ---

    def _start_stats(self, pose):
        self._stats = {'t0': time.monotonic(), 'frames': 0, 'empty': 0, 'near': [],
                       'ok': 0, 'spur_ab_m': None,
                       'needed_m': self._far_needed(pose)}

    def _count_far(self):
        """Jede /detect/lane_far-Meldung beim Queren: leer, zu nah (verworfen) oder gueltig."""
        s = self._stats
        s['frames'] += 1
        pose = self._pose()
        if self._far is None:
            s['empty'] += 1
            self._far_ok.clear()
        elif pose is not None and self._far[1] < self._far_needed(pose):
            s['near'].append(self._far[1])
            self._far_ok.clear()
        else:
            s['ok'] += 1
            self._far_ok.append(self._far[0])

    def _log_stats(self, pose):
        s = self._stats
        if s is None or pose is None:
            return
        total = math.hypot(pose[0] - self._start_pose[0], pose[1] - self._start_pose[1])
        kurs = total if s['spur_ab_m'] is None else s['spur_ab_m']
        near = s['near']
        near_txt = (f'{len(near)} zu nah verworfen ({min(near) * 100:.0f}-{max(near) * 100:.0f} cm '
                    f'voraus)' if near else '0 zu nah verworfen')
        self.get_logger().info(
            f'Messung Queren: {total * 100:.0f} cm in {time.monotonic() - s["t0"]:.1f} s, '
            f'davon nur Kurs {kurs * 100:.0f} cm, auf ferne Spur {(total - kurs) * 100:.0f} cm. '
            f'lane_far: {s["frames"]} Bilder, {s["empty"]} ohne Paar, {near_txt}, {s["ok"]} gueltig '
            f'(gefordert ab Start: >= {s["needed_m"] * 100:.0f} cm voraus).')

    # --- Beenden ---

    def shut_down(self):
        self.get_logger().info('Shutting down intersection_node.')


def main(args=None):
    run_node(intersection_node, args)


if __name__ == '__main__':
    main()
