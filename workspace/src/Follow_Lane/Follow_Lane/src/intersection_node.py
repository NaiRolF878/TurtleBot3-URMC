"""Kreuzung: an der Haltelinie anhalten, Vorfahrt pruefen (rechts vor links), queren oder abbiegen.

Ein:  /detect/intersection (PolygonStamped, Kreuzungsfeld in BEV-Pixeln, Stempel = Kamerabild),
      /detect/objects (YOLO-Boxen im entzerrten Rohbild), /detect/lane_far (fernes Spurpaar),
      /detect/lane_mask (Spurlinien in der BEV: erlaubte Richtungen),
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

Richtung (Parameter turn: geradeaus | links | rechts | auto): aus Kreuzungsfeld und Spurmaske
DESSELBEN Kamerabilds (gleicher Stempel), waehrend der Anfahrt (Haltelinie VOTE_RANGE_M voraus;
beim Halt sieht die Kamera die Seiten nicht mehr), je Bereich und Farbe der Median ueber die Bilder.
Jede Strasse ist einspurig, in Fahrtrichtung gelb links, weiss rechts. Daher:
  links erlaubt    naher Rand der Querstrasse links der Einmuendung ist gelb (sie fuehrt nach links)
  rechts erlaubt   naher Rand rechts der Einmuendung ist weiss (sie fuehrt nach rechts)
  geradeaus        vorn liegt keine Linie quer ueber der eigenen Spurbreite
turn wird genommen, wenn erlaubt; sonst die erlaubte Richtung (mehrere: zufaellig, auto: immer
zufaellig unter den erlaubten). Ohne frische Maske oder ohne erlaubte Richtung: geradeaus wie
bisher. Abbiegen = Viertelkreis bis zur Mitte der Querstrasse, dann uebernimmt der PID.
"""
import math
from collections import deque
import os
import random
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
from follow_lane_msg.msg import LaneMask

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
# ... oder spaetestens nach so vielen Sekunden: von Hand zurueckgesetzt (Test) faehrt der Bot keine
# 60 cm -- ohne Zeitgrenze bliebe er ewig im Nachlauf und saehe die Kreuzung vor sich nicht.
COOLDOWN_MAX_S = 8.0
# Nach dem Abbiegen liegt die alte Kreuzung hinter dem Bot: kurzer Nachlauf, sonst wuerde eine
# dicht folgende Kreuzung (< COOLDOWN_M) uebersehen.
COOLDOWN_TURN_M = 0.20
# Im Nachlauf gemeldete Kreuzung gilt als NEUE, wenn ihre Haltelinie (Mitte, odom) mehr als so
# weit (m) von der gerade gequerten liegt (16.11.: zweite Kreuzung 37 cm nach der Uebergabe, ~85 cm
# hinter der ersten, wurde im 60-cm-Nachlauf ignoriert und geradeaus ueberfahren).
SAME_LINE_M = 0.25

# --- Richtung ---
TURNS = ('geradeaus', 'links', 'rechts', 'auto')
# Spurmaske: Punkte im 2,5-mm-Raster (LANE_MASK_STRIDE 2 bei 1,25 mm/px) -> Flaeche je Punkt.
MASK_CELL_CM2 = 0.25 ** 2
# Bereiche (m) relativ zur Haltelinie (x_line) und den Ecken (Innenkanten gelb/weiss):
# Querrand neben der Einmuendung: von SIDE_X vor/hinter der Linie, SIDE_Y_* seitlich der Ecke.
SIDE_X = (-0.03, 0.08)
SIDE_Y_NEAR, SIDE_Y_FAR = 0.09, 0.30
# Geradeaus: Streifen ueber der eigenen Spurbreite (Ecken -AHEAD_INSET), AHEAD_X hinter der Linie.
AHEAD_X = (0.10, 0.45)
AHEAD_INSET = 0.03
# Ab so viel Linienflaeche (cm2) zaehlt ein Bereich als markiert; die Farbe muss so oft
# ueberwiegen. Live an der T-Kreuzung 16.11.: Rand 28-75 cm2, Querlinie vorn 51 cm2, sonst 0.
MIN_LINE_CM2 = 10.0
COLOR_RATIO = 2.0
# Messen nur, solange die Haltelinie so weit (m, ab Radachse) voraus liegt: bei 15 cm (Halt) liegen
# die Querraender schon ausserhalb des Bilds (16.11.: 0-9 cm2 statt 28-75 bei 24 cm).
VOTE_RANGE_M = (0.16, 0.40)
# Mindestens so viele Bilder fuer eine Entscheidung (Median); so viele Masken/Felder je Stempel
# puffern. 1: auch bei dicht folgenden Kreuzungen (kurze Anfahrt) lieber wenig Bilder als blind
# geradeaus -- an einer T-Kreuzung waere das falsch.
MIN_VOTES = 1
STAMP_BUFFER = 15
# Abbiegen: Viertelkreis; Radius-Zuschlag (turn_radius_add) und Ende (turn_done_deg) in der YAML,
# Bahnfehler -> Lenkung (heading_kp). Kleinster Radius (m):
MIN_TURN_RADIUS_M = 0.10

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
        # Richtung an der Kreuzung (geradeaus | links | rechts | auto), live in rqt. Nicht
        # erlaubte Richtung -> die erlaubte (siehe Kopf).
        self.declare_parameter('turn', Parameter.Type.STRING)
        # Abbiegebogen: Zuschlag (m) auf den berechneten Radius (Linie + halbe Spur), negativ =
        # engere Kurve; ab so viel Drehung (Grad) uebernimmt der PID.
        self.declare_parameter('turn_radius_add', Parameter.Type.DOUBLE, frange(-0.20, 0.30))
        self.declare_parameter('turn_done_deg', Parameter.Type.DOUBLE, frange(30.0, 120.0))
        self._param_names = ['stop_distance', 'min_stop_time', 'cross_speed', 'heading_kp',
                             'far_kp', 'max_cross_distance', 'turn', 'turn_radius_add',
                             'turn_done_deg']
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
        self.sub_lane_mask = self.create_subscription(
            LaneMask, '/detect/lane_mask', self.cb_lane_mask, 1)

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
        self._last_line = None                      # Haltelinie der zuletzt gequerten Kreuzung (odom)
        self._masks = deque(maxlen=STAMP_BUFFER)    # (Stempel ns, LaneMask)
        self._fields = deque(maxlen=STAMP_BUFFER)   # (Stempel ns, Haltelinie (2, 2) base_footprint)
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
        self._direction = 'geradeaus'
        self._arc = None
        self._votes = None          # {'links'|'rechts'|'vorn': ([gelb...], [weiss...])} je Bild (cm2)
        self._allowed = None        # zuletzt berechnete erlaubte Richtungen (Anzeige)            # Abbiegen: (Mittelpunkt (2,), Radius, Vorzeichen +1 links)
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

    def cb_lane_mask(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        self._masks.append((stamp, msg))
        field = next((f for t, f in self._fields if t == stamp), None)
        if field is not None:
            self._vote(msg, field)

    def cb_intersection(self, msg):
        if len(msg.polygon.points) < 4:
            return
        if self.phase == COOLDOWN and self._new_intersection(msg):
            self.get_logger().info('Naechste Kreuzung schon im Nachlauf -- wird angefahren.')
            self._reset(FREE)
        if self.phase not in (FREE, APPROACH, WAIT):
            return
        if self.phase == FREE and self.control_mode != ControlType.Lane:
            return
        # Kreuzungsfeld aus detect_lane: [oben links, oben rechts, unten rechts, unten links],
        # unten = Haltelinie.
        pts = np.array([(p.x, p.y) for p in msg.polygon.points], np.float64)
        X, Y = bev_pixels_to_ground(self._bev, pts[[3, 2], 0], pts[[3, 2], 1])
        line_base = np.column_stack([X, Y])
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        self._fields.append((stamp, line_base))
        mask = next((mk for t, mk in self._masks if t == stamp), None)
        if mask is not None:
            self._vote(mask, line_base)
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

    def _new_intersection(self, msg):
        """Gemeldetes Feld gehoert zu einer anderen Kreuzung als der gerade gequerten?"""
        if self._last_line is None:
            return False
        pose = self._pose(msg.header.stamp)
        if pose is None:
            return False
        pts = np.array([(p.x, p.y) for p in msg.polygon.points], np.float64)
        X, Y = bev_pixels_to_ground(self._bev, pts[[3, 2], 0], pts[[3, 2], 1])
        line = transform_points(pose, np.column_stack([X, Y]))
        return float(np.linalg.norm(line.mean(axis=0) - self._last_line.mean(axis=0))) > SAME_LINE_M

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
            driven = pose is not None and math.hypot(
                pose[0] - self._start_pose[0], pose[1] - self._start_pose[1]) >= self._cooldown_m
            if driven or time.monotonic() - self._cooldown_since >= COOLDOWN_MAX_S:
                self._reset(FREE)
        self.pub_active.publish(Bool(data=self.phase in (WAIT, CROSS)))
        self.pub_state.publish(String(data=self._state))

    # --- Phasen ---

    def _approach(self, pose):
        if pose is None:
            return
        remaining = self._line_distance(pose)
        self._state = f'{APPROACH} {remaining * 100:.0f} cm{self._allowed_text()}'
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
        self._state = f'{WAIT} {waited:.1f} s{self._allowed_text()}'
        if waited < self.min_stop_time:
            return
        if self._last_block is not None and now - self._last_block < CLEAR_DEBOUNCE_S:
            return
        self.phase = CROSS
        self._start_pose = pose
        self._yaw_ref = self._crossing_yaw(pose)
        self._start_stats(pose)
        self._direction = self._choose_direction(pose)
        if self._direction == 'geradeaus':
            self._arc = None
            self.get_logger().info(
                f'Kreuzung frei nach {waited:.1f} s -- quere geradeaus '
                f'(Kurs {math.degrees(wrap(self._yaw_ref - pose[2])):+.1f} Grad zur Haltelage).')
        else:
            self._arc = self._make_arc(pose)
            self.get_logger().info(
                f'Kreuzung frei nach {waited:.1f} s -- biege {self._direction} ab '
                f'(Bogen r = {self._arc[1] * 100:.0f} cm).')

    def _cross(self, pose):
        if pose is None:
            self.pub_cmd_vel.publish(Twist())
            return
        if self._arc is not None:
            self._turn(pose)
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
            self._state = f'{CROSS} Spur {far[1] * 100:.0f} cm{self._allowed_text("geradeaus")}'
        else:
            angular = self.heading_kp * wrap(self._yaw_ref - pose[2])
            self._state = f'{CROSS} Kurs{self._allowed_text("geradeaus")}'
        if travelled >= self.max_cross_distance:
            self.get_logger().warn(
                f'max_cross_distance ({self.max_cross_distance:.2f} m) erreicht, ohne Spur '
                'hinter der Kreuzung -- zurueck zu Lane.')
            self._finish(pose, 'Weggrenze')
            return
        twist.angular.z = float(max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular)))
        self.pub_cmd_vel.publish(twist)

    def _turn(self, pose):
        """Viertelkreis um self._arc; Abstand zum Mittelpunkt != Radius -> nachlenken."""
        center, radius, sign = self._arc
        turned = math.degrees(wrap(pose[2] - self._yaw_ref)) * sign
        self._state = f'{CROSS} {turned:.0f} Grad{self._allowed_text(self._direction)}'
        if turned >= self.turn_done_deg:
            self._finish(pose, f'{self._direction} abgebogen ({turned:.0f} Grad) -- PID uebernimmt')
            return
        travelled = math.hypot(pose[0] - self._start_pose[0], pose[1] - self._start_pose[1])
        if travelled >= self.max_cross_distance:
            self.get_logger().warn(
                f'max_cross_distance ({self.max_cross_distance:.2f} m) beim Abbiegen erreicht '
                f'(erst {turned:.0f} Grad) -- zurueck zu Lane.')
            self._finish(pose, 'Weggrenze beim Abbiegen')
            return
        # Zu weit aussen (Abstand > Radius) -> staerker zur Kurveninnenseite.
        error = math.hypot(pose[0] - center[0], pose[1] - center[1]) - radius
        twist = Twist()
        twist.linear.x = float(self.cross_speed)
        angular = sign * (self.cross_speed / radius + self.heading_kp * error)
        twist.angular.z = float(max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular)))
        self.pub_cmd_vel.publish(twist)

    def _make_arc(self, pose):
        """Viertelkreis von der Haltelage auf die Mitte der (einspurigen) Querstrasse: Radius =
        Abstand Radachse -> Haltelinie + Linie + halbe Spurbreite + turn_radius_add (mind.
        MIN_TURN_RADIUS_M); Mittelpunkt seitlich in
        Abbiegerichtung, bezogen auf den Kurs senkrecht zur Haltelinie."""
        sign = 1.0 if self._direction == 'links' else -1.0
        s_l, s_r = self._line
        width = min(max(float(np.linalg.norm(s_l - s_r)), LANE_WIDTH_RANGE_M[0]),
                    LANE_WIDTH_RANGE_M[1])
        radius = max(MIN_TURN_RADIUS_M, self._line_distance(pose) + LINE_WIDTH_M + width / 2.0
                     + self.turn_radius_add)
        h = self._yaw_ref
        center = np.array([pose[0] - sign * radius * math.sin(h),
                           pose[1] + sign * radius * math.cos(h)])
        return center, radius, sign

    def _vote(self, mask, line):
        """Linienflaechen der drei Bereiche in EINEM Kamerabild (Spurmaske und Haltelinie mit
        gleichem Stempel, beide in base_footprint zum Bildzeitpunkt) als Stimme merken -- nur
        waehrend Anfahrt/Halt und solange die Haltelinie VOTE_RANGE_M voraus liegt."""
        if self.phase not in (FREE, APPROACH, WAIT):
            return
        (xl, yl), (xr, yr) = line
        x_line = (xl + xr) / 2.0
        if not VOTE_RANGE_M[0] <= x_line <= VOTE_RANGE_M[1]:
            return
        yx, yy = bev_pixels_to_ground(self._bev, np.asarray(mask.yellow_x), np.asarray(mask.yellow_y))
        wx, wy = bev_pixels_to_ground(self._bev, np.asarray(mask.white_x), np.asarray(mask.white_y))

        def area(xs, ys, x0, x1, y0, y1):
            inside = (xs >= x0) & (xs < x1) & (ys >= y0) & (ys < y1)
            return float(inside.sum()) * MASK_CELL_CM2

        side_x = (x_line + SIDE_X[0], x_line + SIDE_X[1])
        boxes = {'links': (*side_x, yl + SIDE_Y_NEAR, yl + SIDE_Y_FAR),
                 'rechts': (*side_x, yr - SIDE_Y_FAR, yr - SIDE_Y_NEAR),
                 'vorn': (x_line + AHEAD_X[0], x_line + AHEAD_X[1],
                          yr + AHEAD_INSET, yl - AHEAD_INSET)}
        if self._votes is None:
            self._votes = {name: ([], []) for name in boxes}
        for name, box in boxes.items():
            self._votes[name][0].append(area(yx, yy, *box))
            self._votes[name][1].append(area(wx, wy, *box))
        self._allowed = self._allowed_from_votes()

    def _medians(self):
        """{'links'|'rechts'|'vorn': (gelb, weiss)} Mediane; None mit weniger als MIN_VOTES."""
        if self._votes is None or len(self._votes['vorn'][0]) < MIN_VOTES:
            return None
        return {k: (float(np.median(y)), float(np.median(w))) for k, (y, w) in self._votes.items()}

    def _allowed_from_votes(self):
        """{Richtung: Begruendung} aus den Medianen; None mit weniger als MIN_VOTES Bildern."""
        med = self._medians()
        if med is None:
            return None
        left, right, ahead = med['links'], med['rechts'], med['vorn']
        allowed = {}
        if left[0] >= MIN_LINE_CM2 and left[0] >= COLOR_RATIO * left[1]:
            allowed['links'] = f'links gelb {left[0]:.0f} cm2'
        if right[1] >= MIN_LINE_CM2 and right[1] >= COLOR_RATIO * right[0]:
            allowed['rechts'] = f'rechts weiss {right[1]:.0f} cm2'
        if sum(ahead) < MIN_LINE_CM2:
            allowed['geradeaus'] = 'vorn frei'
        return allowed

    def _allowed_directions(self, pose):
        """Erlaubte Richtungen aus Anfahrt und Halt; None ohne Messung."""
        allowed = self._allowed_from_votes()
        if allowed is None:
            n = 0 if self._votes is None else len(self._votes['vorn'][0])
            self.get_logger().warn(f'Nur {n} Bilder der Kreuzung aus {VOTE_RANGE_M[0] * 100:.0f}-'
                                   f'{VOTE_RANGE_M[1] * 100:.0f} cm (mind. {MIN_VOTES}).')
            return None
        med = self._medians()
        left, right, ahead = med['links'], med['rechts'], med['vorn']
        self.get_logger().info(
            f'Kreuzung: links gelb/weiss {left[0]:.0f}/{left[1]:.0f}, rechts {right[0]:.0f}/'
            f'{right[1]:.0f}, vorn {ahead[0]:.0f}/{ahead[1]:.0f} cm2 (Median aus '
            f'{len(self._votes["vorn"][0])} Bildern) -> erlaubt: {", ".join(allowed) or "nichts"}.')
        return allowed

    def _allowed_text(self, chosen=None):
        """Fuers Debugbild (nach ' | ', Zeilen mit ' / '): erlaubte Richtungen, darunter turn
        bzw. beim Queren die gewaehlte Richtung."""
        if self._allowed is None:
            free = 'frei: ?'
        else:
            names = {'links': 'links', 'geradeaus': 'gerade', 'rechts': 'rechts'}
            free = 'frei: ' + (' '.join(names[d] for d in ('links', 'geradeaus', 'rechts')
                                        if d in self._allowed) or 'nichts')
        second = f'faehrt: {chosen}' if chosen else f'turn: {self.turn}'
        return f' | {free} / {second}'

    def _choose_direction(self, pose):
        wanted = self.turn if self.turn in TURNS else 'geradeaus'
        if wanted != self.turn:
            self.get_logger().warn(f'turn "{self.turn}" unbekannt (erlaubt: {", ".join(TURNS)}) '
                                   '-- geradeaus.')
        allowed = self._allowed_directions(pose)
        if allowed is None:
            self.get_logger().warn('Keine Spurmaske waehrend der Anfahrt -- Richtung unbekannt, '
                                   'geradeaus.')
            return 'geradeaus'
        if not allowed:
            self.get_logger().warn('Keine erlaubte Richtung erkannt -- geradeaus.')
            return 'geradeaus'
        if wanted in allowed:
            return wanted
        choice = random.choice(sorted(allowed))
        why = 'zufaellig' if len(allowed) > 1 else 'einzige erlaubte'
        if wanted != 'auto':
            self.get_logger().info(f'turn = {wanted} hier nicht erlaubt -- {choice} ({why}).')
        return choice

    def _finish(self, pose, reason):
        self.get_logger().info(f'Kreuzung gequert: {reason}.')
        self._log_stats(pose)
        self.pub_cmd_vel.publish(Twist())
        turned = self._direction != 'geradeaus'
        self._last_line = self._line
        self._reset(COOLDOWN)
        self._start_pose = pose
        self._cooldown_since = time.monotonic()
        self._cooldown_m = COOLDOWN_TURN_M if turned else COOLDOWN_M

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
