"""Kreuzung: an der Haltelinie anhalten, Vorfahrt pruefen (rechts vor links), queren oder abbiegen.

Ein:  /detect/intersection (PolygonStamped, Kreuzungsfeld in BEV-Pixeln, Stempel = Kamerabild),
      /detect/objects (YOLO-Boxen im entzerrten Rohbild), /detect/lane_far (fernes Spurpaar),
      /detect/lane_mask (Spurlinien in der BEV: erlaubte Richtungen),
      /control/mode, TF odom -> base_footprint (odom_filter)
Ein (Bedienung): /intersection/choice (String links|geradeaus|rechts, Button im Debugbild)
Aus:  /intersection/active (Bool) -> switch_control schaltet Lane <-> Intersection,
      /intersection/cmd_vel (Twist) -> control_driving reicht ihn im Modus Intersection durch,
      /intersection/state (String) fuers Debugbild,
      /intersection/ask (String: erlaubte Richtungen mit Komma, solange gefragt wird, sonst leer)

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
Prioritaet (turn = geradeaus | links | rechts): 1) turn, wenn erlaubt, 2) geradeaus, wenn erlaubt,
3) die einzige erlaubte Richtung, 4) mehrdeutig (z. B. links + rechts): fragen -- debug_view zeigt
Buttons fuer die erlaubten Richtungen (/intersection/ask, Antwort /intersection/choice), nach
ask_timeout_s ohne Klick zufaellig. turn = auto: immer zufaellig unter den erlaubten. Ohne Messung
oder ohne erlaubte Richtung: geradeaus wie bisher. Abbiegen = Viertelkreis bis zur Mitte der Querstrasse, dann uebernimmt der PID.
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
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64MultiArray, String, UInt8
from std_srvs.srv import SetBool
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
TURNS = ('geradeaus', 'links', 'rechts', 'auto', 'immer_geradeaus')
DIRECTIONS = ('links', 'geradeaus', 'rechts')      # Reihenfolge fuer Anzeige und Buttons
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
# ueberwiegen. Live 16.11.: Rand links 28-75 cm2, rechts nur 9-28 (Fischaugen-Rand, Bot steht
# links in der Spur), Querlinie vorn 51-69 cm2, leere Bereiche 0-3.
MIN_LINE_CM2 = 5.0
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

# Ende-Fahrt (end_lane_steer): Spurmitte OHNE Farben = Mitte zwischen der naechsten Linie links und
# rechts. Ausgewertet in Scheiben (m) dieses Bereichs voraus; Scheiben mit Linienpunkten nahe der
# Mitte (Querlinie) zaehlen nicht; Abstand der beiden Linien muss plausibel sein (Spur ~20 cm).
CORRIDOR_X_M = (0.12, 0.40)
CORRIDOR_SLICE_M = 0.025
CORRIDOR_CENTER_FREE_M = 0.04
CORRIDOR_MAX_SIDE_M = 0.30
CORRIDOR_WIDTH_M = (0.12, 0.34)
CORRIDOR_MIN_POINTS = 3
CORRIDOR_MIN_SLICES = 2
CORRIDOR_MAX_AGE_S = 0.3
FREE, APPROACH, WAIT, CROSS, COOLDOWN = 'Frei', 'Anfahrt', 'Halt', 'Queren', 'Nachlauf'
# Endgueltig: nach dem Geradeausqueren (end_distance > 0) stehen bleiben, nie wieder Lane.
END = 'Ende'


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
        # Beim Geradeausqueren mindestens so weit (m) stur auf Kurs (Linien egal), erst danach
        # darf das Spurpaar dahinter uebernehmen (17.11.: Uebergabe schon nach 0-7 cm, die
        # Spurfolge zog in der Kreuzung zur Seite).
        self.declare_parameter('cross_min_distance', Parameter.Type.DOUBLE, frange(0.0, 1.5))
        # > 0: nach dem Geradeausqueren KEIN PID mehr -- stur auf Kurs bis so viel Weg (m) ab
        # Halt, dann anhalten und stehen bleiben (bis Neustart). 0 = Spurfolge dahinter.
        self.declare_parameter('end_distance', Parameter.Type.DOUBLE, frange(0.0, 3.0))
        # Bis zum Ende nach cross_min_distance mit dem fernen Spurpaar (gelb links, weiss rechts)
        # nachlenken -- nur vorwaerts, hoechstens end_max_dev_deg vom Kreuzungskurs weg.
        self.declare_parameter('end_lane_steer', Parameter.Type.BOOL)
        self.declare_parameter('end_max_dev_deg', Parameter.Type.DOUBLE, frange(0.0, 60.0))
        # Richtung an der Kreuzung (geradeaus | links | rechts | auto), live in rqt. Nicht
        # erlaubte Richtung -> die erlaubte (siehe Kopf).
        self.declare_parameter('turn', Parameter.Type.STRING)
        # Abbiegebogen: Zuschlag (m) auf den berechneten Radius (Linie + halbe Spur), negativ =
        # engere Kurve; ab so viel Drehung (Grad) uebernimmt der PID.
        self.declare_parameter('turn_radius_add', Parameter.Type.DOUBLE, frange(-0.20, 0.30))
        self.declare_parameter('turn_done_deg', Parameter.Type.DOUBLE, frange(30.0, 120.0))
        # Mehrdeutige Kreuzung: so lange (s) auf einen Klick im Debugbild warten, dann zufaellig.
        self.declare_parameter('ask_timeout_s', Parameter.Type.DOUBLE, frange(0.0, 300.0))
        self._param_names = ['stop_distance', 'min_stop_time', 'cross_speed', 'heading_kp',
                             'far_kp', 'max_cross_distance', 'turn', 'turn_radius_add',
                             'turn_done_deg', 'ask_timeout_s', 'cross_min_distance',
                             'end_distance', 'end_lane_steer', 'end_max_dev_deg']
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
        self.sub_choice = self.create_subscription(
            String, '/intersection/choice', self.cb_choice, 1)

        # --- Publisher ---
        self.pub_active = self.create_publisher(Bool, '/intersection/active', 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/intersection/cmd_vel', 1)
        self.pub_state = self.create_publisher(String, '/intersection/state', 1)
        self.pub_ask = self.create_publisher(String, '/intersection/ask', 1)
        # Ende-Fahrt: wie die Leertaste im notaus-Terminal auf STOPP schalten; wird wieder
        # freigegeben (Leertaste -> FAHREN), geht es normal weiter (Phase Frei).
        self.cli_notaus = self.create_client(SetBool, '/notaus/set')
        self._notaus_drive = None
        self.sub_notaus = self.create_subscription(
            Bool, '/notaus/drive', self.cb_notaus_drive,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

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
        self._mask_time = 0.0
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
        self._end_yaw0 = None      # Kreuzungskurs fuer die Ende-Fahrt (Grenze)
        self._direction = 'geradeaus'
        self._arc = None
        self._votes = None          # {'links'|'rechts'|'vorn': ([gelb...], [weiss...])} je Bild (cm2)
        self._allowed = None        # zuletzt berechnete erlaubte Richtungen (Anzeige)
        self._ask = None            # Frage laeuft: {'options': [...], 'since': monotonic}
        self._choice = None         # Antwort aus dem Debugbild            # Abbiegen: (Mittelpunkt (2,), Radius, Vorzeichen +1 links)
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

    def cb_notaus_drive(self, msg):
        released = msg.data and self._notaus_drive is False
        self._notaus_drive = msg.data
        if released and self.phase == END:
            self.get_logger().info('Not-Aus wieder FAHREN -- Ende aufgehoben, weiter wie normal.')
            self._reset(FREE)

    def _notaus_stop(self):
        """/notaus/set false = Leertaste auf STOPP."""
        if not self.cli_notaus.service_is_ready():
            self.get_logger().warn('Service /notaus/set nicht da -- Bot steht nur per cmd_vel 0.')
            return
        self.cli_notaus.call_async(SetBool.Request(data=False))

    def cb_objects(self, msg):
        self.objects = msg
        self._objects_time = time.monotonic()

    def cb_lane_far(self, msg):
        self._far = list(msg.data) if len(msg.data) == 3 else None
        self._far_time = time.monotonic()
        if self.phase == CROSS and self._stats is not None:
            self._count_far()

    def cb_choice(self, msg):
        if self._ask is None:
            return
        if msg.data in self._ask['options']:
            self._choice = msg.data
        else:
            self.get_logger().warn(f'Antwort "{msg.data}" nicht erlaubt (erlaubt: '
                                   f'{", ".join(self._ask["options"])}).')

    def cb_lane_mask(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        self._masks.append((stamp, msg))
        self._mask_time = time.monotonic()
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
        elif self.phase == END:
            self._state = f'{END}: angehalten nach der Kreuzung (Leertaste = weiter)'
            self.pub_cmd_vel.publish(Twist())
        elif self.phase == COOLDOWN:
            self._state = COOLDOWN
            driven = pose is not None and math.hypot(
                pose[0] - self._start_pose[0], pose[1] - self._start_pose[1]) >= self._cooldown_m
            if driven or time.monotonic() - self._cooldown_since >= COOLDOWN_MAX_S:
                self._reset(FREE)
        self.pub_active.publish(Bool(data=self.phase in (WAIT, CROSS, END)))
        self.pub_ask.publish(String(data=','.join(self._ask['options']) if self._ask else ''))
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
        if self._ask is None:
            direction = self._choose_direction(pose)
            if direction is None:
                self._ask = {'options': [d for d in DIRECTIONS if d in self._allowed],
                             'since': now}
                self.get_logger().info(
                    f'Mehrdeutig ({", ".join(self._ask["options"])}) -- frage im Debugbild, nach '
                    f'{self.ask_timeout_s:.0f} s ohne Klick zufaellig.')
                return
        elif self._choice is not None:
            direction = self._choice
            self.get_logger().info(f'Richtung per Klick: {direction}.')
        elif now - self._ask['since'] >= self.ask_timeout_s:
            direction = random.choice(self._ask['options'])
            self.get_logger().info(f'Kein Klick nach {self.ask_timeout_s:.0f} s -- zufaellig '
                                   f'{direction}.')
        else:
            left = self.ask_timeout_s - (now - self._ask['since'])
            self._state = f'{WAIT}: Frage {left:.0f} s{self._allowed_text()}'
            return
        self._ask = None
        self.phase = CROSS
        self._start_pose = pose
        self._yaw_ref = self._crossing_yaw(pose)
        self._start_stats(pose)
        self._direction = direction
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
        twist = Twist()
        twist.linear.x = float(self.cross_speed)
        if self.end_distance > 0.0:
            # Kein Lane-Modus mehr: ueber die Kreuzung stur, danach am Spurpaar entlang
            # (_end_steer), bei end_distance endgueltig stehen.
            if travelled >= self.end_distance:
                self.get_logger().info(
                    f'Kreuzung gequert, {travelled * 100:.0f} cm geradeaus -- ENDE, Bot steht '
                    '(Not-Aus STOPP, Leertaste = weiter).')
                self._log_stats(pose)
                self.pub_cmd_vel.publish(Twist())
                self._reset(END)
                self._notaus_stop()
                return
            twist.angular.z = float(self._end_steer(pose, travelled))
            self.pub_cmd_vel.publish(twist)
            return
        if travelled < self.cross_min_distance:
            # Stur geradeaus auf dem Kurs vom Halt (senkrecht zur Haltelinie), Linien egal.
            self._state = f'{CROSS} geradeaus stur {travelled * 100:.0f}/' \
                          f'{self.cross_min_distance * 100:.0f} cm'
            angular = self.heading_kp * wrap(self._yaw_ref - pose[2])
            twist.angular.z = float(max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular)))
            self.pub_cmd_vel.publish(twist)
            return
        far = self._fresh_far(pose)
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

    def _end_steer(self, pose, travelled):
        """Lenkung bis zum Ende (end_distance): erst stur auf Kurs ueber die Kreuzung, danach
        mit dem fernen Spurpaar, aber nie mehr als end_max_dev_deg vom Kreuzungskurs weg (kein
        Umdrehen). Ohne gueltiges Paar den Kurs von zuletzt halten."""
        if self._end_yaw0 is None:
            self._end_yaw0 = self._yaw_ref
        stur = travelled < self.cross_min_distance or not self.end_lane_steer
        center = None if stur else self._corridor_center()
        if center is not None:
            # Seitenversatz (m, + = Spurmitte links) -> Winkel zum Zielpunkt voraus.
            angular = self.heading_kp * math.atan2(center[0], center[1])
            self._yaw_ref = pose[2]
            how = f'Spurmitte {center[0] * 100:+.1f} cm'
        else:
            angular = self.heading_kp * wrap(self._yaw_ref - pose[2])
            how = 'stur' if stur else 'Kurs (kein Paar)'
        # Begrenzung um den Kreuzungskurs: darueber hinaus zurueck lenken.
        limit = math.radians(self.end_max_dev_deg)
        dev = wrap(pose[2] - self._end_yaw0)
        if abs(dev) > limit and angular * dev > 0.0:
            angular = self.heading_kp * (math.copysign(limit, dev) - dev)
            self._yaw_ref = self._end_yaw0 + math.copysign(limit, dev)
            how += ' (Grenze)'
        self._state = f'{CROSS} bis Ende {travelled * 100:.0f}/' \
                      f'{self.end_distance * 100:.0f} cm, {how}'
        return max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular))

    def _corridor_center(self):
        """(Seitenversatz der Spurmitte (m, + = links), Abstand voraus (m)) aus der neuesten
        Spurmaske, Farben egal: je Scheibe die naechste Linie links und rechts der Bot-Mitte.
        None, wenn alt oder zu wenige plausible Scheiben."""
        if not self._masks or time.monotonic() - self._mask_time > CORRIDOR_MAX_AGE_S:
            return None
        mask = self._masks[-1][1]
        xs = np.concatenate([np.asarray(mask.yellow_x), np.asarray(mask.white_x)])
        ys = np.concatenate([np.asarray(mask.yellow_y), np.asarray(mask.white_y)])
        if xs.size == 0:
            return None
        X, Y = bev_pixels_to_ground(self._bev, xs, ys)
        centers, dists = [], []
        x0 = CORRIDOR_X_M[0]
        while x0 < CORRIDOR_X_M[1]:
            sel = (X >= x0) & (X < x0 + CORRIDOR_SLICE_M) & (np.abs(Y) <= CORRIDOR_MAX_SIDE_M)
            x0 += CORRIDOR_SLICE_M
            y = Y[sel]
            if np.any(np.abs(y) < CORRIDOR_CENTER_FREE_M):
                continue    # Querlinie / Linie unter der Bot-Mitte
            left, right = y[y > 0], y[y < 0]
            if left.size < CORRIDOR_MIN_POINTS or right.size < CORRIDOR_MIN_POINTS:
                continue
            yl, yr = float(left.min()), float(right.max())
            if not CORRIDOR_WIDTH_M[0] <= yl - yr <= CORRIDOR_WIDTH_M[1]:
                continue
            centers.append((yl + yr) / 2.0)
            dists.append(x0 - CORRIDOR_SLICE_M / 2.0)
        if len(centers) < CORRIDOR_MIN_SLICES:
            return None
        return float(np.median(centers)), float(np.median(dists))

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
        if not allowed:
            # T-Kreuzung (vorn Querlinie) hat immer einen Ausgang: eine Seite mit eindeutig
            # FALSCHER Farbe faellt weg, die andere bleibt (16.11.: rechts weiss nur 9 cm2).
            left_wrong = left[1] >= MIN_LINE_CM2 and left[1] >= COLOR_RATIO * left[0]
            right_wrong = right[0] >= MIN_LINE_CM2 and right[0] >= COLOR_RATIO * right[1]
            if left_wrong and not right_wrong:
                allowed['rechts'] = 'links falsche Farbe (weiss) -- Ausgang rechts'
            elif right_wrong and not left_wrong:
                allowed['links'] = 'rechts falsche Farbe (gelb) -- Ausgang links'
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
        if chosen:
            second = f'faehrt: {chosen}'
        elif self._ask is not None:
            second = 'FRAGE: Button klicken'
        else:
            second = f'turn: {self.turn}'
        return f' | {free} / {second}'

    def _choose_direction(self, pose):
        """Richtung nach Prioritaet (Kopf der Datei); None = mehrdeutig, fragen.
        turn = immer_geradeaus: immer geradeaus, ohne Pruefung, ohne Frage."""
        if self.turn == 'immer_geradeaus':
            self.get_logger().info('turn = immer_geradeaus -- quere geradeaus (ohne Richtungspruefung).')
            return 'geradeaus'
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
        self._allowed = allowed
        if wanted == 'auto':
            return random.choice(sorted(allowed))
        if wanted in allowed:
            return wanted
        if 'geradeaus' in allowed:
            choice = 'geradeaus'
        elif len(allowed) == 1:
            choice = next(iter(allowed))
        else:
            return None
        self.get_logger().info(f'turn = {wanted} hier nicht erlaubt -- {choice}.')
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
