"""Spurerkennung auf der Vogelperspektive: Spurfehler fuer den PID und Linienpixel fuer Nav2.
Auf dem Rohbild: Schranke (Blau-Maske) und YOLO-Objekte (TensorRT).

Ein:  /lokal/bev, /lokal/raw_image, /control/mode
Aus:  /detect/lane (Float64), /detect/lane_debug, /detect/mask_debug/compressed,
      /detect/lane_mask (LaneMask fuer lane_costmap_node),
      /detect/lines_lost (Bool, nur TunnelEntry),
      /detect/barrier, /detect/barrier_pixels, /detect/barrier_debug/compressed,
      /detect/stop_sign (Bool), /detect/stop_sign_pixels (Int32),
      /detect/objects (vision_msgs/Detection2DArray, Pixel im Rohbild),
      /detect/intersection (PolygonStamped, Kreuzungsfeld in BEV-Pixeln),
      /detect/lane_far (Float64MultiArray, nur im Modus Intersection),
      /detect/timing (String, JSON, 1x/s: Bildalter, Raten, Rechenzeiten fuer debug_view)
Laeuft als Node detect_lane; im Tunnel pausiert (Reglement).
"""
import array
import json
import os
import time
from collections import defaultdict, deque

import cv2
import numpy as np

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32MultiArray, Float64, Float64MultiArray, Int32, String, UInt8
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from follow_lane_msg.msg import FollowLaneDebug, LaneMask
from Follow_Lane.src import bev_geometry
from Follow_Lane.src.common import (
    SELF_MASK_X_M, SELF_MASK_Y_M, ControlType, bev_pixels_to_ground, control_mode_from_msg,
    frange, inverse_transform_points, irange, read_params, require_params, run_node,
    transform_points, yaw_from_quaternion)

# Rasterweite fuers Ausduennen der auf /detect/lane_mask gesendeten Punkte.
LANE_MASK_STRIDE = 2
# Formfilter (cm, ueber bev.yaml in Pixel), nur fuer Nav2 und das Spurende. Spurlinien (2 cm)
# wirken in der BEV verschmiert breiter, an Ecken bis ~7 cm. Wo ein Kreis dieses Durchmessers
# hineinpasst, ist es keine Linie (helle Waende, Papier, Spiegelungen).
LINE_MAX_WIDTH_CM = 8.0
# Reste kleiner als das (cm^2) nach dem Filtern sind Rauschen.
LINE_MIN_AREA_CM2 = 3.0
# Eigener Bot vorn im Bild (Magnet, Halter): bis SELF_MASK_X_CM vor der Radachse und
# +-SELF_MASK_Y_CM in beiden Farbmasken schwarz. Der Magnet reicht laut Debugbild bis
# ~10,5 cm vor die Achse und +-4,5 cm.
SELF_MASK_X_CM = SELF_MASK_X_M * 100.0
SELF_MASK_Y_CM = SELF_MASK_Y_M * 100.0
# Linien-Tracking: hoechstens so weit (cm) darf sich eine Linie von Bild zu Bild bewegen.
LINE_MAX_JUMP_CM = 5.0
# So lange (s) wird die letzte Position einer verlorenen Linie gehalten.
LINE_HOLD_TIME_S = 1.0
# In diesen Modi laeuft die Spurfolge schon; ein Wechsel zu Lane aus einem anderen Modus loescht
# die Linienanker.
LINE_TRACK_MODES = (ControlType.Lane, ControlType.TunnelEntry)
# Kantenlaenge des Erosionskerns (px), entfernt Einzelpixel aus den Farbmasken.
ERODE_KERNEL = 5
# Ersatzspalte als Anteil der Bildbreite, wenn eine Linie weder gefunden noch
# gehalten wird.
WHITE_ALTERNATIVE = 0.65
YELLOW_ALTERNATIVE = 0.35
# Ecke mit Luecke (17.11., Strecke im Wettbewerb): die eigene Linie endet, die Querlinie beginnt
# erst daneben (teils andere Farbe). Querkante zaehlt, wenn ihr naechster Punkt hoechstens so weit
# (m) vom Ende der eigenen Linie liegt und sie nach aussen weggeht.
CORNER_GAP_M = 0.08
# Kreuzungserkennung: Mindestlaenge einer Kante (px, 1,25 mm/px), Konturen kleiner als
# MIN_CONTOUR_AREA (px^2) sind Rauschen, POLY_EPSILON (px) = Toleranz der Polygone.
MIN_EDGE_LEN = 30
MIN_CONTOUR_AREA = 50
POLY_EPSILON = 3
# Laengskante: hoechstens so weit (Grad) von der Senkrechten, Querkante: so weit von 90 Grad
# zur Laengskante. Dazwischen liegen Boegen abgerundeter Ecken.
LONG_TOL_DEG = 20.0
CROSS_TOL_DEG = 20.0
# Laenger (m) darf der Bogen zwischen Laengs- und Querkante nicht sein (Radius ~12 cm ->
# Viertelbogen ~19 cm); laengere Uebergaenge sind normale Kurven.
MAX_BEND_ARC_M = 0.25
# Querkante mindestens so lang (m): das Ende einer Linie (Stirnseite, ~5 cm) zaehlt nicht.
MIN_CROSS_LEN_M = 0.08
# Schliessen der kombinierten Maske (px, ~1,4 cm), siehe detect_intersection.
CLOSE_KERNEL = 11
# Beide Ecken einer Kreuzung liegen etwa gleich weit voraus (m).
MAX_CORNER_DX_M = 0.06
# Einmuendung von der Seite (T-Kreuzung, durchgehende Strasse): nur eine Ecke, die Gegenlinie
# laeuft mindestens so weit (m) gerade ueber die Hoehe der Ecke hinaus. In einer 90-Grad-Kurve
# biegt die aeussere Linie erst ~Spurbreite + Linie (~25 cm) nach der inneren Ecke ab, daher
# 30 cm; pruefbar nur, solange die Ecke hoechstens x_max - 30 cm voraus liegt (sonst warten).
# Je Zeile Linienpixel in einem Streifen um die Innenkante auf Hoehe der Ecke (senkrecht nach vorn):
# SIDE_T_TOL_PX (5 cm) zur Spur hin und dahinter Linie + Spiel (SIDE_T_OUT_PX) -- Versaetze an
# Mattenkanten (1-2 cm) stoeren nicht, eine abbiegende Linie verlaesst den Streifen. In
# mindestens SIDE_T_ROWS der Zeilen.
SIDE_T_AHEAD_M = 0.30
SIDE_T_TOL_PX = 40
SIDE_T_OUT_PX = 80
SIDE_T_ROWS = 0.8
# So viele der letzten Bilder muessen die Kreuzung zeigen, bevor sie als erkannt gilt.
INTERSECTION_CONFIRM = (3, 5)
# Bestaetigte Kreuzung so lange (s) halten, wenn sie in einzelnen Bildern fehlt; in der Zeit
# per Odometrie mitgeschoben (bleibt am Boden stehen, waehrend der Bot faehrt).
INTERSECTION_HOLD_S = 0.5
# Fernes Linienpaar beim Queren der Kreuzung (Modus Intersection, far_lane_pair): von der
# Suchzeile aus in Baendern nach oben (fern). Je Band +-FAR_BAND_PX Zeilen, davon mindestens
# FAR_MIN_ROWS mit Kante je Linie; Baender alle FAR_ROW_STEP_PX. Abstand gelb -> weiss
# zwischen min_lane_width und FAR_MAX_LANE_WIDTH_M (Spur 20 cm + Spiel).
FAR_BAND_PX = 16
FAR_ROW_STEP_PX = 16
FAR_MIN_ROWS = 8
FAR_MAX_LANE_WIDTH_M = 0.30
# color_side = auto: am Anfang EINMAL bestimmen, auf welcher Seite Gelb und Weiss liegen, dann fest
# (kein laufendes Umschalten: das tauschte falsch, wenn der Bot quer stand oder hochgehoben wurde).
# Ausgewertet im unteren Teil der BEV (Anteil AUTO_BAND der Hoehe), nur +-AUTO_HALF_M um die
# Bot-Mitte (Gegenspur bleibt draussen), Zeile fuer Zeile: Weiss rechts von Gelb (> AUTO_SEP_M)
# = normal, links davon = getauscht. Ein Bild zaehlt, wenn mind. AUTO_MIN_ROWS Zeilen beide Farben
# haben und AUTO_ROW_SHARE davon einig sind. Entschieden nach AUTO_VOTES Bildern, wenn
# AUTO_VOTE_SHARE davon einig sind; bis dahin normal.
AUTO_BAND = 0.35
AUTO_HALF_M = 0.25
# Zeilen, in denen eine Farbe breiter als das (m) liegt, sind Querlinien (Haltelinie) -> nicht zaehlen.
# Solange detect_lane eine Kreuzung vor sich sieht, wird nicht abgestimmt (dort liegt vorn oft die
# Gegenrichtung mit vertauschten Farben, Aufnahme 17.11.).
AUTO_MAX_SPREAD_M = 0.06
AUTO_SEP_M = 0.06
AUTO_ROW_STEP = 2
AUTO_MIN_ROWS = 12
AUTO_ROW_SHARE = 0.8
AUTO_VOTES = 15
AUTO_VOTE_SHARE = 0.8
COLOR_SIDES = ('auto', 'normal', 'getauscht')
# color_side_recheck: nach diesen Modi (Linien waren weg) bei der Rueckkehr nach Lane neu suchen.
RECHECK_AFTER_MODES = (ControlType.Tunnel, ControlType.TunnelEntry, ControlType.Intersection,
                       ControlType.Obstacle)
# Nach der Kreuzung hoechstens so lange (s) auf das ferne Paar regeln, bis die Suchzeile
# wieder beide Linien hat.
EXIT_FAR_S = 5.0
# Schranken-Debugbild (Rohbild) hoechstens so oft (s).
BARRIER_DEBUG_PERIOD_S = 0.1
# Rotes Einfahrt-verboten-Schild ist kein Stoppschild: hat die Objekterkennung 'no_entry' (mind.
# no_entry_min_conf) innerhalb so vieler Sekunden gesehen, wird Rot nicht gemeldet. Die
# Erkennung laeuft nur mit detector_rate_hz (~10/s), die Rotpruefung auf jedem Rohbild.
NO_ENTRY_HOLD_S = 0.5
# Schilder der Objekterkennung: jedes bekommt ein eigenes Topic /detect/sign/<name> (Bool), True =
# in diesem Bild gesehen (Konfidenz >= sign_min_conf, Box mindestens <name>_min_height_px bzw.
# sign_min_height_px hoch, Pixel im Rohbild). Gesendet bei jedem Lauf der Objekterkennung.
SIGN_CLASSES = ('park', 'tunnel', 'stop', 'left', 'right', 'work', 'no_entry')
# Objekterkennung (Schilder, andere Bots, Startlinie) auf dem Rohbild: YOLO oder RF-DETR als
# TensorRT-Engine, relativ zum Repo-Ordner (muss mit der TensorRT-Version im Container gebaut
# sein, siehe yolo_trt.py). Parameter 'detector' (YAML oder live per ros2 param set) waehlt
# einen Kurznamen hieraus, einen Pfad (.engine) oder 'aus' ('off' nur in Anfuehrungszeichen,
# sonst liest YAML es als false). Ohne Engine oder GPU laeuft der Node ohne Objekterkennung
# weiter. Gemessen 15.11. (RTX 2080, FP16): yolo640 ~8 ms, rfdetr384 ~6 ms, rfdetr704 ~16 ms.
DETECTORS = {
    'yolo640': 'Yolo/yolo26s_640_data1311/weights/best_trt10.15_fp16.engine',
    'yolo800': 'Yolo/yolo26s_800/weights/best_trt10.15_fp16.engine',
    'rfdetr384': 'rfdetr/rfdetr-nano_500_trt10.15_fp16.engine',
    'rfdetr704': 'rfdetr/rfdetr-nano-704_500_trt10.15_fp16.engine',
}
YOLO_CONF = 0.25      # Mindest-Konfidenz einer Box (YOLO)
YOLO_IOU = 0.7        # Ueberlappung, ab der NMS die schwaechere Box derselben Klasse verwirft
RFDETR_CONF = 0.5     # Mindest-Konfidenz einer Box (RF-DETR, Werte liegen anders als bei YOLO)
# Aelter (s) darf das Rohbild fuer YOLO nicht sein.
YOLO_MAX_AGE_S = 0.5
# Zeitmessung (/detect/timing) so oft (s) senden, jede TIMING_LOG_EVERY-te auch ins Log.
TIMING_PERIOD_S = 1.0
TIMING_LOG_EVERY = 5


class detect_lane(Node):
    """Vorzeichen: lane_error > 0 heisst Spurmitte links der Bildmitte (gelb links,
    weiss rechts), Wertebereich etwa -1..1.

    Im Tunnel und vor der ersten /control/mode wird kein Bild verarbeitet: in der
    Enclosure ist die Kamera laut Reglement verboten, und ein Neustart dort soll
    nichts durchlassen.
    """

    def __init__(self):
        super().__init__('detect_lane')

        # --- Parameter ---
        # Werte nur in Lab_Farben.yaml (LAB) bzw. Follow_Lane_ROI_LAB.yaml (Rest).
        byte = irange(0, 255)
        # L = Helligkeit, A = gruen(<128)..rot(>128), B = blau(<128)..gelb(>128); 128 = farblos.
        # Gemessen auf BEV-Bildern nach CLAHE (docs/bev-tests/README.md):
        #   Gelb       L 170-244  A 115-127  B 152-210   -> erkennbar am hohen B
        #   Weiss      L 210-255  A 120-128  B 118-157   -> erkennbar am hohen L, A/B nahe 128
        #   Matte      L  35-185  A 124-132  B 100-133
        self._param_names = [
            'lab_l_white_l', 'lab_l_white_h', 'lab_a_white_l', 'lab_a_white_h',
            'lab_b_white_l', 'lab_b_white_h',
            'lab_l_yellow_l', 'lab_l_yellow_h', 'lab_a_yellow_l', 'lab_a_yellow_h',
            'lab_b_yellow_l', 'lab_b_yellow_h',
            # Blau fuer die Schranke (aus detect_lane.py, Kalibrierung-Tests 188048a)
            'lab_l_blue_l', 'lab_l_blue_h', 'lab_a_blue_l', 'lab_a_blue_h',
            'lab_b_blue_l', 'lab_b_blue_h',
            # Rot fuer das Stoppschild. Gemessen 17.11. (CLAHE): Schild A 154-165 B 155-173,
            # lila Pullover A 128-144 B 108-145 -> A und B je >= ~148 trennt beide.
            'lab_l_red_l', 'lab_l_red_h', 'lab_a_red_l', 'lab_a_red_h',
            'lab_b_red_l', 'lab_b_red_h',
        ]
        for name in self._param_names:
            self.declare_parameter(name, Parameter.Type.INTEGER, byte)
        # Suchzeile als Anteil der Bildhoehe (0 = oben, fern).
        self.declare_parameter('detection_row_factor', Parameter.Type.DOUBLE, frange(0.5, 0.95))
        # Mindestabstand gelb -> weiss in BEV-Pixeln (1,25 mm/px: 80 px = 10 cm).
        self.declare_parameter('min_lane_width', Parameter.Type.INTEGER, irange(0, 300))
        # CLAHE gleicht ueber-/unterbelichtete Bildbereiche vor der LAB-Farbfilterung
        # an, damit wechselndes Licht die festen Schwellwerte nicht verfaelscht.
        self.declare_parameter('clahe_clip_limit', Parameter.Type.DOUBLE, frange(0.1, 40.0))
        self.declare_parameter('clahe_tile_grid_size', Parameter.Type.INTEGER, irange(1, 64))
        # Schranke: ab so vielen blauen Pixeln im Suchbereich des Rohbilds gilt sie als da.
        # Die aktuelle Zahl steht im Debugbild, danach einstellen.
        self.declare_parameter('thresh_blue_pixel', Parameter.Type.INTEGER, irange(0, 500000))
        # Stoppschild: rote Pixel (lab_*_red_*) im Bereich stop_roi_* (Anteile der Bildbreite
        # bzw. -hoehe des Rohbilds, oben rechts); ab thresh_red_pixel gilt es als gesehen. Die
        # aktuelle Zahl steht im Schranken-Debugbild ("Rot: n / Schwelle"), danach einstellen.
        self.declare_parameter('thresh_red_pixel', Parameter.Type.INTEGER, irange(0, 500000))
        # Sieht die Objekterkennung 'no_entry' mit mindestens dieser Konfidenz, ist das Rot das
        # Einfahrt-verboten-Schild -> kein Stoppschild.
        self.declare_parameter('no_entry_min_conf', Parameter.Type.DOUBLE, frange(0.0, 1.0))
        # Seite der Spurfarben: normal (Gelb links, Weiss rechts), getauscht (Weiss links, Gelb
        # rechts) oder auto (am Anfang selbst bestimmen, dann fest; neu suchen: in rqt kurz auf
        # normal und wieder auf auto). Live in rqt.
        self.declare_parameter('color_side', Parameter.Type.STRING)
        # Nur mit color_side auto: nach Tunnel, Kreuzung oder Ausweichen (zurueck in Lane) die
        # Farbseite neu bestimmen; bis dahin gilt die alte. Live in rqt.
        self.declare_parameter('color_side_recheck', Parameter.Type.BOOL)
        # Schilder (/detect/sign/<name>): Mindest-Konfidenz und Mindesthoehe der Box (px im
        # Rohbild 800x600); tunnel_min_height_px nur fuer das Tunnelschild. Live in rqt.
        self.declare_parameter('sign_min_conf', Parameter.Type.DOUBLE, frange(0.0, 1.0))
        self.declare_parameter('sign_min_height_px', Parameter.Type.INTEGER, irange(0, 600))
        self.declare_parameter('tunnel_min_height_px', Parameter.Type.INTEGER, irange(0, 600))
        for name in ('stop_roi_x0', 'stop_roi_x1', 'stop_roi_y0', 'stop_roi_y1'):
            self.declare_parameter(name, Parameter.Type.DOUBLE, frange(0.0, 1.0))
        # Objekterkennung: Kurzname aus DETECTORS, Pfad zu einer .engine oder 'aus'. Live
        # umschaltbar (ros2 param set /detect_lane detector rfdetr384), laedt dann neu (~1 s).
        self.declare_parameter('detector', Parameter.Type.STRING)
        # Hoechstens so oft (Hz) laeuft die Objekterkennung, 0 = auf jedem Rohbild. Dazwischen
        # wird nichts gesendet; die Empfaenger behalten die letzten Boxen (siehe vision_detection).
        self.declare_parameter('detector_rate_hz', Parameter.Type.DOUBLE, frange(0.0, 60.0))
        self._param_names += ['detection_row_factor', 'min_lane_width',
                              'clahe_clip_limit', 'clahe_tile_grid_size',
                              'thresh_blue_pixel', 'detector', 'detector_rate_hz',
                              'thresh_red_pixel', 'stop_roi_x0', 'stop_roi_x1', 'stop_roi_y0',
                              'stop_roi_y1', 'no_entry_min_conf', 'color_side', 'color_side_recheck',
                              'sign_min_conf', 'sign_min_height_px',
                              'tunnel_min_height_px']
        require_params(self, self._param_names)
        read_params(self, ['detector', 'detector_rate_hz'])
        try:
            self._bev = bev_geometry.load_bev_spec(
                os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter'))
        except Exception as e:  # noqa: BLE001 -- ohne bev.yaml kein Formfilter, Masken roh
            self._bev = None
            self.get_logger().warn(f'bev.yaml nicht ladbar ({e}) -- Formfilter der Masken aus.')

        # --- Subscriber ---
        self.sub_control_mode = self.create_subscription(
            UInt8, '/control/mode', self.cb_control_mode, 1)
        # Tiefe 1: die Verarbeitung ist langsamer als 30 Bilder/s, nur das neueste zaehlt.
        self.sub_raw_image = self.create_subscription(Image, '/lokal/raw_image', self.cbGetRawImage, 1)

        self.sub_bev = self.create_subscription(Image, '/lokal/bev', self.cb_bev, 1)
        # Nur fuers Mitschieben einer gehaltenen Kreuzung.
        self.sub_odom = self.create_subscription(Odometry, '/odom_filtered', self.cb_odom, 1)

        # --- Publisher ---
        self.pub_lane_error = self.create_publisher(Float64, '/detect/lane', 1)
        self.pub_debug_image = self.create_publisher(FollowLaneDebug, '/detect/lane_debug', 1)
        # Muss auf /compressed enden: image_transport deutet das letzte Namenssegment als Transport
        self.pub_mask_debug = self.create_publisher(CompressedImage, '/detect/mask_debug/compressed', 1)
        # Verlustfreie Pixelkoordinaten der Linienpunkte fuer lane_costmap_node, getrennt
        # vom komprimierten Debug-Bild oben, das nur zum Anschauen gedacht ist.
        self.pub_lane_mask = self.create_publisher(LaneMask, '/detect/lane_mask', 1)
        self.pub_lines_lost = self.create_publisher(Bool, '/detect/lines_lost', 1)
        # Schranke: True = Schwelle erreicht (switch_control schaltet auf Stop), Zahl der
        # blauen Pixel fuers Debugbild, Rohbild mit blauer Maske zum Einstellen.
        self.pub_barrier = self.create_publisher(Bool, '/detect/barrier', 1)
        self.pub_barrier_pixels = self.create_publisher(Int32, '/detect/barrier_pixels', 1)
        self.pub_stop_sign = self.create_publisher(Bool, '/detect/stop_sign', 1)
        self.pub_stop_sign_pixels = self.create_publisher(Int32, '/detect/stop_sign_pixels', 1)
        self.pub_barrier_debug = self.create_publisher(
            CompressedImage, '/detect/barrier_debug/compressed', 1)
        # Kreuzungsfeld (Mitte zwischen den Ecken, BEV-Pixel), sobald bestaetigt; sonst leer.
        self.pub_intersection = self.create_publisher(PolygonStamped, '/detect/intersection', 1)
        # YOLO-Boxen im Rohbild (Pixel), Header = der des Rohbilds.
        self.pub_objects = self.create_publisher(Detection2DArray, '/detect/objects', 1)
        self.pub_signs = {name: self.create_publisher(Bool, f'/detect/sign/{name}', 1)
                          for name in SIGN_CLASSES}
        # Start-/Ziellinie (YOLO 'start_line'): je Box [Abstand Unterkante -> unterer Bildrand
        # (px im Rohbild), linker Rand, rechter Rand (Anteil der Bildbreite 0..1)], leer = keine.
        # switch_control entscheidet daraus, ob gehalten wird (Lauf beendet).
        self.pub_start_line = self.create_publisher(Float32MultiArray, '/detect/start_line', 1)
        # Nur im Modus Intersection: naechstes gelb/weiss-Paar ab der Suchzeile nach vorn,
        # [lane_error, Abstand Paar (m ab Radachse), Abstand Suchzeile (m)]; leer = keins.
        self.pub_lane_far = self.create_publisher(Float64MultiArray, '/detect/lane_far', 1)
        # Bildalter (Kamera-Stempel -> Ankunft hier), Raten und Rechenzeiten, siehe publish_timing.
        self.pub_timing = self.create_publisher(String, '/detect/timing', 1)
        self.timer_timing = self.create_timer(TIMING_PERIOD_S, self.publish_timing)

        # --- Zustand ---
        self.bridge = CvBridge()
        self.control_mode = None
        self.raw_image = None    # letztes /lokal/raw_image (Schranke)
        self._barrier_debug_time = 0.0
        self.is_running = False
        self.img_height = None   # Groesse des Bilds, pro Frame gesetzt
        self.img_width = None
        # Je Linie (Spalte, Zeitpunkt der letzten Detektion) oder None; eigener Logger
        # je Linie, damit die gedrosselten Meldungen sich nicht gegenseitig verschlucken.
        self._anchor = {'Gelb': None, 'Weiss': None}
        self._side_mode = None          # color_side beim letzten Bild (Wechsel -> neu suchen)
        self._side_auto = None          # auto: entschieden getauscht? None = noch am Suchen
        self._side_votes = []           # auto: je Bild True (getauscht) / False (normal)
        self._side_prev = None          # recheck: alte Entscheidung, gilt bis zur neuen
        self._line_log = {name: self.get_logger().get_child(name.lower()) for name in self._anchor}
        self._intersection_hits = deque(maxlen=INTERSECTION_CONFIRM[1])
        self._intersection_seen = False
        # Nach der Kreuzung bis zu diesem Zeitpunkt (monotonic) PID auf das ferne Paar, None = aus.
        self._exit_far_until = None
        # Zuletzt erkannte Kreuzung: Flaechen am Boden (m, base_footprint), Odometrie-Pose
        # dazu, Zeitpunkt (monotonic); None = keine.
        self._held = None
        self._odom_pose = None
        self._detector_name = self.detector
        self.yolo = self.load_detector(self.detector)
        self._yolo_times = []    # Dauer je Bild (ms) fuers gedrosselte Log
        self._detect_last = None   # monotonic des letzten Laufs (detector_rate_hz)
        self._no_entry_time = None  # monotonic der letzten 'no_entry'-Box (Stoppschild-Sperre)
        self._no_entry_score = 0.0
        self._timing = defaultdict(list)   # Messreihe -> Werte seit dem letzten publish_timing
        self._last_arrival = {}            # 'raw'/'bev'/'pid' -> monotonic der letzten Ankunft
        self._timing_count = 0
        self._timing_log = defaultdict(list)   # dasselbe ueber TIMING_LOG_EVERY Sekunden

        self.get_logger().info('Started detect_lane Node (Nav2-Zweig).')

    def load_detector(self, name):
        """YoloTRT/RfDetrTRT oder None ('aus', keine Engine, keine GPU, falsche
        TensorRT-Version). name: Kurzname aus DETECTORS oder Pfad relativ zum Repo-Ordner."""
        if not name or name.lower() in ('aus', 'none', 'off'):
            self.get_logger().info('Objekterkennung aus (detector: aus).')
            return None
        rel = DETECTORS.get(name, name)
        # Repo-Ordner: vom Install-Ordner des Pakets aufwaerts, bis die Datei auftaucht.
        path = rel if os.path.isabs(rel) and os.path.isfile(rel) else None
        d = get_package_share_directory('Follow_Lane')
        while path is None and d != os.path.dirname(d):
            if os.path.isfile(os.path.join(d, rel)):
                path = os.path.join(d, rel)
            d = os.path.dirname(d)
        if path is None:
            self.get_logger().warn(f'{rel} nicht gefunden (Kurznamen: {", ".join(DETECTORS)}) '
                                   '-- Objekterkennung aus.')
            return None
        try:
            from Follow_Lane.src.yolo_trt import YoloTRT
            from Follow_Lane.src.rfdetr_trt import RfDetrTRT
            if 'rfdetr' in os.path.basename(path).lower():
                model = RfDetrTRT(path, conf=RFDETR_CONF)
            else:
                model = YoloTRT(path, conf=YOLO_CONF, iou=YOLO_IOU)
        except Exception as e:  # noqa: BLE001 -- ohne GPU/TensorRT laeuft der Rest weiter
            self.get_logger().warn(f'{path} nicht ladbar ({e}) -- Objekterkennung aus.')
            return None
        self.get_logger().info(f'{model.kind} geladen: {path} ({len(model.names)} Klassen, '
                               f'Eingang {model.in_w}x{model.in_h}).')
        return model

    def switch_detector(self):
        """Parameter 'detector' live geaendert -> altes Modell freigeben, neues laden."""
        if self.detector == self._detector_name:
            return
        self._detector_name = self.detector
        if self.yolo is not None:
            self.yolo.close()
        self.yolo = self.load_detector(self.detector)
        self._yolo_times = []
        # Boxen des alten Modells nicht stehen lassen.
        self.publish_detections([])

    # --- Callbacks ---

    def cbGetRawImage(self, img):
        self.raw_image = img
        self.note_arrival('raw', img.header.stamp)
        read_params(self, ['detector', 'detector_rate_hz'])
        self.switch_detector()
        self.detectBarrier()
        self.vision_detection()

    def cb_odom(self, msg):
        p = msg.pose.pose
        self._odom_pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def cb_control_mode(self, msg):
        new_mode = control_mode_from_msg(self, msg)
        if new_mode is None or new_mode == self.control_mode:
            return
        if new_mode == ControlType.Tunnel:
            self.get_logger().info('Tunnel -- Spurerkennung pausiert.')
        elif self.control_mode == ControlType.Tunnel:
            self.get_logger().info('Tunnel verlassen -- Spurerkennung laeuft wieder.')
        if new_mode == ControlType.Lane and self.control_mode not in LINE_TRACK_MODES:
            # Zurueck in die Spurfolge (nach Maneuver, Stop ...): Linien neu suchen statt der
            # letzten Position zu folgen. Beim Drehen wandern die Linien durchs Bild, die
            # Verfolgung rastete auf der rechten Kante des LINKEN gelben Streifens ein (Lauf 11.11.).
            # Ohne Anker nimmt get_x_for_driving die aeusserste Kante zur Spur hin.
            self._anchor = {name: None for name in self._anchor}
            self.get_logger().info('Spurfolge (wieder) aktiv -- Linien neu gesucht.')
        if new_mode == ControlType.Lane and self.control_mode in RECHECK_AFTER_MODES:
            self._recheck_color_side(self.control_mode)
        if self.control_mode == ControlType.Intersection and new_mode == ControlType.Lane:
            self._exit_far_until = time.monotonic() + EXIT_FAR_S
            self.get_logger().info('Nach Kreuzung: PID auf fernes Paar, bis die Suchzeile Linien hat.')
        elif new_mode != ControlType.Lane:
            self._exit_far_until = None
        self.control_mode = new_mode

    def cb_bev(self, img):
        if self.control_mode is None or self.control_mode == ControlType.Tunnel:
            return
        if self.is_running:
            return
        self.is_running = True
        self.note_arrival('bev', img.header.stamp)
        t0 = time.monotonic()
        # Alles im try: sonst bliebe das Flag nach einer Exception dauerhaft True und
        # der Node verarbeitete still keine Bilder mehr.
        try:
            self.find_lane(img)
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'Spurerkennung fehlgeschlagen: {e}')
        finally:
            self.is_running = False
            self._timing['lane_ms'].append((time.monotonic() - t0) * 1000.0)

    # --- Hilfsmethoden ---

    def find_lane(self, img):
        read_params(self, self._param_names)

        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        self.img_height, self.img_width = cv_image.shape[:2]

        mask_yellow, mask_white = self.color_masks(cv_image)
        kernel = np.ones((ERODE_KERNEL, ERODE_KERNEL), np.uint8)
        mask_yellow = cv2.erode(mask_yellow, kernel, iterations=1)
        mask_white = cv2.erode(mask_white, kernel, iterations=1)
        # Eigenen Bot (Magnet) in beiden Masken schwarz: gilt damit fuer PID, Kreuzung,
        # Linienende (TunnelEntry), Costmap und das Masken-Debugbild.
        self.self_mask(mask_yellow)
        self.self_mask(mask_white)
        # Formfilter (shape_filter) bewusst NICHT hier: er stanzt runde Loecher in echte
        # Linien, wo sie in der BEV breiter als LINE_MAX_WIDTH_CM wirken (Querlinien sind in
        # Fahrtrichtung verschmiert, Abzweige). Er wirkt nur auf die Costmap im Obstacle-Modus
        # (publish_lane_mask).

        width = self.img_width
        white_alternative = int(width * WHITE_ALTERNATIVE)
        yellow_alternative = int(width * YELLOW_ALTERNATIVE)
        now = time.monotonic()
        # Vor track_line: das versetzt die Anker schon auf dieses Bild.
        if self.control_mode == ControlType.TunnelEntry:
            self.pub_lines_lost.publish(Bool(data=self.lines_lost(mask_yellow, mask_white, now)))

        # Gelb fest am konfigurierten detection_row_factor
        center_yellow_y = int(self.img_height * self.detection_row_factor)
        raw_yellow = self.get_x_for_driving(
            mask_yellow, center_yellow_y, left_line=True,
            last_known=self.anchor_column('Gelb', now))
        center_yellow_x = self.track_line('Gelb', raw_yellow, yellow_alternative, now)

        # Weisse Maske: alles links von gelb + min_lane_width ausblenden
        mask_white_filtered = mask_white.copy()
        corridor_start = max(0, int(center_yellow_x) + self.min_lane_width)
        mask_white_filtered[:, :corridor_start] = 0

        # Weiss sucht weiter unten, falls nicht gefunden
        last_white = self.anchor_column('Weiss', now)
        detection_row_factor = self.detection_row_factor
        center_white_y = int(self.img_height * detection_row_factor)
        raw_white = self.get_x_for_driving(
            mask_white_filtered, center_white_y, left_line=False, last_known=last_white)
        while raw_white is None and detection_row_factor <= 0.95:
            detection_row_factor += 0.05
            center_white_y = int(self.img_height * detection_row_factor)
            raw_white = self.get_x_for_driving(
                mask_white_filtered, center_white_y, left_line=False, last_known=last_white)
        center_white_x = self.track_line('Weiss', raw_white, white_alternative, now)

        # Laut statt still raten: Gelb gefunden, Weiss rechts davon nicht, aber links davon
        # -> der Bot steht auf der falschen Spurseite oder quer zur Spur. Die Ersatzspalten
        # unten ergeben dann einen geratenen Spurfehler (am 12.11. genau 0.0, Spur "verloren").
        if raw_yellow is not None and raw_white is None:
            left_of_yellow = mask_white.copy()
            left_of_yellow[:, int(center_yellow_x):] = 0
            if self.get_x_for_driving(left_of_yellow, center_yellow_y, left_line=False) is not None:
                self.get_logger().warn(
                    'Weiss LINKS von Gelb -- falsche Spurseite oder quer zur Spur? '
                    'Spurfehler ist geraten (Ersatzspalten).', throttle_duration_sec=2.0)

        if center_white_x <= center_yellow_x:
            if center_white_x > int(width * 0.4):
                center_yellow_x = yellow_alternative
            else:
                center_white_x = white_alternative

        lane_center = (center_white_x + center_yellow_x) / 2
        lane_error = 1 - (lane_center / width * 2)

        # Direkt nach der Kreuzung (intersection_node hat an Lane zurueckgegeben, sobald das
        # Paar dahinter sicher war): solange an der Suchzeile noch eine Linie fehlt, regelt der
        # PID auf das ferne Paar. Endet, sobald beide Linien an der Suchzeile gefunden sind.
        if self._exit_far_until is not None:
            if raw_yellow is not None and raw_white is not None:
                self.get_logger().info('Nach Kreuzung: Linien an der Suchzeile -- normal weiter.')
                self._exit_far_until = None
            elif now > self._exit_far_until:
                self.get_logger().warn('Nach Kreuzung: Suchzeile nach '
                                       f'{EXIT_FAR_S:.0f} s noch leer -- normal weiter.')
                self._exit_far_until = None
            else:
                _, pair = self.far_lane_pair(mask_yellow, mask_white)
                if pair is not None:
                    center_yellow_x, center_white_x, row = pair
                    center_yellow_y = center_white_y = row
                    # Anker setzen: die Linien laufen von dort in die Suchzeile hinein.
                    self._anchor['Gelb'] = (center_yellow_x, now)
                    self._anchor['Weiss'] = (center_white_x, now)
                    lane_center = (center_white_x + center_yellow_x) / 2
                    lane_error = 1 - (lane_center / width * 2)

        polies, horizontal, vertical = self.detect_intersection(
            mask_white, mask_yellow, (center_yellow_x, center_yellow_y),
            (center_white_x, center_white_y), img.header.stamp)
        if self.control_mode == ControlType.Intersection:
            self.pub_lane_far.publish(self.far_lane_pair(mask_yellow, mask_white)[0])

        self.pub_lane_error.publish(Float64(data=lane_error))
        # Alter beim Senden an den PID (control_driving rechnet sofort, < 1 ms).
        self.note_arrival('pid', img.header.stamp)
        self.pub_debug_follow_lane(center_yellow_x, center_yellow_y, center_white_x, center_white_y,
                                   lane_error, int(lane_center), cv_image, polies, horizontal,
                                   vertical, img.header)
        self.publish_mask_debug(mask_yellow, mask_white,
                                center_yellow_x, center_yellow_y, center_white_x, center_white_y)
        # Ungefilterte Masken: die links der gelben Linie ausgeblendeten weissen Pixel
        # braucht nur der PID; fuer die Costmap trennt lane_costmap_node selbst.
        self.publish_lane_mask(mask_yellow, mask_white, img.header.stamp)

    def lines_lost(self, mask_yellow, mask_white, now):
        """Fehlen beide Spurlinien in der Suchzeile? Mit Formfilter: eine helle Tunnelwand ist
        keine Linie. Eine Kante weiter als LINE_MAX_JUMP_CM vom Anker gilt nicht als Linie."""
        row = int(self.img_height * self.detection_row_factor)
        max_jump = self.max_jump_px()
        for name, mask, left in (('Gelb', mask_yellow, True), ('Weiss', mask_white, False)):
            last = self.anchor_column(name, now)
            raw = self.get_x_for_driving(self.shape_filter(mask), row, left_line=left, last_known=last)
            if raw is not None and (last is None or max_jump is None or abs(raw - last) <= max_jump):
                return False
        return True

    def publish_mask_debug(self, mask_yellow, mask_white,
                           center_yellow_x, center_yellow_y, center_white_x, center_white_y):
        if self.pub_mask_debug.get_subscription_count() == 0:
            return

        overlay = np.zeros((mask_yellow.shape[0], mask_yellow.shape[1], 3), np.uint8)
        both = (mask_yellow > 0) & (mask_white > 0)
        overlay[mask_white > 0] = (255, 255, 255)
        overlay[mask_yellow > 0] = (0, 255, 255)
        overlay[both] = (0, 0, 255)

        cv2.circle(overlay, (center_yellow_x, center_yellow_y), 6, (0, 255, 0), -1, cv2.LINE_AA)
        cv2.circle(overlay, (center_white_x, center_white_y), 6, (255, 0, 255), -1, cv2.LINE_AA)

        success, encoded = cv2.imencode('.jpg', overlay)
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        # array('B') statt bytes: rclpy prueft bytes sonst Byte fuer Byte in Python (~0,7 s/MB)
        msg.data = array.array('B', encoded.tobytes())
        self.pub_mask_debug.publish(msg)

    def publish_lane_mask(self, mask_yellow, mask_white, source_stamp):
        """Linienpixel der BEV-Masken fuer lane_costmap_node (/detect/lane_mask).

        Die Koordinaten sind Pixel in /lokal/bev; lane_costmap_node rechnet sie ueber
        Parameter/bev.yaml in Meter um. source_stamp ist der Stempel des Kamerabilds:
        Nav2s obstacle_layer transformiert die Spurwolke per TF auf genau diesen
        Zeitpunkt, ein spaeterer (now()) verschoebe die Waende gegen die Roboterpose.
        """
        # Nur berechnen, wenn jemand zuschaut.
        if self.pub_lane_mask.get_subscription_count() == 0:
            return

        # Schon hier ausduennen: lane_costmap_node verwirft ohnehin die meisten Punkte
        # wieder, in voller Aufloesung waeren es zehntausende int32 pro Frame.
        # Nur im Obstacle-Modus: helle Flaechen aus den Spurwaenden nehmen, damit Nav2 beim
        # Ausweichen nicht an Scheinwaenden scheitert.
        if self.control_mode == ControlType.Obstacle:
            mask_yellow = self.shape_filter(mask_yellow)
            mask_white = self.shape_filter(mask_white)

        stride = max(1, int(LANE_MASK_STRIDE))
        yellow_ys, yellow_xs = np.where(mask_yellow[::stride, ::stride] > 0)
        white_ys, white_xs = np.where(mask_white[::stride, ::stride] > 0)

        msg = LaneMask()
        msg.header.stamp = source_stamp
        msg.image_height, msg.image_width = mask_yellow.shape[:2]
        msg.yellow_x = (yellow_xs * stride).astype(np.int32).tolist()
        msg.yellow_y = (yellow_ys * stride).astype(np.int32).tolist()
        msg.white_x = (white_xs * stride).astype(np.int32).tolist()
        msg.white_y = (white_ys * stride).astype(np.int32).tolist()
        self.pub_lane_mask.publish(msg)

    def pub_debug_follow_lane(self, center_yellow_x, center_yellow_y, center_white_x,
                              center_white_y, lane_error, lane_center, img, polies,
                              horizontal, vertical, header):
        info = FollowLaneDebug()
        # Stempel des BEV-Bilds: debug_view misst damit Alter und Versatz zum Rohbild.
        info.raw_image.header = header
        info.center_white_x = center_white_x
        info.center_white_y = center_white_y
        info.center_yellow_x = center_yellow_x
        info.center_yellow_y = center_yellow_y
        info.lane_center = lane_center
        info.lane_error = lane_error
        # 0 = das Bild ist schon die BEV; die RoI-Felder der Nachricht bleiben leer.
        info.crop_im_size = 0

        success, encoded = cv2.imencode('.jpg', img)
        if success:
            info.raw_image.format = 'jpeg'
            # array('B') statt bytes: rclpy prueft bytes sonst Byte fuer Byte in Python (~0,7 s/MB)
            info.raw_image.data = array.array('B', encoded.tobytes())

        for poly in polies or []:
            p = Polygon()
            p.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in poly.reshape(-1, 2)]
            info.polygons.append(p)

        for lines, field in ((horizontal, info.horizontal_lines),
                             (vertical, info.vertical_lines)):
            for p1, p2 in lines or []:
                field.append(Polygon(points=[Point32(x=float(p1[0]), y=float(p1[1]), z=0.0),
                                             Point32(x=float(p2[0]), y=float(p2[1]), z=0.0)]))

        self.pub_debug_image.publish(info)

    def lines_to_poly(self, img, y_offset):
        polies = []
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE,
                                       offset=(0, y_offset))
        for contour in contours:
            if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
                continue
            polies.append(cv2.approxPolyDP(contour, epsilon=POLY_EPSILON, closed=True))
        return polies

    def line_dir(self, p1, p2):
        d = np.asarray(p2, float) - np.asarray(p1, float)
        return np.degrees(np.arctan2(d[1], d[0])) % 180.0

    def angle_diff(self, a, b):
        d = abs(a - b) % 180.0
        return min(d, 180.0 - d)

    def far_lane_pair(self, mask_yellow, mask_white):
        """Naechstes Spurpaar ab der Suchzeile nach vorn (Queren der Kreuzung).

        Wie find_lane, aber ohne Tracking (die Linien hinter der Kreuzung tauchen erst
        fern auf): je Band die rechte Kante von gelb (die rechteste) und die linke Kante
        von weiss (die linkeste rechts von gelb + min_lane_width), Median ueber die Zeilen.
        Das erste Band mit plausibler Spurbreite gewinnt. Querlinien der Kreuzung haben
        kaum senkrechte Kanten und stoeren deshalb wenig.
        Rueckgabe: (Nachricht fuer /detect/lane_far, (Spalte gelb, Spalte weiss, Zeile) oder None)."""
        out = Float64MultiArray()
        res = self._res()
        x_max = self._bev['x_max'] if self._bev is not None else 0.60
        yellow = -cv2.Sobel(mask_yellow, cv2.CV_16S, 1, 0, ksize=3) > 127
        white = cv2.Sobel(mask_white, cv2.CV_16S, 1, 0, ksize=3) > 127
        max_width = FAR_MAX_LANE_WIDTH_M / res
        start = int(self.img_height * self.detection_row_factor)
        for row in range(start, FAR_BAND_PX - 1, -FAR_ROW_STEP_PX):
            rows = range(max(0, row - FAR_BAND_PX), min(self.img_height, row + FAR_BAND_PX))
            ys = [hits[-1] for hits in (np.flatnonzero(yellow[r]) for r in rows) if hits.size]
            if len(ys) < FAR_MIN_ROWS:
                continue
            yx = float(np.median(ys))
            ws = []
            for r in rows:
                hits = np.flatnonzero(white[r])
                hits = hits[hits >= yx + self.min_lane_width]
                if hits.size:
                    ws.append(hits[0])
            if len(ws) < FAR_MIN_ROWS:
                continue
            wx = float(np.median(ws))
            if wx - yx > max_width:
                continue
            lane_error = 1 - ((yx + wx) / 2 / self.img_width * 2)
            out.data = [float(lane_error), x_max - (row + 0.5) * res,
                        x_max - (start + 0.5) * res]
            return out, (int(yx), int(wx), row)
        return out, None

    def detect_intersection(self, img_white, img_yellow, yellow_point, white_point, stamp):
        """Kreuzung vor dem Bot: Polygone, Quer- und Laengsgeraden fuers Debugbild.

        Je Seite wird die eigene Linie an der vom PID verfolgten Stelle gesucht und ihre
        Kontur von der zur Spur zeigenden Laengskante aus nach vorn abgelaufen. Knickt
        sie nach aussen ab (Querkante, ~90 Grad), ist der Schnittpunkt der beiden
        verlaengerten Kanten die Ecke der Kreuzung. Ob die Ecke rechtwinklig oder
        abgerundet ist, aendert den Schnittpunkt nicht: Boegen werden uebersprungen,
        es zaehlen nur die geraden Kanten davor und danach.

        Endet die eigene Linie ohne Knick, zaehlt auch eine Querlinie, die bis CORNER_GAP_M
        neben ihrem Ende beginnt und nach aussen weggeht (_gap_corner, Ecken mit Luecke).

        Ecke links und rechts etwa gleich weit voraus = Kreuzung, die Verbindung beider
        Ecken ist die Haltelinie. Nur eine Ecke und die Gegenlinie laeuft gerade weiter =
        Einmuendung von der Seite (T-Kreuzung, _straight_past): Haltelinie von der Ecke quer bis
        zur Gegenlinie. Biegt die Gegenlinie mit ab, ist es eine Kurve. Gemeldet
        wird erst nach INTERSECTION_CONFIRM Bildern; fehlt sie danach in einzelnen Bildern,
        gilt bis INTERSECTION_HOLD_S die letzte, per Odometrie mitgeschobene Lage.
        """
        # Schliessen: am Farbwechsel (weiss -> gelb) liegen nach der Erosion beide Linien
        # getrennt; die Kontur soll ueber die Ecke hinweg durchlaufen.
        full = cv2.morphologyEx(cv2.bitwise_or(img_white, img_yellow), cv2.MORPH_CLOSE,
                                np.ones((CLOSE_KERNEL, CLOSE_KERNEL), np.uint8))
        polies = self.lines_to_poly(self.shape_filter(full), 0)

        horizontal, vertical, corners = [], [], {}
        for side, point, outward in (('links', yellow_point, -1), ('rechts', white_point, 1)):
            found = self._corner_of_line(polies, point, outward)
            if found is None:
                found = self._gap_corner(polies, point, outward)
            if found is None:
                continue
            corner, long_end, cross_end = found
            corners[side] = corner
            vertical.append((long_end, corner))
            horizontal.append((corner, cross_end))

        stop_line = None
        if len(corners) == 2:
            dx = abs(corners['links'][1] - corners['rechts'][1]) * self._res()
            if dx <= MAX_CORNER_DX_M:
                stop_line = (corners['links'], corners['rechts'])
        elif len(corners) == 1:
            # Einmuendung: Haltelinie von der Ecke quer bis zur Innenkante der Gegenlinie,
            # sofern diese gerade weiterlaeuft (sonst Kurve).
            side, corner = next(iter(corners.items()))
            if side == 'links':
                other_x = self._straight_past(polies, white_point, img_white, corner[1], +1)
                if other_x is not None:
                    stop_line = (corner, np.array([other_x, corner[1]]))
            else:
                other_x = self._straight_past(polies, yellow_point, img_yellow, corner[1], -1)
                if other_x is not None:
                    stop_line = (np.array([other_x, corner[1]]), corner)
        self._report_intersection(stop_line, corners)

        # Debug: Haltelinie, Kreuzungsflaechen und kleine Rauten an den Ecken als
        # Polygone (rot).
        marks = []
        areas = None
        if stop_line is not None:
            areas = self.intersection_areas(stop_line, full)
            if self._intersection_seen:
                self._remember_intersection(areas)
        elif self._intersection_seen:
            areas = self._recall_intersection()
        center = None
        if areas is not None:
            center = areas['mitte']
            # Haltelinie = Unterkante der Mitte (unten links -> unten rechts)
            marks.append(np.array([center[3], center[2]], np.int32).reshape(-1, 1, 2))
            for area in areas.values():
                marks.append(area.reshape(-1, 1, 2))
        # Bestaetigte Kreuzung (Mitte, BEV-Pixel) fuers Rohbild-Debug und intersection_node;
        # leer = keine. Stempel des Kamerabilds: intersection_node legt die Haltelinie per
        # TF zu genau diesem Zeitpunkt in odom ab.
        out = PolygonStamped()
        out.header.stamp = stamp
        out.header.frame_id = 'bev'
        if center is not None and self._intersection_seen:
            out.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in center]
        self.pub_intersection.publish(out)
        for c in corners.values():
            x, y = int(c[0]), int(c[1])
            marks.append(np.array([[x, y - 8], [x + 8, y], [x, y + 8], [x - 8, y]],
                                  np.int32).reshape(-1, 1, 2))
        return marks, horizontal, vertical

    def _corner_of_line(self, polies, point, outward):
        """Ecke (x, y), Ende der Laengs- und der Querkante (fuers Debugbild) oder None.
        outward: -1 links (Querkante muss nach links weggehen), +1 rechts."""
        px, py = float(point[0]), float(point[1])
        best = None
        for poly in polies:
            pts = poly.reshape(-1, 2).astype(float)
            n = len(pts)
            for i in range(n):
                p1, p2 = pts[i], pts[(i + 1) % n]
                if np.linalg.norm(p2 - p1) < MIN_EDGE_LEN:
                    continue
                if self.angle_diff(self.line_dir(p1, p2), 90.0) > LONG_TOL_DEG:
                    continue
                if not min(p1[1], p2[1]) - 5 <= py <= max(p1[1], p2[1]) + 5:
                    continue
                dist = abs(self._x_at(p1, p2, py) - px)
                if best is None or dist < best[0]:
                    best = (dist, pts, i)
        # Die verfolgte Stelle liegt auf der zur Spur zeigenden Kante (wenige Pixel).
        if best is None or best[0] > 15:
            return None
        _, pts, i = best
        n = len(pts)
        a, b = pts[i], pts[(i + 1) % n]
        # Nach vorn (kleineres y) laufen: in Richtung des vorderen Endpunkts.
        step = 1 if b[1] < a[1] else -1
        idx = (i + 1) % n if step == 1 else i
        long_start, long_end = (a, b) if step == 1 else (b, a)
        long_dir = self.line_dir(long_start, long_end)

        arc = 0.0
        for _ in range(n):
            q1 = pts[idx]
            q2 = pts[(idx + step) % n]
            seg = np.linalg.norm(q2 - q1)
            ang = self.line_dir(q1, q2)
            if seg >= MIN_EDGE_LEN and self.angle_diff(ang, long_dir) <= LONG_TOL_DEG / 2:
                # Laengskante geht weiter (Polygon hat sie geteilt).
                long_end, arc = q2, 0.0
            elif seg * self._res() >= MIN_CROSS_LEN_M \
                    and abs(self.angle_diff(ang, long_dir) - 90.0) <= CROSS_TOL_DEG:
                if np.sign(q2[0] - q1[0]) != outward:
                    return None             # knickt nach innen: Kurve, keine Kreuzung
                corner = self._intersect(long_start, long_end, q1, q2)
                if corner is None or corner[1] > long_start[1]:
                    return None
                return corner, long_start, q2
            else:
                arc += seg
                if arc * self._res() > MAX_BEND_ARC_M:
                    return None
            idx = (idx + step) % n
        return None

    def _gap_corner(self, polies, point, outward):
        """Ecke ohne durchgehende Linie: die eigene Linie endet vorn, eine Querkante beginnt bis
        CORNER_GAP_M daneben und geht nach aussen (outward) weg. Ecke = Schnittpunkt der
        verlaengerten Innenkante mit der Querkante. Gleiches Ergebnis wie _corner_of_line oder
        None."""
        px, py = float(point[0]), float(point[1])
        best = None
        for k, poly in enumerate(polies):
            pts = poly.reshape(-1, 2).astype(float)
            for i in range(len(pts)):
                p1, p2 = pts[i], pts[(i + 1) % len(pts)]
                if np.linalg.norm(p2 - p1) < MIN_EDGE_LEN \
                        or self.angle_diff(self.line_dir(p1, p2), 90.0) > LONG_TOL_DEG:
                    continue
                if not min(p1[1], p2[1]) - 5 <= py <= max(p1[1], p2[1]) + 5:
                    continue
                dist = abs(self._x_at(p1, p2, py) - px)
                if best is None or dist < best[0]:
                    best = (dist, k, pts, p1, p2)
        if best is None or best[0] > 15:
            return None
        _, own_k, own, a, b = best
        long_start, long_end = (a, b) if b[1] < a[1] else (b, a)      # vorn = kleineres y
        # Vorderes Ende der eigenen Linie: weitester Punkt ihres Polygons in Laengsrichtung.
        fwd = (long_end - long_start) / max(np.linalg.norm(long_end - long_start), 1e-6)
        tip_y = float(own[np.argmax((own - long_start) @ fwd)][1])
        tip = np.array([self._x_at(long_start, long_end, tip_y), tip_y])
        long_dir = self.line_dir(long_start, long_end)
        gap_px = CORNER_GAP_M / self._res()
        own_contour = own.astype(np.float32).reshape(-1, 1, 2)
        found = None
        for k, poly in enumerate(polies):
            if k == own_k:
                continue
            pts = poly.reshape(-1, 2).astype(float)
            for i in range(len(pts)):
                q1, q2 = pts[i], pts[(i + 1) % len(pts)]
                if np.linalg.norm(q2 - q1) * self._res() < MIN_CROSS_LEN_M \
                        or abs(self.angle_diff(self.line_dir(q1, q2), long_dir) - 90.0) > CROSS_TOL_DEG:
                    continue
                near, far = (q1, q2) if np.linalg.norm(q1 - tip) <= np.linalg.norm(q2 - tip) else (q2, q1)
                if np.sign(far[0] - near[0]) != outward:
                    continue                    # geht nach innen (ueber die eigene Spur)
                # Luecke = Abstand zum Umriss der eigenen Linie (nicht zur Innenkante: sonst zaehlte
                # die Linienbreite mit).
                dist = max(0.0, -cv2.pointPolygonTest(own_contour, (float(near[0]), float(near[1])), True))
                if dist > gap_px or (near[0] - tip[0]) * outward < -gap_px / 2:
                    continue
                if found is None or dist < found[0]:
                    found = (dist, near, far)
        if found is None:
            return None
        _, near, far = found
        corner = self._intersect(long_start, long_end, near, far)
        if corner is None or corner[1] > long_start[1] or np.linalg.norm(corner - tip) > 2 * gap_px:
            return None
        return corner, long_start, far

    def _straight_past(self, polies, point, mask, corner_y, outward):
        """x der Innenkante der Gegenlinie auf Hoehe corner_y, wenn sie dort gerade verlaeuft
        und mindestens SIDE_T_AHEAD_M darueber hinaus (nach vorn) weitergeht; sonst None.
        outward: +1 rechte (weisse) Linie, Pixel rechts der Kante; -1 linke (gelbe)."""
        px, py = float(point[0]), float(point[1])
        best = None
        for poly in polies:
            pts = poly.reshape(-1, 2).astype(float)
            for i in range(len(pts)):
                p1, p2 = pts[i], pts[(i + 1) % len(pts)]
                if np.linalg.norm(p2 - p1) < MIN_EDGE_LEN \
                        or self.angle_diff(self.line_dir(p1, p2), 90.0) > LONG_TOL_DEG:
                    continue
                if not min(p1[1], p2[1]) - 5 <= py <= max(p1[1], p2[1]) + 5:
                    continue
                dist = abs(self._x_at(p1, p2, py) - px)
                if best is None or dist < best[0]:
                    best = (dist, p1, p2)
        ahead = int(round(SIDE_T_AHEAD_M / self._res()))
        y0, y1 = int(corner_y) - ahead, int(corner_y)
        if y0 < 0:
            return None                     # zu weit voraus, um es zu pruefen
        # Innenkante bis auf Hoehe der Ecke verlaengern; ohne passende Kante (Versatz an einer
        # Mattenkante genau in der Suchzeile) senkrecht ueber dem verfolgten Punkt.
        x = self._x_at(best[1], best[2], corner_y) if best is not None and best[0] <= 15 else px
        lo, hi = (x - SIDE_T_TOL_PX, x + SIDE_T_OUT_PX) if outward > 0 \
            else (x - SIDE_T_OUT_PX, x + SIDE_T_TOL_PX)
        lo, hi = max(0, int(lo)), min(mask.shape[1], int(hi) + 1)
        if hi <= lo:
            return None
        hits = int(np.count_nonzero(np.any(mask[y0:y1 + 1, lo:hi] > 0, axis=1)))
        if hits < SIDE_T_ROWS * (y1 + 1 - y0):
            return None
        return float(x)

    @staticmethod
    def _x_at(p1, p2, y):
        if abs(p2[1] - p1[1]) < 1e-6:
            return (p1[0] + p2[0]) / 2.0
        return p1[0] + (y - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1])

    @staticmethod
    def _intersect(a1, a2, b1, b2):
        """Schnittpunkt der Geraden a1-a2 und b1-b2, None wenn (fast) parallel."""
        d1, d2 = a2 - a1, b2 - b1
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-6:
            return None
        t = ((b1[0] - a1[0]) * d2[1] - (b1[1] - a1[1]) * d2[0]) / den
        return a1 + t * d1

    def intersection_areas(self, stop_line, mask):
        bot_l, bot_r = (np.asarray(p, float) for p in stop_line)
        width = bot_r[0] - bot_l[0]
        y_near = min(bot_l[1], bot_r[1])
        y_top = self.get_y_for_driving(mask, bot_l[0] + 10, bot_r[0] - 10, y_near, None)
        if y_top is None:
            y_top = max(0.0, y_near - width)
        top_l, top_r = np.array([bot_l[0], y_top]), np.array([bot_r[0], y_top])
        arm = np.array([width, 0.0])
        return {
            'mitte': np.array([top_l, top_r, bot_r, bot_l], np.int32),
            'links': np.array([top_l - arm, top_l, bot_l, bot_l - arm], np.int32),
            'rechts': np.array([top_r, top_r + arm, bot_r + arm, bot_r], np.int32),
        }

    def get_y_for_driving(self, mask, x_from, x_to, y_start, no_lane_value, min_gap=5):
        """Gegenstueck zu get_x_for_driving, aber spaltenweise: sucht in den Spalten
        x_from..x_to von y_start aus NACH OBEN die naechste waagerechte Kante
        (z.B. die querliegende Linie am Ende einer ┴-Kreuzung).
        min_gap: Zeilen direkt ueber y_start ignorieren, damit nicht die Kante
        an der Ecke selbst gefunden wird. (aus detect_lane.py, Kalibrierung-Tests)"""
        grad = cv2.Sobel(mask, cv2.CV_16S, 0, 1, ksize=3, scale=1, delta=0,
                         borderType=cv2.BORDER_DEFAULT)
        th1 = cv2.convertScaleAbs(grad) > 127

        y_end = int(y_start) - min_gap
        if y_end <= 0:
            return no_lane_value

        a = []
        for col in range(max(0, int(x_from)), min(mask.shape[1], int(x_to))):
            hits = np.flatnonzero(th1[:y_end, col])
            if hits.size:
                a.append(hits[-1])      # groesstes y oberhalb = naechste Kante ueber der Ecke
        if len(a) > 10:
            return int(np.median(a))
        return no_lane_value

    def get_x_beside(self, mask, y, x_start, direction, min_gap=5):
        """get_x_for_driving, aber nur auf einer Seite von x_start: direction=-1 sucht
        links davon die naechste Kante, +1 rechts davon, None ohne Kante. Noetig bei der
        kombinierten Maske, sonst findet die Suche den Streifen der Ecke selbst.
        (aus detect_lane.py, Kalibrierung-Tests; noch ungenutzt)"""
        x_start = int(x_start)
        if direction < 0:
            x_end = x_start - min_gap
            if x_end <= 0:
                return None
            return self.get_x_for_driving(mask[:, :x_end], int(y), left_line=True)
        x_begin = x_start + min_gap
        if x_begin >= mask.shape[1]:
            return None
        x = self.get_x_for_driving(mask[:, x_begin:], int(y), left_line=False)
        return None if x is None else x + x_begin

    def _res(self):
        return self._bev['res'] if self._bev is not None else 0.00125

    def _remember_intersection(self, areas):
        """Flaechen (BEV-Pixel) als Bodenpunkte mit der aktuellen Odometrie merken."""
        if self._bev is None:
            return
        ground = {}
        for name, poly in areas.items():
            X, Y = bev_pixels_to_ground(self._bev, poly[:, 0], poly[:, 1])
            ground[name] = np.column_stack([X, Y])
        self._held = {'ground': ground, 'pose': self._odom_pose, 'time': time.monotonic()}

    def _recall_intersection(self):
        """Gemerkte Flaechen in BEV-Pixeln, um die seitdem gefahrene Strecke verschoben;
        None ohne Merker oder nach INTERSECTION_HOLD_S."""
        h, b = self._held, self._bev
        if h is None or time.monotonic() - h['time'] > INTERSECTION_HOLD_S:
            return None
        areas = {}
        for name, pts in h['ground'].items():
            if h['pose'] is not None and self._odom_pose is not None:
                pts = inverse_transform_points(self._odom_pose, transform_points(h['pose'], pts))
            u = (b['y_half'] - pts[:, 1]) / b['res'] - 0.5
            v = (b['x_max'] - pts[:, 0]) / b['res'] - 0.5
            areas[name] = np.round(np.column_stack([u, v])).astype(np.int32)
        return areas

    def _report_intersection(self, stop_line, corners):
        """Entprellt melden: erst nach INTERSECTION_CONFIRM Bildern, einmal je Kreuzung."""
        self._intersection_hits.append(stop_line is not None)
        need = INTERSECTION_CONFIRM[0]
        if sum(self._intersection_hits) >= need and stop_line is not None:
            if not self._intersection_seen:
                # Abstand der Haltelinie ab Radachse: X = x_max - (v + 0.5) * res
                v = (stop_line[0][1] + stop_line[1][1]) / 2.0
                x_max = self._bev['x_max'] if self._bev is not None else 0.60
                dist = x_max - (v + 0.5) * self._res()
                self.get_logger().info(f'Kreuzung erkannt, Haltelinie {dist * 100:.0f} cm voraus.')
            self._intersection_seen = True
        elif stop_line is None and self._intersection_seen and (
                self._held is None or time.monotonic() - self._held['time'] > INTERSECTION_HOLD_S):
            self.get_logger().info('Kreuzung nicht mehr im Bild.')
            self._intersection_seen = False
            self._held = None
        elif len(corners) == 1 and stop_line is None:
            side = next(iter(corners))
            self.get_logger().info(f'Ecke nur {side}, Gegenlinie nicht 30 cm gerade weiter '
                                   '(Kurve oder noch zu weit) -- keine Kreuzung.',
                                   throttle_duration_sec=2.0)

    def self_mask(self, mask):
        """Eigenen Bot (Magnet) in der Maske schwarz setzen, in place: Zeilen mit
        X < SELF_MASK_X_CM, Spalten mit |Y| < SELF_MASK_Y_CM. Ohne bev.yaml unveraendert."""
        b = self._bev
        if b is None or mask.shape[:2] != (b['height'], b['width']):
            return
        if SELF_MASK_X_CM > 0 and SELF_MASK_Y_CM > 0:
            v0 = int((b['x_max'] - SELF_MASK_X_CM / 100.0) / b['res'])
            u0 = int((b['y_half'] - SELF_MASK_Y_CM / 100.0) / b['res'])
            u1 = int(np.ceil((b['y_half'] + SELF_MASK_Y_CM / 100.0) / b['res']))
            mask[max(0, v0):, max(0, u0):min(mask.shape[1], u1)] = 0

    def shape_filter(self, mask):
        """Nur Linienfoermiges behalten: zu dicke Flaechen und kleine Reste entfernen (der
        eigene Bot ist schon in find_lane per self_mask raus). Ohne bev.yaml unveraendert."""
        b = self._bev
        if b is None or mask.shape[:2] != (b['height'], b['width']):
            return mask
        px_per_cm = 0.01 / b['res']
        mask = mask.copy()

        # 2. Dicke Flaechen: Mittelpunkte, um die ein Kreis mit Durchmesser LINE_MAX_WIDTH_CM
        #    ganz in die Maske passt (Abstand zum Rand > r), und alles bis 1,5 r darum
        #    herum (der Kreis selbst + ausgefranster Rand) aus der Maske nehmen.
        #    Fuer die Dickenpruefung erst Luecken bis ~2 cm schliessen (helle Flaechen sind
        #    lueckenhaft), entfernt wird aus der Originalmaske.
        r = LINE_MAX_WIDTH_CM * px_per_cm / 2.0
        g = max(3, int(round(2.0 * px_per_cm)) | 1)
        closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                  cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (g, g)))
        core = cv2.distanceTransform(closed, cv2.DIST_L2, 5) > r
        if core.any():
            to_core = cv2.distanceTransform((~core).astype(np.uint8), cv2.DIST_L2, 5)
            mask[to_core <= 1.5 * r] = 0

        # 3. Kleine Reste
        min_px = LINE_MIN_AREA_CM2 * px_per_cm * px_per_cm
        if min_px > 0:
            num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            small = np.where(stats[1:, cv2.CC_STAT_AREA] < min_px)[0] + 1
            if small.size:
                mask[np.isin(labels, small)] = 0
        return mask

    def color_masks(self, img):
        """Gelb- und Weiss-Maske aus dem BGR-Bild: CLAHE auf L, dann Schwellen in LAB."""
        lab = self.apply_clahe(img)
        mask_yellow = cv2.inRange(lab,
            (self.lab_l_yellow_l, self.lab_a_yellow_l, self.lab_b_yellow_l),
            (self.lab_l_yellow_h, self.lab_a_yellow_h, self.lab_b_yellow_h))
        mask_white = cv2.inRange(lab,
            (self.lab_l_white_l, self.lab_a_white_l, self.lab_b_white_l),
            (self.lab_l_white_h, self.lab_a_white_h, self.lab_b_white_h))
        mode = str(self.color_side).strip().lower()
        if mode != self._side_mode:
            if mode not in COLOR_SIDES:
                self.get_logger().error(f'color_side "{self.color_side}" unbekannt '
                                        f'({", ".join(COLOR_SIDES)}) -- nehme normal.')
            elif mode == 'auto':
                self.get_logger().info('color_side auto: suche, auf welcher Seite Gelb und '
                                       'Weiss liegen ...')
            self._side_mode, self._side_auto, self._side_votes = mode, None, []
            self._side_prev = None
        if mode == 'getauscht':
            swap = True
        elif mode == 'auto':
            swap = self._auto_color_side(mask_yellow, mask_white)
        else:
            swap = False
        if swap:
            return mask_white, mask_yellow
        return mask_yellow, mask_white

    def _color_side_vote(self, mask_yellow, mask_white):
        """Ein Bild: True = Weiss links von Gelb (getauscht), False = normal, None = unklar."""
        h, w = mask_yellow.shape[:2]
        res = self._res()
        half = int(AUTO_HALF_M / res)
        c0, c1 = max(0, w // 2 - half), min(w, w // 2 + half)
        sep = AUTO_SEP_M / res
        spread = AUTO_MAX_SPREAD_M / res
        normal = swapped = 0
        for r in range(int(h * (1.0 - AUTO_BAND)), h, AUTO_ROW_STEP):
            ys = np.flatnonzero(mask_yellow[r, c0:c1])
            ws = np.flatnonzero(mask_white[r, c0:c1])
            if ys.size == 0 or ws.size == 0 or np.ptp(ys) > spread or np.ptp(ws) > spread:
                continue
            d = float(np.median(ws)) - float(np.median(ys))
            if d > sep:
                normal += 1
            elif d < -sep:
                swapped += 1
        rows = normal + swapped
        if rows < AUTO_MIN_ROWS:
            return None
        if normal >= AUTO_ROW_SHARE * rows:
            return False
        if swapped >= AUTO_ROW_SHARE * rows:
            return True
        return None

    def _recheck_color_side(self, old_mode):
        """color_side_recheck: nach old_mode (Tunnel, Kreuzung, ...) neu suchen, alte Seite gilt weiter."""
        if not self.color_side_recheck or self._side_mode != 'auto' or self._side_auto is None:
            return
        self._side_prev, self._side_auto, self._side_votes = self._side_auto, None, []
        self.get_logger().info(f'color_side: nach {old_mode.name} neu suchen (bis dahin '
                               + ('getauscht' if self._side_prev else 'normal') + ').')

    def _auto_color_side(self, mask_yellow, mask_white):
        """color_side auto: bis zur Entscheidung Stimmen sammeln (solange normal), dann fest."""
        if self._side_auto is not None:
            return self._side_auto
        vote = None if self._intersection_seen else self._color_side_vote(mask_yellow, mask_white)
        if vote is not None:
            self._side_votes.append(vote)
        fallback = bool(self._side_prev)    # bis zur Entscheidung: alte Seite bzw. normal
        if len(self._side_votes) < AUTO_VOTES:
            self.get_logger().info(
                f'color_side auto: noch {AUTO_VOTES - len(self._side_votes)} klare Bilder noetig '
                '(beide Spurlinien vor dem Bot ins Bild).', throttle_duration_sec=2.0)
            return fallback
        swapped = sum(self._side_votes) / len(self._side_votes)
        if swapped >= AUTO_VOTE_SHARE or 1.0 - swapped >= AUTO_VOTE_SHARE:
            self._side_auto = swapped >= AUTO_VOTE_SHARE
            rechecked = self._side_prev is not None
            if self._side_prev is not None and self._side_auto == self._side_prev:
                self.get_logger().info('color_side auto: Seite bestaetigt ('
                                       + ('getauscht' if self._side_auto else 'normal') + ').')
                self._side_prev = None
                return self._side_auto
            self._side_prev = None
            self._anchor = {name: None for name in self._anchor}
            self.get_logger().warn(
                'color_side auto: ' + ('Weiss LINKS, Gelb RECHTS -- Masken getauscht'
                                       if self._side_auto else 'Gelb LINKS, Weiss RECHTS -- normal')
                + f' ({len(self._side_votes)} Bilder, {max(swapped, 1 - swapped) * 100:.0f} % einig).'
                + (' Seite GEWECHSELT nach Tunnel/Kreuzung/Ausweichen.' if rechecked
                   else ' Neu gesucht nach Tunnel/Kreuzung/Ausweichen.' if self.color_side_recheck
                   else ' Bleibt so bis zum Neustart.'))
            return self._side_auto
        # Uneinig: die aeltesten Stimmen verwerfen, weiter sammeln.
        self._side_votes = self._side_votes[-AUTO_VOTES // 2:]
        return fallback

    def detectBarrier(self):
        if self.raw_image is None or self.control_mode in (None, ControlType.Tunnel):
            return None

        try:
            # Auch hier: das Rohbild kann vor dem ersten BEV-Bild (find_lane) kommen.
            read_params(self, self._param_names)
            cv_image = self.bridge.imgmsg_to_cv2(self.raw_image, desired_encoding='bgr8')
            img_height, img_width = cv_image.shape[:2]
            x0, y0 = int(img_width * 0.33), int(img_height * 0.10)
            x1, y1 = int(img_width * 0.66), int(img_height * 0.40)
            roi = cv_image[y0:y1, x0:x1]
            mask_blue = self.blue_mask(roi)
            num_blue_pixel = cv2.countNonZero(mask_blue)
            # Stoppschild: rote Pixel im eigenen Bereich (oben rechts).
            red_roi = (int(img_width * self.stop_roi_x0), int(img_height * self.stop_roi_y0),
                       int(img_width * self.stop_roi_x1), int(img_height * self.stop_roi_y1))
            rx0, ry0, rx1, ry1 = red_roi
            mask_red = self.red_mask(cv_image[ry0:ry1, rx0:rx1]) \
                if rx1 > rx0 and ry1 > ry0 else np.zeros((0, 0), np.uint8)
            num_red_pixel = cv2.countNonZero(mask_red) if mask_red.size else 0
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'Schranken-/Stoppschilderkennung fehlgeschlagen: {e}',
                                    throttle_duration_sec=5.0)
            return None

        barrier = num_blue_pixel >= self.thresh_blue_pixel
        self.pub_barrier.publish(Bool(data=barrier))
        self.pub_barrier_pixels.publish(Int32(data=num_blue_pixel))
        no_entry = self._no_entry_time is not None \
            and time.monotonic() - self._no_entry_time <= NO_ENTRY_HOLD_S
        stop_sign = num_red_pixel >= self.thresh_red_pixel and not no_entry
        self.pub_stop_sign.publish(Bool(data=stop_sign))
        self.pub_stop_sign_pixels.publish(Int32(data=num_red_pixel))
        self.publish_barrier_debug(cv_image, mask_blue, (x0, y0, x1, y1), num_blue_pixel,
                                   barrier, mask_red, red_roi, num_red_pixel, stop_sign,
                                   self._no_entry_score if no_entry else None)
        return barrier

    def publish_barrier_debug(self, img, mask_blue, roi, num_blue_pixel, barrier,
                              mask_red=None, red_roi=None, num_red_pixel=0, stop_sign=False,
                              no_entry_score=None):
        """Rohbild mit blauer Maske im Suchbereich (Schranke) und roter im Stoppschild-Bereich,
        Rahmen und Pixelzahlen. Nur wenn es jemand anschaut."""
        if self.pub_barrier_debug.get_subscription_count() == 0:
            return
        # Hoechstens BARRIER_DEBUG_PERIOD_S: das Rohbild kommt 30x/s, JPEG kostet Zeit.
        now = time.monotonic()
        if now - self._barrier_debug_time < BARRIER_DEBUG_PERIOD_S:
            return
        self._barrier_debug_time = now
        x0, y0, x1, y1 = roi
        view = img.copy()
        area = view[y0:y1, x0:x1]
        hit = mask_blue > 0
        area[hit] = (0.4 * area[hit] + 0.6 * np.array([255, 0, 0])).astype(np.uint8)
        # Suchbereich blau umrandet, Text rot sobald die Schwelle erreicht ist.
        cv2.rectangle(view, (x0, y0), (x1 - 1, y1 - 1), (255, 0, 0), 2)
        color = (0, 0, 255) if barrier else (0, 255, 0)
        text = f'Blau: {num_blue_pixel} / {self.thresh_blue_pixel}'
        if barrier:
            text += '  SCHRANKE'
        cv2.putText(view, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
        if red_roi is not None and mask_red is not None and mask_red.size:
            rx0, ry0, rx1, ry1 = red_roi
            area = view[ry0:ry1, rx0:rx1]
            hit = mask_red > 0
            area[hit] = (0.4 * area[hit] + 0.6 * np.array([0, 0, 255])).astype(np.uint8)
            cv2.rectangle(view, (rx0, ry0), (rx1 - 1, ry1 - 1), (0, 0, 255), 2)
            color = (0, 0, 255) if stop_sign else (0, 255, 0)
            text = f'Rot: {num_red_pixel} / {self.thresh_red_pixel}'
            if stop_sign:
                text += '  STOPPSCHILD'
            elif no_entry_score is not None:
                text += f'  no_entry {no_entry_score:.2f}: kein Halt'
            cv2.putText(view, text, (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2,
                        cv2.LINE_AA)

        success, encoded = cv2.imencode('.jpg', view)
        if not success:
            return
        msg = CompressedImage()
        msg.header = self.raw_image.header
        msg.format = 'jpeg'
        msg.data = array.array('B', encoded.tobytes())
        self.pub_barrier_debug.publish(msg)

    def blue_mask(self, img):
        """Blau-Maske aus dem BGR-Bild (Schranke): CLAHE auf L, dann Schwellen in LAB."""
        lab = self.apply_clahe(img)
        return cv2.inRange(lab,
            (self.lab_l_blue_l, self.lab_a_blue_l, self.lab_b_blue_l),
            (self.lab_l_blue_h, self.lab_a_blue_h, self.lab_b_blue_h))

    def red_mask(self, img):
        """Rot-Maske aus dem BGR-Bild (Stoppschild): CLAHE auf L, dann Schwellen in LAB."""
        lab = self.apply_clahe(img)
        return cv2.inRange(lab,
            (self.lab_l_red_l, self.lab_a_red_l, self.lab_b_red_l),
            (self.lab_l_red_h, self.lab_a_red_h, self.lab_b_red_h))

    def apply_clahe(self, img):
        """BGR -> LAB mit CLAHE auf dem L-Kanal."""
        l_channel, a_channel, b_channel = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2LAB))
        tile_size = max(1, int(self.clahe_tile_grid_size))
        clahe = cv2.createCLAHE(clipLimit=float(self.clahe_clip_limit),
                                tileGridSize=(tile_size, tile_size))
        return cv2.merge((clahe.apply(l_channel), a_channel, b_channel))

    def vision_detection(self):
        """Objekterkennung (YOLO/RF-DETR) auf dem letzten Rohbild, Boxen auf /detect/objects.
        Im Tunnel aus (Reglement), wie die Schranke. Gibt die Liste (x0, y0, x1, y1, score,
        class_id, name) zurueck, None ohne Bild/Modell oder wenn dieses Bild uebersprungen wird.

        Mit detector_rate_hz > 0 laeuft sie nur auf jedem so-vielten Bild. Fuer die Bilder
        dazwischen wird NICHTS gesendet (die alten Boxen also nicht wiederholt): die Empfaenger
        speichern ohnehin die letzte Nachricht (intersection_node bis 1 s alt, debug_view
        zeichnet sie bis 1 s Abstand zum Rohbild), und switch_control zaehlt fuer die Ziellinie
        Nachrichten in Folge -- wiederholte Boxen wuerden dort als neue Bestaetigung zaehlen.
        Header-Stempel = Bild, auf dem die Boxen gefunden wurden."""
        if self.yolo is None or self.raw_image is None \
                or self.control_mode in (None, ControlType.Tunnel):
            return None

        age = (self.get_clock().now() - Time.from_msg(self.raw_image.header.stamp)).nanoseconds / 1e9
        if age > YOLO_MAX_AGE_S:
            return None
        now = time.monotonic()
        if self.detector_rate_hz > 0.0 and self._detect_last is not None \
                and now - self._detect_last < 1.0 / self.detector_rate_hz - 0.005:
            return None
        self._detect_last = now
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.raw_image, desired_encoding='bgr8')
            t0 = time.monotonic()
            detections = self.yolo.detect(cv_image)
            self._yolo_times.append((time.monotonic() - t0) * 1000.0)
            self._timing['yolo_ms'].append(self._yolo_times[-1])
        except Exception as e:  # noqa: BLE001 -- ein kaputtes Bild darf den Node nicht stoppen
            self.get_logger().error(f'YOLO fehlgeschlagen: {e}', throttle_duration_sec=5.0)
            return None
        if len(self._yolo_times) >= 100:
            self.get_logger().info(
                f'{self.yolo.kind}: {np.mean(self._yolo_times):.1f} ms/Bild (Mittel), '
                f'max {np.max(self._yolo_times):.1f} ms')
            self._yolo_times = []
        self.publish_detections(detections)
        self.publish_signs(detections)
        no_entry = [score for _, _, _, _, score, _, name in detections if name == 'no_entry']
        if no_entry and max(no_entry) >= self.no_entry_min_conf:
            self._no_entry_time = time.monotonic()
            self._no_entry_score = float(max(no_entry))
        self._timing['yolo_age_ms'].append(self.age_ms(self.raw_image.header.stamp))
        return detections

    def publish_signs(self, detections):
        """Je Schild /detect/sign/<name>: True, wenn eine Box mit genug Konfidenz und Hoehe da ist."""
        for name, pub in self.pub_signs.items():
            min_h = self.tunnel_min_height_px if name == 'tunnel' else self.sign_min_height_px
            boxes = [(y1 - y0, score) for x0, y0, x1, y1, score, _, n in detections
                     if n == name and score >= self.sign_min_conf]
            seen = any(h >= min_h for h, _ in boxes)
            if boxes and name == 'tunnel':
                h, score = max(boxes)
                self.get_logger().info(
                    f'Tunnelschild: Box {h:.0f} px hoch (Schwelle {min_h}), Konfidenz {score:.2f}'
                    + (' -- GEMELDET' if seen else ' -- noch zu klein'), throttle_duration_sec=1.0)
            pub.publish(Bool(data=bool(seen)))

    def publish_detections(self, detections):
        """Boxen auf /detect/objects und die Start-/Ziellinie auf /detect/start_line,
        Header = der des letzten Rohbilds."""
        if self.raw_image is None:
            return
        out = Detection2DArray()
        out.header = self.raw_image.header
        for x0, y0, x1, y1, score, _, name in detections:
            det = Detection2D()
            det.header = self.raw_image.header
            det.bbox.center.position.x = (x0 + x1) / 2.0
            det.bbox.center.position.y = (y0 + y1) / 2.0
            det.bbox.size_x = x1 - x0
            det.bbox.size_y = y1 - y0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = score
            det.results.append(hyp)
            out.detections.append(det)
        self.pub_objects.publish(out)
        w, h = float(self.raw_image.width), float(self.raw_image.height)
        boxes = [v for x0, y0, x1, y1, score, _, name in detections if name == 'start_line'
                 for v in (h - y1, x0 / w, x1 / w)]
        self.pub_start_line.publish(Float32MultiArray(data=[float(v) for v in boxes]))

    # --- Zeitmessung ---

    def age_ms(self, stamp):
        """Alter (ms) eines Kamera-Stempels jetzt. Stimmt nur, wenn die Bot-Uhr synchron ist
        (scripts/tb/sync-time.sh); Raten und Luecken sind davon unabhaengig."""
        return (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e6

    def note_arrival(self, name, stamp):
        """Alter beim Eintreffen und Abstand zur letzten Ankunft (Ruckler) mitschreiben."""
        now = time.monotonic()
        last = self._last_arrival.get(name)
        if last is not None:
            self._timing[name + '_gap_ms'].append((now - last) * 1000.0)
        self._last_arrival[name] = now
        self._timing[name + '_age_ms'].append(self.age_ms(stamp))

    @staticmethod
    def timing_stats(samples, period):
        """{Messreihe: [Mittel, max]} und Raten (1/s) ueber period Sekunden."""
        stats = {key: [round(float(np.mean(v)), 1), round(float(np.max(v)), 1)]
                 for key, v in samples.items() if v}
        for name, key in (('raw', 'raw_age_ms'), ('bev', 'bev_age_ms'), ('yolo', 'yolo_ms'),
                          ('pid', 'pid_age_ms')):
            stats[name + '_hz'] = len(samples.get(key, ())) / period
        return stats

    def publish_timing(self):
        """Je Messreihe Mittel und Maximum seit dem letzten Aufruf, dazu Raten (1/s) als
        JSON auf /detect/timing. raw = /lokal/raw_image (Schranke, YOLO), bev = /lokal/bev,
        soweit die Spurerkennung es verarbeitet hat (Rest verworfen, weil sie noch rechnete),
        pid = Spurfehler an control_driving gesendet."""
        self.pub_timing.publish(String(data=json.dumps(
            self.timing_stats(self._timing, TIMING_PERIOD_S))))
        for key, values in self._timing.items():
            self._timing_log[key].extend(values)
        self._timing.clear()
        self._timing_count += 1
        if self._timing_count % TIMING_LOG_EVERY == 0:
            # Ueber alle TIMING_LOG_EVERY Sekunden, damit kein Ruckler durchrutscht.
            stats = self.timing_stats(self._timing_log, TIMING_PERIOD_S * TIMING_LOG_EVERY)
            self._timing_log.clear()

            def part(name, src, calc=None):
                age, gap = stats.get(f'{src}_age_ms'), stats.get(f'{src}_gap_ms')
                text = f'{name} {stats.get(src + "_hz", 0):.0f} Hz'
                if age:
                    text += f' Alter {age[0]:.0f}/{age[1]:.0f}'
                if gap:
                    text += f' Luecke max {gap[1]:.0f}'
                if calc and stats.get(calc):
                    text += f' Rechnen {stats[calc][0]:.0f}/{stats[calc][1]:.0f}'
                return text
            self.get_logger().info('Zeit (ms, Mittel/max): ' + ' | '.join([
                part('Kamera', 'raw'), part('Spur', 'bev', 'lane_ms'), part('PID', 'pid'),
                part('YOLO', 'yolo', 'yolo_ms')]))


    # Linien-Tracking entsprechend zur Duckiebot Challenge (DuckieRace,
    # intersection_handling) umgebaut: naechste Kante zur letzten Position, verlorene
    # Linie halten, Sprung pro Bild begrenzen. Zusaetzlich: Haltezeit, Sprung in cm.
    # Frage an FaFelF: Gab es einen Grund, warum du dies anders geloest hast?

    def anchor_column(self, name, now):
        """Letzte Spalte der Linie, None ohne Detektion in den letzten LINE_HOLD_TIME_S s."""
        anchor = self._anchor[name]
        if anchor is not None and now - anchor[1] > LINE_HOLD_TIME_S:
            self._anchor[name] = anchor = None
        return None if anchor is None else anchor[0]

    def track_line(self, name, raw, fallback, now):
        """Endgueltige Spalte der Linie aus der Detektion raw (None = keine Kante)."""
        log = self._line_log[name]
        last = self.anchor_column(name, now)
        if raw is None:
            if last is None:
                log.warn(f'keine Kanten -- Ersatzspalte {fallback}', throttle_duration_sec=2.0)
                return fallback
            log.warn(f'keine Kanten -- halte letzte Position {last}', throttle_duration_sec=2.0)
            return last
        max_jump = self.max_jump_px()
        if last is not None and max_jump is not None and abs(raw - last) > max_jump:
            clamped = last + int(np.sign(raw - last) * max_jump)
            log.warn(f'Sprung {abs(raw - last)} px -- begrenzt auf {clamped}',
                     throttle_duration_sec=2.0)
            raw = clamped
        self._anchor[name] = (raw, now)
        return raw

    def max_jump_px(self):
        """LINE_MAX_JUMP_CM in Pixeln, None ohne bev.yaml (dann unbegrenzt)."""
        if self._bev is None or (self.img_height, self.img_width) != (
                self._bev['height'], self._bev['width']):
            return None
        return max(1, int(round(LINE_MAX_JUMP_CM / 100.0 / self._bev['res'])))

    def get_x_for_driving(self, mask, distance, left_line, last_known=None):
        """Spalte der Linienkante um Zeile distance (+/-50 Zeilen, Median), None ohne
        Kante. Gesucht wird nur die Kante zur Spur hin: bei left_line (gelb) die rechte,
        sonst die linke. Je Zeile die naechste zu last_known, ohne last_known die
        aeusserste."""
        grad = cv2.Sobel(mask, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0,
                         borderType=cv2.BORDER_DEFAULT)
        # Rechte Kante = hell -> dunkel (negativ), linke = dunkel -> hell (positiv).
        # Mit beiden Kantenarten rastet das Tracking auf der falschen Kante derselben Linie ein.
        edges = (-grad if left_line else grad) > 127

        a = []
        for row in range(max(0, distance - 50), min(len(mask), distance + 50)):
            hits = np.flatnonzero(edges[row])
            if hits.size == 0:
                continue
            if last_known is not None:
                a.append(hits[np.argmin(np.abs(hits - last_known))])
            else:
                a.append(hits[-1] if left_line else hits[0])

        if len(a) > 10:
            return int(np.median(a))
        return None

    # --- Beenden ---

    def shut_down(self):
        if self.yolo is not None:
            self.yolo.close()
        self.get_logger().info('Shutting down detect_lane Node (Nav2-Zweig).')


def main(args=None):
    run_node(detect_lane, args)


if __name__ == '__main__':
    main()
