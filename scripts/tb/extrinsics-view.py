#!/usr/bin/env python3
"""Sichtpruefung der extrinsischen Kalibrierung: metrisches Bodengitter im Kamerabild.

Live am Bot (ingress_node muss laufen):
    python3 scripts/tb/extrinsics-view.py
    ros2 run rqt_image_view rqt_image_view /lokal/extrinsics_view

Offline auf einem gespeicherten Bild (kein ROS noetig):
    python3 scripts/tb/extrinsics-view.py --image logs/frame.png --save logs/check.png
    python3 scripts/tb/extrinsics-view.py --image logs/frame.png --cx 400   # cx durchspielen

Was das Bild zeigt:
  * Bodengitter alle 10 cm aus der gespeicherten Kameralage. Beschriftung in cm,
    gemessen ab Radachse (oder ab Linse, s. --origin-lens).
  * Gruene Mittellinie = geradeaus (Y = 0). WICHTIG: die landet geometrisch immer
    bei Bildspalte u = cx. Stimmt cx nicht, zeigt das Gitter dauerhaft schief --
    das ist kein Extrinsik-, sondern ein Intrinsikfehler.
  * Gestrichelte Horizontlinie. Kippt sie, steckt Roll drin.
  * Liegt das ChArUco-Brett im Bild, wird seine Lage LIVE gemessen und gegen die
    gespeicherten Werte gestellt -- plus Reprojektionsfehler in Pixeln.
  * Rechts daneben die Vogelperspektive mit demselben Gitter (aus mit --no-bev bzw.
    Parameter show_bev). Die Brettecken (magenta) werden darin auf den Boden gelegt:
    stimmt die Extrinsik, bilden sie ein gleichmaessiges, achsparalleles 3-cm-Raster.

So pruefst du, ob die Kalibrierung stimmt:
  1. Zollstock flach auf den Boden, 0 an der Radachse, entlang der Fahrtrichtung.
     Die Gitterlinie "30" muss auf der 30-cm-Marke liegen. Genauso quer.
  2. Brett auf eine abgemessene Stelle legen -- die Anzeige "Brett: .. cm voraus /
     .. cm seitlich" muss zum Zollstock passen.
  3. Live-Werte (Hoehe/Neigung/Roll) muessen zu den gespeicherten passen.

Live sind alle Werte ROS-Parameter und zur Laufzeit aenderbar:
    ros2 param set /extrinsics_view pitch_deg 21.5
    ros2 run rqt_reconfigure rqt_reconfigure
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _camera_geometry as cg  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PARAM_DIR = os.path.join(REPO, 'workspace/src/Follow_Lane/Follow_Lane/Parameter/camera_info')
INTRINSICS = os.path.join(PARAM_DIR, 'imx219_800x600.yaml')
EXTRINSICS = os.path.join(PARAM_DIR, 'extrinsics_800x600.yaml')

RAW_TOPIC = '/lokal/raw_image'
OUT_TOPIC = '/lokal/extrinsics_view'

COL_MINOR = (140, 140, 0)      # BGR, gedaempftes Tuerkis
COL_MAJOR = (0, 215, 255)      # 50-cm-Linien
COL_CENTER = (0, 255, 0)       # Y = 0
COL_HORIZON = (110, 110, 110)
COL_BOARD = (255, 0, 255)


# --------------------------------------------------------------------------- #
# Zeichnen -- bewusst ohne ROS, damit dieselbe Funktion offline nutzbar ist.
# --------------------------------------------------------------------------- #

def put_lines(img, lines, org, color=(255, 255, 255), scale=0.45, pad=6):
    """Textblock mit halbtransparentem Hintergrund."""
    font, th = cv2.FONT_HERSHEY_SIMPLEX, 1
    sizes = [cv2.getTextSize(t, font, scale, th)[0] for t in lines]
    bw = max(s[0] for s in sizes) + 2 * pad
    lh = max(s[1] for s in sizes) + 8
    bh = lh * len(lines) + pad
    x, y = org
    x = max(0, min(x, img.shape[1] - bw))
    y = max(0, min(y, img.shape[0] - bh))
    roi = img[y:y + bh, x:x + bw]
    cv2.addWeighted(np.zeros_like(roi), 0.45, roi, 0.55, 0, roi)
    for i, t in enumerate(lines):
        cv2.putText(img, t, (x + pad, y + pad + lh * (i + 1) - 6),
                    font, scale, color, th, cv2.LINE_AA)


def draw_ground_polyline(img, H, pts_xy, color, thickness=1):
    """Bodenpolygonzug projizieren, nur Stuecke vor der Kamera zeichnen."""
    uv, ok = cg.project_ground(H, pts_xy)
    h, w = img.shape[:2]
    # Weit ausserhalb liegende Punkte verwerfen, sonst zieht cv2.line lange Streifen.
    on = ok & (np.abs(uv[:, 0]) < 5 * w) & (np.abs(uv[:, 1]) < 5 * h)
    for i in range(len(uv) - 1):
        if on[i] and on[i + 1]:
            cv2.line(img, (int(uv[i, 0]), int(uv[i, 1])),
                     (int(uv[i + 1, 0]), int(uv[i + 1, 1])), color, thickness, cv2.LINE_AA)
    return uv, on


def first_visible(uv, on, shape):
    h, w = shape[:2]
    for i in range(len(uv)):
        if on[i] and 0 <= uv[i, 0] < w and 0 <= uv[i, 1] < h:
            return int(uv[i, 0]), int(uv[i, 1])
    return None


def draw_horizon(img, K, pitch, roll):
    """Fluchtlinie der Bodenebene. Kippt sie, steckt Roll drin."""
    R_gc = cg._rot_ground_to_cam(pitch, roll)
    pts = []
    for d in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)):
        p = K @ (R_gc @ np.array(d))
        if abs(p[2]) < 1e-9:
            return
        pts.append((p[0] / p[2], p[1] / p[2]))
    (x1, y1), (x2, y2) = pts
    if abs(x2 - x1) < 1e-6:
        return
    m = (y2 - y1) / (x2 - x1)
    w = img.shape[1]
    ya, yb = y1 + m * (0 - x1), y1 + m * (w - x1)
    for k in range(0, 25, 2):
        t0, t1 = k / 25.0, (k + 1) / 25.0
        cv2.line(img, (int(t0 * w), int(ya + (yb - ya) * t0)),
                 (int(t1 * w), int(ya + (yb - ya) * t1)), COL_HORIZON, 1, cv2.LINE_AA)
    cv2.putText(img, 'Horizont', (8, int(np.clip(ya - 6, 12, img.shape[0] - 4))),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COL_HORIZON, 1, cv2.LINE_AA)


def measure_board(img, K, D_pnp, dictionary, board, stored):
    """ChArUco vermessen und gegen die gespeicherte Lage stellen.

    Muss auf dem UNBEMALTEN Bild laufen -- ueber das Gitter gezeichnete Linien
    zerstoeren sonst die Eckendetektion.
    Rueckgabe: (textzeilen, pose, corners, ids)
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, rvec, tvec, n, cc, ci = cg.detect_board_pose(gray, K, D_pnp, dictionary, board)
    if not ok:
        return ['Brett: nicht erkannt'], None, None, None
    R, _ = cv2.Rodrigues(rvec)
    pose = cg.pose_from_board(R, tvec)
    rms = cg.reprojection_rms(board, cc, ci, rvec, tvec, K, D_pnp)
    lat = pose['board_lateral_m'] * 100
    lines = [
        f'BRETT LIVE ({n} Ecken)',
        f'Hoehe   {pose["height_m"] * 100:6.2f} cm  (gesp. {stored["height"] * 100:.2f})',
        f'Neigung {pose["pitch_deg"]:6.2f} deg (gesp. {stored["pitch"]:.2f})',
        f'Roll    {pose["roll_deg"]:6.2f} deg (gesp. {stored["roll"]:.2f})',
        f'Brett {pose["board_forward_m"] * 100:.1f} cm voraus, '
        f'{abs(lat):.1f} cm {"links" if lat > 0 else "rechts"}',
        f'Reprojektion RMS {rms:.2f} px',
    ]
    return lines, pose, cc, ci


def scale_K(K, cal_wh, img_wh, warn=None):
    """K gilt fuer die Kalibrieraufloesung. Kommt ein anders grosses Bild an, muss K
    mitskaliert werden -- sonst steht das Bodengitter voellig falsch. Gleiche Logik wie
    in ingress_node.undistort()."""
    (cw, ch), (w, h) = cal_wh, img_wh
    if (w, h) == (cw, ch):
        return K
    K = K.copy()
    K[0] *= w / cw
    K[1] *= h / ch
    if warn is not None and abs((w / h) - (cw / ch)) > 0.01:
        warn(f'Bild {w}x{h} hat ein anderes Seitenverhaeltnis als die Kalibrierung '
             f'{cw}x{ch}. Skalieren reicht dann NICHT (Crop statt Skalierung) -- '
             f'Kamera auf {cw}x{ch} stellen oder neu kalibrieren.')
    return K


def overlay_homography(K, live, origin_lens, H_fit=None):
    """Homographie Boden -> Bild, wie sie render_overlay() und render_bev() benutzen."""
    if H_fit is not None:
        return H_fit
    dx, dy = (0.0, 0.0) if origin_lens else (live['forward'], live['lateral'])
    return cg.ground_homography(K, live['height'], live['pitch'], live['roll'], dx, dy,
                                live.get('yaw', 0.0))


def render_bev(img, H, grid, height_px, board_corners=None):
    """Vogelperspektive des Bodens von der Bildunterkante bis grid['x_max_cm'],
    +/- grid['y_half_cm'], gleicher Massstab in beide Richtungen, mit Gitter.

    board_corners: erkannte ChArUco-Ecken (Bildpixel) -- werden ueber dieselbe
    Homographie auf den Boden gelegt. Stimmt die Extrinsik, bilden sie in der BEV ein
    gleichmaessiges, achsparalleles Raster mit der echten Kachelgroesse.
    """
    h, w = img.shape[:2]
    step = grid['step_cm'] / 100.0
    x_max = grid['x_max_cm'] / 100.0
    y_half = grid['y_half_cm'] / 100.0
    H_i2g = np.linalg.inv(H)

    # Naechster sichtbarer Bodenpunkt: Bildunterkante in der Mitte
    g = H_i2g @ np.array([w / 2.0, h - 1.0, 1.0])
    x_min = max(0.0, float(g[0] / g[2])) if g[2] != 0 else 0.0
    x_min = math.floor(x_min * 20) / 20          # auf 5 cm abrunden
    if x_max <= x_min + 0.05:
        x_max = x_min + 0.5

    ppm = height_px / (x_max - x_min)            # Pixel pro Meter
    bw = int(round(2 * y_half * ppm))
    if bw > 2 * height_px:                       # sehr breit: Breite begrenzen
        bw = 2 * height_px
        y_half = bw / (2 * ppm)
    A = np.array([[0.0, -1.0 / ppm, x_max],      # BEV-Pixel (u,v) -> Boden (X,Y)
                  [-1.0 / ppm, 0.0, y_half],
                  [0.0, 0.0, 1.0]])
    M = np.linalg.inv(H @ A)
    M /= M[2, 2]
    bev = cv2.warpPerspective(img, M, (bw, height_px), flags=cv2.INTER_LINEAR)

    def g2b(X, Y):
        return int(round((y_half - Y) * ppm)), int(round((x_max - X) * ppm))

    # Gitter
    x = math.ceil(x_min / step) * step
    while x <= x_max + 1e-9:
        major = round(x * 100) % 50 == 0
        col = COL_MAJOR if major else COL_MINOR
        cv2.line(bev, g2b(x, y_half), g2b(x, -y_half), col, 2 if major else 1, cv2.LINE_AA)
        u, v = g2b(x, 0)
        cv2.putText(bev, f'{round(x * 100)}', (4, v - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(bev, f'{round(x * 100)}', (4, v - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    col, 1, cv2.LINE_AA)
        x = round(x + step, 4)
    n_y = int(round(y_half / step))
    for i in range(-n_y, n_y + 1):
        y = round(i * step, 4)
        center = abs(y) < 1e-9
        col = COL_CENTER if center else (COL_MAJOR if round(abs(y) * 100) % 50 == 0 else COL_MINOR)
        cv2.line(bev, g2b(x_min, y), g2b(x_max, y), col, 2 if center else 1, cv2.LINE_AA)

    # Brettecken auf den Boden legen
    if board_corners is not None and len(board_corners):
        pts = board_corners.reshape(-1, 2).astype(np.float64)
        gg = H_i2g @ np.vstack([pts.T, np.ones(len(pts))])
        for X, Y in zip(gg[0] / gg[2], gg[1] / gg[2]):
            cv2.circle(bev, g2b(X, Y), 3, COL_BOARD, -1, cv2.LINE_AA)

    put_lines(bev, [f'VOGELPERSPEKTIVE  {round(x_min * 100)}-{round(x_max * 100)} cm, '
                    f'Gitter {int(step * 100)} cm'], (8, 8))
    return bev


def render_overlay(img, K, live, stored, grid, origin_lens,
                   D_pnp=None, dictionary=None, board=None, H_fit=None, found=None):
    """Kernstueck: Bodengitter + Infoblöcke ins Bild zeichnen. Gibt ein neues Bild zurueck.

    live/stored : dicts mit height, pitch, roll, forward, lateral
    grid        : dict mit step_cm, x_max_cm, y_half_cm
    found       : optionales dict, bekommt 'board_corners' (fuer render_bev)
    """
    # Brett ZUERST auf dem unbemalten Bild suchen, danach erst das Gitter zeichnen.
    board_lines, board_corners, board_ids = None, None, None
    if board is not None:
        board_lines, _, board_corners, board_ids = measure_board(
            img, K, D_pnp, dictionary, board, stored)
    if found is not None:
        found['board_corners'] = board_corners

    img = img.copy()
    w = img.shape[1]
    step = grid['step_cm'] / 100.0
    x_max = grid['x_max_cm'] / 100.0
    y_half = grid['y_half_cm'] / 100.0

    # origin_lens: Ursprung unter der Linse. Sonst auf der Radachse -- dann sind die
    # Beschriftungen die Masse, die man vom Bot aus mit dem Zollstock nachmisst.
    # Bei H_fit: an echte Bodenpunkte gefittete Homographie -- Hoehe/Neigung/Roll spielen
    # dann keine Rolle mehr, der Ursprung steckt schon in der Messung.
    H = overlay_homography(K, live, origin_lens, H_fit)

    if H_fit is None:
        draw_horizon(img, K, live['pitch'], live['roll'])

    # --- Linien konstanter Entfernung (quer) ---
    ys = np.linspace(-y_half, y_half, 120)
    x = step
    while x <= x_max + 1e-9:
        major = round(x * 100) % 50 == 0
        col = COL_MAJOR if major else COL_MINOR
        uv, on = draw_ground_polyline(
            img, H, np.column_stack([np.full_like(ys, x), ys]), col, 2 if major else 1)
        pt = first_visible(uv, on, img.shape)
        if pt:
            cv2.putText(img, f'{round(x * 100)}', (pt[0] + 4, pt[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
        x = round(x + step, 4)

    # --- Linien konstanten Seitenversatzes (laengs) ---
    xs = np.linspace(step * 0.5, x_max, 160)
    n_y = int(round(y_half / step))
    for i in range(-n_y, n_y + 1):
        y = round(i * step, 4)
        center = abs(y) < 1e-9
        major = round(abs(y) * 100) % 50 == 0
        col = COL_CENTER if center else (COL_MAJOR if major else COL_MINOR)
        uv, on = draw_ground_polyline(
            img, H, np.column_stack([xs, np.full_like(xs, y)]), col,
            2 if (center or major) else 1)
        # xs laeuft nah -> fern, first_visible liefert also den naechsten sichtbaren
        # Punkt. Frueher stand hier uv[::-1] -- das klebte alle Labels an den Horizont.
        pt = first_visible(uv, on, img.shape)
        if pt and not center:
            cv2.putText(img, f'{round(abs(y) * 100)}{"L" if y > 0 else "R"}',
                        (pt[0] - 10, pt[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)

    if board_corners is not None:
        cv2.aruco.drawDetectedCornersCharuco(img, board_corners, board_ids, COL_BOARD)

    origin_txt = 'Linse' if origin_lens else 'Radachse'
    if H_fit is not None:
        info = [f'BODEN-FIT   Gitter {int(step * 100)} cm, Masse ab Radachse'] + \
               stored.get('fit_info', [])
    else:
        info = [
            f'MODELL   Gitter {int(step * 100)} cm, Masse ab {origin_txt}',
            f'Hoehe   {live["height"] * 100:6.2f} cm  (gespeichert {stored["height"] * 100:.2f})',
            f'Neigung {live["pitch"]:6.2f} deg (gespeichert {stored["pitch"]:.2f})',
            f'Roll    {live["roll"]:6.2f} deg (gespeichert {stored["roll"]:.2f})',
            f'Yaw     {live.get("yaw", 0.0):6.2f} deg (gespeichert {stored.get("yaw", 0.0):.2f}), gruen = Fahrtrichtung',
            f'fx {K[0, 0]:.1f}  cx {K[0, 2]:.1f} (Bildmitte {w / 2:.0f})',
            'Y=0 liegt geometrisch immer bei u = cx',
        ]
    put_lines(img, info, (8, 8))
    if board_lines:
        put_lines(img, board_lines, (img.shape[1] - 320, 8), COL_BOARD)
    return img


def stored_from_yaml(ext):
    return dict(
        height=float(ext.get('camera_height_m', 0.12)),
        pitch=float(ext.get('camera_pitch_deg', 21.0)),
        roll=float(ext.get('camera_roll_deg', 0.0)),
        forward=float(ext.get('forward_offset_m', 0.0)),
        lateral=float(ext.get('lateral_offset_m', 0.0)),
        yaw=float(ext.get('camera_yaw_deg', 0.0)),
    )


# --------------------------------------------------------------------------- #
# ROS-Node (nur im Live-Betrieb importiert)
# --------------------------------------------------------------------------- #

def run_live(K, D_pnp, ext, args, cal_w, cal_h, H_fit=None, fit_info=None):
    import rclpy
    from cv_bridge import CvBridge
    from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    def free_range(lo, hi):
        # step=0.0: rclpy verwirft sonst Werte abseits des Rasters (s. docs/MEMORY.md)
        return ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])

    def int_range(lo, hi):
        return ParameterDescriptor(
            integer_range=[IntegerRange(from_value=lo, to_value=hi, step=1)])

    stored = stored_from_yaml(ext)
    stored['fit_info'] = fit_info or []

    class ExtrinsicsView(Node):
        def __init__(self):
            super().__init__('extrinsics_view')
            self.bridge = CvBridge()
            self.dictionary, self.board = cg.charuco_board()
            self.saved = False

            self.declare_parameter('camera_height_m', stored['height'], free_range(0.02, 0.60))
            self.declare_parameter('pitch_deg', stored['pitch'], free_range(0.0, 80.0))
            self.declare_parameter('roll_deg', stored['roll'], free_range(-20.0, 20.0))
            self.declare_parameter('forward_offset_m', stored['forward'], free_range(-0.3, 0.3))
            self.declare_parameter('lateral_offset_m', stored['lateral'], free_range(-0.3, 0.3))
            self.declare_parameter('yaw_deg', stored['yaw'], free_range(-15.0, 15.0))
            # Defaults aus den CLI-Argumenten, damit live und offline identisch sind.
            self.declare_parameter('grid_step_cm', args.grid_cm, int_range(1, 50))
            self.declare_parameter('x_max_cm', args.x_max_cm, int_range(20, 400))
            self.declare_parameter('y_half_cm', args.y_half_cm, int_range(10, 200))
            self.declare_parameter('show_board', True)
            # Vogelperspektive rechts daneben (Gitter-Reichweite = x_max_cm / y_half_cm)
            self.declare_parameter('show_bev', not args.no_bev)

            self.create_subscription(Image, RAW_TOPIC, self.cb, 1)
            self.pub = self.create_publisher(Image, OUT_TOPIC, 1)
            self.get_logger().info(
                f'Bodengitter aus {os.path.basename(EXTRINSICS)} -> {OUT_TOPIC}')

            # Ohne Bilder auf RAW_TOPIC publiziert diese Node gar nichts -- dann steht
            # man vor einem leeren Topic und sucht den Fehler an der falschen Stelle.
            self.n_frames = 0
            self.img_wh = None
            self.K_img = K
            self.create_timer(5.0, self.watchdog)

        def watchdog(self):
            if self.n_frames:
                self.get_logger().info(f'{self.n_frames} Bilder verarbeitet -> {OUT_TOPIC}',
                                       once=True)
                return
            self.get_logger().warn(
                f'Noch kein Bild auf {RAW_TOPIC}. Laeuft ingress_node, und liefert die '
                'Kamera? Pruefen: ros2 topic hz /camera/image_raw/compressed')

        def cb(self, msg):
            self.n_frames += 1
            try:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'Bild nicht dekodierbar: {e}')
                return
            h, w = img.shape[:2]
            if (w, h) != self.img_wh:
                self.img_wh = (w, h)
                self.K_img = scale_K(K, (cal_w, cal_h), (w, h),
                                     warn=self.get_logger().warn)
                self.get_logger().info(
                    f'Bildgroesse {w}x{h}, Kalibrierung {cal_w}x{cal_h} -> '
                    f'cx {self.K_img[0, 2]:.1f} (Bildmitte {w / 2:.0f})')

            live = dict(
                height=self.get_parameter('camera_height_m').value,
                pitch=self.get_parameter('pitch_deg').value,
                roll=self.get_parameter('roll_deg').value,
                forward=self.get_parameter('forward_offset_m').value,
                lateral=self.get_parameter('lateral_offset_m').value,
                yaw=self.get_parameter('yaw_deg').value,
            )
            grid = dict(step_cm=self.get_parameter('grid_step_cm').value,
                        x_max_cm=self.get_parameter('x_max_cm').value,
                        y_half_cm=self.get_parameter('y_half_cm').value)
            show_board = self.get_parameter('show_board').value
            found = {}
            out = render_overlay(img, self.K_img, live, stored, grid, args.origin_lens,
                                 D_pnp, self.dictionary,
                                 self.board if show_board else None, H_fit, found)
            if self.get_parameter('show_bev').value:
                H = overlay_homography(self.K_img, live, args.origin_lens, H_fit)
                bev = render_bev(img, H, grid, out.shape[0], found.get('board_corners'))
                out = np.hstack([out, bev])
            self.pub.publish(self.bridge.cv2_to_imgmsg(out, encoding='bgr8'))
            if args.save and not self.saved:
                cv2.imwrite(args.save, out)
                self.get_logger().info(f'Geschrieben: {args.save}')
                self.saved = True

    rclpy.init()
    node = ExtrinsicsView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', metavar='PNG', default=None,
                    help='offline auf einem gespeicherten (entzerrten) Bild rechnen, ohne ROS')
    ap.add_argument('--balance', type=float, default=0.0,
                    help='nur Fisheye: muss zum Parameter "balance" von ingress_node passen (Default 0.0)')
    ap.add_argument('--distorted', action='store_true',
                    help=f'{RAW_TOPIC} bzw. --image ist NICHT entzerrt. Default: entzerrt, '
                         'PnP rechnet daher mit D = 0.')
    ap.add_argument('--origin-lens', action='store_true',
                    help='Bodenursprung unter die Linse legen statt auf die Radachse')
    ap.add_argument('--save', metavar='PNG', default=None,
                    help='gerendertes Bild als PNG ablegen')
    ap.add_argument('--ground', action='store_true',
                    help='Gitter aus der an echte Bodenpunkte gefitteten Homographie '
                         '(ground_homography_800x600.yaml) statt aus dem cx/Neigung/Roll-'
                         'Modell zeichnen. Das ist die Kalibrierung fuer echtes Messen.')
    ap.add_argument('--cx', type=float, default=None,
                    help='cx zum Durchspielen ueberschreiben (z.B. --cx 400 = Bildmitte)')
    ap.add_argument('--grid-cm', type=int, default=10, help='Gitterabstand in cm')
    ap.add_argument('--x-max-cm', type=int, default=80, help='weiteste Gitterlinie in cm')
    ap.add_argument('--y-half-cm', type=int, default=40, help='halbe Gitterbreite in cm')
    ap.add_argument('--no-bev', action='store_true',
                    help='keine Vogelperspektive rechts daneben (live auch per Parameter show_bev)')
    args = ap.parse_args()

    if not os.path.isfile(INTRINSICS):
        print(f'Intrinsik fehlt: {INTRINSICS}')
        return 1
    if not os.path.isfile(EXTRINSICS):
        print(f'Extrinsik fehlt: {EXTRINSICS} -- erst camera-extrinsics.py laufen lassen.')
        return 1

    K, D, w, h = cg.load_intrinsics(INTRINSICS)
    model = cg.intrinsics_model(INTRINSICS)
    if cg.is_fisheye(model):
        if args.distorted:
            print('--distorted geht mit dem Fisheye-Modell nicht. ingress_node mit undistort:=true.')
            return 1
        ok_f, lines_f = cg.check_intrinsics(K, D, w, h, model)
        print('Intrinsik roh: ' + lines_f[0])
        for line in lines_f[1:]:
            print('  ! ' + line)
        # Gemessen wird auf dem entzerrten Bild -- das hat beim Fisheye eine eigene Kameramatrix.
        K = cg.undistorted_camera_matrix(K, D, w, h, model, args.balance)
        D = np.zeros((1, 5))
        print(f'Fisheye: entzerrtes Bild (balance {args.balance:.2f}) fx={K[0,0]:.1f} cx={K[0,2]:.1f}')
    if args.cx is not None:
        print(f'cx von {K[0, 2]:.1f} auf {args.cx:.1f} ueberschrieben (nur zum Durchspielen).')
        K = K.copy()
        K[0, 2] = args.cx
    ok, lines = (True, ['']) if cg.is_fisheye(model) else cg.check_intrinsics(K, D, w, h)
    print(f'Intrinsik ({w}x{h}): fx={K[0,0]:.1f} fy={K[1,1]:.1f} '
          f'cx={K[0,2]:.1f} cy={K[1,2]:.1f}')
    if not cg.is_fisheye(model):
        print('  ' + lines[0])
        for line in lines[1:]:
            print('  ! ' + line)
    if not ok:
        print('  -> Das Bodengitter kann dadurch systematisch schief stehen.')

    # Entscheidend: bei bereits entzerrtem Bild darf D NICHT nochmal angewandt werden.
    D_pnp = D if args.distorted else np.zeros((1, 5))
    print(f'PnP rechnet mit '
          f'{"echtem D (Bild verzeichnet)" if args.distorted else "D = 0 (Bild entzerrt)"}\n')

    ext = cg.load_extrinsics(EXTRINSICS)

    H_fit, fit_info = None, []
    if args.ground:
        gpath = os.path.join(PARAM_DIR, 'ground_homography_800x600.yaml')
        if not os.path.isfile(gpath):
            print(f'Kein Boden-Fit vorhanden: {gpath}')
            print('Erst erzeugen mit: python3 scripts/tb/ground-calib.py --grab '
                  'und dann --layout "..."')
            return 1
        g = cg.load_extrinsics(gpath)
        H_fit = np.array(g['H_ground_to_image'], dtype=np.float64).reshape(3, 3)
        # Defensiv: aeltere Dateien wurden ohne Vorzeichennormierung geschrieben.
        H_fit = cg.normalize_ground_homography(
            H_fit, np.mean(np.array(g['markers_ground_m']), axis=0))
        cvr, cvm = g.get('crossval_rms_mm'), g.get('crossval_max_mm')
        fit_info = [
            f'{len(g["markers_ground_m"])} Marker, Fit-RMS {g["fit_rms_mm"]:.1f} mm',
            (f'Kreuzvalidierung RMS {cvr:.1f} mm, max {cvm:.1f} mm'
             if cvr is not None else 'keine Kreuzvalidierung (< 5 Marker)'),
        ]
        print('Boden-Fit geladen: ' + ' | '.join(fit_info))

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f'Bild nicht lesbar: {args.image}')
            return 1
        stored = stored_from_yaml(ext)
        stored['fit_info'] = fit_info
        # wie im Live-Betrieb: K auf die tatsaechliche Bildgroesse skalieren
        K = scale_K(K, (w, h), (img.shape[1], img.shape[0]), warn=print)
        dictionary, board = cg.charuco_board()
        grid = dict(step_cm=args.grid_cm, x_max_cm=args.x_max_cm, y_half_cm=args.y_half_cm)
        found = {}
        out = render_overlay(img, K, dict(stored), stored, grid, args.origin_lens,
                             D_pnp, dictionary, board, H_fit, found)
        if not args.no_bev:
            H = overlay_homography(K, stored, args.origin_lens, H_fit)
            out = np.hstack([out, render_bev(img, H, grid, out.shape[0],
                                             found.get('board_corners'))])
        dest = args.save or os.path.join(REPO, 'logs', 'extrinsik_check.png')
        cv2.imwrite(dest, out)
        print(f'Geschrieben: {dest}')
        return 0

    return run_live(K, D_pnp, ext, args, w, h, H_fit, fit_info)


if __name__ == '__main__':
    sys.exit(main())
