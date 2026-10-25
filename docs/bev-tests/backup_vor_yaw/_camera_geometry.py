"""Gemeinsame Kamerageometrie fuer die tb-Kalibrierwerkzeuge.

Einmal sauber, ueberall gleich -- die Vorzeichen hier wurden mehrfach verifiziert
(siehe docs/MEMORY.md, Abschnitt Kamerakalibrierung). Nicht kopieren, importieren.

Konventionen:
- Bodensystem  : X vorwaerts, Y links, Z nach oben. Ursprung waehlbar (Lot der Linse
                 auf den Boden, oder per forward/lateral verschoben auf die Radachse).
- Kamerasystem : OpenCV -- x rechts, y unten, z optische Achse.
- Brettsystem  : ChArUco flach am Boden, Z aus dem Brett heraus (= nach oben).
"""
import math

import cv2
import numpy as np
import yaml

# ChArUco-Brett wie aufgedruckt (7x5 Kacheln, 30 mm, Marker 22 mm, Dict 5x5_250).
SQUARES_X, SQUARES_Y = 7, 5
SQUARE_M, MARKER_M = 0.030, 0.022
ARUCO_DICT = cv2.aruco.DICT_5X5_250

# IMX219 + verbautes Standardobjektiv laut Datenblatt.
IMX219_HFOV_DEG = 62.2


def charuco_board():
    dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard_create(
        SQUARES_X, SQUARES_Y, SQUARE_M, MARKER_M, dictionary)
    return dictionary, board


def load_intrinsics(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], dtype=np.float64).reshape(1, -1)
    return K, D, int(d['image_width']), int(d['image_height'])


def intrinsics_model(path):
    """'equidistant' (Fisheye, Weitwinkel-Modul) oder 'plumb_bob' (normales Modell)."""
    with open(path) as f:
        return yaml.safe_load(f).get('distortion_model', 'plumb_bob')


def is_fisheye(model):
    return model in ('equidistant', 'fisheye')


def undistorted_camera_matrix(K, D, w, h, model, balance=0.0):
    """Kameramatrix des ENTZERRTEN Bildes -- genau so, wie ingress_node.undistort() es erzeugt.

    plumb_bob: ingress_node entzerrt mit newK = K, also bleibt K.
    Fisheye:   ingress_node erzeugt ein NEUES K (estimateNewCameraMatrixForUndistortRectify,
               abhaengig von balance; bei 0.0 etwa fx 294 statt 385). Wer auf /lokal/raw_image
               misst und das rohe K nimmt, rechnet mit falscher Brennweite -> Hoehe/Neigung falsch.
    balance MUSS zum Parameter von ingress_node passen (Default beider: 0.0).
    """
    if not is_fisheye(model):
        return K
    return cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, np.asarray(D, dtype=np.float64).reshape(4, 1), (w, h), np.eye(3), balance=balance)


def load_extrinsics(path):
    with open(path) as f:
        return yaml.safe_load(f)


def check_intrinsics(K, D, w, h, model='plumb_bob'):
    """Plausibilitaet der Intrinsik. Gibt (ok, zeilen) zurueck; Aufrufer druckt selbst.
    Haeufigste Fehlerquelle: zu wenig Pose-Vielfalt -> Hauptpunkt und tangentiale
    Verzeichnung werden gegeneinander verrechnet.
    """
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d = D.ravel()
    if is_fisheye(model):
        # Weitwinkel-Modul (Okt. 2026): aufgeschraubte Linse, Hauptpunkt sitzt ein paar Prozent
        # neben der Mitte -- das ist echt, kein Kalibrierfehler. Kein p1/p2 im Fisheye-Modell.
        off_x, off_y = (cx - w / 2) / w, (cy - h / 2) / h
        hfov = math.degrees(w / fx)            # equidistant: theta_rand = (w/2) / fx
        info = (f'Fisheye, HFOV {hfov:.0f}deg   Hauptpunkt {off_x * 100:+.1f}% x / '
                f'{off_y * 100:+.1f}% y von der Bildmitte')
        warn = []
        if abs(off_x) > 0.05 or abs(off_y) > 0.05:
            warn.append(f'Hauptpunkt ({cx:.0f}, {cy:.0f}) liegt mehr als 5 % neben der Bildmitte.')
        if abs(fx - fy) / fx > 0.02:
            warn.append(f'fx = {fx:.1f} vs fy = {fy:.1f} weichen > 2 % ab -- Kalibrierung pruefen.')
        if not 90 <= hfov <= 200:
            warn.append(f'HFOV {hfov:.0f}deg unplausibel fuer das Weitwinkel-Modul (~120deg).')
        return (not warn), [info] + warn
    p1, p2 = float(d[2]), float(d[3])
    off_x = (cx - w / 2) / w
    off_y = (cy - h / 2) / h
    hfov = 2 * math.degrees(math.atan((w / 2) / fx))
    vfov = 2 * math.degrees(math.atan((h / 2) / fy))

    info = (f'HFOV {hfov:.1f}deg  VFOV {vfov:.1f}deg   '
            f'Hauptpunkt {off_x * 100:+.1f}% x / {off_y * 100:+.1f}% y von der Bildmitte')

    warn = []
    if abs(off_x) > 0.04:
        warn.append(f'cx = {cx:.1f} liegt {cx - w / 2:+.0f} px ({off_x * 100:+.1f} %) neben '
                    f'der Bildmitte {w / 2:.0f}. Bei fixem Modul < 2 % erwartet. Schiebt '
                    f'"geradeaus" seitlich -> schiefe/unsymmetrische Bodenprojektion.')
    if abs(off_y) > 0.06:
        warn.append(f'cy = {cy:.1f} liegt {cy - h / 2:+.0f} px neben der Bildmitte.')
    if abs(fx - fy) / fx > 0.05:
        warn.append(f'fx = {fx:.1f} vs fy = {fy:.1f} weichen {abs(fx - fy) / fx * 100:.1f} % '
                    f'ab (> 5 %). IMX219 hat quadratische Pixel -> Kalibrierung verdaechtig.')
    if abs(p1) > 0.01 or abs(p2) > 0.01:
        warn.append(f'tangentiale Verzeichnung p1 = {p1:.4f}, p2 = {p2:.4f} gross '
                    f'(typisch < 0.002). Zeichen fuer zu wenige/aehnliche Brettlagen.')
    if not (IMX219_HFOV_DEG - 4 <= hfov <= IMX219_HFOV_DEG + 4):
        warn.append(f'HFOV aus fx = {hfov:.1f}deg (Datenblatt ~{IMX219_HFOV_DEG}deg). '
                    f'fx gegen (Breite/2)/tan(HFOV/2) pruefen.')
    return (not warn), [info] + warn


def _rot_ground_to_cam(pitch_deg, roll_deg):
    """Rotationsmatrix Bodensystem -> Kamerasystem fuer die gegebene Kameralage."""
    th, ph = math.radians(pitch_deg), math.radians(roll_deg)
    # Grunddrehung ohne Neigung: robot-x -> cam-z, robot-y -> -cam-x, robot-z -> -cam-y
    R0 = np.array([[0., -1., 0.],
                   [0., 0., -1.],
                   [1., 0., 0.]])
    Rx = np.array([[1., 0., 0.],
                   [0., math.cos(th), -math.sin(th)],
                   [0., math.sin(th), math.cos(th)]])
    Rz = np.array([[math.cos(ph), -math.sin(ph), 0.],
                   [math.sin(ph), math.cos(ph), 0.],
                   [0., 0., 1.]])
    return Rz @ Rx @ R0


def ground_homography(K, height, pitch_deg, roll_deg, dx=0.0, dy=0.0):
    """Homographie Bodenebene (X vorwaerts, Y links, Z=0) -> Bildpixel.

    dx/dy verschieben den Bodenursprung (z.B. auf die Radachse: dx = Linse vor der
    Achse, dy = seitlicher Linsenversatz, positiv links).
    """
    R_gc = _rot_ground_to_cam(pitch_deg, roll_deg)
    t = -R_gc @ np.array([dx, dy, height])
    return K @ np.column_stack([R_gc[:, 0], R_gc[:, 1], t])


def normalize_ground_homography(H, ref_xy):
    """Vorzeichen/Massstab einer Boden->Bild-Homographie so drehen, dass die dritte
    Komponente (die Tiefe) fuer reale Bodenpunkte POSITIV ist.

    Noetig, weil cv2.findHomography die Matrix nur bis auf einen Faktor liefert --
    inklusive Vorzeichen. Beim aus Hoehe/Neigung/Roll gerechneten Modell ist w die
    echte Tiefe und damit positiv; bei einem gefitteten Ergebnis ist das Zufall.
    Ohne diese Normierung verwirft jeder "liegt vor der Kamera"-Test (w > 0) saemtliche
    Punkte und es wird gar nichts gezeichnet.
    """
    H = np.asarray(H, dtype=np.float64).reshape(3, 3)
    w = H[2, 0] * ref_xy[0] + H[2, 1] * ref_xy[1] + H[2, 2]
    if abs(w) < 1e-12:
        return H
    return H / w          # w am Referenzpunkt wird damit exakt +1


def project_ground(H_g2i, pts_xy):
    """(N,2)-Bodenpunkte -> (N,2)-Pixel + Maske, welche Punkte VOR der Kamera liegen."""
    pts = np.asarray(pts_xy, dtype=np.float64).reshape(-1, 2)
    hom = np.column_stack([pts, np.ones(len(pts))])
    proj = hom @ H_g2i.T
    in_front = proj[:, 2] > 1e-6
    with np.errstate(divide='ignore', invalid='ignore'):
        uv = proj[:, :2] / proj[:, 2:3]
    return uv, in_front


def pose_from_board(R, tvec):
    """solvePnP-Ergebnis (Brett flach am Boden, board->cam) -> Kameralage + Brettlage.

    Rueckgabe-Dict:
      height_m          Kamerahoehe ueber dem Brett/Boden
      pitch_deg         Blick nach unten gegen die Waagerechte
      roll_deg          Verkippung um die optische Achse
      board_forward_m   Brettursprung vor der Linse (entlang Blickrichtung am Boden)
      board_lateral_m   Brettursprung seitlich (positiv links)
      board_yaw_deg     Drehung der Brett-X-Achse gegen die Blickrichtung
    """
    tvec = np.asarray(tvec, dtype=np.float64).reshape(3)
    C = (-R.T @ tvec)                       # Kameraposition im Brettsystem
    axis = R.T @ np.array([0.0, 0.0, 1.0])  # optische Achse im Brettsystem
    right = R.T @ np.array([1.0, 0.0, 0.0])
    s = float(np.sign(C[2])) or 1.0

    height = abs(C[2])
    pitch = math.degrees(math.asin(np.clip(-axis[2] * s, -1.0, 1.0)))
    roll = math.degrees(math.asin(np.clip(right[2] * s, -1.0, 1.0)))

    # Blickrichtung horizontal am Boden (im Brettsystem), normiert.
    fwd = np.array([axis[0], axis[1]])
    n = np.linalg.norm(fwd)
    if n < 1e-9:
        fwd = np.array([1.0, 0.0])
    else:
        fwd = fwd / n
    left = np.array([-fwd[1], fwd[0]])      # +90deg = links

    # Brettursprung (0,0) relativ zum Bodenpunkt unter der Kamera (C[0], C[1]).
    rel = np.array([-C[0], -C[1]])
    board_forward = float(rel @ fwd)
    board_lateral = float(rel @ left)
    board_yaw = math.degrees(math.atan2(fwd[1], fwd[0]))

    return dict(height_m=height, pitch_deg=pitch, roll_deg=roll,
                board_forward_m=board_forward, board_lateral_m=board_lateral,
                board_yaw_deg=board_yaw)


def detect_board_pose(gray, K, D, dictionary, board, min_markers=6, min_corners=8):
    """ChArUco im Graubild finden und Pose schaetzen. Rueckgabe:
        (ok, rvec, tvec, n_corners, ch_corners, ch_ids)
    D bei bereits entzerrtem Bild als Null uebergeben.
    """
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None or len(ids) < min_markers:
        return False, None, None, 0, None, None
    n, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if n is None or n < min_corners:
        return False, None, None, int(n or 0), None, None
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        ch_corners, ch_ids, board, K, D, None, None)
    return bool(ok), rvec, tvec, int(n), ch_corners, ch_ids


def reprojection_rms(board, ch_corners, ch_ids, rvec, tvec, K, D):
    """RMS-Reprojektionsfehler der ChArUco-Ecken in Pixeln."""
    obj = board.chessboardCorners[ch_ids.ravel()]
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    proj = proj.reshape(-1, 2)
    det = ch_corners.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((proj - det) ** 2, axis=1))))
