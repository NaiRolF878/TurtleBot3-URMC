"""Metrische Vogelperspektive (BEV) direkt aus dem ROHEN Kamerabild.

Entzerrung und Bodenprojektion stecken in EINER Remap-Tabelle -- nur eine Interpolation
(schaerfer als erst entzerren, dann warpen; gemessen in docs/bev-tests/README.md).

Die Bodengeometrie ist dieselbe Rechnung wie scripts/tb/_camera_geometry.ground_homography
(dort liegen die Kalibrierwerkzeuge). Aendert sich dort etwas, hier nachziehen -- der Test
docs/bev-tests/bev_geometry_check.py vergleicht beide.
"""
import math
import os

import cv2
import numpy as np
import yaml

BEV_FILE = 'bev.yaml'


def load_bev_spec(param_dir):
    """Bereich/Massstab aus Parameter/bev.yaml. Gibt dict mit x_min, x_max, y_half, res,
    width, height zurueck."""
    with open(os.path.join(param_dir, BEV_FILE)) as f:
        d = yaml.safe_load(f)
    spec = dict(x_min=float(d['x_min_m']), x_max=float(d['x_max_m']),
                y_half=float(d['y_half_m']), res=float(d['res_m']))
    spec['width'] = int(round(2 * spec['y_half'] / spec['res']))
    spec['height'] = int(round((spec['x_max'] - spec['x_min']) / spec['res']))
    return spec


def rot_ground_to_cam(pitch_deg, roll_deg, yaw_deg=0.0):
    """Bodensystem (X vor, Y links, Z hoch) -> Kamerasystem. yaw positiv = Kamera schaut links."""
    th, ph, ps = math.radians(pitch_deg), math.radians(roll_deg), math.radians(yaw_deg)
    Ryaw = np.array([[math.cos(ps), math.sin(ps), 0.],
                     [-math.sin(ps), math.cos(ps), 0.],
                     [0., 0., 1.]])
    R0 = np.array([[0., -1., 0.],
                   [0., 0., -1.],
                   [1., 0., 0.]])
    Rx = np.array([[1., 0., 0.],
                   [0., math.cos(th), -math.sin(th)],
                   [0., math.sin(th), math.cos(th)]])
    Rz = np.array([[math.cos(ph), -math.sin(ph), 0.],
                   [math.sin(ph), math.cos(ph), 0.],
                   [0., 0., 1.]])
    return Rz @ Rx @ R0 @ Ryaw


def ground_homography(K, height, pitch_deg, roll_deg, dx=0.0, dy=0.0, yaw_deg=0.0):
    """Boden (X, Y, 1) -> Pixel im ENTZERRTEN Bild mit Kameramatrix K."""
    R = rot_ground_to_cam(pitch_deg, roll_deg, yaw_deg)
    t = -R @ np.array([dx, dy, height])
    return K @ np.column_stack([R[:, 0], R[:, 1], t])


def build_bev_maps(spec, ext, K_raw, D, fisheye, K_und):
    """Remap-Tabellen BEV-Pixel -> Rohbild-Pixel.

    spec  : aus load_bev_spec
    ext   : Extrinsik-YAML (camera_height_m, camera_pitch_deg, camera_roll_deg,
            camera_yaw_deg, forward_offset_m, lateral_offset_m)
    K_raw, D, fisheye : Intrinsik, bereits auf die Bildgroesse skaliert
    K_und : Kameramatrix des entzerrten Bilds, fuer das die Extrinsik gilt (gleiche Skalierung)
    """
    w, h, res = spec['width'], spec['height'], spec['res']
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    X = spec['x_max'] - v * res
    Y = spec['y_half'] - u * res
    H = ground_homography(K_und, ext['camera_height_m'], ext['camera_pitch_deg'],
                          ext['camera_roll_deg'], ext.get('forward_offset_m', 0.0),
                          ext.get('lateral_offset_m', 0.0), ext.get('camera_yaw_deg', 0.0))
    p = H @ np.stack([X.ravel(), Y.ravel(), np.ones(X.size)])
    behind = p[2] <= 1e-9
    p[2][behind] = 1.0
    # entzerrter Pixel -> normierte Kamerakoordinate
    xn = (p[0] / p[2] - K_und[0, 2]) / K_und[0, 0]
    yn = (p[1] / p[2] - K_und[1, 2]) / K_und[1, 1]
    pts = np.stack([xn, yn], axis=1).reshape(-1, 1, 2).astype(np.float64)
    if fisheye:
        d = cv2.fisheye.distortPoints(pts, K_raw, np.asarray(D, np.float64).reshape(4, 1))
    else:
        obj = np.concatenate([pts.reshape(-1, 2), np.ones((pts.shape[0], 1))], axis=1)
        d, _ = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), K_raw, np.asarray(D, np.float64))
    d = d.reshape(-1, 2)
    d[behind] = -1.0                                  # ausserhalb -> schwarz
    mx = d[:, 0].reshape(h, w).astype(np.float32)
    my = d[:, 1].reshape(h, w).astype(np.float32)
    return cv2.convertMaps(mx, my, cv2.CV_16SC2)
