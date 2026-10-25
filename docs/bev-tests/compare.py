#!/usr/bin/env python3
"""Vergleich der BEV-Schaerfe aus den gespeicherten Testbildern (offline, kein Bot noetig).

Fuer jede Aufloesung zwei Wege, gleicher Bodenausschnitt und gleicher Massstab:
  zweistufig: Rohbild -> (ingress_node) entzerrt -> warpPerspective   [_entzerrt.png]
  direkt:     Rohbild -> EIN remap (Boden -> entzerrt -> Fisheye-Verzeichnung)  [_roh.png]

Aufruf: python3 docs/bev-tests/compare.py   -> docs/bev-tests/vergleich.png
"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'scripts', 'tb'))
import _camera_geometry as cg  # noqa: E402

PARAM = os.path.join(REPO, 'workspace/src/Follow_Lane/Follow_Lane/Parameter/camera_info')
X_MIN, X_MAX, Y_HALF = 0.30, 0.75, 0.20     # Bodenausschnitt [m] (Brett liegt bei ~35-65 cm)
PPCM = 10                                   # Ausgabe: Pixel pro cm
BALANCE = 0.0

K0, D0, CW, CH = cg.load_intrinsics(os.path.join(PARAM, 'imx219_800x600.yaml'))
MODEL = cg.intrinsics_model(os.path.join(PARAM, 'imx219_800x600.yaml'))
EXT = cg.load_extrinsics(os.path.join(PARAM, 'extrinsics_800x600.yaml'))


def scaled(K, w, h):
    K = K.copy()
    K[0] *= w / CW
    K[1] *= h / CH
    return K


def ground_grid():
    """Bodenkoordinaten (X,Y) fuer jeden Ausgabepixel. Oben = fern, links = links."""
    out_w, out_h = int(round(2 * Y_HALF * 100 * PPCM)), int(round((X_MAX - X_MIN) * 100 * PPCM))
    u, v = np.meshgrid(np.arange(out_w) + 0.5, np.arange(out_h) + 0.5)
    X = X_MAX - v / (100 * PPCM)
    Y = Y_HALF - u / (100 * PPCM)
    return X, Y


def H_ground(K_und):
    return cg.ground_homography(K_und, EXT['camera_height_m'], EXT['camera_pitch_deg'],
                                EXT['camera_roll_deg'], EXT.get('forward_offset_m', 0.0),
                                EXT.get('lateral_offset_m', 0.0), EXT.get('camera_yaw_deg', 0.0))


def project(H, X, Y):
    p = H @ np.stack([X.ravel(), Y.ravel(), np.ones(X.size)])
    return (p[0] / p[2]).reshape(X.shape), (p[1] / p[2]).reshape(X.shape)


def bev_two_step(und, interp):
    h, w = und.shape[:2]
    # entzerrtes K wie ingress_node: aus dem auf die Bildgroesse skalierten Roh-K
    K_und = cg.undistorted_camera_matrix(scaled(K0, w, h), D0, w, h, MODEL, BALANCE)
    X, Y = ground_grid()
    mx, my = project(H_ground(K_und), X, Y)
    return cv2.remap(und, mx.astype(np.float32), my.astype(np.float32), interp)


def bev_direct(raw, interp):
    h, w = raw.shape[:2]
    K_raw = scaled(K0, w, h)
    K_und = cg.undistorted_camera_matrix(K_raw, D0, w, h, MODEL, BALANCE)
    X, Y = ground_grid()
    ux, uy = project(H_ground(K_und), X, Y)
    # entzerrter Pixel -> normierte Koordinate -> mit Fisheye-Modell in den Rohpixel
    xn = (ux - K_und[0, 2]) / K_und[0, 0]
    yn = (uy - K_und[1, 2]) / K_und[1, 1]
    pts = np.stack([xn.ravel(), yn.ravel()], axis=1).reshape(-1, 1, 2).astype(np.float64)
    d = cv2.fisheye.distortPoints(pts, K_raw, D0.reshape(-1)[:4]).reshape(X.shape + (2,))
    return cv2.remap(raw, d[..., 0].astype(np.float32), d[..., 1].astype(np.float32), interp)


def grid(img):
    img = img.copy()
    for cm in range(int(X_MIN * 100), int(X_MAX * 100) + 1, 10):
        v = int(round((X_MAX * 100 - cm) * PPCM))
        cv2.line(img, (0, v), (img.shape[1], v), (140, 140, 0), 1)
        cv2.putText(img, str(cm), (3, v - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 140, 0), 1, cv2.LINE_AA)
    return img


def label(img, txt):
    img = img.copy()
    cv2.rectangle(img, (0, 0), (img.shape[1], 24), (0, 0, 0), -1)
    cv2.putText(img, txt, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def main():
    sets = [('0_basis_800x600_q80', '800x600 q80'),
            ('1_q90_800x600', '800x600 q90'),
            ('2_q90_1640x1232', '1640x1232 q90')]
    rows = []
    for prefix, name in sets:
        raw = cv2.imread(os.path.join(HERE, f'{prefix}_roh.png'))
        und = cv2.imread(os.path.join(HERE, f'{prefix}_entzerrt.png'))
        if raw is None or und is None:
            print(f'fehlt: {prefix}')
            continue
        tiles = [label(grid(bev_two_step(und, cv2.INTER_LINEAR)), f'{name} | zweistufig (wie jetzt)'),
                 label(grid(bev_direct(raw, cv2.INTER_LINEAR)), f'{name} | direkt, 1 Remap'),
                 label(grid(bev_direct(raw, cv2.INTER_CUBIC)), f'{name} | direkt, kubisch')]
        rows.append(np.hstack([np.hstack([t, np.full((t.shape[0], 6, 3), 255, np.uint8)]) for t in tiles]))
    out = np.vstack([np.vstack([r, np.full((6, r.shape[1], 3), 255, np.uint8)]) for r in rows])
    dest = os.path.join(HERE, 'vergleich.png')
    cv2.imwrite(dest, out)
    print(f'Geschrieben: {dest}  ({out.shape[1]}x{out.shape[0]})')


if __name__ == '__main__':
    main()
