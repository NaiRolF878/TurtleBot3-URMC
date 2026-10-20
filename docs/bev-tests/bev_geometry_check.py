#!/usr/bin/env python3
"""Prueft, dass Follow_Lane/src/bev_geometry.py dieselbe Bodengeometrie rechnet wie
scripts/tb/_camera_geometry.py (Kalibrierwerkzeuge). Aufruf ohne Argumente."""
import os, sys
import numpy as np
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'scripts', 'tb'))
sys.path.insert(0, os.path.join(REPO, 'workspace', 'src', 'Follow_Lane', 'Follow_Lane', 'src'))
import _camera_geometry as cg
import bev_geometry as bg
K = np.array([[294.1, 0, 373.6], [0, 293.8, 278.1], [0, 0, 1]])
worst = 0.0
for args in [(0.1013, 37.97, -1.03, 0.0579, 0.0, 1.4), (0.12, 21.0, 2.4, 0.0, 0.01, -3.0), (0.1, 38, 0, 0, 0, 0)]:
    worst = max(worst, np.abs(cg.ground_homography(K, *args) - bg.ground_homography(K, *args)).max())
print('max. Abweichung:', worst, '-> OK' if worst < 1e-9 else '-> UNTERSCHIED!')
sys.exit(0 if worst < 1e-9 else 1)
