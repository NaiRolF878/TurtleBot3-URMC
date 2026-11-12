"""Bot-Umriss, Kollisionspruefung und Scan-Korrektur ohne ROS.

Ein Umriss fuer alles, was auf Kollision prueft: Waechter und Freifahren (control_driving),
Pfadpruefung (passage_geometry), Wand-Huelle (tunnel_geometry). Belegt heisst: Zelle mit
LETHAL_COST im OccupancyGrid (Costmap 254).
"""
import math

import cv2
import numpy as np

# Umriss relativ zu base_footprint, wie local_costmap.footprint in nav2_params.yaml (ROBOTIS
# 138 x 178 mm, ganz im Kreis r = 0,105 um die Radachse; Rumpf 3,7 cm vor bis 10,1 cm hinter
# der Achse, Raeder +/-8,9 cm).
FOOTPRINT = np.array([[0.037, -0.069], [0.037, 0.069], [0.033, 0.089], [-0.033, 0.089],
                      [-0.079, 0.069], [-0.092, 0.05], [-0.101, 0.028], [-0.101, -0.028],
                      [-0.092, -0.05], [-0.079, -0.069], [-0.033, -0.089], [0.033, -0.089]])
BOT_LENGTH = 0.138
BOT_WIDTH = 0.178
CIRCUMRADIUS = float(np.hypot(FOOTPRINT[:, 0], FOOTPRINT[:, 1]).max())
LETHAL_COST = 100

# Freifahren: Kandidaten (m geradeaus, rad drehen); gefahren wird der, nach dem der Umriss am
# weitesten von allem Belegten weg ist.
UNWEDGE_MOVES = (('straight', -0.05), ('straight', 0.05), ('rotate', 0.26), ('rotate', -0.26))


def transform(pose, xy):
    """Punkte (N, 2) aus dem Frame von pose = (x, y, yaw) in dessen Eltern-Frame."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    xy = np.asarray(xy, dtype=np.float64)
    return np.column_stack([pose[0] + c * xy[:, 0] - s * xy[:, 1],
                            pose[1] + s * xy[:, 0] + c * xy[:, 1]])


def outline(step):
    """Punkte auf dem Rand des Umrisses, hoechstens step (m) auseinander."""
    corners = np.vstack([FOOTPRINT, FOOTPRINT[:1]])
    return np.vstack([np.linspace(a, b, max(2, int(math.ceil(np.hypot(*(b - a)) / step))) + 1)
                      for a, b in zip(corners[:-1], corners[1:])])


def _cells(points, grid_shape, res, origin):
    """Zellen (Zeile, Spalte) der Punkte, die in der Karte liegen."""
    rc = np.floor((points - np.asarray(origin)) / res).astype(np.int64)[:, ::-1]
    h, w = grid_shape
    ok = (rc[:, 0] >= 0) & (rc[:, 0] < h) & (rc[:, 1] >= 0) & (rc[:, 1] < w)
    return rc[ok]


def lethal_cells(grid, res, origin, pose):
    """Belegte Zellen unter dem Rand des Umrisses an pose, als Menge (Zeile, Spalte)."""
    rc = _cells(transform(pose, outline(res / 2.0)), grid.shape, res, origin)
    hit = rc[grid[rc[:, 0], rc[:, 1]] == LETHAL_COST]
    return set(map(tuple, hit.tolist()))


def simulate(pose, v, w, duration, res):
    """Posen (N, 3) ab pose, wenn der Bot duration s lang mit v (m/s) und w (rad/s) faehrt.
    Zwischen zwei Posen bewegt sich kein Punkt des Umrisses mehr als eine halbe Zelle."""
    reach = (abs(v) + abs(w) * CIRCUMRADIUS) * duration
    n = max(1, int(math.ceil(reach / (res / 2.0))))
    t = np.arange(n + 1) * (duration / n)
    x0, y0, a0 = pose
    yaw = a0 + w * t
    if abs(w) < 1e-9:
        xs, ys = x0 + v * t * math.cos(a0), y0 + v * t * math.sin(a0)
    else:
        xs = x0 + v / w * (np.sin(yaw) - math.sin(a0))
        ys = y0 - v / w * (np.cos(yaw) - math.cos(a0))
    return np.column_stack([xs, ys, yaw])


def motion_collides(grid, res, origin, poses):
    """Beruehrt der Umriss auf dem Weg durch poses (N, 3) eine belegte Zelle? Zellen, die er an
    der ersten Pose schon beruehrt, zaehlen nicht: sonst kaeme ein Bot, der an etwas anliegt,
    nie wieder weg."""
    touching = lethal_cells(grid, res, origin, poses[0])
    return any(lethal_cells(grid, res, origin, p) - touching for p in poses[1:])


def clearance_map(grid, res):
    """Je Zelle der Abstand (m) zur naechsten belegten Zelle, von Zellmitte zu Zellmitte."""
    free = (grid != LETHAL_COST).astype(np.uint8)
    return cv2.distanceTransform(free, cv2.DIST_L2, 5) * res


def clearance(clear, res, origin, pose):
    """Kleinster Abstand (m) des Umrisses an pose zu allem Belegten; inf ausserhalb der Karte."""
    rc = _cells(transform(pose, outline(res / 2.0)), clear.shape, res, origin)
    return float(clear[rc[:, 0], rc[:, 1]].min()) if len(rc) else math.inf


def move_poses(pose, kind, amount, res):
    """Posen einer Freifahr-Bewegung: amount m geradeaus oder amount rad drehen."""
    if kind == 'straight':
        return simulate(pose, math.copysign(1.0, amount), 0.0, abs(amount), res)
    return simulate(pose, 0.0, math.copysign(1.0, amount), abs(amount), res)


def choose_unwedge(grid, res, origin, pose):
    """Freieste Bewegung aus UNWEDGE_MOVES als (Art, Betrag). None, wenn keine den Abstand zu
    allem Belegten vergroessert, ohne unterwegs irgendwo neu anzustossen."""
    clear = clearance_map(grid, res)
    best, best_clear = None, clearance(clear, res, origin, pose)
    for kind, amount in UNWEDGE_MOVES:
        poses = move_poses(pose, kind, amount, res)
        if motion_collides(grid, res, origin, poses):
            continue
        end = clearance(clear, res, origin, poses[-1])
        if end > best_clear:
            best, best_clear = (kind, amount), end
    return best


def correct_scan(ranges, angle_min, angle_increment, range_min, offset, scanner, rel_poses):
    """LiDAR-Entfernungen um offset (m) verlaengert und auf die Pose beim ersten Strahl bezogen.

    scanner: Lage des Scanners in base_footprint (x, y, yaw). rel_poses: (N, 3) Pose des Bots
    beim Messen jedes Strahls, relativ zur Pose beim ersten Strahl; None = Bot stand still.
    Ungueltige Strahlen (kein Echo, unter range_min) bleiben, wie sie sind. Ein Strahl, dessen
    Punkt in einen anderen Winkel wandert, landet dort; bleibt sein alter Platz leer, wird er nan
    (ungueltig), nie inf: inf raeumt in der Costmap bis zur vollen Reichweite frei.
    """
    r = np.asarray(ranges, dtype=np.float64)
    n = r.size
    out = r.copy()
    valid = np.isfinite(r) & (r > range_min)
    if not valid.any():
        return out
    idx = np.nonzero(valid)[0]
    angles = angle_min + idx * angle_increment
    dist = r[idx] + offset
    if rel_poses is None:
        out[idx] = dist
        return out
    local = np.column_stack([dist * np.cos(angles), dist * np.sin(angles)])
    base = transform(scanner, local)
    poses = np.asarray(rel_poses, dtype=np.float64)[idx]
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    moved = np.column_stack([poses[:, 0] + c * base[:, 0] - s * base[:, 1],
                             poses[:, 1] + s * base[:, 0] + c * base[:, 1]])
    sx, sy, syaw = scanner
    cs, ss = math.cos(syaw), math.sin(syaw)
    dx, dy = moved[:, 0] - sx, moved[:, 1] - sy
    px, py = cs * dx + ss * dy, -ss * dx + cs * dy
    new_r = np.hypot(px, py)
    full_turn = abs(angle_increment * n) >= 2.0 * math.pi - 1.5 * abs(angle_increment)
    new_i = np.round((np.arctan2(py, px) - angle_min) / angle_increment).astype(np.int64)
    if full_turn:
        new_i %= n
    keep = (new_i >= 0) & (new_i < n)
    out[idx] = np.nan
    for i, d in zip(new_i[keep], new_r[keep]):
        if not np.isfinite(out[i]) or out[i] <= range_min or d < out[i]:
            out[i] = d
    return out
