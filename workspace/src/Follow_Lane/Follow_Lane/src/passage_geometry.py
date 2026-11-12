"""Engstellen auf einem geplanten Pfad, ohne ROS (Obstacle und Tunnel).

check_path legt den Umriss in Pfadrichtung auf den Pfad. Stoesst er an einer Engstelle an,
entscheidet die Gerade mit dem meisten Abstand (passage_line): passt der Bot ausgerichtet
hindurch, faehrt er vorher im Freien auf ihren Start und dreht sich auf sie; sonst wird die
Engstelle gesperrt. Keine festen Breiten: es zaehlt nur, ob der Umriss passt.
"""
import math

import cv2
import numpy as np

from Follow_Lane.src.robot_geometry import BOT_WIDTH, FOOTPRINT, LETHAL_COST, lethal_cells, outline

# Pfadpruefung: so nah am Bot (m, Umkreis) entscheidet der Regler selbst (Drehen am Start), so
# weit voraus wird geprueft; die Fahrtrichtung je Punkt aus den Pfadpunkten +/-HEADING_WINDOW_M
# (NavFn-Pfade springen von Zelle zu Zelle).
CHECK_SKIP_M = 0.105
CHECK_AHEAD_M = 1.0
HEADING_WINDOW_M = 0.05
# Kanten einer Engstelle: je Seite die naechste Wandzelle bis GAP_SEARCH_M.
GAP_SEARCH_M = 0.26
# Gerade fuer die Durchfahrt: um die Achse aus den Kanten bis so weit gedreht (Grad) und quer
# verschoben (m), Footprint so weit (m) vor und hinter die Mitte geschoben.
PASSAGE_ANGLE_DEG = 30
PASSAGE_SHIFT_M = 0.03
PASSAGE_SHIFT_STEP_M = 0.005
PASSAGE_SWEEP_M = 0.12
# Drehen auf der Stelle: Umkreis des Footprints 0,105 m plus 5 mm bis zur Wandzelle.
ROTATE_CLEAR_M = 0.11
# Engstelle entlang der Geraden: wo auf beiden Seiten Wandzellen bis so weit (m) neben dem Bot
# liegen, gesucht bis PASSAGE_REACH_M vor und hinter der Mitte.
FLANK_M = 0.04
PASSAGE_REACH_M = 0.30
# Tunnelwand hoechstens so dick (m, Angabe Team); ihre Rueckseite sieht der LiDAR von vorn nicht.
# Das Ende liegt so weit dahinter, dass das Heck PASSAGE_EXIT_M Abstand hat.
WALL_M = 0.02
PASSAGE_EXIT_M = 0.03
# Sperrlinie quer ueber eine Engstelle: so weit (m) ueber die Kanten hinaus, Punktabstand.
BLOCK_MARGIN_M = 0.02
BLOCK_POINT_STEP_M = 0.01
# Neuplanung im Takt: der neue Pfad ersetzt den alten nur, wenn er SAME_DIRECTION_M voraus
# hoechstens SAME_DIRECTION_DEG anders zeigt (kein Kippen links/rechts um ein Hindernis).
SAME_DIRECTION_M = 0.3
SAME_DIRECTION_DEG = 45.0

FREE = 'frei'
ALIGN = 'ausrichten'
BLOCK = 'sperren'


def path_headings(path_xy, window):
    """Fahrtrichtung (rad) je Pfadpunkt aus den Punkten bis window (m) davor und dahinter."""
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path_xy, axis=0).T))])
    lo = np.searchsorted(s, s - window, side='left')
    hi = np.minimum(np.searchsorted(s, s + window, side='right') - 1, len(s) - 1)
    d = path_xy[hi] - path_xy[lo]
    return np.arctan2(d[:, 1], d[:, 0])


def gap_edges(grid, info_res, origin_xy, point, direction):
    """Lage einer Engstelle aus ihren Kanten: je Seite des Pfads die Wandzelle naechst point
    (bis GAP_SEARCH_M). (Mitte, Achse in Pfadrichtung, Breite) oder None. Die Achse steht
    senkrecht auf der Verbindung der Kanten, auch wenn der Pfad schraeg hindurchfuehrt. Breite
    von Zellmitte zu Zellmitte: die Wandflaeche liegt im Mittel dort."""
    rows, cols = np.nonzero(grid == LETHAL_COST)
    cells = np.asarray(origin_xy) + (np.column_stack([cols, rows]) + 0.5) * info_res
    rel = cells - np.asarray(point)
    dist = np.hypot(rel[:, 0], rel[:, 1])
    left_of_path = direction[0] * rel[:, 1] - direction[1] * rel[:, 0]
    edges = []
    for side in (left_of_path > 0.0, left_of_path < 0.0):
        near = side & (dist <= GAP_SEARCH_M)
        if not near.any():
            return None
        edges.append(cells[near][np.argmin(dist[near])])
    left, right = edges
    across = left - right
    length = float(np.hypot(*across))
    if length <= info_res:
        return None
    axis = np.array([across[1], -across[0]]) / length
    if axis @ np.asarray(direction) < 0.0:
        axis = -axis
    return (left + right) / 2.0, axis, length


def passage_line(grid, info_res, origin_xy, center, axis):
    """Gerade fuer die Durchfahrt einer Engstelle: (Start, Ende, Luft in m) oder None.

    Richtung und Lage: den Footprint +-PASSAGE_SWEEP_M um center entlang schieben, um axis und
    center variiert (PASSAGE_ANGLE_DEG, PASSAGE_SHIFT_M); es gewinnt der groesste Abstand zur
    naechsten Wandzelle, bei fast gleichem der naechste an axis und center.
    Engstelle: der zusammenhaengende Abschnitt um center, in dem links und rechts Wandzellen
    bis FLANK_M neben dem Bot liegen (eine Wand nur auf einer Seite zaehlt nicht). Start: letzter
    Punkt davor mit ROTATE_CLEAR_M Platz (dort dreht der Bot auf der Stelle), sonst None. Ende:
    das Heck PASSAGE_EXIT_M hinter der Wand, mindestens WALL_M dick angenommen."""
    free = (grid != LETHAL_COST).astype(np.uint8)
    clearance = cv2.distanceTransform(free, cv2.DIST_L2, 5) * info_res
    h, w = grid.shape

    def lookup(pts):
        cells = np.floor((pts - np.asarray(origin_xy)) / info_res).astype(np.int64)
        inside = (cells[..., 0] >= 0) & (cells[..., 0] < w) \
            & (cells[..., 1] >= 0) & (cells[..., 1] < h)
        out = np.full(inside.shape, np.inf)
        out[inside] = clearance[cells[inside][:, 1], cells[inside][:, 0]]
        return out

    rim = outline(info_res)
    sweep = np.arange(-PASSAGE_SWEEP_M, PASSAGE_SWEEP_M + info_res / 2, info_res)
    base = math.atan2(axis[1], axis[0])
    shifts = np.arange(-PASSAGE_SHIFT_M, PASSAGE_SHIFT_M + PASSAGE_SHIFT_STEP_M / 2,
                       PASSAGE_SHIFT_STEP_M)
    normal = np.array([-axis[1], axis[0]])
    best = None
    for a in base + np.radians(np.arange(-PASSAGE_ANGLE_DEG, PASSAGE_ANGLE_DEG + 1)):
        d = np.array([math.cos(a), math.sin(a)])
        body = rim @ np.array([[d[0], d[1]], [-d[1], d[0]]])
        for sh in shifts:
            point = np.asarray(center) + sh * normal
            score = float(lookup(point + sweep[:, None, None] * d + body[None, :, :]).min())
            penalty = abs(a - base) + abs(sh)
            if best is None or score > best[0] + info_res / 4 or \
                    (score > best[0] - info_res / 4 and penalty < best[3]):
                best = (score, point, d, penalty)
    score, point, d, _ = best

    rows, cols = np.nonzero(grid == LETHAL_COST)
    rel = np.asarray(origin_xy) + (np.column_stack([cols, rows]) + 0.5) * info_res - point
    along = rel @ d
    across = rel @ np.array([-d[1], d[0]])
    near = (np.abs(across) <= BOT_WIDTH / 2.0 + FLANK_M) & (np.abs(along) <= PASSAGE_REACH_M)
    s = np.arange(-PASSAGE_REACH_M, PASSAGE_REACH_M + info_res / 2, info_res)
    index = np.clip(np.round((along[near] + PASSAGE_REACH_M) / info_res).astype(np.int64),
                    0, s.size - 1)
    flanked = np.ones(s.size, dtype=bool)
    for side in (across[near] > 0.0, across[near] < 0.0):
        hit = np.zeros(s.size + 2, dtype=bool)
        hit[index[side] + 1] = True
        flanked &= hit[:-2] | hit[1:-1] | hit[2:]
    if not flanked.any():
        return None
    mid = int(np.argmin(np.where(flanked, np.abs(s), np.inf)))
    first = last = mid
    while first > 0 and flanked[first - 1]:
        first -= 1
    while last < s.size - 1 and flanked[last + 1]:
        last += 1
    roomy = lookup(point + s[:, None] * d) >= ROTATE_CLEAR_M
    before = np.flatnonzero(roomy[:first + 1])
    if before.size == 0:
        return None
    s_end = max(s[last], s[first] + WALL_M) - FOOTPRINT[:, 0].min() + PASSAGE_EXIT_M
    return point + s[before[-1]] * d, point + s_end * d, score


def check_path(path_xy, grid, res, origin, robot_xy):
    """Passt der Bot auf dem Pfad (N, 2) bis CHECK_AHEAD_M voraus?

    (FREE, None): der Umriss stoesst in Pfadrichtung nirgends an, oder nicht an einer Engstelle
    (dann entscheiden Regler und Waechter).
    (ALIGN, (start, end, center, axis, width)): an einer Engstelle nur ausgerichtet; vorher auf
    start, drehen, gerade bis end.
    (BLOCK, (center, axis, width)): passt auch ausgerichtet nicht hindurch.
    """
    path_xy = np.asarray(path_xy, dtype=np.float64)
    if len(path_xy) < 3:
        return FREE, None
    start = int(np.argmin(np.hypot(*(path_xy - np.asarray(robot_xy)).T)))
    ahead = path_xy[start:]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(ahead, axis=0).T))])
    headings = path_headings(ahead, HEADING_WINDOW_M)
    from_bot = np.hypot(*(ahead - np.asarray(robot_xy)).T)
    for i in np.flatnonzero((from_bot > CHECK_SKIP_M) & (s <= CHECK_AHEAD_M)):
        pose = (ahead[i, 0], ahead[i, 1], headings[i])
        if not lethal_cells(grid, res, origin, pose):
            continue
        direction = np.array([math.cos(headings[i]), math.sin(headings[i])])
        gap = gap_edges(grid, res, origin, ahead[i], direction)
        if gap is None:
            return FREE, None
        center, axis, width = gap
        line = passage_line(grid, res, origin, center, axis)
        if line is None:
            return FREE, None
        if line[2] <= 0.0:
            return BLOCK, (center, axis, width)
        return ALIGN, (line[0], line[1], center, axis, width)
    return FREE, None


def direction_ahead(path_xy, robot_xy, distance=SAME_DIRECTION_M):
    """Richtung (rad) vom Bot zum Pfadpunkt distance (m) voraus, None bei zu kurzem Pfad."""
    path_xy = np.asarray(path_xy, dtype=np.float64)
    if len(path_xy) < 2:
        return None
    start = int(np.argmin(np.hypot(*(path_xy - np.asarray(robot_xy)).T)))
    ahead = path_xy[start:]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(ahead, axis=0).T))])
    target = ahead[min(int(np.searchsorted(s, distance)), len(ahead) - 1)]
    d = target - np.asarray(robot_xy)
    if np.hypot(*d) < 1e-6:
        return None
    return math.atan2(d[1], d[0])


def same_direction(old_xy, new_xy, robot_xy):
    """Zeigen beide Pfade SAME_DIRECTION_M voraus fast in dieselbe Richtung?"""
    a, b = direction_ahead(old_xy, robot_xy), direction_ahead(new_xy, robot_xy)
    if a is None or b is None:
        return True
    return abs(math.atan2(math.sin(a - b), math.cos(a - b))) <= math.radians(SAME_DIRECTION_DEG)


def truncate_at(path_xy, point):
    """Pfad (N, 2) bis zu seinem Punkt naechst point, mit point als letztem Punkt."""
    path_xy = np.asarray(path_xy, dtype=np.float64)
    i = int(np.argmin(np.hypot(*(path_xy - np.asarray(point)).T)))
    return np.vstack([path_xy[:i + 1], np.asarray(point, dtype=np.float64)[None]])


def block_points(center, axis, width):
    """Punkte (N, 2) einer Sperrlinie von Kante zu Kante quer ueber die Engstelle."""
    normal = np.array([-axis[1], axis[0]])
    half = width / 2.0 + BLOCK_MARGIN_M
    count = int(math.ceil(2.0 * half / BLOCK_POINT_STEP_M)) + 1
    return np.asarray(center) + normal * np.linspace(-half, half, count)[:, None]
