"""Geometrie der Tunnelfahrt ohne ROS: Tuer im Scan, Wand-Huelle, Oeffnungen, Frontier.

Alle Grenzwerte haengen am Bot oder an der Karte, keiner an den Massen der Enclosure.
"""
import math

import cv2
import numpy as np

from Follow_Lane.src.common import COSTMAP_BLOCKED

# Umriss wie local_costmap.footprint in nav2_params.yaml (base_footprint).
FOOTPRINT = np.array([[0.037, -0.069], [0.037, 0.069], [0.033, 0.089], [-0.033, 0.089],
                      [-0.079, 0.069], [-0.092, 0.05], [-0.101, 0.028], [-0.101, -0.028],
                      [-0.092, -0.05], [-0.079, -0.069], [-0.033, -0.089], [0.033, -0.089]])
BOT_LENGTH = 0.138
BOT_WIDTH = 0.178
LETHAL_COST = 100
# Schmaler als das (m) zaehlt eine freie Flaeche nicht als befahrbar: einzelne Strahlen ins
# Unbekannte. Die Waende sind schon um robot_radius aufgeblasen, die Bot-Mitte darf schmal fahren.
MIN_PASSAGE_M = 0.02
# Liegt die naechste befahrbare Flaeche weiter vom Bot weg, steht er in keiner.
MAX_START_GAP_M = 0.15
# Wandzellen mit einer Luecke bis hierhin (m je Seite) gehoeren zusammen: ab ~1,6 m liegen die
# Strahlen (0,9 Grad) weiter als 2,5 cm auseinander.
WALL_GAP_M = 0.025
# So weit (m) hinter die Tuerlinie faehrt die Einfahrt: Heck 0.101 + footprint_padding 0.01 +
# xy_goal_tolerance 0.15, damit der ganze Bot auch bei fruehem "erreicht" drin ist.
DOOR_DEPTH_M = 0.261
# So weit (m) muss ein Strahl hinter der Huelle enden, damit er als "durch die Huelle" zaehlt.
HULL_MARGIN_M = 0.05
# Gibt der Regler auf einem Pfad auf, zaehlt dessen engste Stelle bis BLOCK_LOOKAHEAD_M voraus:
# schmaler als PASSAGE_WIDTH_M kommt MPPI nicht sicher hindurch (Drehen auf der Stelle braucht
# 0,238 m), dann faehrt switch_control sie mit Grundbewegungen; schmaler als BLOCK_WIDTH_M
# (Bot 0,178 m) wird sie gesperrt.
BLOCK_WIDTH_M = 0.21
PASSAGE_WIDTH_M = 0.26
# Gerade fuer die Durchfahrt (passage_line): um die Achse aus den Kanten bis so weit gedreht
# (Grad) und quer verschoben (m), Footprint so weit (m) vor und hinter die Mitte geschoben.
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
BLOCK_LOOKAHEAD_M = 0.6


def scan_rays(angles, ranges):
    """(Winkel, Entfernung) nach Winkel sortiert; ohne Echo (nan, inf) wird zu inf."""
    order = np.argsort(angles)
    angles, ranges = np.asarray(angles)[order], np.asarray(ranges, dtype=np.float64)[order]
    return angles, np.where(np.isfinite(ranges), ranges, np.inf)


def _past_chord(points, a, b, origin):
    """Abstand der Punkte hinter der Strecke a-b, von origin aus gesehen (positiv = dahinter)."""
    chord = b - a
    normal = np.array([-chord[1], chord[0]]) / max(np.hypot(*chord), 1e-9)
    if normal @ (a - origin) < 0:
        normal = -normal
    return (points - a) @ normal


def find_openings(angles, ranges, origin=(0.0, 0.0)):
    """Oeffnungen im Scan, naechste zuerst: [(a, b, mitte, breite, normale nach innen)].

    Eine Tuer sind zwei Wandenden, zwischen denen die Strahlen in der Mitte mindestens eine
    Bot-Laenge weiter reichen. Wandende heisst: Sprung in der Entfernung zu einem Nachbarstrahl
    (bei einem Rohr ohne Vorderwand zeigt der Sprung nach aussen, nicht in die Tuer). Am Rand
    der Tuer darf die Wand weiterlaufen, deshalb zaehlt nur die mittlere Haelfte. Winkel und
    Entfernungen gelten ab origin (Scanner), Ergebnis im selben Frame.
    """
    angles, ranges = scan_rays(angles, ranges)
    n = len(ranges)
    if n < 3:
        return []
    origin = np.asarray(origin, dtype=np.float64)
    dirs = np.column_stack([np.cos(angles), np.sin(angles)])
    finite = np.isfinite(ranges)
    points = origin + dirs * np.where(finite, ranges, 0.0)[:, None]
    jump = (np.roll(ranges, 1) - ranges >= BOT_LENGTH) | (np.roll(ranges, -1) - ranges >= BOT_LENGTH)
    ends = np.nonzero(finite & jump)[0]

    openings = []
    for i in ends:
        for j in ends:
            step = (j - i) % n
            if step < 2 or step >= n // 2:
                continue
            a, b = points[i], points[j]
            width = float(np.hypot(*(b - a)))
            if width < BOT_WIDTH:
                continue
            between = (i + np.arange(step // 4 + 1, step - step // 4)) % n
            seen = between[finite[between]]
            if seen.size and _past_chord(points[seen], a, b, origin).min() < BOT_LENGTH:
                continue
            center = (a + b) / 2.0
            normal = np.array([-(b - a)[1], (b - a)[0]]) / width
            if normal @ (center - origin) < 0:
                normal = -normal
            openings.append((a, b, center, width, normal))
    openings = [o for o in openings if not _contains_narrower(o, openings, origin)]
    openings.sort(key=lambda o: np.hypot(*(o[2] - origin)))
    return openings


def _contains_narrower(opening, openings, origin):
    """Liegt eine schmalere Oeffnung auf derselben Linie in dieser? Blickt der Scanner flach an
    einer Wand entlang, springen auch Nachbarstrahlen derselben Wand: die breite Sehne ist dann
    falsch, die schmalere ist die Tuer. Eine Tuer weiter hinten zaehlt nicht."""
    a, b, _, width, _ = opening
    along = (b - a) / width
    for other in openings:
        if other[3] >= width:
            continue
        edges = np.array([other[0], other[1]])
        t = (edges - a) @ along
        if t.min() >= 0.0 and t.max() <= width                 and np.abs(_past_chord(edges, a, b, origin)).max() < BOT_LENGTH:
            return True
    return False


def transform(pose, xy):
    """Punkte (N, 2) aus dem Frame von pose = (x, y, yaw) in dessen Eltern-Frame."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    xy = np.asarray(xy, dtype=np.float64)
    return np.column_stack([pose[0] + c * xy[:, 0] - s * xy[:, 1],
                            pose[1] + s * xy[:, 0] + c * xy[:, 1]])


def convex_hull(points):
    """Konvexe Huelle (K, 1, 2) float32, None bei weniger als 3 Punkten ohne Flaeche."""
    if len(points) < 3:
        return None
    hull = cv2.convexHull(np.asarray(points, dtype=np.float32))
    return hull if cv2.contourArea(hull) > 0.0 else None


def _hull_distance(hull, point):
    return cv2.pointPolygonTest(hull, (float(point[0]), float(point[1])), True)


def bot_inside(hull, pose):
    """Liegt der ganze Bot-Umriss innerhalb der Huelle?"""
    return all(_hull_distance(hull, p) >= 0.0 for p in transform(pose, FOOTPRINT))


def bot_outside(hull, pose):
    """Liegt der ganze Bot-Umriss ausserhalb der Huelle?"""
    return all(_hull_distance(hull, p) < 0.0 for p in transform(pose, FOOTPRINT))


def hull_openings(hull, origin, angles, ranges):
    """Strahlen, die aus der Huelle hinausreichen, als Luecken in der Huelle.

    origin liegt in der Huelle, angles sind Weltwinkel. Ergebnis, breiteste zuerst:
    [(mitte auf der Huelle, breite, richtung nach aussen)]. Graue Bereiche innerhalb der Huelle
    sind Schatten hinter Hindernissen und tauchen hier nicht auf.
    """
    angles, ranges = scan_rays(angles, ranges)
    origin = np.asarray(origin, dtype=np.float64)
    corners = hull.reshape(-1, 2).astype(np.float64)
    edges = np.roll(corners, -1, axis=0) - corners
    dirs = np.column_stack([np.cos(angles), np.sin(angles)])

    # Austrittspunkt jedes Strahls aus der konvexen Huelle.
    denom = dirs[:, None, 0] * edges[None, :, 1] - dirs[:, None, 1] * edges[None, :, 0]
    to_corner = corners - origin
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (to_corner[None, :, 0] * edges[None, :, 1] - to_corner[None, :, 1] * edges[None, :, 0]) / denom
        u = (to_corner[None, :, 0] * dirs[:, None, 1] - to_corner[None, :, 1] * dirs[:, None, 0]) / denom
    t = np.where((t > 0.0) & (u >= 0.0) & (u <= 1.0), t, np.inf)
    exit_t = t.min(axis=1)
    leaves = np.isfinite(exit_t) & (ranges >= exit_t + HULL_MARGIN_M)
    if not leaves.any():
        return []

    crossings = origin + dirs * np.where(np.isfinite(exit_t), exit_t, 0.0)[:, None]
    n = len(leaves)
    # Beginn am ersten Strahl, der nicht hinausreicht, damit keine Gruppe ueber das Array-Ende reisst.
    start = int(np.argmin(leaves)) if not leaves.all() else 0
    groups, current = [], []
    for k in range(n):
        i = (start + k) % n
        if leaves[i]:
            current.append(i)
        elif current:
            groups.append(current)
            current = []
    if current:
        groups.append(current)

    openings = []
    for group in groups:
        a, b = crossings[group[0]], crossings[group[-1]]
        width = float(np.hypot(*(b - a)))
        center = (a + b) / 2.0
        direction = center - origin
        openings.append((center, width, direction / max(np.hypot(*direction), 1e-9)))
    openings.sort(key=lambda o: -o[1])
    return openings


def reachable_free(grid, info_res, origin_xy, robot_xy):
    """Maske der mit dem Bot verbundenen befahrbaren Flaeche, None wenn er in keiner steht.

    Befahrbar heisst: was NavFn befahren darf (unter COSTMAP_BLOCKED, nicht unbekannt).
    """
    free = (grid >= 0) & (grid < COSTMAP_BLOCKED)
    k = max(1, int(round(MIN_PASSAGE_M / info_res)))
    free = cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_OPEN, np.ones((k, k), np.uint8)).astype(bool)
    if not free.any():
        return None
    _, regions = cv2.connectedComponents(free.astype(np.uint8), connectivity=8)
    rows, cols = np.nonzero(free)
    rc = (robot_xy[0] - origin_xy[0]) / info_res - 0.5
    rr = (robot_xy[1] - origin_xy[1]) / info_res - 0.5
    nearest = np.argmin((cols - rc) ** 2 + (rows - rr) ** 2)
    if math.hypot(cols[nearest] - rc, rows[nearest] - rr) * info_res > MAX_START_GAP_M:
        return None
    return regions == regions[rows[nearest], cols[nearest]]


def frontier_mask(free, grid):
    """Freie Zellen direkt neben unbekannten."""
    unknown = (grid == -1).astype(np.uint8)
    return free & (cv2.dilate(unknown, np.ones((3, 3), np.uint8)) > 0)


def frontier_targets(mask, info_res, origin_xy):
    """Eine Zielzelle (x, y) je Frontier-Gruppe, die mindestens so breit wie der Bot ist:
    die Zelle am Schwerpunkt, denn der Schwerpunkt selbst kann im Unbekannten liegen."""
    min_cells = int(math.ceil(BOT_WIDTH / info_res))
    count, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
    targets = []
    for label in range(1, count):
        rows, cols = np.nonzero(labels == label)
        if rows.size < min_cells:
            continue
        i = np.argmin((cols - cols.mean()) ** 2 + (rows - rows.mean()) ** 2)
        targets.append((origin_xy[0] + (cols[i] + 0.5) * info_res,
                        origin_xy[1] + (rows[i] + 0.5) * info_res))
    return targets


def nearest_free(free, info_res, origin_xy, point, max_distance):
    """Mitte der befahrbaren Zelle am naechsten zu point, None wenn weiter als max_distance."""
    rows, cols = np.nonzero(free)
    if rows.size == 0:
        return None
    xs = origin_xy[0] + (cols + 0.5) * info_res
    ys = origin_xy[1] + (rows + 0.5) * info_res
    d = np.hypot(xs - point[0], ys - point[1])
    i = np.argmin(d)
    return (float(xs[i]), float(ys[i])) if d[i] <= max_distance else None


def hull_mask(hull, shape, info_res, origin_xy):
    """Rastermaske der Flaeche innerhalb der Huelle."""
    cells = (hull.reshape(-1, 2) - np.asarray(origin_xy)) / info_res
    mask = np.zeros(shape, np.uint8)
    cv2.fillConvexPoly(mask, np.round(cells).astype(np.int32), 1)
    return mask.astype(bool)


def connected_walls(grid, seeds_rc, info_res):
    """Wandzellen (Maske), die mit den Seeds (N, 2; Zeile, Spalte) verbunden sind."""
    lethal = (grid == LETHAL_COST).astype(np.uint8)
    size = 2 * max(1, int(round(WALL_GAP_M / info_res))) + 1
    closed = cv2.dilate(lethal, np.ones((size, size), np.uint8))
    _, labels = cv2.connectedComponents(closed, connectivity=8)
    h, w = grid.shape
    keep = set()
    for r, c in np.asarray(seeds_rc, dtype=np.int64).reshape(-1, 2):
        if 0 <= r < h and 0 <= c < w and labels[r, c] > 0:
            keep.add(int(labels[r, c]))
    if not keep:
        return np.zeros(grid.shape, dtype=bool)
    return (lethal > 0) & np.isin(labels, list(keep))


def side_wall_seeds(grid, info_res, origin_xy, pose):
    """Naechste Wandzelle links und rechts des Bots (Zeile, Spalte), fuer den Start im Tunnel."""
    rows, cols = np.nonzero(grid == LETHAL_COST)
    if rows.size == 0:
        return np.empty((0, 2), dtype=np.int64)
    xs = origin_xy[0] + (cols + 0.5) * info_res - pose[0]
    ys = origin_xy[1] + (rows + 0.5) * info_res - pose[1]
    lateral = -math.sin(pose[2]) * xs + math.cos(pose[2]) * ys
    d = np.hypot(xs, ys)
    seeds = []
    for side in (lateral > 0.0, lateral < 0.0):
        if side.any():
            idx = np.nonzero(side)[0]
            i = idx[np.argmin(d[idx])]
            seeds.append((rows[i], cols[i]))
    return np.array(seeds, dtype=np.int64)


def narrowest_ahead(path_xy, grid, info_res, origin_xy, robot_xy):
    """Engste Stelle auf dem Pfad ab dem Bot bis BLOCK_LOOKAHEAD_M: (Punkt, Pfadrichtung,
    Breite) oder None. Breite quer zum Pfad: bis zur Wand links plus bis zur Wand rechts;
    fehlt eine Seite innerhalb PASSAGE_WIDTH_M, ist dort keine Engstelle."""
    lethal = grid == LETHAL_COST
    if not lethal.any() or len(path_xy) < 3:
        return None
    start = int(np.argmin(np.hypot(*(path_xy - np.asarray(robot_xy)).T)))
    seg = np.hypot(*np.diff(path_xy[start:], axis=0).T)
    end = min(len(path_xy), start + int(np.searchsorted(np.cumsum(seg), BLOCK_LOOKAHEAD_M)) + 2)
    tangent = np.gradient(path_xy, axis=0)[start:end]
    tangent /= np.maximum(np.hypot(tangent[:, 0], tangent[:, 1]), 1e-9)[:, None]
    ahead = path_xy[start:end]
    if len(ahead) < 2:
        return None
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    steps = np.arange(1, int(PASSAGE_WIDTH_M / info_res) + 1) * info_res
    h, w = grid.shape

    def wall_distance(side):
        pts = ahead[:, None, :] + side * normal[:, None, :] * steps[None, :, None]
        cells = np.floor((pts - np.asarray(origin_xy)) / info_res).astype(np.int64)
        ok = (cells[..., 0] >= 0) & (cells[..., 0] < w) & (cells[..., 1] >= 0) & (cells[..., 1] < h)
        hit = np.zeros(ok.shape, dtype=bool)
        hit[ok] = lethal[cells[ok][:, 1], cells[ok][:, 0]]
        return np.where(hit.any(axis=1), steps[np.argmax(hit, axis=1)], np.inf)

    width = wall_distance(1.0) + wall_distance(-1.0)
    i = int(np.argmin(width))
    if not np.isfinite(width[i]):
        return None
    return ahead[i], tangent[i], float(width[i])


def gap_edges(grid, info_res, origin_xy, point, direction):
    """Lage einer Engstelle aus ihren Kanten: je Seite des Pfads die Wandzelle naechst point
    (bis PASSAGE_WIDTH_M). (Mitte, Achse in Pfadrichtung, Breite) oder None. Die Achse steht
    senkrecht auf der Verbindung der Kanten, auch wenn der Pfad schraeg hindurchfuehrt. Breite
    von Zellmitte zu Zellmitte: die Wandflaeche liegt im Mittel dort."""
    rows, cols = np.nonzero(grid == LETHAL_COST)
    cells = np.asarray(origin_xy) + (np.column_stack([cols, rows]) + 0.5) * info_res
    rel = cells - np.asarray(point)
    dist = np.hypot(rel[:, 0], rel[:, 1])
    left_of_path = direction[0] * rel[:, 1] - direction[1] * rel[:, 0]
    edges = []
    for side in (left_of_path > 0.0, left_of_path < 0.0):
        near = side & (dist <= PASSAGE_WIDTH_M)
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

    corners = np.vstack([FOOTPRINT, FOOTPRINT[:1]])
    outline = np.vstack([np.linspace(a, b, max(2, int(np.hypot(*(b - a)) / info_res) + 1))
                         for a, b in zip(corners[:-1], corners[1:])])
    sweep = np.arange(-PASSAGE_SWEEP_M, PASSAGE_SWEEP_M + info_res / 2, info_res)
    base = math.atan2(axis[1], axis[0])
    shifts = np.arange(-PASSAGE_SHIFT_M, PASSAGE_SHIFT_M + PASSAGE_SHIFT_STEP_M / 2,
                       PASSAGE_SHIFT_STEP_M)
    normal = np.array([-axis[1], axis[0]])
    best = None
    for a in base + np.radians(np.arange(-PASSAGE_ANGLE_DEG, PASSAGE_ANGLE_DEG + 1)):
        d = np.array([math.cos(a), math.sin(a)])
        body = outline @ np.array([[d[0], d[1]], [-d[1], d[0]]])
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
