"""Geometrie der Tunnelfahrt ohne ROS: Tuer im Scan, Wand-Huelle, Oeffnungen, Frontier.

Alle Grenzwerte haengen am Bot oder an der Karte, keiner an den Massen der Enclosure.
"""
import math

import cv2
import numpy as np

from Follow_Lane.src.common import COSTMAP_BLOCKED
from Follow_Lane.src.robot_geometry import BOT_LENGTH, BOT_WIDTH, FOOTPRINT, LETHAL_COST, transform

# Schmaler als das (m) zaehlt eine freie Flaeche nicht als befahrbar: einzelne Strahlen ins
# Unbekannte. Die Waende sind schon um robot_radius aufgeblasen, die Bot-Mitte darf schmal fahren.
MIN_PASSAGE_M = 0.02
# Liegt die naechste befahrbare Flaeche weiter vom Bot weg, steht er in keiner.
MAX_START_GAP_M = 0.15
# Wandzellen mit einer Luecke bis hierhin (m je Seite) gehoeren zusammen: ab ~1,6 m liegen die
# Strahlen (0,9 Grad) weiter als 2,5 cm auseinander.
WALL_GAP_M = 0.025
# So weit (m) hinter die Tuerlinie faehrt die Einfahrt: Heck 0.101 + xy_goal_tolerance 0.15,
# damit der ganze Bot auch bei fruehem "erreicht" drin ist, plus 1 cm.
DOOR_DEPTH_M = 0.261
# So weit (m) muss ein Strahl hinter der Huelle enden, damit er als "durch die Huelle" zaehlt.
HULL_MARGIN_M = 0.05
# Zielwahl (wie HKA): ein Ausgang liegt in einer Wand. Unbekanntes bis WALL_BAND_M innerhalb der
# Huelle kann ihn verdecken, tiefer drin ist es Schatten hinter Hindernissen. Gezaehlt wird es bis
# GAIN_RADIUS_M um ein Ziel; 1 m^2 davon ist GAIN_WEIGHT m Umweg wert.
WALL_BAND_M = 0.3
GAIN_RADIUS_M = 0.4
GAIN_WEIGHT = 4.0
# Ein Ausgang liegt zwischen zwei geraden Wandstuecken (auch in einer Ecke, dann stehen sie quer
# zueinander); Hindernisse sind klein und meist rund. Je Seite zaehlen die Scanpunkte bis
# WALL_SIDE_M ab der Kante, Nachbarpunkte hoechstens WALL_STEP_M auseinander (sonst anderes
# Objekt). Gerade: mindestens WALL_MIN_LEN_M lang, Abweichung (RMS) von der Geraden hoechstens
# WALL_MAX_RMS_M -- eine Flasche (7 cm) ist zu kurz, ein grosser Zylinder zu krumm.
WALL_SIDE_M = 0.15
WALL_STEP_M = 0.08
WALL_MIN_LEN_M = 0.10
WALL_MAX_RMS_M = 0.012
# Der Kasten ist ein Rechteck aus vier geraden Waenden (box_rectangle). Wandlinie: Wandzellen in
# einem Streifen von BOX_BIN_M, die mindestens BOX_WALL_RUN_M lang zusammenhaengen (Luecken bis
# BOX_RUN_GAP_M) -- runde Hindernisse sind kuerzer. Jede Seite mindestens BOX_MIN_SIDE_M.
BOX_BIN_M = 0.04
BOX_WALL_RUN_M = 0.45
BOX_RUN_GAP_M = 0.04
BOX_MIN_SIDE_M = 0.40
# Auf dem Rechteckrand gilt eine Stelle als Wand, wenn eine Wandzelle naeher als das (m) liegt.
BOX_WALL_TOL_M = 0.025
# Abtastschritt (m) auf dem Rechteckrand.
BOX_STEP_M = 0.01
# So weit (m) hinter einem Ausgang muss der Scan frei nach draussen gesehen haben.
BOX_SEE_THROUGH_M = 0.12


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
    [(mitte auf der Huelle, breite, richtung nach aussen, wandende a, wandende b)]. Graue
    Bereiche innerhalb der Huelle sind Schatten hinter Hindernissen und tauchen hier nicht auf.
    Nur Luecken zwischen zwei geraden Wandstuecken zaehlen (straight_wall_sides); a und b sind
    die Scanpunkte an deren Kanten, a-b ist die Tuerlinie des Ausgangs.
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

    finite = np.isfinite(ranges)
    points = origin + dirs * np.where(finite, ranges, 0.0)[:, None]
    openings = []
    for group in groups:
        side_a = _wall_side(points, finite, leaves, group[0], -1)
        side_b = _wall_side(points, finite, leaves, group[-1], +1)
        if not (straight_wall(side_a) and straight_wall(side_b)) \
                or np.hypot(*(side_b[0] - side_a[0])) < BOT_WIDTH:
            continue
        a, b = crossings[group[0]], crossings[group[-1]]
        width = float(np.hypot(*(b - a)))
        center = (a + b) / 2.0
        direction = center - origin
        openings.append((center, width, direction / max(np.hypot(*direction), 1e-9),
                         side_a[0], side_b[0]))
    openings.sort(key=lambda o: -o[1])
    return openings


def _wall_side(points, finite, leaves, edge, step):
    """Scanpunkte neben einer Luecke, ab dem Strahl hinter edge in Richtung step: solange sie
    zusammenhaengen und hoechstens WALL_SIDE_M von der Kante entfernt liegen."""
    n = len(points)
    side = []
    i = edge
    for _ in range(n - 1):
        i = (i + step) % n
        if not finite[i] or leaves[i]:
            break
        p = points[i]
        if side and (np.hypot(*(p - side[-1])) > WALL_STEP_M
                     or np.hypot(*(p - side[0])) > WALL_SIDE_M):
            break
        side.append(p)
    return np.array(side).reshape(-1, 2)


def straight_wall(points):
    """Liegen die Punkte auf einem geraden Stueck von mindestens WALL_MIN_LEN_M?"""
    if len(points) < 3:
        return False
    centered = points - points.mean(axis=0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    along, across = centered @ axes[0], centered @ axes[1]
    return np.ptp(along) >= WALL_MIN_LEN_M and math.sqrt(np.mean(across ** 2)) <= WALL_MAX_RMS_M


def exit_line(a, b, inside):
    """Tuerlinie a-b eines Ausgangs: (mitte, halbe breite, richtung entlang, normale nach
    aussen), die Normale zeigt von inside weg."""
    a, b = np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64)
    width = max(float(np.hypot(*(b - a))), 1e-9)
    along = (b - a) / width
    normal = np.array([-along[1], along[0]])
    center = (a + b) / 2.0
    if normal @ (center - np.asarray(inside, dtype=np.float64)) < 0:
        normal = -normal
    return center, width / 2.0, along, normal


def _rotate(points, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray(points, dtype=np.float64) @ np.array([[c, -s], [s, c]])


def box_orientation(points):
    """Winkel (rad, 0..90 Grad) der Rechteckachsen: dort haeufen sich die Wandzellen in den
    Projektionen auf beide Achsen am staerksten (schmale, hohe Spitzen)."""
    def score(angle):
        uv = _rotate(points, angle)
        total = 0.0
        for k in (0, 1):
            h = np.bincount(np.floor((uv[:, k] - uv[:, k].min()) / 0.01).astype(np.int64))
            total += float(np.sum(h.astype(np.float64) ** 2))
        return total
    coarse = max(np.radians(np.arange(0.0, 90.0, 1.0)), key=score)
    fine = coarse + np.radians(np.arange(-1.0, 1.01, 0.1))
    return float(max(fine, key=score)) % (math.pi / 2)


def _longest_run(values, gap):
    if values.size == 0:
        return 0.0
    v = np.sort(values)
    breaks = np.nonzero(np.diff(v) > gap)[0]
    starts = np.concatenate([[0], breaks + 1])
    ends = np.concatenate([breaks, [v.size - 1]])
    return float((v[ends] - v[starts]).max())


def _wall_lines(along_axis, across_axis):
    """Lagen (Koordinate auf along_axis) gerader Wandlinien quer zu dieser Achse."""
    lines = []
    lo, hi = along_axis.min(), along_axis.max()
    step = BOX_BIN_M / 2.0
    for start in np.arange(lo, hi + step, step):
        sel = (along_axis >= start) & (along_axis < start + BOX_BIN_M)
        if np.count_nonzero(sel) * 0.01 < BOX_WALL_RUN_M:
            continue
        if _longest_run(across_axis[sel], BOX_RUN_GAP_M) >= BOX_WALL_RUN_M:
            lines.append(float(np.median(along_axis[sel])))
    return lines


def box_rectangle(walls_xy, inside_xy):
    """Rechteck des Kastens um inside_xy (ein Punkt sicher im Kasten): je Richtung die naechste
    gerade Wandlinie. Laeuft eine Kastenwand als Streckenbegrenzung weiter, ist das dieselbe
    Linie -- das Rechteck endet trotzdem an der Querwand. Ergebnis (4, 1, 2) float32 wie eine
    Kontur (fuer pointPolygonTest), None solange eine Seite fehlt."""
    walls_xy = np.asarray(walls_xy, dtype=np.float64).reshape(-1, 2)
    if len(walls_xy) < 4 * BOX_WALL_RUN_M / 0.01:
        return None
    inside_xy = np.asarray(inside_xy, dtype=np.float64)
    angle = box_orientation(walls_xy - inside_xy)
    uv = _rotate(walls_xy - inside_xy, angle)
    u_lines = _wall_lines(uv[:, 0], uv[:, 1])
    v_lines = _wall_lines(uv[:, 1], uv[:, 0])
    sides = []
    for lines, sign in ((u_lines, 1), (u_lines, -1), (v_lines, 1), (v_lines, -1)):
        beyond = [c for c in lines if c * sign > 0.0]
        if not beyond:
            return None
        sides.append(min(beyond, key=abs))
    u1, u0, v1, v0 = sides
    if u1 - u0 < BOX_MIN_SIDE_M or v1 - v0 < BOX_MIN_SIDE_M:
        return None
    corners = np.array([[u0, v0], [u1, v0], [u1, v1], [u0, v1]])
    xy = _rotate(corners, -angle) + inside_xy
    return xy.astype(np.float32).reshape(-1, 1, 2)


def box_exit(rect, walls_xy, known, exclude_xy, exclude_r):
    """Der Ausgang des Kastens: das breiteste Stueck Rechteckrand ohne Wand, mindestens
    BOT_WIDTH breit, auf beiden Seiten von Wand begrenzt (nicht von Unbekanntem) und nicht am
    Eingang (naeher als exclude_r an exclude_xy). known(points) -> bool-Maske: Zelle gescannt.
    Ergebnis (a, b) in odom oder None."""
    corners = rect.reshape(-1, 2).astype(np.float64)
    samples = []
    for p, q in zip(corners, np.roll(corners, -1, axis=0)):
        n = max(1, int(math.ceil(np.hypot(*(q - p)) / BOX_STEP_M)))
        samples.append(p + (q - p) * (np.arange(n)[:, None] / n))
    samples = np.vstack(samples)
    walls_xy = np.asarray(walls_xy, dtype=np.float64).reshape(-1, 2)

    # Wand: Wandzelle naeher als BOX_WALL_TOL_M -- Abstandsbild um das Rechteck (1 cm).
    cell = 0.01
    margin = BOX_SEE_THROUGH_M + 2 * BOX_WALL_TOL_M
    lo = samples.min(axis=0) - margin
    shape = (np.ceil((samples.max(axis=0) + margin - lo) / cell).astype(np.int64) + 1)[::-1]
    free = np.ones(shape, np.uint8)
    wc = np.floor((walls_xy - lo) / cell).astype(np.int64)
    ok = (wc >= 0).all(axis=1) & (wc[:, 0] < shape[1]) & (wc[:, 1] < shape[0])
    free[wc[ok, 1], wc[ok, 0]] = 0
    dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * cell
    sc = np.floor((samples - lo) / cell).astype(np.int64)
    wall = dist[sc[:, 1], sc[:, 0]] < BOX_WALL_TOL_M
    state = np.where(wall, 1, np.where(known(samples), 0, 2))   # 0 offen, 1 Wand, 2 unbekannt
    if not (state == 1).any():
        return None

    n = len(samples)
    start = int(np.argmax(state == 1))
    best = None
    k = 0
    while k < n:
        i = (start + k) % n
        if state[i] != 0:
            k += 1
            continue
        run = []
        while k < n and state[(start + k) % n] == 0:
            run.append((start + k) % n)
            k += 1
        before, after = state[(run[0] - 1) % n], state[(run[-1] + 1) % n]
        if before != 1 or after != 1:
            continue
        a, b = samples[run[0]], samples[run[-1]]
        width = float(np.hypot(*(b - a)))
        if width < BOT_WIDTH - 2 * BOX_WALL_TOL_M:
            continue
        if exclude_xy is not None and \
                np.min(np.hypot(*(samples[run] - np.asarray(exclude_xy)).T)) < exclude_r:
            continue
        if not _sees_through(samples[run], corners.mean(axis=0), known, dist, lo, cell):
            continue
        if best is None or width > best[2]:
            best = (a, b, width)
    return None if best is None else (best[0], best[1])


def _sees_through(gap, box_center, known, dist, lo, cell):
    """Hat der LiDAR durch die Luecke nach draussen gesehen? Hinter der mittleren Haelfte
    (bis BOX_SEE_THROUGH_M) muss die Flaeche gescannt und wandfrei sein. Ein Loch in einer
    schraeg gesehenen Wand (Scanpunkte zu weit auseinander) hat dahinter nichts Gescanntes."""
    a, b = gap[0], gap[-1]
    along = (b - a) / max(np.hypot(*(b - a)), 1e-9)
    normal = np.array([-along[1], along[0]])
    if normal @ ((a + b) / 2.0 - box_center) < 0:
        normal = -normal
    k = len(gap)
    middle = gap[k // 4: k - k // 4]
    probes = np.vstack([middle + normal * d for d in np.arange(BOX_WALL_TOL_M, BOX_SEE_THROUGH_M + 1e-9, 0.02)])
    pc = np.floor((probes - lo) / cell).astype(np.int64)
    clear = known(probes) & (dist[pc[:, 1], pc[:, 0]] >= BOX_WALL_TOL_M)
    return np.mean(clear) >= 0.8


def bot_past_exit(line, pose, margin):
    """Steht der ganze Bot-Umriss mindestens margin hinter der Tuerlinie und nicht weiter als
    eine Bot-Laenge seitlich neben dem Ausgang?"""
    center, half_width, along, normal = line
    rel = transform(pose, FOOTPRINT) - center
    if np.abs(rel @ along).min() > half_width + BOT_LENGTH:
        return False
    return bool((rel @ normal).min() >= margin)


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


def wall_band_unknown(grid, hull, info_res, origin_xy):
    """Maske der unbekannten Zellen in der Huelle bis WALL_BAND_M von ihrem Rand."""
    inside = hull_mask(hull, grid.shape, info_res, origin_xy).astype(np.uint8)
    depth = cv2.distanceTransform(inside, cv2.DIST_L2, 5) * info_res
    return (grid == -1) & (inside > 0) & (depth <= WALL_BAND_M)


def frontier_gain(band, info_res, origin_xy, target):
    """Unbekannte Flaeche an der Wand (m^2) bis GAIN_RADIUS_M um target."""
    r = int(math.ceil(GAIN_RADIUS_M / info_res))
    c = int((target[0] - origin_xy[0]) / info_res)
    row = int((target[1] - origin_xy[1]) / info_res)
    h, w = band.shape
    r0, r1, c0, c1 = max(0, row - r), min(h, row + r + 1), max(0, c - r), min(w, c + r + 1)
    if r0 >= r1 or c0 >= c1:
        return 0.0
    rows, cols = np.mgrid[r0:r1, c0:c1]
    disk = np.hypot(rows - row, cols - c) <= r
    return float(np.count_nonzero(band[r0:r1, c0:c1] & disk)) * info_res * info_res
