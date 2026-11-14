"""Selbstpruefung von robot_geometry und passage_geometry (ohne ROS).

Aufruf: python workspace/src/Follow_Lane/test/test_geometry.py   (oder pytest)
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from Follow_Lane.src import passage_geometry as pg  # noqa: E402
from Follow_Lane.src import robot_geometry as rg  # noqa: E402

RES = 0.01
ORIGIN = (-1.0, -1.0)
SIZE = 200


def empty_grid():
    return np.zeros((SIZE, SIZE), dtype=np.int16)


def wall_x(grid, x, y_from=-1.0, y_to=1.0):
    """Wand aus belegten Zellen entlang x = const (2 Zellen dick)."""
    c = int((x - ORIGIN[0]) / RES)
    r0, r1 = int((y_from - ORIGIN[1]) / RES), int((y_to - ORIGIN[1]) / RES)
    grid[max(0, r0):min(SIZE, r1), c:c + 2] = rg.LETHAL_COST
    return grid


def gap_grid(half_width):
    """Wand bei x = 0,5 mit einer Luecke |y| < half_width."""
    grid = empty_grid()
    wall_x(grid, 0.5, -1.0, -half_width)
    wall_x(grid, 0.5, half_width, 1.0)
    return grid


def test_simulate():
    p = rg.simulate((0.0, 0.0, 0.0), 0.1, 0.0, 1.0, RES)
    assert np.allclose(p[-1], [0.1, 0.0, 0.0])
    p = rg.simulate((0.0, 0.0, 0.0), 0.0, 1.0, math.pi / 2, RES)
    assert np.allclose(p[-1], [0.0, 0.0, math.pi / 2])
    p = rg.simulate((0.0, 0.0, 0.0), 0.1, 0.1, math.pi * 10, RES)   # Halbkreis r = 1
    assert np.allclose(p[-1, :2], [0.0, 2.0], atol=1e-9)


def test_motion_collides_and_touching():
    grid = wall_x(empty_grid(), 0.10)
    forward = rg.simulate((0.0, 0.0, 0.0), 0.1, 0.0, 1.0, RES)
    back = rg.simulate((0.0, 0.0, 0.0), -0.1, 0.0, 1.0, RES)
    assert rg.motion_collides(grid, RES, ORIGIN, forward)
    assert not rg.motion_collides(grid, RES, ORIGIN, back)
    # liegt vorn schon an: rueckwaerts weg erlaubt, weiter vor nicht
    grid = wall_x(empty_grid(), 0.03)
    assert rg.lethal_cells(grid, RES, ORIGIN, (0.0, 0.0, 0.0))
    assert not rg.motion_collides(grid, RES, ORIGIN, back)
    assert rg.motion_collides(grid, RES, ORIGIN, forward)


def test_unwedge():
    grid = wall_x(empty_grid(), 0.045)        # Wand 0,8 cm vor der Front
    assert rg.choose_unwedge(grid, RES, ORIGIN, (0.0, 0.0, 0.0)) == ('straight', -0.05)
    box = empty_grid()                        # eng eingeschlossen: kein Ausweg
    box[:, :] = rg.LETHAL_COST
    c0, c1 = int((-0.11 - ORIGIN[0]) / RES), int((0.045 - ORIGIN[0]) / RES)
    r0, r1 = int((-0.095 - ORIGIN[1]) / RES), int((0.095 - ORIGIN[1]) / RES)
    box[r0:r1, c0:c1] = 0
    assert rg.choose_unwedge(box, RES, ORIGIN, (0.0, 0.0, 0.0)) is None


def test_correct_scan_offset():
    r = [1.0, float('inf'), 0.0, 2.0]
    out = rg.correct_scan(r, 0.0, 0.1, 0.1, 0.017, (0.0, 0.0, 0.0), None)
    assert np.allclose(out[[0, 3]], [1.017, 2.017]) and math.isinf(out[1]) and out[2] == 0.0


def test_correct_scan_deskew_rotation():
    # Bot dreht beim Messen mit 1,2 rad/s, Wand bei x = 1 (Frame der Pose beim ersten Strahl)
    n, inc = 360, 2 * math.pi / 360
    angle_min = -math.pi
    angles = angle_min + np.arange(n) * inc
    turned = 1.2 * np.arange(n) * (0.2 / n)                 # 0,2 s je Umlauf
    world = angles + turned
    ranges = np.where(np.cos(world) > 0.5, 1.0 / np.maximum(np.cos(world), 1e-9), np.inf)
    rel = np.column_stack([np.zeros(n), np.zeros(n), turned])
    out = rg.correct_scan(ranges, angle_min, inc, 0.05, 0.0, (0.0, 0.0, 0.0), rel)
    check = np.abs(angles) < math.radians(40)
    good = check & np.isfinite(out)
    assert good.sum() > 0.9 * check.sum()
    assert np.allclose(out[good], 1.0 / np.cos(angles[good]), atol=0.01)
    raw_err = np.abs(ranges[check] - 1.0 / np.cos(angles[check])).max()
    assert raw_err > 0.05                                   # ohne Korrektur deutlich falsch


def test_check_path():
    straight = np.column_stack([np.arange(0.0, 1.0, 0.01), np.zeros(100)])
    assert pg.check_path(straight, gap_grid(0.10), RES, ORIGIN, (0.0, 0.0))[0] == pg.FREE

    d = np.array([math.cos(math.radians(30)), math.sin(math.radians(30))])
    s = np.arange(-0.6, 0.6, 0.01)
    skew = np.array([0.5, 0.0]) + s[:, None] * d
    verdict, info = pg.check_path(skew, gap_grid(0.10), RES, ORIGIN, skew[0])
    assert verdict == pg.ALIGN, verdict
    start, end, center, axis, width = info
    # Start: Front noch vor der Wand (x 0,50-0,52); Ende: Heck hinter der Wand
    assert start[0] + 0.037 < 0.5 and end[0] - 0.101 > 0.52 and abs(axis[1]) < 0.2
    assert abs(start[1]) < 0.02 and abs(end[1]) < 0.02
    assert abs(width - 0.20) < 0.03

    verdict, _ = pg.check_path(straight, gap_grid(0.075), RES, ORIGIN, (0.0, 0.0))
    assert verdict == pg.BLOCK, verdict


def line_and_block(gap):
    """Linie (Wand x 0,00-0,02) ueber die ganze Hoehe, rechts daneben ein Hindernis 6 x 6 cm bei
    y 0,47-0,53 mit gap (m) Abstand zur Linie. Weg 6 cm neben der Linie, parallel: der Umriss
    beruehrt die Linie schon ab dem Start (wie ein NavFn-Weg, der an Linien nur 3 cm haelt)."""
    grid = wall_x(empty_grid(), 0.0)
    c0 = int(round((0.02 + gap - ORIGIN[0]) / RES))
    r0 = int(round((0.47 - ORIGIN[1]) / RES))
    grid[r0:r0 + 6, c0:c0 + 6] = rg.LETHAL_COST
    x = 0.08
    return grid, np.column_stack([np.full(100, x), np.arange(0.0, 1.0, 0.01)]), (x, 0.0)


def test_check_path_along_line():
    # Erste Beruehrung nur an der Linie: weiterpruefen bis zur Engstelle Linie-Hindernis
    # (vorher "frei" an der ersten Beruehrung, die Luecke wurde nie gesperrt).
    grid, path, robot = line_and_block(0.14)
    assert pg.check_path(path, grid, RES, ORIGIN, robot)[0] == pg.BLOCK
    grid, path, robot = line_and_block(0.22)
    assert pg.check_path(path, grid, RES, ORIGIN, robot)[0] == pg.ALIGN


def test_first_contact():
    grid = wall_x(empty_grid(), 0.0)               # Linie allein, Weg parallel
    along = np.arange(0.0, 1.0, 0.01)
    close = np.column_stack([np.full(100, 0.06), along])      # Umriss 8,9 cm > 4 cm Abstand
    far = np.column_stack([np.full(100, 0.12), along])
    hit = pg.first_contact(close, grid, RES, ORIGIN, (0.06, 0.0))
    assert hit is not None and hit[0][1] > pg.CHECK_SKIP_M - 0.01
    assert pg.first_contact(far, grid, RES, ORIGIN, (0.12, 0.0)) is None
    # Eine Linie allein ist fuer die Pfadpruefung keine Engstelle: "frei" trotz Kontakt --
    # genau den Fall verwirft nav2_goal_publisher._touches_one_side beim Tor-Weg.
    assert pg.check_path(close, grid, RES, ORIGIN, (0.06, 0.0))[0] == pg.FREE


def test_direction_rules():
    robot = (0.0, 0.0)
    ahead = np.column_stack([np.arange(0.0, 1.0, 0.01), np.zeros(100)])
    left = np.column_stack([np.zeros(100), np.arange(0.0, 1.0, 0.01)])
    assert pg.same_direction(ahead, ahead, robot)
    assert not pg.same_direction(ahead, left, robot)
    cut = pg.truncate_at(ahead, (0.505, 0.02))
    assert np.allclose(cut[-1], [0.505, 0.02]) and len(cut) == 52


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'ok  {name}')
