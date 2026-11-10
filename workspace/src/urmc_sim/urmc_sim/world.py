"""Welt des 2D-Simulators: Quader und Zylinder auf dem Boden, LiDAR-Strahlen und Kollision.

Ohne ROS, damit sich die Geometrie auch ohne Simulator pruefen laesst. Weltdatei (YAML):

    start: [x, y, yaw_deg]                   # Radachse, Blickrichtung
    objects:
      - {type: box, center: [x, y], size: [lx, ly], yaw_deg: 0, height: 0.25}
      - {type: cylinder, center: [x, y], radius: 0.03, height: 0.25}

Masse in Metern, x nach rechts, y nach oben (Draufsicht).
"""
import math

import numpy as np
import yaml


class Box:
    def __init__(self, center, size, yaw_deg, height):
        self.center = np.asarray(center, dtype=np.float64)
        self.size = np.asarray(size, dtype=np.float64)
        self.yaw = math.radians(yaw_deg)
        self.height = float(height)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hx, hy = self.size / 2.0
        local = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
        self.corners = local @ np.array([[c, s], [-s, c]]) + self.center


class Cylinder:
    def __init__(self, center, radius, height):
        self.center = np.asarray(center, dtype=np.float64)
        self.radius = float(radius)
        self.height = float(height)


class World:
    def __init__(self, start, objects):
        self.start = (float(start[0]), float(start[1]), math.radians(float(start[2])))
        self.boxes = [o for o in objects if isinstance(o, Box)]
        self.cylinders = [o for o in objects if isinstance(o, Cylinder)]

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
        objects = []
        for o in data.get('objects') or []:
            if o['type'] == 'box':
                objects.append(Box(o['center'], o['size'], o.get('yaw_deg', 0.0), o['height']))
            elif o['type'] == 'cylinder':
                objects.append(Cylinder(o['center'], o['radius'], o['height']))
            else:
                raise ValueError(f'Unbekannter Objekttyp: {o["type"]}')
        return cls(data['start'], objects)

    def bounds(self):
        """(xmin, ymin, xmax, ymax) aller Objekte und des Starts."""
        pts = [np.array(self.start[:2])]
        pts += [b.corners for b in self.boxes]
        pts += [np.array([c.center - c.radius, c.center + c.radius]) for c in self.cylinders]
        allp = np.vstack(pts)
        return (*allp.min(axis=0), *allp.max(axis=0))

    def raycast(self, origin, angles, max_range, min_height):
        """Entfernung je Strahl bis zum ersten Objekt, das hoeher als min_height ist; inf ohne
        Treffer innerhalb max_range."""
        o = np.asarray(origin, dtype=np.float64)
        d = np.column_stack([np.cos(angles), np.sin(angles)])
        best = np.full(len(angles), np.inf)

        boxes = [b for b in self.boxes if b.height > min_height]
        if boxes:
            p = np.vstack([b.corners for b in boxes])
            q = np.vstack([np.roll(b.corners, -1, axis=0) for b in boxes])
            e = q - p
            w = p - o
            denom = d[:, 0:1] * e[None, :, 1] - d[:, 1:2] * e[None, :, 0]
            with np.errstate(divide='ignore', invalid='ignore'):
                t = (w[None, :, 0] * e[None, :, 1] - w[None, :, 1] * e[None, :, 0]) / denom
                u = (w[None, :, 0] * d[:, 1:2] - w[None, :, 1] * d[:, 0:1]) / denom
            hit = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
            best = np.minimum(best, np.where(hit, t, np.inf).min(axis=1))

        for c in self.cylinders:
            if c.height <= min_height:
                continue
            oc = o - c.center
            b = d @ oc
            disc = b * b - (oc @ oc - c.radius ** 2)
            with np.errstate(invalid='ignore'):
                t = -b - np.sqrt(disc)
            best = np.minimum(best, np.where((disc >= 0.0) & (t > 0.0), t, np.inf))

        return np.where(best <= max_range, best, np.inf)

    def collides(self, polygon):
        """Beruehrt das konvexe Polygon (N, 2, Welt) ein Objekt? Hoehe zaehlt hier nicht:
        der Bot stoesst auch an niedrige Objekte."""
        poly = np.asarray(polygon, dtype=np.float64)
        if any(_convex_overlap(poly, b.corners) for b in self.boxes):
            return True
        return any(_circle_overlap(poly, c.center, c.radius) for c in self.cylinders)


def _axes(poly):
    e = np.roll(poly, -1, axis=0) - poly
    n = np.column_stack([-e[:, 1], e[:, 0]])
    return n / np.linalg.norm(n, axis=1, keepdims=True)


def _convex_overlap(a, b):
    """Trennachsen-Test fuer zwei konvexe Polygone."""
    for axis in np.vstack([_axes(a), _axes(b)]):
        pa, pb = a @ axis, b @ axis
        if pa.max() < pb.min() or pb.max() < pa.min():
            return False
    return True


def _circle_overlap(poly, center, radius):
    p = poly
    q = np.roll(poly, -1, axis=0)
    e = q - p
    t = np.clip(((center - p) * e).sum(axis=1) / (e * e).sum(axis=1), 0.0, 1.0)
    nearest = p + e * t[:, None]
    if np.min(np.linalg.norm(nearest - center, axis=1)) <= radius:
        return True
    # Mittelpunkt im Polygon (konvex, gleiche Drehrichtung aller Kanten)?
    cross = e[:, 0] * (center[1] - p[:, 1]) - e[:, 1] * (center[0] - p[:, 0])
    return bool(np.all(cross >= 0.0) or np.all(cross <= 0.0))
