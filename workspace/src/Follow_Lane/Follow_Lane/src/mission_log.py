"""Fahrtprotokoll (Tagebuch) fuer Tunnel und Obstacle, ohne ROS-Abhaengigkeit.

Ein Verzeichnis je Fahrt: <kind>.jsonl (eine JSON-Zeile je Eintrag, z.B. events, poses, scans),
Kartenbilder als PNG. Schreibfehler werden geschluckt -- das Protokoll darf die Fahrt nie
stoeren.
"""
import json
import math
import os
import time

import cv2


class MissionLog:
    """Fahrtprotokoll: events/poses/scans.jsonl, Kartenbilder, map.png am Ende."""

    def __init__(self, path, logger):
        self.dir = None
        self.files = {}
        try:
            os.makedirs(path, exist_ok=True)
            self.dir = path
            logger.info(f'Fahrtprotokoll: {path}')
        except OSError as e:
            logger.warn(f'Kein Fahrtprotokoll ({e}).')

    def write(self, kind, record):
        if self.dir is None:
            return
        try:
            f = self.files.get(kind)
            if f is None:
                f = self.files[kind] = open(os.path.join(self.dir, f'{kind}.jsonl'), 'a',
                                            buffering=1)
            f.write(json.dumps(record, default=float) + '\n')
        except OSError:
            pass

    def picture(self, name, image):
        """Bild unter <name>.png ablegen; Rueckgabe: Dateiname oder None."""
        if self.dir is None or image is None:
            return None
        filename = f'{name}.png'
        try:
            cv2.imwrite(os.path.join(self.dir, filename), image)
        except (OSError, cv2.error):
            return None
        return filename

    def close(self, picture):
        if self.dir is None:
            return
        self.picture('map', picture)
        for f in self.files.values():
            f.close()
        self.files = {}


def scan_record(scan):
    """LaserScan als Protokolleintrag (Entfernungen in mm; -1: kein Echo, 0: ungueltig)."""
    return {
        't': time.monotonic(),
        'stamp': scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9,
        'angle_min': scan.angle_min, 'angle_increment': scan.angle_increment,
        'range_min': scan.range_min,
        'mm': [int(r * 1000) if math.isfinite(r) else (-1 if r > 0 else 0)
               for r in scan.ranges]}
