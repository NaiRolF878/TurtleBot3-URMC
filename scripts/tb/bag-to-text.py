#!/usr/bin/env python3
"""Macht eine ros2-bag-Aufzeichnung lesbar: eine Zeile pro Nachricht (Zeit, Topic, Inhalt).

Aufruf:
    python3 scripts/tb/bag-to-text.py logs/bag-20261022-132414
    python3 scripts/tb/bag-to-text.py logs/bag-...  --topics /detect/lane /cmd_vel
-> schreibt logs/bag-.../kommunikation.txt (und gibt den Pfad aus)

Bilddaten (raw_image, data-Felder) werden weggelassen, Zahlen gekuerzt.
Uhrzeit in UTC wie die Lognamen (Container laeuft auf UTC).
"""
import argparse
import datetime as dt
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def fmt(v, depth=0):
    """Nachricht kompakt als Text -- Bilder/Byte-Felder ausgelassen."""
    if hasattr(v, 'get_fields_and_field_types'):
        parts = []
        for name in v.get_fields_and_field_types():
            if name in ('header',):
                continue
            val = getattr(v, name)
            if name == 'data' and hasattr(val, '__len__') and len(val) > 16:
                parts.append(f'{name}=<{len(val)} Byte>')
                continue
            if name in ('raw_image', 'image') or (hasattr(val, 'format') and hasattr(val, 'data')):
                parts.append(f'{name}=<Bild>')
                continue
            parts.append(f'{name}={fmt(val, depth + 1)}')
        return '{' + ', '.join(parts) + '}' if depth else ', '.join(parts)
    if isinstance(v, float):
        return f'{v:.4g}'
    if isinstance(v, (list, tuple)) or (hasattr(v, '__len__') and not isinstance(v, str)):
        n = len(v)
        if n == 0:
            return '[]'
        if n > 6:
            return f'[{n} Eintraege]'
        return '[' + ', '.join(fmt(x, depth + 1) for x in v) + ']'
    return str(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='Ordner der Aufzeichnung (mit metadata.yaml)')
    ap.add_argument('--topics', nargs='*', default=None, help='nur diese Topics')
    ap.add_argument('--out', default=None, help='Zieldatei (Default: <bag>/kommunikation.txt)')
    args = ap.parse_args()

    if not os.path.isfile(os.path.join(args.bag, 'metadata.yaml')):
        print(f'{args.bag}: keine metadata.yaml -- laeuft die Aufzeichnung noch? (erst mit Strg+C beenden)')
        return 1

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=args.topics))

    out = args.out or os.path.join(args.bag, 'kommunikation.txt')
    width = max(len(t) for t in types)
    n = 0
    t0 = None
    with open(out, 'w') as f:
        f.write(f'# {args.bag}\n# Zeit (UTC)     +s seit Start  Topic  Inhalt\n')
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            msg = deserialize_message(data, get_message(types[topic]))
            t0 = t0 if t0 is not None else t_ns
            clock = dt.datetime.utcfromtimestamp(t_ns / 1e9).strftime('%H:%M:%S.%f')[:-3]
            f.write(f'{clock}  {(t_ns - t0) / 1e9:7.3f}  {topic:<{width}}  {fmt(msg)}\n')
            n += 1
    print(f'{n} Nachrichten -> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
