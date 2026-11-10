"""Fasst einen Ordner von measure-load.sh zusammen (Aufruf dort, oder: python3 measure-summary.py <ordner>)."""
import collections
import os
import re
import sys

out = sys.argv[1]
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                   'workspace', 'src', 'Follow_Lane', 'Follow_Lane', 'src')
OURS = {f for f in os.listdir(SRC) if f.endswith('.py')} if os.path.isdir(SRC) else set()


def read(name):
    path = os.path.join(out, name)
    return open(path, errors='ignore').read() if os.path.isfile(path) else ''


print(f'=== Messung {os.path.basename(out)}')

# CPU je Prozess (zweiter top-Block = Mittel ueber die Messdauer), 100 % = ein Kern.
block = read('top.txt').split('top - ')[-1]
rows = []
for line in block.splitlines():
    p = line.split(None, 11)
    if len(p) < 12 or not p[0].isdigit():
        continue
    cpu, cmd = float(p[8].replace(',', '.')), p[11]
    m = (re.search(r'lib/Follow_Lane/(\w+)', cmd) or re.search(r'lib/nav2_\w+/(\w+)', cmd)
         or re.search(r'(rviz2|rqt_image_view|ros2 launch|gzserver|code)', cmd))
    if m and cpu >= 1.0:
        rows.append((cpu, m.group(1)))
print('\n--- Laptop: CPU je Prozess (100 % = 1 Kern, 16 Kerne)')
for cpu, name in sorted(rows, reverse=True):
    print(f'  {name:28s} {cpu:6.1f} %')
print(f'  {"Summe":28s} {sum(c for c, _ in rows):6.1f} %')
m = re.search(r'%Cpu\(s\):.*', block)
if m:
    print('  ' + m.group(0))

threads = read('threads_detect_lane.txt').split('top - ')[-1]
trows = [(float(p[8].replace(',', '.')), p[11]) for p in (l.split(None, 11) for l in threads.splitlines())
         if len(p) >= 12 and p[0].isdigit() and float(p[8].replace(',', '.')) >= 1.0]
if trows:
    print('\n--- detect_lane: CPU je Thread')
    for cpu, name in sorted(trows, reverse=True)[:8]:
        print(f'  {name:28s} {cpu:6.1f} %')

gpu = [l.split(',') for l in read('gpu.csv').splitlines() if l.strip()]
if gpu:
    util = [float(g[0].strip().rstrip(' %')) for g in gpu]
    print(f'\n--- GPU: Auslastung Mittel {sum(util) / len(util):.0f} %, max {max(util):.0f} %, '
          f'Speicher {gpu[-1][2].strip()}')

print('\n--- Laptop: empfangen (Mbit/s)\n' + read('net_laptop.txt').rstrip())
print('\n--- Bot\n' + (read('bot.txt').rstrip() or '  (keine Bot-Messung)'))
print('\n--- Pipeline-Log (Zeit je Stufe)\n' + read('pipeline_zeiten.txt').rstrip())

# py-spy (raw: "frame;frame;... anzahl"): Anteil der Proben, in denen eine Funktion auf dem
# Stapel liegt (gesamt) bzw. zuoberst (selbst). Nur Funktionen aus unserem Code (Follow_Lane)
# und die grossen Bibliotheksaufrufe darunter.
for name in sorted(f for f in os.listdir(out) if f.startswith('pyspy_')):
    total = 0
    incl = collections.Counter()
    self_ = collections.Counter()
    for line in read(name).splitlines():
        stack, _, count = line.rpartition(' ')
        if not count.isdigit():
            continue
        n = int(count)
        total += n
        frames = [re.sub(r' \((?:.*/)?([^/]+?):\d+\)', r' (\1)', f) for f in stack.split(';')]
        for f in set(frames):
            incl[f] += n
        self_[frames[-1]] += n
    if not total:
        continue
    idle = sum(n for f, n in self_.items() if 'spin_once' in f or 'wait' in f.lower() or 'select' in f)
    print(f'\n--- {name[6:-4]}: {total} Proben, davon ~{100 * idle / total:.0f} % Warten')
    frame = ('spin_until ', 'run_node ', 'main ', 'spin_once ')   # immer auf dem Stapel
    ours = [(n, f) for f, n in incl.items()
            if f.rsplit('(', 1)[-1].rstrip(')') in OURS and not f.startswith(frame)]
    for n, f in sorted(ours, reverse=True)[:12]:
        print(f'  {100 * n / total:5.1f} %  {f}')
