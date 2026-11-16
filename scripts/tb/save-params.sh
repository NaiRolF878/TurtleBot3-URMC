#!/bin/bash
# Speichert die Laufzeit-Parameter eines Nodes (z. B. nach Tunen in rqt) in die Parameter-Datei(en).
#
# Aufruf: bash save-params.sh <node> [datei.yaml] [lab-datei.yaml]
#   bash save-params.sh detect_lane          -> lab_* nach Lab_Farben.yaml, Rest nach Follow_Lane_ROI_LAB.yaml
#   bash save-params.sh lab Lab_Halle.yaml   -> NUR die lab_*-Werte von detect_lane (eigener Farbsatz)
#   bash save-params.sh control_driving      -> Control_driving_PID_Wide.yaml
#
# Statt des Speichern-Knopfs in rqt benutzen: der schreibt alle Parameter eines Nodes in EINE Datei
# und kennt die Aufteilung detect_lane = Rest + LAB nicht.
#
# Bestehende Dateien werden zeilenweise aktualisiert: Kommentare und Reihenfolge bleiben, neue
# Parameter kommen ans Ende. Format flach (name: wert), wie es follow_lane.launch.py und rqt lesen.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PARAM_DIR="$REPO_ROOT/workspace/src/Follow_Lane/Follow_Lane/Parameter"

NODE="$1"
if [ -z "$NODE" ]; then
    echo "Aufruf: bash save-params.sh <node> [datei.yaml] [lab-datei.yaml]"
    echo "Bekannte Nodes: detect_lane, lab (nur LAB-Werte von detect_lane), control_driving"
    exit 1
fi

# MODE: all = alles in REST, split = lab_* nach LAB und Rest nach REST, lab = nur lab_* nach LAB
case "$NODE" in
    detect_lane)
        MODE=split; ROS_NODE=detect_lane
        REST="${2:-Follow_Lane_ROI_LAB.yaml}"; LAB="${3:-Lab_Farben.yaml}" ;;
    lab)
        MODE=lab; ROS_NODE=detect_lane; REST=""; LAB="${2:-Lab_Farben.yaml}" ;;
    control_driving)
        MODE=all; ROS_NODE=control_driving; REST="${2:-Control_driving_PID_Wide.yaml}"; LAB="" ;;
    *)
        if [ -z "$2" ]; then
            echo "Fuer '$NODE' ist keine Standarddatei hinterlegt, bitte als 2. Argument angeben."; exit 1
        fi
        MODE=all; ROS_NODE="$NODE"; REST="$2"; LAB="" ;;
esac

TMP=$(mktemp) || exit 1
trap 'rm -f "$TMP"' EXIT

if ! ros2 param dump "/$ROS_NODE" > "$TMP" 2>/dev/null; then
    echo "Konnte /$ROS_NODE nicht abfragen. Laeuft die Pipeline?"
    exit 1
fi

# Gegenprobe, bevor eine funktionierende Datei ueberschrieben wird.
if ! grep -q "ros__parameters" "$TMP"; then
    echo "Dump hat nicht das erwartete Format, nichts geschrieben:"
    head -3 "$TMP"
    exit 1
fi

python3 - "$TMP" "$MODE" "${REST:+$PARAM_DIR/$REST}" "${LAB:+$PARAM_DIR/$LAB}" <<'PY' || exit 1
import os, re, sys, yaml
src, mode, rest_path, lab_path = sys.argv[1:5]
with open(src) as f:
    data = yaml.safe_load(f) or {}
params = next(iter(data.values()))['ros__parameters']
params.pop('use_sim_time', None)          # gehoert nicht in eine Streckenkonfiguration
if not params:
    print('Keine Parameter im Dump, nichts geschrieben.'); sys.exit(1)

def line(key, value):
    return yaml.safe_dump({key: value}, default_flow_style=False).strip()

def write(path, values):
    """Werte zeilenweise ersetzen (Kommentare bleiben), neue Schluessel sortiert anhaengen."""
    lines = open(path).read().splitlines() if os.path.isfile(path) else []
    done = set()
    for i, text in enumerate(lines):
        m = re.match(r'^([A-Za-z_][A-Za-z0-9_]*)\s*:', text)
        if m and m.group(1) in values:
            lines[i] = line(m.group(1), values[m.group(1)])
            done.add(m.group(1))
    lines += [line(k, values[k]) for k in sorted(values) if k not in done]
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'{len(values):3d} Parameter -> {path}')

lab = {k: v for k, v in params.items() if k.startswith('lab_')}
rest = {k: v for k, v in params.items() if not k.startswith('lab_')}
if mode == 'all':
    write(rest_path, params)
if mode == 'split':
    write(rest_path, rest)
if mode in ('split', 'lab'):
    if not lab:
        print('Keine lab_*-Parameter im Dump.'); sys.exit(1)
    write(lab_path, lab)
PY

# Dank --symlink-install zeigt install/ auf src/ -- bestehende Dateien wirken sofort.
# Nur eine NEUE Datei braucht einmal einen Build, damit der Symlink entsteht.
for FILE in $REST $LAB; do
    if [ ! -e "$REPO_ROOT/workspace/install/Follow_Lane/share/Follow_Lane/Parameter/$FILE" ]; then
        echo "Neue Datei $FILE -- baue einmalig, damit der Symlink entsteht..."
        ( cd "$REPO_ROOT/workspace" && colcon build --symlink-install --packages-select Follow_Lane ) | tail -2
        break
    fi
done

echo "Fertig. Wirkt beim naechsten Start der Pipeline."
