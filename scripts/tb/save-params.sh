#!/bin/bash
# Speichert die Laufzeit-Parameter eines Nodes in die Parameter-Datei und baut neu.
#
# Aufruf: bash save-params.sh <node> [datei.yaml]
#   bash save-params.sh detect_lane
#   bash save-params.sh detect_lane Follow_Lane_ROI_PI_CAM3.yaml
#
# Schreibt FLACHES YAML (name: wert) -- dasselbe Format, das rqt_reconfigure benutzt.
# Das Launch-File liest es selbst ein (siehe load_flat_params dort), statt ueber
# --params-file zu gehen. Dadurch gibt es genau ein Format und die Datei-Knoepfe in
# rqt bleiben verwendbar. "ros2 param dump" liefert verschachtelt, wird hier umgebaut.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PARAM_DIR="$REPO_ROOT/workspace/src/Follow_Lane/Follow_Lane/Parameter"

NODE="$1"
if [ -z "$NODE" ]; then
    echo "Aufruf: bash save-params.sh <node> [datei.yaml]"
    echo "Bekannte Nodes: detect_lane, control_driving"
    exit 1
fi

# Standarddatei je Node, damit der haeufige Fall ohne zweites Argument auskommt.
FILE="$2"
if [ -z "$FILE" ]; then
    case "$NODE" in
        detect_lane)     FILE="Follow_Lane_ROI_PI_CAM2.yaml" ;;
        control_driving) FILE="Control_driving_PID.yaml" ;;
        *) echo "Fuer '$NODE' ist keine Standarddatei hinterlegt, bitte als 2. Argument angeben."; exit 1 ;;
    esac
fi

TARGET="$PARAM_DIR/$FILE"

TMP=$(mktemp) || exit 1
trap 'rm -f "$TMP"' EXIT

if ! ros2 param dump "/$NODE" > "$TMP" 2>/dev/null; then
    echo "Konnte /$NODE nicht abfragen. Laeuft die Pipeline?"
    exit 1
fi

# Gegenprobe, bevor eine funktionierende Datei ueberschrieben wird.
if ! grep -q "ros__parameters" "$TMP"; then
    echo "Dump hat nicht das erwartete Format, nichts geschrieben:"
    head -3 "$TMP"
    exit 1
fi

python3 - "$TMP" "$TARGET" <<'PY' || exit 1
import sys, yaml
src, dst = sys.argv[1], sys.argv[2]
with open(src) as f:
    data = yaml.safe_load(f) or {}
flat = next(iter(data.values()))['ros__parameters']
flat.pop('use_sim_time', None)          # gehoert nicht in eine Streckenkonfiguration
if not flat:
    print('Keine Parameter im Dump, nichts geschrieben.'); sys.exit(1)
with open(dst, 'w') as f:
    yaml.dump(flat, f, default_flow_style=False, sort_keys=True)
print(f'{len(flat)} Parameter geschrieben.')
PY

echo "Gespeichert: $TARGET"

# Dank --symlink-install zeigt install/ auf src/ -- bestehende Dateien wirken sofort.
# Der Build ist nur noetig, wenn die Datei NEU ist und der Symlink noch fehlt.
if [ ! -e "$REPO_ROOT/workspace/install/Follow_Lane/share/Follow_Lane/Parameter/$FILE" ]; then
    echo "Neue Datei -- baue einmalig, damit der Symlink entsteht..."
    ( cd "$REPO_ROOT/workspace" && colcon build --symlink-install --packages-select Follow_Lane ) | tail -2
fi

echo "Fertig. Wirkt beim naechsten Start der Pipeline."
