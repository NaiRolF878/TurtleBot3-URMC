#!/bin/bash
# Laedt eine Parameter-Datei in einen laufenden Node (ohne Neustart der Pipeline).
#
# Aufruf: bash load-params.sh <node> [datei.yaml]
#   bash load-params.sh detect_lane
#   bash load-params.sh detect_lane Follow_Lane_ROI_PI_CAM3.yaml
#
# Gegenstueck zu save-params.sh. Die Dateien liegen flach vor (name: wert);
# "ros2 param load" verlangt aber verschachtelt, deshalb wird hier temporaer umgebaut.
# Alternativ geht auch der Laden-Knopf in rqt_reconfigure -- der versteht flach direkt.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PARAM_DIR="$REPO_ROOT/workspace/src/Follow_Lane/Follow_Lane/Parameter"

NODE="$1"
if [ -z "$NODE" ]; then
    echo "Aufruf: bash load-params.sh <node> [datei.yaml]"
    echo "Bekannte Nodes: detect_lane, control_driving"
    echo ""
    echo "Vorhandene Dateien:"
    ls -1 "$PARAM_DIR"/*.yaml 2>/dev/null | sed 's|.*/|  |'
    exit 1
fi

FILE="$2"
if [ -z "$FILE" ]; then
    case "$NODE" in
        detect_lane)     FILE="Follow_Lane_ROI_PI_CAM2.yaml" ;;
        control_driving) FILE="Control_driving_PID.yaml" ;;
        *) echo "Fuer '$NODE' ist keine Standarddatei hinterlegt, bitte als 2. Argument angeben."; exit 1 ;;
    esac
fi

TARGET="$PARAM_DIR/$FILE"

if [ ! -f "$TARGET" ]; then
    echo "Datei nicht gefunden: $TARGET"
    exit 1
fi

TMP=$(mktemp --suffix=.yaml) || exit 1
trap 'rm -f "$TMP"' EXIT

python3 - "$TARGET" "$TMP" "$NODE" <<'PY' || exit 1
import sys, yaml
src, dst, node = sys.argv[1], sys.argv[2], sys.argv[3]
with open(src) as f:
    data = yaml.safe_load(f) or {}
# Schon verschachtelt? Dann unveraendert durchreichen.
if len(data) == 1 and isinstance(next(iter(data.values())), dict) \
        and 'ros__parameters' in next(iter(data.values())):
    out = data
else:
    out = {f'/{node}': {'ros__parameters': data}}
with open(dst, 'w') as f:
    yaml.dump(out, f, default_flow_style=False, sort_keys=True)
PY

echo "Lade $FILE in /$NODE ..."
ros2 param load "/$NODE" "$TMP" || exit 1
echo "Fertig."
