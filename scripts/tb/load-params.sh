#!/bin/bash
# Laedt Parameter-Dateien in einen laufenden Node (ohne Neustart der Pipeline).
#
# Aufruf: bash load-params.sh <node> [datei.yaml]
#   bash load-params.sh detect_lane          -> Follow_Lane_ROI_LAB.yaml + Lab_Farben.yaml
#   bash load-params.sh lab Lab_Halle.yaml   -> nur einen LAB-Farbsatz in detect_lane
#   bash load-params.sh control_driving      -> Control_driving_PID_Wide.yaml
#
# Gegenstueck zu save-params.sh. Die Dateien liegen flach vor (name: wert);
# "ros2 param load" verlangt aber verschachtelt, deshalb wird hier temporaer umgebaut.
# Alternativ geht auch der Laden-Knopf in rqt_reconfigure -- der versteht flach direkt.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PARAM_DIR="$REPO_ROOT/workspace/src/Follow_Lane/Follow_Lane/Parameter/follow_lane"

NODE="$1"
if [ -z "$NODE" ]; then
    echo "Aufruf: bash load-params.sh <node> [datei.yaml]"
    echo "Bekannte Nodes: detect_lane, lab (nur LAB-Werte von detect_lane), control_driving"
    echo ""
    echo "Vorhandene Dateien:"
    ls -1 "$PARAM_DIR"/*.yaml 2>/dev/null | sed 's|.*/|  |'
    exit 1
fi

case "$NODE" in
    detect_lane)     ROS_NODE=detect_lane;     FILES="${2:-Follow_Lane_ROI_LAB.yaml Lab_Farben.yaml}" ;;
    lab)             ROS_NODE=detect_lane;     FILES="${2:-Lab_Farben.yaml}" ;;
    control_driving) ROS_NODE=control_driving; FILES="${2:-Control_driving_PID_Wide.yaml}" ;;
    *)
        if [ -z "$2" ]; then
            echo "Fuer '$NODE' ist keine Standarddatei hinterlegt, bitte als 2. Argument angeben."; exit 1
        fi
        ROS_NODE="$NODE"; FILES="$2" ;;
esac

TMP=$(mktemp --suffix=.yaml) || exit 1
trap 'rm -f "$TMP"' EXIT

for FILE in $FILES; do
    TARGET="$PARAM_DIR/$FILE"
    if [ ! -f "$TARGET" ]; then
        echo "Datei nicht gefunden: $TARGET"
        exit 1
    fi

    python3 - "$TARGET" "$TMP" "$ROS_NODE" <<'PY' || exit 1
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

    echo "Lade $FILE in /$ROS_NODE ..."
    ros2 param load "/$ROS_NODE" "$TMP" || exit 1
done
echo "Fertig."
