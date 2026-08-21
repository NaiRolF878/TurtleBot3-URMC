#!/bin/bash
# Speichert die Laufzeit-Parameter eines Nodes in die Parameter-Datei und baut neu.
#
# Aufruf: bash save-params.sh <node> [datei.yaml]
#   bash save-params.sh detect_lane
#   bash save-params.sh detect_lane Follow_Lane_ROI_PI_CAM3.yaml
#
# Hintergrund: NICHT den Speichern-Knopf in rqt_reconfigure benutzen -- der schreibt
# flaches YAML ohne Nodenamen. "--params-file" im Launch braucht aber die Struktur
# "<node>: ros__parameters:", sonst stirbt der Node beim Start mit
# "Cannot have a value before ros__parameters at line 1".
# "ros2 param dump" erzeugt das richtige Format, deshalb dieser Weg.

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

# use_sim_time gehoert nicht in die Streckenkonfiguration und erzeugt beim Laden Warnungen.
grep -v "use_sim_time" "$TMP" > "$TARGET"

echo "Gespeichert: $TARGET"
echo "Baue neu (data_files kopiert die YAMLs, ohne Build laedt der naechste Start die alte Fassung)..."
( cd "$REPO_ROOT/workspace" && colcon build --symlink-install --packages-select Follow_Lane ) \
    | tail -2

echo "Fertig."
