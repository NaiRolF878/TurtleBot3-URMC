#!/bin/bash
# Aufruf: [--label NAME] [--keep-res] [--undistorted] [--no-window]
# Bilder fuer einen Vision-Datensatz aufnehmen: Livebild, p = Foto speichern, q = beenden.
# Ablage: dataset/<label>/...
#
# Stellt die Kamera fuer die Aufnahme auf hohe Aufloesung mit vollem Bildwinkel (cam-mode.sh full:
# Cam 2 1600x1200, Cam 3 1920x1080, je 5 Hz)
# und beim Beenden wieder auf 800x600 / 30 Hz zurueck (cam-mode.sh lane).
#   --keep-res     Kamera nicht umstellen, mit der laufenden Aufloesung aufnehmen
#   --undistorted  entzerrtes Bild (startet ingress_node). Die Kalibrierung gilt nur
#                  fuer 800x600, deshalb wird dann ebenfalls NICHT umgestellt.
#
# Startet bewusst NICHT die Follow-Lane-Pipeline (die faehrt sofort los). Den Bot zum
# Positionieren per Hand schieben oder teleop in einem zweiten Terminal benutzen.
#
# Voraussetzung: Bot laeuft (botup), Kamera liefert /camera/image_raw/compressed.

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# Auch bei direktem Aufruf (ohne tb/launch.sh) die Domain-ID des aktiven Bots nehmen,
# sonst sieht der Laptop dessen Kamera nicht.
source "$SCRIPT_DIR/bot.conf" || exit 1
export ROS_DOMAIN_ID=$BOT_DOMAIN_ID

SWITCH=1
UNDIST=0
PY_ARGS=()
for a in "$@"; do
    case "$a" in
        --keep-res)    SWITCH=0 ;;
        --undistorted) SWITCH=0; UNDIST=1; PY_ARGS+=("$a") ;;
        *)             PY_ARGS+=("$a") ;;
    esac
done

PIDS=()
SWITCHED=0
cleanup() {
    for p in "${PIDS[@]}"; do
        pkill -P "$p" 2>/dev/null
        kill "$p" 2>/dev/null
    done
    wait 2>/dev/null
    if [ "$SWITCHED" = 1 ]; then
        SWITCHED=0
        echo ""
        bash "$SCRIPT_DIR/cam-mode.sh" lane
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

if [ "$SWITCH" = 1 ]; then
    SWITCHED=1
    bash "$SCRIPT_DIR/cam-mode.sh" full || exit 1
    echo ""
fi

if [ "$UNDIST" = 1 ]; then
    echo "Starte ingress_node (entzerrt das Kamerabild -> /lokal/raw_image) ..."
    ros2 run Follow_Lane ingress_node &
    PIDS+=($!)
    sleep 3
fi

# Im Vordergrund, damit p/q auch im Terminal ankommen.
python3 "$SCRIPT_DIR/dataset-capture.py" "${PY_ARGS[@]}"
