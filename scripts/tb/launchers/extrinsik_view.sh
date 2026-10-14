#!/bin/bash
# Aufruf: keine Argumente (optional: zusaetzliche Argumente fuer extrinsics-view.py)
# Sichtpruefung der extrinsischen Kalibrierung: Bodengitter mit cm-Massen im Kamerabild.
#
# Startet NUR ingress_node + die View -- bewusst NICHT die Follow-Lane-Pipeline.
# Grund: control_driving hat keinen Startknopf (min_speed_factor = 0.5), die Pipeline
# zu starten heisst losfahren. Fuer die Kalibrierung wird nur das Kamerabild gebraucht.
#
# Voraussetzung: Bot laeuft (botup), Kamera liefert /camera/image_raw/compressed.

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"

PIDS=()
cleanup() {
    echo ""
    echo "Beende ingress_node und extrinsics_view ..."
    for p in "${PIDS[@]}"; do
        pkill -P "$p" 2>/dev/null
        kill "$p" 2>/dev/null
    done
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "Starte ingress_node (entzerrt das Kamerabild -> /lokal/raw_image) ..."
ros2 run Follow_Lane ingress_node &
PIDS+=($!)

# ingress_node braucht einen Moment, bis er die Kalibrierung geladen hat und publiziert.
sleep 3

echo "Starte extrinsics_view -> /lokal/extrinsics_view ..."
python3 "$SCRIPT_DIR/extrinsics-view.py" "$@" &
PIDS+=($!)

sleep 2

echo ""
echo "Gitter 10 cm, Beschriftung in cm ab Radachse. Gruen = geradeaus (Y=0)."
echo "Liegt das ChArUco-Brett im Bild, wird seine Lage live gemessen und gegen die"
echo "gespeicherten Werte gestellt."
echo ""
echo "Werte live nachziehen (zweites Terminal):"
echo "  ros2 run rqt_reconfigure rqt_reconfigure     # Node extrinsics_view waehlen"
echo "  ros2 param set /extrinsics_view pitch_deg 21.5"
echo ""
echo "Strg+C beendet alles."
echo ""

# rqt im Vordergrund: schliesst man es, raeumt der trap den Rest ab.
ros2 run rqt_image_view rqt_image_view /lokal/extrinsics_view
