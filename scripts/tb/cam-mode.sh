#!/bin/bash
# Startet NUR den Kameraknoten auf dem Bot neu, mit anderer Aufloesung. Bringup bleibt an.
# Aufruf: bash cam-mode.sh lane|full
#
# Werte fuer die Cam 2 kommen aus camera.conf (gemeinsam mit bot-up.sh), der Start selbst
# aus _camera-start.sh. Die Kamera wird auf dem Bot automatisch erkannt:
#
#   Modus  Cam 2 (IMX219, nativ 4:3)            Cam 3 (IMX708, nativ 16:9)
#   lane   camera.conf CAM_*      (800x600)     800x600, 30 Hz  (nur 50 % Bildwinkel!)
#   full   camera.conf CAM_FULL_* (1600x1200)   1920x1080, q90, 5 Hz
#
# full = voller Bildwinkel. Das Seitenverhaeltnis muss zum Sensor passen, sonst waehlt
# libcamera einen Sensormodus, der die Mitte ausschneidet: bei der IMX219 jede 16:9-
# Groesse, bei der IMX708 alles unter 1920x1080 (Messungen in docs/MEMORY.md).
# Sensormodus und fester Weissabgleich aus camera.conf gelten nur fuer die IMX219.
#
# Warum full nur 5 Hz: ein Bild ist als JPEG q90 ~250-300 KB. Bei 30 Hz waeren das
# ~60-70 Mbit/s, die Funkstrecke schafft ~25-35 Mbit/s; 5 Hz ~ 10-12 Mbit/s.
#
# Warum full RELIABLE statt best_effort: das DDS-Profil zerlegt jedes Bild in 1400-Byte-
# Stuecke, ~200 pro Bild. Bei best_effort ist das ganze Bild verloren, sobald EIN Stueck
# fehlt -- im WLAN kaeme kaum ein Bild an. Reliable sendet fehlende Stuecke nach; bei
# 5 Hz staut sich dabei nichts. Der Leser muss dann ebenfalls reliable abonnieren
# (dataset-capture.py erkennt das selbst).

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1
source "$SCRIPT_DIR/camera.conf" || exit 1

case "$1" in
    lane|full) MODE="$1" ;;
    *) echo "Aufruf: bash cam-mode.sh lane|full"; exit 1 ;;
esac

BOT_IP=$(bash "$SCRIPT_DIR/find-bot-ip.sh") || exit 1

REMOTE=$(cat <<EOF
eval "\$(grep -E '^[[:space:]]*export[[:space:]]+(ROS_DOMAIN_ID|RMW_IMPLEMENTATION)=' "\$HOME/.bashrc" 2>/dev/null)"
export ROS_DOMAIN_ID="\${ROS_DOMAIN_ID:-$BOT_DOMAIN_ID}"
[ -f "\$HOME/fastdds_profile.xml" ] && export FASTRTPS_DEFAULT_PROFILES_FILE="\$HOME/fastdds_profile.xml"
source /opt/ros/humble/setup.bash
[ -f "\$HOME/camera_ws/install/setup.bash" ] && source "\$HOME/camera_ws/install/setup.bash"

# Sensor erkennen: ein gebundenes Geraet erscheint als z.B. 10-001a im Treiberverzeichnis.
SENSOR=unbekannt
for s in imx708 imx219; do
    ls "/sys/bus/i2c/drivers/\$s" 2>/dev/null | grep -q '^[0-9]*-00' && SENSOR=\$s && break
done
echo "  Sensor: \$SENSOR"
[ "\$SENSOR" = unbekannt ] && echo "  WARNUNG: Sensor nicht erkannt, nehme die Werte der IMX219."

case "$MODE-\$SENSOR" in
    lane-imx708)
        CAM_W=800;  CAM_H=600;  CAM_Q=80; CAM_FRAME_US=33333;  CAM_REL=best_effort
        CAM_SENSOR_MODE=""; CAM_COLOUR_GAINS=""
        echo "  WARNUNG: 800x600 ist bei der IMX708 ein Crop (nur 50 % Bildwinkel)." ;;
    full-imx708)
        CAM_W=1920; CAM_H=1080; CAM_Q=90; CAM_FRAME_US=200000; CAM_REL=reliable
        CAM_SENSOR_MODE=""; CAM_COLOUR_GAINS="" ;;
    lane-*)
        CAM_W=$CAM_W; CAM_H=$CAM_H; CAM_Q=$CAM_Q; CAM_FRAME_US=$CAM_FRAME_US; CAM_REL=best_effort
        CAM_SENSOR_MODE='$CAM_SENSOR_MODE'; CAM_COLOUR_GAINS='$CAM_COLOUR_GAINS' ;;
    full-*)
        CAM_W=$CAM_FULL_W; CAM_H=$CAM_FULL_H; CAM_Q=$CAM_FULL_Q; CAM_FRAME_US=$CAM_FULL_FRAME_US; CAM_REL=reliable
        CAM_SENSOR_MODE='$CAM_SENSOR_MODE'; CAM_COLOUR_GAINS='$CAM_COLOUR_GAINS' ;;
esac
EOF
)
REMOTE="$REMOTE
$(cat "$SCRIPT_DIR/_camera-start.sh")
pgrep -f camera_node >/dev/null"

echo "=== Kamera auf '$MODE' umstellen ($BOT_IP) ==="
ssh "$BOT_USER@$BOT_IP" "bash -s" <<< "$REMOTE"
