#!/bin/bash
# Faehrt den Bot komplett hoch: Uhr stellen, Reste aufraeumen, Bringup + Kamera starten.
# Aufruf: bash bot-up.sh [--no-time]
#         --no-time  ueberspringt die Zeitsynchronisation (die fragt nach dem sudo-Passwort)
#
# Die Prozesse laufen per setsid/nohup losgeloest weiter -- bricht die SSH-Verbindung
# weg (WLAN!), sterben sie nicht mit. Logs landen auf dem Bot in ~/logs/.
# Beenden mit bot-down.sh.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BOT_USER="gertrude"

# Kameraeinstellungen. 4:3 beibehalten -- beim IMX219 (Cam 2) bleibt damit der volle
# Bildwinkel erhalten, 16:9 landet im beschnittenen Sensormodus. Details in docs/MEMORY.md.
CAM_W=800
CAM_H=600
CAM_Q=80
# Bilddauer in Mikrosekunden, min und max gleich = feste Rate. 33333 us = 30 Hz.
# Ohne diesen Wert waehlt libcamera selbst und laesst die Rate je nach Belichtung sinken.
CAM_FRAME_US=33333

BOT_IP=$(bash "$SCRIPT_DIR/find-bot-ip.sh") || exit 1

if [ "$1" != "--no-time" ]; then
    echo ""
    echo "=== Uhr stellen (Pi hat keine gepufferte Echtzeituhr) ==="
    bash "$SCRIPT_DIR/sync-time.sh" || echo "  Zeitsynchronisation fehlgeschlagen, mache trotzdem weiter."
fi

REMOTE=$(cat <<'EOF'
# Ubuntus .bashrc steigt bei nicht-interaktiven Shells sofort aus -- alles, was dort
# exportiert wird, fehlt hier. Deshalb die relevanten Exports von dort herausziehen...
eval "$(grep -E '^[[:space:]]*export[[:space:]]+(TURTLEBOT3_MODEL|LDS_MODEL|ROS_DOMAIN_ID|RMW_IMPLEMENTATION)=' "$HOME/.bashrc" 2>/dev/null)"

# ...und fuer alles, was dort nicht steht, Standardwerte setzen.
# TURTLEBOT3_MODEL fehlte hier zuerst -- turtlebot3_bringup startet ohne die Variable nicht.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
[ -f "$HOME/fastdds_profile.xml" ] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_profile.xml"
echo "  Umgebung: TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL LDS_MODEL=${LDS_MODEL:-(nicht gesetzt)} ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

source /opt/ros/humble/setup.bash
[ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
# camera_ws liefert camera_ros gegen libcamera 0.5.2; ohne das laeuft die apt-Version mit 0.1.0.
[ -f "$HOME/camera_ws/install/setup.bash" ] && source "$HOME/camera_ws/install/setup.bash"

mkdir -p "$HOME/logs"

echo "--- Reste beenden ---"
# Verwaiste camera_node-Prozesse halten die Kamera belegt ("failed to acquire camera").
pkill -9 -f camera_node          2>/dev/null && echo "  alte camera_node beendet"
pkill -f turtlebot3_bringup      2>/dev/null && echo "  altes Bringup beendet"
pkill -f robot.launch.py         2>/dev/null
sleep 2

# Der OpenCR springt nach Reset oder Akkuwechsel zwischen ttyACM0 und ttyACM1.
PORT=""
for p in /dev/ttyACM0 /dev/ttyACM1; do
    [ -e "$p" ] && PORT="$p" && break
done
if [ -z "$PORT" ]; then
    echo "  FEHLER: kein /dev/ttyACM* gefunden -- OpenCR nicht erreichbar (Strom? Kabel?)"
    exit 1
fi
echo "  OpenCR auf $PORT"

if [ ! -e /dev/tb3_lidar ]; then
    echo "  WARNUNG: /dev/tb3_lidar fehlt -- der LIDAR wird kein /scan liefern."
fi

echo "--- Bringup starten ---"
setsid nohup ros2 launch turtlebot3_bringup robot.launch.py usb_port:="$PORT" \
    > "$HOME/logs/bringup.log" 2>&1 < /dev/null &
sleep 6

echo "--- Kamera starten ---"
# QoS-Overrides: der Publisher stand sonst auf RELIABLE. Fuer einen Livestrom ist das
# falsch -- DDS haelt verlorene Samples vor und sendet sie erneut, bei Paketverlust
# staut sich der Sender und liefert alte Bilder nach. BEST_EFFORT + Tiefe 1 heisst:
# immer nur das neueste Bild, verlorene werden verworfen statt nachgereicht.
setsid nohup ros2 run camera_ros camera_node --ros-args \
    -p format:=RGB888 -p width:=__CAM_W__ -p height:=__CAM_H__ -p jpeg_quality:=__CAM_Q__ \
    -p FrameDurationLimits:="[__CAM_FRAME_US__,__CAM_FRAME_US__]" \
    -p qos_overrides./camera/image_raw/compressed.publisher.reliability:=best_effort \
    -p qos_overrides./camera/image_raw/compressed.publisher.history:=keep_last \
    -p qos_overrides./camera/image_raw/compressed.publisher.depth:=1 \
    > "$HOME/logs/camera.log" 2>&1 < /dev/null &
sleep 4

echo "--- laufende Prozesse ---"
# Bei Fehlschlag gleich das Log zeigen, statt nur darauf zu verweisen.
if pgrep -f robot.launch.py >/dev/null; then
    echo "  Bringup: laeuft"
else
    echo "  Bringup: NICHT gestartet -- letzte Zeilen aus ~/logs/bringup.log:"
    tail -15 "$HOME/logs/bringup.log" 2>/dev/null | sed 's/^/    /'
fi
if pgrep -f camera_node >/dev/null; then
    echo "  Kamera:  laeuft"
else
    echo "  Kamera:  NICHT gestartet -- letzte Zeilen aus ~/logs/camera.log:"
    tail -15 "$HOME/logs/camera.log" 2>/dev/null | sed 's/^/    /'
fi
EOF
)

REMOTE=${REMOTE//__CAM_W__/$CAM_W}
REMOTE=${REMOTE//__CAM_H__/$CAM_H}
REMOTE=${REMOTE//__CAM_Q__/$CAM_Q}
REMOTE=${REMOTE//__CAM_FRAME_US__/$CAM_FRAME_US}

echo ""
echo "=== Bot hochfahren ($BOT_IP) ==="
ssh "$BOT_USER@$BOT_IP" "bash -s" <<< "$REMOTE" || exit 1

echo ""
echo "=== Gegenprobe vom Laptop ==="
# Der Daemon cacht nach Bot-Neustart oder IP-Wechsel falsch -> hier bewusst --no-daemon.
source /opt/ros/humble/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=33
sleep 3
FOUND=$(timeout 25 ros2 topic list --no-daemon 2>/dev/null | grep -cE "^/(scan|odom|battery_state|camera/image_raw/compressed)$")
echo "  $FOUND von 4 erwarteten Topics sichtbar (scan, odom, battery_state, camera compressed)"
if [ "$FOUND" -lt 4 ]; then
    echo "  Fehlt etwas, Logs auf dem Bot pruefen:"
    echo "    botssh 'tail -30 ~/logs/bringup.log'"
    echo "    botssh 'tail -30 ~/logs/camera.log'"
fi

echo ""
echo "Fertig. Beenden mit: bash scripts/tb/bot-down.sh"
