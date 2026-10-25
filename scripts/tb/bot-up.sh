#!/bin/bash
# Faehrt den Bot komplett hoch: Uhr stellen, Reste aufraeumen, Bringup + Kamera starten.
# Aufruf: bash bot-up.sh [--no-time]
#         --no-time  ueberspringt die Zeitsynchronisation (die fragt nach dem sudo-Passwort)
#
# Die Prozesse laufen per setsid/nohup losgeloest weiter -- bricht die SSH-Verbindung
# weg (WLAN!), sterben sie nicht mit. Logs landen auf dem Bot in ~/logs/.
# Beenden mit bot-down.sh.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1

# Kameraeinstellungen (Aufloesung, Sensormodus, Weissabgleich) stehen in camera.conf.
source "$SCRIPT_DIR/camera.conf" || exit 1

# Klang am Ende (Dienst /sound, fest in der OpenCR-Firmware -- keine eigenen Melodien):
# 0 OFF, 1 ON, 2 LOW_BATTERY, 3 ERROR. Leer = kein Klang.
# 4 BUTTON1 / 5 BUTTON2 werden angenommen (success=True), spielen bei uns aber NICHTS (getestet).
SOUND_OK=1
SOUND_FAIL=3

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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-__DOMAIN_ID__}"
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

__CAMERA_START__

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

# Kamerastart aus _camera-start.sh einsetzen, die Werte aus camera.conf davor.
# Ersatztext in Anfuehrungszeichen: sonst wuerde bash >= 5.2 jedes & darin als
# "gefundener Text" interpretieren (patsub_replacement).
CAMERA_START="CAM_W=$CAM_W CAM_H=$CAM_H CAM_Q=$CAM_Q CAM_FRAME_US=$CAM_FRAME_US CAM_REL=best_effort
CAM_SENSOR_MODE='$CAM_SENSOR_MODE' CAM_COLOUR_GAINS='$CAM_COLOUR_GAINS' CAM_ORIENTATION='$CAM_ORIENTATION' CAM_TUNING='$CAM_TUNING'
$(cat "$SCRIPT_DIR/_camera-start.sh")"
REMOTE="${REMOTE/__CAMERA_START__/"$CAMERA_START"}"
REMOTE=${REMOTE//__DOMAIN_ID__/$BOT_DOMAIN_ID}
# Tuning-Datei mitschicken (Details in camera.conf)
if [ -n "$CAM_TUNING" ]; then
    ssh "$BOT_USER@$BOT_IP" "mkdir -p ~/tuning" && \
        scp -q "$SCRIPT_DIR/tuning/$CAM_TUNING" "$BOT_USER@$BOT_IP:tuning/$CAM_TUNING" || \
        echo "  WARNUNG: Tuning-Datei $CAM_TUNING konnte nicht auf den Bot kopiert werden."
fi
ssh "$BOT_USER@$BOT_IP" "bash -s" <<< "$REMOTE" || exit 1

echo ""
echo "=== Gegenprobe vom Laptop ==="
# Der Daemon cacht nach Bot-Neustart oder IP-Wechsel falsch -> hier bewusst --no-daemon.
source /opt/ros/humble/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=$BOT_DOMAIN_ID
# Bis zu 3 Versuche: nach dem Start (und mit WLAN + Kabel gleichzeitig) tauchen nicht alle
# Topics sofort auf -- einmal pruefen meldete sonst Fehler, obwohl kurz darauf alles da war.
EXPECTED="/scan /odom /battery_state /camera/image_raw/compressed"
for attempt in 1 2 3; do
    sleep 3
    TOPICS=$(timeout 25 ros2 topic list --no-daemon 2>/dev/null)
    MISSING=""
    for t in $EXPECTED; do
        grep -qx "$t" <<< "$TOPICS" || MISSING="$MISSING $t"
    done
    FOUND=$(( 4 - $(wc -w <<< "$MISSING") ))
    [ -z "$MISSING" ] && break
    [ "$attempt" -lt 3 ] && echo "  Versuch $attempt: $FOUND von 4, fehlt:$MISSING -- warte kurz ..."
done
echo "  $FOUND von 4 erwarteten Topics sichtbar (scan, odom, battery_state, camera compressed)"
if [ -n "$MISSING" ]; then
    echo "  FEHLT:$MISSING"
    echo "  Logs auf dem Bot pruefen:"
    case "$MISSING" in *camera*) echo "    botssh 'tail -30 ~/logs/camera.log'    (Kamera)";; esac
    case "$MISSING" in *scan*|*odom*|*battery*) echo "    botssh 'tail -30 ~/logs/bringup.log'   (LiDAR / OpenCR)";; esac
fi

# Hoerbare Rueckmeldung: alles da -> SOUND_OK, sonst SOUND_FAIL.
SOUND=$([ "$FOUND" -ge 4 ] && echo "$SOUND_OK" || echo "$SOUND_FAIL")
if [ -n "$SOUND" ]; then
    # turtlebot3_msgs (Sound.srv) liegt im turtlebot3_ws, nicht in /opt/ros.
    [ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash" >/dev/null 2>&1
    timeout 10 ros2 service call /sound turtlebot3_msgs/srv/Sound "{value: $SOUND}" >/dev/null 2>&1 \
        || echo "  (Klang konnte nicht abgespielt werden -- /sound nicht erreichbar)"
fi

echo ""
echo "Fertig. Beenden mit: bash scripts/tb/bot-down.sh"
