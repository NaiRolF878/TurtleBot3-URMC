#!/bin/bash
# Misst AUF DEM BOT, ob die Kamera ueberhaupt Bilder produziert -- ohne WLAN dazwischen.
# Aufruf: bash bot-check.sh   oder per Alias: botcheck
#
# Sinn: Alle Messungen vom Laptop aus vermischen "Kamera liefert nicht" mit
# "Netz transportiert nicht". Lokal auf dem Bot faellt das Netz als Faktor weg.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BOT_USER="gertrude"
export ROS_DOMAIN_ID=33

BOT_IP=$(bash "$SCRIPT_DIR/find-bot-ip.sh") || exit 1

read -r -d '' REMOTE_CMD <<'REMOTE'
source /opt/ros/humble/setup.bash 2>/dev/null

echo "=== Sucht in ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} ==="
echo "  (wird vom Container durchgereicht -- muss zu dem passen, in dem die"
echo "   Bot-Prozesse laufen, sonst sieht man grundsaetzlich nichts)"
echo

# Das Profil steht in der .bashrc des Bots -- die bricht bei NICHT-interaktiven Shells
# aber sofort ab ("If not running interactively, don't do anything"), und genau so eine
# ist "bash -l -s" hier. Ohne die naechste Zeile meldete botcheck faelschlich
# "<nicht gesetzt>", obwohl die echten Nodes das Profil sehr wohl geladen haben --
# und maass zusaetzlich mit anderen Transporten als die Nodes selbst.
if [ -z "$FASTRTPS_DEFAULT_PROFILES_FILE" ] && [ -f "$HOME/fastdds_profile.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_profile.xml"
    PROFIL_HINWEIS=" (aus \$HOME nachgeladen, .bashrc greift hier nicht)"
fi
echo "=== Kamera-Prozess laeuft? ==="
if pgrep -fa camera_node | grep -v pgrep; then
    :
else
    echo "  KEIN camera_node!"
    echo
    echo "  >>> ABBRUCH: Auf dem Bot laeuft gerade keine Kamera."
    echo "  >>> Alles Weitere waere zwangslaeufig 0 und sagt nichts aus."
    echo "  >>> Erst auf dem Bot starten:"
    echo "  >>>   ros2 launch turtlebot3_bringup robot.launch.py"
    echo "  >>>   ros2 run camera_ros camera_node --ros-args -p format:=RGB888"
    echo "  >>> und DANN botcheck nochmal laufen lassen."
    exit 1
fi
echo
echo "=== Sichtbare Kamera-Topics ==="
# --no-daemon: der Daemon cacht seine Sicht und ist direkt nach einem "daemon stop"
# oder Domainwechsel noch leer -- er meldete "keine", waehrend "topic hz" gleichzeitig
# 30 Hz mass. Ohne Daemon macht der Aufruf seine eigene Discovery und sagt die Wahrheit.
ros2 topic list --no-daemon 2>/dev/null | grep -i camera || echo "  keine"
echo
echo "=== Bildrate roh (8s, lokal auf dem Bot) ==="
timeout 8 ros2 topic hz /camera/image_raw 2>&1 | tail -3
echo
echo "=== Bildrate komprimiert (8s) ==="
timeout 8 ros2 topic hz /camera/image_raw/compressed 2>&1 | tail -3
echo
echo "=== LIDAR (8s) ==="
timeout 8 ros2 topic hz /scan 2>&1 | tail -3
echo
echo "=== WLAN-Energiesparmodus (an = Aussetzer) ==="
iwconfig wlan0 2>/dev/null | grep -i "power management" || echo "  nicht ermittelbar"
echo
echo "=== DDS-Profil auf dem Bot aktiv? ==="
echo "  FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<nicht gesetzt>}${PROFIL_HINWEIS}"
if [ -f "${FASTRTPS_DEFAULT_PROFILES_FILE:-/nonexistent}" ]; then
    if grep -q "SHM" "$FASTRTPS_DEFAULT_PROFILES_FILE"; then
        echo "  Stand: AKTUELL (enthaelt SHM-Transport)"
    else
        echo "  Stand: VERALTET -- ohne SHM-Transport."
        echo "         Im Container neu deployen: bash scripts/tb/deploy-dds-profile.sh"
    fi
fi
REMOTE

# Optionen muessen VOR den Hostnamen: alles danach wertet ssh als Remote-Befehl.
# -t erzwingt ein TTY, damit die Messungen nicht gepuffert erst am Ende erscheinen.
#
# ROS_DOMAIN_ID wird explizit vorangestellt: "bash -l -s" ist NICHT interaktiv, und
# Ubuntus .bashrc steigt oben mit "If not running interactively, don't do anything"
# aus -- alles was dort exportiert wird (ROS_DOMAIN_ID, FASTRTPS_...) fehlt also.
# Ohne das suchte botcheck in Domain 0, waehrend Kamera und Bringup in Domain 30
# liefen: es meldete "keine Topics", obwohl auf dem Bot alles sauber lief.
ssh -t "$BOT_USER@$BOT_IP" "bash -l -s" <<< "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
$REMOTE_CMD"
