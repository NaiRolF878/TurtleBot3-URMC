#!/bin/bash
# Aufruf: keine Argumente
# Startet Spurverfolgung, Zonen-Ausweichen (avoid_node) und Nav2 fuer den Tunnel im
# Hintergrund und den NOT-AUS (notaus) im Vordergrund dieses Terminals. Nur
# Spurverfolgung ohne Ausweichen und Nav2: follow_lane.sh
#
# Fahrbefehle erzeugt nur control_driving: je nach /control/mode PID + Offset (Lane),
# das Ausweichmanoever aus /avoid/zones (Obstacle) oder die Befehle von Nav2
# (/nav2/cmd_vel, TunnelEntry/Tunnel), sonst Null. Er sendet auf /control/cmd_vel;
# erst der Not-Aus reicht an /cmd_vel durch.
#
# Der Bot faehrt NICHT von selbst: erst LEERTASTE in diesem Terminal gibt die Fahrt frei,
# nochmal Leertaste = STOPP. q oder Strg+C beendet den Not-Aus (sendet Stopp) und danach
# alles andere.

# Laeuft schon eine Pipeline? Zwei gleichzeitig senden auf dieselben Topics, und ein
# zweiter Not-Aus im STOPP schickt dauernd Null an den Bot.
PATTERN='lib/Follow_Lane/(ingress_node(_nav2)?|detect_lane(_nav2)?|control_driving|switch_control|debug_view|notaus|avoid_node|lane_costmap_node|nav2_goal_publisher|keepout_zone_node|tunnel_explorer_node|nav2_view)( |$)|lib/nav2_(controller|planner|lifecycle_manager)/'
if pgrep -f "$PATTERN" >/dev/null; then
    echo "Es laeuft bereits eine Pipeline:"
    ps -eo pid,etime,args | grep -E "$PATTERN" | grep -v grep | sed 's#/[^ ]*/lib/[^/]*/##; s# --ros-args.*##'
    echo ""
    read -r -p "Beenden und neu starten? [j/N] " answer
    if [ "$answer" != "j" ] && [ "$answer" != "J" ]; then
        echo "Abgebrochen -- die laufende Pipeline bleibt aktiv."
        exit 1
    fi
    # Zuerst den Not-Aus (sendet beim Beenden Stopp), dann die Launch-Prozesse und Knoten
    pkill -INT -f 'lib/Follow_Lane/notaus( |$)'
    sleep 1
    pkill -INT -f 'ros2 launch Follow_Lane (follow_lane|nav2_obstacle).launch.py'
    sleep 3
    pkill -TERM -f "$PATTERN"
    sleep 1
    if pgrep -f "$PATTERN" >/dev/null; then
        echo "Konnte nicht alles beenden -- bitte pruefen: ps -ef | grep -E 'Follow_Lane|nav2'"
        exit 1
    fi
    echo "Alte Pipeline beendet."
    echo ""
fi

# Ausgaben aller Knoten: pro Start eine eigene Datei in logs/ (logs/ ist in .gitignore).
REPO_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
mkdir -p "$REPO_ROOT/logs"
NODE_LOG="$REPO_ROOT/logs/nav2_obstacle-$(date +%Y%m%d-%H%M%S).log"
ln -sfn "$(basename "$NODE_LOG")" "$REPO_ROOT/logs/nav2_obstacle-latest.log"

# set -m: eigene Prozessgruppe fuer den Hintergrundjob, sonst ignoriert er SIGINT und laesst
# sich nicht mehr sauber beenden (Begruendung ausfuehrlich in follow_lane.sh).
set -m
ros2 launch Follow_Lane nav2_obstacle.launch.py > "$NODE_LOG" 2>&1 &
LAUNCH_PID=$!
set +m
cleanup() {
    stty sane 2>/dev/null
    echo ""
    echo "Beende Pipeline ..."
    kill -INT -- -"$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 1 15); do
        kill -0 "$LAUNCH_PID" 2>/dev/null || break
        sleep 1
    done
    kill -TERM -- -"$LAUNCH_PID" 2>/dev/null
    wait "$LAUNCH_PID" 2>/dev/null
    echo "Pipeline beendet."
}
trap cleanup EXIT

echo "Pipeline startet im Hintergrund (Log der Knoten: logs/$(basename "$NODE_LOG"))."
echo ""
echo "Tunnel von Hand: im Fenster rqt_image_view einmal \"Publish click location\""
echo "anhaken, dann im Debugbild auf TUNNEL STARTEN klicken."
echo "Das zweite Bildfenster zeigt die Nav2-Draufsicht mit Statuszeile (/nav2/view)."
echo ""
echo "Not-Aus: LEERTASTE = Fahren/Stopp, q = beenden."
echo ""
ros2 run Follow_Lane notaus
