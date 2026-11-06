#!/bin/bash
# Aufruf: keine Argumente
# Startet die komplette Follow-Lane-Pipeline (ingress_node, detect_lane, control_driving, switch_control,
# debug_view) im Hintergrund und den NOT-AUS (notaus) im Vordergrund dieses Terminals.
#
# Der Bot faehrt NICHT von selbst: erst LEERTASTE in diesem Terminal gibt die Fahrt frei,
# nochmal Leertaste = STOPP. Alternativ rqt_reconfigure -> Node notaus -> Haekchen "drive".
# q oder Strg+C beendet den Not-Aus (sendet Stopp) und danach die ganze Pipeline.

# Laeuft schon eine Pipeline? Zwei senden auf dieselben Topics, ein zweiter Not-Aus im STOPP
# schickt dauernd Null an den Bot.
PATTERN='lib/Follow_Lane/(ingress_node(_nav2)?|detect_lane(_nav2)?|control_driving|switch_control|debug_view|notaus|odom_filter|lane_costmap_node|nav2_goal_publisher|keepout_zone_node|tunnel_explorer_node|nav2_view)( |$)|lib/nav2_(controller|planner|lifecycle_manager)/'
if pgrep -f "$PATTERN" >/dev/null; then
    echo "Es laeuft bereits eine Follow-Lane-Pipeline:"
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

# Ausgaben aller Knoten: je Start eine Datei in logs/, follow_lane-latest.log zeigt auf die letzte.
REPO_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
mkdir -p "$REPO_ROOT/logs"
NODE_LOG="$REPO_ROOT/logs/follow_lane-$(date +%Y%m%d-%H%M%S).log"
ln -sfn "$(basename "$NODE_LOG")" "$REPO_ROOT/logs/follow_lane-latest.log"

# set -m: eigene Prozessgruppe, sonst ignoriert der Hintergrundjob SIGINT (Bash-Regel fuer '&'
# ohne Job-Control). Strg+C trifft so nur den Not-Aus, cleanup() beendet danach die Pipeline.
set -m
ros2 launch Follow_Lane follow_lane.launch.py > "$NODE_LOG" 2>&1 &
LAUNCH_PID=$!
set +m
cleanup() {
    stty sane 2>/dev/null          # Sicherheitsnetz, falls der Not-Aus das Terminal nicht zurueckgestellt hat
    echo ""
    echo "Beende Pipeline ..."
    # SIGINT an die ganze Prozessgruppe: ros2 launch leitet es nicht an die Knoten weiter.
    kill -INT -- -"$LAUNCH_PID" 2>/dev/null
    # notfalls nachhelfen
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
echo "Not-Aus: LEERTASTE = Fahren/Stopp, q = beenden."
echo ""
ros2 run Follow_Lane notaus
