#!/bin/bash
# Hilfsskript, kein Launcher: wird von launch.sh im Hintergrund mitgestartet,
# wenn beim Start eines Launchers "mitloggen" bejaht wird.
# Direkt aufrufbar ist es trotzdem: bash scripts/tb/status_log.sh
#
# Loggt alle 10s den Status der wichtigsten Follow_Lane-Topics/Nodes nach logs/status/.
# Zweck: Wenn Claude gerade im Bot-WLAN nicht erreichbar ist, laeuft das Logging
# trotzdem weiter -- die Datei kann man spaeter einfach reinposten.
#
# "ros2 topic echo" statt "hz": die Anzahl je Fenster unterscheidet 0 (nichts) von 1 (sehr
# langsam), hz bleibt in beiden Faellen stumm.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="$REPO_ROOT/logs/status"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/tb-status-$(date +%Y%m%d-%H%M%S).log"

# Nicht /camera/image_raw: echo abonniert wirklich (~345 Mbit/s), das ueberlastet das WLAN.
# Ob die Kamera roh liefert, misst botcheck auf dem Bot.
TOPICS=(
    /camera/image_raw/compressed
    /lokal/raw_image
    /lokal/debug_view
    /detect/lane_debug
    /scan
    /detect/lane
    /nav2/cmd_vel
    /control/cmd_vel
    /cmd_vel
    /control/mode
)

MEASURE_SECS=3

echo "[log] Schreibe nach: $LOG_FILE"

trap 'exit 0' INT TERM

# Jede Runde pruefen, ob der Starter (launch.sh) noch lebt, sonst laeuft der Logger als Waise
# weiter. PPID 1 = direkt gestartet.
PARENT_PID=$PPID

# IP-Reassemblierung als Differenz je Intervall: /proc/net/snmp zaehlt seit dem Booten.
read_reasm() {
    grep -E "^Ip:" -A1 /proc/net/snmp | awk '
        NR==1{for(i=1;i<=NF;i++)h[i]=$i}
        NR==2{for(i=1;i<=NF;i++){if(h[i]=="ReasmOKs")ok=$i; if(h[i]=="ReasmFails")f=$i} print ok" "f}'
}
prev=($(read_reasm))

while true; do
    if [ "$PARENT_PID" -gt 1 ] && ! kill -0 "$PARENT_PID" 2>/dev/null; then
        exit 0
    fi
    {
        echo "===== $(date '+%Y-%m-%d %H:%M:%S') ====="
        echo "--- hostname -I ---"
        hostname -I
        echo "--- ros2 node list ---"
        timeout 5 ros2 node list
        echo "--- doppelte rqt-Instanzen? (jede zieht den Bildstream separat) ---"
        pgrep -fa rqt_image_view | grep -v pgrep || echo "  keine"
        echo "--- Nachrichten in ${MEASURE_SECS}s (BEST_EFFORT-Subscriber) ---"
        for t in "${TOPICS[@]}"; do
            # --no-arr verhindert, dass Bilddaten megabyteweise ins Log laufen.
            # Jede Nachricht endet mit einer '---'-Zeile -> zaehlbar.
            n=$(timeout "$MEASURE_SECS" ros2 topic echo --qos-reliability best_effort \
                    --no-arr "$t" 2>/dev/null | grep -c -- '^---$')
            printf "  %-32s %s\n" "$t" "$n"
        done
        cur=($(read_reasm))
        d_ok=$(( ${cur[0]} - ${prev[0]} ))
        d_fail=$(( ${cur[1]} - ${prev[1]} ))
        prev=("${cur[@]}")
        echo "--- IP-Fragmente seit letztem Snapshot (Differenz, nicht kumulativ) ---"
        echo "  reassembliert OK : $d_ok"
        echo "  fehlgeschlagen   : $d_fail"
        echo ""
    } >> "$LOG_FILE" 2>&1
    # sleep im Hintergrund + wait: so reagiert bash sofort auf ein Signal,
    # statt erst nach Ablauf des sleep.
    sleep 10 &
    wait $!
done
