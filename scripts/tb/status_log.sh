#!/bin/bash
# Hilfsskript, kein Launcher: wird von launch.sh im Hintergrund mitgestartet,
# wenn beim Start eines Launchers "mitloggen" bejaht wird.
# Direkt aufrufbar ist es trotzdem: bash scripts/tb/status_log.sh
#
# Loggt alle 10s den Status der wichtigsten Follow_Lane-Topics/Nodes nach logs/.
# Zweck: Wenn Claude gerade im Bot-WLAN nicht erreichbar ist, laeuft das Logging
# trotzdem weiter -- die Datei kann man spaeter einfach reinposten.
#
# Gemessen wird mit "ros2 topic echo --qos-reliability best_effort" statt "ros2 topic hz":
# echo liefert eine Nachrichten-ANZAHL pro Zeitfenster. hz braucht mehrere Nachrichten,
# um ueberhaupt eine Rate zu bilden, und bleibt bei sehr langsamen Topics (< 1 Nachricht
# pro Fenster) stumm -- ununterscheidbar von "gar keine Daten". Die Anzahl unterscheidet
# genau das: 0 = nichts, 1 = kommt an, aber sehr langsam.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="$REPO_ROOT/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/tb-status-$(date +%Y%m%d-%H%M%S).log"

# ACHTUNG: /camera/image_raw gehoert hier bewusst NICHT rein.
# "ros2 topic echo" legt einen echten DDS-Subscriber an -- das Rohbild
# (800x600x3 = 1.44 MB/Frame, bei 30 fps ~345 Mbit/s) wuerde dadurch alle 10 s
# fuer 3 s ueber das WLAN gezogen. Der Pi schafft real 30-80 Mbit/s, das Netz
# ist damit um Faktor 5-10 ueberbucht und der komprimierte Stream, auf dem
# Follow_Lane arbeitet, verhungert. --no-arr unterdrueckt nur die AUSGABE,
# nicht den Transport. Genau so hat sich dieses Logging-Skript in der Sitzung
# vom 12.08.2026 selbst das Problem gebaut, das es messen sollte.
# Ob die Kamera roh liefert, gehoert lokal auf den Bot gemessen: botcheck.
TOPICS=(
    /camera/image_raw/compressed
    /lokal/raw_image
    /lokal/debug_view
    /detect/lane_debug
    /scan
    /lokal/scan
    /detect/lane
    /control/cmd_vel
    /control/mode
)

MEASURE_SECS=3

echo "[log] Schreibe nach: $LOG_FILE"

trap 'exit 0' INT TERM

# Wer uns gestartet hat (launch.sh). Signale allein reichen nicht: stirbt der
# Starter hart (kill -9) oder haengt sein Trap hinter einem Vordergrund-Kommando,
# liefe der Logger sonst als Waise weiter. Deshalb pro Runde nachsehen, ob es ihn
# noch gibt. PPID 1 = direkt gestartet, dann gibt es nichts zu ueberwachen.
PARENT_PID=$PPID

# IP-Reassemblierung: NUR als Differenz pro Intervall auswerten. Die Werte in
# /proc/net/snmp sind seit dem Booten kumuliert -- eine hohe Summe sagt nichts
# darueber, ob es GERADE klemmt, und vermischt laengst behobene Phasen mit dem
# aktuellen Zustand.
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
