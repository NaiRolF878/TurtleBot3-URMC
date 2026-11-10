#!/bin/bash
# Misst Last und Zeiten der LAUFENDEN Pipeline auf Laptop und Bot, gleichzeitig ueber DAUER s.
# Aufruf: bash scripts/tb/measure-load.sh <name> [dauer_s]     (z.B. lane_mit_gui 20)
# Ergebnis: logs/messung/<datum>_<name>/  (Rohdaten + zusammenfassung.txt)
#
# Laptop: CPU je Prozess und je Thread (detect_lane), GPU, empfangene Daten je Schnittstelle,
#         py-spy (Zeit je Funktion der Python-Nodes, --nonblocking: haelt die Nodes nicht an),
#         Zeit-Zeilen aus dem Pipeline-Log.
# Bot (SSH): CPU je Prozess, gesendete Daten je Schnittstelle, Kamera-Rate am Pi, Temperatur.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1
NAME="${1:?Name der Messung angeben, z.B. lane_mit_gui}"
DUR="${2:-20}"
OUT="$REPO_ROOT/logs/messung/$(date +%Y%m%d-%H%M%S)_$NAME"
mkdir -p "$OUT"
NODES="detect_lane_nav2 ingress_node_nav2 debug_view lane_costmap_node intersection_node switch_control control_driving nav2_view"

echo "Messe $DUR s -> $OUT"
command -v py-spy >/dev/null || echo "  py-spy fehlt (pip install py-spy) -- keine Zeit je Funktion."

# --- Bot ---
BOT_IP="${BOT_LAN_IP:-}"
ping -c1 -W1 "$BOT_IP" >/dev/null 2>&1 || BOT_IP=$(bash "$SCRIPT_DIR/find-bot-ip.sh" 2>/dev/null)
if [ -n "$BOT_IP" ] && ssh -o BatchMode=yes -o ConnectTimeout=3 "$BOT_USER@$BOT_IP" true 2>/dev/null; then
    ssh -o BatchMode=yes "$BOT_USER@$BOT_IP" "DUR=$DUR DOMAIN=$BOT_DOMAIN_ID bash -s" > "$OUT/bot.txt" 2>&1 <<'REMOTE' &
source /opt/ros/humble/setup.bash 2>/dev/null
[ -f ~/camera_ws/install/setup.bash ] && source ~/camera_ws/install/setup.bash
[ -f ~/fastdds_profile.xml ] && export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_profile.xml
export ROS_DOMAIN_ID=$DOMAIN
r0=$(cat /proc/net/dev)
top -b -d "$DUR" -n 2 -w 200 > /tmp/m_top.txt &
timeout $((DUR + 2)) ros2 topic hz /camera/image_raw/compressed --window 1000 > /tmp/m_hz.txt 2>&1 &
sleep "$DUR"; r1=$(cat /proc/net/dev); wait
echo "== gesendet (Mbit/s)"
paste <(echo "$r0" | tail -n +3) <(echo "$r1" | tail -n +3) | tr ':' ' ' |
    awk -v d="$DUR" '{x=($27-$10)*8/d/1e6; if (x>0.05) printf "  %-10s %6.2f\n", $1, x}'
echo "== Kamera-Rate am Pi"; grep -E "average rate|min:" /tmp/m_hz.txt | tail -2
echo "== CPU je Prozess (%)"
awk '/^top -/{n++} n==2' /tmp/m_top.txt | awk 'NR>7 && $9+0>0.5 {printf "  %6s  %s\n", $9, $12}' | head -12
grep "Cpu(s)" /tmp/m_top.txt | tail -1
echo "== Temperatur (mC) / Drosselung"; cat /sys/class/thermal/thermal_zone0/temp; vcgencmd get_throttled 2>/dev/null
REMOTE
    BOT_PID=$!
else
    echo "  Bot nicht per SSH erreichbar -- nur Laptop."
fi

# --- Laptop ---
r0=$(cat /proc/net/dev)
top -b -c -d "$DUR" -n 2 -w 512 > "$OUT/top.txt" &
DL_PID=$(pgrep -f "lib/Follow_Lane/detect_lane_nav2" | head -1)
[ -n "$DL_PID" ] && top -b -H -p "$DL_PID" -d "$DUR" -n 2 -w 200 > "$OUT/threads_detect_lane.txt" &
nvidia-smi --query-gpu=utilization.gpu,clocks.sm,memory.used --format=csv,noheader -l 1 > "$OUT/gpu.csv" 2>/dev/null &
GPU_PID=$!
if command -v py-spy >/dev/null; then
    for n in $NODES; do
        p=$(pgrep -f "lib/Follow_Lane/$n( |$)" | head -1)
        [ -n "$p" ] && py-spy record --nonblocking -p "$p" -d "$DUR" -r 50 -f raw \
            -o "$OUT/pyspy_$n.txt" > /dev/null 2>&1 &
    done
fi
sleep "$DUR"; r1=$(cat /proc/net/dev)
sleep 3; kill "$GPU_PID" 2>/dev/null; wait

paste <(echo "$r0" | tail -n +3) <(echo "$r1" | tail -n +3) | tr ':' ' ' |
    awk -v d="$DUR" '{x=($19-$2)*8/d/1e6; if (x>0.05) printf "  %-12s %6.2f\n", $1, x}' > "$OUT/net_laptop.txt"
LOG=$(ls -t "$REPO_ROOT"/logs/pipeline/*-latest.log 2>/dev/null | head -1)
[ -n "$LOG" ] && grep -h "Zeit (ms\|Anzeige (ms\|YOLO: " "$LOG" | tail -6 | sed 's/^.*\]: //' > "$OUT/pipeline_zeiten.txt"

python3 "$SCRIPT_DIR/measure-summary.py" "$OUT" | tee "$OUT/zusammenfassung.txt"
