#!/bin/bash
# Aufruf: sim.sh <worktree> [welt]     z.B. sim.sh wt-nav2 tunnel_gap
#
# Baut den Simulator und die Pipeline des Worktrees (Ordner neben wt-sim) in ~/urmc_sim_ws,
# startet beides im Hintergrund und den Not-Aus im Vordergrund. Welten: urmc_sim/worlds.
SIM_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
BASE="$(dirname "$SIM_DIR")"
TREE="${1:?Worktree angeben, z.B.: bash scripts/sim/sim.sh wt-nav2}"
WORLD="${2:-tunnel_gap}"
if [ ! -d "$BASE/$TREE/workspace/src/Follow_Lane" ]; then
    echo "Kein workspace/src/Follow_Lane in $BASE/$TREE" >&2
    exit 1
fi
source "$SIM_DIR/scripts/sim/sim-env.sh"

# Die Quellen bleiben in den Worktrees (Links), gebaut wird in WSL. Anderer Worktree:
# dessen Pakete neu bauen.
mkdir -p "$SIM_WS/src"
if [ "$(cat "$SIM_WS/.tree" 2>/dev/null)" != "$TREE" ]; then
    rm -rf "$SIM_WS"/build/Follow_Lane "$SIM_WS"/install/Follow_Lane \
           "$SIM_WS"/build/follow_lane_msg "$SIM_WS"/install/follow_lane_msg
    echo "$TREE" > "$SIM_WS/.tree"
fi
ln -sfn "$BASE/$TREE/workspace/src/Follow_Lane" "$SIM_WS/src/Follow_Lane"
ln -sfn "$BASE/$TREE/workspace/src/follow_lane_msg" "$SIM_WS/src/follow_lane_msg"
ln -sfn "$SIM_DIR/workspace/src/urmc_sim" "$SIM_WS/src/urmc_sim"
if ! (cd "$SIM_WS" && colcon build --symlink-install); then
    echo "Bauen fehlgeschlagen." >&2
    exit 1
fi
source "$SIM_WS/install/setup.bash"

# Log der Knoten unter wt-sim/logs, damit es auch unter Windows lesbar ist.
mkdir -p "$SIM_DIR/logs"
NODE_LOG="$SIM_DIR/logs/sim-$TREE-$WORLD-$(date +%Y%m%d-%H%M%S).log"

# set -m: eigene Prozessgruppe, sonst ignoriert der Hintergrundjob SIGINT.
set -m
ros2 launch urmc_sim sim.launch.py world:="$WORLD" > "$NODE_LOG" 2>&1 &
LAUNCH_PID=$!
set +m
cleanup() {
    stty sane 2>/dev/null
    echo ""
    echo "Beende Simulation ..."
    kill -INT -- -"$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 1 15); do
        kill -0 "$LAUNCH_PID" 2>/dev/null || break
        sleep 1
    done
    kill -TERM -- -"$LAUNCH_PID" 2>/dev/null
    wait "$LAUNCH_PID" 2>/dev/null
    echo "Simulation beendet. Log: $NODE_LOG"
}
trap cleanup EXIT

echo "Simulation ($TREE, Welt $WORLD) startet im Hintergrund. Log: $NODE_LOG"
echo ""
echo "Fenster: /sim/view (Welt von oben), /nav2/view (Nav2), /lokal/debug_view."
echo "Tunnel starten: im Debugbild TUNNEL STARTEN klicken (\"Publish click location\" anhaken)"
echo "  oder in einem zweiten Terminal:"
echo "  source scripts/sim/sim-env.sh"
echo "  ros2 topic pub --once /control/tunnel_request std_msgs/msg/Bool '{data: true}'"
echo "Zurueck an den Start: ros2 topic pub --once /sim/reset std_msgs/msg/Empty '{}', dann L."
echo ""
echo "Not-Aus: LEERTASTE = Fahren/Stopp, q = beenden, L = Karten leeren, O = Obstacle aus/an."
echo ""
ros2 run Follow_Lane notaus
