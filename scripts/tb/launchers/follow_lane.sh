#!/bin/bash
# Aufruf: keine Argumente
# Startet die komplette Follow-Lane-Pipeline (ingress_node, detect_lane, control_driving, switch_control,
# debug_view) im Hintergrund und den NOT-AUS (notaus) im Vordergrund dieses Terminals.
#
# Der Bot faehrt NICHT von selbst: erst LEERTASTE in diesem Terminal gibt die Fahrt frei,
# nochmal Leertaste = STOPP. Alternativ rqt_reconfigure -> Node notaus -> Haekchen "drive".
# q oder Strg+C beendet den Not-Aus (sendet Stopp) und danach die ganze Pipeline.

ros2 launch Follow_Lane follow_lane.launch.py > /tmp/follow_lane_launch.log 2>&1 &
LAUNCH_PID=$!
cleanup() {
    echo ""
    echo "Beende Pipeline ..."
    kill -INT "$LAUNCH_PID" 2>/dev/null
    wait "$LAUNCH_PID" 2>/dev/null
}
trap cleanup EXIT

echo "Pipeline startet im Hintergrund (Log: /tmp/follow_lane_launch.log)."
echo "Not-Aus: LEERTASTE = Fahren/Stopp, q = beenden."
echo ""
ros2 run Follow_Lane notaus
