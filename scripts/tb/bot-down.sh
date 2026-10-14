#!/bin/bash
# Beendet Bringup und Kamera auf dem Bot.
# Aufruf: bash bot-down.sh
#
# Noetig, weil bot-up.sh die Prozesse losgeloest startet -- ein Schliessen der
# SSH-Sitzung beendet sie nicht.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1

BOT_IP=$(bash "$SCRIPT_DIR/find-bot-ip.sh") || exit 1

echo "=== Bot-Prozesse beenden ($BOT_IP) ==="
ssh "$BOT_USER@$BOT_IP" "bash -s" <<'EOF'
# Erst freundlich, dann bestimmt: der Bringup soll seine Nodes selbst abraeumen koennen.
pkill -f robot.launch.py    2>/dev/null && echo "  Bringup beendet"
pkill -f camera_node        2>/dev/null && echo "  Kamera beendet"
sleep 3
pkill -9 -f robot.launch.py 2>/dev/null
pkill -9 -f camera_node     2>/dev/null
pkill -9 -f turtlebot3_node 2>/dev/null

echo "--- verbliebene Prozesse ---"
pgrep -fa "camera_node|robot.launch.py|turtlebot3_node" || echo "  keine"
EOF

echo ""
echo "ACHTUNG: Der turtlebot3_node fuehrt den letzten /cmd_vel-Befehl unbegrenzt weiter aus."
echo "Stand der Bot noch unter Fahrbefehl, ist er jetzt nicht mehr steuerbar -- Kippschalter."
