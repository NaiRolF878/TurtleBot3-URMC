#!/bin/bash
# Spielt Sweet Victory (Klang 8 der eigenen OpenCR-Firmware, misc/firmware) auf dem Bot.
# STAND 16.11.: Klang 8 ist noch NICHT in der Firmware (MIDI ohne Melodiestimme) -> bleibt stumm.
# Aufruf: bash scripts/tb/victory.sh        (Bringup muss laufen: botup)
#         bash scripts/tb/victory.sh 0      anderer Klang, z. B. 0 = aus (bricht die Melodie ab)
# Klaenge: 0 OFF, 1 ON, 2 LOW_BATTERY, 3 ERROR, 6 Diamonds, 7 DuckTales Moon, 8 Sweet Victory.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
# turtlebot3_msgs (Sound.srv) liegt im turtlebot3_ws, nicht in /opt/ros.
for ws in /opt/turtlebot3_ws "$HOME/turtlebot3_ws"; do
    [ -f "$ws/install/setup.bash" ] && source "$ws/install/setup.bash" >/dev/null 2>&1
done
export ROS_DOMAIN_ID=$BOT_DOMAIN_ID

SOUND="${1:-8}"
timeout 10 ros2 service call /sound turtlebot3_msgs/srv/Sound "{value: $SOUND}" >/dev/null 2>&1 \
    && echo "Klang $SOUND gestartet." \
    || { echo "/sound nicht erreichbar -- laeuft botup?"; exit 1; }
