#!/bin/bash
# Setzt die ROS2-Umgebung fuer TurtleBot3-URMC.
# Wird von launch.sh gesourct, kann aber auch direkt gesourct werden:
#   source scripts/tb/tb-env.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

export ROS_DOMAIN_ID=33

source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# Verhindert, dass grosse DDS-Nachrichten der IP-Fragmentierung ueberlassen werden.
# Ohne das scheitern auf diesem WLAN ~98 % der Reassemblierungen -> Bilder und
# Scans kommen nicht an. Details im Kopf der XML-Datei.
export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO_ROOT/.devcontainer/fastdds_profile.xml"

if [ ! -f "$REPO_ROOT/workspace/install/setup.bash" ]; then
    echo "Follow_Lane-Workspace noch nicht gebaut. Baue jetzt (colcon build --symlink-install)..."
    ( cd "$REPO_ROOT/workspace" && colcon build --symlink-install ) || return 1
fi
source "$REPO_ROOT/workspace/install/setup.bash"

# Daemon nur starten, NICHT vorher stoppen: tb laeuft in mehreren Terminals
# gleichzeitig (Launch, rqt, Logger). Ein "daemon stop" in Terminal 2 reisst
# Terminal 1 den Daemon weg -> dort scheitert danach jedes ros2-Kommando mit
# "xmlrpc.client.Fault: RuntimeError: !rclpy.ok()". "start" allein ist harmlos,
# wenn schon einer laeuft.
# Nach einem WLAN-/IP-Wechsel muss der Daemon dagegen wirklich neu: dafuer
# bewusst von Hand "ros2 daemon stop" in EINEM Terminal, waehrend nichts laeuft.
ros2 daemon start >/dev/null 2>&1

echo "ROS2-Umgebung bereit (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, Follow_Lane-Workspace gesourct)."
echo "Hinweis: Die Kamera (camera_ros) laeuft nur auf dem Bot, nicht hier im Devcontainer."
echo "  Auf dem Bot separat per SSH starten: ros2 run camera_ros camera_node --ros-args -p format:=RGB888"
