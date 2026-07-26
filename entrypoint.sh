#!/bin/bash
set -e

# ROS2 Jazzy sourcing
source /opt/ros/jazzy/setup.bash

# TurtleBot3 Workspace
if [ -f /turtlebot3_ws/install/setup.bash ]; then
    source /turtlebot3_ws/install/setup.bash
fi

# Overlay Workspace (eigener Code)
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Umgebungsvariablen sicherstellen
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

echo "──────────────────────────────────────────────"
echo "  TurtleBot3 URMC Dev Container"
echo "  ROS2:   $ROS_DISTRO"
echo "  Modell: $TURTLEBOT3_MODEL"
echo "  DDS:    $RMW_IMPLEMENTATION"
echo "  Domain: $ROS_DOMAIN_ID"
echo "──────────────────────────────────────────────"

exec "$@"
