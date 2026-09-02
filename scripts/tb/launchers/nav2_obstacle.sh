#!/bin/bash
# Aufruf: keine Argumente
# Startet den Nav2-Zweig fuer Hindernisse und Tunnel -- inklusive der Spurverfolgung,
# auf die er angewiesen ist. Also alles in einem.
#
# Die Zweige schliessen sich ueber /control/mode gegenseitig aus: control_driving
# faehrt bei ControlType.Lane, der Nav2-Zweig bei Obstacle bzw. Tunnel.
#
# Nur Spurverfolgung ohne Nav2: follow_lane.sh

echo ""
echo "Not-Aus bereithalten -- der Bot faehrt eigenstaendig, sobald Bilder ankommen:"
echo "  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \"{}\""
echo ""

ros2 launch Follow_Lane nav2_obstacle.launch.py
