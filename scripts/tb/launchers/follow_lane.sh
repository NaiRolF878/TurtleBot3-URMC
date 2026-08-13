#!/bin/bash
# Aufruf: keine Argumente
# Startet die komplette Follow-Lane-Pipeline (ingress_node, detect_lane, control_driving, switch_control, debug_view).
ros2 launch Follow_Lane follow_lane.launch.py
