#!/bin/bash
# Aufruf: [image_view:=false] [avoid_params:=/pfad/datei.yaml]
# Startet Spurfolge + zonenbasiertes Ausweichen.
# Prioritaet der Fahrbefehle: /cmd_avoid > /cmd_lane (Mux in switch_control).
ros2 launch Follow_Lane zone_avoid.launch.py "$@"
