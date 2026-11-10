#!/bin/bash
# Aufruf: keine Argumente
# Startet rviz2 mit der Projekt-Konfiguration: Roboter, TF, LaserScan, beide Costmaps
# und die geplanten Pfade.
#
# Fixed Frame "odom": beide Costmaps laufen in odom, es gibt weder Karte noch AMCL.

CONFIG="$(ros2 pkg prefix Follow_Lane 2>/dev/null)/share/Follow_Lane/rviz/urmc.rviz"

if [ ! -f "$CONFIG" ]; then
    echo "Konfiguration nicht gefunden: $CONFIG"
    echo "Workspace gebaut und gesourct? (colcon build, dann tb-env.sh)"
    exit 1
fi

# Scheitert rviz mit "could not connect to display": auf dem Host "xhost +local:root".
if [ -z "$DISPLAY" ]; then
    echo "WARNUNG: DISPLAY ist nicht gesetzt -- rviz kann kein Fenster oeffnen."
    echo ""
fi

echo "Fixed Frame: odom. Ohne laufendes Bringup gibt es keine TF und die Anzeige"
echo "bleibt leer -- vorher botup starten."
echo ""

ros2 run rviz2 rviz2 -d "$CONFIG"
