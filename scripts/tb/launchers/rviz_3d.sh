#!/bin/bash
# Aufruf: keine Argumente
# Startet rviz2 mit der Projekt-Konfiguration: Roboter, TF, LaserScan, beide Costmaps
# und die geplanten Pfade.
#
# Fixed Frame ist bewusst "odom", nicht "map": Beide Costmaps laufen auf diesem Branch
# in odom (siehe global_frame in nav2_params.yaml), es gibt weder Karte noch AMCL.
# Mit "map" bliebe die Anzeige leer.

CONFIG="$(ros2 pkg prefix Follow_Lane 2>/dev/null)/share/Follow_Lane/rviz/urmc.rviz"

if [ ! -f "$CONFIG" ]; then
    echo "Konfiguration nicht gefunden: $CONFIG"
    echo "Workspace gebaut und gesourct? (colcon build, dann tb-env.sh)"
    exit 1
fi

# Kein xset-Test hier: das Programm ist im Container nicht installiert und meldete
# deshalb faelschlich fehlenden X-Zugriff. Scheitert rviz mit "could not connect to
# display", auf dem HOST "xhost +local:root" ausfuehren (nicht persistent).
if [ -z "$DISPLAY" ]; then
    echo "WARNUNG: DISPLAY ist nicht gesetzt -- rviz kann kein Fenster oeffnen."
    echo ""
fi

echo "Fixed Frame: odom. Ohne laufendes Bringup gibt es keine TF und die Anzeige"
echo "bleibt leer -- vorher botup starten."
echo ""

ros2 run rviz2 rviz2 -d "$CONFIG"
