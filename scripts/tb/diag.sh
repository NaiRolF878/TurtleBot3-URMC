#!/bin/bash
# Sammelt alle relevanten Diagnose-Infos zur Kamera-Pipeline in eine Datei.
# Aufruf (im Devcontainer, waehrend Kamera + Follow_Lane bereits laufen):
#   bash scripts/tb/diag.sh
# Ausgabe: /tmp/tb-diag.txt -- Inhalt danach einfach hier reinposten.

OUT=/tmp/tb-diag.txt
source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/TurtleBot3-URMC/workspace/install/setup.bash 2>/dev/null

{
    echo "=== date ==="; date
    echo "=== hostname -I ==="; hostname -I
    echo "=== node list ==="; timeout 6 ros2 node list
    echo "=== topic list ==="; timeout 6 ros2 topic list
    # Bewusst KEIN "ros2 topic hz /camera/image_raw": das legt einen Subscriber an und
    # zieht 1.44 MB/Frame ueber das WLAN -- das saettigt den Link und zerstoert genau den
    # komprimierten Stream, den wir hier vermessen wollen. "info -v" ist dagegen reine
    # Discovery-Abfrage ohne Datenverkehr und darf bleiben.
    echo "=== camera/image_raw info -v (kein Datenabruf!) ==="; timeout 5 ros2 topic info /camera/image_raw -v
    echo "=== camera/image_raw/compressed hz (6s) ==="; timeout 7 ros2 topic hz /camera/image_raw/compressed
    echo "=== camera/image_raw/compressed bw (6s) ==="; timeout 7 ros2 topic bw /camera/image_raw/compressed
    echo "=== lokal/raw_image hz (6s) ==="; timeout 7 ros2 topic hz /lokal/raw_image
    echo "=== lokal/raw_image info -v ==="; timeout 5 ros2 topic info /lokal/raw_image -v
    echo "=== camera/image_raw echo --once (Header+Encoding, ohne Bilddaten) ==="
    timeout 5 ros2 topic echo --once --field height /camera/image_raw 2>&1
    timeout 5 ros2 topic echo --once --field width /camera/image_raw 2>&1
    timeout 5 ros2 topic echo --once --field encoding /camera/image_raw 2>&1
    echo "=== scan QoS (bekannter Mismatch) ==="; timeout 5 ros2 topic info /scan -v
} > "$OUT" 2>&1

echo "Fertig. Inhalt von $OUT:"
cat "$OUT"
