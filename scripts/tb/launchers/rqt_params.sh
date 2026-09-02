#!/bin/bash
# Aufruf: keine Argumente
# Startet rqt_reconfigure zum Einstellen der Parameter von detect_lane und control_driving.

echo ""
echo "Regler links nach Node auswaehlen (detect_lane, control_driving)."
echo "Aenderungen greifen sofort -- beide Nodes lesen ihre Parameter bei jedem Frame neu."
echo ""
echo "Die Datei-Knoepfe in rqt funktionieren -- Parameterdateien liegen im flachen"
echo "Format vor, das rqt liest und schreibt. Verzeichnis:"
echo "  workspace/src/Follow_Lane/Follow_Lane/Parameter/"
echo ""
echo "Alternativ im Terminal, ohne Dateidialog:"
echo "  bash scripts/tb/save-params.sh detect_lane      # sichern"
echo "  bash scripts/tb/load-params.sh detect_lane      # zurueckladen"
echo ""

ros2 run rqt_reconfigure rqt_reconfigure
