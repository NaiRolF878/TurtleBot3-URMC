#!/bin/bash
# Kurz: RENNEN: nur Spurfolge, Start an der Schranke, volle Geschwindigkeit
# Aufruf: keine Argumente
# Wie follow_lane.sh, aber fuer das Rennen: ohne Kreuzungen (intersection_node laeuft nicht), ohne
# Objekterkennung (keine Schilder, Bots, Ziellinie), ohne Stoppschild, ohne Nav2. Startet in Stop und
# faehrt los, sobald die Schranke offen ist (blaue Pixel weg). Werte: Parameter/rennen/Rennen.yaml.
#
# Ablauf: Bot vor die geschlossene Schranke stellen, rennen.sh starten, LEERTASTE (Fahrt frei).
# Er steht, bis die Schranke aufgeht. q oder Strg+C = Stopp und Ende.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SHARE="$(ros2 pkg prefix Follow_Lane 2>/dev/null)/share/Follow_Lane"
export FOLLOW_LANE_ARGS="intersections:=false detector_node:=true extra_params:=$SHARE/Parameter/Rennen.yaml"
export FOLLOW_LANE_LOG=rennen
exec bash "$SCRIPT_DIR/follow_lane.sh"
