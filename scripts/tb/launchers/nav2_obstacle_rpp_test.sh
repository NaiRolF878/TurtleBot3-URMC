#!/bin/bash
# Kurz: wie nav2_obstacle, aber Nav2 mit RPP ohne Kollisionspruefung (Test)
# Aufruf: keine Argumente
# TESTPROFIL: wie nav2_obstacle.sh, aber Nav2 faehrt mit Regulated Pure Pursuit OHNE eigene
# Kollisionspruefung (Parameter/nav2_params_rpp_test.yaml). Der Plan von NavFn gilt als wahr,
# der Bot faehrt ihn einfach ab -- Test, ob er so durch enge Luecken kommt.
# Not-Aus bereithalten. Tunnel mit diesem Profil NICHT testen (tunnel_explorer_node setzt
# MPPI-Parameter). Zurueck zum Normalbetrieb: nav2_obstacle.sh.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PARAMS="$(ros2 pkg prefix Follow_Lane)/share/Follow_Lane/Parameter/nav2_params_rpp_test.yaml"
if [ ! -f "$PARAMS" ]; then
    echo "Testprofil fehlt: $PARAMS -- Workspace neu bauen (tb baut beim Start)."
    exit 1
fi
echo "*** TESTPROFIL: Regulated Pure Pursuit ohne Kollisionspruefung ***"
exec bash "$SCRIPT_DIR/nav2_obstacle.sh" nav2_params:="$PARAMS"
