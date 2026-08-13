#!/bin/bash
# Setzt die TurtleBot3-Umgebung und startet einen Launcher interaktiv.
# Aufruf: bash launch.sh   oder per Alias: tb

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

source "$SCRIPT_DIR/tb-env.sh"
[ $? -ne 0 ] && exit 1

shopt -s nullglob
launchers=("$SCRIPT_DIR/launchers/"*.sh)

if [ ${#launchers[@]} -eq 0 ]; then
    echo "Keine Launcher gefunden in launchers/"
    exit 1
fi

echo ""
echo "Verfuegbare Launcher:"
PS3="Auswahl (Nummer): "
select launcher in "${launchers[@]#$SCRIPT_DIR/launchers/}"; do
    if [ -z "$launcher" ]; then
        echo "Ungueltige Auswahl."
        continue
    fi

    usage=$(grep -m1 '^# *Aufruf:' "$SCRIPT_DIR/launchers/$launcher" | sed 's/^# *//')
    [ -n "$usage" ] && echo "  $usage"
    read -r -p "Argumente (leer = keine): " args

    read -r -p "Status mitloggen? [j/N]: " do_log
    if [[ "$do_log" =~ ^[jJyY] ]]; then
        bash "$SCRIPT_DIR/status_log.sh" &
        LOG_PID=$!
        # Logger zusammen mit dem Launcher beenden, sonst laeuft er als Waise weiter.
        # Erst die Kinder (sleep bzw. das laufende "ros2 topic echo"): bash arbeitet
        # ein Signal erst ab, wenn das aktuelle Vordergrund-Kommando zurueckkehrt --
        # ohne das bliebe der kill bis zu einer halben Minute liegen.
        stop_logger() {
            [ -n "$LOG_PID" ] || return
            pkill -P "$LOG_PID" 2>/dev/null
            kill "$LOG_PID" 2>/dev/null
        }
        trap stop_logger EXIT INT TERM
    fi

    # $args bewusst ohne Anfuehrungszeichen: soll in einzelne Argumente zerlegt werden.
    bash "$SCRIPT_DIR/launchers/$launcher" $args
    break
done
