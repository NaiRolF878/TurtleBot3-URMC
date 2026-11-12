#!/bin/bash
# Setzt die TurtleBot3-Umgebung und startet einen Launcher interaktiv.
# Aufruf: bash launch.sh   oder per Alias: tb

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

source "$SCRIPT_DIR/tb-env.sh"
[ $? -ne 0 ] && exit 1

# Bei jedem Start bauen (~2-3 s ohne Aenderungen): nach git pull oder Branch-Wechsel fehlen
# sonst neue Programme oder Nachrichten. --symlink-install: Python und YAML wirken sofort.
echo "Baue Workspace (colcon build --symlink-install) ..."
BUILD_LOG=$(cd "$REPO_ROOT/workspace" && colcon build --symlink-install 2>&1)
BUILD_RC=$?
if [ $BUILD_RC -ne 0 ]; then
    # Meist Reste eines anderen Branches: --symlink-install-Verweise auf Dateien, die es hier
    # nicht gibt ("can't copy ... not a regular file"), oder geaenderte .msg. Sauber neu (~8 s).
    echo "Build fehlgeschlagen -- raeume workspace/build und install auf und baue neu ..."
    rm -rf "$REPO_ROOT/workspace/build" "$REPO_ROOT/workspace/install"
    BUILD_LOG=$(cd "$REPO_ROOT/workspace" && colcon build --symlink-install 2>&1)
    BUILD_RC=$?
fi
if [ $BUILD_RC -ne 0 ]; then
    echo "$BUILD_LOG" | tail -30
    echo ""
    echo "Build fehlgeschlagen -- es wird nichts gestartet. Fehler siehe oben."
    exit 1
fi
echo "$BUILD_LOG" | grep -E "^Summary:"
# Neu sourcen, damit neu hinzugekommene Pakete/Programme gefunden werden
source "$REPO_ROOT/workspace/install/setup.bash"

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
        # Logger mit dem Launcher beenden, erst seine Kinder: bash arbeitet ein Signal erst
        # nach dem laufenden Vordergrund-Kommando ab.
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
