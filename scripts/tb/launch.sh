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

# Diese Launcher stehen im Untermenue "Debug / Werkzeuge" statt im Hauptmenue. Die Dateien
# bleiben in launchers/ (relative Pfade und Doku verweisen dorthin).
DEBUG_LAUNCHERS=(dataset_capture.sh extrinsik_view.sh follow_lane_record.sh)
DEBUG_ENTRY="Debug / Werkzeuge ..."

shopt -s nullglob
all=("$SCRIPT_DIR/launchers/"*.sh)
if [ ${#all[@]} -eq 0 ]; then
    echo "Keine Launcher gefunden in launchers/"
    exit 1
fi
main_launchers=()
debug_launchers=()
for f in "${all[@]}"; do
    name=$(basename "$f")
    if [[ " ${DEBUG_LAUNCHERS[*]} " == *" $name "* ]]; then
        debug_launchers+=("$name")
    else
        main_launchers+=("$name")
    fi
done

# Menuezeile: Name und die Zeile "# Kurz:" aus dem Launcher.
label() {
    local kurz
    kurz=$(grep -m1 '^# *Kurz:' "$SCRIPT_DIR/launchers/$1" | sed 's/^# *Kurz: *//')
    printf '%-28s %s' "$1" "$kurz"
}

# Zeigt ein Menue aus Launcher-Namen (plus optional einem Extra-Eintrag), setzt $choice.
pick() {
    local title=$1 extra=$2; shift 2
    local items=("$@") labels=() i
    for i in "${items[@]}"; do labels+=("$(label "$i")"); done
    [ -n "$extra" ] && labels+=("$extra")
    echo ""
    echo "$title"
    PS3="Auswahl (Nummer): "
    choice=""
    select entry in "${labels[@]}"; do
        if [ -z "$entry" ]; then
            echo "Ungueltige Auswahl."
            continue
        fi
        if [ -n "$extra" ] && [ "$entry" == "$extra" ]; then
            choice="$extra"
        else
            choice="${items[$((REPLY - 1))]}"
        fi
        break
    done
}

pick "Verfuegbare Launcher:" "$DEBUG_ENTRY" "${main_launchers[@]}"
if [ "$choice" == "$DEBUG_ENTRY" ]; then
    pick "Debug / Werkzeuge:" "" "${debug_launchers[@]}"
fi
launcher="$choice"
[ -z "$launcher" ] && exit 1

{
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
}
