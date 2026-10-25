# Startet camera_node AUF DEM BOT. Wird nicht direkt aufgerufen, sondern von bot-up.sh und
# cam-mode.sh per ssh mitgeschickt. Die ROS-Umgebung muss schon gesourct sein.
#
# Erwartet: CAM_W CAM_H CAM_Q CAM_FRAME_US CAM_REL,
#   optional CAM_SENSOR_MODE CAM_COLOUR_GAINS CAM_ORIENTATION CAM_TUNING (Dateiname in ~/tuning/)

echo "--- Kamera starten: ${CAM_W}x${CAM_H}, q$CAM_Q, $((1000000 / CAM_FRAME_US)) Hz, $CAM_REL," \
     "Sensormodus ${CAM_SENSOR_MODE:-auto}, Weissabgleich ${CAM_COLOUR_GAINS:-auto}, Drehung ${CAM_ORIENTATION:-0}, Tuning ${CAM_TUNING:-standard} ---"

# Verwaiste camera_node-Prozesse halten die Kamera belegt ("failed to acquire camera").
pkill -9 -f camera_node 2>/dev/null && echo "  alte camera_node beendet" && sleep 2
mkdir -p "$HOME/logs"

# QoS-Overrides: der Publisher stand sonst auf RELIABLE. Fuer einen Livestrom ist das
# falsch -- DDS haelt verlorene Samples vor und sendet sie erneut, bei Paketverlust
# staut sich der Sender und liefert alte Bilder nach. BEST_EFFORT + Tiefe 1 heisst:
# immer nur das neueste Bild, verlorene werden verworfen statt nachgereicht.
# (cam-mode.sh full nimmt bewusst RELIABLE, Begruendung dort.)
CAM_ARGS=(
    -p format:=RGB888 -p width:=$CAM_W -p height:=$CAM_H -p jpeg_quality:=$CAM_Q
    -p FrameDurationLimits:="[$CAM_FRAME_US,$CAM_FRAME_US]"
    -p qos_overrides./camera/image_raw/compressed.publisher.reliability:=$CAM_REL
    -p qos_overrides./camera/image_raw/compressed.publisher.history:=keep_last
    -p qos_overrides./camera/image_raw/compressed.publisher.depth:=1
)
if [ -n "$CAM_ORIENTATION" ] && [ "$CAM_ORIENTATION" != 0 ]; then
    CAM_ARGS+=(-p orientation:=$CAM_ORIENTATION)
fi
if [ -n "$CAM_COLOUR_GAINS" ]; then
    CAM_ARGS+=(-p AwbEnable:=false -p ColourGains:="[$CAM_COLOUR_GAINS]")
fi

# Tuning-Datei: libcamera liest LIBCAMERA_RPI_TUNING_FILE beim Start. Fehlt die Datei, lieber mit
# Standard-Tuning starten als gar nicht -- aber laut warnen.
unset LIBCAMERA_RPI_TUNING_FILE
if [ -n "$CAM_TUNING" ]; then
    if [ -f "$HOME/tuning/$CAM_TUNING" ]; then
        export LIBCAMERA_RPI_TUNING_FILE="$HOME/tuning/$CAM_TUNING"
    else
        echo "  WARNUNG: Tuning-Datei ~/tuning/$CAM_TUNING fehlt -- starte mit Standard-Tuning."
    fi
fi

start_camera() {
    setsid nohup ros2 run camera_ros camera_node --ros-args "$@" \
        > "$HOME/logs/camera.log" 2>&1 < /dev/null &
    sleep 5
    pgrep -f camera_node >/dev/null
}

if [ -n "$CAM_SENSOR_MODE" ]; then
    if ! start_camera "${CAM_ARGS[@]}" -p sensor_mode:="$CAM_SENSOR_MODE"; then
        echo "  WARNUNG: Start mit sensor_mode=$CAM_SENSOR_MODE fehlgeschlagen, letzte Zeilen:"
        tail -5 "$HOME/logs/camera.log" 2>/dev/null | sed 's/^/    /'
        echo "  Starte ohne sensor_mode neu ..."
        start_camera "${CAM_ARGS[@]}"
    fi
else
    start_camera "${CAM_ARGS[@]}"
fi

if pgrep -f camera_node >/dev/null; then
    echo "  Kamera laeuft. Sensormodus laut Log:"
    grep -i "sensor format\|configuration" "$HOME/logs/camera.log" | tail -2 | sed 's/^/    /'
else
    echo "  Kamera NICHT gestartet -- letzte Zeilen aus ~/logs/camera.log:"
    tail -15 "$HOME/logs/camera.log" 2>/dev/null | sed 's/^/    /'
fi
