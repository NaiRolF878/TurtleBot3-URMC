#!/bin/bash
# Kurz: wie follow_lane, speichert jede Sekunde ein YOLO-Trainingsbild
# Aufruf: keine Argumente
# Wie follow_lane.sh (Spurverfolgung + Not-Aus, Leertaste = Fahren/Stopp), dazu speichert
# image_recorder waehrend der Fahrt jede Sekunde ein Trainingsbild fuer YOLO, abwechselnd
# Original-Kamerabild (_roh) und intrinsisch entzerrtes (_entzerrt).
# Ablage: dataset/fahrt_<datum>_<zeit>/ (dataset/ ist in .gitignore). Im Stand und im Tunnel
# wird nichts gespeichert. Takt: RECORD_PERIOD_S in Follow_Lane/src/image_recorder.py.
FOLLOW_LANE_LAUNCH=follow_lane_record.launch.py exec bash "$(dirname "$0")/follow_lane.sh" "$@"
