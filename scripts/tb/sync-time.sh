#!/bin/bash
# Setzt die Uhr des Bots auf die Laptop-Zeit.
# Aufruf: bash sync-time.sh
#
# Hintergrund: Der Raspberry Pi hat keine gepufferte Echtzeituhr und holt sich die Zeit
# sonst per NTP -- das LAN am Router hat aber keinen Internetzugang. Die Uhr laeuft
# deshalb ab dem Einschalten frei und driftet weg (gemessen: 5,4 h an einem Tag).
#
# Folgen einer falschen Uhr: ROS-Zeitstempel vom Bot sind auf dem Laptop wertlos.
# rviz zeigt "Showing [0] points", weil der TF-Puffer nur 10 s vorhaelt und Nachrichten
# mit uraltem Stempel komplett verwirft. Ebenso betroffen: Message-Filter, TF-Lookups
# und zeitbasiertes Matching im debug_view.
#
# WICHTIG: Danach alle ROS-Prozesse auf dem Bot neu starten (Bringup, Kamera) --
# laufende Nodes behalten sonst intern die alte Zeitbasis.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1

BOT_IP=$(bash "$SCRIPT_DIR/find-bot-ip.sh") || exit 1

# Erst den Versatz ueber EINE offene Verbindung messen (Ping-Pong, schnellster Rundlauf),
# dann die Bot-Uhr um genau diesen Wert verschieben (Ablesen und Setzen in EINEM Aufruf). Frueher wurde die Laptop-Zeit
# sekundengenau abgelesen und erst nach dem SSH-Verbindungsaufbau gesetzt -> Bot 1-2 s zu
# spaet (gemessen 2,17 s), YOLO verwarf danach jedes Bild als zu alt.
offset() {
    python3 - "$BOT_USER@$BOT_IP" <<'PY'
import subprocess, sys, time
p = subprocess.Popen(['ssh', '-o', 'BatchMode=yes', sys.argv[1], 'while read l; do date +%s.%N; done'],
                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
best = None
for _ in range(20):
    t0 = time.time(); p.stdin.write('x\n'); p.stdin.flush(); b = float(p.stdout.readline()); t1 = time.time()
    if best is None or t1 - t0 < best[0]:
        best = (t1 - t0, b - (t0 + t1) / 2)
p.stdin.close(); p.wait()
print(f'{best[1]:.4f}')
PY
}
OFF=$(offset) || { echo "Versatz nicht messbar (SSH ohne Passwort noetig)."; exit 1; }
echo "Bot minus Laptop vorher: $OFF s -- korrigiere ..."
ssh -o BatchMode=yes "$BOT_USER@$BOT_IP" \
    "sudo -n python3 -c 'import time; time.clock_settime(time.CLOCK_REALTIME, time.clock_gettime(time.CLOCK_REALTIME) - ($OFF))'" \
    || { echo "Setzen fehlgeschlagen (sudo ohne Passwort auf dem Bot noetig)."; exit 1; }
echo "Bot minus Laptop nachher: $(offset) s"
echo ""
echo "WICHTIG: Jetzt auf dem Bot Bringup und Kamera neu starten,"
echo "sonst laufen die Nodes mit der alten Zeitbasis weiter."
