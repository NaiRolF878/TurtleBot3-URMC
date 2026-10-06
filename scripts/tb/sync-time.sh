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

NOW=$(date -u +'%Y-%m-%d %H:%M:%S')
echo "Setze Bot-Uhr auf $NOW UTC ..."

# -t: sudo fragt sonst bei nicht-interaktivem ssh ins Leere und haengt.
ssh -t "$BOT_USER@$BOT_IP" "sudo date -u -s '$NOW'" || exit 1

echo ""
echo "Gesetzt. Versatz zur Kontrolle:"
ssh "$BOT_USER@$BOT_IP" "date -u +'%Y-%m-%d %H:%M:%S'" | while read -r botzeit; do
    echo "  Bot:    $botzeit UTC"
    echo "  Laptop: $(date -u +'%Y-%m-%d %H:%M:%S') UTC"
done

echo ""
echo "WICHTIG: Jetzt auf dem Bot Bringup und Kamera neu starten,"
echo "sonst laufen die Nodes mit der alten Zeitbasis weiter."
