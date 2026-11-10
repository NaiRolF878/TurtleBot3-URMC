#!/bin/bash
# Laptop als Zeitserver fuer den Bot: startet chronyd (falls noch nicht laeuft) und traegt auf dem
# Bot einmalig den Laptop als NTP-Server ein. Ruft bot-up.sh auf; ist beides schon so, passiert nichts.
# Aufruf: bash time-server.sh          (Status: bash time-server.sh status)
#
# Warum: Ohne Bot-WLAN hat der Bot kein NTP, seine Uhr laeuft frei weg (gemessen ~5 ms/min).
# sync-time.sh stellt sie nur einmal; timesyncd fuehrt sie hiermit laufend nach.
# Der Container laeuft mit --network=host, chronyd ist also direkt unter LAPTOP_LAN_IP erreichbar.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1
CONF="$SCRIPT_DIR/chrony-laptop.conf"
BOT_IP="${BOT_LAN_IP:-}"
DROPIN=/etc/systemd/timesyncd.conf.d/urmc-laptop.conf

if [ "$1" = "status" ]; then
    pgrep -x chronyd >/dev/null && echo "Laptop: chronyd laeuft" || echo "Laptop: chronyd laeuft NICHT"
    chronyc -n clients 2>/dev/null | tail -n +3
    ssh -o BatchMode=yes "$BOT_USER@$BOT_IP" "timedatectl timesync-status | grep -E 'Server|Offset|Poll|Frequency'"
    exit 0
fi

# --- Laptop ---
if ! command -v chronyd >/dev/null; then
    echo "  chronyd fehlt (apt-get install -y chrony, steht im Dockerfile) -- kein Zeitserver."
    exit 1
fi
if ! pgrep -x chronyd >/dev/null; then
    chronyd -x -f "$CONF" && echo "  Zeitserver auf dem Laptop gestartet ($LAPTOP_LAN_IP)."
fi

# --- Bot: Laptop als Zeitquelle (Internet-NTP bleibt Ersatz, falls das Bot-WLAN an ist) ---
[ -z "$BOT_IP" ] && exit 0
ssh -o BatchMode=yes -o ConnectTimeout=3 "$BOT_USER@$BOT_IP" "bash -s" <<REMOTE
want="[Time]
NTP=$LAPTOP_LAN_IP
FallbackNTP=ntp.ubuntu.com
PollIntervalMaxSec=64"
if [ "\$(cat $DROPIN 2>/dev/null)" != "\$want" ]; then
    sudo -n mkdir -p $(dirname $DROPIN) && echo "\$want" | sudo -n tee $DROPIN >/dev/null \
        && sudo -n systemctl restart systemd-timesyncd \
        && echo "  Bot holt die Zeit jetzt vom Laptop ($LAPTOP_LAN_IP)." \
        || echo "  Bot: Zeitserver eintragen fehlgeschlagen (sudo ohne Passwort noetig)."
fi
REMOTE
