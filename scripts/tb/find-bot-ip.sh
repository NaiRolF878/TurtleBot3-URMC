#!/bin/bash
# Gibt die aktuelle IP des TurtleBot3 auf stdout aus (sonst nichts).
# Sucht per MAC-Adresse, weil die IP je nach WLAN/DHCP wechselt, die MAC aber nicht.
# Meldungen gehen bewusst nach stderr, damit "$(bash find-bot-ip.sh)" nur die IP liefert.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1

IFACE=$(ip -4 -o addr show scope global | awk '{print $2}' | grep -v -E "^(lo|docker0)$" | head -n1)
if [ -z "$IFACE" ]; then
    echo "Kein aktives Netzwerk-Interface gefunden." >&2
    exit 1
fi

CIDR=$(ip -4 -o addr show dev "$IFACE" | awk '{print $4}')
if [ -z "$CIDR" ]; then
    echo "Konnte kein Subnetz fuer $IFACE ermitteln." >&2
    exit 1
fi

echo "Suche Bot $BOT (MAC $BOT_MAC) in $CIDR ueber $IFACE ..." >&2

for attempt in 1 2 3 4 5 6; do
    BOT_IP=$(nmap -sn "$CIDR" 2>/dev/null | grep -B2 -i "$BOT_MAC" \
             | grep "Nmap scan report" | grep -oE "[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+")
    if [ -n "$BOT_IP" ]; then
        echo "Bot gefunden: $BOT_IP" >&2
        echo "$BOT_IP"
        exit 0
    fi
    echo "  (Versuch $attempt/6 hat den Bot nicht gesehen, warte kurz...)" >&2
    sleep 2
done

echo "Bot nicht gefunden. Ist er im selben WLAN und eingeschaltet?" >&2
exit 1
