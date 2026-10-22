#!/bin/bash
# Gibt die aktuelle IP des TurtleBot3 auf stdout aus (sonst nichts).
# Sucht per MAC-Adresse, weil die IP je nach WLAN/DHCP wechselt, die MAC aber nicht.
# Meldungen gehen bewusst nach stderr, damit "$(bash find-bot-ip.sh)" nur die IP liefert.
#
# Reihenfolge:
#   1. Direkte Kabelverbindung (net-lan.sh up): Bot unter seiner festen BOT_LAN_IP pingen.
#   2. MAC-Suche (WLAN- und LAN-MAC) in jedem Netz, in dem der Laptop eine Adresse hat.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1

# 1. Direkte Kabelverbindung -- schnell und ohne nmap
if [ -n "$BOT_LAN_IP" ] && ip -4 -o addr show | grep -q " $LAPTOP_LAN_IP/"; then
    if ping -c1 -W1 "$BOT_LAN_IP" >/dev/null 2>&1; then
        echo "Bot $BOT per Kabel: $BOT_LAN_IP" >&2
        echo "$BOT_LAN_IP"
        exit 0
    fi
    echo "Kabelnetz aktiv, aber $BOT_LAN_IP antwortet nicht -- suche weiter per MAC ..." >&2
fi

# 2. MAC-Suche in allen Netzen des Laptops (ohne lo/docker und ohne das feste Kabelnetz,
#    das in Schritt 1 schon geprueft wurde)
MACS="$BOT_MAC${BOT_MAC_LAN:+|$BOT_MAC_LAN}"
NETS=$(ip -4 -o addr show scope global | awk '$2 !~ /^(lo|docker0)$/ {print $2" "$4}' \
       | grep -v " $LAN_NET_PREFIX\.")
if [ -z "$NETS" ]; then
    echo "Kein aktives Netzwerk-Interface gefunden." >&2
    exit 1
fi

for attempt in 1 2 3 4 5 6; do
    while read -r IFACE CIDR; do
        [ "$attempt" = 1 ] && echo "Suche Bot $BOT (MAC $MACS) in $CIDR ueber $IFACE ..." >&2
        BOT_IP=$(nmap -sn "$CIDR" 2>/dev/null | grep -B2 -i -E "$MACS" \
                 | grep "Nmap scan report" | grep -oE "[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+" | head -n1)
        if [ -n "$BOT_IP" ]; then
            echo "Bot gefunden: $BOT_IP" >&2
            echo "$BOT_IP"
            exit 0
        fi
    done <<< "$NETS"
    echo "  (Versuch $attempt/6 hat den Bot nicht gesehen, warte kurz...)" >&2
    sleep 2
done

echo "Bot nicht gefunden. Ist er im selben WLAN bzw. per Kabel verbunden (net-lan.sh up) und eingeschaltet?" >&2
exit 1
