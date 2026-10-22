#!/bin/bash
# Direkte Kabelverbindung Laptop <-> Bot (ohne Router) an- und abschalten -- Laptop-Seite.
# Aufruf: bash net-lan.sh up|down|status   (Kurzbefehle: lanup / landown / lanstatus)
#
# Ohne Router verteilt niemand Adressen (DHCP), deshalb feste Adressen aus bot.conf:
# Laptop LAPTOP_LAN_IP, Bot BOT_LAN_IP. Der Bot braucht dafuer EINMALIG bot-lan-setup.sh.
# WLAN bleibt unberuehrt -- beides geht parallel, find-bot-ip.sh nimmt bevorzugt das Kabel.
#
# Laeuft im Container: der hat --network=host und --privileged, "ip addr" wirkt also direkt
# auf den LAN-Anschluss des Laptops. Die Adresse ist nicht dauerhaft (weg nach Neustart des
# Laptops oder wenn NetworkManager den Anschluss neu konfiguriert) -- dann einfach erneut "up".
#
# Anderes Interface erzwingen: LAN_IFACE=enx... bash net-lan.sh up

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1

# Kabel-Interface: erstes Ethernet (en*/eth*) mit Verbindung, sofern nicht vorgegeben
if [ -z "$LAN_IFACE" ]; then
    for i in /sys/class/net/en* /sys/class/net/eth*; do
        [ -e "$i/carrier" ] && [ "$(cat "$i/carrier" 2>/dev/null)" = 1 ] && LAN_IFACE=$(basename "$i") && break
    done
fi

has_ip() { ip -4 -o addr show dev "$LAN_IFACE" 2>/dev/null | grep -q " $LAPTOP_LAN_IP/"; }

case "$1" in
    up)
        if [ -z "$LAN_IFACE" ]; then
            echo "Kein LAN-Anschluss mit Verbindung gefunden. Kabel eingesteckt? (oder LAN_IFACE=... setzen)"
            exit 1
        fi
        if has_ip; then
            echo "$LAN_IFACE hat schon $LAPTOP_LAN_IP."
        else
            ip addr add "$LAPTOP_LAN_IP/24" dev "$LAN_IFACE" || exit 1
            echo "$LAN_IFACE: $LAPTOP_LAN_IP/24 gesetzt."
        fi
        if [ -n "$BOT_LAN_IP" ]; then
            if ping -c2 -W1 "$BOT_LAN_IP" >/dev/null 2>&1; then
                echo "Bot $BOT antwortet per Kabel unter $BOT_LAN_IP."
            else
                echo "Bot antwortet (noch) nicht unter $BOT_LAN_IP."
                echo "  Einmalig auf dem Bot einrichten: lansetup"
            fi
        fi
        echo ""
        echo "Hinweis: Haengt der Laptop zusaetzlich im WLAN, kann ROS die Daten trotzdem darueber"
        echo "schicken. Fuer reinen Kabelbetrieb das WLAN am Laptop ausschalten."
        ;;
    down)
        if [ -n "$LAN_IFACE" ] && has_ip; then
            ip addr del "$LAPTOP_LAN_IP/24" dev "$LAN_IFACE" && echo "$LAN_IFACE: $LAPTOP_LAN_IP entfernt."
        else
            # auch ohne Kabel aufraeumen, falls die Adresse noch irgendwo haengt
            DEV=$(ip -4 -o addr show | awk -v ip="$LAPTOP_LAN_IP/24" '$4==ip {print $2; exit}')
            if [ -n "$DEV" ]; then
                ip addr del "$LAPTOP_LAN_IP/24" dev "$DEV" && echo "$DEV: $LAPTOP_LAN_IP entfernt."
            else
                echo "$LAPTOP_LAN_IP ist nirgends gesetzt -- nichts zu tun."
            fi
        fi
        ;;
    status)
        echo "LAN-Anschluss: ${LAN_IFACE:-keiner mit Verbindung}"
        ip -4 -br addr show | grep -v -E "^(lo|docker0)"
        if [ -n "$BOT_LAN_IP" ]; then
            ping -c1 -W1 "$BOT_LAN_IP" >/dev/null 2>&1 && echo "Bot per Kabel: $BOT_LAN_IP erreichbar" \
                                                      || echo "Bot per Kabel: $BOT_LAN_IP nicht erreichbar"
        fi
        ;;
    *)
        echo "Aufruf: bash net-lan.sh up|down|status"
        exit 1
        ;;
esac
