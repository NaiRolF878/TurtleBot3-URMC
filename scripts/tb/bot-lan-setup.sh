#!/bin/bash
# EINMALIG: gibt dem Bot eine feste Zusatzadresse (BOT_LAN_IP) auf seinem LAN-Anschluss eth0.
# Aufruf: bash bot-lan-setup.sh   (Kurzbefehl: lansetup)
#
# Danach funktioniert die direkte Kabelverbindung zum Laptop (net-lan.sh up). DHCP auf eth0
# bleibt wie bisher an -- am Router bekommt der Bot weiter seine normale Adresse, die feste
# kommt nur dazu. WLAN wird nicht angefasst.
#
# Macht zwei Dinge auf dem Bot (braucht sudo, fragt nach dem Passwort):
#   1. sofort:   ip addr add BOT_LAN_IP/24 dev eth0         (stoert keine laufende Verbindung)
#   2. dauerhaft: /etc/netplan/60-urmc-lan.yaml            (gilt ab dem naechsten Start)
#      Bewusst KEIN "netplan apply" -- das wuerde WLAN/LAN kurz neu aufbauen und die
#      SSH-Verbindung kappen.
#
# Erreicht den Bot ueber WLAN (find-bot-ip.sh) oder, falls nur das Kabel steckt, ueber die
# IPv6-Link-Local-Adresse, die sich aus der LAN-MAC ergibt und ohne Konfiguration funktioniert.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1

if [ -z "$BOT_LAN_IP" ]; then
    echo "Fuer Bot '$BOT' ist keine BOT_LAN_IP in bot.conf eingetragen."
    exit 1
fi

TARGET=$(bash "$SCRIPT_DIR/find-bot-ip.sh" 2>/dev/null)
if [ -z "$TARGET" ] && [ -n "$BOT_MAC_LAN" ]; then
    # IPv6-Link-Local aus der MAC (EUI-64): 2c:cf:67:47:38:53 -> fe80::2ecf:67ff:fe47:3853
    IFS=: read -r a b c d e f <<< "$(echo "$BOT_MAC_LAN" | tr 'A-F' 'a-f')"
    a=$(printf '%02x' $(( 0x$a ^ 0x02 )))
    for i in /sys/class/net/en* /sys/class/net/eth*; do
        [ -e "$i/carrier" ] && [ "$(cat "$i/carrier" 2>/dev/null)" = 1 ] && LAN_IFACE=$(basename "$i") && break
    done
    [ -n "$LAN_IFACE" ] && TARGET="fe80::$a$b:${c}ff:fe$d:$e$f%$LAN_IFACE"
fi
if [ -z "$TARGET" ]; then
    echo "Bot weder per WLAN noch per Kabel (IPv6-Link-Local) erreichbar."
    exit 1
fi

echo "=== Feste LAN-Adresse $BOT_LAN_IP auf $BOT einrichten (ueber $TARGET) ==="
echo "sudo fragt gleich nach dem Passwort von $BOT_USER."
ssh -t "$BOT_USER@$TARGET" "
if ! ip -4 -o addr show dev eth0 | grep -q ' $BOT_LAN_IP/'; then
    sudo ip addr add $BOT_LAN_IP/24 dev eth0 && echo '  sofort aktiv: $BOT_LAN_IP auf eth0'
else
    echo '  $BOT_LAN_IP ist auf eth0 schon gesetzt'
fi
printf 'network:\n  version: 2\n  ethernets:\n    eth0:\n      addresses: [$BOT_LAN_IP/24]\n' \
    | sudo tee /etc/netplan/60-urmc-lan.yaml >/dev/null \
    && sudo chmod 600 /etc/netplan/60-urmc-lan.yaml \
    && echo '  dauerhaft: /etc/netplan/60-urmc-lan.yaml (gilt ab dem naechsten Start)'
"
echo ""
echo "Laptop-Seite: lanup"
