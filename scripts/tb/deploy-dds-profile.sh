#!/bin/bash
# Kopiert das FastDDS-Profil auf den Bot und traegt es dort dauerhaft in die .bashrc ein.
# Aufruf: bash deploy-dds-profile.sh
#
# Hintergrund: Fragmentiert wird auf der SENDER-Seite. Kamera und LIDAR senden vom
# Bot, also muss vor allem dort das Profil aktiv sein - sonst bringt die Einstellung
# auf dem Laptop fuer genau die Topics nichts, um die es geht.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PROFILE="$REPO_ROOT/.devcontainer/fastdds_profile.xml"
source "$SCRIPT_DIR/bot.conf" || exit 1
REMOTE_PATH="/home/$BOT_USER/fastdds_profile.xml"

if [ ! -f "$PROFILE" ]; then
    echo "Profil nicht gefunden: $PROFILE"
    exit 1
fi

BOT_IP=$(bash "$SCRIPT_DIR/find-bot-ip.sh") || exit 1

echo "Kopiere Profil nach $BOT_USER@$BOT_IP:$REMOTE_PATH ..."
scp "$PROFILE" "$BOT_USER@$BOT_IP:$REMOTE_PATH" || exit 1

echo "Trage Export in die .bashrc des Bots ein (nur falls noch nicht vorhanden) ..."
ssh "$BOT_USER@$BOT_IP" \
    "grep -q FASTRTPS_DEFAULT_PROFILES_FILE ~/.bashrc \
     || echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=$REMOTE_PATH' >> ~/.bashrc" || exit 1

echo ""
echo "Fertig. WICHTIG: Auf dem Bot alle laufenden ROS2-Prozesse (Bringup, Kamera)"
echo "beenden und in einem NEUEN Terminal neu starten - die .bashrc wirkt nur"
echo "fuer neu gestartete Shells, und das Profil wird beim Prozessstart gelesen."
