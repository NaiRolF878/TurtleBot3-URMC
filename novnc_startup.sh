#!/bin/bash
# =============================================================================
#  noVNC Startup – nur für Windows-Betrieb
#  Aktiviere den noVNC-Block im Dockerfile und docker-compose.yml
# =============================================================================

# VNC-Server starten
mkdir -p ~/.vnc
echo "ros" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

vncserver :1 -geometry 1920x1080 -depth 24 -localhost no

# noVNC starten (Browser-Zugriff auf Port 6080)
websockify --web /usr/share/novnc/ 6080 localhost:5901 &

echo "──────────────────────────────────────────────"
echo "  noVNC gestartet: http://localhost:6080"
echo "  VNC-Passwort:    ros"
echo "──────────────────────────────────────────────"

# Container am Leben halten
wait
