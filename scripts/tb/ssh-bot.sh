#!/bin/bash
# Findet den TurtleBot3 im aktuellen Netz und verbindet per SSH.
# Aufruf: bash ssh-bot.sh            -> interaktive Shell auf dem Bot
#         bash ssh-bot.sh <befehl>   -> Befehl auf dem Bot ausfuehren

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BOT_USER="gertrude"

BOT_IP=$(bash "$SCRIPT_DIR/find-bot-ip.sh") || exit 1

ssh "$BOT_USER@$BOT_IP" "$@"
