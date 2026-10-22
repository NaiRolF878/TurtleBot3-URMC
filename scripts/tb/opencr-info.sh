#!/bin/bash
# Analyse der OpenCR-Firmware -- NUR LESEN, aendert nichts auf dem Bot.
# Aufruf: bash opencr-info.sh
#
# Sammelt:
#   * welche Firmware-Dateien/Werkzeuge auf dem Pi liegen (opencr_update, *.opencr, dfu-util)
#   * wie das OpenCR am USB haengt (ttyACM*, USB-ID)
#   * Modell und Firmware-Version direkt aus dem OpenCR (Register 0-1 model_number,
#     Register 6 firmware_version; Adressen aus turtlebot3_node/control_table.hpp)
#
# Das Register-Auslesen braucht den seriellen Port exklusiv. Laeuft das Bringup, wird es
# NICHT gestoppt -- der Schritt wird uebersprungen. Vorher selbst botdown, dann erneut.
# Geschickt werden ausschliesslich Dynamixel-READ-Pakete (Instruktion 0x02), kein WRITE.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/bot.conf" || exit 1

BOT_IP=$(bash "$SCRIPT_DIR/find-bot-ip.sh") || exit 1

echo "=== OpenCR-Analyse auf $BOT ($BOT_IP) -- nur lesen ==="
ssh "$BOT_USER@$BOT_IP" "bash -s" <<'EOF'
echo
echo "--- Firmware-Dateien und Werkzeuge auf dem Pi ---"
ls -d "$HOME"/opencr_update* 2>/dev/null || echo "  kein ~/opencr_update*"
find "$HOME" -maxdepth 4 \( -name "*.opencr" -o -name "opencr_ld*" -o -name "update.sh" \) 2>/dev/null \
    | grep -i opencr | sed 's/^/  /' | head -20
for f in $(find "$HOME" -maxdepth 4 -name "*.opencr" 2>/dev/null | head -5); do
    echo "  $(ls -l --time-style=long-iso "$f" | awk '{print $5" Byte, "$6" "$7}')  md5 $(md5sum "$f" | cut -c1-12)  $f"
done
command -v dfu-util >/dev/null && echo "  dfu-util: $(dfu-util --version 2>&1 | head -1)" || echo "  dfu-util: nicht installiert"

echo
echo "--- OpenCR am USB ---"
ls -l /dev/ttyACM* 2>/dev/null | sed 's/^/  /' || echo "  kein /dev/ttyACM*"
command -v lsusb >/dev/null && lsusb | grep -i -E "0483:|robotis|stm" | sed 's/^/  /'

echo
echo "--- Laeuft das Bringup? ---"
if pgrep -f "turtlebot3_ros|robot.launch.py" >/dev/null; then
    echo "  JA -- der Port ist belegt, Register werden NICHT gelesen."
    echo "  (Zum Auslesen: botdown, dann dieses Skript erneut starten.)"
    exit 0
fi
echo "  nein -- Port frei."

echo
echo "--- Register aus dem OpenCR (nur READ) ---"
python3 - <<'PY'
import glob, struct, sys, time
try:
    import serial
except ImportError:
    print("  pyserial fehlt (python3-serial) -- Register koennen nicht gelesen werden."); sys.exit(0)

def crc16(data):
    # Dynamixel-Protokoll 2.0: CRC-16, Polynom 0x8005, Start 0, nicht reflektiert
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc

def read(port, dxl_id, addr, length):
    body = bytes([dxl_id]) + struct.pack('<H', 7) + bytes([0x02]) + struct.pack('<HH', addr, length)
    pkt = b'\xFF\xFF\xFD\x00' + body
    pkt += struct.pack('<H', crc16(pkt))
    port.reset_input_buffer(); port.write(pkt)
    # Antwort: Header(4) ID(1) LEN(2) INST 0x55(1) ERR(1) DATA(length) CRC(2)
    resp = port.read(11 + length)
    if len(resp) < 11 + length or resp[:4] != b'\xFF\xFF\xFD\x00' or resp[7] != 0x55:
        raise IOError(f"keine gueltige Antwort ({resp.hex() or 'leer'})")
    if crc16(resp[:-2]) != struct.unpack('<H', resp[-2:])[0]:
        raise IOError("CRC-Fehler in der Antwort")
    if resp[8]:
        raise IOError(f"OpenCR meldet Fehlercode {resp[8]}")
    return resp[9:9 + length]

for dev in sorted(glob.glob('/dev/ttyACM*')):
    try:
        with serial.Serial(dev, 1000000, timeout=0.5) as p:
            time.sleep(0.2)
            model = struct.unpack('<H', read(p, 200, 0, 2))[0]
            info = struct.unpack('<I', read(p, 200, 2, 4))[0]
            fw = read(p, 200, 6, 1)[0]
            print(f"  {dev}: model_number = {model}, model_information = {info}, firmware_version = {fw}")
            print("  (Quellcode ROBOTIS-GIT/OpenCR master, Aug 2023: FIRMWARE_VER = 5)")
            break
    except Exception as e:
        print(f"  {dev}: {e}")
PY

echo
echo "--- Bringup-Log (letzter Start) ---"
grep -i -E "firmware|version|model|opencr|connect" "$HOME/logs/bringup.log" 2>/dev/null | head -10 | sed 's/^/  /' \
    || echo "  kein ~/logs/bringup.log"
EOF
