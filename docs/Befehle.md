# Befehle – Kurzübersicht

Alle Befehle laufen **im Devcontainer** vom Repo-Ordner aus (`/workspaces/TurtleBot3-URMC`).
Die Kurzbefehle (`botup`, `lanup` …) stehen in der `~/.bashrc`. Fehlen sie nach einem
Container-Rebuild, geht immer die lange Form `bash scripts/tb/<skript>.sh`.

Vor ROS-Befehlen in einem neuen Terminal: `source scripts/tb/tb-env.sh` (setzt u. a. `ROS_DOMAIN_ID=33`).

`tb` baut beim Start automatisch (`colcon build --symlink-install`, ~2 s) – nach `git pull` oder
Branch-Wechsel ist also nichts extra nötig. Wer ohne `tb` startet: vorher selbst bauen.

---

## Bot starten

### Mit Kabel (LAN) – empfohlen

```bash
lanup          # Laptop bekommt 192.168.77.1 am LAN-Anschluss, prüft ob der Bot antwortet
botup          # Uhr stellen (fragt sudo-Passwort), Bringup + Kamera auf dem Bot starten
```

- Reihenfolge wichtig: **erst `lanup`, dann `botup`** – sonst nimmt `botup` das WLAN.
- Laptop-WLAN für reinen Kabelbetrieb ausschalten (sonst schickt ROS die Kameradaten doppelt).
- Nach Neustart des Laptops ist die LAN-Adresse weg → einfach wieder `lanup`.
- **Einmalig pro Bot** vorher: `lansetup` (gibt dem Bot die feste Adresse 192.168.77.2).

### Ohne Kabel (WLAN)

```bash
botup          # findet den Bot automatisch per MAC-Adresse im WLAN
```

### Beenden

```bash
botdown        # Bringup + Kamera auf dem Bot beenden
landown        # optional: LAN-Adresse am Laptop wieder entfernen
```

---

## Pipeline und Werkzeuge (Menü)

```bash
tb             # Menü mit allen Launchern
```

| Launcher | Was er macht |
|---|---|
| `follow_lane.sh` | komplette Spurverfolgung + **Not-Aus** im selben Terminal – Bot fährt erst nach **Leertaste** |
| `nav2_obstacle.sh` | wie `follow_lane.sh`, dazu Nav2 (Obstacle/Tunnel), RViz und Draufsicht `/nav2/view` |
| `extrinsik_view.sh` | Kamerabild mit 10-cm-Bodengitter + Vogelperspektive, Brett wird live vermessen |
| `rqt_debug.sh` | Bildanzeige (`/lokal/raw_image`, `/lokal/bev`, Debug-Bilder) |
| `rqt_params.sh` | Parameter von `detect_lane` / `control_driving` live einstellen |
| `dataset_capture.sh` | Fotos für einen Datensatz aufnehmen (`p` = Foto, `q` = Ende) |

### Not-Aus (Fahrfreigabe)

Der Bot fährt **nie von selbst** los. Freigabe und Stopp, jeweils umschalten:

- **Leertaste** im Terminal von `follow_lane.sh` (`q` = beenden, sendet Stopp)
- **rqt**: `rqt_params.sh` → Node `notaus` → Häkchen `drive`
- **Befehl**: `ros2 service call /notaus/set std_srvs/srv/SetBool "{data: true}"` (`false` = Stopp)

Zustand: `ros2 topic echo /notaus/drive`. Kommt von `control_driving` 0,5 s nichts, sendet der Not-Aus selbst Stopp.
Läuft der Not-Aus nicht, kommt **kein** Fahrbefehl zum Bot. Kette: `control_driving` → `/control/cmd_vel` → `notaus` → `/cmd_vel`;
Nav2 sendet auf `/nav2/cmd_vel`, `control_driving` reicht das nur im Obstacle-/Tunnel-Modus durch.
Läuft schon eine Pipeline (z. B. in einem anderen Terminal), fragt `follow_lane.sh`, ob sie beendet werden soll –
zwei gleichzeitig führen zu flackernden Debug-Bildern und einem Not-Aus, der dauernd Stopp sendet.
Uhrzeiten im Container und im Debug-Bild sind **UTC** (2 h hinter deutscher Sommerzeit).

Parameter dauerhaft machen bzw. laden:

```bash
bash scripts/tb/save-params.sh detect_lane     # aktuelle Werte in die Parameterdatei schreiben
bash scripts/tb/load-params.sh detect_lane     # Datei in den laufenden Node laden
```

---

## Logs (Ordner `logs/` im Repo)

| Datei | Inhalt |
|---|---|
| `logs/follow_lane-<Zeit>.log` / `follow_lane-latest.log` | Ausgaben aller Knoten der Pipeline (Fehler, Tracebacks) |
| `logs/tb-status-<Zeit>.log` | alle 10 s Knoten/Topic-Raten/Netz, wenn bei `tb` „mitloggen“ = j |

Zeiten in Dateinamen sind UTC (2 h hinter deutscher Sommerzeit).

## Wichtige Topics

| Topic | Inhalt |
|---|---|
| `/camera/image_raw/compressed` | Rohbild vom Bot (800x600, 30 Hz) |
| `/lokal/raw_image` | entzerrtes Bild (ingress_node) |
| `/lokal/bev` | **Vogelperspektive**, 5–60 cm vor der Radachse, ±40 cm, 1,25 mm/px (640x440) |
| `/detect/lane` | Spurfehler für den Regler |
| `/control/mode` | Fahrmodus von `switch_control` (1 Lane, 2 Obstacle, 7 TunnelEntry, 6 Tunnel) |
| `/nav2/cmd_vel` | Fahrbefehle von Nav2 (`controller_server`) an `control_driving` |
| `/control/cmd_vel` | Fahrbefehle von `control_driving` an den Not-Aus |
| `/odom`, `/cmd_vel`, `/battery_state` | vom Bringup (`turtlebot3_node`) |

Anschauen: `ros2 run rqt_image_view rqt_image_view /lokal/bev`

---

## Bot, Kamera, Netz

```bash
botssh                         # SSH auf den Bot (findet die IP selbst)
botssh <befehl>                # Befehl auf dem Bot ausführen
lanstatus                      # Kabelverbindung prüfen
botcheck                       # liefert die Kamera auf dem Bot selbst Bilder?
bash scripts/tb/cam-mode.sh lane   # nur Kamera neu starten (Werte aus camera.conf)
bash scripts/tb/sync-time.sh       # nur Bot-Uhr stellen (danach Bringup/Kamera neu starten)
bash scripts/tb/diag.sh            # Diagnose sammeln -> /tmp/tb-diag.txt
```

Kamera-Einstellungen: `scripts/tb/camera.conf` – **bleibt bei 800x600** (Entscheidung 20.10.).
Welcher Bot: `scripts/tb/bot.conf`.

---

## Kalibrierung (nur bei Umbau der Kamera nötig)

```bash
python3 scripts/tb/camera-intrinsics.py               # Intrinsik (ChArUco-Brett, Fisheye)
python3 scripts/tb/camera-extrinsics.py               # Höhe/Neigung/Roll messen (Brett flach am Boden)
python3 scripts/tb/camera-extrinsics.py --recompute --preview   # nur neu rechnen + Vorschau
bash scripts/tb/launchers/extrinsik_view.sh           # live prüfen
```

- `forward` (Linse vor Radachse, 5,79 cm) und `yaw` (1,4°) bleiben bei `--recompute` erhalten.
- Ablauf und Messwerte: `docs/bev-tests/README.md`.

---

## Wenn etwas nicht geht

| Symptom | Ursache / Abhilfe |
|---|---|
| `/scan` da, aber `/odom` und `/cmd_vel` fehlen | Bringup halb oben, meist **Akku** leer → Akku prüfen, `botup` |
| Bot reagiert nicht auf Fahrbefehle | Motoren aus (`torque: false` in `/sensor_state`) → `ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"` |
| `botup` bricht mit „Host key verification failed“ ab | neue Adresse unbekannt → `ssh-keyscan -H 192.168.77.2 >> ~/.ssh/known_hosts` |
| `ros2 topic list` fast leer | `--no-daemon` anhängen; Domain prüfen (`echo $ROS_DOMAIN_ID` muss 33 sein) |
| OpenCR nicht gefunden (`ttyACM0`) | nach Reset liegt er auf `ttyACM1` |
| rviz zeigt keine LIDAR-Punkte | Bot-Uhr falsch → `sync-time.sh`, danach Bringup neu starten |
