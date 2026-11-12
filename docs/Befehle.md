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
| `follow_lane_record.sh` | wie `follow_lane.sh`, dazu während der Fahrt jede Sekunde ein YOLO-Trainingsbild, abwechselnd Original (`_roh`) und entzerrt (`_entzerrt`), nach `dataset/fahrt_<datum>_<zeit>/` (nicht im Stand/Tunnel) |
| `nav2_obstacle.sh` | wie `follow_lane.sh`, dazu Nav2 (Obstacle/Tunnel), RViz und Draufsicht `/nav2/view` |
| `storage.sh` | Storage-Aufgabe (Palette): Spurverfolgung + Storage-Detektoren, ohne Nav2. Bot steht, bis im Debugbild **STORAGE STARTEN** geklickt wird |
| `extrinsik_view.sh` | Kamerabild mit 10-cm-Bodengitter + Vogelperspektive, Brett wird live vermessen |
| `rqt_debug.sh` | Bildanzeige (`/lokal/raw_image`, `/lokal/bev`, Debug-Bilder) |
| `rqt_params.sh` | Parameter von `detect_lane` / `control_driving` live einstellen |
| `dataset_capture.sh` | Fotos für einen Datensatz aufnehmen (`p` = Foto, `q` = Ende) |

### Not-Aus (Fahrfreigabe)

Der Bot fährt **nie von selbst** los. Freigabe und Stopp, jeweils umschalten:

- **Leertaste** im Terminal von `follow_lane.sh` (`q` = beenden, sendet Stopp)
- **rqt**: `rqt_params.sh` → Node `notaus` → Häkchen `drive`
- **Befehl**: `ros2 service call /notaus/set std_srvs/srv/SetBool "{data: true}"` (`false` = Stopp)

**Taste L** im Terminal von `nav2_obstacle.sh`: Spurgedächtnis und beide Costmaps leeren, wenn sie nicht mehr zur Odometrie passen (Bot angehoben). Erst absetzen, dann im Stand drücken.

**Taste O** im Not-Aus-Terminal: Obstacle-Modus an/aus (`obstacle_enabled` von `switch_control`, auch als Häkchen in rqt). Start: `nav2_obstacle.sh` an, `follow_lane.sh` aus. Aus heißt: kein Wechsel nach Obstacle, aus Obstacle sofort zurück zu Lane.

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

| Ordner / Datei | Inhalt |
|---|---|
| `logs/pipeline/follow_lane-<Zeit>.log`, `nav2_obstacle-<Zeit>.log`, `*-latest.log` | Ausgaben aller Knoten der Pipeline (Fehler, Tracebacks); `-latest` zeigt auf den letzten Lauf |
| `logs/status/tb-status-<Zeit>.log` | alle 10 s Knoten/Topic-Raten/Netz, wenn bei `tb` „mitloggen“ = j |
| `logs/bags/` | Aufzeichnungen (`ros2 bag record -o logs/bags/bag-<Zeit> ...`), lesbar mit `scripts/tb/bag-to-text.py` |
| `logs/kalibrierung/` | Bilder und Aufnahmen der Kamerakalibrierung (Intrinsik-Archive, Vogelperspektive, Rasterbilder) |
| `logs/bilder/` | Screenshots und Skizzen |
| `logs/notizen/` | lose Notizen |
| `logs/storage-<Zeit>.log` / `storage-latest.log` | Ausgaben der Pipeline bei `storage.sh` |

Zeiten in Dateinamen sind UTC (Winterzeit 1 h, Sommerzeit 2 h hinter deutscher Zeit).

## Wichtige Topics

| Topic | Inhalt |
|---|---|
| `/camera/image_raw/compressed` | Rohbild vom Bot (800x600, 30 Hz) |
| `/lokal/raw_image` | entzerrtes Bild (ingress_node) |
| `/lokal/bev` | **Vogelperspektive**, 5–60 cm vor der Radachse, ±40 cm, 1,25 mm/px (640x440) |
| `/detect/lane` | Spurfehler für den Regler |
| `/control/mode` | Fahrmodus von `switch_control` (1 Lane, 2 Obstacle, 3 Stop, 7 TunnelEntry, 6 Tunnel, 8 Dock, 9 Maneuver) |
| `/control/storage_state`, `/control/storage_info` | Storage: Ablaufschritt und Klartext (Zonen, Ziel, Rueckmeldungen, Fehlergrund) |
| `/detect/pallet`, `/detect/pallet_grip` | Storage: Fuellung der Zone und Vorderkante der Palette; Korrelation beim Griff |
| `/detect/zone_debug/compressed` | Storage: Zonenlinien, Zonenmodell, Boden, Vorderkante der Palette mit Rahmen und Abstand |
| `/detect/pallet_debug/compressed`, `/detect/storage_sign_debug/compressed` | Storage: Magnetlage und Griff; Schilder |
| `/nav2/cmd_vel` | Fahrbefehle von Nav2 (`controller_server`) an `control_driving` |
| `/control/cmd_vel` | Fahrbefehle von `control_driving` an den Not-Aus |
| `/control/clear_maps` | Taste L: `lane_costmap_node` leert Spurgedächtnis und Costmaps |
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

## GPU im Container (TensorRT / YOLO)

Einmalig **auf dem Host** (nicht im Container), Ubuntu mit installiertem NVIDIA-Treiber
(`nvidia-smi` muss auf dem Host laufen):

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --gpus all ubuntu nvidia-smi     # Test: muss die GPU zeigen
```

Danach in VS Code „Dev Containers: Rebuild Container“. `devcontainer.json` fordert die GPU mit
`hostRequirements.gpu: "optional"` an -- ohne GPU/Toolkit (z.B. anderer Laptop) startet der
Container trotzdem, nur ohne GPU. Im Container prüfen: `nvidia-smi`,
`python3 -c "import tensorrt as trt; print(trt.__version__)"`.

TensorRT-Engines (`.engine`) sind an GPU und TensorRT-Version gebunden: auf dem Rechner bauen,
auf dem sie laufen (z.B. `YOLO('best.pt').export(format='engine')`). Auf dem Pi des Bots gibt es
kein TensorRT.

### YOLO-Engine bauen (im Container, ohne torch)

Aus der ONNX-Datei mit der TensorRT-Version des Containers (~4 min, FP16):

```bash
cd Yolo/yolo26s_800/weights
python3 -c "
import tensorrt as trt
L = trt.Logger(trt.Logger.WARNING); b = trt.Builder(L)
net = b.create_network(0); p = trt.OnnxParser(net, L)
assert p.parse(open('best.onnx', 'rb').read())
c = b.create_builder_config(); c.set_flag(trt.BuilderFlag.FP16)
open('best_trt10.15_fp16.engine', 'wb').write(b.build_serialized_network(net, c))"
```

`detect_lane_nav2` lädt `YOLO_ENGINE` (Konstante oben in der Datei, relativ zum Repo-Ordner) und
sendet die Boxen auf `/detect/objects` (vision_msgs/Detection2DArray, Pixel im Rohbild). Die
Rohbild-Ansicht `/lokal/debug_view_raw` zeichnet sie ein. Ohne Engine/GPU läuft der Node ohne YOLO
(Warnung im Log).
