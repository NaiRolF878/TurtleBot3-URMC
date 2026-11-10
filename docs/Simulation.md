# Simulation (2D, ohne Gazebo)

Branch `Simulation`, Paket `workspace/src/urmc_sim`. Der Simulator ersetzt den Bot: er faehrt
nach `/cmd_vel` und sendet, was der Bot sendet. Die Pipeline eines Arbeitsbranches
(`Nav2-Costmap-Fix`, `LDS03-Doku`, `Storage-Retrieval`) laeuft unveraendert dagegen. Der
Branch wird nie in die Arbeitsbranches gemergt.

## Stand

| Schritt | Inhalt | Stand |
|---|---|---|
| 1 | Fahren, `/joint_states`, `/imu`, `/scan` (LDS-03), TF des Bots, Tunnelwelten | fertig |
| 2 | Kamerabild mit unserer Kalibrierung, Streckenboden | offen |
| 3 | Lagerbereich nach Reglement Abbildung 5 | offen |

Ohne Kamera laufen Spurerkennung, Zonen- und Palettenerkennung noch nicht.

## Einrichten (einmalig, Windows)

1. WSL2 mit Ubuntu 22.04 (unter Windows 10: WSL aus dem Microsoft Store, dann laufen
   `rqt_image_view` und RViz als Fenster).
2. In Ubuntu ROS 2 Humble mit `tb3-wsl-setup.sh` der anderen Gruppe.
3. Danach:
   ```bash
   bash /mnt/c/Users/Flori/Documents/Florian/Master/Turtlebot/wt-sim/scripts/sim/sim-setup.sh
   ```
   installiert die Nav2-Plugins aus unserer `nav2_params.yaml` (MPPI, RotationShim) und legt
   `~/urmc_sim_ws` an. Auf CPUs ohne AVX2/FMA (z.B. Pentium 3558U) stuerzt das MPPI aus apt
   ab (`controller_server`, Code -4); dann baut `sim-setup.sh` es mit `build-mppi.sh` selbst
   nach `~/nav2_mppi_ws` (einige Minuten), `sim-env.sh` bindet es ein.

## Starten

```bash
cd /mnt/c/Users/Flori/Documents/Florian/Master/Turtlebot/wt-sim
bash scripts/sim/sim.sh wt-nav2 tunnel_gap
```

- Erstes Argument: der Worktree-Ordner, dessen Code getestet wird (neben `wt-sim`).
- Zweites Argument: die Welt aus `urmc_sim/worlds`.
- `sim.sh` baut in `~/urmc_sim_ws` (Quellen bleiben verlinkt in den Worktrees), startet
  Simulator und Pipeline im Hintergrund und den Not-Aus im Vordergrund.
- Die Simulation spricht nur auf diesem Rechner (`ROS_LOCALHOST_ONLY=1`, Domain 99). Ein
  zweites Terminal braucht dieselbe Umgebung: `source scripts/sim/sim-env.sh`.
- Log der Knoten: `wt-sim/logs/sim-<worktree>-<welt>-<zeit>.log`.

## Bedienen

| Was | Wie |
|---|---|
| Fahren freigeben / stoppen | LEERTASTE im Not-Aus |
| Tunnel starten | im Debugbild TUNNEL STARTEN oder `ros2 topic pub --once /control/tunnel_request std_msgs/msg/Bool '{data: true}'` |
| Zurueck an den Start | `ros2 topic pub --once /sim/reset std_msgs/msg/Empty '{}'`, danach L im Not-Aus |
| Welt von oben | Fenster `/sim/view`: Objekte (hellgrau: der LiDAR sieht sie), Bot gruen, bei Kollision rot, LiDAR-Treffer rot |

Ohne Kamerabild schaltet TunnelEntry nach 1 s auf Tunnel.

## Welten

| Welt | Inhalt |
|---|---|
| `tunnel_gap` | Tunnel 1,0 x 1,0 m, Eingang unten, Ausgang rechts oben, Trennwand mit 22 cm Luecke |
| `tunnel_wide` | wie `tunnel_gap`, Luecke 30 cm (Vergleich) |
| `tunnel_wide_room` | wie `tunnel_wide`, dazu Waende um die Plattform (4 x 3 m) |

Der Bot startet mit der Radachse 10 cm vor dem Eingang: dort enden am echten Bot die
Spurlinien, und er schaltet auf Tunnel. Ohne Waende um die Plattform liefert der LiDAR
draussen kein Echo (inf); mit ihnen sieht er ueberall eines. Welcher Fall bei der Challenge
gilt, haengt von der Halle ab.

Format (Meter, Draufsicht, x nach rechts, y nach oben):
```yaml
start: [x, y, yaw_deg]           # Radachse, Blickrichtung
objects:
  - {type: box, center: [x, y], size: [lx, ly], yaw_deg: 0, height: 0.25}
  - {type: cylinder, center: [x, y], radius: 0.03, height: 0.25}
```
Der LiDAR sieht nur Objekte ueber seiner Ebene (18,2 cm); anstossen kann der Bot an alle.

## Was der Simulator nachbildet

- Differentialantrieb des Burger: 0,22 m/s, 2,84 rad/s, Radgrenze, Motoren mit 0,1 s
  Verzoegerung; ohne Befehl haelt er nach 0,5 s an.
- Kollision mit dem Umriss aus `nav2_params.yaml`: der Bot bleibt stehen, Meldung im Log.
- LDS-03: 400 Strahlen (0,9 Grad), 0,05 bis 12 m, 10 Hz, Streuung `scan_noise_m`
  (`config/sim.yaml`).
- `/joint_states` und `/imu` wie der Bot; die odom-TF rechnet `odom_filter` daraus.
- Kein Schlupf, keine Ausreisser in `/joint_states`, Echtzeit statt Simulationszeit.
