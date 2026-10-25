# Obstacle- und Tunnel-Modus – offene Punkte (Stand 23.10.2026)

Branch `Nav2-Costmap-Fix`. Gesammelt aus den Tests vom 22./23.10. –
**beobachtet** = am Bot gesehen, **vermutet** = Ursache aus Code/Konfiguration abgeleitet, noch nicht bestätigt.

---

## Obstacle-Modus

### 1. Karte wird beim Ausweichen teilweise gelöscht – **beobachtet**, Gegenmaßnahme **ungetestet**

Beim Wechsel Lane ↔ Obstacle verschwanden Teile der eingetragenen Spurwände aus der `local_costmap`, und
der Bot fuhr über Linien.

- Ursache (vermutet): Die Wolke enthielt nur das aktuelle Kamerabild. `lane_layer` mit `clearing: true`
  zieht Strahlen vom Bot zu jedem Linienpunkt und löscht alles auf dem Weg; dreht sich der Bot, wischen
  diese Strahlen über vorher eingetragene Spurwände.
- Gegenmaßnahme: `lane_costmap_node` führt ein **Spurgedächtnis** (odom-Raster, Zelle wechselt erst nach
  3 bzw. 10 Bildern in Folge, verdeckte Linien werden nicht vergessen) und sendet jedes Mal alle gemerkten
  Linien. Was gelöscht wird, wird im selben Zyklus wieder eingetragen. Details:
  `docs/Nav2_Costmap_Werte.md`, Abschnitt 4. Am Bot zu prüfen.
- Nicht damit verwechseln: das Gedächtnis reicht 2 m um den Bot – was weiter weg ist, fällt gewollt heraus.

### 2. Wann der Obstacle-Modus anspringt (Clearance)

In `switch_control`, Werte aus `Parameter/Switch_Control.yaml` (live in rqt änderbar):

| Parameter | Wert | Bedeutung |
|---|---|---|
| `obstacle_trigger_distance` | **0,20 m** (Code-Default 0,4) | LiDAR-Punkt im Prüfkegel näher als das → Obstacle, wenn keine Spurlinie dazwischen liegt |
| `obstacle_check_angle_deg` | 20° | halber Öffnungswinkel nach vorn (±20°) |
| `obstacle_clear_distance` / `obstacle_clear_angle_deg` | 0,20 m / 135° | Austritt: nichts näher als das im Bereich ±135° (270°, ohne den Keil hinten) |
| `obstacle_clear_debounce` | 1,0 s | so lange frei, bevor zurück zu Lane |

Im Obstacle-Modus plant `planner_server` (NavFn) jede Sekunde einen Weg um die Hindernisse zu einem Punkt
1 m voraus auf der Spurmitte; MPPI fährt ihn ab. Am Bot noch nicht getestet.

Beim Wechsel Lane → Obstacle sendet `control_driving` sofort Null; der Bot steht, bis Nav2 den ersten
Befehl schickt.

Abstand beim Umfahren selbst: `inflation_radius` der Costmaps (lokal 0,20 m, global 0,3 m,
`cost_scaling_factor` 3,0). Noch nicht am Bot abgestimmt.

### 3. Fahrgeschwindigkeit über dem Limit des Burger – **beobachtet** (Lane-Modus)

Aufzeichnung `logs/bag-20261022-132414`: `control_driving` verlangte bis 0,40 m/s am äußeren Rad,
der Burger schafft 0,22 m/s. Das schnellere Rad blieb stehen, der Bot drehte entgegen dem Lenkbefehl.
`MAX_VEL` wurde auf 0,15 gesetzt; ob das Problem damit weg ist, ist **noch nicht im Fahrtest bestätigt**.
Nav2 selbst ist auf `vx_max: 0.22` begrenzt (betrifft nur Obstacle/Tunnel).

### 4. Feinere Costmap – **ungetestet**

`local_costmap.resolution` 0,05 → 0,025 m (~4× Zellen). Prüfen, ob `controller_server` die 20 Hz hält
(Warnung „Control loop missed its desired rate“ in `nav2_view` bzw. `logs/nav2_obstacle-latest.log`).
Zurück: Wert in `nav2_params.yaml` auf 0,05.

---

## Tunnel-Modus

### 5. Starten

- Von Hand: im `rqt_image_view` mit `/lokal/debug_view` „Publish click location“ anhaken, Knopf
  **TUNNEL STARTEN** im Bild klicken.
- Per Befehl: `ros2 topic pub --once /control/tunnel_request std_msgs/msg/Bool "{data: true}"`
- Automatisch: Schild-Wert 6 auf `/detect/sign`.
- Kontrolle: `ros2 topic echo /control/mode` → 7 (Einfahrt), dann 6. Not-Aus muss auf FAHREN stehen.
- Im Tunnel verarbeiten `ingress_node` und `detect_lane` (Pipeline-Kopien `*_nav2`) keine Kamerabilder;
  das Debugbild zeigt „Kamera aus (Tunnel)“. Der Kameratreiber auf dem Bot sendet trotzdem weiter.
- Ablauf: Auslösen setzt `TunnelEntry` (Einfahrt mit Kamera, NavFn plant durch die Öffnung). Sieht der LiDAR
  links und rechts die Tunnelwände (`tunnel_side_distance` 0,35 m), folgt ~20 cm vor der Öffnung `Tunnel`:
  Kamera aus, Karten geleert, Eingangssperre an der Öffnung. Am Bot noch nicht getestet.
- Verlassen: automatisch, wenn der LiDAR rundum frei ist (erst ab `Tunnel` geprüft) (`tunnel_exit_open_distance` 1,5 m,
  `tunnel_exit_open_ratio` 0,8, `tunnel_exit_debounce` 1 s), oder Knopf/Befehl mit `false`.

### 6. Beim Betreten werden beide Karten geleert – **so gewollt**, beobachten

`lane_costmap_node` leert die `local_costmap`, `keepout_zone_node` die `global_costmap`. Danach baut
der LiDAR die Tunnelkarte neu auf. Alles, was vorher über die Umgebung bekannt war, ist dann weg.

### 7. Feinere Tunnelkarte – **ungetestet**

`global_costmap.resolution` 0,05 → 0,025 m (280×280 Zellen). Damit die Frontier-Suche gleich bleibt,
`tunnel_explorer_node.min_frontier_cluster_cells` 6 → 12 (`Lane_Costmap.yaml`). Prüfen: Kommt der Pfad
rechtzeitig, warnt `planner_server`? Zurück: beide Werte zurücksetzen (Kommentar in der YAML).

---

## Umgebung (betrifft beide Modi)

- **Nach Branch-Wechsel:** `colcon build --symlink-install` für **alle** Pakete (auch `follow_lane_msg`),
  sonst fehlen Launch-Dateien/Nachrichten (`LaneMask`) und Knoten stürzen beim Start ab.
- **`lifecycle_manager` startet nicht (exit 127):** `ros-humble-diagnostic-updater` 4.0.6 statt 4.0.7.
  Fix: `apt-get update && apt-get install --only-upgrade ros-humble-diagnostic-updater`.
  Geht bei jedem Container-Rebuild verloren, solange es nicht in `devcontainer.json` steht.
- **`nav2_view`** sendet unkomprimiert auf `/nav2/view` (mit `/nav2/view/compressed` stürzte
  `rqt_image_view` ab). Farben: grün befahrbar, rot blockiert, grau unbekannt, gelb/weiß Spurlinien.
- **Kamera** (23.10.): IMX219 wurde nach Neustart nicht mehr erkannt (`detected=0`, kein `imx219` in
  `dmesg`) – ohne Kamera keine Spurwände, Obstacle/Tunnel sehen dann nur den LiDAR. Noch offen.
