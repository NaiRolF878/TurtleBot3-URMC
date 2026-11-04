# Konzept des Branches `Nav2-Costmap-Fix`

Spurfolge mit PID, Hindernisse auf der Strecke und Tunnel mit **Nav2** (Costmap, NavFn,
MPPI). Im Konzeptpapier auf `main` (`docs/URMC–Technisches_Konzept.md`, Abschnitt 3)
entspricht das der Stufe „Fortgeschritten“ (Nav2 Costmap), mit MPPI statt DWA. Die
Variante mit Zonenaufteilung fuer den Obstacle-Modus liegt auf `LDS03-Doku`.

## Architektur

- **switch_control** entscheidet als einziger ueber den Fahrmodus (`/control/mode`).
- **control_driving** erzeugt als einziger Fahrbefehle (`/control/cmd_vel`): PID oder
  die Befehle von Nav2, auf die Radgrenze skaliert.
- **notaus** gibt sie nach Freigabe an den Bot weiter (`/cmd_vel`).
- Alle anderen Nodes liefern nur Daten: Spurfehler, Spurgedaechtnis, Pfade, Karten.

| Modus         | Ausloeser (switch_control)                                         | Fahrbefehl (control_driving) |
|---------------|--------------------------------------------------------------------|------------------------------|
| `Lane`        | Start, Ende von Obstacle; nach dem Tunnel der Modus davor          | PID auf den Spurfehler       |
| `Obstacle`    | LiDAR-Punkt im Kegel voraus naeher als `obstacle_trigger_distance`, keine Spurlinie dazwischen | Nav2 (`/nav2/cmd_vel`) |
| `TunnelEntry` | Button im Debugbild oder Tunnelschild                              | PID auf den Spurfehler       |
| `Tunnel`      | Beide Spurlinien enden kurz vor dem Bot                            | Nav2 (`/nav2/cmd_vel`)       |
| `Stop`        | Storage: Start mit `storage.launch.py`, Schild lesen, Greifen, Ablegen | Null                     |
| `Dock`        | Storage: Feinanfahrt an die Palette                                | Regler auf `/detect/pallet`  |
| `Maneuver`    | Storage: Drehen, Strecke, Punkt anfahren                           | Regler in odom (`/control/maneuver`) |

Jeder andere Modus, veraltete Eingangsdaten oder ein fehlendes `/control/mode`
fuehren zu einem Nullbefehl.

```
detect_lane_nav2 --/detect/lane----------------------.
controller_server --/nav2/cmd_vel------------------. |
                                                   v v
switch_control --/control/mode----------> control_driving --/control/cmd_vel--> notaus --/cmd_vel--> Bot

lane_costmap_node --Spurwaende, Spurmitte--> nav2_goal_publisher --Ziel/Pfad--> planner_server, controller_server
```

## Challenges

**Spurfolge (Lane).** Kamera -> metrische Vogelperspektive (ingress_node_nav2) ->
Spurerkennung in LAB (detect_lane_nav2) -> PID in control_driving. Kalibrierung:
`Kalibrierung_Checkliste.md`.

**Hindernisse auf der Strecke (Obstacle).** lane_costmap_node merkt sich die gelbe und
die weisse Spurlinie als virtuelle Waende (verfolgt, Querlinien und helle Flaechen
nicht), NavFn plant zwischen ihnen um das Hindernis herum, MPPI faehrt
den Weg ab. Kollision ueber den Umriss des Burgers, unabhaengig von der Spurbreite.

**Tunnel (TunnelEntry, Tunnel).** Einfahrt mit Kamera und PID auf der 20-cm-Spur (dort
passt der Bot fuer Nav2 nicht zwischen die Spurwaende), bis die Linien an der Oeffnung
enden. Ab dann nur LiDAR (Kamera laut Reglement verboten): tunnel_explorer_node findet die
Tuer im Scan und faehrt hindurch, keepout_zone_node sperrt sie danach. Die Waende bilden
eine Huelle: Strahlen durch die Huelle nach aussen zeigen auf den Ausgang, Graues innerhalb
ist Schatten hinter Hindernissen. Draussen ist der Bot, wenn er die Huelle ganz verlassen
hat. Das alles kommt ohne Masse der Enclosure aus.

**Storage (Palette, Stop/Dock/Maneuver).** Nur Kamera und Odometrie: Erkundung entlang
der Spur, Schilder lesen, Palette per Kamera andocken und greifen, per odom in die
Zielzone. Eigenes Konzept: `Storage_Retrieval_Konzept.md`.

## Detaildoku

| Datei | Inhalt |
|-------|--------|
| `Nav2LocalCoastmap_Doku.md` | Fahrmodi, Nodes, Parameter, Ablauf von Obstacle und Tunnel |
| `Nav2_Costmap_Werte.md` | Herkunft und Abhaengigkeiten der Costmap- und MPPI-Werte |
| `Obstacle_Tunnel_Offene_Punkte.md` | Testbeobachtungen und offene Punkte |
| `Kalibrierung_Checkliste.md` | Kamera, BEV und Live-Test Schritt fuer Schritt |
| `Storage_Retrieval_Konzept.md` | Storage-Aufgabe: Ablauf, Zustaende, Topics, Entscheidungen |
| `Befehle.md` | Start, Not-Aus, Topics, Werkzeuge |
| `challenges.md` | Ueberblick ueber die Wettbewerbsaufgaben |

## Start

```bash
bash scripts/tb/launchers/nav2_obstacle.sh   # alles inkl. Nav2, Not-Aus im Vordergrund
bash scripts/tb/launchers/follow_lane.sh     # nur Spurfolge, ohne Nav2
bash scripts/tb/launchers/storage.sh         # Storage-Aufgabe, Start per Button im Debugbild
```
