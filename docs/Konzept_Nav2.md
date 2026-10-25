# Konzept des Branches `Nav2-Costmap-Fix`

Spurfolge mit PID, Hindernisse auf der Strecke und Tunnel mit **Nav2** (Costmap, NavFn,
MPPI). Im Konzeptpapier auf `main` (`docs/URMC–Technisches_Konzept.md`, Abschnitt 3)
entspricht das der Stufe „Fortgeschritten“ (Nav2 Costmap), mit MPPI statt DWA. Die
Variante mit Zonenaufteilung fuer den Obstacle-Modus liegt auf `LDS03-Doku`.

## Architektur

- **switch_control** entscheidet als einziger ueber den Fahrmodus (`/control/mode`).
- **control_driving** erzeugt als einziger Fahrbefehle (`/control/cmd_vel`): PID oder
  die unveraenderten Befehle von Nav2.
- **notaus** gibt sie nach Freigabe an den Bot weiter (`/cmd_vel`).
- Alle anderen Nodes liefern nur Daten: Spurfehler, Spurgedaechtnis, Pfade, Karten.

| Modus         | Ausloeser (switch_control)                                         | Fahrbefehl (control_driving) |
|---------------|--------------------------------------------------------------------|------------------------------|
| `Lane`        | Start, Ende von Obstacle; nach dem Tunnel der Modus davor          | PID auf den Spurfehler       |
| `Obstacle`    | LiDAR-Punkt im Kegel voraus naeher als `obstacle_trigger_distance`, keine Spurlinie dazwischen | Nav2 (`/nav2/cmd_vel`) |
| `TunnelEntry` | Button im Debugbild oder Tunnelschild                              | Nav2 (`/nav2/cmd_vel`)       |
| `Tunnel`      | LiDAR sieht die Tunnelwaende oder Einfahrtsstrecke erreicht        | Nav2 (`/nav2/cmd_vel`)       |

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

**Hindernisse auf der Strecke (Obstacle).** lane_costmap_node merkt sich die Spurlinien
als virtuelle Waende, NavFn plant zwischen ihnen um das Hindernis herum, MPPI faehrt
den Weg ab. Kollision ueber den Umriss des Burgers, unabhaengig von der Spurbreite.

**Tunnel (TunnelEntry, Tunnel).** Einfahrt mit Kamera ueber die Spurmitte, ab der
Oeffnung nur LiDAR (Kamera laut Reglement verboten). tunnel_explorer_node sucht den
Ausgang, keepout_zone_node sperrt den Eingang.

## Detaildoku

| Datei | Inhalt |
|-------|--------|
| `Nav2LocalCoastmap_Doku.md` | Fahrmodi, Nodes, Parameter, Ablauf von Obstacle und Tunnel |
| `Nav2_Costmap_Werte.md` | Herkunft und Abhaengigkeiten der Costmap- und MPPI-Werte |
| `Obstacle_Tunnel_Offene_Punkte.md` | Testbeobachtungen und offene Punkte |
| `Kalibrierung_Checkliste.md` | Kamera, BEV und Live-Test Schritt fuer Schritt |
| `Befehle.md` | Start, Not-Aus, Topics, Werkzeuge |
| `challenges.md` | Ueberblick ueber die Wettbewerbsaufgaben |

## Start

```bash
bash scripts/tb/launchers/nav2_obstacle.sh   # alles inkl. Nav2, Not-Aus im Vordergrund
bash scripts/tb/launchers/follow_lane.sh     # nur Spurfolge, ohne Nav2
```
