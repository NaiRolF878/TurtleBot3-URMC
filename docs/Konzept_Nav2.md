# Konzept des Branches `Nav2-Costmap-Fix`

Spurfolge mit PID, Hindernisse auf der Strecke und Tunnel mit **Nav2** (Costmap, NavFn,
Regulated Pure Pursuit). Im Konzeptpapier auf `main` (`docs/URMC–Technisches_Konzept.md`,
Abschnitt 3) entspricht das der Stufe „Fortgeschritten“ (Nav2 Costmap), mit RPP statt DWA. Die
Variante mit Zonenaufteilung für den Obstacle-Modus liegt auf `LDS03-Doku`.

## Architektur

- **switch_control** entscheidet als einziger über den Fahrmodus (`/control/mode`) und führt
  auf Anfrage Abläufe aus Grundbewegungen aus (Engstelle durchfahren, freifahren).
- **control_driving** erzeugt als einziger Fahrbefehle (`/control/cmd_vel`): PID, die
  Befehle von Nav2 oder eine Grundbewegung in odom, auf die Radgrenze skaliert. Davor steht
  der **Wächter**: In Obstacle, Tunnel und Align prüft er jeden Befehl 0,6 s voraus mit dem
  Umriss gegen die local_costmap und sendet Null, wenn der Bot irgendwo neu anstoßen würde.
- **notaus** gibt die Befehle nach Freigabe an den Bot weiter (`/cmd_vel`).
- **scan_correct** reicht den LiDAR korrigiert weiter (`/scan_corrected`): gemessener
  Entfernungsfehler und Bewegung während des Umlaufs. Alle Nodes und Costmaps lesen ihn.
- Alle anderen Nodes liefern nur Daten: Spurfehler, Spurgedächtnis, Pfade, Karten.

| Modus         | Auslöser (switch_control)                                         | Fahrbefehl (control_driving) |
|---------------|--------------------------------------------------------------------|------------------------------|
| `Lane`        | Start, Ende von Obstacle; nach dem Tunnel der Modus davor          | PID auf den Spurfehler       |
| `Obstacle`    | LiDAR-Punkt im Kegel voraus näher als `obstacle_trigger_distance`, keine Spurlinie dazwischen | RPP (`/nav2/cmd_vel`); Engstelle, Freifahren: Grundbewegungen |
| `Align`       | Ende von Obstacle, Bot steht mehr als 15° schräg zur Spur          | auf der Stelle drehen        |
| `TunnelEntry` | Button im Debugbild oder Tunnelschild                              | PID auf den Spurfehler       |
| `Tunnel`      | Beide Spurlinien enden kurz vor dem Bot                            | RPP (`/nav2/cmd_vel`); Engstelle, Freifahren: Grundbewegungen |
| `Stop`        | Storage: Start mit `storage.launch.py`, Schild lesen, Greifen, Ablegen | Null                     |
| `Maneuver`    | Storage: Drehen, Strecke, Punkt anfahren, Andocken                 | Regler in odom (`/control/maneuver`) |

Jeder andere Modus, veraltete Eingangsdaten oder ein fehlendes `/control/mode`
führen zu einem Nullbefehl.

```
detect_lane_nav2 --/detect/lane----------------------.
controller_server (RPP) --/nav2/cmd_vel------------. |
switch_control --/control/maneuver---------------. | |
                                                 v v v
switch_control --/control/mode----------> control_driving [Wächter] --/control/cmd_vel--> notaus --/cmd_vel--> Bot

lane_costmap_node --Spurwände, Spurmitte--> nav2_goal_publisher --Ziel/Pfad--> planner_server, controller_server
tunnel_explorer_node --Pfad--> nav2_goal_publisher;  beide --/control/maneuver_request--> switch_control
```

## Ein Umriss für alles

Wer auf Kollision prüft, nimmt den echten Umriss des Burgers ohne Polster
(`robot_geometry.FOOTPRINT` = `local_costmap.footprint`): RPP, der Wächter, das Freifahren und
die Pfadprüfung. Nur NavFn plant für einen Kreis (`robot_radius` 0,09 m plus 1 cm Polster). Er
hält die Bot-Mitte 0,0988 m von jeder Wand fern; das reicht seitlich, beim Drehen auf der Stelle
aber nicht (hintere Ecken 10,5 cm). Das fangen die Prüfungen mit dem Umriss ab.

## Challenges

**Spurfolge (Lane).** Kamera → metrische Vogelperspektive (ingress_node_nav2) →
Spurerkennung in LAB (detect_lane_nav2) → PID in control_driving. Kalibrierung:
`Kalibrierung_Checkliste.md`.

**Hindernisse auf der Strecke (Obstacle).** lane_costmap_node merkt sich die gelbe und
die weiße Spurlinie als virtuelle Wände (verfolgt, Querlinien und helle Flächen
nicht), NavFn plant zwischen ihnen um das Hindernis herum, RPP fährt den Weg ab und prüft dabei
den Umriss, auch beim Drehen.

**Tunnel (TunnelEntry, Tunnel).** Einfahrt mit Kamera und PID auf der 20-cm-Spur (dort
passt der Bot für Nav2 nicht zwischen die Spurwände), bis die Linien an der Öffnung
enden. Ab dann nur LiDAR (Kamera laut Reglement verboten): tunnel_explorer_node findet die
Tür im Scan und fährt hindurch, keepout_zone_node sperrt sie danach. Die Wände bilden
eine Hülle: Strahlen durch die Hülle nach außen zeigen auf den Ausgang. Gesucht wird, wo
Unbekanntes an der Wand liegt (dort kann ein Ausgang sein); Graues tiefer in der Hülle ist
Schatten hinter Hindernissen. Draußen ist der Bot, wenn er die Hülle ganz verlassen hat. Das
alles kommt ohne Maße der Enclosure aus.

## Engstellen, Abbrüche, Neuplanung (Obstacle und Tunnel)

**Vor dem Losfahren prüfen.** Jeder neue Pfad wird bis 1 m voraus mit dem Umriss in
Pfadrichtung abgefahren (`passage_geometry.check_path`). Stößt er an einer Engstelle an, sucht
die Prüfung die Gerade mit dem meisten Abstand zu den Wänden:
- passt der Bot ausgerichtet hindurch: RPP fährt bis zum Start der Geraden (dort ist Platz zum
  Drehen), switch_control dreht ihn auf die Gerade und fährt langsam gerade hindurch, danach
  wird neu geplant;
- passt er auch ausgerichtet nicht oder ist die Durchfahrt dort schon gescheitert: die Engstelle
  wird gesperrt (`/nav2/blocked`), NavFn sucht einen anderen Weg.

Es gibt keine festen Breiten: Es zählt nur, ob der Umriss passt.

**Abbruch.** Bricht das Ziel wegen Kollisionsgefahr ab (Meldung von RPP oder Wächter länger als
1 s), fährt der Bot frei: control_driving probiert kleine Bewegungen (5 cm vor oder zurück, 15°
drehen) gegen die local_costmap und fährt die, nach der er am weitesten von allem weg ist. Gibt es
keine, bleibt er stehen und meldet das. Andere Abbrüche: neu planen, im Tunnel gilt das Ziel nach
dem zweiten als gescheitert und wird 15 s gemieden.

**Neuplanung.** Zum selben Ziel wird jede Sekunde neu geplant. Der neue Pfad gilt nur, wenn er
30 cm voraus höchstens 45° anders zeigt oder der alte blockiert ist (kein Kippen links/rechts um
ein Hindernis), und nicht während der Fahrt durch eine Engstelle.

**Tunnel-Uhr.** Ab 40 s nach dem Eintritt zählt bei der Zielwahl nur noch die Nähe. Jede
Tunnelfahrt wird protokolliert (`logs/tunnel/<Zeit>/`: Ereignisse, Posen, Scans, Kartenbild).

**Storage (Palette, Stop/Maneuver).** Nur Kamera und Odometrie: Erkundung entlang
der Spur mit Zonenmodell, leere Zonen erkennen, rechtwinklig an die Palette andocken und
greifen, per odom in die Zielzone. Eigenes Konzept: `Storage_Retrieval_Konzept.md`.

## Detaildoku

| Datei | Inhalt |
|-------|--------|
| `Nav2LocalCoastmap_Doku.md` | Fahrmodi, Nodes, Parameter, Ablauf von Obstacle und Tunnel |
| `Nav2_Costmap_Werte.md` | Herkunft und Abhängigkeiten der Costmap- und Reglerwerte |
| `Obstacle_Tunnel_Offene_Punkte.md` | Testbeobachtungen und offene Punkte |
| `Kalibrierung_Checkliste.md` | Kamera, BEV und Live-Test Schritt für Schritt |
| `Storage_Retrieval_Konzept.md` | Storage-Aufgabe: Ablauf, Zustaende, Topics, Entscheidungen |
| `Befehle.md` | Start, Not-Aus, Topics, Werkzeuge |
| `challenges.md` (auf `main`) | Überblick über die Wettbewerbsaufgaben |

## Start

```bash
bash scripts/tb/launchers/nav2_obstacle.sh   # alles inkl. Nav2, Not-Aus im Vordergrund
bash scripts/tb/launchers/follow_lane.sh     # nur Spurfolge, ohne Nav2
bash scripts/tb/launchers/storage.sh         # Storage-Aufgabe, Start per Button im Debugbild
```
