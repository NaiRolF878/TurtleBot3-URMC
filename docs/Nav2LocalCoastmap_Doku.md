# Nav2-Fahrmodi: Hindernisvermeidung & Tunnel-Navigation

Dokumentation für alle, die nicht direkt am Code gearbeitet haben, aber verstehen wollen/müssen, wie die Nav2-Fahrmodi funktionieren und welche Werte am Wettbewerbstag ggf. angepasst werden müssen.

---

## 1. Worum geht's

Neben der PID-Spurregelung (`Lane`-Modus) haben wir eine zweite, parallele Fahr-Pipeline auf Basis von **Nav2** (dem Standard-Navigationsframework von ROS2) gebaut. Diese eine Pipeline deckt **zwei unterschiedliche Situationen** ab, die sich beide Infrastruktur teilen (dieselbe Costmap, derselbe RPP-Regler, derselbe Übergabe-Node), sich aber in ihrem Zweck unterscheiden:

- **Obstacle-Modus:** liegt im Hindernisbereich außerhalb des Tunnels ein Hindernis in der per Kamera erkannten Fahrbahn, übernimmt Nav2 kurzzeitig, weicht aus und bleibt dabei zwischen den Spurlinien. Läuft automatisch, ohne Zeitdruck, mitten im normalen Rennen.
- **Tunnel-Modus:** eigener Streckenabschnitt (im Reglement „Enclosure“), ausgelöst per Schild oder per Button im Debugbild: der Roboter fährt in einen geschlossenen Raum mit Hindernissen, muss ihnen ausweichen und **innerhalb eines Zeitlimits** einen Ausgang finden — aber nicht den, durch den er reingefahren ist. **Die Kamera ist dort laut Reglement verboten**, es zählt nur der LiDAR.

Beide Modi laufen über dieselben Nodes, aktivieren sich aber unabhängig voneinander und nutzen teils unterschiedliche Zusatz-Nodes (siehe Abschnitt 3/4). Die Pipeline läuft die ganze Zeit mit, wird aber nur aktiv, wenn der Roboter tatsächlich im Obstacle- oder Tunnel-Modus ist.

---

## 2. Die drei Fahrmodi

Ein einziger Node (`switch_control`) entscheidet, welcher der folgenden Modi gerade aktiv ist, und alle anderen Nodes richten sich danach:

```mermaid
flowchart LR
    Lane["Lane\nPID-Regler, Kamera-Spurerkennung"]
    Obstacle["Obstacle\nNav2/RPP, Hindernis auf der Fahrbahn"]
    Tunnel["Tunnel\nNav2/RPP, geschlossener Raum, kein Kamera-Einsatz"]
    Align["Align\nauf der Stelle zur Spur drehen"]

    Lane -- "LiDAR: Hindernis vorn, keine Spurlinie dazwischen" --> Obstacle
    Obstacle -- "1 s nichts in 0,4 m (270 Grad)" --> Align
    Align -- "ausgerichtet oder 5 s" --> Lane
    TunnelEntry["TunnelEntry\nPID-Regler, Einfahrt mit Kamera"]
    Lane -- "Tunnel-Schild (auto)\noder Button (manuell)" --> TunnelEntry
    Obstacle -- "Tunnel-Schild (auto)\noder Button (manuell)" --> TunnelEntry
    TunnelEntry -- "Beide Spurlinien enden\n(an der Oeffnung)" --> Tunnel
    Tunnel -- "Bot ausserhalb der Wand-Huelle" --> Lane
```

- **Lane**: der normale Betrieb, PID-Regler folgt der Spur.
- **Obstacle**: sobald der LiDAR etwas direkt vor dem Roboter erkennt und zwischen Roboter und Objekt keine Spurlinie liegt, übernimmt Nav2, plant einen Weg darum herum und fährt ihn ab — automatisch, ohne Schild. Objekte auf der anderen Seite einer Spurlinie (z. B. Hindernisse neben dem Rückweg) lösen nichts aus. Zurück zu Lane geht es, wenn 1 s lang im Umkreis von 0,4 m nichts liegt, gemessen nach vorn, zur Seite und schräg nach hinten (270°). Der Keil direkt hinter dem Bot zählt nicht, von dort kam er gerade. Mit `obstacle_enabled` = false (Taste O im Not-Aus, Häkchen in rqt, Standard bei `follow_lane.sh`) gibt es keinen Wechsel nach Obstacle, ein laufendes Obstacle endet sofort.
- **TunnelEntry**: Einfahrt. Der Bot folgt wie im Lane-Modus mit dem PID der Spur; auf der 20-cm-Spur passt er für Nav2 nicht zwischen die Spurwände. Die Kamera ist hier noch erlaubt. Die Linien reichen nur bis zur Öffnung: Fehlen beide in 5 Bildern in Folge in der Suchzeile (oder kommt 1 s gar kein Bild), schaltet `switch_control` auf `Tunnel`. Es gibt keinen Abstands- oder Breitenwert.
- **Tunnel**: eigener Modus für den geschlossenen Raum, ohne Kamera. Beim Umschalten wird das Spurgedächtnis verworfen, eine leere Spurwolke gesendet und 0,3 s später werden beide Karten geleert; die Tunnelkarte bleibt bis zum Austritt. `tunnel_explorer_node` findet die Tür im Scan, fährt hindurch und sucht den Ausgang (Abschnitt 7). Zurück geht es, wenn der ganze Bot außerhalb der Wand-Hülle ist und vorher einmal drin war. Stand beim Umschalten keine Linie im Bild, ist er schon im Eingang oder im Tunnel: Die Einfahrt entfällt. Ohne TunnelEntry davor (Start im Tunnel) gibt es keine Eingangssperre. Taste L im Tunnel setzt Sperre und Explorer an der aktuellen Pose neu auf. Ausgelöst wird der Tunnel auf zwei Wegen:
  - **Schild:** ein Erkennungsmodell sendet `6` (`std_msgs/UInt8`) auf `/detect/sign`. Auf diesem Branch gibt es noch keinen Sender. Die Schildnummern stehen in `SignType` (`common.py`), getrennt von den Modusnummern; welcher Modus aus einem Schild folgt, entscheidet `switch_control`.
  - **Button „TUNNEL STARTEN“ im Debugbild oder unten in der Nav2-Draufsicht (`nav2_view`):** um die Challenge einzeln zu fahren. Der Bot steht dabei vor der Öffnung. In TunnelEntry und Tunnel wird daraus „TUNNEL ABBRECHEN“. Nach dem automatischen Austritt springt der Button von selbst zurück.

  Wer ausgelöst hat, steht im Debugbild unter dem Modus: „(manuell)“ oder „(auto)“, auch auf `/control/tunnel_source`.

`switch_control` merkt sich beim Eintritt in den Tunnel, welcher Modus vorher aktiv war, und stellt genau den beim Verlassen wieder her (nicht stur "zurück zu Lane"). Jeder Moduswechsel wird sofort auf `/control/mode` gesendet, zusätzlich alle 0,5 s.

- **Align**: Nach Obstacle steht der Bot oft schräg. Weicht er mehr als 15° von der Spurrichtung ab (Hauptachse der gemerkten Linien), dreht `control_driving` ihn auf der Stelle, bis 5° Rest erreicht oder 5 s um sind; dann Lane. Ohne eindeutige Spurrichtung direkt Lane.

Die Werte `Stop` und `Intersection` gibt es im Enum schon, sie haben aber noch keinen Auslöser (vorgesehen für Schranke und Kreuzung an Tag 1). In diesen Modi sendet `control_driving` Null.

### Wer fährt, wer nutzt die Kamera?

Fahrbefehle erzeugt nur `control_driving`. Er wählt je nach Modus die Quelle und sendet auf `/control/cmd_vel`; der Not-Aus (`notaus`) reicht das nach Freigabe an `/cmd_vel` durch.

| | **Lane-Modus** | **Obstacle-Modus** | **Tunnel-Modus** |
|---|---|---|---|
| Quelle in `control_driving` | PID auf `/detect/lane` | `/nav2/cmd_vel` (RPP), auf Radgrenze skaliert, Wächter; Engstelle und Freifahren: Grundbewegung (`/control/maneuver`) | wie Obstacle |
| Wer nutzt die Kamera? | `control_driving` über `/detect/lane`; `lane_costmap_node` für die Spurwände | `lane_costmap_node` für die Spurwände | niemand: `ingress_node` und `detect_lane` pausieren (Reglement) |
| Was steht in local_ und global_costmap? | LiDAR + Spurgedächtnis | LiDAR + Spurgedächtnis | LiDAR + Eingangssperre |
| Wird geplant? | nein, nichts geplant und nichts angezeigt | ja: NavFn plant einen Weg um die Hindernisse, RPP fährt ihn ab | ja: Explorer + NavFn |

**Wie die Spurwände ohne bekannte Breite funktionieren:** Die Breite des Hindernisbereichs kennen wir erst am Wettbewerbstag. Deshalb hängt kein Wert davon ab. RPP, der Wächter und die Pfadprüfung prüfen Kollisionen mit dem echten Umriss des Burger (138 × 178 mm, ohne Polster; `footprint` in `nav2_params.yaml` = `robot_geometry.FOOTPRINT`). Eine Fahrbewegung ist verboten, sobald der Umriss eine Linie oder ein Hindernis berührt, und erlaubt, wenn er dazwischen passt. Die Inflation um die Linien ist für NavFn ein Kostengefälle, das zur Mitte zieht. Details: `docs/Nav2_Costmap_Werte.md`, Abschnitt 3.

Weil nur `control_driving` auf `/control/cmd_vel` sendet, konkurrieren nie zwei Regler. Ist die aktive Quelle veraltet (PID: 0,3 s ohne `/detect/lane`, Nav2: 0,25 s ohne `/nav2/cmd_vel`) oder kommt 1,5 s kein `/control/mode`, sendet er Null. Bei jedem Moduswechsel sendet er sofort Null und nimmt nur Befehle an, die danach eintreffen. Nach dem Wechsel in den Tunnel ignoriert er Nav2 zusätzlich 0,3 s lang, bis das alte Obstacle-Ziel (Spurmitte, also Kameradaten) abgebrochen ist.

**Wächter.** In Obstacle, Tunnel und Align prüft `control_driving` jeden Befehl, bevor er hinausgeht: Der Umriss wird 0,6 s mit diesem Befehl vorausgerechnet und gegen die belegten Zellen der `local_costmap` gelegt. Würde er irgendwo neu anstoßen, geht Null hinaus (was er schon berührt, zählt nicht, sonst käme er nie wieder weg). Hält der Wächter Nav2 länger als 1 s an, bricht `nav2_goal_publisher` das Ziel wie bei einer Kollision ab; eine Grundbewegung gilt nach 1 s als gescheitert.

---

## 3. Gesamtarchitektur

```mermaid
flowchart TB
    subgraph Sensorik
        SCAN["/scan_corrected\nLiDAR 360 Grad (scan_correct)"]
        LANE["/detect/lane_mask\nSpur-Pixel (Kamera)"]
        SIGN["/detect/sign\nSchild-Erkennung (noch nicht gebaut)"]
        BTN["/control/tunnel_request\nButton im Debugbild"]
    end

    SCAN --> SC[switch_control]
    SIGN --> SC
    BTN --> SC
    SC -->|"/control/mode"| ALL[alle anderen Nodes]

    LANE --> LCM[lane_costmap_node\nSpurgedaechtnis]
    LCM -->|"/detect/lane_obstacles\n(nicht im Tunnel)"| LC[local_costmap\nin controller_server]
    LCM -->|"/detect/lane_obstacles"| GC[global_costmap\nin planner_server]
    LCM -->|"/detect/lane_obstacles\n(Linienfilter)"| SC
    LCM -->|"/lane/center_path\n(nur Obstacle)"| GP

    SC -->|"/control/mode"| KZ[keepout_zone_node]
    KZ -->|"/detect/tunnel_entrance_obstacle"| LC
    KZ -->|"/detect/tunnel_entrance_obstacle"| GC
    TE[tunnel_explorer_node] -->|"/tunnel/outside"| SC
    SCAN --> GC

    GC -->|"/global_costmap/costmap"| TE
    TE -->|"ComputePathToPose"| PS[planner_server\nNavFn]
    PS --> TE
    TE -->|"/tunnel/explore_path"| GP[nav2_goal_publisher]
    GP -->|"ComputePathToPose\n(Obstacle)"| PS
    GC --> PS

    GP -->|"FollowPath-Ziel"| CS[controller_server\nRPP]
    GP -->|"/nav2/active_path\n(Anzeige)"| VIEW[nav2_view / RViz]
    LC --> CS
    CS -->|"/nav2/cmd_vel"| CD[control_driving]
    DL["/detect/lane\nSpurfehler (Kamera)"] --> CD
    SC -->|"/control/mode"| CD
    TE -->|"/control/maneuver_request"| SC
    GP -->|"/control/maneuver_request"| SC
    SC -->|"/control/maneuver"| CD
    CD -->|"/control/maneuver_status"| SC
    CD -->|"/control/cmd_vel"| NA[notaus]
    NA -->|"/cmd_vel"| MOT[Motoren]
```

Wichtig zum Verständnis: **`lane_costmap_node` sendet die Spurwände in jedem Modus außer Tunnel, die Spurmitte nur in Obstacle. Beim Wechsel auf Tunnel verwirft er das Gedächtnis, sendet eine leere Wolke und leert 0,3 s später beide Costmaps. `nav2_goal_publisher` plant nur in Obstacle und Tunnel. `tunnel_explorer_node` arbeitet nur im Tunnel-Modus, `keepout_zone_node` im Tunnel (Eingangssperre) und in Obstacle (Startsperre).** Außerhalb ihres jeweiligen Modus veröffentlichen sie nichts außer einer leeren Wolke beim Verlassen.

---

## 4. Die Nodes im Einzelnen

### `switch_control` — die Zustandsmaschine
Entscheidet, welcher Modus aktiv ist (siehe Abschnitt 2), und veröffentlicht das laufend auf `/control/mode`. Alle anderen Nodes lesen nur mit, niemand sonst darf diesen Wert verändern. Für Eintritt und Austritt des Obstacle-Modus prüft er jeden LiDAR-Punkt gegen das Spurgedächtnis (`/detect/lane_obstacles`): Liegt auf der Strecke Bot → Punkt eine Spurlinie, gehört der Punkt nicht zur eigenen Fahrbahn und zählt nicht. Ist das Gedächtnis älter als `LANE_MAX_AGE_S`, zählt jeder Punkt. Den Tunnel-Wunsch des Buttons bekommt er auf `/control/tunnel_request` (`True` = Start, `False` = Abbruch), den Auslöser meldet er auf `/control/tunnel_source` (`manuell`, `auto` oder leer). `obstacle_enabled` (Taste O) schaltet den Obstacle-Modus ab. In Obstacle und Tunnel fährt er auf Anfrage (`/control/maneuver_request` von `nav2_goal_publisher` bzw. `tunnel_explorer_node`) Grundbewegungen: eine Engstelle ausgerichtet durchfahren oder freifahren (`maneuver_sequence.py`, Abschnitte 6 und 7). Den Stand meldet er auf `/control/maneuver_sequence`; der Modus bleibt dabei, im Tunnel also die Kamera aus.

### `control_driving` — die einzige Quelle für Fahrbefehle
Wählt je nach `/control/mode` den PID (Lane, TunnelEntry) oder reicht `/nav2/cmd_vel` durch (Obstacle, Tunnel); in jedem anderen Modus Null. In Obstacle und Tunnel hat eine laufende Grundbewegung von `switch_control` (`/control/maneuver`: auf der Stelle drehen, geradeaus mit gehaltenem Kurs, Punkt anfahren, freifahren; in odom) Vorrang vor Nav2. Ist sie fertig oder abgebrochen (Wächter, dreht sich im Kreis, Pose länger als 1 s veraltet, kein freier Ausweg), meldet er das samt Grund auf `/control/maneuver_status` und folgt wieder Nav2. Ein Moduswechsel verwirft sie. Freifahren: Er probiert 5 cm vor/zurück und 15° links/rechts gegen die `local_costmap` und fährt die Bewegung, nach der der Umriss am weitesten von allem Belegten weg ist. Würde das äußere Rad schneller als 0,22 m/s (`MAX_WHEEL_VEL`), skaliert er v und ω gemeinsam herunter, die Bahn bleibt gleich. Den Wächter beschreibt Abschnitt 2. Zeitgrenzen und Verhalten beim Moduswechsel siehe Abschnitt 2.

### `notaus` — die Fahrfreigabe
Letzte Stufe zwischen `/control/cmd_vel` und `/cmd_vel`. Erzeugt keine Bewegung, reicht nur nach Freigabe (Leertaste, rqt `drive`, `/notaus/set`) durch oder sendet Null, auch wenn 0,5 s (`cmd_timeout_s`) nichts kommt. Taste L sendet `/control/clear_maps` (Spurgedächtnis und Costmaps leeren, im Tunnel Neustart der Einfahrt), Taste O schaltet `obstacle_enabled` von `switch_control` um.

### `debug_view` — das Debugbild mit Tunnel-Button
Zeichnet das Debugbild auf `/lokal/debug_view`, angezeigt in `rqt_image_view`. Im Tunnel steht statt des Kamerabilds ein Platzhalter. Unten im Mode-Panel liegt der Tunnel-Button. Ein Klick darauf kommt als Klickposition von `rqt_image_view` auf `/lokal/debug_view_mouse_left` an. Dafür muss in `rqt_image_view` einmal die Checkbox **„Publish click location“** angehakt sein.

### `lane_costmap_node` — das Spurgedächtnis (nicht im Tunnel)
Wandelt die von der Kamera erkannten Spurlinien-Pixel aus der metrischen Vogelperspektive `/lokal/bev` über `Parameter/bev.yaml` in Meter um. Wand werden nur die gelbe und die weiße Spurlinie: Jede wird vom bot-nahen Ende aus verfolgt (auch durch enge Kurven), nur Pixel bis `line_margin` um ihre Mitte zählen. Querlinien, Papier und helle Flächen neben der Strecke werden so keine Wand; knickt eine Linie gegen die andere ab (Querlinie in L-Form), endet sie dort. Pixel am Hindernis und in seinem LiDAR-Schatten fallen vorher weg, weil die BEV hohe helle Gegenstände lang zieht. Diese Linien merkt er sich in einem ortsfesten Raster (odom-Frame, 1 cm). Eine Zelle wird erst nach `line_confirm_frames` (3) Bildern in Folge zur Linie und erst nach `free_confirm_frames` (10) Bildern in Folge ohne Linie wieder frei. Abgestimmt wird nur, was die Kamera gerade sieht (Sichtmaske `/lokal/bev_view` von `ingress_node`); Zellen im Sichtschatten eines LiDAR-Hindernisses bekommen keine „frei“-Stimme, weil das Hindernis die Linie dort nur verdeckt. So kennt Nav2 auch Linien neben und hinter dem Bot, die die Kamera (5–60 cm voraus) nicht mehr sieht.

Gesendet wird jedes Mal das ganze Gedächtnis im Umkreis von 2 m auf `/detect/lane_obstacles` — an die `lane_layer` beider Costmaps und an `switch_control`. Sendet auch im Lane-Modus, damit der Korridor beim Umschalten schon in der Karte steht. Die Spurmitte (`/lane/center_path`, mit laufend gemessener Spurbreite) sendet er nur im Obstacle-Modus. Im Tunnel sendet er nur beim Eintritt eine leere Wolke, verwirft das Gedächtnis und leert 0,3 s später beide Costmaps. Dasselbe passiert in jedem Modus auf `/control/clear_maps` (Taste L), wenn die Karten nicht mehr zur Odometrie passen. Die Eingangssperre bleibt dabei stehen.

### `nav2_view` — die Draufsicht mit Statuszeile und Tunnel-Button
Unten liegt derselbe Tunnel-Button wie im Debugbild (Start, im Tunnel Abbruch, über `/control/tunnel_request`). Im zweiten `rqt_image_view`-Fenster dafür einmal **„Publish click location“** anhaken; der Klick kommt auf `/nav2/view_mouse_left` an.

Zeichnet auf `/nav2/view` ein Bild davon, was Nav2 sieht und plant: im Tunnel die `global_costmap`, sonst die `local_costmap` mit den Spurwänden. Grün ist befahrbar (dunkler = nah am Hindernis), rot blockiert, grau unbekannt, gelb/weiß die erkannten Spurlinien (`/detect/lane_mask`). Blau der Umriss des Bots (`/local_costmap/published_footprint`) mit Fahrtrichtung (magenta), cyan der Pfad, dem Nav2 gerade folgt (`/nav2/active_path`), mit Ziel (X) — nur während einer Fahrt, im Lane-Modus nie, orange die Eingangssperre, hellblau die LiDAR-Punkte. Unten eine Legende. Die Ausrichtung ist fest (odom-X nach rechts), sodass Wände stillstehen und nur der Bot sich bewegt.

Oben steht die Statuszeile: Modus und Auslöser, was `nav2_goal_publisher` gerade tut (`/nav2/status`, z. B. „faehrt: Bahnplaner (Spurmitte)“ oder „kein Weg gefunden“), und darunter die letzte Warnung bzw. der letzte Fehler der Nav2-Knoten aus `/rosout`, zum Beispiel „controller_server: Failed to make progress“. Das ist der Abbruchgrund: Das FollowPath-Ergebnis meldet in Humble nur „abgebrochen“, den Grund schreibt `controller_server` nur ins Log. Rot heißt jünger als 10 s.

Gezeigt wird ein Ausschnitt: außerhalb des Tunnels ein Quadrat von `LOCAL_VIEW_SIZE_M` (2,4 m) um den Bot. Das deckt das Planziel im Obstacle-Modus (~1 m) mit Rand ab. Im Tunnel werden keine Spurlinien gezeichnet. Es ist zentriert, damit das Bild beim Lenken nicht springt. Im Tunnel ist es nur die schon gesehene Fläche plus `TUNNEL_VIEW_MARGIN_M` (0,3 m), sodass das Bild mit der Karte herauszoomt.

`nav2_obstacle` öffnet dafür ein zweites `rqt_image_view`-Fenster.

### `keepout_zone_node` — die Eingangs-Sperrzone (nur Tunnel-Modus)
Setzt beim Wechsel TunnelEntry → Tunnel eine virtuelle Wand quer zur Fahrtrichtung durch die Radachse, 40 cm lang, ortsfest im odom-Frame, damit der Roboter nicht durch den Eingang wieder hinausfährt. Sie erscheint erst, wenn der Bot 16 cm weiter ist, sonst stünde er in ihr. Ohne TunnelEntry davor (Start im Tunnel) gibt es keine Wand; Taste L legt sie an der aktuellen Pose neu. Beim Verlassen des Tunnels sendet er eine leere Wolke, sonst bliebe die Wand in den Karten. Im Obstacle-Modus setzt er beim Eintritt eine Startsperre 20 cm hinter den Bot (60 cm breit), damit NavFn nicht rückwärts plant; beim Verlassen von Obstacle verschwindet sie wieder. Der Node hat keine Parameter.

### `tunnel_explorer_node` — die Ausgangssuche (nur Tunnel-Modus)
Fährt zuerst durch die Tür und sucht danach den Ausgang, siehe Abschnitt 7. Meldet auf `/tunnel/outside`, wenn der Bot draußen ist. Im Tunnel stellt er den Regler langsamer und den Fortschritts-Check lockerer (`TUNNEL_CONTROLLER`: `desired_linear_vel` 0,10 m/s, 0,05 m in 6 s). Jede Tunnelfahrt protokolliert er nach `logs/tunnel/<Zeit>/` (`TUNNEL_LOG_DIR`, gesetzt von `nav2_obstacle.sh`): Ereignisse, Posen, Scans, am Ende ein Kartenbild. Der Node hat keine Parameter: Alle Grenzen hängen am Bot oder an der Karte (`tunnel_geometry.py`, `passage_geometry.py`).

### `nav2_goal_publisher` — der Übergeber
Schickt das eigentliche Fahrkommando an Nav2. Im Obstacle-Modus setzt er ein Ziel `goal_distance` (1 m) voraus auf der Spurmitte von `lane_costmap_node` (über ihr Ende hinaus in ihrer Richtung verlängert; ohne aktuelle Spurmitte geradeaus), lässt `planner_server` einen Weg dorthin um die Hindernisse rechnen, prüft ihn mit dem Umriss (Engstellen, Abschnitt 6) und gibt ihn `controller_server`. Neu geplant wird, wenn eine gesperrte Zelle auf dem Pfad liegt, wenn er bis auf `goal_distance`/2 abgefahren ist, wenn die Fahrt endet, und jede Sekunde (der neue Weg gilt dann nur bei gleicher Richtung). Den Abbruchgrund liest er aus `/rosout` (RPP: „detected collision ahead“), hält der Wächter länger als 1 s an (`/control/guard`), bricht er das Ziel selbst ab. Im Tunnel-Modus nimmt er den Pfad, den `tunnel_explorer_node` geliefert hat, und meldet das Ergebnis samt Grund auf `/tunnel/path_result` (`erreicht`, `kollision`, `abgebrochen`). Den Pfad, dem Nav2 gerade folgt, veröffentlicht er auf `/nav2/active_path`; beim Verlassen des Modus leert er ihn. Im Lane-Modus ruht er.

### `controller_server` (Nav2, Regulated Pure Pursuit) — die eigentliche Fahrsteuerung
Nav2-Standardkomponente, nicht selbst geschrieben. Nimmt den Pfad entgegen und berechnet daraus in Echtzeit die tatsächlichen Fahrbefehle (auf `/nav2/cmd_vel`): Er zielt auf einen Punkt 15–35 cm voraus auf dem Pfad, fährt in Kurven langsamer und dreht auf der Stelle, wenn der Pfad mehr als 45° abweicht. Er weicht nicht aus, sondern fährt den Pfad von NavFn ab. Vor jedem Befehl prüft er den Umriss bis 1 s voraus gegen die `local_costmap`, auch beim Drehen; bei Gefahr sendet `controller_server` 0,3 s Null (`failure_tolerance`) und bricht dann das Ziel ab.

### `planner_server` (Nav2, NavFn) — der Bahnplaner (Obstacle und Tunnel)
Nav2-Standardkomponente. Berechnet auf der `global_costmap` (LiDAR + Spurgedächtnis, im Tunnel LiDAR + Eingangssperre) einen Weg zu einem Zielpunkt: im Obstacle-Modus auf Anfrage von `nav2_goal_publisher` um die Hindernisse herum, im Tunnel auf Anfrage von `tunnel_explorer_node`. Immer **nur durch bereits bekanntes, freies Gebiet** (siehe Abschnitt 7). NavFn plant für einen Kreis (`robot_radius` 0,09 m plus Polster: Saum 0,0988 m um jedes Hindernis), nicht für den echten Umriss. Lücken ab etwa 0,20 m sind planbar. Ob der Bot dort wirklich hindurchpasst, entscheidet erst die Pfadprüfung (Abschnitt 6).

---

## 5. Parameter-Referenz — was anpassen, was bedeutet es

In den YAMLs steht nur, was man vor Ort einstellt. Diese Parameter haben keinen Code-Default: Fehlt ein Wert, stoppt der Node mit klarer Meldung. Namen in GROSSBUCHSTABEN sind feste Konstanten im Code des Nodes.

### 5.1 `switch_control.py` → Datei `Switch_Control.yaml`

| Parameter | Wert (YAML) | Bedeutung | Auswirkung |
|---|---|---|---|
| `obstacle_check_angle_deg` | 45.0° | **Halb**winkel des Prüfkegels nach vorne (Gesamtkegel 90°), in dem nach Hindernissen gesucht wird | Größer = reagiert auch auf seitlichere Objekte |
| `obstacle_trigger_distance` | 0.30 m | Ab welcher Nähe (ab LiDAR) gilt "da ist ein Hindernis" | Größer = reagiert früher, Nav2 hat mehr Platz |
| `OBSTACLE_CLEAR_ANGLE_DEG` | 135° | **Halb**winkel des Austrittsbereichs (270°, ohne den Keil direkt hinten) | Kleiner = Hindernisse schräg hinten halten den Obstacle-Modus nicht mehr |
| `obstacle_clear_distance` | 0.40 m | In diesem Umkreis darf nichts mehr liegen, damit es zurück zu Lane geht | Größer = bleibt länger im Obstacle-Modus |
| `OBSTACLE_CLEAR_DEBOUNCE_S` | 1.0 s | Wie lange der Austrittsbereich durchgehend frei sein muss | Größer = stabiler, aber langsamer zurück in den Lane-Modus; kleiner = kann bei flackernden Messwerten hin- und herspringen |
| `LINE_BLOCK_TOLERANCE` | 0.02 m | So nah muss ein Linienpunkt an der Strecke Bot → Hindernis liegen, damit er sie trennt | Größer = filtert großzügiger |
| `LINE_OBSTACLE_MARGIN` | 0.05 m | Linienpunkte so dicht vor dem Hindernis zählen nicht (Hindernis steht auf der Linie, ragt also in die Fahrbahn) | Größer = Hindernisse nahe der Linie lösen eher aus |
| `LANE_MAX_AGE_S` | 0.5 s | Ist das Spurgedächtnis älter, wird nicht gefiltert | — |

Die Tunnelausfahrt läuft bewusst nur über den LiDAR — die Kamera ist in der Enclosure verboten.

**Muss angepasst werden:** `obstacle_trigger_distance`, je nachdem wie schnell der Roboter fährt.

### 5.2 `lane_costmap_node.py` → Block `/lane_costmap_node` in `Lane_Costmap.yaml`

| Parameter | Wert | Bedeutung | Auswirkung |
|---|---|---|---|
| `PIXEL_STRIDE` | 2 | Wie stark die Maskenpunkte ausgedünnt werden, bevor sie ins Gedächtnis gehen | Größer = weniger Punkte, spart Rechenzeit, aber gröbere Wand |
| `line_confirm_frames` | 3 | So viele Bilder in Folge mit Linie, bis eine Zelle als Linie gilt | Größer = robuster gegen Fehldetektionen, Linien erscheinen später |
| `free_confirm_frames` | 10 | So viele Bilder in Folge ohne Linie (sichtbar, nicht verdeckt), bis eine Linie vergessen wird | Größer = Linien halten länger, falsche bleiben aber auch länger |
| `MEMORY_RADIUS` | 2.0 m | Weiter entfernte Zellen werden vergessen | Nicht größer als die Reichweite der `lane_layer` (2,0 m) |
| `MEMORY_RESOLUTION` | 0.01 m | Zellgröße des Gedächtnisses | = `resolution` der Costmaps |
| `SHADOW_RADIUS` / `SHADOW_MARGIN` | 0.03 / 0.05 m | Sichtschatten eines LiDAR-Punkts für die Kamera; Linienpixel darin fallen weg | Größer = verdeckte Linien werden sicherer behalten |
| `line_margin` | 0.0 m | Pixel bis so weit um die verfolgte Linienmitte werden Wand | Halbe Klebebandbreite plus Reserve; zu klein = Wand lückenhaft, zu groß = Reste von Querlinien |
| `TRACE_STEP` / `TRACE_MAX_TURN_DEG` | 0.03 m / 35° | Schrittweite und größte Drehung je Schritt beim Verfolgen | 35° auf 3 cm folgt Kurven bis r ≈ 5 cm |
| `TRACE_MAX_GAP` | 0.09 m | Längere Lücken in einer Linie beenden sie | Größer = überbrückt mehr, kann aber auf Fremdes springen |
| `TRACE_PARALLEL_DEG` | 60° | Knickt eine Linie gegen die andere stärker ab, endet sie (Querlinie in L-Form) | Kleiner = schneidet früher, auch in Spurverbreiterungen |

Sendet in jedem Modus außer Tunnel. Pixel → Meter und Reichweite (bis 0,60 m) kommen aus `Parameter/bev.yaml`, nicht aus dieser Datei. Details: `docs/Nav2_Costmap_Werte.md`, Abschnitt 4.

### 5.3 `keepout_zone_node.py` und `tunnel_explorer_node.py`

Keine Parameter. Einfahrt, Zielwahl und Ausfahrt kommen ohne Maße der Enclosure aus; alle Grenzen hängen am Bot oder an der Karte und stehen als Konstanten in `tunnel_geometry.py`.

### 5.5 `nav2_goal_publisher.py` → Block `/nav2_goal_publisher` in `Lane_Costmap.yaml`

| Parameter | Standard | Bedeutung | Auswirkung |
|---|---|---|---|
| `goal_distance` | 1.0 m | Obstacle-Modus: wie weit voraus das Planziel auf der (verlängerten) Spurmitte liegt | Muss hinter dem Hindernis liegen; größer = plant weiter voraus, aber jenseits von ~0,6 m kennt das Spurgedächtnis nur, was die Kamera auf dem Weg dorthin gesehen hat |
| `CENTER_PATH_MAX_AGE_S` | 1.0 s | bis zu welchem Alter die Spurmitte die Richtung des Ziels vorgibt; sonst geradeaus | Größer = folgt länger einer alten Spurmitte, wenn die Linien verdeckt sind |

Die übrigen Werte (`ROBOT_FRAME`, `ODOM_FRAME`, `WATCHDOG_PERIOD_S`) sind technische Details, an denen normalerweise nichts verändert werden muss.

### 5.6 `control_driving.py` → `Control_driving_PID_Wide.yaml`

PID- und Grundbewegungswerte stehen in `Control_driving_PID_Wide.yaml` (Pflicht, ohne sie startet der Node nicht): `maneuver_max_vel` und `maneuver_max_ang` begrenzen, `maneuver_k_lin` und `maneuver_k_ang` sind die Verstärkung auf den Restweg bzw. Restwinkel. Die Zeitgrenzen sind Konstanten:

| Parameter | Standard | Bedeutung |
|---|---|---|
| `LANE_CMD_TIMEOUT` | 0.3 s | So lange ohne `/detect/lane` im Lane-Modus, dann Null |
| `NAV2_CMD_TIMEOUT` | 0.25 s | So lange ohne `/nav2/cmd_vel` im Obstacle-/Tunnel-Modus, dann Null (5 Perioden à 20 Hz) |
| `MODE_TIMEOUT` | 1.5 s | So lange ohne `/control/mode`, dann Null (switch_control sendet mit 2 Hz) |
| `MODE_SWITCH_HOLD` | 0.3 s | Nach dem Wechsel in den Tunnel wird Nav2 so lange ignoriert |

### 5.7 `nav2_params.yaml` — Nav2-Kernkonfiguration

Alle Costmap- und Reglerwerte mit Herkunft, Abhängigkeiten und Fehlerbildern stehen in **`docs/Nav2_Costmap_Werte.md`**. Hier nur der eine Wert, der dort nicht vorkommt:

| Parameter | Ort | Wert | Bedeutung | Auswirkung |
|---|---|---|---|---|
| `allow_unknown` | `planner_server` | false | Ob der Wegplaner auch durch unbekanntes Gebiet planen darf | **Nicht auf true stellen** — sonst könnte ein Pfad durch eine (noch nicht gesehene) Wand geplant werden |
| `track_unknown_space` | `global_costmap` | true | Nie gesehene Zellen bleiben „unbekannt“ statt „frei“ | **Nicht auf false stellen** — ohne „unbekannt“ findet der Tunnel-Explorer keine Grenze zum Unerkundeten und der Bot steht |

Beide Costmaps fahren mit dem Roboter mit (`rolling_window: true`), ein Odometrie-Reset vor der Fahrt ist nicht nötig. Ein kompletter Tunnel-Durchlauf auf echter Hardware steht noch aus — vor dem Wettkampf unbedingt einplanen.

---

## 6. Wie die Hindernisvermeidung im Obstacle-Modus genau funktioniert

```mermaid
flowchart LR
    A["LiDAR: Hindernis im Kegel vorn,\nkeine Spurlinie dazwischen"] --> B["switch_control:\nModus -> Obstacle"]
    B --> C["control_driving:\nsofort Null, dann nur Nav2"]
    B --> D["nav2_goal_publisher:\nZiel 1 m voraus auf der Spurmitte"]
    D --> P["planner_server (NavFn):\nWeg um die Hindernisse,\nzwischen den gemerkten Linien"]
    P --> Q["nav2_goal_publisher:\nPfad mit dem Umriss pruefen"]
    Q -->|"passt"| E["controller_server (RPP):\nfaehrt den Weg ab,\nprueft den Umriss"]
    Q -->|"nur ausgerichtet"| G["RPP bis zum Start,\ndann Grundbewegungen hindurch"]
    Q -->|"passt nicht"| H["Engstelle sperren"]
    G --> D
    H --> D
    E -->|"Pfad blockiert, fast abgefahren,\nTakt 1 s oder Abbruch"| D
    E -->|"Kollisionsgefahr"| U["freifahren"]
    U --> D
    E -->|"1 s nichts in 0,4 m (270 Grad)"| F["switch_control:\nAlign, dann Lane"]
```

Beim Umschalten hält der Bot sofort an, bis Nav2 den ersten Befehl schickt. `nav2_goal_publisher` setzt ein Ziel 1 m voraus auf der Spurmitte, also hinter das Hindernis. `planner_server` rechnet auf der `global_costmap` einen Weg dorthin, der um die LiDAR-Hindernisse herum und zwischen den Spurlinien aus dem Spurgedächtnis verläuft. Vor dem Losfahren prüft `nav2_goal_publisher` den Weg mit dem Umriss (Engstellen, siehe unten), dann fährt RPP ihn ab und prüft dabei den Umriss gegen die `local_costmap`. Neu geplant wird, wenn eine gesperrte Zelle auf dem Pfad liegt, der Pfad fast abgefahren ist, die Fahrt endet, und jede Sekunde; der Takt-Pfad gilt nur, wenn er 30 cm voraus höchstens 45° anders zeigt (kein Kippen links/rechts um das Hindernis).

**Engstellen.** Stößt der Umriss in Pfadrichtung an einer Engstelle an, sucht die Prüfung die Gerade mit dem meisten Abstand zu den Wänden. Passt der Bot ausgerichtet hindurch, fährt RPP bis zu ihrem Start (Platz zum Drehen), dann dreht `switch_control` ihn auf die Gerade und fährt langsam gerade hindurch; danach wird neu geplant. Passt er nicht oder ist die Durchfahrt dort gescheitert, wird die Engstelle gesperrt (`/nav2/blocked`, bis Obstacle endet) und NavFn sucht einen anderen Weg.

**Abbruch.** Bei Kollisionsgefahr (RPP-Meldung in `/rosout` oder Wächter) fährt der Bot frei und plant neu; bei anderen Abbrüchen plant er sofort neu.

Hindernisse auf der anderen Seite einer Spurlinie werden ignoriert, beim Eintritt wie beim Austritt. Das Spurgedächtnis kennt auch Linien neben und hinter dem Bot, deshalb funktioniert der Filter auch für seitliche Hindernisse.

Sobald im 270°-Bereich (vorn, seitlich, schräg hinten) für `OBSTACLE_CLEAR_DEBOUNCE_S` Sekunden nichts näher als `obstacle_clear_distance` (0,4 m) liegt, schaltet `switch_control` zurück zu `Lane` (steht der Bot schräg, erst über `Align`), und der PID übernimmt wieder.

**Bekannte Grenze:** Auf der normalen Spur (20 cm zwischen den Linien, Burger 17,8 cm) ist neben einem Hindernis kein Platz. NavFn findet dort keinen Weg (Status „kein Weg gefunden“), der Bot bleibt stehen, statt die Spur zu verlassen.

**Bekannte Grenze:** Die Kamera sieht nur 5–60 cm voraus. Weiter vorn kennt das Spurgedächtnis nur, was sie auf dem Weg dorthin schon gesehen hat. Das Planziel 1 m voraus liegt deshalb anfangs teils hinter dem bekannten Korridor; der Weg wird mit jeder Neuplanung genauer.

**Bekannte Grenze:** NavFn braucht etwa 0,20 m, der echte Umriss gerade 0,18–0,19 m (Mindestbreiten in `Nav2_Costmap_Werte.md`, Abschnitt 3). Schmaler plant NavFn nicht, auch wenn der Bot ausgerichtet passen würde.

---

## 7. Wie die Tunnelfahrt genau funktioniert

Der Roboter hat **keine vorab gespeicherte Karte**. Er baut sich rein aus dem LiDAR eine Karte, die für jede Stelle **frei**, **belegt** oder **unbekannt** kennt, und vertraut dabei seiner Odometrie (kein SLAM).

**Einfahrt.** Eine Tür sind zwei Wandenden im Scan, zwischen denen die Strahlen in der Mitte mindestens eine Bot-Länge weiter reichen. Die Lücke muss mindestens so breit sein wie der Bot (17,8 cm), genommen wird die nächste vor dem Bot. Die Tür wird einmal je Tunnelfahrt festgelegt, auch wenn das Ziel dahinter scheitert. Das Ziel liegt so weit dahinter, dass der ganze Bot drin ist. Findet er keine Tür, wartet er und meldet das; er rät nicht.

**Wand-Hülle.** Von den Türkanten aus sammelt der Node alle damit verbundenen Wandzellen (bei Start im Tunnel von den nächsten Wänden links und rechts). Raumwände und freistehende Hindernisse gehören nicht dazu. Ihre konvexe Hülle ist der Innenraum: Drinnen umgeben die Wände den Bot, draußen liegen alle auf einer Seite. Erst wenn der ganze Umriss einmal drin war, wird die Ausfahrtprüfung scharf.

**Zielwahl in fester Reihenfolge:**

1. **Öffnung:** Strahlen, die die Hülle verlassen, zeigen auf eine Lücke, außer am Eingang. Das Ziel dahinter muss für NavFn erreichbar sein und außerhalb der Hülle liegen; sonst war die Lücke zu eng.
2. **Frontier in der Hülle mit Unbekanntem an der Wand** (bis 0,3 m innerhalb der Hülle, gezählt bis 0,4 m um das Ziel): 1 m² davon ist 4 m Umweg wert, gewählt wird der beste Wert. Ab 40 s nach dem Eintritt (Tunnel-Uhr) nur noch das nächste solche Ziel. Graues tiefer in der Hülle ist Schatten hinter Hindernissen, kein Ausgang, und zählt nicht.
3. **Frontier jenseits der Eingangslinie**, weit weg vom Eingang (Rückfall).
4. **Weitester erreichbarer Punkt.** Gibt es auch den nicht, meldet der Node das.

Befahrbar heißt dabei immer: was NavFn befahren darf. Besuchte und gescheiterte Ziele werden 15 s lang nicht wieder gewählt (danach wieder, falls sich die Lage geändert hat).

**Engstellen: vor dem Losfahren prüfen.** Jeder neue Pfad wird bis 1 m voraus mit dem Umriss in Pfadrichtung abgefahren (`passage_geometry.check_path`). Stößt er an einer Engstelle an (Wandzellen links und rechts), sucht der Explorer die Gerade, auf der der Umriss den meisten Abstand zu den Wandzellen hat (um die Achse aus den Kanten ±30° gedreht und ±3 cm verschoben). Start ist der letzte Punkt davor mit 11 cm Platz zum Drehen auf der Stelle (Umkreis des Footprints 10,5 cm), Ende der Punkt, an dem das Heck 3 cm hinter der Wand ist; deren Rückseite sieht der LiDAR von vorn nicht, angenommen werden 2 cm Wandstärke (Tunnelwand höchstens 2 cm).
- **Luft auf der Geraden:** RPP fährt nur bis zum Start (gekürzter Pfad), dann fordert der Explorer bei `switch_control` die Durchfahrt an (`/control/maneuver_request`): zum Start (vor oder zurück), auf der Stelle Richtung Ende drehen, mit 0,05 m/s geradeaus. Danach (`/control/maneuver_sequence`) plant er zum selben Ziel neu.
- **keine Luft oder Durchfahrt dort schon gescheitert:** Der Explorer **sperrt** die Engstelle mit einer Linie von Wand zu Wand (`/nav2/blocked`, `entrance_layer`); NavFn sucht eine andere Route.
- **kein Platz zum Drehen davor oder keine Engstelle** (nur eine Wand): RPP fährt den Pfad, Kollisionsprüfung und Wächter passen auf.

Es gibt keine festen Breiten: Es zählt nur, ob der Umriss passt. Sperren gelten bis zum Ende der Tunnelfahrt oder Taste L; beim Verlassen des Tunnels geht eine leere Wolke hinaus.

**Abbruch des Reglers.** Meldet `nav2_goal_publisher` Kollisionsgefahr (RPP oder Wächter), fährt der Bot frei (höchstens zweimal je Ziel) und plant neu. Bei anderen Abbrüchen plant er einmal neu, beim zweiten gilt das Ziel als gescheitert. Gibt es kein Ziel mehr, wartet der Bot und meldet „Kein Weg nach draußen gefunden“.

**Neuplanung.** Ein Ziel wechselt bei Ankunft, wenn es unerreichbar wird oder wenn eine Öffnung auftaucht. Zum selben Ziel wird jede Sekunde neu geplant; der neue Pfad gilt nur, wenn er 30 cm voraus höchstens 45° anders zeigt oder der alte blockiert ist, und nicht auf dem Weg durch eine Engstelle. Anders als im Obstacle-Modus gibt es keine Rückwärtssperre, der Bot muss aus Sackgassen wieder herauskommen.

**Bekannte Grenzen:** Berührt die Enclosure eine Raumwand oder steht etwas dicht daneben, wird die Hülle zu groß: Der Wechsel kommt dann zu spät, und es bleibt der Abbruch von Hand. Eine sehr verwinkelte Enclosure kann Zeit kosten, das Reglement gibt für die Ausfahrt 1 Minute.

---

## 8. Checkliste vor dem Wettkampf

**Obstacle-Modus:**
- [ ] In RViz prüfen, ob `/detect/lane_obstacles` auf den echten Linien liegt (BEV/Extrinsik)
- [ ] In RViz prüfen, ob die Spurwände beim Ausweichen stehen bleiben (Spurgedächtnis) und ob der „Aktive Pfad (Nav2)“ um die Hindernisse herum zwischen den Linien liegt
- [ ] Rückweg neben dem Hindernisbereich (Hindernisse hinter der Linie): Modus muss `Lane` bleiben
- [ ] `line_confirm_frames`/`free_confirm_frames` prüfen: erscheinen Linien schnell genug, verschwinden Fehldetektionen?
- [ ] Prüfen, ob `controller_server` (RPP mit Kollisionsprüfung) und der Wächter die 20 Hz halten (Rechenlast)
- [ ] `inflation_radius` im Hindernisbereich testen (Zug zur Mitte vs. enge Durchfahrten)
- [ ] `obstacle_check_angle_deg`/`obstacle_trigger_distance`/`obstacle_clear_distance`/`OBSTACLE_CLEAR_DEBOUNCE_S` mit dem echten Roboter/der echten Strecke testen (löst zu früh/zu spät aus? schaltet zu oft hin und her?)
- [ ] Kompletten Ausweich-Durchlauf auf echter Hardware testen (ein Hindernis in die Fahrbahn stellen, beobachten ob sauber ausgewichen und zurückgeschaltet wird)

**Tunnel-Modus:**
- [ ] Findet die Türsuche die echte Öffnung, auch bei schräger Anfahrt? Stimmt die im Log gemeldete Breite?
- [ ] `inflation_radius` der local_costmap in der Enclosure testen (Abstand zu Wänden vs. enge Durchfahrten)
- [ ] Einfahrt: schaltet `TunnelEntry` → `Tunnel` am Linienende um, Bot-Nase noch vor der Öffnung? Liegt die orange Eingangssperre zwischen den Türkanten?
- [ ] Ausfahrt: Wechselt der Modus erst, wenn der ganze Bot draußen ist? Berührt die Enclosure eine Raumwand, wird die Hülle zu groß und der Wechsel kommt zu spät: dann von Hand abbrechen.
- [ ] Kompletten Tunnel-Durchlauf auf echter Hardware testen
- [ ] In beiden `rqt_image_view`-Fenstern „Publish click location“ anhaken und prüfen, ob der Tunnel-Button im Debugbild und in der Nav2-Draufsicht umschaltet
- [ ] Draufsicht `nav2_view` prüfen: Erscheint der Umriss (`published_footprint`), und taucht ein Abbruchgrund von `controller_server` in der Statuszeile auf?
- [ ] Schild-Erkennung (YOLO auf `Kreuzung-Yolo-Schranke`) an `/detect/sign` anbinden; testbar per `ros2 topic pub /detect/sign std_msgs/msg/UInt8 "{data: 6}"`

**Allgemein:**
- [ ] LiDAR-Fehler messen und in `Scan_Correct.yaml` (`range_offset_m`) eintragen
- [ ] Fahrtprotokoll einer Tunnelfahrt in `logs/tunnel/` ansehen (`map.png`, `events.jsonl`)
