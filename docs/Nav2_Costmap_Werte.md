# Nav2-Costmap-Werte: Herkunft und Abhängigkeiten

Für alle, die einen der Werte in `Parameter/nav2_params.yaml` oder
`Parameter/Lane_Costmap.yaml` ändern müssen. Zu jedem Wert steht hier, was er
bewirkt, woher er kommt und was kaputtgeht, wenn er falsch steht. Viele Werte
hängen voneinander ab; diese Abhängigkeiten sind als nachrechenbare Regeln
angegeben.

Pfade sind relativ zu `workspace/src/Follow_Lane/Follow_Lane/`. Maße der
Strecke und des Roboters stammen aus dem Reglement 2026: normale Spur 20 cm
zwischen den Linien, Linien ~2 cm, Burger 138 × 178 mm (L × B), max. 0,22 m/s.
Der Hindernisbereich außerhalb des Tunnels ist breiter, seine Maße sind vorab
nicht bekannt.

**Herkunft** heißt:
- *gemessen*: am Bot oder an der Strecke vermessen
- *abgeleitet*: aus anderen Werten berechnet, die Rechnung steht dabei
- *gewählt*: eine Entscheidung, die man begründet ändern darf
- *nicht dokumentiert*: die Dateien enthalten keine Begründung, nichts geraten

**Wer wann was sieht:**

| Modus | fährt | local_costmap enthält |
|---|---|---|
| Lane | PID (`control_driving`) | LiDAR + Spurwände (Nav2 ruht) |
| Obstacle | Nav2/MPPI | LiDAR + Spurwände |
| Tunnel | Nav2/MPPI | LiDAR + Eingangssperre, **keine Kamera** (Reglement) |

Beim Tunnel-Eintritt leert `lane_costmap_node` die local_costmap. Warum keiner
der Werte von der Breite der Spur abhängt, steht in Abschnitt 3.

---

## 1. Kartengeometrie

| Wert | local_costmap | global_costmap |
|---|---|---|
| `rolling_window` | `true` | `true` |
| `width` / `height` | 4 / 4 m | 7 / 7 m |
| `resolution` | 0.05 m | 0.05 m |
| Robotergröße | `footprint` (Umriss, 138 × 178 mm) | `robot_radius` 0.105 m |
| `global_frame` | `odom` | `odom` |

### `rolling_window`
- **Was er bewirkt:** Die Karte fährt mit dem Roboter mit und ist immer um ihn
  zentriert.
- **Woher:** gewählt.
- **Was kaputtgeht:** Bei `false` ist die Karte fest um den odom-Nullpunkt
  verankert. Wenn man den Bot zwischen Testfahrten von Hand zurückträgt, drehen
  die Räder nicht, die Odometrie zählt also einfach weiter. Nach wenigen Läufen
  liegt der Bot rechnerisch außerhalb der Karte, und `planner_server` findet
  keinen Pfad mehr. Mit `true` spielt das keine Rolle, und **vor einer Fahrt muss
  die Odometrie nicht zurückgesetzt werden**. `origin_x`/`origin_y` werden bei
  `rolling_window: true` ignoriert und stehen deshalb nicht mehr in der Datei.
- Nebenwirkung bei der global_costmap: Was aus dem 7×7-m-Fenster herausrollt,
  ist vergessen. Für die Enclosure reicht das.

### `width` / `height`
- **Was sie bewirken:** Kantenlänge der Karte in Metern. Die Karte reicht in
  jede Richtung die halbe Kantenlänge weit.
- **Woher:** abgeleitet. Zwei Obergrenzen bestimmen den Wert:
  1. **Die Reichweite, die der Sensor aktuell halten kann.** Der LDS-03 misst
     bis 12 m, hat aber nur 0,9° Winkelauflösung. Der Abstand zwischen zwei
     Strahlen ist `r · 0,9° · π/180 = r · 0,0157`. Ab etwa **3,2 m** ist dieser
     Abstand größer als eine Zelle (0,05 m). Darüber wird das Freiräumen
     lückenhaft, und schmale Hindernisse fallen zwischen zwei Strahlen durch.
     Eine halbe Kartenbreite weit über ~3,2 m hinaus bringt also nur unsichere
     Zellen. Global: 7 m → 3,5 m halbe Breite, passend zu `raytrace_max_range` 3.5.
  2. **Nur local_costmap: die Vorausschau des Reglers.** MPPI plant aktuell
     etwa 1 m nach vorn (siehe Abschnitt 5). Was weiter weg liegt, kostet bei
     `update_frequency` 10 Hz nur Rechenzeit. 4×4 m (2 m halbe Breite) deckt
     die Vorausschau ab und lässt `prune_distance` 1.7 genug Platz. Bei 3×3 m
     sah der Regler laut Kommentar in der Datei nur 1,5 m weit, zu knapp fürs
     Umfahren bei einem 1-m-Ziel.
- **Was kaputtgeht:** Ist die Karte zu klein, ragen Pfad oder Vorausschau über
  den Rand hinaus. Ist sie zu groß, landen unsichere Zellen in der Karte und die
  Updates brauchen mehr Rechenzeit. Bei `prune_distance` ≥ halbe Breite der
  local_costmap reicht der Pfad über die Karte hinaus.

### `resolution`
- **Was er bewirkt:** Kantenlänge einer Zelle, 0,05 m.
- **Woher:** gewählt. Die Dateien begründen den Wert nicht.
- **Was kaputtgeht:** Andere Werte hängen an ihm: die 3,2-m-Grenze oben und
  die Quantisierung des `inflation_radius` (Abschnitt 3). Außerdem wird jede
  Linie auf ganze Zellen vergröbert. Wo nur wenige Zentimeter Luft sind, kann
  der Footprint deshalb schon eine Linienzelle berühren, obwohl real noch Platz
  wäre. Wer `resolution` ändert, muss diese Werte neu rechnen.

### `footprint` (local_costmap)
- **Was er bewirkt:** der Umriss des Burger relativ zu `base_footprint`. Mit
  `consider_footprint: true` im `CostCritic` gilt eine Trajektorie als
  Kollision, sobald dieser Umriss eine belegte Zelle berührt. Das gilt für
  Linien, Hindernisse und Enclosure-Wände gleichermaßen.
- **Woher:** abgeleitet aus 138 × 178 mm (Reglement) und der TurtleBot3-URDF.
  Der Rumpf sitzt dort 3,2 cm hinter der Radachse, reicht also 3,7 cm nach vorn
  und 10,1 cm nach hinten, seitlich ±6,9 cm. Die Räder liegen bei ±8,9 cm außen
  und ±3,3 cm vor und hinter der Achse. Eingetragen ist die konvexe Hülle aus
  beidem. Nav2 legt zusätzlich `footprint_padding` (Standard 1 cm) darum.
- **Was kaputtgeht:** Ein zu kleiner Umriss lässt den Bot Linien überfahren und
  Wände streifen, ein zu großer macht enge Durchfahrten unpassierbar. Da alle
  Burger laut Reglement identisch sind, muss der Wert nie angepasst werden.

### `robot_radius` (global_costmap)
- **Was er bewirkt:** Zellen, die näher als dieser Radius an einem Hindernis
  liegen, gelten für den Mittelpunkt des Roboters als Kollision. Nur für
  `planner_server` (NavFn) im Tunnel. NavFn plant für einen Punkt und prüft
  keinen Umriss, deshalb bleibt hier der Kreis.
- **Woher:** 1,6 cm über der halben Breite des Burger (178 mm / 2 = 0,089 m).
- **Was kaputtgeht:** Ist der Wert zu klein, plant NavFn Wege zu dicht an den
  Enclosure-Wänden. Jede Wandberührung kostet einen Punkt.

---

## 2. Sensorreichweiten der Layer

Die Reichweiten werden ab dem Ursprung der Quelle gemessen. Weil überall
`sensor_frame: ""` steht, ist das der Ursprung des Frames, in dem die Nachricht
ankommt.

| Layer (Karte) | Quelle | Frame der Quelle | obstacle min / max | raytrace min / max | clearing |
|---|---|---|---|---|---|
| `obstacle_layer` (local + global) | `/scan` | LiDAR | 0.0 / 3.0 m | 0.0 / 3.5 m | `true` |
| `lane_layer` (nur local) | `/detect/lane_obstacles` | `base_footprint` | 0.0 / 2.0 m | 0.0 / 2.0 m | `true` |
| `entrance_layer` (local + global) | `/detect/tunnel_entrance_obstacle` | `odom` | 0.0 / 100.0 m | 0.0 / 3.0 m | `false` |

### `obstacle_layer` (LiDAR)
- **`obstacle_max_range` 3.0:** Bis zu dieser Entfernung tragen Messpunkte ein
  Hindernis ein. *Woher:* abgeleitet, knapp unter der 3,2-m-Grenze aus
  Abschnitt 1. Weiter draußen werden die Lücken zwischen den Strahlen zu groß.
- **`raytrace_max_range` 3.5:** Bis hierhin räumt ein Strahl die Zellen auf
  seinem Weg frei. *Woher:* gewählt, etwas größer als `obstacle_max_range`,
  damit alte Hindernisse am Rand des Markierbereichs auch wieder verschwinden.
  Das entspricht der halben Breite der global_costmap.
- **`obstacle_min_range` / `raytrace_min_range` 0.0:** Kein zusätzlicher
  Nahfilter. Der LDS-03 liefert ohnehin erst ab 0,05 m (range_min).
- **Was kaputtgeht:** Ist `obstacle_max_range` zu groß, stehen lückenhafte
  Einträge in der Karte. Ist `raytrace_max_range` kleiner als
  `obstacle_max_range`, bleiben Geisterhindernisse am Rand stehen.

### `lane_layer` (Spurwände)
- Wird in jedem Modus außer Tunnel gefüllt. Er steht in der `plugins`-Liste
  **vor** dem `inflation_layer` (siehe Abschnitt 3).
- **`clearing: true`:** Die Strahlen von `base_footprint` zu den aktuellen
  Linienpunkten löschen alte Spurpunkte, die dazwischen liegen. Das wirkt nur
  auf die Zellen des `lane_layer`. Spurpunkte außerhalb des Kamerabilds bleiben
  erhalten. Das ist das Gedächtnis, das den Korridor neben und hinter dem Bot
  hält.
- **`obstacle_max_range` / `raytrace_max_range` 2.0:** Gemessen ab
  `base_footprint`. *Woher:* nicht dokumentiert. Faktisch **begrenzen sie
  nichts**: `lane_costmap_node` schickt nur Punkte aus der BEV, also bis 0,60 m
  voraus und ±0,40 m seitlich (zusammen < 0,75 m Abstand).

### `entrance_layer` (Tunnel-Eingangssperre)
- **`clearing: false`:** *Woher:* gewählt, siehe Kommentar in der Datei. Die
  Punkte sind virtuell und haben keinen echten Sensorursprung. Ein Raytrace
  liefe entlang einer willkürlichen Linie im odom-Frame und würde dabei echte
  Hindernisse in diesem Layer mitlöschen. Deshalb wirkt `raytrace_*` hier nicht.
- **`obstacle_max_range` 100.0:** Weil die Wolke im Frame `odom` ankommt, zählt
  diese Reichweite **ab dem odom-Nullpunkt**, nicht ab dem Roboter. *Woher:*
  gewählt, praktisch unbegrenzt. Was außerhalb des mitfahrenden Fensters liegt,
  verwirft die Costmap ohnehin. *Was kaputtging:* Mit 3.0 fiel die Sperre ohne
  Meldung weg, sobald der Tunneleingang mehr als 3 m vom Startpunkt entfernt
  lag. Das ist aus der Nav2-Logik abgeleitet, auf dem Bot nicht beobachtet.

---

## 3. Kollision und Inflation: unabhängig von der Spurbreite

| Wert | local_costmap | global_costmap |
|---|---|---|
| `consider_footprint` (`CostCritic`) | `true` | – |
| `inflation_radius` | 0.20 m | 0.3 m |
| `cost_scaling_factor` | 3.0 | 3.0 |

### Das Prinzip

Die Breite des Hindernisbereichs ist vorab nicht bekannt, und die normale Spur
ist mit 20 cm nur knapp breiter als der Burger (17,8 cm). Deshalb hängt kein
Wert von der Korridorbreite ab:

- **Kollision** prüft MPPI mit dem echten Umriss (`footprint`, Abschnitt 1).
  Eine Trajektorie ist verboten, wenn der Umriss eine Linienzelle oder ein
  Hindernis berührt, und erlaubt, wenn er dazwischen passt, egal wie breit der
  Bereich ist.
- **Inflation** ist kein Kollisionskriterium mehr, nur ein Kostengefälle. In
  einem schmalen Korridor überlappen sich die Säume beider Linien. Die Mitte
  bleibt trotzdem die günstigste Stelle, und MPPI wird zu ihr gezogen. Eine
  „Zelle mit Kosten 0“ ist dafür nicht nötig.

### Warum der `lane_layer` vor dem `inflation_layer` steht

MPPI prüft den Umriss aus Rechenzeitgründen nur dort, wo die Kosten am
Mittelpunkt schon erhöht sind, also im Saum einer belegten Zelle. Stünde der
`lane_layer` hinter dem `inflation_layer`, hätten die Linien keinen Saum.
MPPI würde dann an den Linien gar nicht mit dem Umriss prüfen, und die Räder
könnten unbemerkt über eine Linie fahren.

### `inflation_radius`
- **Was er bewirkt:** Um jede belegte Zelle (LiDAR-Hindernis, Spurlinie,
  Enclosure-Wand, Eingangssperre) zieht der `inflation_layer` einen
  Kostengradienten. Ohne ihn hätte die Zelle direkt neben einer Wand noch
  Kosten 0, und der Regler wiche erst im letzten Moment aus.
- **Woher:** gewählt, nicht vermessen. Global 0.3 m für `planner_server`.
- **Bedingung:** `inflation_radius` > Umkreis des Footprints (0,122 m +
  `footprint_padding`). Sonst beginnt der Saum zu weit innen, und die Prüfung
  mit dem Umriss setzt zu spät ein (siehe oben).
- **Was kaputtgeht:** Ist der Wert zu klein, fährt der Bot dicht an Wänden und
  Linien entlang. Ist er zu groß, werden enge Durchfahrten teuer und MPPI
  zögert. **Im Hindernisbereich und in der Enclosure testen.**
- Nav2 rundet den Inflationsradius auf ganze Zellen auf. Werte, die kein
  Vielfaches von `resolution` (0.05) sind, wirken wie das nächste größere
  Vielfache.

### Auf der normalen 20-cm-Spur

Dort fährt im Normalfall der PID, nicht Nav2. Löst der Obstacle-Modus trotzdem
aus, bleibt zwischen Umriss und Linienzellen nur ~1 cm Luft pro Seite, weniger
als eine Zelle. Erwartbar findet MPPI dann keine kollisionsfreie Trajektorie,
und der Bot bleibt stehen, statt die Spur zu verlassen. Das ist aus der
Konfiguration abgeleitet und nicht auf dem Bot beobachtet.

---

## 4. Spurwände (`Lane_Costmap.yaml`, `/lane_costmap_node`)

| Wert | aktuell |
|---|---|
| `pixel_stride` | 2 |
| `frame_id` | `base_footprint` |
| Umrechnung Pixel → Meter | aus `Parameter/bev.yaml` |

Der Node sendet in jedem Modus außer Tunnel. Die Linienpixel stammen aus der
metrischen Vogelperspektive `/lokal/bev`, die `ingress_node` aus dem Rohbild
und der vermessenen Extrinsik (`Parameter/camera_info/extrinsics_800x600.yaml`)
erzeugt. `detect_lane` erkennt die Linien direkt in diesem Bild. In der BEV ist
die Umrechnung in Meter linear. Sie steht in `Parameter/bev.yaml`, derselben
Datei, mit der `ingress_node` die BEV baut:

```
X = x_max - (v + 0.5) * res        X vorwärts ab Radachse
Y = y_half - (u + 0.5) * res       Y nach links
```

Das entspricht genau `base_footprint`. Aktuell (`bev.yaml`): 1,25 mm pro
Pixel, X von 0,05 bis 0,60 m, Y ±0,40 m, also 640 × 440 px. Was die Kamera
nicht sieht, ist in der BEV schwarz und liefert keine Linienpunkte. Mit der
aktuellen Extrinsik (Höhe 10,1 cm, Neigung 38,0°) sieht die Kamera geradeaus
ab etwa 0,07 m. Seitlich sind es bei 0,20 m etwa ±0,23 m und bei 0,60 m etwa
±0,65 m. Das ist aus der Extrinsik gerechnet, nicht gemessen. Die volle
BEV-Breite von ±0,40 m ist damit ab etwa 0,35 m abgedeckt.

**Genauigkeit:** hängt an der Extrinsik (Höhe, Neigung, Roll der Kamera). Wie
genau die BEV am Boden ist, steht in `docs/bev-tests/`. In diesem Dokument ist
das nicht nachgemessen.

### Bildgröße
- Weicht die Größe der `LaneMask` von dem ab, was `bev.yaml` beschreibt, loggt
  der Node einmal einen Fehler und sendet keine Spurwände. Die Meter wären sonst
  falsch. Wer `bev.yaml` ändert, muss `ingress_node` und `lane_costmap_node` neu
  starten. Beide lesen die Datei nur beim Start.

### Reichweite
- Bestimmt durch `x_max_m` in `bev.yaml` (0,60 m). Laut Kommentar dort wird es
  dahinter in Fahrtrichtung sehr unscharf. Ein eigener Grenzwert im Node ist
  nicht nötig.
- **Folge für breite Bereiche:** Bei 50 cm Spurbreite kommen beide Linien ab
  etwa 0,23 m ins Bild. Bis 0,60 m bleiben gut 30 cm, in denen die Breite
  gemessen und die Mitte berechnet wird (siehe „Spurmitte“).

### `pixel_stride`
- **Was er bewirkt:** Nur Pixel, deren x und y Vielfache von `pixel_stride`
  sind, werden umgerechnet.
- **Woher:** gewählt. Er wirkt zusammen mit `lane_mask_stride` (Default 2 in
  `detect_lane`, nicht in einer YAML gesetzt), das schon beim Senden ausdünnt.
  Solange beide 2 sind, dünnt `pixel_stride` nichts weiter aus. Bei 1,25 mm/px
  sind das 2,5 mm Abstand, weit unter einer Costmap-Zelle.

### `frame_id`
- `base_footprint` liegt in der TurtleBot3-URDF (burger) am Boden direkt unter
  der Radachse und passt damit zur Konvention der BEV (X ab Radachse). Nicht
  ändern, solange `bev.yaml` ab Radachse rechnet.

### Spurmitte als Pfad für MPPI

Aus denselben Linienpunkten berechnet `lane_costmap_node` die Spurmitte und
sendet sie als `nav_msgs/Path` auf `/lane/center_path`. Im Obstacle-Modus ist
sie das Ziel für MPPI (Abschnitt 5). So folgt der Bot auch in Kurven der Spur
statt einer Geraden, die irgendwann in die Spurwand zeigt.

**Verfahren** (Funktionen `lane_center_points`, `fit_center_line`):
1. Pro Linie und pro 5-cm-Streifen in Fahrtrichtung den Median-Punkt der
   Linienpixel nehmen. Weiß links der gelben Linie gehört nicht zur Spur und
   fällt heraus.
2. Jede Linie für sich durch ein Polynom 2. Grades annähern, wie ROBOTIS in
   `turtlebot3_autorace`.
3. **Spurbreite messen**, wo beide Linien zu sehen sind: senkrecht zur Linie,
   von Linienmitte zu Linienmitte. Die Linienbreite fällt dabei heraus.
4. Jeden Linienpunkt senkrecht zur Linie um die halbe Breite zur Mitte
   versetzen. Senkrecht, weil in Kurven der Abstand entlang Y größer ist als
   die echte Breite.
5. Durch alle Mittelpunkte ein Polynom 2. Grades legen und alle 5 cm abtasten,
   nur zwischen nächstem und fernstem Mittelpunkt.
6. Mit der TF `odom ← base_footprint` **zum Bildzeitpunkt** nach `odom`
   umrechnen. Dort bleibt der Pfad liegen, auch wenn der Bot beim Ausweichen
   schräg steht und die Linien kurz nicht sieht.

Sieht ein Bild nur eine Linie (Lücke in Weiß, enge Kurve, Hindernis verdeckt),
gilt die Breite aus den letzten Bildern (`center_width_history`). ROBOTIS
versetzt an dieser Stelle um feste 280 Pixel; hier wird die Breite gemessen,
damit derselbe Code auf der 20-cm-Spur und im breiteren Hindernisbereich
funktioniert.

| Wert | aktuell | Herkunft |
|---|---|---|
| `center_path_topic` | `/lane/center_path` | gewählt |
| `odom_frame` | `odom` | = `global_frame` der Costmaps |
| `center_bin_size` | 0.05 m | = `resolution`, feiner kann MPPI nicht unterscheiden |
| `center_min_points` | 3 | gewählt; darunter kein Pfad |
| `center_width_history` | 20 Bilder | gewählt; Median der gemessenen Breite |

**Offline geprüft** (synthetische Spuren direkt auf das BEV-Raster aus
`bev.yaml` gelegt, Sichtfeld aus der aktuellen Extrinsik ausgeblendet,
`lane_mask_stride` 2 wie in `detect_lane`, nicht auf dem Bot). Das prüft die
Rechnung, nicht die Genauigkeit der Extrinsik:

| Fall | Pfad von–bis | Abweichung von der echten Mitte |
|---|---|---|
| gerade, 20 cm | 0,09–0,54 m | 0,6 mm |
| gerade, 50 cm | 0,24–0,54 m | 0,7 mm |
| Kurve R = 1,2 m, 20 cm | 0,08–0,58 m | 0,7 mm |
| Kurve R = 0,7 m, 20 cm, Breite gelernt | 0,09–0,59 m | 5,1 mm |
| Kurve R = 0,7 m, nur Gelb, Breite gelernt | 0,09–0,49 m | 3,5 mm |

**Grenzen:**
- Die Spurmitte reicht höchstens bis zum Ende der BEV (0,60 m). Mit `x_max_m`
  größer als 0,60 m in `bev.yaml` würde sie länger, aber laut `bev.yaml` wird
  die BEV dort unscharf.
- Sieht ein Bild nur eine Linie (Lücke in Weiß, verdeckt), gilt die gelernte
  Breite. Ist sie noch unbekannt, gibt es in diesem Bild keinen Pfad.
- Gelbe oder weiße Linien, die quer durch den Hindernisbereich laufen, würden
  die Anpassung verfälschen. Das muss man vor Ort in RViz prüfen (Anzeige
  „Spurmitte“).

---

## 5. Regler und Vorausschau

| Wert | Datei | aktuell |
|---|---|---|
| `controller_frequency` | nav2_params | 20.0 Hz |
| `model_dt` | nav2_params | 0.05 s |
| `time_steps` | nav2_params | 90 |
| `vx_max` | nav2_params | 0.22 m/s |
| `prune_distance` | nav2_params | 1.7 m |
| `goal_distance` | Lane_Costmap (`/nav2_goal_publisher`) | 1.0 m (nur Ersatz-Gerade) |
| `center_path_max_age` | Lane_Costmap (`/nav2_goal_publisher`) | 1.0 s |

### Die Zusammenhänge

```
Vorausschau [m]       =  time_steps * model_dt * vx_max
controller_frequency  == 1 / model_dt
```

Bedingungen:
- **Vorausschau ≥ `goal_distance`.** Aktuell 90 · 0,05 · 0,22 = **0,99 m** bei
  1,0 m `goal_distance`. Das deckt die Strecke bis auf 1 cm ab. Wer es exakt
  will, nimmt 91 Schritte.
- **`prune_distance` < halbe Breite der local_costmap.** 1,7 m < 2,0 m.
- **Reglerperiode ≤ `model_dt`.** 1/20 Hz = 0,05 s = `model_dt`. Ist die
  Reglerperiode größer als `model_dt`, lehnt MPPI `configure` ab („Controller
  period more then model dt“), und der ganze Nav2-Zweig bleibt inaktiv.

### Einzelwerte
- **`time_steps`:** Anzahl der Schritte, die MPPI vorausrechnet. *Woher:*
  abgeleitet aus der Formel oben. *Was kaputtging:* Bei 56 Schritten war die
  Vorausschau 2,8 s · 0,22 = 0,62 m. Das 1-m-Ziel war nie im Blick.
- **`model_dt`:** Zeitschritt der Simulation. *Woher:* gewählt, gekoppelt an
  `controller_frequency`.
- **`controller_frequency`:** *Woher:* abgeleitet, = 1 / `model_dt`.
- **`vx_max`:** *Woher:* Höchstgeschwindigkeit des Burger laut Reglement
  (0,22 m/s). *Was kaputtgeht:* Wer ihn senkt, verkürzt die Vorausschau und muss
  `time_steps` nachziehen.
- **`prune_distance`:** Wie weit voraus MPPI den Pfad berücksichtigt. *Woher:*
  gewählt, durch die Kartengröße begrenzt.
- **Ziel im Obstacle-Modus:** die Spurmitte (Abschnitt 4), solange sie jünger
  ist als `center_path_max_age` (Empfangszeit, damit die Uhren von Bot und
  Rechner nicht synchron sein müssen). Sie reicht bis ~0,8 m voraus. Ist sie
  älter, weil die Linien eine Weile nicht zu sehen waren, gilt die Gerade
  `goal_distance` (1,0 m) geradeaus. Ein neues Ziel kommt erst, wenn das alte
  erreicht (`xy_goal_tolerance`) oder abgebrochen ist; dann wird die jeweils
  neueste Spurmitte genommen. *Woher:* gewählt.

---

## 6. Kritiken (MPPI `FollowPath`)

Die Schwelle `threshold_to_consider` wirkt je nach Kritik in entgegengesetzter
Richtung:
- **Pfad-Kritiken** (`PathFollowCritic`, `PathAlignCritic`, `PathAngleCritic`,
  `PreferForwardCritic`) greifen nur, wenn das Ziel **weiter weg** ist als ihr
  Schwellwert.
- **Ziel-Kritiken** (`GoalCritic`, `GoalAngleCritic`) greifen nur, wenn das Ziel
  **näher** ist als ihr Schwellwert.

Bei Zieldistanzen zwischen 1,0 und 0,15 m im Obstacle-Modus (Abschnitt 5)
ergibt das:

| Kritik | enabled | Gewicht | `threshold_to_consider` | greift im Obstacle-Modus |
|---|---|---|---|---|
| `ConstraintCritic` | true | 4.0 | – | immer |
| `CostCritic` | true | 20.0 | – (`near_goal_distance` 0.5) | immer |
| `GoalCritic` | true | 5.0 | 1.4 | immer (Ziel stets < 1,4 m) |
| `GoalAngleCritic` | **false** | 3.0 | 0.5 | nie (abgeschaltet) |
| `PathAlignCritic` | true | 2.0 | 0.5 | Ziel > 0,5 m |
| `PathFollowCritic` | true | 3.0 | 0.5 | Ziel > 0,5 m |
| `PathAngleCritic` | true | 2.0 | 0.5 | Ziel > 0,5 m |
| `PreferForwardCritic` | true | 5.0 | 0.5 | Ziel > 0,5 m |

- **`CostCritic`** (20.0, höchstes Gewicht): bewertet die Costmap-Kosten. Das
  Ausweichen soll dominieren. `critical_cost` 600, `collision_cost` 1e6.
  `consider_footprint: true`: Kollision prüft er mit dem Umriss des Burger
  (Abschnitt 3).
- **`PathFollowCritic` Schwelle 0.5:** Vorher stand 1.4 m. Bei 1,0 m Pfadlänge
  griff die Kritik damit nie, ausgerechnet die, die den Vorwärtsdrang halten
  soll.
- **`GoalAngleCritic` abgeschaltet:** Die Ersatz-Gerade endet mit
  `orientation.w = 1.0`, also „geradeaus, gemessen im Moment der
  Pfaderzeugung“. Ab 0,5 m vor dem Ziel zog die Kritik den Bot in genau diese
  Ausgangsrichtung zurück, während er am Hindernis vorbei sollte. Der
  `goal_checker` lässt das Ziel mit `yaw_goal_tolerance` 3.15 ohnehin
  durchgehen, die Endausrichtung spielt keine Rolle.
- **`PathAlignCritic` niedrig gewichtet (2.0):** Unser Pfad (Spurmitte oder
  Gerade) führt *durch* das Hindernis, kein geplanter Pfad darum herum. Mit 14.0 (Nav2-Beispiel)
  hat MPPI das Hindernis vor sich hergeschoben statt auszuweichen (Test 02.09.).
  „Auf der Linie bleiben“ ist deshalb nur ein Hinweis. Zusätzlich schaltet
  `max_path_occupancy_ratio` 0.05 die Kritik ab, sobald mehr als 5 % der
  Pfadpunkte in belegten Zellen liegen.

Im Tunnel-Modus kommt der Pfad von `planner_server` zu einem Frontier-Ziel.
Die Distanzen sind dort andere, die Regeln für die Schwellen gelten genauso.

---

## 7. Zeitverhalten

| Wert | Wo | aktuell | Herkunft |
|---|---|---|---|
| `update_frequency` | local_costmap | 10.0 Hz | nicht dokumentiert |
| `publish_frequency` | local_costmap | 5.0 Hz | nicht dokumentiert |
| `update_frequency` | global_costmap | 2.0 Hz | nicht dokumentiert |
| `publish_frequency` | global_costmap | 1.0 Hz | nicht dokumentiert |
| `required_movement_radius` | progress_checker | 0.1 m | nicht dokumentiert |
| `movement_time_allowance` | progress_checker | 3.0 s | gewählt |
| `xy_goal_tolerance` | goal_checker | 0.15 m | nicht dokumentiert |
| `yaw_goal_tolerance` | goal_checker | 3.15 rad | gewählt |

- **`update_frequency`:** Wie oft die Layer neu in die Karte geschrieben werden.
  Lokal 10 Hz entspricht der LiDAR-Rate des LDS-03. *Was kaputtgeht:* Ist der
  Wert zu niedrig, reagiert der Regler auf veraltete Hindernisse. Ist er zu
  hoch, kostet das CPU ohne neue Daten.
- **`publish_frequency`:** Wie oft die Karte auf dem Topic erscheint (RViz,
  `tunnel_explorer_node`). Sie beeinflusst den Regler nicht, denn der liest die
  Karte direkt.
- **`movement_time_allowance` / `required_movement_radius`:** Bewegt sich der
  Bot 3 s lang nicht um mindestens 0,1 m, bricht der Regler das Ziel ab, und
  `nav2_goal_publisher` setzt ein neues. *Was kaputtging:* Mit 10 s drückte
  MPPI am Hindernis zehn Sekunden dagegen.
- **`xy_goal_tolerance`:** Ab dieser Entfernung gilt das Ziel als erreicht, und
  das nächste wird gesetzt. Damit endet jedes Segment bei ~0,15 m.
- **`yaw_goal_tolerance` 3.15:** Größer als π, also wird jede Endausrichtung
  akzeptiert. Gewollt, siehe `GoalAngleCritic`.

---

## Prüfen in RViz

| Display | zeigt |
|---|---|
| `/local_costmap/costmap` | alle Layer der local_costmap zusammen, dazu die Inflation. Außerhalb des Tunnels auch die Spurwände. |
| `/global_costmap/costmap` | die Tunnelkarte (LiDAR + Eingangssperre, keine Spur), 7×7 m um den Bot |
| `/scan` | LiDAR-Rohdaten, unabhängig von Nav2 |
| `/detect/lane_obstacles` (PointCloud2) | die Spurpunkte in Metern, außerhalb des Tunnels. Hier prüft man, ob BEV und Extrinsik stimmen. |
| `/lane/center_path` (Anzeige „Spurmitte“) | die berechnete Spurmitte, das Ziel für MPPI im Obstacle-Modus |
| `/nav2/view` (in `rqt_image_view`) | Draufsicht von `nav2_view`: Costmap, Bot, Pfad, Eingangssperre, LiDAR und Statuszeile mit dem letzten Abbruchgrund. Ohne RViz. Unkomprimiert, weil nav2_view und rqt beide auf dem Laptop laufen (mit `/compressed` stürzte rqt_image_view ab) |

**Der Test, mit dem wir den Fehler gefunden haben:** Eine Wand vor den LiDAR
halten und schauen, ob sie überhaupt in `/local_costmap/costmap` auftaucht.
- **Sie taucht auf, die Spurlinien aber nicht** (außerhalb des Tunnels): Der Fehler
  liegt in der Spurkette (`detect_lane` → `/detect/lane_mask` →
  `lane_costmap_node` → `/detect/lane_obstacles`). Prüfen, ob
  `/detect/lane_obstacles` Punkte enthält und ob `lane_costmap_node` einen
  Auflösungs- oder Ladefehler geloggt hat.
- **Es taucht gar nichts auf:** Der Fehler liegt vor den Layern, bei TF oder
  beim Zeitstempel. Nav2 verwirft eine Beobachtung **ohne Meldung**, wenn der
  TF-Lookup zu ihrem Zeitstempel fehlschlägt. Ursachen sind z. B. eine falsch
  gehende Uhr auf Bot oder Rechner (`scripts/tb/sync-time.sh`) oder ein
  Stempel, der in der Zukunft liegt.

---

## Was nach einer Streckenänderung nachgezogen werden muss

- **Andere Spurbreite oder anderer Hindernisbereich** → nichts. Kollision prüft
  der Umriss, nicht ein auf die Breite abgestimmter Wert (Abschnitt 3).
- **Kamera verstellt** (Neigung, Höhe, Halterung) → Extrinsik neu messen
  (`scripts/tb/camera-extrinsics.py`). Daraus baut `ingress_node` die BEV.
- **`bev.yaml` geändert** (Bereich oder Auflösung) → `ingress_node` und
  `lane_costmap_node` neu starten. Beide lesen die Datei nur beim Start.
- **Andere Höchstgeschwindigkeit** (`vx_max`) oder anderer `goal_distance` →
  `time_steps` nach der Formel in Abschnitt 5 nachrechnen.
- **Enclosure-Öffnung gemessen** → `entrance_wall_width` (Block
  `/keepout_zone_node` in `Lane_Costmap.yaml`) an die echte Breite anpassen.
