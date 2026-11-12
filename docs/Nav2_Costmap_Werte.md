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

| Modus | fährt | local_ und global_costmap enthalten |
|---|---|---|
| Lane | PID (`control_driving`) | LiDAR + Spurgedächtnis (Nav2 ruht, plant nichts) |
| Obstacle | Nav2: NavFn plant, RPP fährt | LiDAR + Spurgedächtnis + Startsperre |
| Tunnel | Nav2: Explorer + NavFn plant, RPP fährt | LiDAR + Eingangssperre, **keine Kamera** (Reglement) |

Beim Tunnel-Eintritt verwirft `lane_costmap_node` das Spurgedächtnis, sendet eine
leere Wolke und leert 0,3 s später beide Costmaps. Erst die leere Wolke, dann leeren:
ein lane_layer behält die letzte Wolke und trägt sie nach dem Leeren sonst wieder
ein. Die Einfahrt (TunnelEntry) fährt der PID. Warum keiner
der Werte von der Breite der Spur abhängt, steht in Abschnitt 3.

---

## 1. Kartengeometrie

| Wert | local_costmap | global_costmap |
|---|---|---|
| `rolling_window` | `true` | `true` |
| `width` / `height` | 4 / 4 m | 7 / 7 m |
| `resolution` | 0.01 m | 0.01 m |
| Robotergröße | `footprint` (Umriss, 138 × 178 mm, ohne Polster) | `robot_radius` 0.09 m + Polster 0.01 |
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
  `rolling_window: true` ignoriert und stehen deshalb nicht in der Datei.
- Nebenwirkung bei der global_costmap: Was aus dem 7×7-m-Fenster herausrollt,
  ist vergessen. Für die Enclosure reicht das.

### `width` / `height`
- **Was sie bewirken:** Kantenlänge der Karte in Metern. Die Karte reicht in
  jede Richtung die halbe Kantenlänge weit.
- **Woher:** abgeleitet. Zwei Obergrenzen bestimmen den Wert:
  1. **Die Reichweite, die der Sensor aktuell halten kann.** Der LDS-03 misst
     bis 12 m, hat aber nur 0,9° Winkelauflösung. Der Abstand zwischen zwei
     Strahlen ist `r · 0,9° · π/180 = r · 0,0157`. Ab etwa **3,2 m** ist dieser
     Abstand größer als 5 cm. Darüber wird das Freiräumen lückenhaft, und schmale
     Hindernisse fallen zwischen zwei Strahlen durch.
     Eine halbe Kartenbreite weit über ~3,2 m hinaus bringt also nur unsichere
     Zellen. Global: 7 m → 3,5 m halbe Breite, passend zu `raytrace_max_range` 3.5.
  2. **Nur local_costmap: die Vorausschau von Regler und Wächter.** RPP prüft
     bis 1 s voraus (bei 0,12 m/s gut 0,1 m), der Wächter 0,6 s, die Pfadprüfung nutzt
     die global_costmap. 4×4 m (2 m halbe Breite) deckt das mit viel Rand ab und
     zeigt in `nav2_view` das ganze Planziel (1 m).
- **Was kaputtgeht:** Ist die Karte zu klein, ragen Pfad oder Vorausschau über
  den Rand hinaus. Ist sie zu groß, landen unsichere Zellen in der Karte und die
  Updates brauchen mehr Rechenzeit.

### `resolution`
- **Was er bewirkt:** Kantenlänge einer Zelle, 0,01 m in beiden Karten.
- **Woher:** gewählt: Spurlinien (~2 cm), Hindernisränder und der Umriss werden auf
  1 cm genau abgebildet. Die Mindestbreiten in Abschnitt 3 hängen daran.
- **Was kaputtgeht:** Jeder Treffer belegt die ganze Zelle, in die er fällt: Lücken
  werden je Seite um bis zu eine Zelle schmaler. Feiner kostet Rechenzeit: 4 × 4 m
  sind 160 000 Zellen.

### `footprint` (local_costmap)
- **Was er bewirkt:** der Umriss des Burger relativ zu `base_footprint`. RPP
  (`use_collision_detection`) bricht ab, sobald dieser Umriss in der Vorausschau eine belegte
  Zelle berührt. Das gilt für Linien, Hindernisse, Enclosure-Wände und Sperren gleichermaßen.
- **Woher:** ROBOTIS: 138 × 178 mm, der ganze Bot liegt im Kreis r = 105 mm um die
  Radachse. Der Rumpf reicht 3,7 cm nach vorn und 10,1 cm nach hinten, seitlich
  ±6,9 cm, hinten auf den Kreis abgerundet. Die Räder liegen bei ±8,9 cm außen und
  ±3,3 cm vor und hinter der Achse. `footprint_padding` steht auf 0: geprüft wird der echte
  Umriss (Umkreis 0,105 m). Derselbe Umriss steht in `robot_geometry.FOOTPRINT` (Wächter,
  Freifahren, Pfadprüfung, Wand-Hülle); wer ihn ändert, ändert beide. Den Magneten vorn enthält
  er nicht: an ihm gab es nie eine Kollision.
- **Was kaputtgeht:** Ein zu kleiner Umriss lässt den Bot Linien überfahren und
  Wände streifen, ein zu großer macht enge Durchfahrten unpassierbar. Da alle
  Burger laut Reglement identisch sind, muss der Wert nie angepasst werden.

### `robot_radius` (global_costmap)
- **Was er bewirkt:** Die Inflation sperrt um jedes Hindernis einen Saum aus
  `robot_radius` + `footprint_padding`, als 16-Eck gerechnet 0,098 m. Diese Zellen
  (Kosten 253) sind für NavFn Wand. Nur für `planner_server` (NavFn), im Obstacle- und
  im Tunnel-Modus. NavFn plant für einen Punkt ohne Ausrichtung, deshalb bleibt hier
  der Kreis als Ersatz für die halbe Bot-Breite.
- **Woher:** die halbe Breite des Burger (178 mm / 2 = 0,089 m), aufgerundet. Mit 0,10
  bliebe in einer 22-cm-Lücke rechnerisch eine Zellreihe frei, die Rasterung und die
  Streuung des LiDAR schließen sie: NavFn findet keinen Weg (Simulation, `tunnel_gap`).
- **Was kaputtgeht:** Ist der Wert zu klein, plant NavFn Wege zu dicht an den
  Enclosure-Wänden. Jede Wandberührung kostet einen Punkt. Ist eine Lücke für
  den Kreis zu eng, findet NavFn keinen Weg, auch wenn der echte Umriss passen
  würde (Status „kein Weg gefunden“).

---

## 2. Sensorreichweiten der Layer

Die Reichweiten werden ab dem Ursprung der Quelle gemessen. Weil überall
`sensor_frame: ""` steht, ist das der Ursprung des Frames, in dem die Nachricht
ankommt.

| Layer (Karte) | Quelle | Frame der Quelle | obstacle min / max | raytrace min / max | clearing |
|---|---|---|---|---|---|
| `obstacle_layer` (local + global) | `/scan_corrected` | LiDAR | 0.0 / 3.0 m | 0.0 / 3.5 m | `true` |
| `lane_layer` (local + global) | `/detect/lane_obstacles` | `base_footprint` | 0.0 / 2.0 m | 0.0 / 2.0 m | `true` |
| `entrance_layer` (local + global) | `/detect/tunnel_entrance_obstacle`, `/nav2/blocked` | `odom` | 0.0 / 100.0 m | 0.0 / 3.0 m | `false` |

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
- **`inf_is_valid` true:** Ein Strahl ohne Echo (`inf`) räumt bis `raytrace_max_range`
  frei. *Woher:* Ohne ihn bleibt hinter einer Öffnung alles unbekannt, wenn dahinter
  nichts in Reichweite steht, und NavFn plant nicht hinaus (Simulation, `tunnel_wide`:
  ohne bleibt der Bot im Tunnel, mit fährt er hinaus). Wirkt nur, wenn der LDS-03 `inf`
  sendet; eine `0` gilt als ungültig und räumt nichts. *Was kaputtgeht:* Liefert der
  LiDAR an einer Wand kein Echo (schwarze oder spiegelnde Fläche, sehr flacher Winkel),
  räumt der Strahl ihre Zellen bis zum nächsten Treffer frei.
- **Was kaputtgeht:** Ist `obstacle_max_range` zu groß, stehen lückenhafte
  Einträge in der Karte. Ist `raytrace_max_range` kleiner als
  `obstacle_max_range`, bleiben Geisterhindernisse am Rand stehen.

### `lane_layer` (Spurwände)
- Wird in jedem Modus außer Tunnel gefüllt, in beiden Costmaps gleich. Er steht
  in der `plugins`-Liste **vor** dem `inflation_layer` (siehe Abschnitt 3).
- **Quelle ist das Spurgedächtnis** von `lane_costmap_node` (Abschnitt 4): Die
  Wolke enthält jedes Mal **alle** gemerkten Linienzellen im Umkreis von 2 m,
  auch neben und hinter dem Bot, nicht nur das aktuelle Kamerabild.
- **`clearing: true`:** Die Strahlen von `base_footprint` zu jedem Punkt der
  Wolke räumen die Zellen dazwischen frei, danach werden alle Punkte wieder
  eingetragen. Gemerkte Linien bleiben so stehen, vom Gedächtnis vergessene
  verschwinden. Das wirkt nur auf die Zellen des `lane_layer`.
- **`obstacle_max_range` / `raytrace_max_range` 2.0:** Gemessen ab
  `base_footprint`. *Woher:* gewählt, = `MEMORY_RADIUS` des Gedächtnisses. Wer
  einen der beiden Werte ändert, muss den anderen mitziehen.

### `entrance_layer` (Tunnel-Eingangssperre, gesperrte Engstellen)
Zwei Quellen mit denselben Werten: die Eingangs- bzw. Startsperre von `keepout_zone_node` und
die Sperrlinien quer über Engstellen, durch die der Umriss nicht passt oder deren Durchfahrt
gescheitert ist (`/nav2/blocked`: im Tunnel von `tunnel_explorer_node`, in Obstacle von
`nav2_goal_publisher`). Jede Quelle sendet beim Verlassen ihres Modus eine leere Wolke: ein
`ObstacleLayer` trägt sonst die letzte Wolke weiter ein.

- **`clearing: false`:** *Woher:* gewählt, siehe Kommentar in der Datei. Die
  Punkte sind virtuell und haben keinen echten Sensorursprung. Ein Raytrace
  liefe entlang einer willkürlichen Linie im odom-Frame und würde dabei echte
  Hindernisse in diesem Layer mitlöschen. Deshalb wirkt `raytrace_*` hier nicht.
- **`obstacle_max_range` 100.0:** Weil die Wolke im Frame `odom` ankommt, zählt
  diese Reichweite **ab dem odom-Nullpunkt**, nicht ab dem Roboter. *Woher:*
  gewählt, praktisch unbegrenzt. Was außerhalb des mitfahrenden Fensters liegt,
  verwirft die Costmap ohnehin. *Was kaputtgeht:* Mit 3.0 fiele die Sperre ohne
  Meldung weg, sobald der Tunneleingang mehr als 3 m vom odom-Nullpunkt entfernt liegt.

---

## 3. Kollision und Inflation: unabhängig von der Spurbreite

| Wert | local_costmap | global_costmap |
|---|---|---|
| Kollisionsprüfung | RPP (`use_collision_detection`), Wächter, Pfadprüfung: Umriss | – (NavFn: Kreis über die Inflation) |
| `footprint_padding` | 0.0 | 0.01 |
| `inflation_radius` | 0.13 m | 0.25 m |
| `cost_scaling_factor` | 20.0 | 10.0 |

### Das Prinzip

Die Breite des Hindernisbereichs ist vorab nicht bekannt, und die normale Spur
ist mit 20 cm nur knapp breiter als der Burger (17,8 cm). Deshalb hängt kein
Wert von der Korridorbreite ab:

- **Kollision** prüfen RPP, der Wächter (`control_driving`) und die Pfadprüfung
  (`passage_geometry`) mit dem echten Umriss ohne Polster (`footprint`, Abschnitt 1). Eine
  Bewegung ist verboten, wenn der Umriss eine belegte Zelle (Linie, Hindernis, Wand, Sperre)
  berührt, und erlaubt, wenn er dazwischen passt, egal wie breit der Bereich ist. Was er schon
  berührt, zählt beim Wächter und beim Freifahren nicht, sonst käme er nie wieder weg.
- **NavFn** kennt nur einen Kreis: Zellen bis `robot_radius` + Polster (0,0988 m) um jede
  belegte Zelle sind für ihn Wand, danach fällt der Saum der Inflation ab. Er zieht den Weg
  zur Mitte, ist aber kein Kollisionskriterium für den Umriss.

### Warum der `lane_layer` vor dem `inflation_layer` steht

Nur was vor dem `inflation_layer` eingetragen ist, bekommt einen Saum. Stünde der `lane_layer`
dahinter, hätten die Linien in der global_costmap keine Sperrzone, und NavFn plante Wege, auf
denen die Bot-Mitte direkt neben einer Linie liegt.

### `inflation_radius` und `cost_scaling_factor`
- **Was sie bewirken:** Um jede belegte Zelle (LiDAR-Hindernis, Spurlinie,
  Enclosure-Wand, Sperre) zieht der `inflation_layer` einen Kostengradienten.
  - global: 0,25 m (wie HKA), Faktor 10. Nähe ist teuer, mit Abstand wird es schnell billig;
    NavFn bevorzugt die Mitte von Lücken und hält Abstand zu Hindernissen. *Woher:* gewählt;
    mit Faktor 3 war der ganze Saum fast gleich teuer (Kosten 230–247 in jeder Lücke von
    0,21–0,26 m), und Wege klebten an Hindernissen.
  - local: nur zur Anzeige (`nav2_view`); RPP prüft den Umriss direkt gegen belegte Zellen,
    `use_cost_regulated_linear_velocity_scaling` ist aus.
- **Bedingung:** global `inflation_radius` ≥ `robot_radius` + Polster (0,0988 m), sonst warnt
  Nav2 beim Start und der Sperrsaum fehlt.
- **Was kaputtgeht:** NavFn rechnet Kosten 0–252 als 50 + 0,8 · Kosten; frei ist also 50,
  direkt an der Sperrzone 251, höchstens das Fünffache. Ein flacher Saum (kleiner Faktor)
  macht nahe und weite Wege fast gleich teuer, der Weg klebt am Hindernis.

### Mindestbreiten bei 1-cm-Zellen

Gerechnet mit der Rasterung von Nav2 (Umriss über seine Kanten auf Zellen gelegt, Inflation nach
Zellabstand); W = Abstand der Wandflächen, je nach Lage zum Raster eine Zelle mehr.

| Stufe | braucht | Herkunft |
|---|---|---|
| Bot real | 0,178 m gerade | Umriss |
| NavFn | W > 0,19, sicher ab 0,20 m | Pfadzelle ≥ 10 Zellen von jeder Wand (Saum 0,0988) |
| Umriss gerade (RPP, Wächter) | W > 0,18, sicher ab 0,19 m | 18 freie Zellen |
| Umriss 15° / 30° / 45° schräg | 0,19–0,20 / 0,20–0,21 / 0,20–0,21 m | Umriss ohne Polster |
| Drehen auf der Stelle | 0,210 m | Umkreis 0,105 |

NavFn ist damit die engste Stufe: Lücken unter etwa 0,20 m plant er nicht, auch wenn der Bot
ausgerichtet passen würde. Zwischen 0,20 und etwa 0,21 m passt der Bot nur gerade; deshalb
prüft `passage_geometry.check_path` jeden Pfad vor dem Losfahren und lässt den Bot vor einer
solchen Engstelle im Freien ausrichten (Abschnitt 6).

### Auf der normalen 20-cm-Spur

Dort fährt im Normalfall der PID, nicht Nav2. Die Spurwände liegen auf den
Linienmitten, 22 cm auseinander: Für NavFn bleiben drei Zellreihen in der Mitte,
der Umriss braucht 18 der 21 freien Zellen. Neben einem Hindernis ist dort kein Platz; der Bot
bleibt stehen, statt die Spur zu verlassen.

---

## 4. Spurwände (`Lane_Costmap.yaml`, `/lane_costmap_node`)

| Wert | aktuell |
|---|---|
| `PIXEL_STRIDE` | 2 |
| `ROBOT_FRAME` | `base_footprint` |
| Umrechnung Pixel → Meter | aus `Parameter/bev.yaml` |
| `MEMORY_RESOLUTION` / `MEMORY_RADIUS` | 0.01 m / 2.0 m |
| `line_confirm_frames` / `free_confirm_frames` | 3 / 10 |
| `SHADOW_RADIUS` / `SHADOW_MARGIN` | 0.03 m / 0.05 m |
| `line_margin` | 0.0 m |
| `TRACE_STEP` / `TRACE_MAX_TURN_DEG` / `TRACE_MAX_GAP` | 0.03 m / 35° / 0.09 m |
| `TRACE_PARALLEL_DEG` | 60° |

### Spurgedächtnis
Die Kamera sieht nur 5–60 cm voraus. Damit Nav2 auch die Linien neben und hinter
dem Bot kennt, merkt sich `lane_costmap_node` die Linien in einem ortsfesten
Raster im odom-Frame (`MEMORY_RESOLUTION`, = `resolution` der Costmaps):
- Linienpixel sind nur die der gelben und der weißen Spurlinie. Jede Linie wird
  vom bot-nahen Ende aus in Schritten von `TRACE_STEP` verfolgt (Median der Pixel
  voraus, höchstens `TRACE_MAX_TURN_DEG` Drehung je Schritt, Lücken bis
  `TRACE_MAX_GAP`), Weiß rechts von Gelb. Nur Pixel bis `line_margin` um diese
  Mitte zählen; knickt eine Linie gegen die andere um mehr als
  `TRACE_PARALLEL_DEG` ab, endet sie dort. Querlinien, Papier und helle Flächen
  neben der Strecke werden so keine Wand. Pixel am Hindernis und in seinem
  LiDAR-Schatten fallen vorher weg: Die BEV zieht hohe helle Gegenstände lang.
  *Woher:* gewählt.
- Pro Kamerabild bekommt jede **sichtbare** Zelle eine Stimme: „Linie“, wenn ein
  Linienpixel darin liegt, sonst „frei“. Sichtbar heißt: in der Sichtmaske
  `/lokal/bev_view` (die BEV-Ecken liegen außerhalb des Kamerabilds) und nicht
  im Sichtschatten eines LiDAR-Hindernisses. Ein Hindernis vor der Linie
  verdeckt sie nur; ohne diese Prüfung würde die Linie dort nach ein paar Bildern
  gelöscht, genau neben dem Hindernis.
- Eine Zelle wechselt erst, wenn `line_confirm_frames` (Linie) bzw.
  `free_confirm_frames` (frei) Bilder hintereinander das Gegenteil sagen. Linien
  erscheinen nach ~3 Bildern, verschwinden nach ~10. Einzelne Fehldetektionen
  kommen so nicht hinein. *Woher:* gewählt.
- Zellen außerhalb von `MEMORY_RADIUS` bleiben gemerkt, werden aber nicht gesendet.
  Geleert wird das Gedächtnis beim Tunnel-Eintritt (keine Kameradaten im Tunnel) und
  mit Taste L im Not-Aus.
- Gesendet wird jedes Mal das ganze Gedächtnis auf `/detect/lane_obstacles`;
  `switch_control` nutzt es zusätzlich für den Linienfilter.

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

### `PIXEL_STRIDE`
- **Was er bewirkt:** Nur Pixel, deren x und y Vielfache von `PIXEL_STRIDE`
  sind, werden umgerechnet.
- **Woher:** gewählt. Er wirkt zusammen mit `LANE_MASK_STRIDE` (Default 2 in
  `detect_lane`, nicht in einer YAML gesetzt), das schon beim Senden ausdünnt.
  Solange beide 2 sind, dünnt `PIXEL_STRIDE` nichts weiter aus. Bei 1,25 mm/px
  sind das 2,5 mm Abstand, weit unter einer Costmap-Zelle.

### `ROBOT_FRAME`
- `base_footprint` liegt in der TurtleBot3-URDF (burger) am Boden direkt unter
  der Radachse und passt damit zur Konvention der BEV (X ab Radachse). Nicht
  ändern, solange `bev.yaml` ab Radachse rechnet.

### Spurmitte als Richtung für das Planziel

Aus den Linienpunkten des aktuellen Bilds berechnet `lane_costmap_node` die
Spurmitte und sendet sie **nur im Obstacle-Modus** als `nav_msgs/Path` auf
`/lane/center_path`. `nav2_goal_publisher` setzt das Planziel `goal_distance`
voraus auf diese Linie (über ihr Ende hinaus verlängert, Abschnitt 5). So folgt
der Bot auch in Kurven der Spur statt einer Geraden, die irgendwann in die
Spurwand zeigt. Die Spurbreite wird auch im Lane-Modus gemessen, damit sie beim
Umschalten schon bekannt ist.

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
gilt die Breite aus den letzten Bildern (`CENTER_WIDTH_HISTORY`). ROBOTIS
versetzt an dieser Stelle um feste 280 Pixel; hier wird die Breite gemessen,
damit derselbe Code auf der 20-cm-Spur und im breiteren Hindernisbereich
funktioniert.

| Wert | aktuell | Herkunft |
|---|---|---|
| `CENTER_PATH_TOPIC` | `/lane/center_path` | gewählt |
| `ODOM_FRAME` | `odom` | = `global_frame` der Costmaps |
| `CENTER_BIN_SIZE` | 0.05 m | gewählt; Streifentiefe und Abstand der Posen im Pfad |
| `CENTER_MIN_POINTS` | 3 | gewählt; darunter kein Pfad |
| `CENTER_WIDTH_HISTORY` | 20 Bilder | gewählt; Median der gemessenen Breite |

**Offline geprüft** (synthetische Spuren direkt auf das BEV-Raster aus
`bev.yaml` gelegt, Sichtfeld aus der aktuellen Extrinsik ausgeblendet,
`LANE_MASK_STRIDE` 2 wie in `detect_lane`, nicht auf dem Bot). Das prüft die
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
- Die Spurmitte nutzt dieselben gefilterten Linienpixel wie das Gedächtnis,
  Querlinien verfälschen sie also nicht. Sie ist aber ein Polynom Y(X): In engen
  Kurven, die im Sichtbereich fast quer laufen, passt sie schlecht. Das muss man
  vor Ort in RViz prüfen (Anzeige „Aktiver Pfad (Nav2)“ im Obstacle-Modus).

---

## 5. Regler und Vorausschau

| Wert | Datei | aktuell |
|---|---|---|
| `controller_frequency` | nav2_params | 20.0 Hz (= `WATCHDOG_RATE_HZ` in `control_driving`) |
| `desired_linear_vel` | nav2_params (`FollowPath`) | 0.12 m/s, im Tunnel 0.10 |
| `lookahead_dist` / `min_` / `max_` | nav2_params | 0.20 / 0.15 / 0.35 m |
| `use_collision_detection` | nav2_params | true |
| `max_allowed_time_to_collision_up_to_carrot` | nav2_params | 1.0 s |
| `rotate_to_heading_min_angle` / `_angular_vel` | nav2_params | 0.785 rad / 1.2 rad/s |
| `failure_tolerance` | nav2_params (`controller_server`) | 0.3 s |
| `goal_distance` | Lane_Costmap (`/nav2_goal_publisher`) | 1.0 m (Abstand des Planziels) |
| `CENTER_PATH_MAX_AGE_S` | `nav2_goal_publisher.py` | 1.0 s |

### Einzelwerte
- **RPP** fährt den NavFn-Pfad ab: Er zielt auf einen Punkt `lookahead_dist` voraus (mit der
  Geschwindigkeit zwischen min und max), fährt in engen Kurven langsamer
  (`regulated_linear_scaling_min_radius` 0,9 m, bis 0,05 m/s) und dreht auf der Stelle, wenn der
  Pfad mehr als `rotate_to_heading_min_angle` abweicht. Er weicht nicht aus. *Woher:* gewählt,
  Test 12.11.: kam durch Lücken, an denen MPPI scheiterte. `allow_reversing` geht nicht
  zusammen mit `use_rotate_to_heading`.
- **`use_collision_detection` / `max_allowed_time_to_collision_up_to_carrot`:** Vor jedem
  Befehl rechnet RPP die Bewegung bis 1 s voraus (höchstens bis zum Zielpunkt) und prüft den
  Umriss gegen belegte Zellen der local_costmap, beim Drehen auf der Stelle die Drehwinkel.
  Bei Gefahr meldet er „detected collision ahead“.
- **`failure_tolerance`:** So lange sendet `controller_server` bei dieser Meldung Null, erst
  dann bricht er das Ziel ab. Eine einzelne flackernde Zelle beendet die Fahrt nicht.
- **`lookahead_dist`:** Kürzer schneidet weniger Kurven, länger fährt ruhiger. Im Tunnel
  bremst `desired_linear_vel` 0,10 (`TUNNEL_CONTROLLER`).
- **Ziel im Obstacle-Modus:** ein Punkt `goal_distance` (1,0 m) voraus auf der
  Spurmitte (Abschnitt 4), über ihr Ende (~0,6 m) hinaus in ihrer Richtung
  verlängert, solange sie jünger ist als `CENTER_PATH_MAX_AGE_S` (Empfangszeit,
  damit die Uhren von Bot und Rechner nicht synchron sein müssen). Sonst liegt
  das Ziel geradeaus. `planner_server` (NavFn) rechnet auf der global_costmap
  einen Weg dorthin, RPP fährt ihn ab. Neu geplant wird bei einer gesperrten Zelle auf dem
  Pfad, wenn er bis auf `goal_distance`/2 abgefahren ist, wenn die Fahrt endet, und jede
  Sekunde (`REPLAN_PERIOD_S`); der Takt-Pfad gilt nur, wenn er 0,3 m voraus höchstens 45°
  anders zeigt (`passage_geometry.same_direction`), sonst kippte der Weg links/rechts um ein
  Hindernis. Nach dem Umschalten wartet die erste Planung bis zu 0,5 s auf die erste
  Spurmitte. *Woher:* gewählt.

---

## 6. Wächter, Pfadprüfung, Freifahren

Alle drei nutzen denselben Umriss ohne Polster (`robot_geometry.FOOTPRINT`) und prüfen gegen
belegte Zellen; Konstanten im Code, keine Parameter.

| Wert | Datei | aktuell |
|---|---|---|
| `GUARD_HORIZON_S` | `control_driving.py` | 0.6 s (wie HKA) |
| `GUARD_MAP_MAX_AGE_S` | `control_driving.py` | 1.0 s |
| `GUARD_MANEUVER_FAIL_S` / `GUARD_ABORT_S` | `control_driving.py` / `nav2_goal_publisher.py` | 1.0 s |
| `CHECK_AHEAD_M` / `CHECK_SKIP_M` | `passage_geometry.py` | 1.0 m / 0.105 m |
| `ROTATE_CLEAR_M` | `passage_geometry.py` | 0.11 m |
| `UNWEDGE_MOVES` | `robot_geometry.py` | 5 cm vor/zurück, 15° links/rechts |

- **Wächter** (`control_driving`, Modi Obstacle, Tunnel, Align): jeder Befehl wird
  `GUARD_HORIZON_S` mit v und ω vorausgerechnet und der Umriss gegen die local_costmap gelegt.
  Stößt er neu an, geht Null hinaus. Ist die Karte älter als `GUARD_MAP_MAX_AGE_S`, ebenfalls
  Null. Hält er Nav2 länger als `GUARD_ABORT_S` an, bricht `nav2_goal_publisher` das Ziel ab
  (wie Kollision); eine Grundbewegung gilt nach `GUARD_MANEUVER_FAIL_S` als gescheitert.
- **Pfadprüfung** (`passage_geometry.check_path`, Obstacle und Tunnel): der Umriss in
  Pfadrichtung (gemittelt über ±5 cm) entlang des neuen Pfads, ab `CHECK_SKIP_M` vom Bot (dort
  entscheidet RPP selbst) bis `CHECK_AHEAD_M`. An der ersten Engstelle, an der er anstößt: die
  Gerade mit dem meisten Abstand (±30°, ±3 cm), Start mit `ROTATE_CLEAR_M` Platz zum Drehen.
  Luft auf der Geraden → ausrichten und durchfahren, keine Luft → sperren (`/nav2/blocked`).
- **Freifahren** (`control_driving`, auf Anfrage nach Kollisionsgefahr): die Bewegung aus
  `UNWEDGE_MOVES`, nach der der Umriss am weitesten von allem Belegten weg ist und die
  unterwegs nirgends neu anstößt. Gibt es keine, bleibt der Bot stehen und meldet das.
- **Was kaputtgeht:** Zu großer Horizont oder ein Polster lassen den Wächter in Engstellen
  anhalten, durch die der Bot passt. Was der LiDAR nie gesehen hat (unter 10 cm am Bot),
  kennt keine der Prüfungen.

---

## 7. Zeitverhalten

| Wert | Wo | aktuell | Herkunft |
|---|---|---|---|
| `update_frequency` | local_costmap | 10.0 Hz | LiDAR-Rate |
| `publish_frequency` | local_costmap | 5.0 Hz | nicht dokumentiert |
| `update_frequency` | global_costmap | 5.0 Hz | gewählt (12.11.: mit 2 Hz kamen Hindernisse bis ~1 s spät) |
| `publish_frequency` | global_costmap | 2.0 Hz | gewählt |
| `required_movement_radius` | progress_checker | 0.1 m (Tunnel 0.05 m) | gewählt |
| `movement_time_allowance` | progress_checker | 3.0 s (Tunnel 6.0 s) | gewählt |
| `FollowPath.desired_linear_vel` | im Tunnel | 0.10 m/s | gewählt |
| `xy_goal_tolerance` | goal_checker | 0.15 m | nicht dokumentiert |
| `yaw_goal_tolerance` | goal_checker | 3.15 rad | gewählt |

- **`update_frequency`:** Wie oft die Layer neu in die Karte geschrieben werden.
  Lokal 10 Hz entspricht der LiDAR-Rate des LDS-03. *Was kaputtgeht:* Ist der
  Wert zu niedrig, reagieren RPP und Wächter auf veraltete Hindernisse. Ist er zu
  hoch, kostet das CPU ohne neue Daten.
- **`publish_frequency`:** Wie oft die Karte auf dem Topic erscheint (RViz, `nav2_view`,
  `tunnel_explorer_node`, `nav2_goal_publisher`, und lokal der Wächter in `control_driving`).
  RPP liest die Karte direkt, der Wächter das Topic: lokal 5 Hz, also bis 0,2 s alt.
- **`movement_time_allowance` / `required_movement_radius`:** Bewegt sich der
  Bot 3 s lang nicht um mindestens 0,1 m, bricht der Regler das Ziel ab, und
  `nav2_goal_publisher` plant neu. Im Tunnel stellt `tunnel_explorer_node` per
  `SetParameters` auf 0,05 m in 6 s und `desired_linear_vel` 0,10 m/s um
  (`TUNNEL_CONTROLLER`), beim Verlassen zurück. *Was kaputtgeht:* zu lang, und RPP drückt lange
  gegen ein Hindernis; zu kurz im Tunnel, und jedes Kriechen zwischen engen Wänden bricht ab.
- **`xy_goal_tolerance`:** Ab dieser Entfernung gilt das Ziel als erreicht, und
  das nächste wird gesetzt. Damit endet jedes Segment bei ~0,15 m; den Rest bis zum Start einer
  Engstelle fährt die Grundbewegung.
- **`yaw_goal_tolerance` 3.15:** Größer als π, also wird jede Endausrichtung
  akzeptiert: das Ziel rollt mit.

---

## Prüfen in RViz

| Display | zeigt |
|---|---|
| `/local_costmap/costmap` | alle Layer der local_costmap zusammen, dazu die Inflation. Außerhalb des Tunnels auch die Spurwände. |
| `/global_costmap/costmap` | Karte des Bahnplaners, 7×7 m um den Bot: außerhalb des Tunnels LiDAR + Spurgedächtnis, im Tunnel LiDAR + Eingangssperre |
| `/scan` | LiDAR-Rohdaten, unabhängig von Nav2; Nav2 und alle Nodes lesen `/scan_corrected` |
| `/detect/lane_obstacles` (Anzeige „Spurgedächtnis“) | alle gemerkten Linienzellen in Metern, außerhalb des Tunnels. Hier prüft man, ob BEV und Extrinsik stimmen und ob die Linien beim Ausweichen stehen bleiben. |
| `/nav2/active_path` (Anzeige „Aktiver Pfad (Nav2)“) | der Weg, dem Nav2 gerade folgt; nur im Obstacle- und Tunnel-Modus während einer Fahrt |
| `/plan` | was `planner_server` zuletzt gerechnet hat |
| `/nav2/view` (in `rqt_image_view`) | Draufsicht von `nav2_view`: Costmap, Bot, aktiver Pfad, Eingangssperre, LiDAR und Statuszeile mit dem letzten Abbruchgrund. Ohne RViz. Unkomprimiert, weil rqt_image_view das Topic teils als Image, teils als CompressedImage abonniert und mit `/compressed` abbricht |

**Test, wenn Hindernisse fehlen:** Eine Wand vor den LiDAR halten und schauen, ob sie
überhaupt in `/local_costmap/costmap` auftaucht.
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
- **LiDAR getauscht oder neu montiert** → Entfernungsfehler neu messen
  (`Scan_Correct.yaml`, `range_offset_m`).
