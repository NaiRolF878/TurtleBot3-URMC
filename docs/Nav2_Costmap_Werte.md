# Nav2-Costmap-Werte: Herkunft und Abhängigkeiten

Für alle, die einen der Werte in `Parameter/nav2_params.yaml` oder
`Parameter/Lane_Costmap.yaml` ändern müssen. Zu jedem Wert steht hier, was er
bewirkt, woher er kommt und was kaputtgeht, wenn er falsch steht. Viele Werte
hängen voneinander ab; diese Abhängigkeiten sind als nachrechenbare Regeln
angegeben.

Pfade sind relativ zu `workspace/src/Follow_Lane/Follow_Lane/`.

**Herkunft** heißt:
- *gemessen*: am Bot oder an der Strecke vermessen
- *abgeleitet*: aus anderen Werten berechnet, die Rechnung steht dabei
- *gewählt*: eine Entscheidung, die man begründet ändern darf
- *nicht dokumentiert*: die Dateien enthalten keine Begründung, nichts geraten

---

## 1. Kartengeometrie

| Wert | local_costmap | global_costmap |
|---|---|---|
| `rolling_window` | `true` | `true` |
| `width` / `height` | 4 / 4 m | 7 / 7 m |
| `resolution` | 0.05 m | 0.05 m |
| `robot_radius` | 0.105 m | 0.105 m |
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
  ist vergessen. Für die Länge eines einzelnen Tunnels reicht das.

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
     die Vorausschau ab und lässt `prune_distance` 1.7 genug Platz. Laut Kommentar
     in der Datei sah der Regler bei 3×3 m zu wenig.
- **Was kaputtgeht:** Ist die Karte zu klein, ragen Pfad oder Vorausschau über
  den Rand hinaus. Ist sie zu groß, landen unsichere Zellen in der Karte und die
  Updates brauchen mehr Rechenzeit. Bei `prune_distance` ≥ halbe Breite der
  local_costmap reicht der Pfad über die Karte hinaus.

### `resolution`
- **Was er bewirkt:** Kantenlänge einer Zelle, 0,05 m.
- **Woher:** gewählt. Die Dateien begründen den Wert nicht.
- **Was kaputtgeht:** Viele andere Werte hängen an ihm: die 3,2-m-Grenze oben,
  die Quantisierung des `inflation_radius` (Abschnitt 3) und die Breite des
  kostenfreien Kanals. Wer `resolution` ändert, muss diese Werte neu rechnen.

### `robot_radius`
- **Was er bewirkt:** Zellen, die näher als dieser Radius an einem Hindernis
  oder einer Spurwand liegen, gelten für den Mittelpunkt des Roboters als
  Kollision.
- **Woher:** laut Kopfkommentar der Datei „an die Burger-Spezifikation
  angelehnt“. Eine Messung ist nicht dokumentiert. Statt eines Footprint-Polygons
  wird ein Radius verwendet, deshalb steht `consider_footprint: false` beim
  `CostCritic`.
- **Was kaputtgeht:** Ein zu großer Wert macht die Spur unbefahrbar (Abschnitt 3).
  Ein zu kleiner Wert lässt den Bot Hindernisse streifen.

---

## 2. Sensorreichweiten der Layer

Die Reichweiten werden ab dem Ursprung der Quelle gemessen. Weil überall
`sensor_frame: ""` steht, ist das der Ursprung des Frames, in dem die Nachricht
ankommt.

| Layer (Karte) | Quelle | Frame der Quelle | obstacle min / max | raytrace min / max | clearing |
|---|---|---|---|---|---|
| `obstacle_layer` (local + global) | `/scan` | LiDAR | 0.0 / 3.0 m | 0.0 / 3.5 m | `true` |
| `lane_layer` (nur local) | `/detect/lane_obstacles` | `base_footprint` | 0.0 / 2.0 m | 0.0 / 2.0 m | `true` |
| `entrance_layer` (local + global) | `/detect/tunnel_entrance_obstacle` | `odom` | 0.0 / 3.0 m | 0.0 / 3.0 m | `false` |

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
- **`obstacle_max_range` / `raytrace_max_range` 2.0:** Gemessen ab
  `base_footprint`. *Woher:* nicht dokumentiert. Faktisch **begrenzen sie
  nichts**: `lane_costmap_node` schickt nur Punkte bis `ground_x_max` (0.80 m)
  voraus, seitlich sieht die Kamera dort etwa ±0,5 m (zusammen < 1 m Abstand).
- **`clearing: true`:** Die Strahlen von `base_footprint` zu den aktuellen
  Linienpunkten löschen alte Spurpunkte, die dazwischen liegen. Das wirkt nur
  auf die Zellen des `lane_layer`, LiDAR-Hindernisse bleiben stehen (jeder
  Layer hat ein eigenes Gitter). Spurpunkte außerhalb des Kamerabilds bleiben
  erhalten. Das ist das Gedächtnis, das den Korridor neben und hinter dem Bot
  hält.

### `entrance_layer` (Tunnel-Eingangssperre)
- **`clearing: false`:** *Woher:* gewählt, siehe Kommentar in der Datei. Die
  Punkte sind virtuell und haben keinen echten Sensorursprung. Ein Raytrace
  liefe entlang einer willkürlichen Linie im odom-Frame und würde dabei echte
  Hindernisse in diesem Layer mitlöschen.
- **`obstacle_max_range` 3.0:** Weil die Wolke im Frame `odom` ankommt, wird
  diese Reichweite **ab dem odom-Nullpunkt** gemessen, nicht ab dem Roboter.
  **Was kaputtgehen kann:** Liegt der Tunneleingang mehr als 3 m vom
  odom-Nullpunkt entfernt, verwirft Nav2 die Sperrpunkte ohne Meldung. Mit
  `rolling_window` entfällt zwar der Odometrie-Reset, diese Grenze bleibt aber
  bestehen. Auf der echten Strecke ist das noch nicht geprüft. Das hier ist
  aus der Konfiguration abgeleitet und nicht beobachtet.

---

## 3. Inflation und Korridor

Das ist der wichtigste Abschnitt. Der `inflation_layer` zieht um jede belegte
Zelle einen Kostengradienten, und zwar um **Hindernisse UND Spurwände**. In der
local_costmap sind beide Spurlinien Wände, der Korridor dazwischen ist die
Fahrbahn.

| Wert | local_costmap | global_costmap |
|---|---|---|
| `inflation_radius` | 0.20 m | 0.3 m |
| `cost_scaling_factor` | 3.0 | 3.0 |

### Die Regel

```
inflation_radius  <  Korridorbreite / 2
befahrbare Breite =  Korridorbreite - 2 * robot_radius
kostenfreier Kanal = Korridorbreite - 2 * inflation_radius
```

`Korridorbreite` ist der Abstand zwischen den **Innenkanten** der beiden
Spurlinien, denn die Linienpixel selbst werden zu Wänden.

**Begründung:** Ist der Radius größer als die halbe Korridorbreite, überlappen
sich die Inflationsbereiche der linken und der rechten Spurlinie vollständig.
Dann bleibt keine Zelle mit Kosten 0 übrig, und der Regler wird auf der ganzen
Spur bestraft. Zusätzlich gelten Zellen näher als `robot_radius` an einer Wand
als Kollision. Befahrbar ohne Kollision bleibt also nur
`Korridorbreite - 2 * 0.105`.

**Aktueller Stand:** 0.20 m Radius bei angenommenen 0,45 m Spurbreite ergeben
einen kostenfreien Kanal von 0,45 − 0,40 = **0,05 m** und einen
kollisionsfreien Kanal von 0,45 − 0,21 = **0,24 m**. Die 0,45 m sind **nicht
gemessen**. Sobald die echte Spurbreite bekannt ist, die Tabelle unten benutzen.

**Quantisierung:** Nav2 rundet den Inflationsradius auf ganze Zellen auf.
Werte, die kein Vielfaches von `resolution` (0.05) sind, wirken wie das nächste
größere Vielfache. Die Tabelle nimmt deshalb das größte Vielfache von 0,05,
das echt kleiner als die halbe Korridorbreite ist.

### Tabelle: welcher `inflation_radius` zu welcher gemessenen Spurbreite passt

| gemessene Spurbreite | Obergrenze (b/2) | `inflation_radius` | kostenfreier Kanal | kollisionsfrei (b − 0,21) |
|---|---|---|---|---|
| 0,40 m | 0,20 m | **0.15** | 0,10 m | 0,19 m |
| 0,45 m | 0,225 m | **0.20** (aktuell) | 0,05 m | 0,24 m |
| 0,50 m | 0,25 m | **0.20** | 0,10 m | 0,29 m |
| 0,60 m | 0,30 m | **0.25** | 0,10 m | 0,39 m |
| 0,70 m | 0,35 m | **0.30** | 0,10 m | 0,49 m |

Unter etwa 0,30 m Spurbreite bleibt kein sinnvoller Gradient mehr (Radius ≤
`robot_radius`). Dann ist die Spur für Nav2 praktisch nicht befahrbar.

**Was kaputtgeht:** Mit dem früheren Wert 0.45 m gab es auf der ganzen Spur
keine kostenfreie Zelle, und der Regler wurde überall bestraft. Ist der Radius
zu klein, wird der Kostengradient sehr kurz und der Regler weicht erst im
letzten Moment aus.

**global_costmap ist davon nicht betroffen:** Dort gibt es keinen `lane_layer`
und damit keinen Korridor. 0.3 m bleibt.

---

## 4. Spurwände (`Lane_Costmap.yaml`, `/lane_costmap_node`)

| Wert | aktuell |
|---|---|
| `ground_homography_file` | `ground_homography_800x600.yaml` |
| `ground_x_min` | 0.15 m |
| `ground_x_max` | 0.80 m |
| `pixel_stride` | 2 |
| `frame_id` | `base_footprint` |

Die Pixel aus dem entzerrten Kamerabild werden über die Homographie
`H_image_to_ground` in Bodenkoordinaten umgerechnet: X vorwärts ab Radachse,
Y nach links, in Metern. Das entspricht genau `base_footprint`. Die Homographie
liegt in `Parameter/camera_info/ground_homography_800x600.yaml` und wird mit
`scripts/tb/ground-calib.py` neu erzeugt.

**Dokumentierte Genauigkeit** (aus der YAML):

| | RMS | max |
|---|---|---|
| Fit-Restfehler (`fit_rms_mm` / `fit_max_mm`) | 2,99 mm | 7,72 mm |
| Kreuzvalidierung (`crossval_rms_mm` / `crossval_max_mm`) | **4,11 mm** | **9,66 mm** |

Der Kreuzvalidierungswert ist der ehrliche, der Fit-Restfehler ist immer
optimistisch. Beide gelten nur im Bereich der ausgelegten Marker: X von 0,20
bis 0,70 m, seitlich bis ±0,20 m.

### `ground_homography_file`
- **Was er bewirkt:** Dateiname in `Parameter/camera_info/`. Wird nur beim
  Start des Nodes gelesen.
- **Woher:** gemessen (Fit an abgemessene Bodenpunkte).
- **Was kaputtgeht:** Die Datei gilt nur für das **entzerrte** Bild
  (`ingress_node` mit `undistort:=true`) und nur für genau die eingetragene
  Auflösung (`image_width`/`image_height`: 800×600). Weicht die Auflösung der
  `LaneMask` ab, loggt der Node einmal einen Fehler und sendet keine Spurwände
  mehr. Fehlt die Datei, sendet er ebenfalls nichts. Bei abgeschalteter
  Entzerrung sind die Meter falsch, ohne dass es eine Warnung gibt.

### `ground_x_min`
- **Was er bewirkt:** Punkte näher als 0,15 m vor der Radachse werden verworfen.
- **Woher:** gewählt. Sehr nah wird die Homographie unzuverlässig, und der
  nächste Marker lag bei 0,20 m.
- **Hinweis:** Mit der aktuellen Homographie entspricht die unterste Bildzeile
  etwa X = 0,19 m. Der Wert greift im Moment also praktisch nie. Nach einer
  Neukalibrierung erneut prüfen.

### `ground_x_max`
- **Was er bewirkt:** Punkte weiter als 0,80 m voraus werden verworfen.
- **Woher:** abgeleitet. Die Marker reichten beim Fit nur bis 0,70 m, darüber
  wird extrapoliert und die Fehler wachsen stark. **`ground_x_max` darf nicht
  weit über den am weitesten entfernten Marker hinausgehen.** Wer weiter
  hinaus will, legt beim nächsten `ground-calib.py`-Lauf Marker weiter
  draußen aus.
- **Was kaputtgeht:** Ist der Wert zu groß, stehen ferne Spurpunkte an falschen
  Stellen in der Karte und verengen oder verschieben den Korridor.

### `pixel_stride`
- **Was er bewirkt:** Nur Pixel, deren x und y Vielfache von `pixel_stride`
  sind, werden umgerechnet.
- **Woher:** gewählt. Er wirkt zusammen mit `lane_mask_stride` (Default 2 in
  `detect_lane`, nicht in einer YAML gesetzt), das schon beim Senden ausdünnt.
  Solange beide 2 sind, dünnt `pixel_stride` nichts weiter aus.
- **Was kaputtgeht:** Ist der Wert zu groß, entstehen Lücken in den Spurwänden,
  durch die MPPI planen kann. Ist er zu klein, steigt die Rechenzeit.

### `frame_id`
- `base_footprint` liegt in der TurtleBot3-URDF (burger) am Boden direkt unter
  der Radachse und passt damit zur Konvention der Homographie. Nicht ändern,
  solange die Homographie ab Radachse gemessen ist.

---

## 5. Regler und Vorausschau

| Wert | Datei | aktuell |
|---|---|---|
| `controller_frequency` | nav2_params | 20.0 Hz |
| `model_dt` | nav2_params | 0.05 s |
| `time_steps` | nav2_params | 90 |
| `vx_max` | nav2_params | 0.22 m/s |
| `prune_distance` | nav2_params | 1.7 m |
| `goal_distance` | Lane_Costmap (`/nav2_goal_publisher`) | 1.0 m |

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
- **`vx_max`:** *Woher:* laut Kopfkommentar die Burger-Spezifikation (0,22 m/s).
  *Was kaputtgeht:* Wer ihn senkt, verkürzt die Vorausschau und muss
  `time_steps` nachziehen.
- **`prune_distance`:** Wie weit voraus MPPI den Pfad berücksichtigt. *Woher:*
  gewählt, durch die Kartengröße begrenzt.
- **`goal_distance`:** So weit voraus setzt `nav2_goal_publisher` im
  Obstacle-Modus das Ziel. Ein neues Ziel kommt erst, wenn das alte erreicht
  (`xy_goal_tolerance`) oder abgebrochen ist. Die Distanz zum Ziel läuft also
  von 1,0 m auf ~0,15 m herunter. *Woher:* gewählt.

---

## 6. Kritiken (MPPI `FollowPath`)

Die Schwelle `threshold_to_consider` wirkt je nach Kritik in entgegengesetzter
Richtung:
- **Pfad-Kritiken** (`PathFollowCritic`, `PathAlignCritic`, `PathAngleCritic`,
  `PreferForwardCritic`) greifen nur, wenn das Ziel **weiter weg** ist als ihr
  Schwellwert.
- **Ziel-Kritiken** (`GoalCritic`, `GoalAngleCritic`) greifen nur, wenn das Ziel
  **näher** ist als ihr Schwellwert.

Bei Zieldistanzen zwischen 1,0 und 0,15 m (Abschnitt 5) ergibt das:

| Kritik | enabled | Gewicht | `threshold_to_consider` | greift bei uns |
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
- **`PathFollowCritic` Schwelle 0.5:** Vorher stand 1.4 m. Bei 1,0 m Pfadlänge
  griff die Kritik damit nie, ausgerechnet die, die den Vorwärtsdrang halten
  soll.
- **`GoalAngleCritic` abgeschaltet:** Jeder rollierende Pfad endet mit
  `orientation.w = 1.0`, also „geradeaus, gemessen im Moment der
  Pfaderzeugung“. Ab 0,5 m vor dem Ziel zog die Kritik den Bot in genau diese
  Ausgangsrichtung zurück, während er am Hindernis vorbei sollte. Der
  `goal_checker` lässt das Ziel mit `yaw_goal_tolerance` 3.15 ohnehin
  durchgehen, die Endausrichtung spielt keine Rolle.
- **`PathAlignCritic` niedrig gewichtet (2.0):** Unser Pfad ist eine Gerade
  *durch* das Hindernis, kein geplanter Pfad darum herum. Mit 14.0 (Nav2-Beispiel)
  hat MPPI das Hindernis vor sich hergeschoben statt auszuweichen (Test 02.09.).
  „Auf der Linie bleiben“ ist deshalb nur ein Hinweis. Zusätzlich schaltet
  `max_path_occupancy_ratio` 0.05 die Kritik ab, sobald mehr als 5 % der
  Pfadpunkte in belegten Zellen liegen.

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
| `/local_costmap/costmap` | alle Layer der local_costmap zusammen: LiDAR-Hindernisse, Spurwände, Eingangssperre, dazu die Inflation. Hier sieht man, ob zwischen den Spurlinien noch ein kostenfreier Kanal bleibt (Abschnitt 3). |
| `/global_costmap/costmap` | die Tunnelkarte (LiDAR + Eingangssperre, keine Spur), 7×7 m um den Bot |
| `/scan` | LiDAR-Rohdaten, unabhängig von Nav2 |
| `/detect/lane_obstacles` (PointCloud2) | die Spurpunkte in Metern, bevor sie in die Costmap gehen |

**Der Test, mit dem wir den Fehler gefunden haben:** Eine Wand vor den LiDAR
halten und schauen, ob sie überhaupt in `/local_costmap/costmap` auftaucht.
- **Sie taucht auf, die Spurlinien aber nicht:** Der Fehler liegt in der
  Spurkette (`detect_lane` → `/detect/lane_mask` → `lane_costmap_node` →
  `/detect/lane_obstacles`). Prüfen, ob `/detect/lane_obstacles` Punkte enthält
  und ob `lane_costmap_node` einen Auflösungs- oder Ladefehler geloggt hat.
- **Es taucht gar nichts auf:** Der Fehler liegt vor den Layern, bei TF oder
  beim Zeitstempel. Nav2 verwirft eine Beobachtung **ohne Meldung**, wenn der
  TF-Lookup zu ihrem Zeitstempel fehlschlägt. Ursachen sind z. B. eine falsch
  gehende Uhr auf Bot oder Rechner (`scripts/tb/sync-time.sh`) oder ein
  Stempel, der in der Zukunft liegt.

---

## Was nach einer Streckenänderung nachgezogen werden muss

- **Spurbreite neu gemessen** → `local_costmap.inflation_layer.inflation_radius`
  aus der Tabelle in Abschnitt 3 nehmen.
- **Kamera verstellt** (Neigung, Höhe, Halterung) → Homographie mit
  `scripts/tb/ground-calib.py` neu erzeugen. Danach `ground_x_min` (unterste
  Bildzeile) und `ground_x_max` (weitester Marker) prüfen.
- **Andere Bildauflösung** → Die Homographie ist ungültig. `lane_costmap_node`
  verweigert dann den Dienst (Fehler im Log, keine Spurwände), bis eine
  Homographie für die neue Auflösung vorliegt.
- **Andere Höchstgeschwindigkeit** (`vx_max`) oder anderer `goal_distance` →
  `time_steps` nach der Formel in Abschnitt 5 nachrechnen.
