# Zonen-Ausweichen (Branch `Zone`)

Hindernisvermeidung ueber eine Zoneneinteilung des 360-Grad-LiDAR, kombiniert
mit der bestehenden Spurverfolgung.

Start: `ros2 launch Follow_Lane zone_avoid.launch.py`
(oder im `tb`-Menue der Launcher `zone_avoid.sh`)

## Signalfluss

```
                                  /avoid/offset
                     .------------------------------.
                     |                              v
detect_lane --/detect/lane--------------------> control_driving --/cmd_lane--.
                     |                                                       |
/scan --> avoid_node -+--/avoid/takeover--> switch_control                   |
          |                                   (Mux)                          |
          '--/cmd_avoid-------------------------+<------------------------- -'
                                                |
/scan --> safety_node --/cmd_safe-------------->|
                                                v
                                        /control/cmd_vel --(remap)--> /cmd_vel
```

Prioritaet im Mux (hoechste zuerst):

| Prio | Topic        | Node            | Wann                                  |
|------|--------------|-----------------|---------------------------------------|
| 1    | `/cmd_safe`  | safety_node     | Hindernis < `emergency_distance`      |
| 2    | `/cmd_avoid` | avoid_node      | Frontzone < `takeover_distance`       |
| 3    | `/cmd_lane`  | control_driving | Normalbetrieb                         |

Der Mux waehlt die hoechste Prioritaet, von der noch **aktuelle** Nachrichten
kommen (`cmd_timeout_*`). safety_node und avoid_node senden deshalb nur, wenn
sie tatsaechlich zustaendig sind -- hoert eine Quelle auf, faellt der Mux von
selbst zurueck. Kommt von gar keiner Quelle etwas, geht ein Nullbefehl raus.

## Zonen

Winkel in Grad gegen den Uhrzeigersinn ab Fahrtrichtung (0 = geradeaus,
90 = links). Konfigurierbar in `Parameter/Avoid_Node.yaml`.

| Zone          | Bereich     | Kosten |
|---------------|-------------|--------|
| Vorne         | 350deg-10deg| 0      |
| Vorne-Links   | 10deg-50deg | +1     |
| Vorne-Rechts  | 310deg-350deg| -1    |
| Links         | 50deg-90deg | +2     |
| Rechts        | 270deg-310deg| -2    |

**Vorzeichen = Drehrichtung** (positiv = links), **Betrag = Kosten**. Gewaehlt
wird der freie Sektor mit dem kleinsten Betrag, bei Gleichstand der mit mehr
Platz. Waere der reine Zahlenwert massgeblich, gewaenne immer der negativste
Sektor und der Roboter wiche grundsaetzlich nach rechts aus.

Ist kein Sektor frei (`clear_distance`), wird der mit der groessten gemessenen
Distanz genommen.

## Zwei Betriebsarten

**Normalbetrieb -- PID-Offset.** avoid_node faehrt nicht selbst, sondern legt
einen Offset auf `/avoid/offset`, den control_driving auf den Spur-Error
addiert:

- Hindernis rechts (`front_right`) -> positiver Offset -> Ausweichen nach links
- Hindernis links (`front_left`) -> negativer Offset -> Ausweichen nach rechts
- Staerke proportional zur Naehe (`offset_trigger_distance` bis
  `offset_saturation_distance`)
- beide Seiten belegt -> die Anteile heben sich teilweise auf, der Roboter
  faehrt mittig durch

Die Spurerkennung laeuft dabei weiter und haelt dagegen, sodass die gelbe Linie
nicht ueberfahren wird.

> **Vorzeichen:** `detect_lane` liefert `lane_error = 1 - 2*lane_center/breite`,
> ein **positiver** Error laesst control_driving nach **links** lenken. Der
> Offset folgt derselben Konvention. Wird das in detect_lane gedreht, genuegt
> `offset_direction: -1.0` in der YAML.

**Uebernahme.** Ist die Frontzone unter `takeover_distance` (0.20 m), reicht der
Offset nicht mehr: avoid_node meldet `/avoid/takeover`, switch_control geht auf
`Obstacle` (womit control_driving verstummt) und avoid_node faehrt selbst ueber
`/cmd_avoid`. Zurueck erst ueber `takeover_release_distance` (0.45 m) und nach
`takeover_release_debounce` Sekunden -- ohne diese Hysterese wuerde der Roboter
an der Schwelle zwischen beiden Betriebsarten flattern.

## Warum switch_control und nicht ein zweiter Node

`switch_control` war schon vorher der einzige Publisher von `/control/mode`, an
dem `nav2_goal_publisher`, `lane_costmap_node`, `keepout_zone_node`,
`tunnel_explorer_node` und `debug_view` haengen. Der Mux sitzt deshalb im selben
Node -- so bleiben Modus und Fahrbefehl konsistent.

Der Mux ist per `cmd_mux_enabled` abschaltbar und **standardmaessig aus**:
`follow_lane.launch.py` und `nav2_obstacle.launch.py` laufen unveraendert
weiter, dort mappt control_driving nach wie vor direkt auf `/cmd_vel`.

Ebenso `obstacle_source`:

- `'scan'` (Default, Nav2-Zweig): switch_control entscheidet selbst per
  LiDAR-Kegel ab `obstacle_trigger_distance` (1.0 m)
- `'avoid'` (Zonen-Zweig): avoid_node entscheidet ueber `/avoid/takeover`

Beides gleichzeitig waere ein Fehler: switch_control haette bei 1.0 m schon auf
`Obstacle` geschaltet und damit den PID stummgeschaltet, waehrend avoid_node
erst ab 0.20 m selbst faehrt -- der Roboter stuende dazwischen.

## Not-Aus (safety_node)

Loest aus, sobald mindestens `min_hits` Messpunkte unter `emergency_distance`
liegen, und haelt ueber `min_stop_duration` und `release_distance`.

> **Sensorgrenze:** der LDS-01 meldet `range_min = 0.12 m`; alles darunter ist
> unbrauchbar und wird verworfen. Bei `emergency_distance: 0.15` bleibt also nur
> ein 3 cm breites Band, in dem der Not-Aus ueberhaupt ausloesen kann -- ein
> Hindernis kann bei voller Fahrt zwischen zwei Scans hindurchrutschen.
> safety_node warnt beim Start, wenn dieses Band unter 5 cm liegt. Fuer
> verlaessliches Ausloesen eher 0.20-0.25 einstellen und `release_distance`
> mit anheben.

`safety_node` (LiDAR) ersetzt **nicht** `safety_stop.py` (Watchdog auf
`/cmd_vel`, schlaegt bei ausbleibenden Fahrbefehlen an) -- die beiden decken
verschiedene Faelle ab. `safety_stop` wird in dieser Launch-Datei wie bisher
nicht gestartet.

## Diagnose

```bash
ros2 topic echo /avoid/zones          # Zonendistanzen + Betriebsart
ros2 topic echo /control/cmd_source   # welche Quelle gerade faehrt
ros2 topic echo /avoid/offset         # aktueller Offset
ros2 topic echo /safety/emergency     # Not-Aus aktiv?
ros2 run rqt_reconfigure rqt_reconfigure   # Parameter live nachziehen
```

## Parameterdateien

| Datei                             | Node            |
|-----------------------------------|-----------------|
| `Parameter/Avoid_Node.yaml`       | avoid_node      |
| `Parameter/Safety_Node.yaml`      | safety_node     |
| `Parameter/Switch_Control_Zone.yaml` | switch_control (Mux an, `obstacle_source: avoid`) |

Flaches Format (`name: wert`) wie die uebrigen Dateien, damit die
Datei-Knoepfe in `rqt_reconfigure` benutzbar bleiben.
