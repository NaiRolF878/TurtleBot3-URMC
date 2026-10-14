# Zonen-Ausweichen (Branch `Zone`)

Spurverfolgung mit zonenbasierter Hindernisvermeidung.

Der Branch ist bewusst auf das reduziert, was zum Fahren gebraucht wird: der
komplette Nav2-Zweig (Costmaps, MPPI, Planner, Tunnel-Exploration) ist hier
entfernt. Wer ihn braucht, findet ihn unveraendert auf `Nav2LocalCostmap`.

Start: `ros2 launch Follow_Lane zone_avoid.launch.py`
(oder im `tb`-Menue der Launcher `zone_avoid.sh`)

## Nodes

| Node              | Aufgabe                                              |
|-------------------|------------------------------------------------------|
| `ingress_node`    | Kamerabild entzerren, `/scan` spiegeln                |
| `detect_lane`     | Spurerkennung (HSV + CLAHE + Perspektivtransformation)|
| `control_driving` | PID auf den Spur-Error, + Ausweich-Offset             |
| `avoid_node`      | Zoneneinteilung, Offset bzw. komplettes Ausweichen    |
| `switch_control`  | Fahrmodus + Prioritaets-Mux                           |
| `debug_view`      | Debugbild auf `/lokal/debug_view`                     |

## Signalfluss

```
                                  /avoid/offset
                     .------------------------------.
                     |                              v
detect_lane --/detect/lane--------------------> control_driving --/cmd_lane--.
                     |                                                       |
/scan --> avoid_node -+--/avoid/takeover--> switch_control                   |
          |                                   (Mux)                          |
          '--/cmd_avoid-------------------------+<-------------------------- '
                                                v
                                        /control/cmd_vel --(remap)--> /cmd_vel
```

Prioritaet im Mux (hoechste zuerst):

| Prio | Topic        | Node            | Wann                             |
|------|--------------|-----------------|----------------------------------|
| 1    | `/cmd_avoid` | avoid_node      | Frontzone < `takeover_distance`  |
| 2    | `/cmd_lane`  | control_driving | Normalbetrieb                    |

Der Mux waehlt die hoechste Prioritaet, von der noch **aktuelle** Nachrichten
kommen (`cmd_timeout_*`). avoid_node sendet deshalb nur, wenn er tatsaechlich
zustaendig ist -- hoert eine Quelle auf, faellt der Mux von selbst zurueck.
Kommt von gar keiner Quelle etwas, geht ein Nullbefehl raus statt des letzten
Fahrbefehls. Damit ist auch der Fall "control_driving abgestuerzt" abgedeckt.

## Zonen

Winkel in Grad gegen den Uhrzeigersinn ab Fahrtrichtung (0 = geradeaus,
90 = links). Konfigurierbar in `Parameter/Avoid_Node.yaml`.

| Zone          | Bereich       | Kosten |
|---------------|---------------|--------|
| Vorne         | 350deg-10deg  | 0      |
| Vorne-Links   | 10deg-50deg   | +1     |
| Vorne-Rechts  | 310deg-350deg | -1     |
| Links         | 50deg-90deg   | +2     |
| Rechts        | 270deg-310deg | -2     |

**Vorzeichen = Drehrichtung** (positiv = links), **Betrag = Kosten**. Gewaehlt
wird der freie Sektor mit dem kleinsten Betrag, bei Gleichstand der mit mehr
Platz. Waere der reine Zahlenwert massgeblich, gewaenne immer der negativste
Sektor und der Roboter wiche grundsaetzlich nach rechts aus.

Ist kein Sektor frei (`clear_distance`), wird der mit der groessten gemessenen
Distanz genommen. Eine Zone ganz ohne Echo zaehlt als frei -- `inf` heisst beim
LiDAR "nichts in Reichweite", nicht "kein Messwert".

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

Der Offset wird auch waehrend der Uebernahme weiter gesendet. Ihn dort auf 0 zu
zwingen waere ein Sprung im Sollwert: zwischen `/avoid/takeover` und dem
`/control/mode`-Wechsel liegen ein bis zwei Nachrichten, und in diesem Fenster
macht der D-Anteil (`kd`/0.1) daraus sofort einen Lenkausschlag bis an die
Begrenzung.

## Fahrmodus

`switch_control` ist der einzige Publisher von `/control/mode`:

| Modus      | Wert | Wer faehrt   |
|------------|------|--------------|
| `Lane`     | 1    | control_driving |
| `Obstacle` | 2    | avoid_node   |
| `Stop`     | 3    | niemand      |

Der Modus sagt nur, wer fahren **duerfte** -- ob von dort auch aktuelle
Befehle kommen, zeigt erst `/control/cmd_source` (`none` = Nullbefehl). Beim
Testen ist das deshalb die aussagekraeftigere Anzeige, und sie steht auch im
Debugbild.

## Diagnose

```bash
ros2 topic echo /avoid/zones          # Zonendistanzen + Betriebsart
ros2 topic echo /control/cmd_source   # welche Quelle gerade faehrt
ros2 topic echo /control/mode         # Fahrmodus (1 Lane, 2 Obstacle, 3 Stop)
ros2 topic echo /avoid/offset         # aktueller Offset
bash scripts/tb/launchers/rqt_params.sh    # Parameter live nachziehen
```

## Parameterdateien

| Datei                             | Node            |
|-----------------------------------|-----------------|
| `Parameter/Follow_Lane_ROI_PI_CAM2_V3.yaml` | detect_lane |
| `Parameter/Control_driving_PID.yaml` | control_driving |
| `Parameter/Avoid_Node.yaml`       | avoid_node      |
| `Parameter/Switch_Control.yaml`   | switch_control  |

Flaches Format (`name: wert`) wie die uebrigen Dateien, damit die
Datei-Knoepfe in `rqt_reconfigure` benutzbar bleiben. Ueberschreibbar beim
Start, z.B.:

```bash
ros2 launch Follow_Lane zone_avoid.launch.py avoid_params:=/pfad/datei.yaml
```
