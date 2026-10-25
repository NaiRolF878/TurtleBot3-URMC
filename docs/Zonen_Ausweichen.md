# Zonen-Ausweichen (Obstacle-Modus auf `LDS03-Doku`)

Hindernisse auf der Strecke werden ueber eine feste Zoneneinteilung des LiDAR
umfahren. Nav2 wird dafuer nicht benutzt; es faehrt auf diesem Branch nur
TunnelEntry und Tunnel. Ueberblick ueber den ganzen Branch: `Konzept_LDS03.md`.

## Aufgabenteilung

| Node              | Aufgabe im Ausweichen                                          |
|-------------------|----------------------------------------------------------------|
| `avoid_node`      | Zonen auswerten, Offset und Ausweichrichtung melden (`/avoid/zones`) |
| `switch_control`  | entscheidet Lane <-> Obstacle (Frontzone, Hysterese)           |
| `control_driving` | rechnet den Fahrbefehl: PID + Offset bzw. Ausweichmanoever     |
| `notaus`          | Fahrfreigabe, reicht `/control/cmd_vel` an `/cmd_vel` durch    |

avoid_node faehrt nicht selbst und entscheidet keinen Modus.

## Signalfluss

```
detect_lane --/detect/lane------------------------------> control_driving --/control/cmd_vel--> notaus --> /cmd_vel
                                                             ^      ^
/scan --> avoid_node --/avoid/zones (AvoidZones)-------------+      |
                              |                                     |
                              '--------------> switch_control --/control/mode
```

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

## Lane-Modus: Offset auf den Spurfehler

Solange die Front nicht blockiert ist, faehrt der Spur-PID weiter.
control_driving addiert den Offset aus `/avoid/zones` auf den Spurfehler:

- Hindernis rechts (`front_right`) -> positiver Offset -> Ausweichen nach links
- Hindernis links (`front_left`) -> negativer Offset -> Ausweichen nach rechts
- Staerke proportional zur Naehe (`offset_trigger_distance` bis
  `offset_saturation_distance`)
- beide Seiten belegt -> die Anteile heben sich teilweise auf, der Roboter
  faehrt mittig durch

Die Spurerkennung laeuft dabei weiter und haelt dagegen, sodass die Linien
nicht ueberfahren werden. Ist `/avoid/zones` aelter als `avoid_timeout`, rechnet
control_driving ohne Offset.

> **Vorzeichen:** Ein **positiver** Spurfehler laesst control_driving nach
> **links** lenken. Der Offset folgt derselben Konvention. Wird das in
> detect_lane gedreht, genuegt `offset_direction: -1.0` in `Avoid_Node.yaml`.

## Obstacle-Modus: Ausweichmanoever

Ist die Frontzone naeher als `takeover_distance` (0.20 m), reicht der Offset
nicht mehr und switch_control schaltet auf `Obstacle`. control_driving faehrt
dann aus den Daten von avoid_node:

- Drehrichtung `direction` (+1 links, -1 rechts, 0 geradeaus) mal `avoid_angular_speed`
- vorwaerts mit `avoid_linear_speed`, ausser `forward_allowed` ist false: Front
  naeher als `avoid_turn_only_distance` oder kein Sektor frei und keine Drehrichtung

Zurueck zu `Lane` erst, wenn die Front `takeover_release_debounce` Sekunden lang
weiter als `takeover_release_distance` (0.45 m) frei ist. Ohne diese Hysterese
springt der Modus an der Schwelle zwischen PID und Ausweichmanoever.

Bleibt `/avoid/zones` im Obstacle-Modus laenger als `avoid_timeout` aus, sendet
control_driving Null.

## Diagnose

```bash
ros2 topic echo /avoid/zones          # Zonendistanzen, Offset, Richtung, Sektor
ros2 topic echo /control/mode         # Fahrmodus (1 Lane, 2 Obstacle, 7 TunnelEntry, 6 Tunnel)
bash scripts/tb/launchers/rqt_params.sh    # Parameter live nachziehen
```

Im Debugbild steht je Modus, wer faehrt (`PID + Offset`, `Zonen`, `Nav2/MPPI`).

## Parameter

| Datei / Ort                     | Inhalt                                                    |
|---------------------------------|-----------------------------------------------------------|
| `Parameter/Avoid_Node.yaml`     | Zonen, Kosten, `clear_distance`, `avoid_turn_only_distance`, Offset |
| `Parameter/Switch_Control.yaml` | `takeover_distance`, `takeover_release_distance`, `takeover_release_debounce` |
| control_driving (Code-Default)  | `avoid_linear_speed` 0.06, `avoid_angular_speed` 0.7, `avoid_timeout` 0.5 |

`Avoid_Node.yaml` ist beim Start ueberschreibbar:

```bash
ros2 launch Follow_Lane nav2_obstacle.launch.py avoid_params:=/pfad/datei.yaml
```
