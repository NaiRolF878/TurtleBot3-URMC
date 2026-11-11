# Konzept des Branches `LDS03-Doku`

Dieser Branch entspricht `Nav2-Costmap-Fix` bis auf den **Obstacle-Modus**: Hindernisse
auf der Strecke werden ueber eine **Zonenaufteilung des LiDAR** umfahren statt ueber
Costmap und Nav2. Wie die gemeinsamen Teile aktuell gehalten werden, steht in
`Branch_Abgleich.md`. Die Sammlung aller Konzeptvarianten liegt unveraendert auf `main`
(`docs/URMC–Technisches_Konzept.md`).

## Architektur

- **switch_control** entscheidet als einziger ueber den Fahrmodus (`/control/mode`).
- **control_driving** erzeugt als einziger Fahrbefehle (`/control/cmd_vel`).
- **notaus** gibt sie nach Freigabe an den Bot weiter (`/cmd_vel`).
- Alle anderen Nodes liefern nur Daten.

| Modus         | Ausloeser (switch_control)                                   | Fahrbefehl (control_driving)          |
|---------------|--------------------------------------------------------------|---------------------------------------|
| `Lane`        | Start, Ende von Obstacle; nach dem Tunnel der Modus davor    | PID auf den Spurfehler + Ausweich-Offset |
| `Obstacle`    | Frontzone naeher als `takeover_distance`, `obstacle_enabled` | Ausweichmanoever aus `/avoid/zones`   |
| `TunnelEntry` | Button im Debugbild oder Tunnelschild                        | PID auf den Spurfehler, ohne Offset   |
| `Tunnel`      | Beide Spurlinien enden an der Oeffnung                       | Nav2 (`/nav2/cmd_vel`); Engstelle: Grundbewegungen |

Jeder andere Modus, veraltete Eingangsdaten oder ein fehlendes `/control/mode`
fuehren zu einem Nullbefehl.

```
detect_lane_nav2 --/detect/lane--------------------.
avoid_node -------/avoid/zones-------------------. |
controller_server --/nav2/cmd_vel--------------. | |
                                               v v v
switch_control --/control/mode--------> control_driving --/control/cmd_vel--> notaus --/cmd_vel--> Bot
      ^
      '-- /avoid/zones, /detect/lines_lost, /tunnel/outside, Button, Schild
```

## Challenges

**Spurfolge (Lane).** Kamera -> metrische Vogelperspektive (ingress_node_nav2) ->
Spurerkennung in LAB (detect_lane_nav2) -> PID in control_driving. Kalibrierung:
`Kalibrierung_Checkliste.md`.

**Hindernisse auf der Strecke (Obstacle).** Zonenaufteilung, Offset im Lane-Modus und
Ausweichmanoever mit Hysterese, Details in `Zonen_Ausweichen.md`. Nur LiDAR. Kein
Linienfilter und keine Spurwaende: Ob ein Hindernis auf der eigenen Spur steht, sieht
das Zonen-Ausweichen nicht. Die Spurerkennung haelt im Lane-Modus gegen die Linien.

**Tunnel (TunnelEntry, Tunnel).** Wie auf `Nav2-Costmap-Fix`: Einfahrt mit Kamera und PID
(ohne Offset), ab dem Linienende nur LiDAR (Kamera laut Reglement verboten), Ausgangssuche
mit tunnel_explorer_node und NavFn/MPPI, Eingangssperre mit keepout_zone_node. Engstellen von
0,21 bis 0,26 m faehrt switch_control (tunnel_passage.py) mit Grundbewegungen, der Modus bleibt
Tunnel. Details:
`Nav2LocalCoastmap_Doku.md`, `Nav2_Costmap_Werte.md`.

**Nav2 plant hier nur den Tunnel.** `PLAN_MODES` (nav2_goal_publisher) und
`CENTER_PATH_MODES` (lane_costmap_node) sind leer, der Code dahinter bleibt. Das
Spurgedaechtnis laeuft weiter (Debugbild, lane_layer), ausser im Tunnel; niemand plant
damit, und switch_control nutzt keinen Linienfilter.

## Start

```bash
bash scripts/tb/launchers/nav2_obstacle.sh   # alles inkl. avoid_node, Not-Aus im Vordergrund
bash scripts/tb/launchers/follow_lane.sh     # nur Spurfolge, ohne Ausweichen und Tunnel
```

Weitere Befehle: `Befehle.md`.

## Gemeinsame Doku, die hier nur eingeschraenkt gilt

Diese Dateien kommen unveraendert von `Nav2-Costmap-Fix`. Ihre Aussagen zum
Obstacle-Modus beschreiben die Nav2-Variante und gelten auf diesem Branch nicht:

- `Befehle.md`: Nav2 faehrt hier nur den Tunnel.
- `Nav2LocalCoastmap_Doku.md`, `Nav2_Costmap_Werte.md`, `Obstacle_Tunnel_Offene_Punkte.md`,
  `Kalibrierung_Checkliste.md`: Abschnitte zu Obstacle, `obstacle_*`-Werten, Linienfilter
  und Bahnplanung um Hindernisse.
- Kommentare in `Parameter/nav2_params.yaml` und `Parameter/Lane_Costmap.yaml`:
  dort genannte Obstacle-Bezuege. `goal_distance` und `max_path_backward` muessen in
  `Lane_Costmap.yaml` bleiben (nav2_goal_publisher verlangt sie), wirken hier aber nicht.

Die Tunnel-Teile dieser Dateien gelten unveraendert.
