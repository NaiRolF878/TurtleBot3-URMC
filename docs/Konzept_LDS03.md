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
| `Obstacle`    | Frontzone naeher als `takeover_distance`                     | Ausweichmanoever aus `/avoid/zones`   |
| `TunnelEntry` | Button im Debugbild oder Tunnelschild                        | Nav2 (`/nav2/cmd_vel`)                |
| `Tunnel`      | LiDAR sieht die Tunnelwaende oder Einfahrtsstrecke erreicht  | Nav2 (`/nav2/cmd_vel`)                |

Jeder andere Modus, veraltete Eingangsdaten oder ein fehlendes `/control/mode`
fuehren zu einem Nullbefehl.

```
detect_lane_nav2 --/detect/lane--------------------.
avoid_node -------/avoid/zones-------------------. |
controller_server --/nav2/cmd_vel--------------. | |
                                               v v v
switch_control --/control/mode--------> control_driving --/control/cmd_vel--> notaus --/cmd_vel--> Bot
      ^   ^
/scan-'   '--/avoid/zones
```

## Challenges

**Spurfolge (Lane).** Kamera -> metrische Vogelperspektive (ingress_node_nav2) ->
Spurerkennung in LAB (detect_lane_nav2) -> PID in control_driving. Kalibrierung:
`Kalibrierung_Checkliste.md`.

**Hindernisse auf der Strecke (Obstacle).** Zonenaufteilung, Offset im Lane-Modus und
Ausweichmanoever mit Hysterese, Details in `Zonen_Ausweichen.md`. Nur LiDAR. Kein
Linienfilter und keine Spurwaende: Ob ein Hindernis auf der eigenen Spur steht, sieht
das Zonen-Ausweichen nicht. Die Spurerkennung haelt im Lane-Modus gegen die Linien.

**Tunnel (TunnelEntry, Tunnel).** Wie auf `Nav2-Costmap-Fix`: Einfahrt mit Kamera ueber
die Spurmitte und NavFn/MPPI, ab der Oeffnung nur LiDAR (Kamera laut Reglement verboten),
Ausgangssuche mit tunnel_explorer_node, Eingangssperre mit keepout_zone_node. Details:
`Nav2LocalCoastmap_Doku.md`, `Nav2_Costmap_Werte.md`.

## Start

```bash
bash scripts/tb/launchers/nav2_obstacle.sh   # alles inkl. avoid_node, Not-Aus im Vordergrund
bash scripts/tb/launchers/follow_lane.sh     # nur Spurfolge, ohne Ausweichen und Tunnel
```

Weitere Befehle: `Befehle.md`.

## Gemeinsame Doku, die hier nur eingeschraenkt gilt

Diese Dateien kommen unveraendert von `Nav2-Costmap-Fix`. Ihre Aussagen zum
Obstacle-Modus beschreiben die Nav2-Variante und gelten auf diesem Branch nicht:

- `Befehle.md`: Nav2 faehrt hier nur TunnelEntry und Tunnel.
- `Nav2LocalCoastmap_Doku.md`, `Nav2_Costmap_Werte.md`, `Obstacle_Tunnel_Offene_Punkte.md`:
  Abschnitte zu Obstacle, Linienfilter und Bahnplanung um Hindernisse.
- Kommentare in `Parameter/nav2_params.yaml` und `Parameter/Lane_Costmap.yaml`:
  dort genannte Obstacle-Bezuege.

Die Tunnel-Teile dieser Dateien gelten unveraendert.
