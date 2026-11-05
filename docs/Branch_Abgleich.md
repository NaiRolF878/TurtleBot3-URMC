# Abgleich mit `Nav2-Costmap-Fix`

`LDS03-Doku` uebernimmt alles von `Nav2-Costmap-Fix` per Merge und unterscheidet sich nur
im Obstacle-Modus (Zonen-Ausweichen statt Nav2, siehe `Konzept_LDS03.md`).

## Dateien, die bewusst abweichen

Nur auf diesem Branch:

| Datei | Inhalt |
|-------|--------|
| `workspace/src/Follow_Lane/Follow_Lane/src/avoid_node.py` | Zonenauswertung |
| `workspace/src/Follow_Lane/Follow_Lane/Parameter/Avoid_Node.yaml` | Zonen, Kosten, Offset |
| `workspace/src/follow_lane_msg/msg/AvoidZones.msg` | Message von avoid_node |
| `docs/Konzept_LDS03.md`, `docs/Zonen_Ausweichen.md`, `docs/Branch_Abgleich.md` | Doku dieses Branches |

Gemeinsame Dateien mit Aenderungen (hier entstehen beim Merge Konflikte):

| Datei | Abweichung |
|-------|------------|
| `src/switch_control.py` | Lane <-> Obstacle ueber `/avoid/zones` mit Hysterese statt LiDAR-Kegel + Linienfilter; `takeover_*` ohne Code-Default; `obstacle_enabled` wirkt auf diese Uebernahme |
| `src/control_driving.py` | Lane: PID + Offset; Obstacle: Ausweichmanoever; TunnelEntry: PID ohne Offset; Nav2 nur im Tunnel (`NAV2_MODES`); `avoid_*` ohne Code-Default, `AVOID_TIMEOUT` |
| `src/nav2_goal_publisher.py` | plant nie (`PLAN_MODES` leer), Nav2 nur im Tunnel; Docstring, Kommentare |
| `src/lane_costmap_node.py` | keine Spurmitte (`CENTER_PATH_MODES` leer), Docstring |
| `src/debug_view.py` | Anzeige der Fahrquelle je Modus (`MODE_SOURCE`) |
| `Parameter/Switch_Control.yaml` | `takeover_*` statt `obstacle_*` |
| `Parameter/Control_driving_PID_Wide.yaml` | zusaetzlich `avoid_linear_speed`, `avoid_angular_speed` |
| `launch/nav2_obstacle.launch.py` | startet avoid_node, Docstring, Kommentare zu `/nav2/cmd_vel` und `obstacle:=true` |
| `launch/follow_lane.launch.py` | Kommentar zu `obstacle_enabled`: Obstacle braucht avoid_node statt Nav2 |
| `scripts/tb/launchers/nav2_obstacle.sh` | avoid_node im Prozessmuster, Kopfkommentar |
| `setup.py` | Eintrag `avoid_node` |
| `workspace/src/follow_lane_msg/CMakeLists.txt` | `msg/AvoidZones.msg` |

Auf diesem Branch geloescht: `docs/URMC–Technisches_Konzept.md` (liegt nur auf `main`)
und `docs/Konzept_Nav2.md`.

Aus den nicht uebernommenen Commits `b989a0f` und `838ca32` (per `-s ours` als abgeglichen
markiert) weichen ausserdem ab, ohne Bezug zum Ausweichen:

| Datei | Abweichung |
|-------|------------|
| `README.md` | Stand vor `b989a0f` |
| `docs/Kalibrierung_Checkliste.md`, `docs/Nav2LocalCoastmap_Doku.md`, `docs/Nav2_Costmap_Werte.md` | je eine Formulierung vor `b989a0f` |
| `docs/ToDo.md`, `entrypoint.sh`, `novnc_startup.sh` | hier noch vorhanden, in `838ca32` entfernt |

Alles andere ist identisch zu `Nav2-Costmap-Fix`.

## Ablauf fuer den naechsten Abgleich

```bash
cd wt-lds03
git fetch
git merge --no-commit origin/Nav2-Costmap-Fix
```

1. Konflikte in den oben gelisteten Dateien: die Aenderung von `Nav2-Costmap-Fix`
   uebernehmen und die Abweichung dieses Branches wieder einsetzen. Betrifft die
   Aenderung den Obstacle-Modus der Nav2-Variante (z. B. Linienfilter), bleibt sie
   hier draussen.
2. Das Konzeptdokument von `Nav2-Costmap-Fix` (`docs/Konzept_Nav2.md`) und
   `docs/URMC–Technisches_Konzept.md` gehoeren nicht auf diesen Branch: beim Merge
   loeschen (`git rm`), auch wenn Git dafuer einen Konflikt meldet.
3. Pruefen:
   ```bash
   git diff origin/Nav2-Costmap-Fix --stat     # nur die Dateien aus dieser Liste
   grep -rn "create_publisher(Twist" workspace/src/Follow_Lane/Follow_Lane/src   # nur control_driving, notaus
   grep -rn "'/control/mode'" workspace/src/Follow_Lane/Follow_Lane/src | grep create_publisher   # nur switch_control
   ```
4. Neue Nodes oder Modi auf `Nav2-Costmap-Fix`, die `ControlType.Obstacle` verwenden:
   pruefen, ob sie hier etwas tun duerfen, und die Liste oben ergaenzen. Neue
   Pflicht-Parameter (`require_params`) muessen in der YAML stehen, die der Launch laedt.
5. Commit, danach am Bot: Lane mit Offset, Obstacle-Uebernahme und Rueckgabe,
   TunnelEntry ohne Offset, Tunnel per Button.
