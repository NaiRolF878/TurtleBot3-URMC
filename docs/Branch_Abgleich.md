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
| `src/switch_control.py` | Lane <-> Obstacle ueber `/avoid/zones` mit Hysterese statt LiDAR-Kegel + Linienfilter |
| `src/control_driving.py` | Lane: PID + Offset; Obstacle: Ausweichmanoever; Nav2 nur in TunnelEntry/Tunnel |
| `src/nav2_goal_publisher.py` | plant nicht im Obstacle-Modus (`PLAN_MODES`) |
| `src/lane_costmap_node.py` | Spurmitte nur in TunnelEntry (`CENTER_PATH_MODES`), Docstring |
| `src/debug_view.py` | Anzeige der Fahrquelle je Modus (`MODE_SOURCE`) |
| `Parameter/Switch_Control.yaml` | `takeover_*` statt `obstacle_*`, `line_*`, `lane_max_age` |
| `launch/nav2_obstacle.launch.py` | startet avoid_node, Docstring |
| `scripts/tb/launchers/nav2_obstacle.sh` | avoid_node im Prozessmuster, Kopfkommentar |
| `setup.py` | Eintrag `avoid_node` |
| `workspace/src/follow_lane_msg/CMakeLists.txt` | `msg/AvoidZones.msg` |

Auf diesem Branch geloescht: `docs/URMC–Technisches_Konzept.md` (liegt nur auf `main`).

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
   pruefen, ob sie hier etwas tun duerfen, und die Liste oben ergaenzen.
5. Commit, danach am Bot: Lane mit Offset, Obstacle-Uebernahme und Rueckgabe,
   Tunnel per Button.
