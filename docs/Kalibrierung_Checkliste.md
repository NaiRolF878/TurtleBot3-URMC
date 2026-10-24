# Kalibrierung und Live-Test — Checkliste

Für den Stand auf `Nav2-Costmap-Fix`: Spurerkennung auf der metrischen
Vogelperspektive (`/lokal/bev`), Spurwände und Spurmitte für Nav2 aus derselben
BEV, Fahrfreigabe über den Not-Aus (`notaus`).

Reihenfolge einhalten: Jeder Schritt baut darauf auf, dass der vorherige stimmt.
Ist die Kamera falsch vermessen, ist jede Meterangabe danach falsch. Und solange
der PID die Spur nicht hält, lässt sich Nav2 nicht sinnvoll testen.

Ersetzt die Checkliste vom 10.09. Deren Schritte zu ROI-Ecken, `x_near`/`x_far`
und einer zur Spurbreite passenden Inflation gibt es nicht mehr.

---

## Phase 0 — Bauen und Voraussetzungen

- [ ] **Sauber bauen.** `LaneMask` hat sich geändert. Ohne Löschen bleibt die
      alte Nachrichtendefinition im Environment liegen:
      ```bash
      cd workspace
      rm -rf build/follow_lane_msg install/follow_lane_msg
      colcon build --symlink-install
      ```
- [ ] **Kamera unverändert seit der Extrinsik-Messung vom 20.10.** Laut
      `Parameter/camera_info/extrinsics_800x600.yaml`: Höhe 10,1 cm, Neigung
      38,0°, Roll −1,0°, Yaw 1,4°, Linse 5,79 cm vor der Radachse. Die BEV
      wird daraus gerechnet. Ist die Halterung verstellt, geht es mit Phase 1
      weiter, sonst kann Phase 1 kurz ausfallen.
- [ ] **Kamera auf 800×600** (`scripts/tb/camera.conf`). Intrinsik und
      Extrinsik gelten nur dafür.

---

## Phase 1 — Kamera und BEV prüfen

- [ ] **Bodengitter ansehen:** `extrinsik_view.sh` im `tb`-Menü. Es zeigt das
      Kamerabild mit 10-cm-Gitter, ohne dass der Bot losfährt. Das Gitter muss
      auf dem echten Boden liegen.
- [ ] **BEV ansehen:** `rqt_debug.sh` → `/lokal/bev`. Gerade Spurlinien müssen
      gerade und parallel erscheinen. Ein Band 30 cm vor der Radachse liegt
      200 px über der Unterkante (1,25 mm/px, Unterkante = 5 cm).
- [ ] **Nur wenn das nicht stimmt:** Extrinsik neu messen (Brett flach am
      Boden), danach `--recompute --preview`:
      ```bash
      python3 scripts/tb/camera-extrinsics.py
      python3 scripts/tb/camera-extrinsics.py --recompute --preview
      ```
      Ablauf und Messwerte: `docs/bev-tests/README.md`, Kurzform in
      `docs/Befehle.md`.

`Parameter/camera_info/ground_homography_800x600.yaml` gehört zur alten
Kameramontage (12,3 cm Höhe). Nav2 benutzt sie nicht mehr.

---

## Phase 2 — Spurerkennung, PID als Referenz

**Der PID muss die Spur zuverlässig halten, bevor Nav2 getestet wird.**

- [ ] `follow_lane.sh` starten. Der Bot fährt erst nach **Leertaste** im
      selben Terminal (Not-Aus), `q` beendet.
- [ ] `/detect/mask_debug/compressed` ansehen: Gelb und Weiß sauber getrennt,
      keine Linie am Rand abgeschnitten?
- [ ] Schwellen in `rqt_params.sh` → `detect_lane` nachziehen (wirkt sofort):
  - Farbfilter in **LAB** (`lab_l_*`, `lab_a_*`, `lab_b_*`, je `_l`/`_h`):
    L = Helligkeit, A = grün (<128) bis rot (>128), B = blau (<128) bis gelb
    (>128), 128 = farblos. **Gelb** erkennt man am hohen B (`lab_b_yellow_l`
    150), **Weiß** am hohen L bei A/B nahe 128 (`lab_l_white_l` 200). Die
    Messwerte vom 22.10. stehen als Kommentar in `detect_lane.py`.
  - `min_lane_width` ist in BEV-Pixeln: 80 px = 10 cm.
  - `detection_row_factor`: Suchzeile im Bild (0,80 = 80 % der Höhe).
  - Sichern: `bash scripts/tb/save-params.sh detect_lane`
- [ ] **Abnahme:** eine Runde mit `follow_lane.sh`. Die Linien dürfen nicht
      überfahren werden (laut Reglement −1 Punkt).

---

## Phase 3 — Meter-Test, der wichtigste Test

- [ ] `nav2_obstacle.sh` starten, noch **nicht** freigeben (keine Leertaste).
- [ ] Ein Klebeband quer in gemessenem Abstand vor den Bot legen, z. B. 40 cm
      vor der Radachse. In RViz das Display `Spurwaende (nicht im Tunnel)`
      bzw. die Draufsicht `/nav2/view/compressed` ansehen: Das Band muss dort
      erscheinen, wo es real liegt. Die Draufsicht hat einen 0,5-m-Maßstab
      unten rechts.
- [ ] Bot auf die Spur stellen: Beide Linien müssen als schwarze Zellreihen
      erscheinen, die Anzeige `Spurmitte` mittig dazwischen.
- [ ] Weicht etwas ab: zurück zu Phase 1. Nicht an Nav2 drehen.

---

## Phase 4 — Modusumschaltung

Werte in `Switch_Control.yaml`. `obstacle_check_angle_deg` (20°) ist der
**Halbwinkel**, der Kegel ist 40° breit. Ausgelöst wird bei
`obstacle_trigger_distance` (1,0 m).

- [ ] **Fehlauslösung:** eine Runde **ohne** Hindernis. Im Debugbild muss
      durchgehend `Lane` stehen. Springt es auf `Obstacle`, sieht der LiDAR
      die Streckenbegrenzung. Dann Winkel oder Distanz verkleinern.
- [ ] **Auslösung:** Hindernis aufstellen. Der Wechsel auf `Obstacle` muss
      früh genug kommen.
- [ ] Hinweis: Die BEV reicht nur bis 0,60 m. Beim Umschalten bei 1,0 m
      fehlen zwischen 0,6 und 1,0 m noch Spurwände, bis der Bot näher kommt.

---

## Phase 5 — Ausweichen testen

- [ ] Freigabe mit **Leertaste**, Stopp mit Leertaste.
- [ ] Draufsicht im Blick behalten. Die Statuszeile zeigt, was Nav2 tut
      (z. B. „faehrt: Spurmitte“), und den letzten Abbruchgrund von
      `controller_server`. Rot heißt jünger als 10 s.
- [ ] Im Debugbild muss `Spur -> Costmap` eine Punktzahl in Grün zeigen. Steht
      dort `still`, kommt nichts an. Dann die Ursache suchen und nicht an MPPI
      drehen.
- [ ] Weicht er falsch oder gar nicht aus, in dieser Reihenfolge prüfen:
      1. Liegen die Spurwände auf den echten Linien? (Phase 3)
      2. Gibt es eine grüne Spurmitte? Ohne sie fährt Nav2 die Ersatz-Gerade.
      3. Meldet die Statuszeile, dass die Regelschleife zu langsam ist?
         Die Prüfung mit dem echten Umriss (`consider_footprint`) kostet
         Rechenzeit.
      4. Erst dann MPPI-Gewichte (`CostCritic` hoch, `PathAlignCritic` runter).

---

## Phase 6 — Tunnel (Enclosure)

- [ ] In `rqt_image_view` (Debugbild) einmal **„Publish click location“**
      anhaken.
- [ ] Bot direkt vor oder in die Öffnung stellen, im Debugbild auf **TUNNEL
      STARTEN** klicken. Unter dem Modus muss „(von Hand)“ stehen.
- [ ] Draufsicht: Nach ~16 cm Fahrt erscheint die orange Eingangssperre an der
      Startstelle. `entrance_wall_width` (0,6 m, `Lane_Costmap.yaml`) muss
      mindestens so breit sein wie die echte Öffnung.
- [ ] Nach dem Austritt springt der Button zurück, der Bot wechselt in `Lane`.
      Stoppen bei Bedarf mit der Leertaste.

---

## Nützliche Befehle

```bash
ros2 topic echo /control/mode --once      # aktueller Modus
ros2 topic echo /notaus/drive             # Fahrfreigabe an?
ros2 topic echo /nav2/status              # was Nav2 gerade tut
ros2 topic hz /lokal/bev                  # kommt die BEV?
ros2 topic hz /detect/lane_obstacles      # kommen Spurwände an?
ros2 topic hz /local_costmap/costmap      # lebt die Costmap?
ros2 topic echo /lokal/debug_view_mouse_left   # kommen Klicks aus rqt_image_view an?
```

Weitere Befehle: `docs/Befehle.md`.

---

## Noch offen

- **`avoid_obstacle.py`:** Er prüft `scan.range_min`. Das ist eine Konstante
  des Sensors und kein Messwert, deshalb meldet `/detect/obstacle` immer
  `True`. Harmlos, weil kein Node das Topic liest.
