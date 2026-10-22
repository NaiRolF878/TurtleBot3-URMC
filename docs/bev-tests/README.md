# BEV-Schaerfe-Tests (2026-10-20)

Ziel: Vogelperspektive im oberen Bereich (40-70 cm) schaerfer bekommen. Die Geometrie
stimmt bereits (ChArUco-Kacheln in der BEV 3,0 x 3,0 cm, Kanten gerade/parallel) --
es geht NUR um Schaerfe/Detail.

Szene fuer alle Bilder: Bot auf dem Schreibtisch, ChArUco-Brett ca. 35-65 cm voraus.
Bilder pro Schritt (Praefix = Schritt): `_roh` = JPEG vom Bot (verzeichnet),
`_entzerrt` = /lokal/raw_image, `_ansicht` = /lokal/extrinsics_view (Gitter + BEV).
Aufnehmen mit `python3 docs/bev-tests/grab.py <praefix>`.

## Ausgangszustand (Schritt 0) -- so war alles VOR den Tests

| Was | Wert |
|---|---|
| Kamera (camera.conf) | `CAM_W=800 CAM_H=600 CAM_Q=80 CAM_FRAME_US=33333` (30 Hz), Sensormodus 1640:1232 |
| Tuning | `CAM_TUNING="imx219_weitwinkel.json"`, `CAM_ORIENTATION=0`, AWB automatisch |
| Intrinsik | `imx219_800x600.yaml`, Fisheye (equidistant), fx 385.4, cx 389.8, cy 285.8, HFOV ~119 Grad |
| Entzerrung | ingress_node `undistort:=true balance:=0.0` -> entzerrtes K fx 294.1, cx 373.6, cy 278.1 |
| Extrinsik | `extrinsics_800x600.yaml`: Hoehe 10.13 cm, Neigung 37.97 Grad, Roll -1.03 Grad, forward 0 (Ursprung = Linse!) |
| JPEG-Groesse | ~34 KB/Bild bei 800x600 q80 |
| Netz | Kabel, Laptop 192.168.77.1, Bot gertrude 192.168.77.2 |

Aufloesung am Boden bei 800x600 (in Fahrtrichtung, pro Kamerapixel): 20 cm ~0,2 cm,
40 cm ~0,6 cm, 60 cm ~1,2 cm, 80 cm ~2,5 cm. Eine 3-cm-Kachel bei 55 cm = nur 2-3 Pixelzeilen.

Ursachen der Unschaerfe: (1) zu wenige Pixel in der Ferne, (2) doppelte Interpolation
(ingress_node entzerrt, BEV warpt nochmal), (3) JPEG q80.

## Schritte

1. JPEG q80 -> q90
2. 800x600 -> 1640x1232 (gleiches Seitenverhaeltnis +-0,2 %, K wird mitskaliert)
3. Entzerrung + BEV in EINEM Remap direkt aus dem Rohbild

Ergebnisse siehe unten (wird pro Schritt ergaenzt).

## Ergebnisse (2026-10-20, alle Bilder in diesem Ordner)

Vergleichsbild: `vergleich.png` (Zeilen: 800 q80 / 800 q90 / 1640 q90; Spalten: zweistufig wie
jetzt / direkt 1 Remap / direkt kubisch; Ausschnitt 30-75 cm, +-20 cm, 10 px/cm).
Rohbild-Ausschnitt ums Brett: `roh_ausschnitt_800_vs_1640.png` (links 800, rechts 1640).
Neu rechnen: `python3 docs/bev-tests/compare.py`.

| Schritt | Ergebnis |
|---|---|
| q80 -> q90 | **kein sichtbarer Gewinn.** JPEG 34 -> 58 KB. |
| 800x600 -> 1640x1232 | **Rohbild deutlich schaerfer** (Marker im Brett lesbar). In der BEV sichtbar, aber kleiner als erhofft (Marker bis ~50 cm klarer). 30 Hz gehalten, JPEG 267 KB = ~64 Mbit/s -> nur per Kabel, nicht WLAN. Pi-Last nicht gemessen. ingress_node + extrinsics-view skalieren K korrekt (Warnung "besser neu kalibrieren" ist hier unkritisch, Seitenverhaeltnis +-0,2 %). |
| Direkt-Remap statt zweistufig | **leicht schaerfer**, kleiner Effekt. Kubisch kaum Unterschied zu linear. |

**Hauptbefund: die Unschaerfe ist GEOMETRISCH, nicht die Kamera.** Bei 10 cm Hoehe und 38 Grad
Neigung sieht die Kamera den Boden bei 40-65 cm sehr flach: das Brett (28 x 30 cm) belegt im
1640er-Rohbild ~550 px Breite, aber nur ~100 px Hoehe. Laengs (Fahrtrichtung) also ~3 px/cm,
quer ~20 px/cm. Die BEV muss laengs stark hochskalieren -> Schlieren in Fahrtrichtung. Das
bleibt bei jeder Aufloesung so (1640 halbiert es nur). Abhilfe nur ueber Geometrie (Kamera
hoeher/steiler -- laut Nutzer nicht moeglich) oder Bereich enger fassen (bis ~50 cm).

Stand nach den Tests: `camera.conf` wieder auf Ausgangszustand (800x600 q80), Sicherung als
`camera.conf.vorher`. Auf dem Bot lief die Kamera zuletzt noch mit 1640x1232 q90 (Bot danach
ausgeschaltet, Akku leer) -- naechster botup startet wieder mit camera.conf.
Offen/Entscheidung Nutzer: 1640x1232 fuer den Fahrbetrieb (Kabel) uebernehmen? Direkt-Remap in
ingress_node/detect_lane einbauen?

## Naechster Schritt: forward-Versatz (Nullpunkt Radachse) bestimmen

Offen seit 2026-10-16: Gitter lag 6 cm neben dem Zollstock, `forward_offset_m` = 0 (Ursprung = Linse).
Ablauf (Bot auf dem Boden, Akku voll):
1. Brett gerade + mittig vor den Bot, vordere KACHELREIHE in gemessenem Abstand von der RADACHSE (z.B. 20 cm).
2. Live-Ansicht (`tb` -> `extrinsik_view.sh`): Anzeige "Brett .. cm voraus / seitlich" + magenta Ecken in der BEV.
3. Differenz Anzeige <-> Zollstock = forward. Live: `ros2 param set /extrinsics_view forward_offset_m <m>`.
4. Festschreiben: `python3 scripts/tb/camera-extrinsics.py --recompute --preview --forward <m>`.
5. Gegenprobe: Brett 15 cm weiter + 10 cm seitlich -- Anzeige muss wieder passen.
Bei Neumessung von Hoehe/Neigung: Brett nah legen (20-35 cm), dort ist die Aufloesung am besten.

## ERLEDIGT 2026-10-20: forward-Versatz bestimmt -> 5,79 cm

Brett gerade + mittig, vordere Rasterkante per Zollstock **21,5 cm ab Radachse** (Bild `3_brett_21.5cm_*`).
`python3 docs/bev-tests/board-position.py --zollstock-cm 21.5` (20 Messungen, Median):
Kante 15,71 cm vor der LINSE (+-0,00), Rastermitte 0,34 cm rechts (= mittig), Hoehe 10,30 cm, Neigung 38,20,
Roll -1,13 (passt auf 2 mm / 0,2 Grad zur gespeicherten Lage 10,13 / 37,97 / -1,03), RMS 0,42 px.
-> **Linse 5,79 cm vor der Radachse.** Erklaert die 6 cm Abweichung vom 16.10. vollstaendig.
Festgeschrieben: `camera-extrinsics.py --recompute --preview --forward 0.0579` (Hoehe/Neigung/Roll
unveraendert gelassen). Alte Datei: `extrinsics_800x600.yaml.vorher_forward0`. Vorschau: `4_vorschau_forward_5.8cm.png`.
Hinweis: Brett liegt bei 800x600 so flach im Bild, dass nur 5 von 17 Markern erkannt werden (Minimum 6) --
board-position.py skaliert das Bild deshalb 2x hoch (`--upscale`). camera-extrinsics.py (Messmodus) und
extrinsics-view.py tun das NICHT und erkennen das Brett in dieser Lage nicht.
Folge fuer detect_lane: der metrische Ausschnitt (20-60 cm) zaehlt jetzt ab Radachse statt ab Linse
(wirkt nur mit use_metric_warp=true, Default aus).

## Abnahme durch Nutzer (2026-10-20)
Zollstock gegen BEV: Entfernungen auf dem Brett passen auf ca. +-1 cm. Kacheln real 3,0 cm (Druckmassstab
stimmt -> Verdacht "Brett falsch skaliert" vom 16.10. ausgeraeumt). Linsenhoehe per Hand 10,1-10,5 cm
(passt zu 10,13/10,30). forward eindeutig: Nutzer hat immer 21,5 cm bis zur ersten Rasterkante gemessen (die Messung
"5_brett_20cm_*" mit --zollstock-cm 20 beruhte auf einem Missverstaendnis von mir und ist ungueltig).
Gespeichert: 5,79 cm. Gesamtgenauigkeit ca. +-1-1,5 cm -- reicht fuer Spurerkennung.

## Test 1640x1232 im Fahrbetrieb (2026-10-20, Kabel)

| | 800x600 q80 | 1640x1232 q80 |
|---|---|---|
| Kamera-Rate am Laptop | 30 Hz | 30 Hz |
| Bandbreite | 11,7 Mbit/s (47 KB/Bild) | 45,5 Mbit/s (185 KB/Bild) |
| camera_node CPU auf dem Pi | ~68 % (Pi gesamt ~20 %, 62 C) | ~177 % (Pi gesamt ~48 %, 66 C) |
| ingress_node (Laptop) | -- | 29,6 Hz, ~45 % CPU |
| detect_lane pro Bild (vorher) | 95 ms | 322 ms (= 2,3 Hz live gemessen) |
| detect_lane pro Bild (nach array-Fix) | **68 ms** | **176 ms (~6 Hz)** |

**detect_lane-Bremse gefunden + behoben (gilt fuer JEDE Aufloesung):** `msg.data = encoded.tobytes()` bei
CompressedImage -- rclpy prueft bytes Byte fuer Byte in Python (~0,7 s/MB). Fix: `array.array('B', ...)`.
Bei 800x600: 95 -> 68 ms.
Rest bei 1640 (pro Bild): Debug-JPEG volle Groesse ~40 ms, cvtColor+inRange ~40 ms, detect_intersection
(Python-Schleifen) ~47 ms, CLAHE/erode/warp/Debug-Polygone Rest.

detect_lane ist jetzt aufloesungsunabhaengig: REF_W/REF_H = 800x600, `px_scale` = Bildbreite/800 skaliert
Erosionskernel, min_edge_len, Konturflaeche/epsilon; RoI-Punkte (handgetunt UND metric_warp) werden auf die
echte Bildgroesse umgerechnet.

Fazit: 1640x1232 laeuft auf Kamera/Netz/Pi problemlos, aber detect_lane schafft nur ~6 Hz. camera.conf
wieder 800x600 q80. Moegliche Wege (Entscheidung Nutzer): Debug-Bild nur bei Abonnenten/verkleinert;
Farbfilter erst NACH dem Warp auf dem 400x400-Ausschnitt; detect_intersection auf verkleinerten Masken.

## Yaw (Kamera gegen Fahrtrichtung) per Fahrtest (2026-10-20, Bot auf Schaumstoffmatte)

Brett fest, Bot faehrt gerade rueckwaerts; Wandern der Brettmitte im Kamera-Bodensystem = echte Fahrtrichtung.
Skripte: `board-position.py --save`, `drive-straight.py` (Stopp per Odometrie), `drive-until-board.py`
(Stopp per KAMERA, nur rueckwaerts -- bevorzugt), `yaw-auswertung.py a.json b.json`.

| Fahrt | Strecke real | seitlich | Brettdrehung | Ergebnis |
|---|---|---|---|---|
| 1 (`yaw_vorher/nachher`) | 7,3 cm (Odom 11,3!) | -0,22 cm | -0,03 Grad | 1,72 Grad |
| 2 (vorwaerts, `yaw_2_start`) | Bot ist AUF DAS BRETT gefahren | -- | +1,0 | ungueltig |
| 3 (`yaw_3_vorher/nachher`) | 10,9 cm (Odom 10,0) | -0,22 cm | +0,12 | **1,17 Grad** |

=> Kamera schaut ca. **1,2-1,7 Grad nach LINKS** gegen die Fahrtrichtung (~1-1,5 cm seitlich bei 50 cm).
Offen: beide Fahrten gleiche seitliche Verschiebung trotz anderer Strecke -> evtl. Teil abstandsabhaengiger
Messfehler; laengere Fahrt (15 cm) klaert das. Yaw ist in `_camera_geometry.ground_homography` noch NICHT
vorgesehen (nur Hoehe/Neigung/Roll/forward/lateral).

LEHREN: (1) `/sensor_state torque: false` nach Bot-Neustart -> Motoren ignorieren cmd_vel; einschalten mit
`ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"`. (2) Odometrie auf der Matte unzuverlaessig
(einmal 11,3 statt 7,3 cm) -> nie per Odometrie auf ein Hindernis zu fahren; Stopp per Kamera.
(3) Kamera lieferte zwischendurch 1640x1232, obwohl camera.conf 800x600 sagt (Ursache ungeklaert).

## ERLEDIGT: Yaw in die Extrinsik eingebaut (2026-10-20)

**Gespeichert: `camera_yaw_deg: 1.4`** (nach Strecke gewichtetes Mittel aus Fahrt 1 und 3).
Sicherung ALLER vorherigen Dateien: `backup_vor_yaw/` (_camera_geometry.py, camera-extrinsics.py,
extrinsics-view.py, bev-live.py, extrinsics_800x600.yaml).
- `_camera_geometry.ground_homography(..., yaw_deg=0.0)` / `_rot_ground_to_cam(..., yaw_deg)`: Drehung um die
  Senkrechte VOR Neigung/Roll; positiv = Kamera schaut nach links. X zeigt jetzt in die echte Fahrtrichtung.
  Mit yaw=0 bit-identisch zur alten Rechnung (geprueft).
- `camera-extrinsics.py`: `--yaw`; `--forward/--lateral/--yaw` Default jetzt **aus der YAML** (vorher setzte
  jedes `--recompute` ohne `--forward` den Versatz still auf 0 zurueck!). YAML-Feld `camera_yaw_deg`.
  Vorschau skaliert K jetzt auf die tatsaechliche Bildgroesse (lief sonst bei 1640x1232 voellig falsch).
- `extrinsics-view.py` (Parameter `yaw_deg`, Infozeile), `bev-live.py` (`yaw_deg`), `compare.py` lesen den Yaw.
- detect_lane nutzt die warp_matrix aus der YAML -> Yaw automatisch drin (colcon build gemacht).
Vorschau: `9_vorschau_yaw_1.4.png` (Brett gerade + parallel zum Gitter).
