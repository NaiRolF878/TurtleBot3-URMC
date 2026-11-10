# Konzept Storage and Retrieval Maneuver (URMRC 2026, Abschnitt 4.2.2)

Branch `Storage-Retrieval`. **[V]** = Vermutung, am Bot oder bei der Orga zu klaeren.

## Ziel und Randbedingungen

Die Palette aus ihrer Zone in die Zielzone bringen, dort in der Zonenmitte ablegen und die
Zone verlassen. Palettenzone und Zielzone bestimmt die Jury vor der Challenge.

**Aus dem Reglement 2026** (`R_glement_Robohub_v_2026.pdf`, 4.2.2, Figure 5, Table 1):

- Drei Zonen ① bis ③, je ein Schild hinter der Zone.
- Die Zonen liegen rechts der Spur (in Fahrtrichtung des Bots in Figure 5), direkt
  nebeneinander und gleich breit, mit gemeinsamen weissen Trennlinien. Zur Spur hin sind sie
  offen: Dort ersetzt eine gestrichelte weisse Linie die rechte Spurlinie, genau entlang der
  drei Zonen.
- Transport ohne andere Zonen zu ueberqueren und mit der urspruenglichen Ausrichtung der Palette.
- Vier Landmarken an den Ecken des Bereichs duerfen zur Lokalisierung genutzt werden.
- Der Bot darf nicht umgebaut werden; der Grove-Elektromagnet gehoert zur Ausstattung.

**Antworten der Organisatoren (Mails):**

- Palette: gruener Rahmen auf einem grauen Kasten (Stuetze), vorne senkrecht ein
  Metallstreifen 15 x 5 cm fuer den Magneten. Keine Markierung; die Orga schlaegt vor, sie als
  Hindernis zu erkennen. In der Zielzone steht keine Stuetze.
- Schilder: blauer Kreis mit weisser Ziffer 1–3, unterhalb der LiDAR-Ebene.
- Zonen in aufsteigender Reihenfolge, angenommen in Fahrtrichtung (wie Figure 5) **[V]**.
- Start per ROS-Befehl von Hand; am Ende stoppt die Orga den Bot von Hand.

**Folgerungen:**

- Palette, Stuetze und Schilder liegen unter der LiDAR-Ebene (ca. 18 cm, Burger-URDF). Die
  Aufgabe wird **nur mit Kamera und Odometrie** geloest.
- Die Magnethoehe ist fest, der Magnet kann nur an oder aus. Ohne Stuetze in der Zielzone
  faellt die Palette beim Ablegen von Stuetzenhoehe auf den Boden; ob sie dabei gerade
  bleibt **[V]**.
- Die Kamera sieht den Magneten. Der Magnet haelt nur, wenn der Bot rechtwinklig auf den
  Streifen trifft. Magnetmitte 7,5 cm ueber dem Boden.
- Die Palette wird von Hand platziert: weder mittig noch sicher gerade in ihrer Zone.
- Der Boden ist schwarz (Teppich). In einer leeren Zone ist zwischen den weissen Raendern alles
  schwarz.

### Punkte (Table 1, Mission 5)

| Teil | Punkte | Bonus | Strafe | Zustand im Ablauf |
|------|--------|-------|--------|-------------------|
| Zone mit Palette erkennen, Bot positionieren | – | +2 | −1, wenn der Bot nicht anhaelt | Erkundung, `POSITIONIERT` |
| Palette aufnehmen | 1 | | | `AUSRICHTEN` bis `GRIFF_PRUEFEN` |
| Transport | 3 | | −1, wenn eine andere Zone ueberquert wird | `HERAUSFAHREN` bis `ABLAGE_ANFAHREN` |
| Ablegen | 1 | +1, wenn der Bot die Zone verlaesst | | `ABLEGEN`, `VERLASSEN` |

Eine Zeitwertung gilt nur fuer die Team Challenge, nicht fuer die Palette.

### Was erst vor Ort bekannt ist

Groesse der Zonen, Groesse der Schilder, Aussehen der Landmarken, Palettenzone, Zielzone.

Das Konzept kommt ohne feste Streckenwerte aus:

- Die Lage jeder Zone misst der Bot selbst bei der Erkundung (Zonenmodell).
- Die Zonenmitte ergibt sich aus den Zonenlinien.
- Die Zielzone wird vor dem Start eingegeben.
- In den YAMLs stehen nur Werte, die vor Ort eingestellt werden: Ablauf in
  `Parameter/Switch_Control.yaml`, Detektoren in `Parameter/Storage.yaml`, Regler in
  `Parameter/Control_driving_PID_Wide.yaml`. Alles andere sind Konstanten im Code.

## Ablauf

1. **Start** mit `scripts/tb/launchers/storage.sh`, Fahrfreigabe per Leertaste, dann Button
   STORAGE STARTEN im Debugbild. Bis dahin steht der Bot (Modus `Stop`). Er steht auf der Spur
   vor dem Lagerbereich, die Zonen liegen rechts (Figure 5).
2. **Erkundung**: Spurfolge. `storage_zone_node` meldet Querlinien neben der Spur;
   switch_control buendelt sie entlang der Spur zum **Zonenmodell**
   (siehe unten). Erreicht der Bot die Mitte einer Zone, faehrt er genau dorthin, dreht zur
   Zone, liest Schild, Fuellung und Zonentiefe (vordere und hintere Querlinie), speichert die Pose
   und dreht zurueck. Belegt ist die Zone, wenn genug Bilder zwischen ihren Linien genug
   Flaeche zeigen, die weder dunkler Boden noch weisse Linie ist: „Palette in Zone X gefunden“.
   Ohne Modell (weniger als zwei Linien) dreht er `fallback_step` hinter der letzten Querlinie
   zur Seite und liest nur das Schild; ohne lesbares Schild wird die Stelle verworfen.
   Die Erkundung endet, sobald Paletten- und Zielzone bekannt sind, nach drei Zonen oder nach
   `explore_max_distance`. Ohne gesehene Palette: „keine Palette gefunden“, Abbruch (nicht raten).
3. **Zur Palettenzone** per Odometrie auf die gespeicherte Pose (vor- oder rueckwaerts), zur
   Zone drehen, 1 s anhalten (Positionierung, Wertung). Dabei die **Vorderkante der Palette**
   vermessen: Abstand und seitliche Lage (siehe unten). Den Winkel braucht es nicht: die Palette
   liegt laut Reglement gerade, der Kurs in die Zone kommt aus den Zonenlinien bzw. der Spur.
4. **Ausrichten** (`Maneuver`): ab 1,5 cm Versatz auf der Stelle parallel zur Spur drehen, um den
   Versatz entlang der Spur vor- oder zurueckfahren, auf der Stelle zurueck zur Zone drehen;
   sonst nur den Kurs in die Zone setzen. Quer zur Spur faehrt er nie rueckwaerts (sonst verliesse
   er die Spur).
5. **Andocken** (`Maneuver`, geradeaus mit gehaltenem Kurs): zuegig bis 10 cm vor den erwarteten
   Kontakt, dann mit 3 cm/s, hoechstens 4 cm darueber hinaus (sonst neu ansetzen). Kontakt, sobald das Bild ueber dem Magneten stillsteht, obwohl der Bot vorfaehrt: der
   Streifen ist buendig mit der Stuetze, der Magnet liegt an und schiebt die Palette auf der
   Stuetze vor sich her (`pallet_node`, `/detect/pallet_still`). Kein Abstandswert; der Bot
   muss nur (Odometrie) bis auf 5 cm an den erwarteten Kontakt heran sein. Neu ansetzen und
   Herausfahren: gerade zurueck auf den Spurpunkt, von dem aus angedockt wurde.
6. **Greifen**: Magnet an, warten, 3 cm zuruecksetzen. Bleibt das Bild ueber dem Magneten gleich
   (Korrelation ab `GRIP_MIN` 0,8, Konstante), haengt die Palette: „Palette erfolgreich aufgenommen“.
7. **Transport** („lege Palette in die Zone X“): rueckwaerts auf die gespeicherte Pose auf der
   Spur, zur Zielzone drehen, auf der Spur bis vor die Zielzone, zur Zone drehen. Fremde Zonen
   werden nie betreten, auch wenn eine dazwischen liegt.
8. **Ablegen**: so tief in die Zielzone wie beim Aufnehmen in die Palettenzone (Spurpunkt bis
   Kontakt; die Palette lag dort in der Zonenmitte), bei gemessenen Zonentiefen um deren
   Unterschied verschoben. Palettenmasse braucht es dafuer nicht.
   Magnet aus, warten: „Palette abgelegt“. Die Ausrichtung der Palette entspricht der beim
   Aufnehmen, weil der Bot wieder frontal in die Zone schaut.
9. **Verlassen**: rueckwaerts auf die Spur, anhalten.

Die Rueckmeldungen stehen im Log und im Debugbild (`/control/storage_info`).

### Palette in der Vogelperspektive

`storage_zone_node` sucht zwischen den Seitenlinien der Zone (aus dem Zonenmodell) alles, was
heller oder farbiger als der Boden und nicht weiss ist (dunkler zaehlt nicht: Schatten). Den
Boden lernt er beim Erkunden selbst aus der Spurmitte vor dem Bot (Helligkeit und Farbe mit
Streuung), es gibt keinen Schwellwert. Weiss und CLAHE kommen von `detect_lane`
(`Follow_Lane_ROI_LAB.yaml`, live ueber `/parameter_events`), sie gibt es nur einmal; ebenso in
`storage_sign_node` fuer die Ziffer. Der eigene Magnet (bis 11 cm vor der Achse, +-5 cm) zaehlt
nie als Palette.

- **Fuellung** (beim Lesen vor jeder Zone): Anteil davon zwischen Vorder- und Hinterkante der
  Zone. Ab 10 % zaehlt das Bild als belegt, ab der Haelfte der Bilder die Zone.
- **Vorderkante**: je Spalte die naechste solche Stelle. Das ist die Bodenkante der Stuetze,
  in der Vogelperspektive also unverzerrt und metrisch. Eine Gerade durch diese Punkte gibt
  Abstand, seitliche Lage und Winkel (`/detect/pallet`). Punkte an den Seiten der Stuetze
  fallen als Ausreisser heraus.
- Annahme: Der Metallstreifen liegt parallel zur Vorderkante der Stuetze **[V]**.
- Im Debugbild `/detect/zone_debug/compressed`: Palette rot getoent, Zonengrenzen gelb,
  Kantenpunkte lila und der Rahmen des Metallstreifens (15 cm) an der gemessenen Lage tuerkis,
  dazu Fuellung, Abstand, seitliche Lage und Winkel.

### Griff pruefen

`pallet_node` nimmt beim Greifen das Bild direkt ueber dem Magneten als Vorlage und vergleicht
danach jedes Bild damit (`/detect/pallet_grip`, Korrelation). Haengt die Palette, bewegt sie sich
mit dem Bot und das Bild bleibt gleich; bleibt sie liegen, wird sie beim Zuruecksetzen kleiner.
Ob es reicht, entscheidet switch_control mit der Konstante `GRIP_MIN`, auch waehrend des Tragens.
Die Lage des Magneten im Bild (`MAGNET_X`, `MAGNET_TOP`) ist Bot-Mechanik und fest eingetragen
(gemessen 10.11.); neu nur nach einem Umbau von Kamera oder Magnet. Aus der Geometrie laesst sie
sich nicht rechnen: Der Magnet sitzt 7,5 cm hoch, seine Lage vor der Achse kennt man nur aus der
BEV, und dort erscheint er wegen der Hoehe viel weiter vorn.

### Zonenmodell

- Linien L0..L3 im gleichen Abstand; Zone k liegt zwischen L(k-1) und L(k).
- Nur aus den bestaetigten Querlinien (je drei Bilder), ohne Streckenmasse: Jede Querlinie darf
  L0 sein, jede spaetere mit 1-3 Zonen dazwischen gibt die Breite vor. Es gewinnt das Raster,
  das die meisten Linien erklaert. Dagegen zaehlen Linien innerhalb des Rasters, die nicht
  passen (Schatten, Palette, Markierungen), und fehlende Linien, wo der Bot schon hingesehen
  hat. Eine Zone ist mindestens so breit wie der Bot (`BOT_WIDTH`).
- Keine Strichel-Erkennung mehr (bis 10.11.): Sie meldete bei langen Strichen weitere Anfaenge
  und Enden mitten in der Strichelung, das Raster wurde zu schmal und jede Zone um eine
  verschoben. Mit den Laeufen vom 10.11. getestet: alle Raster 0,27-0,30 m.
- Die Ziffer einer Zone kommt aus `zone_order` nach ihrem Platz (Standard `1,2,3` in
  Fahrtrichtung). Das Schild dient nur zur Kontrolle; ein Widerspruch steht als Warnung im Log.
  Mit leerem `zone_order` entscheidet das Schild.
- Gezeichnet im Debugbild `/detect/zone_debug/compressed`: gemessene Linien gruen, ergaenzte
  orange, Ziffer je Zone. Mit
  Zuschauer laeuft die Erkennung auch ohne Storage-Ablauf, damit sie sich einstellen laesst.

### Zustandsdiagramm

Unterzustaende (`StorageState` in `common.py`, Ablauf in `storage_sequence.py`); in Klammern
der `ControlType` (Fahrart), den switch_control dabei auf `/control/mode` sendet.

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> ERKUNDEN : Button STORAGE STARTEN

    state "ERKUNDEN (Lane)" as ERKUNDEN
    state "ZUR_ZONE_DREHEN (Maneuver)" as ZUR_ZONE_DREHEN
    state "SCHILD_LESEN (Stop)" as SCHILD_LESEN
    state "ZURUECKDREHEN (Maneuver)" as ZURUECKDREHEN
    state "ZUR_PALETTENZONE (Maneuver)" as ZUR_PALETTENZONE
    state "POSITIONIERT (Stop)" as POSITIONIERT
    state "AUSRICHTEN (Maneuver)" as AUSRICHTEN
    state "ANDOCKEN (Maneuver)" as ANDOCKEN
    state "GREIFEN (Stop, Magnet an)" as GREIFEN
    state "GRIFF_PRUEFEN (Maneuver)" as GRIFF_PRUEFEN
    state "NEU_ANSETZEN (Maneuver, Magnet aus)" as NEU_ANSETZEN
    state "HERAUSFAHREN (Maneuver)" as HERAUSFAHREN
    state "TRANSPORT (Maneuver)" as TRANSPORT
    state "ABLAGE_ANFAHREN (Maneuver)" as ABLAGE_ANFAHREN
    state "ABLEGEN (Stop, Magnet aus)" as ABLEGEN
    state "VERLASSEN (Maneuver)" as VERLASSEN
    state "FERTIG (Stop)" as FERTIG
    state "FEHLER (Stop, Magnet aus)" as FEHLER

    ERKUNDEN --> ZUR_ZONE_DREHEN : Zonenmitte erreicht oder Rueckfall
    ZUR_ZONE_DREHEN --> SCHILD_LESEN : auf Zonenmitte, zur Zone gedreht
    SCHILD_LESEN --> ZURUECKDREHEN : Abstimmung fertig, Pose gespeichert
    ZURUECKDREHEN --> ERKUNDEN : Paletten- oder Zielzone noch unbekannt
    ZURUECKDREHEN --> ZUR_PALETTENZONE : beide bekannt
    ERKUNDEN --> ZUR_PALETTENZONE : explore_max_distance, beide bekannt
    ZUR_PALETTENZONE --> POSITIONIERT : vor der Zone, zur Zone gedreht
    POSITIONIERT --> AUSRICHTEN : Haltezeit vorbei, Vorderkante vermessen
    POSITIONIERT --> NEU_ANSETZEN : keine Vorderkante gefunden
    AUSRICHTEN --> ANDOCKEN : Versatz ausgeglichen, Kurs in die Zone
    ANDOCKEN --> GREIFEN : Bild ueber dem Magneten steht still (Kontakt)
    ANDOCKEN --> NEU_ANSETZEN : kein Kontakt bis 4 cm hinter dem erwarteten, Zeit
    GREIFEN --> GRIFF_PRUEFEN : Wartezeit vorbei
    GRIFF_PRUEFEN --> HERAUSFAHREN : Palette kommt mit
    GRIFF_PRUEFEN --> NEU_ANSETZEN : Bild ueber dem Magneten aendert sich
    NEU_ANSETZEN --> POSITIONIERT : zurueck auf der Spur, ausgerichtet
    HERAUSFAHREN --> TRANSPORT : auf der Spur
    TRANSPORT --> ABLAGE_ANFAHREN : vor der Zielzone, zur Zone gedreht
    ABLAGE_ANFAHREN --> ABLEGEN : Zonenmitte erreicht
    ABLEGEN --> VERLASSEN : Wartezeit vorbei
    VERLASSEN --> FERTIG : auf der Spur

    ERKUNDEN --> FEHLER : Zone nicht gefunden
    NEU_ANSETZEN --> FEHLER : dritter Fehlversuch
    TRANSPORT --> FEHLER : Palette nicht mehr am Magneten
    FEHLER --> ERKUNDEN : Button STORAGE STARTEN
    FERTIG --> [*]
```

Nicht eingezeichnet:

- STORAGE ABBRECHEN fuehrt aus jedem Zustand zu IDLE (Magnet aus, Modus `initial_mode`).
- Jeder `Maneuver`-Zustand endet nach 30 s in FEHLER.
- „Palette nicht mehr am Magneten“ gilt auch in HERAUSFAHREN und ABLAGE_ANFAHREN.

## Nodes und Topics

| Node / Modul | neu/geaendert | Ein | Aus | Aufgabe |
|--------------|---------------|-----|-----|---------|
| `switch_control` + `storage_sequence.py` | geaendert / neu | `/control/storage_request`, `/detect/zone_boundaries`, `/detect/zone_cross_lines`, `/detect/storage_sign`, `/detect/pallet`, `/detect/pallet_grip`, `/control/maneuver_status`, TF | `/control/mode`, `/control/storage_state`, `/control/storage_info`, `/control/storage_zones`, `/control/maneuver`, `/lokal/magnet_control` | Einziger Entscheider: Unterzustand, Zonenmodell und Zonenposen in `odom`, Abstimmung ueber Schilder und Palette, Ziele fuer `Maneuver`, Magnet an/aus, Rueckmeldungen. Kein Obstacle und kein Tunnel waehrend Storage |
| `control_driving` | geaendert | `/control/mode`, `/detect/lane`, `/nav2/cmd_vel`, `/control/maneuver`, TF | `/control/cmd_vel`, `/control/maneuver_status` | Einzige Quelle fuer Fahrbefehle, neu: Regler fuer `Maneuver` (auch Andocken; frueher eigener Modus `Dock` mit Kamera-Lenkung, entfernt) |
| `storage_zone_node` | neu | `/lokal/bev`, `/control/storage_state`, `/control/storage_zones`, TF | `/detect/zone_boundaries`, `/detect/zone_cross_lines`, `/detect/pallet` | Weisse Querlinien per HoughLinesP: neben der Spur (ERKUNDEN) bzw. vor dem Bot (SCHILD_LESEN); Fuellung der Zone und Vorderkante der Palette; zeichnet Zonenmodell und Palette ins Debugbild |
| `storage_sign_node` | neu | `/lokal/raw_image`, `/control/storage_state` | `/detect/storage_sign` | Blau-Maske (LAB), Ellipse und weisser Inhalt pruefen, Ziffer mit Vorlagen korrelieren |
| `pallet_node` | neu | `/lokal/raw_image`, `/control/storage_state` | `/detect/pallet_grip` | Griff pruefen: Bild ueber dem Magneten gegen die Vorlage vom Greifen |
| `magnet_controll` | registriert | `/lokal/magnet_control` | `/lokal/magnet_status` | GPIO 17, laeuft **auf dem Bot** (`ros2 run Follow_Lane magnet_controll`) |
| `debug_view` | geaendert | wie bisher, dazu `/control/storage_state`, `/control/storage_info` | `/control/storage_request` | Button STORAGE STARTEN/ABBRECHEN, Zustand, Zonen, Fehlergrund |

Die Detektoren arbeiten nur in den Zustaenden, die sie brauchen. `pallet_node` und
`storage_zone_node` arbeiten zusaetzlich, wenn jemand ihr Debugbild ansieht,
`storage_sign_node` mit `always_on`, damit sich Werte und Vorlagen einstellen lassen.

**Neue Nachrichten** in `follow_lane_msg`: `StorageSign` (Ziffer, Sicherheit, Lage im Bild,
Quelle), `PalletDetection` (Fuellung der Zone; Vorderkante: Abstand, seitlich, Winkel), `Maneuver`
(id, ROTATE/STRAIGHT/PATH, Zielwinkel, Strecke, Pfad, rueckwaerts, Hoechstgeschwindigkeit),
`StorageZones` (Zonenmodell fuer die Anzeige: Linien, gemessen/ergaenzt, Ziffern).
Zonenlinien laufen als `geometry_msgs/PoseArray` in `odom`.

**Neue Fahrarten** in `common.py`:

| `ControlType` | Fahrbefehl (control_driving) |
|---------------|------------------------------|
| `Maneuver` (9) | Grundbewegung aus `/control/maneuver` in `odom`: auf Winkel drehen, Strecke fahren, Punkte anfahren (vor- oder rueckwaerts). Meldet die id auf `/control/maneuver_status` |
| `Lane`, `Stop` (vorhanden) | Spurfolge bei der Erkundung, Nullbefehl beim Lesen, Greifen und Ablegen |

`Maneuver` laeuft wie die Nav2-Befehle durch die Radgrenze (`MAX_WHEEL_VEL`).

```
storage_sign_node -------------------------/detect/storage_sign------------.
storage_zone_node --/detect/zone_boundaries, _dash_start/_end, _cross_lines,-+
                    /detect/pallet                                            |
pallet_node --/detect/pallet_grip---------------------------------------------+--> switch_control
debug_view --/control/storage_request-----------------------------------------'       |
                                                                     /control/mode,   |
                                                                  /control/maneuver   v
detect_lane_nav2 --/detect/lane--> control_driving <--/detect/pallet-- storage_zone_node
                                       |
                                       '--/control/cmd_vel--> notaus --/cmd_vel--> Bot
switch_control --/lokal/magnet_control--> magnet_controll (auf dem Bot, GPIO 17)
```

**Start:** `storage.launch.py` (ueber `storage.sh`) startet `follow_lane.launch.py` (mit
`odom_filter`, der die TF odom -> base_footprint liefert) mit `Parameter/Storage.yaml` und die vier
Storage-Nodes, ohne Nav2. `Storage.yaml` setzt `initial_mode: Stop` fuer switch_control und
ueberschreibt damit `initial_mode: Lane` aus `Switch_Control.yaml`. Kein Node hat Code-Defaults.
Die Werte von switch_control (auch die des Storage-Ablaufs) stehen in `Switch_Control.yaml`,
die von control_driving in `Control_driving_PID_Wide.yaml`, damit auch `follow_lane.launch.py`
und `nav2_obstacle.launch.py` ohne `Storage.yaml` starten.

## Entscheidungen

| Thema | Entscheidung | Begruendung |
|-------|--------------|-------------|
| Sensoren | Nur Kamera und Odometrie, kein LiDAR | Palette, Stuetze und Schilder liegen unter der LiDAR-Ebene |
| Zustaende | `ControlType` je Fahrart (`Maneuver`, dazu `Lane`, `Stop`); der Ablaufschritt ist ein Unterzustand nur in switch_control (Modul `storage_sequence.py`) | control_driving bleibt bei „Modus -> Regler“, nur zwei neue Werte, der Ablauf liegt an einer Stelle (Architekturregel) |
| Magnet | schaltet switch_control | An/aus ist eine Entscheidung, kein Fahrbefehl |
| Start | Button STORAGE STARTEN (sendet `/control/storage_request`) | Start per ROS-Befehl von Hand ist erlaubt; gleiches Muster wie beim Tunnel |
| Lagebild | Erst Erkundung, Posen in `odom` speichern, danach Palette holen | Liefert die Erkennung (Bonus) und die Ziele fuer den Transport; unabhaengig von der Zonengroesse |
| Zone erkennen | Zonenmodell aus Querlinien (gleichmaessigstes Raster), Ziffer aus der Reihenfolge; Rueckfall ohne Modell: zur Seite drehen und Schild lesen | Keine festen Abstaende; eine uebersehene Linie verschiebt die Zaehlung nicht |
| Schilder | Blau-Maske (LAB) + Ellipse + Ziffern-Vorlagen; parallel ein trainiertes Modell | Aussehen bekannt; die Kreisform trennt das Schild von der blauen Schranke (Balken, auch beim Oeffnen) |
| Zusammenfuehrung | Beide Detektoren stimmen ueber mehrere Bilder ab, waehrend der Bot steht; jeder per Parameter abschaltbar | Robust; faellt einer aus, arbeitet der andere weiter |
| Manuelle Eingabe | `target_zone` (Jury), `zone_order` (Ziffern in Fahrtrichtung) | Die Jury bestimmt die Zielzone; die Palettenzone erkennt der Bot selbst (Wertung) |
| Zone leer? | Zwischen den Zonenlinien alles dunkel; sonst belegt | Palette hat keine Markierung; jede Farbe ist heller als der schwarze Boden |
| Andocken | Seitlichen Versatz der Stuetzenkante in der Vogelperspektive messen, entlang der Spur ausgleichen (drehen auf der Stelle, parallel zur Spur fahren), Kurs aus den Zonenlinien, dann nur vorwaerts geradeaus (`Maneuver`) bis das Bild ueber dem Magneten stillsteht | Die Palette liegt laut Reglement gerade; quer zur Spur rueckwaerts verliesse der Bot die Spur; kein Abstandswert noetig |
| Griff pruefen | Kurz zuruecksetzen, Bild ueber dem Magneten mit der Vorlage vom Greifen vergleichen | Kein eigener Sensor, kein Motorstrom (`turtlebot3_node` liest ihn nicht aus) |
| Entscheiden | Detektoren liefern nur Messwerte (Fuellung, Vorderkante, Korrelation); belegt, Kontakt und Griff entscheidet switch_control | Architekturregel: ein Entscheider |
| Route | Rueckwaerts raus, zur Zielzone drehen, an der Spur entlang, zur Zielzone drehen | Fremde Zonen werden nie betreten |
| Ausrichtung | Muss nur beim Ablegen stimmen | Seitliches Versetzen ohne Drehen geht mit Differentialantrieb nicht |
| Maneuver | Eigener Regler in control_driving, kein Nav2 | Kurze, einfache Wege; exaktes Drehen auf der Stelle; Costmaps und LiDAR helfen hier nicht |
| Ablegen | Tiefe aus den Zonenlinien der Erkundung, Magnet aus | Magnethoehe fest, Magnet nur an/aus; mit Palette vor der Kamera sind die Linien der Zielzone womoeglich verdeckt |
| Verlassen | Rueckwaerts aus der Zielzone auf die Spur, anhalten | Kurz, ohne Drehung |
| Fehler | Andocken/Greifen bis zum dritten Versuch, danach FEHLER; Palette verloren -> FEHLER | Haengt nie in einer Schleife; eine Palette am Boden ist mit fester Magnethoehe nicht mehr greifbar |

## Vor Ort einstellen

1. **Boden und Weiss**: nichts eigenes. Der Boden wird beim Erkunden gelernt (Debugbild
   `/detect/zone_debug/compressed`: „Boden L ..“, eine leere Zone darf nicht rot sein). Weiss
   wird fuer die Spur in `detect_lane` eingestellt und gilt automatisch auch hier.
2. **Kontakt**: nichts einzustellen (Bot-Geometrie: Magnetende 7,5 cm vor der Achse, Magnetmitte
   7,5 cm hoch, in `common.py`).
3. **Magnet** (`pallet_node`): nichts einzustellen. Nach einem Umbau von Kamera oder Magnet
   in `/detect/pallet_debug/compressed` pruefen, ob das gelbe Rechteck direkt ueber dem Magneten
   liegt, sonst `MAGNET_X`/`MAGNET_TOP` in `pallet_node.py` anpassen.
4. **Schilder** (`storage_sign_node`): Bot vor jedes Schild stellen, `always_on: true`,
   `/detect/storage_sign_debug/compressed` ansehen, Blau-Werte pruefen, dann je Schild
   `ros2 param set /storage_sign_node capture_digit <1-3>`. Die Vorlagen liegen in
   `~/.ros/urmc_storage_templates` und gelten auch nach einem Neustart.
5. **Zonenlinien** (`storage_zone_node`): `/detect/zone_debug/compressed` beim Vorbeifahren;
   `side_near_m` so, dass die gestrichelte Linie gerade nicht mehr im Streifen liegt.
6. **Zielzone**: `ros2 param set /switch_control target_zone <1-3>` vor dem Start.

## Offene Punkte

- **Nachbau der Palette:** fehlt; ohne ihn lassen sich Andocken und Griff nicht pruefen.
- **Orga:** Groesse der Schilder; Aussehen der Landmarken; ob „exit the storage area“ mehr
  verlangt als die Zone zu verlassen.
- **Am Bot zu pruefen:** Magnetlage im Bild, Kontakt per Stillstand, Genauigkeit von `Maneuver`,
  Strichelung und Zonenmodell, ob die Vorderkante der Stuetze bis kurz vor dem Kontakt im
  Bild bleibt, ob der Streifen parallel zu ihr liegt.
- **Magnet auf dem Bot:** `magnet_controll` braucht das Paket `Follow_Lane` auf dem Bot
  (gebaut, mit `python3-gpiozero`). `scripts/tb/bot/magnet_node.py` hoert dagegen auf
  `/magnet/cmd`; sein Kommentar sagt, `bot-up.sh` starte ihn, `bot-up.sh` tut das nicht.
- **Ablegen ohne Stuetze:** Die Palette faellt von Stuetzenhoehe auf den Boden **[V]**.
- **BEV seitlich:** Reicht der Bereich der BEV nach rechts bis zu den Zonengrenzen? Die BEV
  reicht +-0,40 m, gesucht wird ab `side_near_m` 0,14 m.
- **Drehen auf der Spur mit Palette:** Das Heck schwenkt aus; es darf keine Zone streifen
  **[V]**. Am Bot messen.
- **Sicht beim Transport:** Die Palette verdeckt womoeglich die nahen Spurlinien **[V]**,
  deshalb faehrt der Transport per `Maneuver` statt `Lane`.
- **Spur-Tracking nach Drehungen:** `detect_lane_nav2` verfolgt die Linien mit Anker
  (`LINE_MAX_JUMP_CM`, `LINE_HOLD_TIME_S`) und laeuft auch in `Maneuver` weiter. Ob die Spurfolge
  nach `ZURUECKDREHEN` sofort wieder die richtigen Linien findet **[V]**, am Bot pruefen.
- **Geschwindigkeiten mit Last:** `carry_max_vel` 0,05 m/s, am Bot festlegen.
- **Odometrie:** Genauigkeit von Drehung und Strecke ueber ca. 1 m am Bot messen.
- **Ziffern per YOLO:** Das Modell kennt noch keine Ziffern (Trainingsnotizen); der fruehere
  `storage_sign_model_node` (leeres Geruest) ist entfernt, Ziffern liest `storage_sign_node`.

## Testplan

Real auf dem nachgebauten Parcours. Dafuer wird der Lagerbereich ergaenzt: drei Zonen aus
Klebeband mit gestrichelter Linie zur Spur, drei Schilder (blauer Kreis, weisse Ziffer), ein
Nachbau der Palette (Rahmen mit Metallstreifen 15 x 5 cm auf einem Kasten). Zonen absichtlich
ungleich breit (z. B. 25 / 30 / 35 cm) und die Strichelung verschieden lang, damit nichts an
festen Massen haengt.

1. **Bauen:** `colcon build` fuer `follow_lane_msg` und `Follow_Lane` (neue Nachrichten).
2. **Magnet:** `ros2 run Follow_Lane magnet_controll` auf dem Bot,
   `ros2 topic pub --once /lokal/magnet_control std_msgs/msg/Bool '{data: true}'`. Haelt er den
   Streifen, haengt die Palette sauber, faellt sie beim Ausschalten ab?
3. **Einstellen** wie oben (Boden, Kontakt, Magnetlage, Vorlagen).
4. **Schilder:** Erkennung bei verschiedenen Abstaenden, Winkeln und Licht; die blaue Schranke
   daneben darf nicht als Schild gelten.
5. **Zonenlinien:** Werden die Querlinien beim Fahren gefunden (Log „Zonengrenze bestaetigt“)? Stimmt das Zonenmodell
   im Debugbild, auch wenn eine Trennlinie abgeklebt ist?
6. **Maneuver:** 90°-Drehungen, Strecken vor/zurueck und Punkte anfahren, mit und ohne Palette.
   Einzeln testbar: `/control/mode` und `/control/maneuver` von Hand senden, ohne switch_control.
7. **Andocken und Griff pruefen:** Palette seitlich versetzt (gerade, Reglement) in die Zone
   stellen; der Bot muss den Versatz entlang der Spur ausgleichen und gerade andocken. Fehlgriff (Magnet aus) muss NEU_ANSETZEN
   ausloesen.
8. **Gesamtablauf** mit der Palette in jeder Zone und jeder Zielzone, auch mit einer Zone
   zwischen Start und Ziel; Abbrechen per Button, Fehlerfaelle.

## Quellen

- Reglement 2026: `R_glement_Robohub_v_2026.pdf`, Abschnitt 3 und 4.2.2, Figure 5, Table 1.
- Antworten der Organisatoren: Mail an alle Teams (siehe `Reglement_Ergaenzungen_Orga.md` auf
  `main`) und Mail mit Fotos der Palette.
- GalapagosGang URMRC 2025: <https://github.com/scti1057/GalapagosGang_URRMC2025>
  (`src/galapagos_regelt/galapagos_regelt/paletting_node.py`, `challenge4_palleting.launch.py`,
  `src/galapagos_checked_yolo/`, `2025_URRMC_Reglement_En.pdf`). Uebernommen: Grundidee der Route
  (zuruecksetzen, 90° drehen, an der Spur entlang, zurueckdrehen). Nicht uebernommen: feste
  Ablaufkette, Pixel-Offsets, direktes `cmd_vel`, YOLO-Gewichte.
- ZoZo-Campo/URMC (2026): <https://github.com/ZoZo-Campo/URMC>, `src/urmc_mission_05/` —
  Zustandsliste mit Griff- und Ablagebestaetigung, Wahrnehmung dort noch nicht umgesetzt.
- Offizielle URMRC-Simulation: <https://github.com/URMRC/Turtlebot-simulation-Upper-Rhine-Robotic-Challenge-ROS2-humble-25>
  — kein Modell des Lagerbereichs.
- TurtleBot3 Burger URDF (`turtlebot3_burger.urdf`, `scan_joint` 0,172 m + `base_joint` 0,010 m):
  Hoehe der LiDAR-Ebene.
- `turtlebot3_node/src/sensors/joint_state.cpp`: Motorstrom auskommentiert, kein Effort.
- Seeed Grove Electromagnet: <https://wiki.seeedstudio.com/Grove-Electromagnet/> (5 V, 400 mA, bis 1 kg).
- Schilderkennung bei AutoRace (Vorlagenabgleich, YOLOv8 + SIFT):
  Twieg/Menghani, ICAIIT 2023, <https://repo.bibliothek.uni-halle.de/bitstream/1981185920/114950/1/2_3_ICAIIT_Paper_2023%282%29_Twieg_26.pdf>
