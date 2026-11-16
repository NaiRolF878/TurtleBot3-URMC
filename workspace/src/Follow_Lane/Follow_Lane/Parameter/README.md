# Parameterdateien

| Ordner | Inhalt | Anfassen? |
|---|---|---|
| `follow_lane/` | Spurerkennung (`Follow_Lane_ROI_LAB.yaml`, LAB-Farben `Lab_Farben.yaml`), PID (`Control_driving_PID_Wide.yaml`), Modusumschaltung (`Switch_Control.yaml`), Kreuzung (`Intersection.yaml`) | ja, Tuning vor Ort |
| `obstacle_tunnel/` | Nav2 und Tunnel (`nav2_params.yaml`), Spurwände für die Costmap (`Lane_Costmap.yaml`) | ja |
| `storage/` | Storage-Aufgabe (`Storage.yaml`) | ja |
| `nicht_anfassen/` | Kalibrierung: Kamera (`camera_info/`), Vogelperspektive (`bev.yaml`), LiDAR (`Scan_Correct.yaml`) | **nein**, nur bei neuer Kalibrierung (Skripte in `scripts/tb/` schreiben dorthin) |
| `deprecated/` | alte Dateien, nicht mehr geladen | nein |

Die Ordner gibt es nur hier im Quellbaum. `setup.py` installiert alles **flach** nach
`install/Follow_Lane/share/Follow_Lane/Parameter/` (Kamera nach `.../Parameter/camera_info/`), deshalb
brauchen Launch-Dateien und Nodes keine Ordnernamen. Folge: **Dateinamen müssen über alle Ordner
eindeutig sein.** Eine neue Datei braucht einmal `colcon build` (macht `tb` automatisch).
