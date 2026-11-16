# Veraltete Parameterdateien

Werden von keinem Launch und keinem Skript mehr geladen (Stand 2026-11-16). Nur zum Nachschlagen.

| Datei | Ersetzt durch |
|---|---|
| Follow_Lane_ROI.yaml, Follow_Lane_ROI_PI_CAM2*.yaml | Follow_Lane_ROI_LAB.yaml (ROI-Ecken entfallen, BEV aus camera_info/) |
| Control_driving_PID.yaml (kp 3,6) | Control_driving_PID_Wide.yaml |

Dieser Ordner wird nicht installiert (setup.py nimmt nur Parameter/*.yaml).
