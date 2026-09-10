"""Parameterdateien fuer Launch-Dateien einlesen.

Liegt bewusst im Paket und nicht in einer der Launch-Dateien: sonst muesste
jede weitere Launch-Datei die Funktion kopieren, und kopierte Defaults sind in
diesem Repo schon einmal auseinandergelaufen (der Nav2-Zweig fuhr mit einer
anderen Kameradatei als der Spur-Zweig).

Importiert bewusst kein rclpy -- die Funktion laeuft im Launch-Prozess.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

# Default-Parameterdateien -- die eine Stelle, an der sie stehen. Vorher standen
# sie in mehreren Launch-Dateien und liefen auseinander.
DEFAULT_DETECT_PARAMS = 'Follow_Lane_ROI_PI_CAM2_V3.yaml'
DEFAULT_CONTROL_PARAMS = 'Control_driving_PID.yaml'
DEFAULT_SWITCH_PARAMS = 'Switch_Control.yaml'
# Abweichende switch_control-Konfiguration fuer zone_avoid.launch.py: Mux an,
# Obstacle-Umschaltung ueber avoid_node statt ueber den eigenen LiDAR-Kegel.
DEFAULT_SWITCH_ZONE_PARAMS = 'Switch_Control_Zone.yaml'
DEFAULT_AVOID_PARAMS = 'Avoid_Node.yaml'
DEFAULT_SAFETY_PARAMS = 'Safety_Node.yaml'


def param_path(filename):
    """Vollstaendiger Pfad einer Datei im installierten Parameter-Verzeichnis."""
    return os.path.join(
        get_package_share_directory('Follow_Lane'), 'Parameter', filename)


def load_flat_params(path):
    """Liest eine flache Parameterdatei (name: wert) als Dictionary.

    Bewusst NICHT ueber --params-file: das verlangt die Struktur
    "<node>: ros__parameters:", waehrend rqt_reconfigure flaches YAML schreibt und
    liest. Da Node(parameters=[...]) auch ein Dictionary akzeptiert, kommen wir mit
    einem einzigen Format aus und die Datei-Knoepfe in rqt bleiben benutzbar.
    """
    if not os.path.isfile(path):
        print(f'[launch] Parameterdatei nicht gefunden, nutze Code-Defaults: {path}')
        return {}

    with open(path) as f:
        data = yaml.safe_load(f) or {}

    # Aeltere Dateien lagen verschachtelt vor -- der Vollstaendigkeit halber mitnehmen.
    if len(data) == 1 and isinstance(next(iter(data.values())), dict) \
            and 'ros__parameters' in next(iter(data.values())):
        data = next(iter(data.values()))['ros__parameters']

    # use_sim_time gehoert nicht in eine Streckenkonfiguration.
    return {k: v for k, v in data.items() if k != 'use_sim_time'}
