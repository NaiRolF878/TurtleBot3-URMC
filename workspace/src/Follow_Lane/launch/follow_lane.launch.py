import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAM_DIR = os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter')


def load_flat_params(path):
    """Liest eine flache Parameterdatei (name: wert).

    Bewusst NICHT ueber --params-file: das verlangt die Struktur
    "<node>: ros__parameters:", waehrend rqt_reconfigure flaches YAML schreibt und
    liest. Da Node(parameters=[...]) auch ein Dictionary akzeptiert, kommen wir mit
    einem einzigen Format aus und die Datei-Knoepfe in rqt bleiben benutzbar.
    """
    if not os.path.isfile(path):
        print(f'[follow_lane.launch] Parameterdatei nicht gefunden, nutze Code-Defaults: {path}')
        return {}

    with open(path) as f:
        data = yaml.safe_load(f) or {}

    # Aeltere Dateien lagen verschachtelt vor -- der Vollstaendigkeit halber mitnehmen.
    if len(data) == 1 and isinstance(next(iter(data.values())), dict) \
            and 'ros__parameters' in next(iter(data.values())):
        data = next(iter(data.values()))['ros__parameters']

    # use_sim_time gehoert nicht in eine Streckenkonfiguration.
    return {k: v for k, v in data.items() if k != 'use_sim_time'}


def launch_setup(context, *args, **kwargs):
    detect_params  = load_flat_params(LaunchConfiguration('detect_params').perform(context))
    control_params = load_flat_params(LaunchConfiguration('control_params').perform(context))

    return [
        Node(package='Follow_Lane', executable='ingress_node',    name='ingress_node'),
        Node(
            package='Follow_Lane', executable='detect_lane', name='detect_lane',
            parameters=[detect_params],
        ),
        Node(
            package='Follow_Lane', executable='control_driving', name='control_driving',
            parameters=[control_params],
            # control_driving publiziert auf /control/cmd_vel. KEIN Remapping auf /cmd_vel mehr:
            # dazwischen sitzt der Not-Aus (notaus.py), der nur nach Freigabe (Leertaste / rqt-
            # Parameter drive / /notaus/set) durchreicht. Er laeuft NICHT in diesem Launch-File,
            # weil er ein Terminal fuer die Leertaste braucht -- launchers/follow_lane.sh startet
            # ihn im Vordergrund. Ohne notaus faehrt der Bot nicht (sicherer Zustand).
        ),
        Node(package='Follow_Lane', executable='switch_control',  name='switch_control'),
        Node(package='Follow_Lane', executable='debug_view',      name='debug_view'),
        Node(package='Follow_Lane', executable='avoid_obstacle',  name='avoid_obstacle'),
        # safety_stop bewusst NICHT gestartet (17.08.2026): loeste beim Testen des
        # Farbfehlers/detect_lane wegen der 0.5s-Timeout-Schwelle staendig unnoetig aus.
        # Datei bleibt im Repo, kann jederzeit wieder als Node(...) eingehaengt werden.
        # WICHTIG: ohne diesen Node gibt es KEINEN automatischen Stopp mehr, wenn
        # control_driving abstuerzt oder das WLAN abreisst -- manueller Not-Aus siehe
        # docs/MEMORY.md ("Not-Aus von Hand").
    ]


def generate_launch_description():
    return LaunchDescription([
        # Ueberschreibbar, z.B. fuer eine zweite Streckenkonfiguration:
        #   ros2 launch Follow_Lane follow_lane.launch.py detect_params:=/pfad/zur/datei.yaml
        DeclareLaunchArgument('detect_params',
            default_value=os.path.join(PARAM_DIR, 'Follow_Lane_ROI_PI_CAM2_V3.yaml')),
        DeclareLaunchArgument('control_params',
            default_value=os.path.join(PARAM_DIR, 'Control_driving_PID.yaml')),
        # OpaqueFunction, damit die Pfade aufgeloest sind, bevor die Dateien gelesen werden.
        OpaqueFunction(function=launch_setup),
    ])
