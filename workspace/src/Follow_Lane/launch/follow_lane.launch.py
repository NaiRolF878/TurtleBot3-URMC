"""Spurverfolgung: Kamerakette, Moduswahl, Fahrbefehle und Debugbild.

control_driving sendet auf /control/cmd_vel. Zum Bot gelangt das erst ueber den
Not-Aus (notaus), der nur nach Freigabe durchreicht. Er laeuft nicht hier, weil er
ein Terminal fuer die Leertaste braucht: launchers/follow_lane.sh bzw.
nav2_obstacle.sh starten ihn im Vordergrund. Ohne notaus faehrt der Bot nicht.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAM_DIR = os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter')


def load_flat_params(path):
    """Liest eine Parameterdatei als Dictionary fuer Node(parameters=[...]).

    Nicht ueber --params-file: das verlangt "<node>: ros__parameters:", waehrend
    rqt_reconfigure flaches YAML (name: wert) schreibt und liest. So bleiben die
    Datei-Knoepfe in rqt benutzbar. Verschachtelte Dateien mit genau einem
    Node-Block (z.B. Switch_Control.yaml) werden ebenfalls gelesen.
    """
    if not os.path.isfile(path):
        print(f'[follow_lane.launch] Parameterdatei nicht gefunden, nur Code-Defaults '
              f'(control_driving hat keine und startet nicht): {path}')
        return {}

    with open(path) as f:
        data = yaml.safe_load(f) or {}

    if len(data) == 1 and isinstance(next(iter(data.values())), dict) \
            and 'ros__parameters' in next(iter(data.values())):
        data = next(iter(data.values()))['ros__parameters']

    # use_sim_time gehoert nicht in eine Streckenkonfiguration.
    return {k: v for k, v in data.items() if k != 'use_sim_time'}


def launch_setup(context, *args, **kwargs):
    detect_params = load_flat_params(LaunchConfiguration('detect_params').perform(context))
    control_params = load_flat_params(LaunchConfiguration('control_params').perform(context))
    switch_params = load_flat_params(LaunchConfiguration('switch_control_params').perform(context))
    # Obstacle nur mit avoid_node (nav2_obstacle.launch.py setzt obstacle:=true). Live
    # umschaltbar in rqt.
    switch_params['obstacle_enabled'] = \
        LaunchConfiguration('obstacle').perform(context).lower() == 'true'

    # Die *_nav2-Nodes pausieren im Tunnel. Sie laufen als ingress_node/detect_lane,
    # damit rqt, save-params.sh und ros2 param gleich funktionieren.
    return [
        # odom -> base_footprint ohne Radwert-Ausreisser; der Bot sendet die TF dafuer
        # nicht selbst (bot-up.sh, BOT_ODOM_TF=false).
        Node(package='Follow_Lane', executable='odom_filter', name='odom_filter'),
        Node(package='Follow_Lane', executable='ingress_node_nav2', name='ingress_node'),
        Node(
            package='Follow_Lane', executable='detect_lane_nav2', name='detect_lane',
            parameters=[detect_params],
        ),
        Node(
            package='Follow_Lane', executable='control_driving', name='control_driving',
            parameters=[control_params],
        ),
        Node(
            package='Follow_Lane', executable='switch_control', name='switch_control',
            parameters=[switch_params],
        ),
        Node(package='Follow_Lane', executable='debug_view', name='debug_view'),
    ]


def generate_launch_description():
    return LaunchDescription([
        # Ueberschreibbar, z.B. fuer eine zweite Streckenkonfiguration:
        #   ros2 launch Follow_Lane follow_lane.launch.py detect_params:=/pfad/zur/datei.yaml
        DeclareLaunchArgument('detect_params',
            default_value=os.path.join(PARAM_DIR, 'Follow_Lane_ROI_LAB.yaml')),
        DeclareLaunchArgument('control_params',
            default_value=os.path.join(PARAM_DIR, 'Control_driving_PID_Wide.yaml')),
        DeclareLaunchArgument('switch_control_params',
            default_value=os.path.join(PARAM_DIR, 'Switch_Control.yaml')),
        DeclareLaunchArgument('obstacle', default_value='false'),
        # OpaqueFunction, damit die Pfade aufgeloest sind, bevor die Dateien gelesen werden.
        OpaqueFunction(function=launch_setup),
    ])
