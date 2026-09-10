"""Spurfolge + zonenbasiertes Ausweichen + LiDAR-Not-Aus.

Aufbau (Prioritaet im Mux von switch_control absteigend):

    safety_node    --/cmd_safe --.
    avoid_node     --/cmd_avoid--+--> switch_control (Mux) --/control/cmd_vel--> /cmd_vel
    control_driving--/cmd_lane --'

Im Normalbetrieb faehrt control_driving. avoid_node schickt dabei nur einen
Offset auf /avoid/offset, den control_driving auf den Spur-Error addiert -- der
Roboter zieht innerhalb der Spur am Hindernis vorbei, die Spurerkennung haelt
dagegen und die gelbe Linie wird nicht ueberfahren.

Ist die Frontzone komplett blockiert, meldet avoid_node die Uebernahme auf
/avoid/takeover, switch_control geht auf Obstacle (womit control_driving
verstummt) und avoid_node faehrt selbst ueber /cmd_avoid.

Einzige Launch-Datei des Pakets -- alle Parameter-Defaults stehen hier.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
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
        print(f'[zone_avoid.launch] Parameterdatei nicht gefunden, nutze Code-Defaults: {path}')
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
    def params(arg_name):
        return load_flat_params(LaunchConfiguration(arg_name).perform(context))

    detect_params = params('detect_params')
    control_params = params('control_params')
    switch_params = params('switch_control_params')
    avoid_params = params('avoid_params')
    safety_params = params('safety_params')

    return [
        # Kamera entzerren + /scan spiegeln
        Node(package='Follow_Lane', executable='ingress_node', name='ingress_node'),

        Node(
            package='Follow_Lane', executable='detect_lane', name='detect_lane',
            parameters=[detect_params],
        ),

        # Prioritaet 3: Spurfolge. Der Regler-Ausgang geht hier NICHT direkt an
        # den Roboter, sondern als /cmd_lane in den Mux.
        Node(
            package='Follow_Lane', executable='control_driving', name='control_driving',
            parameters=[control_params],
            remappings=[('/control/cmd_vel', '/cmd_lane')],
        ),

        # Prioritaet 2: Zonen-Ausweichen (+ /avoid/offset im Normalbetrieb)
        Node(
            package='Follow_Lane', executable='avoid_node', name='avoid_node',
            parameters=[avoid_params],
        ),

        # Prioritaet 1: Not-Aus
        Node(
            package='Follow_Lane', executable='safety_node', name='safety_node',
            parameters=[safety_params],
        ),

        # Modus-Zustandsmaschine + Mux. Erst hier wird auf /cmd_vel gemappt,
        # dem einzigen Topic, auf das der turtlebot3_node aus dem Bringup hoert.
        # Bewusst im Launch statt im Code: so bleibt der Mux auch in Gazebo o.ae.
        # nutzbar.
        Node(
            package='Follow_Lane', executable='switch_control', name='switch_control',
            parameters=[switch_params],
            remappings=[('/control/cmd_vel', '/cmd_vel')],
        ),

        Node(package='Follow_Lane', executable='debug_view', name='debug_view'),

        # Ohne DISPLAY scheitert das, ohne den Rest mitzureissen --
        # dann mit image_view:=false starten.
        Node(
            package='rqt_image_view', executable='rqt_image_view', name='rqt_image_view',
            arguments=['/lokal/debug_view'],
            condition=IfCondition(LaunchConfiguration('image_view')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # Ueberschreibbar, z.B. fuer eine zweite Streckenkonfiguration:
        #   ros2 launch Follow_Lane zone_avoid.launch.py avoid_params:=/pfad/datei.yaml
        DeclareLaunchArgument('detect_params',
            default_value=os.path.join(PARAM_DIR, 'Follow_Lane_ROI_PI_CAM2_V3.yaml')),
        DeclareLaunchArgument('control_params',
            default_value=os.path.join(PARAM_DIR, 'Control_driving_PID.yaml')),
        DeclareLaunchArgument('switch_control_params',
            default_value=os.path.join(PARAM_DIR, 'Switch_Control.yaml')),
        DeclareLaunchArgument('avoid_params',
            default_value=os.path.join(PARAM_DIR, 'Avoid_Node.yaml')),
        DeclareLaunchArgument('safety_params',
            default_value=os.path.join(PARAM_DIR, 'Safety_Node.yaml')),
        DeclareLaunchArgument('image_view', default_value='true'),
        # OpaqueFunction, damit die Pfade aufgeloest sind, bevor die Dateien gelesen werden.
        OpaqueFunction(function=launch_setup),
    ])
