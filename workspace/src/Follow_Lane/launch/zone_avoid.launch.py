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

Bewusst NICHT ueber follow_lane.launch.py eingebunden: dort ist control_driving
direkt auf /cmd_vel gemappt und der alte avoid_obstacle-Node laeuft mit -- das
wuerde den Mux umgehen. Die Parameter-Defaults der gemeinsamen Nodes werden
aber von dort uebernommen (siehe LANE_LAUNCH), damit sie nicht doppelt gepflegt
werden muessen.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Die Dateinamen der Defaults stehen ausschliesslich in launch_params -- hier
# nur nachschlagen, nicht kopieren.
from Follow_Lane.src.launch_params import (
    DEFAULT_AVOID_PARAMS, DEFAULT_CONTROL_PARAMS, DEFAULT_DETECT_PARAMS,
    DEFAULT_SAFETY_PARAMS, DEFAULT_SWITCH_ZONE_PARAMS,
    load_flat_params, param_path,
)


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
        # Ueberschreibbar wie in follow_lane.launch.py, z.B.:
        #   ros2 launch Follow_Lane zone_avoid.launch.py avoid_params:=/pfad/datei.yaml
        DeclareLaunchArgument('detect_params',
            default_value=param_path(DEFAULT_DETECT_PARAMS)),
        DeclareLaunchArgument('control_params',
            default_value=param_path(DEFAULT_CONTROL_PARAMS)),
        # Abweichend zum Spur-Zweig: hier braucht switch_control den Mux und die
        # Umschaltung ueber avoid_node statt ueber den eigenen LiDAR-Kegel.
        DeclareLaunchArgument('switch_control_params',
            default_value=param_path(DEFAULT_SWITCH_ZONE_PARAMS)),
        DeclareLaunchArgument('avoid_params',
            default_value=param_path(DEFAULT_AVOID_PARAMS)),
        DeclareLaunchArgument('safety_params',
            default_value=param_path(DEFAULT_SAFETY_PARAMS)),
        DeclareLaunchArgument('image_view', default_value='true'),
        # OpaqueFunction, damit die Pfade aufgeloest sind, bevor die Dateien gelesen werden.
        OpaqueFunction(function=launch_setup),
    ])
