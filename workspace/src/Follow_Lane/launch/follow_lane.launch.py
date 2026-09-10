from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Liegt im Paket, damit zone_avoid.launch.py dieselbe Funktion und dieselben
# Default-Dateinamen benutzt statt einer Kopie -- kopierte Launch-Logik ist hier
# schon einmal auseinandergelaufen.
from Follow_Lane.src.launch_params import (
    DEFAULT_CONTROL_PARAMS, DEFAULT_DETECT_PARAMS, DEFAULT_SWITCH_PARAMS,
    load_flat_params, param_path,
)


def launch_setup(context, *args, **kwargs):
    detect_params  = load_flat_params(LaunchConfiguration('detect_params').perform(context))
    control_params = load_flat_params(LaunchConfiguration('control_params').perform(context))
    switch_params  = load_flat_params(LaunchConfiguration('switch_control_params').perform(context))

    return [
        Node(package='Follow_Lane', executable='ingress_node',    name='ingress_node'),
        Node(
            package='Follow_Lane', executable='detect_lane', name='detect_lane',
            parameters=[detect_params],
        ),
        Node(
            package='Follow_Lane', executable='control_driving', name='control_driving',
            parameters=[control_params],
            # control_driving publiziert bewusst auf /control/cmd_vel -- das ist der
            # Ausgang des Reglers, nicht zwingend der Eingang des Roboters. Der
            # turtlebot3_node aus dem Bringup hoert aber ausschliesslich auf /cmd_vel.
            # Ohne dieses Remapping laufen die Twist-Nachrichten ins Leere: der Regler
            # rechnet korrekt, "ros2 topic echo /control/cmd_vel" zeigt Werte, aber
            # "Subscription count" bleibt 0 und der Bot steht.
            # Bewusst hier statt im Code: so bleibt der Regler wiederverwendbar (z.B.
            # in Gazebo oder hinter einem Mux, der mehrere Regler arbitriert).
            remappings=[('/control/cmd_vel', '/cmd_vel')],
        ),
        Node(
            package='Follow_Lane', executable='switch_control', name='switch_control',
            parameters=[switch_params],
        ),
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
            default_value=param_path(DEFAULT_DETECT_PARAMS)),
        DeclareLaunchArgument('control_params',
            default_value=param_path(DEFAULT_CONTROL_PARAMS)),
        DeclareLaunchArgument('switch_control_params',
            default_value=param_path(DEFAULT_SWITCH_PARAMS)),
        # OpaqueFunction, damit die Pfade aufgeloest sind, bevor die Dateien gelesen werden.
        OpaqueFunction(function=launch_setup),
    ])
