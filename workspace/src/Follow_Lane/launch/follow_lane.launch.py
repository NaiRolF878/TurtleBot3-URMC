import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter')

    return LaunchDescription([
        # Ueberschreibbar, z.B. fuer eine zweite Streckenkonfiguration:
        #   ros2 launch Follow_Lane follow_lane.launch.py detect_params:=/pfad/zur/datei.yaml
        DeclareLaunchArgument('detect_params',
            default_value=os.path.join(param_dir, 'Follow_Lane_ROI_PI_CAM2.yaml')),
        DeclareLaunchArgument('control_params',
            default_value=os.path.join(param_dir, 'Control_driving_PID.yaml')),

        Node(package='Follow_Lane', executable='ingress_node',    name='ingress_node'),
        Node(
            package='Follow_Lane', executable='detect_lane', name='detect_lane',
            parameters=[LaunchConfiguration('detect_params')],
        ),
        Node(
            package='Follow_Lane', executable='control_driving', name='control_driving',
            parameters=[LaunchConfiguration('control_params')],
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
        Node(package='Follow_Lane', executable='switch_control',  name='switch_control'),
        Node(package='Follow_Lane', executable='debug_view',      name='debug_view'),
        # safety_stop bewusst NICHT gestartet (17.08.2026): loeste beim Testen des
        # Farbfehlers/detect_lane wegen der 0.5s-Timeout-Schwelle staendig unnoetig aus.
        # Datei bleibt im Repo, kann jederzeit wieder als Node(...) eingehaengt werden.
        # WICHTIG: ohne diesen Node gibt es KEINEN automatischen Stopp mehr, wenn
        # control_driving abstuerzt oder das WLAN abreisst -- manueller Not-Aus siehe
        # docs/MEMORY.md ("Not-Aus von Hand").
    ])
