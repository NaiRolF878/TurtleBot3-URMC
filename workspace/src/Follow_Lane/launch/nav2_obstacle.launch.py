import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = os.path.join(get_package_share_directory('Follow_Lane'), 'Parameter')

    return LaunchDescription([
        # Ueberschreibbar, z.B. fuer eine andere Kamera-/Streckenkalibrierung:
        #   ros2 launch Follow_Lane nav2_obstacle.launch.py lane_costmap_params:=/pfad/datei.yaml
        DeclareLaunchArgument('lane_costmap_params',
            default_value=os.path.join(param_dir, 'Lane_Costmap.yaml')),
        DeclareLaunchArgument('nav2_params',
            default_value=os.path.join(param_dir, 'nav2_params.yaml')),

        # Voraussetzung: detect_lane_node, ingress_node und switch_control laufen
        # bereits (siehe follow_lane.launch.py) -- dieses Launch-File startet nur
        # den Nav2-Zweig zusaetzlich dazu. switch_control entscheidet ueber
        # /control/mode (ControlType.Obstacle), welcher der beiden Zweige aktiv
        # Fahrbefehle ausgibt.
        Node(
            package='Follow_Lane', executable='lane_costmap_node', name='lane_costmap_node',
            parameters=[LaunchConfiguration('lane_costmap_params')],
        ),
        Node(
            package='Follow_Lane', executable='nav2_goal_publisher', name='nav2_goal_publisher',
            parameters=[LaunchConfiguration('lane_costmap_params')],
        ),

        # Nur Local Costmap + Local Controller (im controller_server-Prozess
        # zusammengefasst) -- kein Global Planner, kein Global Costmap, kein
        # AMCL, siehe Kommentar in nav2_params.yaml.
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('nav2_params')],
        ),
        # Nav2-Lifecycle-Nodes starten unkonfiguriert -- ohne diesen Manager
        # bleibt controller_server inaktiv und veroeffentlicht nie /cmd_vel.
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[LaunchConfiguration('nav2_params')],
        ),
    ])
