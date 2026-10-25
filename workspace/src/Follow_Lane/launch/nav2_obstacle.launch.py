import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('Follow_Lane')
LAUNCH_DIR = os.path.join(SHARE, 'launch')
PARAM_DIR = os.path.join(SHARE, 'Parameter')
RVIZ_CONFIG = os.path.join(SHARE, 'rviz', 'urmc.rviz')


def generate_launch_description():
    """Startet alles: Spurverfolgung, Nav2-Zweig, RViz und Debug-Fenster, ausser dem
    Not-Aus (notaus), der ein Terminal fuer die Leertaste braucht. launchers/nav2_obstacle.sh
    startet ihn im Vordergrund; ohne ihn faehrt der Bot nicht.

    Fahrbefehle: controller_server -> /nav2/cmd_vel -> control_driving -> /control/cmd_vel
    -> notaus -> /cmd_vel. control_driving waehlt je nach /control/mode PID oder Nav2.

    detect_params/control_params/switch_control_params sind nur in follow_lane.launch.py
    deklariert, damit beide Zweige mit denselben Dateien fahren.
    """
    return LaunchDescription([
        DeclareLaunchArgument('lane_costmap_params',
            default_value=os.path.join(PARAM_DIR, 'Lane_Costmap.yaml')),
        DeclareLaunchArgument('nav2_params',
            default_value=os.path.join(PARAM_DIR, 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('image_view', default_value='true'),

        # Basis: ingress_node_nav2, detect_lane_nav2, control_driving, switch_control, debug_view
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, 'follow_lane.launch.py'))),

        Node(
            package='Follow_Lane', executable='lane_costmap_node', name='lane_costmap_node',
            parameters=[LaunchConfiguration('lane_costmap_params')],
        ),
        Node(
            package='Follow_Lane', executable='nav2_goal_publisher', name='nav2_goal_publisher',
            parameters=[LaunchConfiguration('lane_costmap_params')],
        ),
        Node(
            package='Follow_Lane', executable='keepout_zone_node', name='keepout_zone_node',
            parameters=[LaunchConfiguration('lane_costmap_params')],
        ),
        Node(
            package='Follow_Lane', executable='tunnel_explorer_node', name='tunnel_explorer_node',
            parameters=[LaunchConfiguration('lane_costmap_params')],
        ),
        # Draufsicht: Karte, Bot, Pfad, Statuszeile mit Abbruchgrund (/nav2/view)
        Node(package='Follow_Lane', executable='nav2_view', name='nav2_view'),

        # Local Costmap + MPPI. Sendet nicht direkt an den Bot: control_driving reicht
        # /nav2/cmd_vel nur im Obstacle- und Tunnel-Modus weiter.
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('nav2_params')],
            remappings=[('cmd_vel', '/nav2/cmd_vel')],
        ),
        # Global Planner + Tunnelkarte, liefert nur Pfade
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen',
            parameters=[LaunchConfiguration('nav2_params')],
        ),
        # Ohne Lifecycle-Manager bleiben beide Server unkonfiguriert und inaktiv.
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[LaunchConfiguration('nav2_params')],
        ),

        # Anzeige. Ohne DISPLAY scheitern beide, ohne den Rest mitzureissen --
        # dann mit rviz:=false image_view:=false starten.
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', RVIZ_CONFIG],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='rqt_image_view', executable='rqt_image_view', name='rqt_image_view',
            arguments=['/lokal/debug_view'],
            condition=IfCondition(LaunchConfiguration('image_view')),
        ),
        # Zweites Fenster fuer die Nav2-Draufsicht; eigener Name, sonst kollidieren die Knoten.
        Node(
            package='rqt_image_view', executable='rqt_image_view', name='rqt_nav2_view',
            arguments=['/nav2/view'],
            condition=IfCondition(LaunchConfiguration('image_view')),
        ),
    ])
