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
    """Startet alles: Spurverfolgung, Nav2-Zweig, RViz und Debug-Fenster.

    Die Fahrzweige schliessen sich ueber /control/mode gegenseitig aus --
    control_driving faehrt bei Lane, der Nav2-Zweig bei Obstacle bzw. Tunnel.

    detect_params/control_params/switch_control_params werden hier bewusst NICHT
    deklariert -- die Defaults stehen ausschliesslich in follow_lane.launch.py.
    Vorher standen sie doppelt und liefen auseinander: der Nav2-Zweig fuhr mit
    einer anderen Kameradatei als der Spur-Zweig. Zum Aendern dort anfassen.
    """
    return LaunchDescription([
        DeclareLaunchArgument('lane_costmap_params',
            default_value=os.path.join(PARAM_DIR, 'Lane_Costmap.yaml')),
        DeclareLaunchArgument('nav2_params',
            default_value=os.path.join(PARAM_DIR, 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('image_view', default_value='true'),

        # Basis: ingress_node, detect_lane, control_driving, switch_control, debug_view
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

        # Local Costmap + MPPI
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('nav2_params')],
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
    ])
