import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('Follow_Lane')
LAUNCH_DIR = os.path.join(SHARE, 'launch')
PARAM_DIR = os.path.join(SHARE, 'Parameter')


def generate_launch_description():
    """Startet den Nav2-Zweig samt der Basis, auf die er angewiesen ist.

    Bindet follow_lane.launch.py mit ein: Der Nav2-Zweig braucht ingress_node,
    detect_lane und switch_control als Eingang und laeuft ohne sie ins Leere.

    Die beiden Zweige stoeren sich nicht -- sie schliessen sich ueber /control/mode
    gegenseitig aus: control_driving faehrt nur bei ControlType.Lane, lane_costmap_node
    nur bei Obstacle, nav2_goal_publisher bei Obstacle oder Tunnel (und bricht laufende
    Ziele beim Moduswechsel ab). switch_control entscheidet.

    Nur Spurverfolgung ohne Nav2: follow_lane.launch.py.
    """
    return LaunchDescription([
        # Ueberschreibbar, z.B. fuer eine andere Kamera-/Streckenkalibrierung:
        #   ros2 launch Follow_Lane nav2_obstacle.launch.py lane_costmap_params:=/pfad/datei.yaml
        DeclareLaunchArgument('lane_costmap_params',
            default_value=os.path.join(PARAM_DIR, 'Lane_Costmap.yaml')),
        DeclareLaunchArgument('nav2_params',
            default_value=os.path.join(PARAM_DIR, 'nav2_params.yaml')),
        # ACHTUNG: muss immer dieselbe Datei sein wie in follow_lane.launch.py --
        # gleicher Bot, gleiche Kamera. Beide liefen versehentlich auf
        # unterschiedlichen Dateien, dadurch fuhr der Nav2-Zweig mit anderem ROI
        # und anderem detection_row_factor (0.75 statt 0.94) als der Spur-Zweig.
        DeclareLaunchArgument('detect_params',
            default_value=os.path.join(PARAM_DIR, 'Follow_Lane_ROI_PI_CAM2_V3.yaml')),
        DeclareLaunchArgument('control_params',
            default_value=os.path.join(PARAM_DIR, 'Control_driving_PID.yaml')),
        DeclareLaunchArgument('switch_control_params',
            default_value=os.path.join(PARAM_DIR, 'Switch_Control.yaml')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, 'follow_lane.launch.py')),
            launch_arguments=[
                ('detect_params', LaunchConfiguration('detect_params')),
                ('control_params', LaunchConfiguration('control_params')),
                ('switch_control_params', LaunchConfiguration('switch_control_params')),
            ],
        ),

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
        # Nur aktiv im Tunnel-Modus -- sucht Frontier-Ziele auf global_costmap
        # und laesst planner_server (siehe unten) echte Pfade dorthin planen.
        Node(
            package='Follow_Lane', executable='tunnel_explorer_node', name='tunnel_explorer_node',
            parameters=[LaunchConfiguration('lane_costmap_params')],
        ),

        # Local Costmap + Local Controller (im controller_server-Prozess
        # zusammengefasst) fuer den Obstacle-Zweig, siehe Kommentar in
        # nav2_params.yaml.
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('nav2_params')],
        ),
        # Global Planner + die dazugehoerige (nicht-mitfahrende) Costmap fuer
        # den Tunnel-Zweig -- siehe Kommentar in nav2_params.yaml. Faehrt
        # selbst keine Fahrbefehle, liefert nur Pfade an tunnel_explorer_node
        # -> nav2_goal_publisher -> controller_server.
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen',
            parameters=[LaunchConfiguration('nav2_params')],
        ),
        # Nav2-Lifecycle-Nodes starten unkonfiguriert -- ohne diesen Manager
        # bleiben controller_server/planner_server inaktiv.
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[LaunchConfiguration('nav2_params')],
        ),
    ])
