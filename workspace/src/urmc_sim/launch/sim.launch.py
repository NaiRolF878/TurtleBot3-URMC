import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_SHARE = get_package_share_directory('urmc_sim')
LANE_LAUNCH = os.path.join(get_package_share_directory('Follow_Lane'), 'launch')


def generate_launch_description():
    """Simulator plus die Pipeline des getesteten Branches (nav2_obstacle.launch.py), ohne
    Not-Aus: den startet sim.sh im Vordergrund."""
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='tunnel_gap'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('image_view', default_value='true'),

        Node(
            package='urmc_sim', executable='sim_node', name='urmc_sim', output='screen',
            parameters=[os.path.join(SIM_SHARE, 'config', 'sim.yaml'),
                        {'world': LaunchConfiguration('world')}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(LANE_LAUNCH, 'nav2_obstacle.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz'),
                              'image_view': LaunchConfiguration('image_view')}.items()),
        Node(
            package='rqt_image_view', executable='rqt_image_view', name='rqt_sim_view',
            arguments=['/sim/view'],
            condition=IfCondition(LaunchConfiguration('image_view')),
        ),
    ])
