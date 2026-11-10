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


def generate_launch_description():
    """Storage-Aufgabe (Palette): Spurverfolgung plus die Storage-Detektoren, ohne Nav2,
    ausser dem Not-Aus (notaus), den launchers/storage.sh im Vordergrund startet.

    switch_control startet hier in Stop und faehrt erst nach dem Button STORAGE STARTEN.
    Der Magnet-Node (magnet_controll) laeuft auf dem Bot, nicht hier.
    """
    storage_params = LaunchConfiguration('storage_params')
    return LaunchDescription([
        DeclareLaunchArgument('storage_params',
            default_value=os.path.join(PARAM_DIR, 'Storage.yaml')),
        DeclareLaunchArgument('image_view', default_value='true'),

        # Basis: odom_filter, ingress_node_nav2, detect_lane_nav2, control_driving, switch_control,
        # debug_view
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, 'follow_lane.launch.py')),
            launch_arguments={'storage_params': storage_params}.items()),

        Node(package='Follow_Lane', executable='storage_zone_node', name='storage_zone_node',
             parameters=[storage_params]),
        Node(package='Follow_Lane', executable='storage_sign_node', name='storage_sign_node',
             parameters=[storage_params]),
        Node(package='Follow_Lane', executable='pallet_node', name='pallet_node',
             parameters=[storage_params]),

        # Ohne DISPLAY scheitert das Fenster, ohne den Rest mitzureissen -- dann mit
        # image_view:=false starten.
        Node(
            package='rqt_image_view', executable='rqt_image_view', name='rqt_image_view',
            arguments=['/lokal/debug_view'],
            condition=IfCondition(LaunchConfiguration('image_view')),
        ),
    ])
