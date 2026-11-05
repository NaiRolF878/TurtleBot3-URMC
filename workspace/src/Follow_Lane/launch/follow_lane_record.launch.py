"""Spurverfolgung wie follow_lane.launch.py, dazu image_recorder: speichert waehrend der
Fahrt periodisch Trainingsbilder fuer YOLO (Original und entzerrt im Wechsel) nach
dataset/fahrt_<datum>_<zeit>/. Start ueber scripts/tb/launchers/follow_lane_record.sh
(mit Not-Aus).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    follow_lane = os.path.join(
        get_package_share_directory('Follow_Lane'), 'launch', 'follow_lane.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(follow_lane)),
        Node(package='Follow_Lane', executable='image_recorder', name='image_recorder'),
    ])
