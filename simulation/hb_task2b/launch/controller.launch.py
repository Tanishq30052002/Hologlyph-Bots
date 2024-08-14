
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_hb_task2b = get_package_share_directory('hb_task2b')

    return LaunchDescription([
        Node(
            package='hb_task2b',
            executable='nextgoalpub',
        ),
        Node(
            package='hb_task2b',
            executable='controller',
        ),
        Node(
            package='hb_task2b',
            executable='feedback',
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(pkg_hb_task2b, 'rviz', 'display.rviz')]]
        )
    ])