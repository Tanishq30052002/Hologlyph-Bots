from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_hb_task2a = get_package_share_directory('hb_task2a')
    
    return LaunchDescription(
        [
            Node(
                package="hb_task2a",           # Enter the name of your ROS2 package
                executable="controller",    # Enter the name of your executable
                output="screen",
            ),

            Node(
                package="hb_task2a",           # Enter the name of your ROS2 package
                executable="service_node",    # Enter the name of your executable
                output="screen",
            ),

            Node(
                package="hb_task2a",           # Enter the name of your ROS2 package
                executable="feedback",    # Enter the name of your executable
                output="screen",
            ),

            Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(pkg_hb_task2a, 'rviz', 'display.rviz')]]
        )
        ]
    )