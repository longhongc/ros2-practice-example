from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from glob import glob

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='data_processor',
            namespace='',
            executable='laser_subscriber',
            name='laser_subscriber'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('data_processor'), 'laser_scan_config.rviz')]
        )
    ])
