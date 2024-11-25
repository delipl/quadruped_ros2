import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('quadruped_joy'),
        'config',
        'quadruped_joy_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='quadruped_joy',
            executable='quadruped_joy',
            name='quadruped_joy',
            output='screen',
            parameters=[config],
        ),
    ])
