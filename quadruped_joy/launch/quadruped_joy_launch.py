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
            # parameters=[config],
        ),
        Node(
            package="joy2twist",
            executable="joy2twist",
            output='screen',
            parameters=[config],
            remappings=[
                ("joy", "joy_cmd_vel")
            ]
        ), 
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            output='screen',
            arguments=["/dev/input/js0 "],
        ),
        Node(
            package="quadruped_controller",
            executable="twist_to_trajectory.py",
        )
    ])