import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory("quadruped_robot_description"),
        "urdf",
        "quadruped_robot.urdf.xacro",
    )



    passive_joint_state_broadcaster = Node(
        package="quadruped_controller",
        executable="passive_joint_state_broadcaster",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="robot_description",
                default_value=Command(["xacro ", urdf_file]),
                description="Absolute path to robot urdf file",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": LaunchConfiguration("robot_description"),
                    }
                ],
            ),
            # passive_joint_state_broadcaster,
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
        ]
    )
