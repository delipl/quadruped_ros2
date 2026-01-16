#!/usr/bin/env python3

# Copyright 2026 Jakub Delicat
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory("quadruped_robot_description"),
        "urdf",
        "quadruped_robot.urdf.xacro",
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("quadruped_controller"),
            "config",
            "controllers.yaml",
        ]
    )

    use_hardware = LaunchConfiguration("use_hardware")
    delcare_use_hardware = DeclareLaunchArgument(
        name="use_hardware",
        default_value="false",
        description="Use hardware or not",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=IfCondition(use_hardware),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        emulate_tty=True,
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        condition=IfCondition(use_hardware),
    )

    quadruped_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "quadruped_controller",
            "-c",
            "controller_manager",
        ],
        # condition=IfCondition(use_hardware),
    )

    return LaunchDescription(
        [
            delcare_use_hardware,
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                name="use_verbose",
                default_value="false",
                description='Set to "true" to run verbose logging.',
            ),
            DeclareLaunchArgument(
                name="robot_description",
                default_value=Command(["xacro ", urdf_file]),
                description="Absolute path to robot urdf file",
            ),
            joint_state_broadcaster_spawner,
            control_node,
            quadruped_controller,
        ]
    )
