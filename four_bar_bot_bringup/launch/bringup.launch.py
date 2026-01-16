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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("four_bar_bot_description"), "urdf", "four_bar_bot.urdf.xacro"]
    )

    use_hardware = LaunchConfiguration("use_hardware")
    use_sim = LaunchConfiguration("use_sim")

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("four_bar_bot_bringup"),
            "config",
            PythonExpression(
                ["'controllers.yaml' if ", use_hardware, " else 'mock_controllers.yaml'"]
            ),
        ]
    )

    declare_use_hardware = DeclareLaunchArgument(
        name="use_hardware",
        default_value="True",
        description="Use hardware or not",
    )

    declare_use_sim = DeclareLaunchArgument(
        name="use_sim",
        default_value="False",
        description="Use simulation or not",
    )

    declare_robot_description = DeclareLaunchArgument(
        name="robot_description",
        default_value=Command(
            ["xacro ", urdf_file, " use_hardware:=", use_hardware, " use_sim:=", use_sim]
        ),
        description="URDF robot description content",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": LaunchConfiguration("robot_description"),
            }
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        emulate_tty=True,
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node, on_exit=[Shutdown(reason="ROS Control failed to launch")]
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # quadruped_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "quadruped_controller",
    #         "-c",
    #         "controller_manager",
    #     ],
    # )

    return LaunchDescription(
        [
            declare_use_hardware,
            declare_use_sim,
            declare_robot_description,
            robot_state_publisher_node,
            control_node,
            shutdown_handler,
            joint_state_broadcaster_spawner,
        ]
    )
