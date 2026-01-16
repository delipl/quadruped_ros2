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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_hardware = LaunchConfiguration("use_hardware")
    delcare_use_hardware = DeclareLaunchArgument(
        name="use_hardware",
        default_value="false",
        description="Use hardware or not",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim = DeclareLaunchArgument(
        name="use_sim",
        default_value="true",
        description="Use simulation or not",
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "-c",
            "controller_manager",
        ],
        # condition=IfCondition(use_hardware),
    )

    passive_joint_state_broadcaster = Node(
        package="quadruped_controller",
        executable="passive_joint_state_broadcaster",
        name="passive_joint_state_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": use_sim}],
    )

    inverse_test_controller = Node(
        package="quadruped_controller",
        executable="quadruped_controller_node",
        name="quadruped_controller_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim, "use_hardware": use_hardware}],
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "-c",
            "controller_manager",
        ],
        # condition=IfCondition(use_hardware),
    )

    passive_joint_state_broadcaster = Node(
        package="quadruped_controller",
        executable="passive_joint_state_broadcaster",
        parameters=[{"use_sim_time": use_sim}],
    )

    return LaunchDescription(
        [
            delcare_use_hardware,
            declare_use_sim,
            # Node(
            #     package="joint_state_publisher_gui",
            #     executable="joint_state_publisher_gui",
            #     name="joint_state_publisher_gui",
            #     output="screen",
            #     # remappings= {("joint_states", "rqt_joint_states")},
            # ),
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     output="screen",
            #     arguments=['-d', os.path.join(get_package_share_directory("quadruped_robot_description"), "rviz", "quadruped_robot.rviz")],
            # ),
            passive_joint_state_broadcaster,
            # joint_state_broadcaster_spawner,
            # control_node,
            # # joint_trajectory_controller,
            imu_sensor_broadcaster,
            inverse_test_controller,
            position_controller,
            passive_joint_state_broadcaster,
        ]
    )
