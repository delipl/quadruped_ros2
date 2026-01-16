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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_reload = DeclareLaunchArgument(
        name="reload",
        default_value="False",
        description="Whether to reload the controller on unclean shutdown",
    )

    reload = LaunchConfiguration("reload")

    quadruped_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "quadruped_controller",
            "-c",
            "controller_manager",
        ],
    )

    unload_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "unload_controller",
            "quadruped_controller",
        ],
        on_exit=[quadruped_controller],
    )

    inactivate_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "set_controller_state",
            "quadruped_controller",
            "inactive",
        ],
        on_exit=[unload_controller],
        condition=IfCondition(reload),
    )

    spawner = ExecuteProcess(
        cmd=["ls"],
        on_exit=[quadruped_controller],
        condition=UnlessCondition(reload),
    )

    return LaunchDescription([declare_reload, spawner, inactivate_controller])
