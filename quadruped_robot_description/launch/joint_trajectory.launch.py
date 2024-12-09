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
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("quadruped_controller"),
            "config",
            "jtc_controllers.yaml",
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
        # condition=IfCondition(use_hardware),
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "controller_manager",
        ],
        # condition=IfCondition(use_hardware),
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
    )

    inverse_test_controller = Node(
        package="quadruped_controller",
        executable="quadruped_controller_node",
        name="quadruped_controller_node",
        output="screen",
        parameters=[{"use_hardware": use_hardware}],
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
                default_value=Command(
                    ["xacro ", urdf_file, " use_hardware:=", use_hardware]
                ),
                description="Absolute path to robot urdf file",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "robot_description": LaunchConfiguration("robot_description"),
                    }
                ],
            ),
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
            joint_state_broadcaster_spawner,
            control_node,
            joint_trajectory_controller,
            imu_sensor_broadcaster,
            inverse_test_controller,
        ]
    )
