import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():

    mujoco_ros2_control_demos_path = os.path.join(
        get_package_share_directory("mujoco_ros2_control_demos")
    )
    xacro_file = os.path.join(
        mujoco_ros2_control_demos_path, "urdf", "quadruped_to_mujoco.urdf"
    )
    controller_config_file = os.path.join(
        mujoco_ros2_control_demos_path, "config", "position_controller.yaml"
    )
    # rviz_config_file = os.path.join(mujoco_ros2_control_demos_path,
    #                                       'launch',
    #                                       'camera_demo.rviz')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {"robot_description": doc.toxml()}

    use_sim_time = {"use_sim_time": True}

    node_mujoco_ros2_control = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            robot_description,
            controller_config_file,
            use_sim_time,
            {
                "mujoco_model_path": os.path.join(
                    mujoco_ros2_control_demos_path, "mujoco_models", "quadruped.xml"
                )
            },
        ],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            use_sim_time,
            robot_description,
        ],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_position_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "effort_controllers",
        ],
        output="screen",
    )

    passive_joint_state_broadcaster = Node(
        package="quadruped_controller",
        executable="passive_joint_state_broadcaster",
        parameters=[{"use_sim_time": True}],
    )

    
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     parameters=[use_sim_time],
    #     arguments=["-d", rviz_config_file],
    # )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=node_mujoco_ros2_control,
                    on_start=[load_joint_state_controller],
                )
            ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=load_joint_state_controller,
            #         on_exit=[load_position_controller],
            #     )
            # ),
            node_mujoco_ros2_control,
            node_robot_state_publisher,
            # passive_joint_state_broadcaster
            # rviz_node,
        ]
    )
