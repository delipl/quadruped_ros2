## Build

```bash
sudo apt install can-utils

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i
```

## Run 

```bash
colcon build --symlink-install --packages-up-to quadruped_robot_description --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint1],
    points: [
        { positions: [0.0 ], time_from_start: { sec: 0, nanosec: 100000000 } },
        { positions: [1.57 ], time_from_start: { sec: 0, nanosec: 200000000 } },
             { positions: [0.0 ], time_from_start: { sec: 0, nanosec: 300000000 } },
        { positions: [3.14 ], time_from_start: { sec: 0, nanosec: 400000000 } },
             { positions: [0.0 ], time_from_start: { sec: 0, nanosec: 500000000 } },
        { positions: [4.71 ], time_from_start: { sec: 0, nanosec: 600000000 } },
         { positions: [0.0 ], time_from_start: { sec: 1, nanosec: 300000000 } },
    ]
  }
}"

ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint20, joint21],
    points: [
        { positions: [0.0, 0.0 ], time_from_start: { sec: 0, nanosec: 100000000 } },
        { positions: [0.5, 1.57 ], time_from_start: { sec: 0, nanosec: 200000000 } },
          { positions: [0.0, 0.0 ], time_from_start: { sec: 0, nanosec: 300000000 } },

    ]
  }
}"

ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names:[joint10, joint11, joint20, joint21],
    points: [
        { positions: [0.0, 0.0 ,0.0, 0.0 ], time_from_start: { sec: 0, nanosec: 100000000 } },
        { positions: [0.5, 1.57, 0.5, 1.57 ], time_from_start: { sec: 0, nanosec: 200000000 } },
          { positions: [0.0, 0.0 ,0.0, 0.0 ], time_from_start: { sec: 0, nanosec: 300000000 } },

    ]
  }
}"


ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names:[joint10, joint11, joint20, joint21],
    points: [
        { positions: [0.0, 0.0 ,0.0, 0.0 ], time_from_start: { sec: 0, nanosec: 100000000 } },
        { positions: [0.5, 1.57, 0.5, 1.57 ], time_from_start: { sec: 2, nanosec: 200000000 } },
          { positions: [0.0, 0.0 ,0.0, 0.0 ], time_from_start: { sec: 4, nanosec: 300000000 } },

    ]
  }
}"
# quadruped_ros2
