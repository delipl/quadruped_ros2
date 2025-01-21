ARG ROS_DISTRO=humble

FROM husarnet/ros:${ROS_DISTRO}-ros-core

STOPSIGNAL SIGINT

WORKDIR /ros2_ws

COPY ./src /ros2_ws/src/

RUN apt-get update  && \
    apt-get install -y \
        ros-dev-tools && \
    # Install dependencies
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src -y -i && \
    # Build
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --packages-up-to quadruped_robot_description --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Get version
    echo $(cat /ros2_ws/src/quadruped_robot_description/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') >> /version.txt && \
    # Size optimalization
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
        ros-dev-tools && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN sed -i "/# <additional-user-commands>/i set -e -a " /*_entrypoint.sh