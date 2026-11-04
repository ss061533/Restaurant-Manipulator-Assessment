# syntax=docker/dockerfile:1
# =============================================================================================
#  Vision-guided café tray pick-and-place  -  ROS 2 Humble / Ubuntu 22.04
#
#  Build (default = Gazebo Fortress / "ign", binary packages, the combination tested by the author):
#      docker build -t cafe-ur30:fortress .
#  Build with the new Gazebo "gz" (Harmonic)  - EXPERIMENTAL, compiles ros_gz + gz_ros2_control (~25 min):
#      docker build --build-arg GZ=harmonic -t cafe-ur30:harmonic .
# =============================================================================================
FROM osrf/ros:humble-desktop

ARG GZ=fortress
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    QT_X11_NO_MITSHM=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# ---- common dependencies ------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential git curl wget lsb-release gnupg ca-certificates \
      python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool \
      python3-opencv python3-numpy python3-pytest \
      ros-humble-moveit ros-humble-ur-description \
      ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager \
      ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
      ros-humble-xacro ros-humble-cv-bridge ros-humble-message-filters ros-humble-tf2-ros-py \
      ros-humble-rviz2 ros-humble-robot-state-publisher \
      mesa-utils libgl1-mesa-dri libegl1-mesa x11-apps \
    && rm -rf /var/lib/apt/lists/*

# ---- Gazebo ---------------------------------------------------------------------------------
# fortress : ros-humble-ros-ign + ros-humble-ign-ros2-control (apt)
# harmonic : gz-harmonic (apt) + ros_gz and gz_ros2_control built from source for Harmonic
RUN set -eux; \
    if [ "$GZ" = "fortress" ]; then \
        apt-get update; \
        apt-get install -y --no-install-recommends ros-humble-ros-ign ros-humble-ign-ros2-control; \
        rm -rf /var/lib/apt/lists/*; \
    elif [ "$GZ" = "harmonic" ]; then \
        curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg; \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
            > /etc/apt/sources.list.d/gazebo-stable.list; \
        apt-get update; \
        apt-get install -y --no-install-recommends gz-harmonic; \
        rm -rf /var/lib/apt/lists/*; \
    else \
        echo "build-arg GZ must be 'fortress' or 'harmonic'"; exit 1; \
    fi

RUN set -eux; \
    if [ "$GZ" = "harmonic" ]; then \
        mkdir -p /opt/gz_ws/src; cd /opt/gz_ws/src; \
        git clone --depth 1 -b humble https://github.com/gazebosim/ros_gz.git; \
        git clone --depth 1 -b humble https://github.com/ros-controls/gz_ros2_control.git; \
        cd /opt/gz_ws; \
        export GZ_VERSION=harmonic; \
        apt-get update; (rosdep init || true); rosdep update; \
        rosdep install -r --from-paths src -i -y --rosdistro humble; \
        . /opt/ros/humble/setup.sh; \
        colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release; \
        rm -rf /var/lib/apt/lists/* /opt/gz_ws/build /opt/gz_ws/log; \
    fi

# ---- UR30 description: use the apt one if it already knows the UR30, otherwise build humble branch
RUN set -eux; \
    if [ ! -d /opt/ros/humble/share/ur_description/config/ur30 ]; then \
        mkdir -p /opt/ur_ws/src; \
        git clone --depth 1 -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git /opt/ur_ws/src/ur_description; \
        . /opt/ros/humble/setup.sh; cd /opt/ur_ws; \
        colcon build --merge-install; \
        rm -rf /opt/ur_ws/build /opt/ur_ws/log; \
    fi

# ---- our workspace ------------------------------------------------------------------------
WORKDIR /ws
COPY src /ws/src
RUN set -eux; \
    . /opt/ros/humble/setup.sh; \
    [ -f /opt/gz_ws/install/setup.sh ] && . /opt/gz_ws/install/setup.sh; \
    [ -f /opt/ur_ws/install/setup.sh ] && . /opt/ur_ws/install/setup.sh; \
    apt-get update; (rosdep init || true); rosdep update; \
    SKIP=""; if [ "$GZ" = "harmonic" ]; then SKIP="ros_ign_gazebo ros_ign_bridge ign_ros2_control"; fi; \
    rosdep install --from-paths src --ignore-src -r -y ${SKIP:+--skip-keys "$SKIP"} || true; \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
COPY kill_sim.sh /usr/local/bin/kill_sim
RUN chmod +x /entrypoint.sh /usr/local/bin/kill_sim

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "cafe_bringup", "cafe.launch.xml"]
