# =============================================================================
#  TurtleBot3 Burger – URMC Development Container
#  ROS2 Jazzy | Gazebo Harmonic | Ubuntu 24.04
# =============================================================================
#  GUI-Modus:
#    Linux  → X11 (Standard, aktiv)
#    Windows→ noVNC (auskommentiert, bei Bedarf aktivieren)
# =============================================================================

FROM osrf/ros:jazzy-desktop-full-noble AS base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ── Basis-Werkzeuge ──────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        git wget curl nano htop \
        python3-pip python3-colcon-common-extensions python3-vcstool \
        python3-argcomplete \
        mesa-utils libgl1-mesa-glx libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# ── ROS2 TurtleBot3 + Gazebo Pakete ─────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-gazebo-ros-pkgs \
        ros-jazzy-ros-gz \
        ros-jazzy-dynamixel-sdk \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-cartographer \
        ros-jazzy-cartographer-ros \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-rviz2 \
        ros-jazzy-rqt* \
        ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# ── TurtleBot3 Workspace aus Quellen bauen ───────────────────────────────────
RUN mkdir -p /turtlebot3_ws/src && cd /turtlebot3_ws/src \
    && git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
    && git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
    && git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
    && cd /turtlebot3_ws \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install

# ── Umgebungsvariablen ───────────────────────────────────────────────────────
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=30
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV GZ_VERSION=harmonic

# =============================================================================
#  YOLO / Computer Vision (auskommentiert – bei Bedarf aktivieren)
# =============================================================================
# RUN apt-get update && apt-get install -y --no-install-recommends \
#         python3-opencv \
#         libopencv-dev \
#     && rm -rf /var/lib/apt/lists/*
#
# RUN pip3 install --break-system-packages \
#         ultralytics \
#         opencv-python-headless \
#         numpy
#
# # Für YOLO mit ROS2-Integration:
# # RUN pip3 install --break-system-packages \
# #         rclpy-message-filters \
# #         cv_bridge

# =============================================================================
#  noVNC für Windows (auskommentiert – bei Bedarf aktivieren)
# =============================================================================
# RUN apt-get update && apt-get install -y --no-install-recommends \
#         tigervnc-standalone-server tigervnc-common \
#         novnc websockify \
#         supervisor \
#         xfce4 xfce4-terminal dbus-x11 \
#     && rm -rf /var/lib/apt/lists/*
#
# ENV DISPLAY=:1
# ENV QT_QPA_PLATFORM=xcb
#
# COPY novnc_startup.sh /opt/novnc_startup.sh
# RUN chmod +x /opt/novnc_startup.sh
# EXPOSE 6080 5901

# =============================================================================
#  Entrypoint
# =============================================================================
COPY entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

WORKDIR /workspace
ENTRYPOINT ["/opt/entrypoint.sh"]
CMD ["bash"]
