FROM osrf/ros:humble-desktop-full

# Basis-Tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    wget \
    nano \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot3-relevante ROS-Pakete (wie in der ROBOTIS Doku)
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=33
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> /etc/bash.bashrc

WORKDIR /workspace
