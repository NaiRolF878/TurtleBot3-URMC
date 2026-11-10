FROM osrf/ros:humble-desktop-full

# Basis-Tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    wget \
    nano \
    iproute2 \
    iputils-ping \
    nmap \
    chrony \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot3-relevante ROS-Pakete (wie in der ROBOTIS Doku)
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Neuere diagnostic-updater-Version: mit der aus dem Basis-Image (4.0.6) startet der
# Nav2-lifecycle_manager nicht (exit 127, docs/MEMORY.md 2026-10-23).
RUN apt-get update && apt-get install -y \
    ros-humble-diagnostic-updater \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# TensorRT fuer YOLO-Modelle (.engine). Laeuft nur mit NVIDIA-GPU im Container
# (hostRequirements.gpu in devcontainer.json); ohne GPU geht der Import, das Ausfuehren nicht.
# Version fest: .engine-Dateien laufen nur mit der TensorRT-Version, mit der sie gebaut wurden.
RUN python3 -m pip install --no-cache-dir tensorrt==10.15.1.29
# CUDA-Runtime fuer Python (GPU-Speicher/Kopien fuer TensorRT, Follow_Lane/src/yolo_trt.py).
RUN python3 -m pip install --no-cache-dir cuda-python==13.0.3
# Profiler fuer scripts/tb/measure-load.sh (Zeit je Funktion der laufenden Nodes)
RUN python3 -m pip install --no-cache-dir py-spy==0.4.3

ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=33
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> /etc/bash.bashrc

WORKDIR /workspace
