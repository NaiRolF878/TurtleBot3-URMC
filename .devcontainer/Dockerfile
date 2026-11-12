# Gemeinsames Devcontainer-Image fuer ALLE Branches (Obermenge aller Abhaengigkeiten).
# Auf jedem Branch identisch halten: nur dann trifft der Docker-Cache beim Branchwechsel
# und es wird nichts neu geladen. Aenderungen hier immer auf alle Branches uebernehmen.
# Reihenfolge: selten Geaendertes oben, damit eine Aenderung unten nicht alles neu baut.
FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Basis-Tools, ROS-Pakete (TurtleBot3/ROBOTIS-Doku, Nav2 inkl. der Plugins aus
# nav2_params.yaml) und Python-Bibliotheken aus den package.xml
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    wget \
    curl \
    nano \
    iproute2 \
    iputils-ping \
    nmap \
    chrony \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-mppi-controller \
    ros-humble-nav2-rotation-shim-controller \
    ros-humble-rqt-image-view \
    python3-opencv \
    python3-yaml \
    python3-numpy \
    python3-gpiozero \
    && rm -rf /var/lib/apt/lists/*

# Neuere diagnostic-updater-Version: mit der aus dem Basis-Image (4.0.6) startet der
# Nav2-lifecycle_manager nicht (exit 127, docs/MEMORY.md 2026-10-23).
RUN apt-get update && apt-get install -y \
    ros-humble-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# Node.js (LTS) fuer die Hooks des Claude-Code-Plugins ponytail (.claude/settings.json).
# Nicht aus dem Ubuntu-Repo: Ubuntu 22.04 liefert nur Node 12.
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# TensorRT fuer YOLO-Modelle (.engine). Laeuft nur mit NVIDIA-GPU im Container
# (hostRequirements.gpu in devcontainer.json); ohne GPU geht der Import, das Ausfuehren nicht.
# Version fest: .engine-Dateien laufen nur mit der TensorRT-Version, mit der sie gebaut wurden.
# cuda-python: GPU-Speicher/Kopien fuer TensorRT (Follow_Lane/src/yolo_trt.py).
# py-spy: Profiler fuer scripts/tb/measure-load.sh.
RUN python3 -m pip install --no-cache-dir \
    tensorrt==10.15.1.29 \
    cuda-python==13.0.3 \
    py-spy==0.4.3

# TurtleBot3-Workspace (frueher im postCreateCommand -> bei jedem Rebuild neu geklont
# und gebaut). Pfad /root/turtlebot3_ws bleibt, devcontainer.json verweist darauf.
RUN mkdir -p /root/turtlebot3_ws/src && cd /root/turtlebot3_ws/src \
    && git clone --depth 1 -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
    && git clone --depth 1 -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
    && git clone --depth 1 -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && cd /root/turtlebot3_ws \
    && source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=33
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> /etc/bash.bashrc && \
    echo "source /root/turtlebot3_ws/install/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace
