#!/bin/bash
# Einmalig in WSL (Ubuntu 22.04), nach tb3-wsl-setup.sh (ROS 2 Humble): die Nav2-Plugins aus
# unserer nav2_params.yaml und der Workspace fuer die Simulation.
set -e
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2 Humble fehlt -- zuerst tb3-wsl-setup.sh ausfuehren." >&2
    exit 1
fi
sudo apt-get update -qq
sudo apt-get install -y ros-humble-navigation2 ros-humble-nav2-mppi-controller \
    ros-humble-nav2-rotation-shim-controller ros-humble-rqt-image-view \
    python3-opencv python3-yaml python3-numpy
mkdir -p "$HOME/urmc_sim_ws/src"
echo ""
echo "Fertig. Start z.B.: bash scripts/sim/sim.sh wt-nav2 tunnel_gap"
