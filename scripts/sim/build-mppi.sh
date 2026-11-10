#!/bin/bash
# Baut Nav2-MPPI ohne AVX2/FMA nach ~/nav2_mppi_ws. Das Paket aus apt nutzt diese Befehle;
# CPUs ohne sie (z.B. Pentium 3558U) brechen mit "illegal instruction" ab (controller_server,
# Code -4). sim-env.sh bindet das Ergebnis ein, wenn es existiert. Dauert auf 2 Kernen lange.
set -e
WS="$HOME/nav2_mppi_ws"
VER="$(dpkg-query -W -f='${Version}' ros-humble-nav2-mppi-controller | cut -d- -f1)"
mkdir -p "$WS/src"
cd "$WS/src"
if [ ! -d navigation2 ]; then
    git clone -q --depth 1 --branch "$VER" https://github.com/ros-navigation/navigation2.git
fi
sed -i 's/^\(\s*\)add_compile_options(-mavx2)/\1# ohne AVX2 (build-mppi.sh)/; s/^\(\s*\)add_compile_options(-mfma)/\1# ohne FMA (build-mppi.sh)/' \
    navigation2/nav2_mppi_controller/CMakeLists.txt
cd "$WS"
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 colcon build --packages-select nav2_mppi_controller \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
echo "Fertig: $WS (Version $VER, ohne AVX2/FMA)."
