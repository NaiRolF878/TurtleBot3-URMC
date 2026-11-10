# Einbinden mit: source scripts/sim/sim-env.sh -- in jedem Terminal, das mit der Simulation spricht.
# Nur dieser Rechner und eine eigene Domain: die Simulation darf nie mit Bot oder Live-Rechner reden.
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=99
export SIM_WS="$HOME/urmc_sim_ws"
source /opt/ros/humble/setup.bash
[ -f "$SIM_WS/install/setup.bash" ] && source "$SIM_WS/install/setup.bash"
