# Copyright Ioannis Tsikelis
#
# Script to set up the network interface for CycloneDDS

#!/bin/bash
if [ $# -eq 0 ]
then
    echo "Usage: source setup.sh [network interface (or lo)]"
    return 1
fi

echo "Sourcing installed packages"
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/unitree_ros2_ws/install/setup.bash
source ./install/setup.bash

# For autocompletion to work in terminal
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

echo "Setting up DDS"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Use the requested interface only if it exists, is up, and has an IPv4 address.
# Otherwise fall back to loopback so the stack runs without the robot.
if [ -d "/sys/class/net/$1" ] \
   && [ "$(cat /sys/class/net/$1/operstate 2>/dev/null)" = "up" ] \
   && ip -o -4 addr show "$1" 2>/dev/null | grep -q "inet "; then
    IFACE="$1"
else
    echo "WARNING: network interface '$1' is missing, down, or has no IPv4 — falling back to 'lo' (local-only, no robot)."
    IFACE="lo"
fi

export CYCLONEDDS_URI='<CycloneDDS><Domain><General>
                        <Interfaces>
                        <NetworkInterface name="'$IFACE'" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

if [ "$IFACE" = "lo" ]
then
    # Need to enable multicast if using localhost
    echo "Enabling multicast on lo"
    ip link set lo multicast on
fi

echo "Done, let's try!"
