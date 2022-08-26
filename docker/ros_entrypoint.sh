#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "$ROS_WS/install/setup.bash" --

# create robot_config.yaml based on robot_ip environment variable
if [[ -z $robot_ip ]]; then
    echo "[ERROR] robot_ip environment variable is not set. Must be set to construct the needed robot config"
    exit 1
else
    # echo $robot_ip
    python3 ~/create_rc.py -ip $robot_ip
fi

exec "$@"