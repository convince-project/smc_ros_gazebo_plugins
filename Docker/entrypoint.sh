#!/bin/bash
export GZ_VERSION=harmonic
export GZ_CONFIG_PATH=/opt/ros/jazzy/opt/gz_sim_vendor/share/gz
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /opt/ros/jazzy/setup.bash
source /home/$USER/smc_gazebo_ws/install/setup.bash

# Execute what was passed into this entrypoint.
exec "$@"