#!/bin/bash

DEVEL_DIR=$HOME/catkin_ws/devel
BAG_DIR=/tmp/

source "${DEVEL_DIR}/setup.bash"


# Launch ROS nodes Gazebo simulator in background
roslaunch cym_device test_hector_quad_random_waypoints.launch bag_dir:=${BAG_DIR} num_waypoints:=100 \
    time_itvl_min:="5.5" dist_min:="4.9" dist_max:="5.1"&
ROSLAUNCH_PID=$!

# Wait for all nodes finish initialization
sleep 15s

# Unpause to start simulation
rosservice call --wait /gazebo/unpause_physics

# Wait for background roslaunch to finish
wait ${ROSLAUNCH_PID}
