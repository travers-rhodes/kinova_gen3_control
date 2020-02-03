#!/bin/bash
# set the ROS_MASTER_URI to my laptop machine (no reason to run rosmaster on the realtime computer)
export ROS_MASTER_URI="http://helvellyn:11311"
# source the kortex_ros_control_ws workspace
source ~/kortex_ros_control_ws/devel/setup.bash
exec "$@"
