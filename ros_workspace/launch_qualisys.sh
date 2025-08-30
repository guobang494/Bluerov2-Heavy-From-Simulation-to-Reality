#!/bin/bash
cd /home/zeb/ros_workspace
source install/setup.bash
roslaunch install/ros-qualisys/share/ros-qualisys/launch/qualisys_bauzil_bringup.launch server_address:=${1:-192.168.0.100} base_port:=${2:-22222} 