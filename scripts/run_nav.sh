#!/bin/bash
# Usage: run_nav.sh [headless True|False]   (default False = Gazebo window on)
# Kills any old simulation, then starts Gazebo + Nav2 with the CNC cell world.
PKG=$HOME/ros2_ws/src/antropi_nav_demo
pkill -f "gz sim" 2>/dev/null; pkill -f /opt/ros/jazzy 2>/dev/null; sleep 3
ros2 launch nav2_bringup tb3_simulation_launch.py \
  slam:=False headless:=${1:-False} \
  params_file:=$PKG/config/nav2_params.yaml \
  map:=$PKG/maps/antropi_cnc_cell.yaml \
  world:=$PKG/worlds/antropi_cnc_cell.sdf \
  x_pose:=-8.4 y_pose:=-5.5 yaw:=0.0
