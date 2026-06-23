#!/bin/bash

echo "Starting AUV Simulation..."

cd /workspace

source /opt/ros/jazzy/setup.bash
source install/setup.bash


ros2 launch auv_bringup simulation.launch.py

