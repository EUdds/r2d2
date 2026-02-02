#!/bin/bash
# Script to open the top arm

source /home/eric/ros2_ws/install/setup.bash

ros2 topic pub --once /top_arm_request interfaces/msg/ArmRequest "{open: true, position_override: false, angle: 0}"

echo "Top arm open command sent"
