#!/bin/bash
# Script to close the bottom arm

source /home/eric/ros2_ws/install/setup.bash

ros2 topic pub --once /bottom_arm_request interfaces/msg/ArmRequest "{open: false, position_override: false, angle: 0}"

echo "Bottom arm close command sent"
