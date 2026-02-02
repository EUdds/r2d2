#!/bin/bash
# Script to open both arms simultaneously

source /home/eric/ros2_ws/install/setup.bash

ros2 topic pub --once /top_arm_request interfaces/msg/ArmRequest "{open: true, position_override: false, angle: 0}" &
ros2 topic pub --once /bottom_arm_request interfaces/msg/ArmRequest "{open: true, position_override: false, angle: 0}" &

wait

echo "Both arms open command sent"
