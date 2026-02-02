#!/bin/bash
# Script to close both arms simultaneously

source /home/eric/ros2_ws/install/setup.bash

ros2 topic pub --once /top_arm_request interfaces/msg/ArmRequest "{open: false, position_override: false, angle: 0}" &
ros2 topic pub --once /bottom_arm_request interfaces/msg/ArmRequest "{open: false, position_override: false, angle: 0}" &

wait

echo "Both arms close command sent"
