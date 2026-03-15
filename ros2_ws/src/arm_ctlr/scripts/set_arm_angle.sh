#!/bin/bash
# Script to set a specific arm to a custom angle
# Usage: ./set_arm_angle.sh <top|bottom> <angle>
# Example: ./set_arm_angle.sh top 90

source /home/eric/ros2_ws/install/setup.bash

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <top|bottom> <angle>"
    echo "Example: $0 top 90"
    exit 1
fi

ARM=$1
ANGLE=$2

if [ "$ARM" == "top" ]; then
    TOPIC="/top_arm_request"
elif [ "$ARM" == "bottom" ]; then
    TOPIC="/bottom_arm_request"
else
    echo "Error: First argument must be 'top' or 'bottom'"
    exit 1
fi

if ! [[ "$ANGLE" =~ ^[0-9]+$ ]] || [ "$ANGLE" -lt 0 ] || [ "$ANGLE" -gt 180 ]; then
    echo "Error: Angle must be between 0 and 180"
    exit 1
fi

ros2 topic pub --once $TOPIC interfaces/msg/ArmRequest "{open: false, position_override: true, angle: $ANGLE}"

echo "$ARM arm set to $ANGLE degrees"
