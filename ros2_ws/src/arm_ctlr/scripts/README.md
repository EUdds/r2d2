# Arm Controller Scripts

Command-line scripts for controlling the robotic arms.

## Prerequisites

Make sure the `arm_controller` node is running:
```bash
ros2 run arm_ctlr arm_controller
```

## Available Scripts

### Basic Control

- `open_top_arm.sh` - Opens the top arm (0 degrees)
- `close_top_arm.sh` - Closes the top arm (170 degrees)
- `open_bottom_arm.sh` - Opens the bottom arm (0 degrees)
- `close_bottom_arm.sh` - Closes the bottom arm (170 degrees)
- `open_both_arms.sh` - Opens both arms simultaneously
- `close_both_arms.sh` - Closes both arms simultaneously

### Custom Angle Control

- `set_arm_angle.sh <top|bottom> <angle>` - Set a specific arm to a custom angle (0-180)

## Usage Examples

```bash
# Navigate to scripts directory
cd /home/eric/ros2_ws/src/arm_ctlr/scripts

# Open the top arm
./open_top_arm.sh

# Close the bottom arm
./close_bottom_arm.sh

# Open both arms at once
./open_both_arms.sh

# Set top arm to 90 degrees
./set_arm_angle.sh top 90

# Set bottom arm to 45 degrees
./set_arm_angle.sh bottom 45
```

## Direct ROS2 Commands

You can also use ROS2 commands directly:

```bash
# Open top arm
ros2 topic pub --once /top_arm_request interfaces/msg/ArmRequest "{open: true, position_override: false, angle: 0}"

# Close bottom arm
ros2 topic pub --once /bottom_arm_request interfaces/msg/ArmRequest "{open: false, position_override: false, angle: 0}"

# Set top arm to custom angle (e.g., 90 degrees)
ros2 topic pub --once /top_arm_request interfaces/msg/ArmRequest "{open: false, position_override: true, angle: 90}"
```

## Arm Configuration

- Open angle: 0 degrees
- Closed angle: 170 degrees
- Top arm channel: 15
- Bottom arm channel: 14
- I2C address: 0x40
