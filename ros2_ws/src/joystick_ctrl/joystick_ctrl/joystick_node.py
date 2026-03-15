#!/usr/bin/env python3
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.parameter import ParameterDescriptor

from interfaces.msg import HoloprojectorState
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class JoystickCtrl(Node):
    def __init__(self) -> None:
        super().__init__("joystick_ctrl")

        # Parameters to make the mapping adjustable without code changes.
        self.joy_topic = self.declare_parameter("joy_topic", "joy").value
        self.command_topic = self.declare_parameter("command_topic", "debug").value
        self.servo1_axis = int(self.declare_parameter("servo1_axis", 0).value)
        self.servo2_axis = int(self.declare_parameter("servo2_axis", 1).value)
        self.servo1_invert = bool(self.declare_parameter("servo1_invert", False).value)
        self.servo2_invert = bool(self.declare_parameter("servo2_invert", True).value)
        self.servo_angle_mid = float(self.declare_parameter("servo_angle_mid", 90.0).value)
        self.servo_angle_range = float(self.declare_parameter("servo_angle_range", 90.0).value)
        self.servo_min = float(self.declare_parameter("servo_min", 0.0).value)
        self.servo_max = float(self.declare_parameter("servo_max", 180.0).value)
        self.deadzone = float(self.declare_parameter("deadzone", 0.05).value)

        # LED and brightness mapping.
        self.default_color: Tuple[int, int, int] = (
            int(self.declare_parameter("default_led_r", 255).value),
            int(self.declare_parameter("default_led_g", 255).value),
            int(self.declare_parameter("default_led_b", 255).value),
        )
        self.current_color: Tuple[int, int, int] = self.default_color
        self.current_brightness = int(self.declare_parameter("default_led_brightness", 128).value)
        self.brightness_step = int(self.declare_parameter("brightness_step", 8).value)
        self.brightness_axis = int(self.declare_parameter("brightness_axis", -1).value)

        # Button mapping defaults to common gamepad layout (A/B/X/Y + bumpers).
        self.red_button = int(self.declare_parameter("red_button", 0).value)
        self.green_button = int(self.declare_parameter("green_button", 1).value)
        self.blue_button = int(self.declare_parameter("blue_button", 2).value)
        self.white_button = int(self.declare_parameter("white_button", 3).value)
        self.brightness_up_button = int(self.declare_parameter("brightness_up_button", 5).value)
        self.brightness_down_button = int(self.declare_parameter("brightness_down_button", 4).value)

        # Dome mapping (left stick by default). Publish both angle and speed so callers can pick the mode.
        self.dome_seek_topic = self.declare_parameter("dome_seek_topic", "dome/seek").value
        self.dome_speed_topic = self.declare_parameter("dome_speed_topic", "dome/speed").value
        self.dome_axis = int(self.declare_parameter("dome_axis", 0).value)
        self.dome_invert = bool(self.declare_parameter("dome_invert", False).value)
        self.dome_range_deg = float(self.declare_parameter("dome_range_deg", 180.0).value)
        self.dome_offset_deg = float(self.declare_parameter("dome_offset_deg", 0.0).value)
        self.dome_max_speed_dps = float(
            self.declare_parameter(
                "dome_max_speed_dps",
                90.0,
                descriptor=ParameterDescriptor(dynamic_typing=True),
            ).value
        )

        self.prev_buttons: List[int] = []
        self.missing_axis_warned = set()

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher_ = self.create_publisher(HoloprojectorState, self.command_topic, cmd_qos)
        self.subscription = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, qos_profile_sensor_data
        )
        self.dome_seek_pub = self.create_publisher(Float64, self.dome_seek_topic, cmd_qos)
        self.dome_speed_pub = self.create_publisher(Float64, self.dome_speed_topic, cmd_qos)

        self.get_logger().info(
            f"Listening for Joy messages on '{self.joy_topic}' "
            f"and publishing HoloprojectorState on '{self.command_topic}'."
        )

    def joy_callback(self, msg: Joy) -> None:
        servo1_angle = self._axis_to_angle(msg.axes, self.servo1_axis, self.servo1_invert)
        servo2_angle = self._axis_to_angle(msg.axes, self.servo2_axis, self.servo2_invert)

        brightness_from_axis = self._axis_to_255(msg.axes, self.brightness_axis)
        if brightness_from_axis is not None:
            self.current_brightness = brightness_from_axis

        self._handle_buttons(msg.buttons)

        out = HoloprojectorState()
        out.servo1_angle = int(round(servo1_angle))
        out.servo2_angle = int(round(servo2_angle))
        out.led_r = int(self.current_color[0])
        out.led_g = int(self.current_color[1])
        out.led_b = int(self.current_color[2])
        out.led_brightness = int(clamp(self.current_brightness, 0, 255))

        self.publisher_.publish(out)

        dome_target = self._dome_target_deg(msg.axes)
        if dome_target is not None:
            dome_msg = Float64()
            dome_msg.data = dome_target
            self.dome_seek_pub.publish(dome_msg)

        dome_speed = self._dome_speed_dps(msg.axes)
        if dome_speed is not None:
            speed_msg = Float64()
            speed_msg.data = dome_speed
            self.dome_speed_pub.publish(speed_msg)

    def _handle_buttons(self, buttons: List[int]) -> None:
        if not self.prev_buttons or len(self.prev_buttons) != len(buttons):
            self.prev_buttons = [0 for _ in buttons]

        color_buttons = [
            (self.red_button, (255, 0, 0)),
            (self.green_button, (0, 255, 0)),
            (self.blue_button, (0, 0, 255)),
            (self.white_button, (255, 255, 255)),
        ]

        for idx, color in color_buttons:
            if self._is_rising_edge(buttons, idx):
                self.current_color = color
                break

        if self._is_rising_edge(buttons, self.brightness_up_button):
            self.current_brightness = min(255, self.current_brightness + self.brightness_step)
        if self._is_rising_edge(buttons, self.brightness_down_button):
            self.current_brightness = max(0, self.current_brightness - self.brightness_step)

        self.prev_buttons = list(buttons)

    def _is_rising_edge(self, buttons: List[int], idx: int) -> bool:
        if idx < 0 or idx >= len(buttons):
            return False
        prev_pressed = self.prev_buttons[idx] == 1 if idx < len(self.prev_buttons) else False
        return buttons[idx] == 1 and not prev_pressed

    def _axis_to_angle(self, axes: List[float], idx: int, invert: bool) -> float:
        value = self._axis_value(axes, idx, warn=True)
        if invert:
            value *= -1.0
        angle = self.servo_angle_mid + value * self.servo_angle_range
        return clamp(angle, self.servo_min, self.servo_max)

    def _dome_target_deg(self, axes: List[float]) -> Optional[float]:
        if self.dome_axis < 0 or self.dome_axis >= len(axes):
            self._warn_missing_axis(self.dome_axis, len(axes))
            return None
        value = axes[self.dome_axis]
        if abs(value) < self.deadzone:
            value = 0.0
        if self.dome_invert:
            value *= -1.0
        return self.dome_offset_deg + value * self.dome_range_deg

    def _axis_to_255(self, axes: List[float], idx: int) -> Optional[int]:
        if idx < 0:
            return None
        if idx >= len(axes):
            self._warn_missing_axis(idx, len(axes))
            return None
        value = self._axis_value(axes, idx, warn=False)
        scaled = int(round(clamp((value + 1.0) * 0.5 * 255, 0, 255)))
        return scaled

    def _dome_speed_dps(self, axes: List[float]) -> Optional[float]:
        if self.dome_axis < 0 or self.dome_axis >= len(axes):
            self._warn_missing_axis(self.dome_axis, len(axes))
            return None
        value = axes[self.dome_axis]
        if abs(value) < self.deadzone:
            value = 0.0
        if self.dome_invert:
            value *= -1.0
        return value * self.dome_max_speed_dps

    def _axis_value(self, axes: List[float], idx: int, warn: bool) -> float:
        if idx < 0 or idx >= len(axes):
            if warn:
                self._warn_missing_axis(idx, len(axes))
            return 0.0
        value = axes[idx]
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _warn_missing_axis(self, idx: int, available: int) -> None:
        if idx in self.missing_axis_warned:
            return
        self.missing_axis_warned.add(idx)
        self.get_logger().warning(
            f"Axis index {idx} not present in Joy message (only {available} axes provided)."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoystickCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
