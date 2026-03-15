import rclpy
from rclpy.node import Node

from adafruit_servokit import ServoKit

from interfaces.msg import ArmRequest

ARM_MAPPINGS = {
    "top_arm":
    {
        "channel": 15,
        "open_angle": 0,
        "closed_angle": 170,
    },
    "bottom_arm":
    {
        "channel": 14,
        "open_angle": 170,
        "closed_angle": 170,
    },
}

class ArmController(Node):
    
    TOP_ARM_CHANNEL = 15
    BOTTOM_ARM_CHANNEL = 14

    OPEN_ANGLE = 0
    CLOSED_ANGLE = 170

    def __init__(self):
        super().__init__('arm_controller')
        self.kit = ServoKit(channels=16, address=0x40)
        for channel in [self.TOP_ARM_CHANNEL, self.BOTTOM_ARM_CHANNEL]:
            self.kit.servo[channel].set_pulse_width_range(500, 2500)
        
        self.top_arm_sub = self.create_subscription(
            ArmRequest,
            'top_arm_request',
            self.top_arm_request_callback,
            10)
        self.top_arm_sub  # prevent unused variable warning

        self.bottom_arm_sub = self.create_subscription(
            ArmRequest,
            'bottom_arm_request',
            self.bottom_arm_request_callback,
            10)
        self.bottom_arm_sub  # prevent unused variable warning

        self.get_logger().info('ArmController node has been started.')
    
    def top_arm_request_callback(self, msg):
        self.get_logger().info(f'Received top arm request: {msg}')
        if msg.position_override:
            angle = msg.angle
        else:
            if msg.open:
                angle = self.OPEN_ANGLE
            else:
                angle = self.CLOSED_ANGLE
        
        self.kit.servo[self.TOP_ARM_CHANNEL].angle = angle
        
    
    def bottom_arm_request_callback(self, msg):
        self.get_logger().info(f'Received bottom arm request: {msg}')
        if msg.position_override:
            angle = msg.angle
        else:
            if msg.open:
                angle = self.OPEN_ANGLE
            else:
                angle = self.CLOSED_ANGLE
        
        self.kit.servo[self.BOTTOM_ARM_CHANNEL].angle = angle


def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    rclpy.spin(arm_controller)
    arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

