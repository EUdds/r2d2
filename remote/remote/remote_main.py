import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from interfaces.msg import HoloprojectorState

import threading
import sys

class RemoteMain(Node):
    def __init__(self):
        super().__init__('remote_main')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.publisher_ = self.create_publisher(HoloprojectorState, 'debug', qos)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Remote Main Node has been started.')
    
    def timer_callback(self):
        msg = HoloprojectorState()
        msg.servo1_angle = 45
        msg.servo2_angle = 90
        msg.led_r = 255
        msg.led_g = 0
        msg.led_b = 0
        msg.led_brightness = 128
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    remote_main = RemoteMain()

    rclpy.spin(remote_main)

    remote_main.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
