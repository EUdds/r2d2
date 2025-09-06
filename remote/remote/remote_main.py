import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class RemoteMain(Node):
    def __init__(self):
        super().__init__('remote_main')
        self.publisher_ = self.create_publisher(String, 'debug', 10)
        timer_period = 1.0  # seconds
        self.get_logger().info('Remote Main Node has been started.')
        self.run()
    
    def run(self):
        try:
            while rclpy.ok():
                user_input = input("Enter a message to publish (or 'exit' to quit): ")
                if user_input.lower() == 'exit':
                    break
                msg = String()
                msg.data = user_input
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: "{msg.data}"')
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    remote_main = RemoteMain()
    rclpy.spin(remote_main)
    remote_main.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()