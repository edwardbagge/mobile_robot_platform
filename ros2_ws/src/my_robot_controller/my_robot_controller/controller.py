import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.subscription = self.create_subscription(
            String,
            'cmd',
            self.listener_callback,
            10
        )

        self.get_logger().info('Controller ready')

    def listener_callback(self, msg):
        command = msg.data
        self.send_to_esp32(command)

    def send_to_esp32(self, command):
        self.get_logger().info(f"(SIM) Sending: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()