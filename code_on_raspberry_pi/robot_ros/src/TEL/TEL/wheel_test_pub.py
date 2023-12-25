import rclpy
import TEL.parameter
from rclpy.node import Node

from std_msgs.msg import Int16

class Wheel_test_pub(Node):

    def __init__(self):
        super().__init__('wheel_test_pub')
        self.publisher = self.create_publisher(Int16, 'wheel_cmd', 1)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int16()
        msg.data = ((( self.i // 200 ) % 7) << 8) + 25
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: %x' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    wheel_test_pub = Wheel_test_pub()
    rclpy.spin(wheel_test_pub)

    wheel_test_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
