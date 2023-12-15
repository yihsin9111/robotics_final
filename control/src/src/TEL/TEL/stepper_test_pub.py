import rclpy
import TEL.parameter
from rclpy.node import Node

from std_msgs.msg import Int8

class Stepper_test_pub(Node):

    def __init__(self):
        super().__init__('stepper_test_pub')
        self.publisher = self.create_publisher(Int8, 'stepper_cmd', 1)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 500

    def timer_callback(self):
        msg = Int8()
        msg.data = (self.i // 500) % 3 - 1
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: %x' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    stepper_test_pub = Stepper_test_pub()
    rclpy.spin(stepper_test_pub)

    stepper_test_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
