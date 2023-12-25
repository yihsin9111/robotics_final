import rclpy
from TEL.parameter import BOTTOM_LINEAR_PIN
from TEL.parameter import BOTTOM_LINEAR_CMD
from TEL.parameter import SUBSCRIBER_TS
from rclpy.node import Node
from std_msgs.msg import Int8
import lgpio
from TEL.nonblocking_delay import delayMicroseconds
import time

class Bottom_linear(Node):

    def __init__(self):
        global SUBSCRIBER_TS
        self.red_pin = BOTTOM_LINEAR_PIN.red_pin
        self.black_pin = BOTTOM_LINEAR_PIN.black_pin

        self.gpio_handle = lgpio.gpiochip_open(0)
        lgpio.group_claim_output(self.gpio_handle, [self.red_pin, self.black_pin])

        super().__init__('bottom_linear_sub')
        self.subscription = self.create_subscription(
                Int8,
                'Bottom_linear',
                self.listener_callback,
                1)
        self.subscription

        self.get_cmd = False
        self.timer = self.create_timer(SUBSCRIBER_TS / 1000, self.timer_callback)
        self.stop()

    def listener_callback(self, msg):
        self.get_logger().info("msg: %d" % msg.data)
        if msg.data == BOTTOM_LINEAR_CMD.PUSHUP:
            # self.get_logger().info("release")
            self.pushbottom()
            # self.get_logger().info("stop")
        elif msg.data == BOTTOM_LINEAR_CMD.PULLDOWN:
            # self.get_logger().info("armed")
            self.pulldown()
            # self.get_logger().info("stop")

    def timer_callback(self):
        if not self.get_cmd:
            self.stop()
        self.get_cmd = False

    def pushbottom(self):
        global SUBSCRIBER_TS
        self.get_cmd = True
        lgpio.group_write(self.gpio_handle, self.red_pin, 1)
        delayMicroseconds(SUBSCRIBER_TS * 1000)

    def pulldown(self):
        global SUBSCRIBER_TS
        self.get_cmd = True
        lgpio.group_write(self.gpio_handle, self.red_pin, 2)
        delayMicroseconds(SUBSCRIBER_TS * 1000)

    def stop(self):
        lgpio.group_write(self.gpio_handle, self.red_pin, 0)

    def __del__(self):
        lgpio.group_free(self.gpio_handle, self.red_pin)

def main(args=None):
    rclpy.init(args=args)
    bottom_linear_sub = Bottom_linear()
    rclpy.spin(bottom_linear_sub)

    bottom_linear_sub.destroy_node()
    del bottom_linear_sub
    rclpy.shutdown()

if __name__ == '__main__':
    main()
