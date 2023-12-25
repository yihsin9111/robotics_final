import rclpy
from TEL.parameter import LINEAR_PIN
from TEL.parameter import LINEAR_CMD
from TEL.parameter import SUBSCRIBER_TS
from rclpy.node import Node
from std_msgs.msg import Int8
import lgpio
from TEL.nonblocking_delay import delayMicroseconds
import time

class Linear(Node):

    def __init__(self):
        global SUBSCRIBER_TS
        self.red_pin = LINEAR_PIN.red_pin
        self.black_pin = LINEAR_PIN.black_pin

        self.gpio_handle = lgpio.gpiochip_open(0)
        lgpio.group_claim_output(self.gpio_handle, [self.red_pin, self.black_pin])

        super().__init__('linear_sub')
        self.subscription = self.create_subscription(
                Int8,
                'Linear',
                self.listener_callback,
                1)
        self.subscription

        self.get_cmd = False
        self.timer = self.create_timer(SUBSCRIBER_TS / 1000, self.timer_callback)
        self.stop()

    def listener_callback(self, msg):
        self.get_logger().info("msg: %d" % msg.data)
        if msg.data == LINEAR_CMD.PUSHUP:
            # self.get_logger().info("release")
            self.pushup()
            # self.get_logger().info("stop")
        elif msg.data == LINEAR_CMD.PULLDOWN:
            # self.get_logger().info("armed")
            self.pulldown()
            # self.get_logger().info("stop")

    def timer_callback(self):
        if not self.get_cmd:
            self.stop()
        self.get_cmd = False

    def pushup(self):
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
    linear_sub = Linear()
    rclpy.spin(linear_sub)

    linear_sub.destroy_node()
    del linear_sub
    rclpy.shutdown()

if __name__ == '__main__':
    main()
