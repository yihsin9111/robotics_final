import rclpy
from rclpy.node import Node
from TEL.parameter import SHOOTER_CIM_PIN
from TEL.parameter import SUBSCRIBER_TS
from TEL.nonblocking_delay import delayMicroseconds
from std_msgs.msg import Int8
import lgpio


class Shooter_CIM(Node):

    def __init__(self, pin = SHOOTER_CIM_PIN.pin):
        global SUBSCRIBER_TS
        self.pin = pin
        self.gpio_handle = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.gpio_handle, pin)

        super().__init__('shooter_CIM_sub')
        self.subscription = self.create_subscription(
            Int8,
            'Shooter_CIM_switch',
            self.listener_callback,
            1)
        self.timer = self.create_timer(SUBSCRIBER_TS / 1000, self.timer_callback)
        self.subscription  # prevent unused variable warning
        self.shooting = False
        self.get_cmd = False
        lgpio.gpio_write(self.gpio_handle, self.pin, 0)

    def listener_callback(self, msg):
        global SUBSCRIBER_TS
        self.get_logger().info("msg: %d" % msg.data)
        self.get_cmd = True
        if msg.data == 1:
            lgpio.gpio_write(self.gpio_handle, self.pin, 1)
        else:
            lgpio.gpio_write(self.gpio_handle, self.pin, 0)
        
        delayMicroseconds(SUBSCRIBER_TS * 1e3)
        # lgpio.gpio_write(self.gpio_handle, self.pin, 0)

    def timer_callback(self):
        if not self.get_cmd:
            lgpio.gpio_write(self.gpio_handle, self.pin, 0)
        self.get_cmd = False
 
    def __del__(self):
        lgpio.gpio_write(self.gpio_handle, self.pin, 0)
        lgpio.gpio_free(self.gpio_handle, self.pin)



def main(args=None):
    rclpy.init(args=args)

    shooter_cim = Shooter_CIM()

    rclpy.spin(shooter_cim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    shooter_cim.destroy_node()
    del shooter_cim
    rclpy.shutdown()


if __name__ == '__main__':
    main()
