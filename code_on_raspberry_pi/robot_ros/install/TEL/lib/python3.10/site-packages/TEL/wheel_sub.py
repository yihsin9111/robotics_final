import rclpy
from TEL.parameter import WHEEL_CMD
from TEL.parameter import SUBSCRIBER_TS
from TEL.parameter import WHEEL_PIN
from rclpy.node import Node
from std_msgs.msg import Int16

from TEL.nonblocking_delay import delayMicroseconds
import lgpio
import time

class Wheel_sub(Node):

    def __init__(self, PWM_period = 5000, PWM_full = 100, PWM_zero = 0, PWM_factor = 1):
        global SUBSCRIBER_TS
        self.PWM_period = PWM_period
        self.PWM_full = PWM_full
        self.PWM_zero = PWM_zero
        self.PWM_factor = PWM_factor

        self.left_frontwheel_forward_pin = WHEEL_PIN.left_frontwheel_LPWM
        self.left_frontwheel_backward_pin = WHEEL_PIN.left_frontwheel_RPWM

        self.right_frontwheel_forward_pin = WHEEL_PIN.right_frontwheel_RPWM
        self.right_frontwheel_backward_pin = WHEEL_PIN.right_frontwheel_LPWM
    
        self.left_backwheel_forward_pin = WHEEL_PIN.left_backwheel_LPWM
        self.left_backwheel_backward_pin = WHEEL_PIN.left_backwheel_RPWM
    
        self.right_backwheel_forward_pin = WHEEL_PIN.right_backwheel_RPWM
        self.right_backwheel_backward_pin = WHEEL_PIN.right_backwheel_LPWM

        self.pins = [self.left_frontwheel_forward_pin,
                     self.left_frontwheel_backward_pin,
                     self.right_frontwheel_forward_pin,
                     self.right_frontwheel_backward_pin,
                     self.left_backwheel_forward_pin,
                     self.left_backwheel_backward_pin,
                     self.right_backwheel_forward_pin,
                     self.right_backwheel_backward_pin]

        self.gpio_handle = lgpio.gpiochip_open(0)
    
        super().__init__('wheel_sub')
        # WHEEL_CMD = Wheel_cmd()
        self.subscription = self.create_subscription(
                Int16,
                'wheel_cmd',
                self.listener_callback,
                1)
        self.subscription
        
        lgpio.group_claim_output(self.gpio_handle, self.pins)
        self.group_leader = self.left_frontwheel_forward_pin
        
    def listener_callback(self, msg):
        cmd = msg.data >> 8
        speed = msg.data & 0b11111111
        if   cmd == WHEEL_CMD.STOP:
            level = 0b00000000
        elif cmd == WHEEL_CMD.FORWARD:
            level = 0b10101010
        elif cmd == WHEEL_CMD.BACK:
            level = 0b01010101
        elif cmd == WHEEL_CMD.LEFT:
            level = 0b01101001
        elif cmd == WHEEL_CMD.RIGHT:
            level = 0b10010110
        elif cmd == WHEEL_CMD.TURNLEFT:
            level = 0b10011001
        elif cmd == WHEEL_CMD.TURNRIGHT:
            level = 0b01100110
        else:
            level = 0b00000000
            self.get_logger().warn('Invalid command %d' % cmd)
        self.get_logger().info('Your input: cmd = %d, speed = %d' % (cmd, speed))
        self.PWMoutput(speed, level)
    
    def __del__(self):
        lgpio.group_free(self.gpio_handle, self.group_leader)    
        
    def PWMoutput(self, speed, PWMdata):
        global SUBSCRIBER_TS
        PWM_holdup = self.PWM_period * (speed - self.PWM_zero) / (self.PWM_full - self.PWM_zero)
        PWM_holdup = PWM_holdup * self.PWM_factor

        start_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        end_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)

        while (end_time - start_time < SUBSCRIBER_TS * 1000000):
            lgpio.group_write(self.gpio_handle, self.group_leader, PWMdata)
            delayMicroseconds(PWM_holdup)
            lgpio.group_write(self.gpio_handle, self.group_leader, 0)
            delayMicroseconds(self.PWM_period - PWM_holdup)
            end_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)

def main(args=None):
    rclpy.init(args=args)
    wheel_sub = Wheel_sub()
    rclpy.spin(wheel_sub)

    wheel_sub.destroy_node()
    del wheel_sub
    rclpy.shutdown()

if __name__ == '__main__':
    main()
