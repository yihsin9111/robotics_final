import rclpy
from TEL.parameter import STEPPER_CMD
from TEL.parameter import SUBSCRIBER_TS
from TEL.parameter import STEPPER_PIN
from TEL.parameter import STEPPER_PAR
from rclpy.node import Node
from std_msgs.msg import Int8

from TEL.nonblocking_delay import delayMicroseconds
import lgpio
import time

class Stepper_sub(Node):

    def __init__(self, pulse_per_rev = STEPPER_PAR.pulse_per_revolution,
                       pulse_pin = STEPPER_PIN.pulse_pin, 
                       direction_pin = STEPPER_PIN.direction_pin,
                       pulse_length_us = STEPPER_PAR.PULSE_US,
                       down_limit_pin = STEPPER_PIN.down_limit_pin,
                       up_limit_pin = STEPPER_PIN.up_limit_pin):
        self.gpio_handle = lgpio.gpiochip_open(0)

        super().__init__('stepper_sub')
        # WHEEL_CMD = Wheel_cmd()
        self.subscription = self.create_subscription(
                Int8,
                'stepper_cmd',
                self.listener_callback,
                1)
        self.subscription
        
        lgpio.gpio_claim_output(self.gpio_handle, pulse_pin)
        lgpio.gpio_claim_output(self.gpio_handle, direction_pin)
        lgpio.gpio_claim_input(self.gpio_handle, up_limit_pin)
        lgpio.gpio_claim_input(self.gpio_handle, down_limit_pin)

        self.pulse_per_rev = pulse_per_rev
        self.pulse_length_us = pulse_length_us
        self.pulse_pin = pulse_pin
        self.direction_pin = direction_pin
        self.down_limit_pin = down_limit_pin
        self.up_limit_pin = up_limit_pin

        
    def listener_callback(self, msg):
        global SUBSCRIBER_TS
        cmd = msg.data

        if   cmd == STEPPER_CMD.STOP:
            self.stop_rotate()
        elif cmd == STEPPER_CMD.UP:
            lgpio.gpio_write(self.gpio_handle, self.direction_pin, 0)
            
            if lgpio.gpio_read(self.gpio_handle, self.up_limit_pin) == 1:
                self.stop_rotate()
            else:
                self.start_rotate()

        elif cmd == STEPPER_CMD.DOWN:
            lgpio.gpio_write(self.gpio_handle, self.direction_pin, 1)
            self.start_rotate()
            # if lgpio.gpio_read(self.gpio_handle, self.down_limit_pin) == 1:
            #     self.stop_rotate()
            # else:
            #     self.start_rotate()
        else:
            self.stop_rotate()
            self.get_logger().warn('Invalid command %d' % cmd)

        # self.get_logger().info('Your input: cmd = %d' % cmd)
    
    def __del__(self):
        lgpio.gpio_free(self.gpio_handle, self.pulse_pin)    
        lgpio.gpio_free(self.gpio_handle, self.direction_pin)    
        lgpio.gpio_free(self.gpio_handle, self.up_limit_pin)
        lgpio.gpio_free(self.gpio_handle, self.down_limit_pin)

    def start_rotate(self, angular_speed_rev_min = STEPPER_PAR.REV_PER_MIN):
        
        # calculate the period between two pin
        us_between_pulse = 60*1000000 / (angular_speed_rev_min * self.pulse_per_rev)
        # print("us_between_pulse: ", us_between_pulse)
        start_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        end_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        while (end_time - start_time < SUBSCRIBER_TS * 1000000):
            lgpio.gpio_write(self.gpio_handle, self.pulse_pin, 1)
            delayMicroseconds(self.pulse_length_us)
            lgpio.gpio_write(self.gpio_handle, self.pulse_pin, 0)
            delayMicroseconds(us_between_pulse - self.pulse_length_us)
            end_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)


    def stop_rotate(self):
        lgpio.gpio_write(self.gpio_handle, self.direction_pin, 0)
        lgpio.gpio_write(self.gpio_handle, self.pulse_pin, 0)  
        delayMicroseconds(SUBSCRIBER_TS * 1000)

def main(args=None):
    rclpy.init(args=args)
    stepper_sub = Stepper_sub()
    rclpy.spin(stepper_sub)

    stepper_sub.destroy_node()
    del test
    rclpy.shutdown()

if __name__ == '__main__':
    main()
