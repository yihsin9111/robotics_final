import rclpy
from TEL.parameter import LOAD1_SERVO_PIN
from TEL.parameter import LOAD1_SERVO_CMD
from TEL.parameter import LOAD1_SERVO_PAR
from TEL.parameter import SUBSCRIBER_TS
from TEL.servo import Servo
from rclpy.node import Node
from std_msgs.msg import Int8

from TEL.nonblocking_delay import delayMicroseconds
import time

class Load1_servo(Node):

    def __init__(self):
        self.servo = Servo(LOAD1_SERVO_PIN.servo_pin.value, LOAD1_SERVO_PAR.servo_pwm_period)
    
        super().__init__('load1_servo_sub')
        self.subscription = self.create_subscription(
                Int8,
                'Load1_servo',
                self.listener_callback,
                1)
        self.subscription

    def listener_callback(self, msg):
        # self.get_logger().info("msg: %d" % msg.data)
        if msg.data == LOAD1_SERVO_CMD.RELEASE:
            self.get_logger().info("release")
            self.release()
            # self.get_logger().info("stop")
        elif msg.data == LOAD1_SERVO_CMD.ARMED:
            self.get_logger().info("armed")
            self.armed()
            # self.get_logger().info("stop")

        else:
            self.get_logger().info("msg: %d" % msg.data)

    def armed(self):
        global SUBSCRIBER_TS
        self.servo.output(LOAD1_SERVO_PAR.servo_armed_pwm_high, SUBSCRIBER_TS)

    def release(self):
        global SUBSCRIBER_TS
        self.servo.output(LOAD1_SERVO_PAR.servo_release_pwm_high, SUBSCRIBER_TS)
        
    def __del__(self):
        del self.servo

def main(args=None):
    rclpy.init(args=args)
    load1_servo_sub = Load1_servo()
    rclpy.spin(load1_servo_sub)

    load1_servo_sub.destroy_node()
    del load1_servo_sub
    rclpy.shutdown()

if __name__ == '__main__':
    main()
