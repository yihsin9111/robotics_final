import rclpy
from TEL.parameter import SHOOTER_SERVO_PIN
from TEL.parameter import SHOOTER_SERVO_PAR
from TEL.servo import Servo
from rclpy.node import Node
from std_msgs.msg import String

from TEL.nonblocking_delay import delayMicroseconds
import time

class Shooter_servo(Node):

    def __init__(self):
        self.servo1 = Servo(SHOOTER_SERVO_PIN.servo1_pin, SHOOTER_SERVO_PAR.servo1_pwm_period)
        self.servo2 = Servo(SHOOTER_SERVO_PIN.servo2_pin, SHOOTER_SERVO_PAR.servo2_pwm_period)
    
        super().__init__('shooter_servo_sub')
        self.subscription = self.create_subscription(
                String,
                'Shooter_servo',
                self.listener_callback,
                1)
        self.subscription
        self.release()

    def listener_callback(self, msg):
        if msg.data == "SWITCH":
            self.armed()
            delayMicroseconds(SHOOTER_SERVO_PAR.time_between_arm_release * 1e3)
            self.release()


    def armed(self):
        self.servo1.output(SHOOTER_SERVO_PAR.servo1_armed_pwm_high, SHOOTER_SERVO_PAR.servo1_armed_time)
        self.servo2.output(SHOOTER_SERVO_PAR.servo2_armed_pwm_high, SHOOTER_SERVO_PAR.servo2_armed_time)

    def release(self):
        self.servo1.output(SHOOTER_SERVO_PAR.servo1_release_pwm_high, SHOOTER_SERVO_PAR.servo1_release_time)
        self.servo2.output(SHOOTER_SERVO_PAR.servo2_release_pwm_high, SHOOTER_SERVO_PAR.servo2_release_time)
        
    def __del__(self):
        del self.servo1
        del self.servo2    

def main(args=None):
    rclpy.init(args=args)
    shooter_servo_sub = Shooter_servo()
    rclpy.spin(shooter_servo_sub)

    shooter_servo_sub.destroy_node()
    del shooter_servo_sub
    rclpy.shutdown()

if __name__ == '__main__':
    main()
