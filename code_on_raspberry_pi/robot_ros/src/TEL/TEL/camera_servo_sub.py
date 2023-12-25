import rclpy
from TEL.parameter import CAMERA_SERVO_PIN
from TEL.parameter import CAMERA_SERVO_CMD
from TEL.parameter import CAMERA_SERVO_PAR
from TEL.parameter import SUBSCRIBER_TS
from TEL.servo import Servo
from rclpy.node import Node
from std_msgs.msg import Int8

from TEL.nonblocking_delay import delayMicroseconds
import time

class Camera_servo(Node):

    def __init__(self):
        global SUBSCRIBER_TS
        self.servo = Servo(CAMERA_SERVO_PIN.servo_pin.value, CAMERA_SERVO_PAR.servo_pwm_period)
    
        super().__init__('camera_servo_sub')
        self.subscription = self.create_subscription(
                Int8,
                'Camera_servo',
                self.listener_callback,
                1)
        self.subscription
        
        self.timer = self.create_timer(SUBSCRIBER_TS / 1000, self.timer_callback)
        self.get_cmd = False
        self.pos = CAMERA_SERVO_PAR.servo_pwm_period / 2
        self.servo.output(self.pos, 1000)
        self.get_logger().info("READY")

    def listener_callback(self, msg):
        self.get_cmd = True
        self.get_logger().info("msg = %d" % msg.data)
        if msg.data == CAMERA_SERVO_CMD.LEFT:
            self.pos += CAMERA_SERVO_PAR.servo_step
            if self.pos > CAMERA_SERVO_PAR.servo_left_pwm_limit:
                self.pos = CAMERA_SERVO_PAR.servo_left_pwm_limit
            self.servo.output(self.pos, CAMERA_SERVO_PAR.tracking_time)
            self.get_logger().info("LEFT")
        
        elif msg.data == CAMERA_SERVO_CMD.RIGHT:
            self.pos -= CAMERA_SERVO_PAR.servo_step
            if self.pos < CAMERA_SERVO_PAR.servo_right_pwm_limit:
                self.pos = CAMERA_SERVO_PAR.servo_right_pwm_limit
            self.servo.output(self.pos, CAMERA_SERVO_PAR.tracking_time)
            self.get_logger().info("RIGHT")
        
        elif msg.data == CAMERA_SERVO_CMD.RESET:
            self.pos = CAMERA_SERVO_PAR.servo_pwm_period // 2
            self.servo.output(self.pos, 750)
            self.get_logger().info("RESET")

        else:
            self.servo.stop()

    def timer_callback(self):
        if not self.get_cmd:
            self.servo.stop()
        self.get_cmd = False
            
        
    def __del__(self):
        del self.servo

def main(args=None):
    rclpy.init(args=args)
    camera_servo_sub = Camera_servo()
    rclpy.spin(camera_servo_sub)

    camera_servo_sub.destroy_node()
    del camera_servo_sub
    rclpy.shutdown()

if __name__ == '__main__':
    main()
