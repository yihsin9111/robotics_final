import enum

SUBSCRIBER_TS = 100      # ms
class WHEEL_CMD(enum.IntEnum):
    STOP        = 0
    FORWARD     = 1
    BACK        = 2
    LEFT        = 3
    RIGHT       = 4
    TURNLEFT    = 5
    TURNRIGHT   = 6

class WHEEL_PIN(enum.IntEnum):
    left_frontwheel_RPWM            = 6
    left_frontwheel_LPWM            = 13
    right_frontwheel_RPWM           = 12
    right_frontwheel_LPWM           = 16
    left_backwheel_RPWM             = 19 
    left_backwheel_LPWM             = 26
    right_backwheel_RPWM            = 20
    right_backwheel_LPWM            = 21

# for shooter CIM, send String "SWITCH" to Shooter_CIM_switch

class SHOOTER_CIM_PIN(enum.IntEnum):
    pin = 25

# for shooter servo, send String "SWITCH" to Shooter_servo

class SHOOTER_SERVO_PIN(enum.IntEnum):
    servo1_pin = 27
    servo2_pin = 22

class SHOOTER_SERVO_PAR(enum.IntEnum):
    servo1_pwm_period = 3000     # us
    servo2_pwm_period = 3000     # us
    
    # Position for armed
    servo1_armed_pwm_high = 1000  # us
    servo1_armed_time = 2000 # ms
    servo2_armed_pwm_high = 1000  # us
    servo2_armed_time = 1000 # ms

    time_between_arm_release = 500 # ms   

    # Position for release
    servo1_release_pwm_high = 2500  # us
    servo1_release_time = 1200 # ms
    servo2_release_pwm_high = 2500  # us 
    servo2_release_time = 1200 # ms

class LOAD1_SERVO_PIN(enum.Enum):
    servo_pin = [5, 11]

class LOAD1_SERVO_CMD(enum.IntEnum):
    RELEASE = -1
    ARMED = 1

class LOAD1_SERVO_PAR(enum.IntEnum):
    servo_pwm_period = 3000     # us
    
    # Position for armed
    servo_armed_pwm_high = 2700  # us
    # servo_armed_pwm_high = 1500  # us
    # servo_armed_time = 750 # ms

    # time_between_arm_release = 1000 # ms   

    # Position for release
    servo_release_pwm_high = 300  # us
    # servo_release_pwm_high = 1500  # us
    # servo_release_time = 750 # ms

class CAMERA_SERVO_PIN(enum.IntEnum):
    servo_pin = 4

class CAMERA_SERVO_CMD(enum.IntEnum):
    LEFT = -1
    STOP = 0
    RIGHT = 1
    RESET = 2

class CAMERA_SERVO_PAR(enum.IntEnum):
    servo_pwm_period = 5000     # us
    # Position for armed
    servo_left_pwm_limit = 4000  # us

    servo_step = 20     # us
    tracking_time = SUBSCRIBER_TS # ms   
    # Position for release
    servo_right_pwm_limit = 1000  # us

class UP_LINEAR_CMD(enum.IntEnum):
    PUSHUP = 1
    PULLDOWN = -1

class UP_LINEAR_PIN(enum.IntEnum):
    red_pin = 10
    black_pin = 9

class BOTTOM_LINEAR_CMD(enum.IntEnum):
    PUSHUP = 1
    PULLDOWN = -1

class BOTTOM_LINEAR_PIN(enum.IntEnum):
    red_pin = 23
    black_pin = 24

'''
class LOAD2_SERVO_PIN(enum.Enum):
    servo_pin = [9, 10]


class LOAD2_SERVO_PAR(enum.IntEnum):
    servo_pwm_period = 3000     # us
    
    # Position for armed
    servo_armed_pwm_high = 2500  # us
    servo_armed_time = 750 # ms

    time_between_arm_release = 1000 # ms   

    # Position for release
    servo_release_pwm_high = 1000  # us
    servo_release_time = 750 # ms

class STEPPER_CMD(enum.IntEnum):
    STOP        = 0
    UP          = 1
    DOWN        = -1

class STEPPER_PIN(enum.IntEnum):
    pulse_pin = 23
    direction_pin = 24
    down_limit_pin = 17
    up_limit_pin = 4

class STEPPER_PAR(enum.IntEnum):
    REV_PER_MIN = 450
    PULSE_US    = 50
    pulse_per_revolution = 200
'''


