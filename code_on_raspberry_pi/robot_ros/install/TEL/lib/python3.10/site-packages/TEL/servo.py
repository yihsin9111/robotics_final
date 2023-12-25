import lgpio
import time
from TEL.nonblocking_delay import delayMicroseconds

class Servo():

    def __init__(self, cmd_pin, PWM_period_us):
        if not isinstance(cmd_pin, list):
            cmd_pin = [cmd_pin]

        self.cmd_pin = cmd_pin
        self.PWM_period_us = PWM_period_us

        self.gpio_handle = lgpio.gpiochip_open(0)
        lgpio.group_claim_output(self.gpio_handle, self.cmd_pin)
        lgpio.group_write(self.gpio_handle, self.cmd_pin[0], 0)

        self.high_cmd = 2 ** len(cmd_pin) - 1

    def output(self, PWM_high_us, length_ms):
        if (PWM_high_us > self.PWM_period_us):
            PWM_high_us = self.PWM_period_us

        start_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        end_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        while (end_time - start_time) < length_ms * 1e6:
            lgpio.group_write(self.gpio_handle, self.cmd_pin[0], self.high_cmd)
            delayMicroseconds(PWM_high_us)
            lgpio.group_write(self.gpio_handle, self.cmd_pin[0], 0)
            delayMicroseconds(self.PWM_period_us - PWM_high_us)
            end_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)

    def stop(self):
        lgpio.group_write(self.gpio_handle, self.cmd_pin[0], 0)

    def __del__(self):
        lgpio.group_write(self.gpio_handle, self.cmd_pin[0], 0)
        lgpio.group_free(self.gpio_handle, self.cmd_pin[0])

def main():
    servo = Servo([20, 21], 3000)
    servo.output(1500, 1000)
    servo.output(2000, 2000)
    servo.output(1000, 3000)
    del servo

if __name__ == "__main__":
    main()




