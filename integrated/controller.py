from simple_pid import PID
import time
import math
from controller_parameters import *
# all parameters in controller_parameters.py, tune there, tune gentally


class Controller:
    def __init__(self):

        self.depth_Kp = depth_Kp
        # never ever set to nonzero except you very sure you need this
        self.depth_Ki = depth_Ki
        self.depth_Kd = depth_Kd

        self.rotation_Kp = rotation_Kp
        # never ever set to nonzero except you very sure you need this
        self.rotation_Ki = rotation_Ki
        self.rotation_Kd = rotation_Kd

        # output value limits
        self.output_limit = output_limit

        # image update parameters
        self.camera_update_time = camera_update_time

        # from camera view, x to right, y to down, z to forward
        # setup and tolerance
        self.depth_target_cm = depth_target_cm       # z obtained by camera
        # if in target +- tolerance, the robot would not move
        self.depth_tolerance_cm = depth_tolerance_cm

        self.rotation_target_cm = rotation_target_cm    # x obtained by camera
        # outer tolerance: start controller if greater then
        self.rotation_outer_tolerance_rad = math.radians(
            rotation_outer_tolerance_deg)
        self.rotation_inner_tolerance_rad = math.radians(
            rotation_inner_tolerance_deg)  # inner tolerance: stop controller if inside

        self.depth_pid = PID(self.depth_Kp, self.depth_Ki, self.depth_Kd,
                             setpoint=0, sample_time=camera_update_time, auto_mode=False)
        self.depth_pid.output_limits = (-self.output_limit, self.output_limit)

        self.rotation_pid = PID(self.rotation_Kp, self.rotation_Ki, self.rotation_Kd,
                                setpoint=0, sample_time=camera_update_time, auto_mode=False)
        self.rotation_pid.output_limits = (-self.output_limit,
                                           self.output_limit)

        # -1 stands for stop, 0 stands for finish, 1 stands for rotation, 2 stands for depth
        self.status = -1

        self.depth_pid.reset()
        self.rotation_pid.reset()

    def go_to_sight(self, position_cm):
        # during initialize, let the target be in the center of the sight
        # therefore, only rotation is required
        # position send in tuple as (x, y, z)
        # return value: tuple with 3 parameters:
        # first is status (-1 for error, 1 for rotation, 2 for depth, 0 for finish)
        try:
            x, y, z = position_cm
            angle_now = math.atan2(x, z)
            angle_setpoint = math.atan2(self.rotation_target_cm, z)
            self.depth_pid.auto_mode = False
            self.depth_pid.reset()

            # check controller status
            if math.fabs(angle_now - angle_setpoint) > self.rotation_inner_tolerance_rad:
                self.rotation_pid.auto_mode = True
                self.rotation_pid.setpoint = angle_setpoint

                self.status = 1
            else:
                self.rotation_pid.reset()
                self.rotation_pid.auto_mode = False
                self.status = 0

            cmd = 0
            speed = 0
            # give commmand
            if self.status == 1:
                speed = -int(self.rotation_pid(angle_now))
                cmd = 1

        except Exception as e:
            self.status = -1
            self.depth_pid.auto_mode = False
            self.depth_pid.reset()

            self.rotation_pid.auto_mode = False
            self.rotation_pid.reset()

            print(e)
            cmd = 0
            speed = 0

        if math.fabs(speed) < output_lower_limit and speed != 0:
            speed = output_lower_limit
        return (self.status, cmd, speed)

    def set_setpoint(self, rotation_target_input_cm=None, depth_target_input_cm=None):
        # change the setpoint
        # if no such parameter, no change
        if rotation_target_input_cm is not None:
            self.rotation_target_cm = rotation_target_input_cm
        if depth_target_cm is not None:
            self.depth_target_cm = depth_target_input_cm

    def get_cmd(self, position_cm):
        # position send in tuple as (x, y, z)
        # return value: tuple with 3 parameters:
        # first is status (-1 for error, 1 for rotation, 2 for depth, 0 for finish)
        # try:
        x, y, z = position_cm
        angle_now = math.atan2(x, z)
        angle_setpoint = math.atan2(self.rotation_target_cm, z)
        depth_now = z
        # check controller status
        # print(angle_now, angle_setpoint, self.rotation_inner_tolerance_rad)
        if math.fabs(angle_now - angle_setpoint) > self.rotation_outer_tolerance_rad:
            self.depth_pid.reset()
            self.depth_pid.auto_mode = False
            self.rotation_pid.auto_mode = True
            self.rotation_pid.setpoint = angle_setpoint

            self.status = 1
        elif math.fabs(angle_now - angle_setpoint) > self.rotation_inner_tolerance_rad:
            self.status = self.status
            if self.status == -1:
                self.status = 1
                self.depth_pid.reset()
                self.depth_pid.auto_mode = False
                self.rotation_pid.auto_mode = True
                self.rotation_pid.setpoint = angle_setpoint
        elif math.fabs(depth_now - self.depth_target_cm) > self.depth_tolerance_cm:
            self.rotation_pid.reset()
            self.rotation_pid.auto_mode = False
            self.depth_pid.auto_mode = True
            self.depth_pid.setpoint = self.depth_target_cm

            self.status = 2
        else:
            self.status = 0
            self.depth_pid.auto_mode = False
            self.depth_pid.reset()

            self.rotation_pid.auto_mode = False
            self.rotation_pid.reset()

        cmd = 0
        speed = 0
        # give commmand
        if self.status == 1:
            speed = -int(self.rotation_pid(angle_now))
            cmd = 1

        elif self.status == 2:
            if math.fabs(depth_now - self.depth_target_cm) > self.depth_tolerance_cm:
                speed = -int(self.depth_pid(depth_now))
                cmd = 0
            else:
                self.status = 0
        # except Exception as e:
        #     self.status = -1
        #     self.depth_pid.auto_mode = False
        #     self.depth_pid.reset()

        #     self.rotation_pid.auto_mode = False
        #     self.rotation_pid.reset()

        #     print(e)
        #     cmd = 0
        #     speed = 0
        if math.fabs(speed) < output_lower_limit and speed != 0:
            speed = output_lower_limit
        return (self.status, cmd, speed)


def main():
    controller = Controller()
    controller.set_setpoint(depth_target_input_cm=770)
    controller.get_cmd((6, 12, 150))

    # # move the target into sight
    # target_in_sight = -1
    # while target_in_sight != 0:
    #     # position_now = shoot_a_photo()
    #     position_now = (10, 20, 10)
    #     target_in_sight, cmd, speed = controller.go_to_sight(position_now)
    #     # send(cmd, speed)
    #     print("Sending command: %d, %d" % cmd, speed)

    # # calculate_required_depth_and_inclination
    # depth_calculated = 50

    # controller.set_setpoint(depth_target_cm = depth_calculated)

    # depth_arrive = -1
    # while depth_arrive != 0:
    #     position_now = shoot_a_photo()
    #     depth_arrive, cmd, speed = controller.get_cmd(position_now)
    #     # send(cmd, speed)
    #     print("Sending command: %d, %d" % cmd, speed)

    # go to modify inclination


if __name__ == '__main__':
    main()
