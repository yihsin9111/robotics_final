# for PID paraameters, first tune P, then D, with necessary then I

depth_Kp = 1
depth_Ki = 0        # never ever set to nonzero except you very sure you need this
depth_Kd = 0

rotation_Kp = 200
rotation_Ki = 0      # never ever set to nonzero except you very sure you need this
rotation_Kd = 0

# output value limits
output_limit = 20   # in cmd unit (0 ~ 255)
output_lower_limit = 7
# image update parameters
camera_update_time = 0.5    # in seconds

# from camera view, x to right, y to down, z to forward
# setup and tolerance
depth_target_cm = 0       # z obtained by camera
depth_tolerance_cm = 5    # if in target +- tolerance, the robot would not move

rotation_target_cm = 0    # x obtained by camera
rotation_outer_tolerance_deg = 4 # outer tolerance: start controller if greater then
rotation_inner_tolerance_deg = 1.5 # inner tolerance: stop controller if inside
