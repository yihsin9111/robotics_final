import cv2
import imagezmq
import numpy as np
from GestureDetection import Gesture
from TargetDetection import CircleTarget, ArucoTarget

# image_hub = imagezmq.ImageHub()
image_hub = imagezmq.ImageHub(open_port="tcp://192.168.10.13:5555", REQ_REP=False)
count = 0

# Target parameters
b, g, r = 59, 51, 172
circle_target = CircleTarget(bgr_color=[b, g, r], depth_c=[1, 1, 1])
aruco_target = ArucoTarget()
gesture = Gesture()
shooter_state = 'waiting'

while True:
    image_hub.init_pubsub("tcp://192.168.10.29:5555")
    rpi_name, jpg_buffer = image_hub.recv_jpg()

    print(f"Received image {count}.")
    count += 1

    image = cv2.imdecode(np.frombuffer(jpg_buffer, dtype='uint8'), -1)
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # TODO: Image processing here
    # gesture control
    circle_target.get_target(image)
    aruco_target.get_target(image)
    gesture.get_gesture(image)

    # gesture control
    if shooter_state == 'waiting':
        if gesture.pred == 'fist':
            shooter_state = 'ready'

    elif shooter_state == 'ready':
        if gesture.pred == 'stop' or gesture.pred == 'live long':
            shooter_state = 'shoot'

    elif shooter_state == 'shoot': 
        shooter_state = 'waiting'

    # return result
    result = {
        "circle_coord" : circle_target.real_coord,
        "aruco_tvec" : aruco_target.tvec,
        "aruco_rvec" : aruco_target.rvec,
        "shooter_state" : shooter_state,
    }

    cv2.imshow(rpi_name, image)  # 1 window for each RPi
    cv2.waitKey(1)
