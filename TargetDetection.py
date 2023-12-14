import cv2
import numpy as np
from cv2 import aruco

# target classes
class CircleTarget:
    def __init__(self, bgr_color=[59,51,172]):

        self.x = 0
        self.y = 0
        self.radius = 0

        # target position relative to camera
        # set camera as origin
        self.x_real = 0
        self.y_real = 0
        self.z_real = 0

        # set color detection threshold
        color = np.uint8([[[bgr_color[0], bgr_color[1], bgr_color[2]]]])
        hue = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        low, high = [hue - 10, 100, 100], [hue + 10, 255, 255]
        self.low, self.high = np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)

    def get_target(self, cap):
        # read frame from webcam
        success, img = cap.read()
        if not success: 
            raise IOError("Cannot open webcam (circle target, get target)")
        
        # convert to hsv
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, self.low, self.high)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours & centroids
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center = None

        # if there exists contour
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # find moment of contour
            M = cv2.moments(c)
            # find center of contour
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # update target position
            self.x = center[0]
            self.y = center[1]
            self.radius = radius

            # get z coordinate from radius and focal length

            # draw circle around contour
            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)

class ArucoTarget:
    def __init__(self, aruco_id):
        self.id = aruco_id
        self.x = 0
        self.y = 0

