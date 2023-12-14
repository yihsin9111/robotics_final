import cv2
import numpy as np
from cv2 import aruco

# calibration parameters
cameraMatrix = [ [6.8148813235276111e+02, 0., 1.2918934885396354e+03], [0., 6.8350370246497005e+02, 9.7566227421055237e+02], [0., 0., 1.] ]
distCoeffs = [ -3.2460343148028625e-01, 1.0309934732790764e-01, 1.6143827622055185e-03, 1.3462626627824006e-03, -1.3665170713443517e-02 ]

# target classes
class CircleTarget:
    def __init__(self, bgr_color=[59,51,172], depth_c=[0,0,0]):

        self.x = 0
        self.y = 0
        self.radius = 0

        # target position relative to camera
        # set camera as origin
        self.x_real = 0
        self.y_real = 0
        self.z_real = 0

        # depth, width, pixel
        self.f = depth_c[0]*depth_c[2]/depth_c[1]
        self.w = depth_c[1]

        # set color detection threshold
        color = np.uint8([[[bgr_color[0], bgr_color[1], bgr_color[2]]]])
        hue = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        low, high = [hue - 10, 100, 100], [hue + 10, 255, 255]
        self.low, self.high = np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)

    def get_target(self, img):

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
            self.z = self.f/radius*self.w
            
            # get x and y coordinates from z coordinate and camera matrix
            img_pos = np.array([self.x, self.y, self.z])
            cam_mat = np.array(cameraMatrix)
            self.x_real, self.y_real, self.z_real = np.matmul(np.linalg.inv(cam_mat), np.transpose(img_pos))

            # draw circle around contour
            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)

        return img


class ArucoTarget:
    def __init__(self, aruco_id):
        self.id = aruco_id
        self.tvec = None
        self.rvec = None

    def get_target(self, img):
        # convert to grayscale
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        parameters =  aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)

        # if aruco marker exists
        if markerIds is not None:
            for i in range(len(markerIds)):
                if markerIds[i][0] == self.id:
                    self.tvec = detector.get_tvec(markerCorners[i][0])
                    self.rvec = detector.get_rvec(markerCorners[i][0])
                    frame_markers = aruco.drawDetectedMarkers(img, markerCorners, markerIds)
                    frame_markers = aruco.drawAxis(frame_markers, cameraMatrix, distCoeffs, self.rvec, self.tvec, 0.1)
                    break

        frame_markers = aruco.drawDetectedMarkers(img, markerCorners, markerIds)
        [self.rvecs, self.tvecs, _objPoints] = aruco.estimatePoseSingleMarkers(markerCorners, 
                                                                     0.05, 
                                                                     np.float32(cameraMatrix),
                                                                     np.float32(distCoeffs))

        return img
    
    


