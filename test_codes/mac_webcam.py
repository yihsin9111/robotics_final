import cv2
import argparse
import yaml
import numpy as np
from collections import deque
from cv2 import aruco
# interaction : MediaPipe
# gesture recognition : https://techvidvan.com/tutorials/hand-gesture-recognition-tensorflow-opencv/
width = 640
height = 480

# target color
blue, green, red = 59, 51, 172

# object detection threshold
color = np.uint8([[[blue, green, red]]])
hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
hue = hsv_color[0][0][0]
low, high = [hue - 10, 100, 100], [hue + 10, 255, 255]
low, high = np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)
print(low, high)
pts = deque(maxlen=64)

cap = cv2.VideoCapture(1) # 0 for webcam, 1 for external camera
# camera_ip = "rtsp://username:password@IP/port"
# stream = cv2.VideoCapture(camera_ip)

cap.set(3, width)
cap.set(4, height)
cap.set(10, 150)

'''
with open('params.yaml') as f:
    calibration_dict = yaml.safe_load(f)

print('yaml:', calibration_dict)
'''

cameraMatrix = [ [6.8148813235276111e+02, 0., 1.2918934885396354e+03], [0., 6.8350370246497005e+02, 9.7566227421055237e+02], [0., 0., 1.] ]
distCoeffs = [ -3.2460343148028625e-01, 1.0309934732790764e-01, 1.6143827622055185e-03, 1.3462626627824006e-03, -1.3665170713443517e-02 ]

if not cap.isOpened():
    raise IOError("Cannot open webcam")

while True:
    success, img = cap.read()
    if not success: break

    # convert to grayscale
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # aruco detection
    frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters =  aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)

    # create mask & remove small blobs
    mask = cv2.inRange(img_hsv, low, high)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours & centroids
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = None

    # if contours exist, draw & label largest contour
    if len(contours) > 0:
        # find largest contour
        c = max(contours, key=cv2.contourArea)
        # find center of contour
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        # find moment of contour
        M = cv2.moments(c)
        # find center of contour
        #print("M:", M)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # draw circle around contour
        cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        # draw centroid
        cv2.circle(img, center, 5, (0, 0, 255), -1)
        # label centroid
        cv2.putText(img, "centroid", (center[0] - 25, center[1] - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
    frame_markers = aruco.drawDetectedMarkers(img, markerCorners, markerIds)
    [rvecs, tvecs, _objPoints] = aruco.estimatePoseSingleMarkers(markerCorners, 0.05, np.float32(cameraMatrix), np.float32(distCoeffs))
    cv2.putText(img, f"translation matrix: {tvecs}", (25,25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # update points queue & draw trail
    pts.appendleft(center)

    cv2.imshow("Video", img)
    cv2.imshow("Mask", mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
