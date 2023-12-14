import cv2
import numpy as np

from TargetDetection import CircleTarget, ArucoTarget
from GestureDetection import Gesture

cameraMatrix = [ [6.8148813235276111e+02, 0., 1.2918934885396354e+03], [0., 6.8350370246497005e+02, 9.7566227421055237e+02], [0., 0., 1.] ]
distCoeffs = [ -3.2460343148028625e-01, 1.0309934732790764e-01, 1.6143827622055185e-03, 1.3462626627824006e-03, -1.3665170713443517e-02 ]

width, height = 640, 480
b, g, r = 59, 51, 172

cap = cv2.VideoCapture(1)
cap.set(3, width)
cap.set(4, height)
cap.set(10, 150)

if not cap.isOpened():
    raise IOError("Cannot open webcam")

circle = CircleTarget(bgr_color=[b, g, r], depth_c=[0, 0, 0])

while True:
    

