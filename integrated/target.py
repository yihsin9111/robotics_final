from typing import Optional, Tuple
import cv2
import argparse
import yaml
import numpy as np
from collections import deque
from cv2 import aruco
# interaction : MediaPipe
# gesture recognition : https://techvidvan.com/tutorials/hand-gesture-recognition-tensorflow-opencv/
# width = 640
# height = 480
from TargetDetection import CircleTarget, ArucoTarget
from GestureDetection import Gesture


'''
with open('params.yaml') as f:
    calibration_dict = yaml.safe_load(f)

print('yaml:', calibration_dict)
'''


class TargetMetaData:
    def __init__(
        self,
        image: np.ndarray,
        mask: np.ndarray,
        center: Optional[Tuple[int, int]] = None,
    ):
        self.image = image
        self.mask = mask
        self.center = center


class ArucoMetaData:
    def __init__(
        self,
        image: np.ndarray,
        markerCorners: np.ndarray,
        markerIds: np.ndarray,
        rvecs: np.ndarray,
        tvecs: np.ndarray,
    ):
        self.image = image
        self.markerCorners = markerCorners
        self.markerIds = markerIds
        self.rvecs = rvecs
        self.tvecs = tvecs


class Detector:

    def __init__(
        self,
        targetHSVRange: Tuple[np.ndarray, np.ndarray],
        cameraParamFile: Optional[str] = None,
        cameraMatrix: Optional[np.ndarray] = None,
        distCoeffs: Optional[np.ndarray] = None,
    ):
        if len(targetHSVRange) != 2:
            raise ValueError(
                "targetHSVRange must be a tuple of two numpy arrays.")
        self.targetHSVLow, self.targetHSVHigh = targetHSVRange

        if cameraParamFile is not None:
            # TODO: load camera params from file
            pass
        elif cameraMatrix is not None and distCoeffs is not None:
            self.cameraMatrix = cameraMatrix
            self.distCoeffs = distCoeffs
        else:
            raise ValueError(
                "Must provide either cameraParamFile or cameraMatrix and distCoeffs.")

        self.targetTrace = deque(maxlen=64)

        b, g, r = 59, 51, 172
        self.circle_target = CircleTarget(bgr_color=[b, g, r], depth_c=[1, 1, 1])
        self.aruco_target = ArucoTarget()
        self.gesture = Gesture()

    def detectTarget(self, img):

        circle_target = self.circle_target
        img = circle_target.get_target(img, self.cameraMatrix, self.distCoeffs)

        return TargetMetaData(
            image=img,
            mask=circle_target.mask,
            center=circle_target.real_coord,
        )

    def detectAruco(self, img):
        
        aruco = self.aruco_target
        img = aruco.get_target(img, self.cameraMatrix, self.distCoeffs)

        return ArucoMetaData(
            image=img,
            markerCorners=aruco.markerCorners,
            markerIds=aruco.markerIds,
            rvecs=aruco.rvecs,
            tvecs=aruco.tvecs,
        )
    
    def detectGesture(self, img):
        self.gesture.get_gesture(img)
        return self.gesture.pred

    def detect(self, img: np.ndarray):
        targetMetaData = self.detectTarget(img.copy())
        arucoMetaData = self.detectAruco(img)
        gesturePred = self.detectGesture(img)

        return targetMetaData, arucoMetaData, gesturePred
