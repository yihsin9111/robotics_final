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

    def detectTarget(self, img):
        # convert to HSV
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # create mask & remove small blobs
        mask = cv2.inRange(imgHSV, self.targetHSVLow, self.targetHSVHigh)

        erodeKernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, erodeKernel, iterations=2)

        dilateKernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(mask, dilateKernel, iterations=2)

        # find contours & centroids
        contours, _ = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
            # print("M:", M)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # draw circle around contour
            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            # draw centroid
            cv2.circle(img, center, 5, (0, 0, 255), -1)
            # label centroid
            cv2.putText(img, "centroid", (center[0] - 25, center[1] - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # update points queue & draw trail
        self.targetTrace.appendleft(center)

        return TargetMetaData(
            image=img,
            mask=mask,
            center=center,
        )

    def detectAruco(self, img):
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(arucoDict, parameters)

        markerCorners, markerIds, _ = detector.detectMarkers(imgHSV)

        frame_markers = aruco.drawDetectedMarkers(
            img, markerCorners, markerIds)
        [rvecs, tvecs, _] = aruco.estimatePoseSingleMarkers(
            markerCorners, 0.05,
            np.array(self.cameraMatrix, dtype=np.float32),
            np.array(self.distCoeffs, dtype=np.float32))

        cv2.putText(img, f"translation matrix: {tvecs}", (25, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        return ArucoMetaData(
            image=img,
            markerCorners=markerCorners,
            markerIds=markerIds,
            rvecs=rvecs,
            tvecs=tvecs,
        )

    def detect(self, img: np.ndarray):
        targetMetaData = self.detectTarget(img.copy())
        arucoMetaData = self.detectAruco(img)

        return targetMetaData, arucoMetaData
