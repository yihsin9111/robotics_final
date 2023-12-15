import cv2
import numpy as np
import imagezmq

from target import Detector

# target color
targetColorRGB = np.array([[[59, 51, 172]]], dtype=np.uint8)

# object detection threshold
targetColorHSV = cv2.cvtColor(targetColorRGB, cv2.COLOR_BGR2HSV)
hue = targetColorHSV[0][0][0]

low, high = [hue - 10, 100, 100], [hue + 10, 255, 255]
low, high = np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)

cameraMatrix = np.array([[6.8148813235276111e+02, 0., 1.2918934885396354e+03],
                         [0., 6.8350370246497005e+02, 9.7566227421055237e+02], [0., 0., 1.]])
distCoeffs = np.array([-3.2460343148028625e-01, 1.0309934732790764e-01,
                       1.6143827622055185e-03, 1.3462626627824006e-03, -1.3665170713443517e-02])


if __name__ == "__main__":
    targetDetector = Detector(
        targetHSVRange=(low, high),
        cameraMatrix=cameraMatrix,
        distCoeffs=distCoeffs,
    )

    image_hub = imagezmq.ImageHub(
        open_port="tcp://192.168.10.13:5555", REQ_REP=False)

    while True:
        image_hub.init_pubsub("tcp://192.168.10.29:5555")
        rpi_name, jpg_buffer = image_hub.recv_jpg()

        img = cv2.imdecode(np.frombuffer(jpg_buffer, dtype='uint8'), -1)

        targetMetaData, arucoMetaData = targetDetector.detect(img)

        cv2.imshow("Aruco", targetMetaData.image)
        cv2.imshow("Target", arucoMetaData.image)
        cv2.waitKey(1)

        # print(targetMetaData.center)
        print(arucoMetaData.tvecs)
