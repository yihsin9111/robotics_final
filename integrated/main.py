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

cameraMatrix = np.array([[9.3507503118225702e+02, 0., 2.5305835155343249e+02],
                         [0., 9.3765863543522573e+02, 3.0807631925655937e+02], [0., 0., 1.]])
distCoeffs = np.array([1.3640642180519275e-01, -5.3139149725040591e-01,
                       -4.5392856265975600e-03, -1.8678725716376961e-02, 9.3728818573236672e-01])


id = 0

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
        orig = img.copy()

        targetMetaData, arucoMetaData, gesture = targetDetector.detect(img)

        cv2.imshow("Aruco", arucoMetaData.image)
        cv2.imshow("Target", targetMetaData.image)
        print(gesture)
        # cv2.waitKey(1)

        # take picture
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.imwrite(f'./image_{id:02d}.jpg', orig)
            print(f'Image {id:02d} saved')
            id += 1

        # print(targetMetaData.center)
        # print(arucoMetaData.tvecs)
