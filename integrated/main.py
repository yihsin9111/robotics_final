import cv2
import numpy as np
import imagezmq
from dotenv import load_dotenv
import os
import time
import socket

from target import Detector
from frisbee import Frisbee
from controller import Controller
from client import Client

# # target color
# targetColorRGB = np.array([[[59, 51, 172]]], dtype=np.uint8)
#
# # object detection threshold
# targetColorHSV = cv2.cvtColor(targetColorRGB, cv2.COLOR_BGR2HSV)
# hue = targetColorHSV[0][0][0]
#
# low, high = [hue - 10, 100, 100], [hue + 10, 255, 255]
# low, high = np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)
#
# cameraMatrix = np.array([[9.3507503118225702e+02, 0., 2.5305835155343249e+02],
#                          [0., 9.3765863543522573e+02, 3.0807631925655937e+02], [0., 0., 1.]])
# distCoeffs = np.array([1.3640642180519275e-01, -5.3139149725040591e-01,
#                        -4.5392856265975600e-03, -1.8678725716376961e-02, 9.3728818573236672e-01])
#
# id = 0
#
# targetDetector = Detector(
#     targetHSVRange=(low, high),
#     cameraMatrix=cameraMatrix,
#     distCoeffs=distCoeffs,
# )
#
# cap = cv2.VideoCapture(0)
#
# while True:
#     ret, img = cap.read()
#     _, arucoMetaData = targetDetector.detect(img)
#
#     cv2.imshow("Aruco", arucoMetaData.image)
#     cv2.waitKey(1)


def getArucoPos(imageHub: imagezmq.ImageHub, imageHubHost: str, detector: Detector):
    imageHub.init_pubsub(imageHubHost)
    rpi_name, jpg_buffer = imageHub.recv_jpg()

    img = cv2.imdecode(np.frombuffer(jpg_buffer, dtype='uint8'), -1)

    # t0 = time.time()
    _, arucoMetaData = detector.detect(img)
    # print(f"Detection time: {time.time() - t0}")

    cv2.imshow("Aruco", arucoMetaData.image)
    # cv2.imshow("Target", targetMetaData.image)

    cv2.waitKey(1)

    # print(targetMetaData.center)
    # print(arucoMetaData.tvecs)

    return arucoMetaData.tvecs[0][0] if arucoMetaData.tvecs is not None else None


load_dotenv()

SERVER_HOST = os.getenv("SERVER_HOST") or "localhost"
SERVER_PORT = int(os.getenv("SERVER_PORT") or 8000)

imageHubHost = f"tcp://{SERVER_HOST}:5555"


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

targetDetector = Detector(
    targetHSVRange=(low, high),
    cameraMatrix=cameraMatrix,
    distCoeffs=distCoeffs,
)
frisbee = Frisbee()
controller = Controller()
client = Client(SERVER_HOST, SERVER_PORT)

imageHub = imagezmq.ImageHub(open_port=imageHubHost, REQ_REP=False)

IDLE = 0
INITIALIZE = 1
# ROTATE = 2
# FORWARD = 3
MOVE = 2
FINALIZE = 3
TILT = 4
SHOOT = 5

# state = TILT
state = IDLE

target_z = 0.0
target_angle = 0.0
# target_angle = 15.0

ultra_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ultra_server_params = (SERVER_HOST, 1485)
ultra_socket.connect(ultra_server_params)

while True:

    if state == IDLE:
        print("Enter go to start tracking: ", end='')
        if input() == 'go':
            # print("Initializing...")
            state = INITIALIZE

    elif state == INITIALIZE:
        print("Initializing...")

        arucoPos = getArucoPos(imageHub, imageHubHost, targetDetector)
        print("Aruco position: ", arucoPos)

        if arucoPos is None:
            continue

        targetInSight, cmd, speed = controller.go_to_sight(arucoPos)

        if targetInSight == 0:
            target_z = frisbee.get_z(-arucoPos[1] / 100.0) * 100.0
            # target_z = frisbee.get_z(0) * 100.0
            target_angle = frisbee.get_angle()

            print(f"Target z: {target_z}, Target angle: {target_angle}")
            controller.set_setpoint(depth_target_input_cm=target_z)

            # print("Moving to target...")
            state = MOVE
        else:
            print(f"Command: {cmd}, Speed: {speed}")
            try:
                client.run(cmd, speed)
            except:
                pass

    elif state == MOVE:
        print(f"Moving to target: ({target_z}, {target_angle})...")

        arucoPos = getArucoPos(imageHub, imageHubHost, targetDetector)
        print("Aruco position: ", arucoPos)

        if arucoPos is None:
            continue

        depthArrive, cmd, speed = controller.get_cmd(arucoPos)
        print(depthArrive)

        if depthArrive == 0:
            print("Arrived at target.")
            state = FINALIZE
        else:
            print(f"Command: {cmd}, Speed: {speed}")
            try:
                client.run(cmd, speed)
            except:
                pass

    elif state == FINALIZE:
        print("Finalizing...")

        arucoPos = getArucoPos(imageHub, imageHubHost, targetDetector)
        print("Aruco position: ", arucoPos)

        if arucoPos is None:
            continue

        targetInSight, cmd, speed = controller.go_to_sight(arucoPos)

        if targetInSight == 0:
            state = TILT
        else:
            print(f"Command: {cmd}, Speed: {speed}")
            try:
                client.run(cmd, speed)
            except:
                pass


    elif state == TILT:
        print(f"Tilting to angle: {target_angle}")

        message = ultra_socket.recv(128).decode()
        message = message.split("\n")

        if len(message) != 3:
            continue

        message = message[1]

        angle = 0.0
        try:
            angle = float(message)
        except:
            pass

        print(f"Angle: {angle}")
        if abs(target_angle - angle) > 0.3:
            try:
                if angle > target_angle:
                    client.run(4, -255)
                else:
                    client.run(4, 255)
            except:
                pass
        else:
            state = SHOOT

    elif state == SHOOT:
        # print("Shooting...")
        # pass
        client.run(2, 255)
        time.sleep(1.5)
        client.run(3, 255)
        time.sleep(1)
        client.run(2, 255)
        state = IDLE
