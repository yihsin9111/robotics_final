import sys
import socket
import signal
import time

import cv2
import imagezmq


# sender = imagezmq.ImageSender(connect_to='tcp://192.168.10.13:5555')
sender = imagezmq.ImageSender(connect_to='tcp://*:5555', REQ_REP=False)

rpi_name = socket.gethostname()  # send RPi hostname with each image
cam = cv2.VideoCapture(0)
# cam.set(cv2.CAP_PROP_EXPOSURE, 40)

if not cam.isOpened():
    print("Could not open camera.")
    sys.exit(1)


def signal_handler(sig, frame):
    print("Closing camera.")
    cam.release()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

print("Camera opened.")

fps = 60
delay = 1 / fps

count = 0
# jpeg_quality = 70
jpeg_quality = 95

while True:
    ret, image = cam.read()
    if ret:
        ret_code, jpg_buffer = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])

        sender.send_jpg(rpi_name, jpg_buffer)
        print(f"Image {count} sent.")

        count += 1
        time.sleep(delay)
