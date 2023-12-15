import socket
import time

import cv2
from imutils.video import VideoStream
import imagezmq


# sender = imagezmq.ImageSender(connect_to='tcp://192.168.10.13:5555')
sender = imagezmq.ImageSender(connect_to='tcp://*:5555', REQ_REP=False)

rpi_name = socket.gethostname()  # send RPi hostname with each image
cam = VideoStream(src=0).start()
print("Camera opened.")

fps = 30
delay = 1 / fps

count = 0
jpeg_quality = 70

while True:
    image = cam.read()
    if image is not None:
        ret_code, jpg_buffer = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])

        sender.send_jpg(rpi_name, jpg_buffer)
        print(f"Image {count} sent.")

        count += 1
        time.sleep(delay)
