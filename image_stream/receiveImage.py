import cv2
import imagezmq
import numpy as np


# image_hub = imagezmq.ImageHub()
image_hub = imagezmq.ImageHub(open_port="tcp://192.168.10.13:5555", REQ_REP=False)
count = 0

while True:
    image_hub.init_pubsub("tcp://192.168.10.29:5555")
    rpi_name, jpg_buffer = image_hub.recv_jpg()

    print(f"Received image {count}.")
    count += 1

    image = cv2.imdecode(np.frombuffer(jpg_buffer, dtype='uint8'), -1)
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # TODO: Image processing here

    cv2.imshow(rpi_name, image)  # 1 window for each RPi
    cv2.waitKey(1)
