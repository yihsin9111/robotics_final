import math
from gpiozero import DistanceSensor

import time
import os
import socket
from dotenv import load_dotenv

ultrasonic = DistanceSensor(echo = 17, trigger = 4)

def get_angle(dis=10):
    h = 0
    for i in range(10):
        h += ultrasonic.distance
    h /= 10
    h -= 0.01
    degree = math.atan(h/dis) * 180 / math.pi
    print(f"degree: {degree}, h: {h}")
    return degree


load_dotenv()
HOST = os.getenv('HOST') or 'localhost'
PORT = int(os.getenv('PORT') or 8000)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Listening on {HOST}:{PORT}")

    while True:
        conn, addr = s.accept()
        with conn:
            print('Connected by', addr)
            while True:
                try:
                    conn.sendall((str(get_angle(dis = 0.48)) + "\n").encode())
                    conn.sendall((str(get_angle(dis = 0.48)) + "\n").encode())
                    time.sleep(0.01)
                except:
                    break

