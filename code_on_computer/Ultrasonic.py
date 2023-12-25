import math
from gpiozero import DistanceSensor

ultrasonic = DistanceSensor(echo=17, trigger=4)

def get_angle(dis=10):
    h = ultrasonic.distance
    degree = math.atan(h/dis)*180/math.pi
    print(f"degree: {degree}, h: {h}")
    return degree

while True:
    print(ultrasonic.distance)

