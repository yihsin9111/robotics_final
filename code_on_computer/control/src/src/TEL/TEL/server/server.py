import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import String

import os
import socket
from dotenv import load_dotenv

from .routes import Routes
from .types.state import State


def main_callback(host, port, publisher_dict):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Listening on {host}:{port}")

        while True:
            conn, addr = s.accept()
            with conn:
                print('Connected by', addr)
                while True:
                    data = conn.recv(128)
                    if not data:
                        break

                    try:
                        data = data.decode('utf-8')
                        route_name, command = data.split(':')
                        command = tuple(command.split(", "))

                        route = Routes[route_name]
                        publisher = publisher_dict[route_name]
                        route(command, publisher)

                    except ValueError:
                        print(f"Invalid message: {data}")
                        break


class MinimalPublisher(Node):
    def __init__(self, host, port):
        super().__init__('shooter_CIM_test_pub')
        self.host = host
        self.port = port
        self.publisher_dict = {}
        # MOVE
        # TURN
        # LOAD
        # RISE
        # RISE_SHOOTER
        # SHOOTER
        # SHOOT
        # TURN_CAMERA
        self.publisher_dict["MOVE"] = self.create_publisher(Int16, 'wheel_cmd', 1)
        self.publisher_dict["TURN"] = self.publisher_dict["MOVE"]
        self.publisher_dict["LOAD"] = self.create_publisher(Int8, 'Load1_servo', 1)
        self.publisher_dict["RISE"] = self.create_publisher(Int8, 'stepper_cmd', 1)
        self.publisher_dict["SHOOTER_RISE"] = self.create_publisher(Int8, 'Linear', 1)
        self.publisher_dict["SHOOTER"] = self.create_publisher(Int8, 'Shooter_CIM_switch', 1)
        self.publisher_dict["SHOOT"] = self.create_publisher(String, 'Shooter_servo', 1)
        self.publisher_dict["TURN_CAMERA"] = self.create_publisher(Int8, 'Camera_servo', 1)
        # self.publisher_ = self.create_publisher(Int8, 'Shooter_CIM_switch', 1)
        self.did_run = False
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.did_run = True
        main_callback(self.host, self.port, self.publisher_dict)


def main(args=None):

    load_dotenv()
    # HOST = os.getenv('HOST') or 'localhost'
    # PORT = int(os.getenv('PORT') or 8000)
    HOST = '192.168.0.64'
    PORT = 1484

    rclpy.init(args=args)

    node = MinimalPublisher(HOST, PORT)

    while rclpy.ok() and not node.did_run:
        rclpy.spin_once(node)

    node.timer.cancel()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
