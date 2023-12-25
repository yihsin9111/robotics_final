import os
import socket
from dotenv import load_dotenv

from .routes import Routes
from .types.state import State


if __name__ == '__main__':

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
                    data = conn.recv(128)
                    if not data:
                        break

                    try:
                        data = data.decode('utf-8')
                        route_name, command = data.split(':')

                        route = Routes[route_name]
                        route(command)

                    except ValueError:
                        print(f"Invalid message: {data}")
                        break
