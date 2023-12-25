from typing import Tuple
import socket

from .types import Action


def move(server_params: Tuple[str, int], state: Tuple[int, int]):
    state_str = str(state).strip("()")
    message = f"MOVE:{state_str}"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(server_params)
        s.sendall(message.encode())
    print(message)


def turn(server_params: Tuple[str, int], state: Tuple[int, int]):
    state_str = str(state).strip("()")
    message = f"TURN:{state_str}"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(server_params)
        s.sendall(message.encode())
    print(message)


def shooter(server_params: Tuple[str, int], state: Tuple[int, int]):
    state_str = str(state).strip("()")
    message = f"SHOOTER:{state_str}"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(server_params)
        s.sendall(message.encode())
    print(message)


def shoot(server_params: Tuple[str, int], state: Tuple[int, int]):
    state_str = str(state).strip("()")
    message = f"SHOOT:{state_str}"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(server_params)
        s.sendall(message.encode())
    print(message)


def shooter_rise(server_params: Tuple[str, int], state: Tuple[int, int]):
    state_str = str(state).strip("()")
    message = f"SHOOTER_RISE:{state_str}"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(server_params)
        s.sendall(message.encode())
    print(message)


def rise(server_params: Tuple[str, int], state: Tuple[int, int]):
    state_str = str(state).strip("()")
    message = f"RISE:{state_str}"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(server_params)
        s.sendall(message.encode())
    print(message)


def load(server_params: Tuple[str, int], state: Tuple[int, int]):
    state_str = str(state).strip("()")
    message = f"LOAD:{state_str}"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(server_params)
        s.sendall(message.encode())
    print(message)


def turn_camera(server_params: Tuple[str, int], state: Tuple[int, int]):
    state_str = str(state).strip("()")
    message = f"TURN_CAMERA:{state_str}"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(server_params)
        s.sendall(message.encode())
    print(message)


Actions = dict(
    MOVE=Action(move),
    TURN=Action(turn),
    SHOOTER=Action(shooter),
    SHOOT=Action(shoot),
    SHOOTER_RISE=Action(shooter_rise),
    RISE=Action(rise),
    LOAD=Action(load),
    TURN_CAMERA=Action(turn_camera),
)
