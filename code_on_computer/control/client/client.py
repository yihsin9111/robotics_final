import os
from dotenv import load_dotenv

from .actions import Actions

CONTROL_ID = {
    "L_X": 3,
    "L_Y": 4,
    "R_X": 5,
    "R_Y": 6,
    "RIGHT": 7,
    "LEFT": 8,
    "FORWARD": 9,
    "BACKWARD": 10,
    "Y": 11,
    "B": 12,
    "A": 13,
    "X": 14,
    "L_BUMPER": 15,
    "R_BUMPER": 16,
    "L_TRIGGER": 17,
    "R_TRIGGER": 18,
}

ACTIONS = [
    "MOVE",
    "TURN",
    "SHOOTER",
    "SHOOT",
    "SHOOTER_RISE",
    "RISE",
    "LOAD",
    "TURN_CAMERA",
]


if __name__ == "__main__":

    load_dotenv()
    SERVER_HOST = os.getenv("SERVER_HOST") or "localhost"
    SERVER_PORT = int(os.getenv("SERVER_PORT") or 8000)

    print(SERVER_HOST)
    print(SERVER_PORT)

    server_params = (SERVER_HOST, SERVER_PORT)

    # actionsState["TURN_CAMERA"] = (0, -255)
    # if actionsState["MOVE"] is not None:
    #     Actions["MOVE"](server_params, actionsState["MOVE"])
    # if actionsState["TURN"] is not None:
    #     Actions["TURN"](server_params, actionsState["TURN"])
    # if actionsState["SHOOTER"] is not None:
    #     Actions["SHOOTER"](server_params, actionsState["SHOOTER"])
    # if actionsState["SHOOT"] is not None:
    #     Actions["SHOOT"](server_params, actionsState["SHOOT"])
    #     Actions["SHOOT"](server_params, (0, 0))
    # if actionsState["SHOOTER_RISE"] is not None:
    #     Actions["SHOOTER_RISE"](server_params, actionsState["SHOOTER_RISE"])
    # if actionsState["RISE"] is not None:
    #     Actions["RISE"](server_params, actionsState["RISE"])
    # if actionsState["LOAD"] is not None:
    #     Actions["LOAD"](server_params, actionsState["LOAD"])
    # if actionsState["TURN_CAMERA"] is not None:
    #     Actions["TURN_CAMERA"](server_params, actionsState["TURN_CAMERA"])
