import os

from .actions import Actions

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


class Client:

    def __init__(self, host, port) -> None:

        print(f"Server host: {host}:{port}")
        self.server_params = (host, port)

    def run(self, cmd, speed) -> None:
        Actions[ACTIONS[cmd]](self.server_params, (0, speed))


if __name__ == "__main__":
    pass

    # server_params = (host, port)

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
