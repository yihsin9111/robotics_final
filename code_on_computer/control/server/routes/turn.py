from typing import Tuple
from ..types.route import ActionRoute


def turn(command: Tuple[int, int]):

    print(f"turn: {command}")

turnRoute = ActionRoute(
    initial_state=(0, 0),
    run=turn
)
