from typing import Tuple

from ..types.route import ActionRoute


def move(command: Tuple[int, int]):

    print(f"move: {command}")

moveRoute = ActionRoute(
    initial_state=(0, 0),
    run=move
)
