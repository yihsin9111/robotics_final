from typing import Callable, Awaitable, Tuple

class Action:
    def __init__(self, run: Callable[[Tuple[str, int], Tuple[int, int]], None]):
        self.run = run

    def __str__(self):
        return f"Action(run={self.run})"

    def __repr__(self):
        return self.__str__()

    def __call__(self, *args, **kwargs):
        return self.run(*args, **kwargs)

