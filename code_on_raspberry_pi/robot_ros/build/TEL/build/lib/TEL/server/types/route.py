from typing import Callable, Awaitable, Tuple


class ActionRoute:
    def __init__(
            self,
            initial_state: Tuple[int, int],
            run: Callable[[Tuple[int, int], object], None]):
        self.initial_state = initial_state
        self.run = run

    def __str__(self):
        return f"Route(initial_state={self.initial_state}, run={self.run})"

    def __repr__(self):
        return self.__str__()

    def __call__(self, *args, **kwargs):
        return self.run(*args, **kwargs)
