import time


class State:
    def __init__(self, state):
        self.state = state
        self.last_update_time = time.time()

    def update(self, state):
        self.state = state
        self.last_update_time = time.time()

    def get(self):
        return self.state

    def get_last_update_time(self):
        return self.last_update_time
