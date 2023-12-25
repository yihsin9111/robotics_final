from typing import Tuple
from ..types.route import ActionRoute
from std_msgs.msg import Int8

def load(command: Tuple[int, int], publisher):

    input_cmd = int(command[0])
    input_speed = int(command[1])
    msg = Int8()
    if input_speed > 0:
        msg.data = 1
    elif input_speed < 0:
        msg.data = -1
    publisher.publish(msg)
    # print("Sending: %d" % msg.data)
    

loadRoute = ActionRoute(
    initial_state=(0, 0),
    run=load
)
