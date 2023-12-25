from typing import Tuple
from ..types.route import ActionRoute
from std_msgs.msg import Int8
from TEL.parameter import UP_LINEAR_CMD

def shooter_rise(command: Tuple[int, int], publisher):

    input_cmd = int(command[0])
    input_speed = int(command[1])
    msg = Int8()
    if input_speed > 0:
        msg.data = UP_LINEAR_CMD.PUSHUP
    elif input_speed < 0:
        msg.data = UP_LINEAR_CMD.PULLDOWN
    publisher.publish(msg)
    # print("Sending: %d" % msg.data)
    

shooterRiseRoute = ActionRoute(
    initial_state=(0, 0),
    run=shooter_rise
)
