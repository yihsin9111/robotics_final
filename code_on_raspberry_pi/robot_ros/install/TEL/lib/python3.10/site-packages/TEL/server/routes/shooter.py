from typing import Tuple
from TEL.parameter import WHEEL_CMD
from ..types.route import ActionRoute
from std_msgs.msg import Int8

def shooter(command: Tuple[int, int], publisher):

    input_cmd = int(command[0])
    input_speed = int(command[1])
       
    msg = Int8()
    msg.data = 1
    publisher.publish(msg)
    print("Sending: %d" % msg.data)
    

shooterRoute = ActionRoute(
    initial_state=(0, 0),
    run=shooter
)
