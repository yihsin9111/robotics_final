from typing import Tuple
from ..types.route import ActionRoute
from std_msgs.msg import String

def shoot(command: Tuple[int, int], publisher):

    input_cmd = int(command[0])
    input_speed = int(command[1])
    
    if input_speed > 0:
        msg = String()
        msg.data = "SWITCH"
        publisher.publish(msg)
        print("Sending: %s" % msg.data)
    else:
        msg = String()
        msg.data = "STOP"
        publisher.publish(msg)
        print("Sending: %s" % msg.data)
         

shootRoute = ActionRoute(
    initial_state=(0, 0),
    run=shoot
)
