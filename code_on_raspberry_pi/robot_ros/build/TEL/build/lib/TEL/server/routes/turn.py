from typing import Tuple
from TEL.parameter import WHEEL_CMD
from ..types.route import ActionRoute
from std_msgs.msg import Int16

def turn(command: Tuple[int, int], publisher):

    input_cmd = int(command[0])
    input_speed = int(command[1])
    cmd = WHEEL_CMD.STOP
    speed = 0
    MAXIMUM_INPUT = 25
    MAXIMUM_OUTPUT = 25 

    # print(f"move: {command}")
    # print(command[0], command[1])
    
    if input_cmd == 0:
        # print("HI")
        if input_speed > 0:
            cmd = WHEEL_CMD.TURNRIGHT
            speed = input_speed
        elif input_speed < 0:
            cmd = WHEEL_CMD.TURNLEFT
            speed = -input_speed
        else:
            cmd = WHEEL_CMD.STOP
   # print(speed * MAXIMUM_OUTPUT)
    msg = Int16()
    msg.data = (cmd << 8) + int(speed * MAXIMUM_OUTPUT / MAXIMUM_INPUT)
    publisher.publish(msg)
    # print("Sending: %d" % msg.data)
    

turnRoute = ActionRoute(
    initial_state=(0, 0),
    run=turn
)
