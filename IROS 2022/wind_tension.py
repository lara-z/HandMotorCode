# automatically tensions full finger

import numpy as np
from functions import *

com_num = '/dev/ttyUSB0'

# comment out one of the lines below to change operating mode
operating_mode = 'current_position' # current-based position control
# operating_mode = 'extended_position'

# specify current goal and limit if using current-based position control
current_limit = amps2curr(1.2) # input the number of amps and it will convert to motor units
current_goal = amps2curr(1.0) # input the goal number of amps

inputs = input('What is the ID of the motor you would like to control?  ').split(", ")

class dxl_info:
    ids = [int(id) for id in inputs]
    joint_num = [1, -1, 2, -2]
    joint_torque = [0.25, 0.2]
TORQUE_MARGIN = 0.01

ROTATE_AMOUNT = deg2pulse(5) # the increment in degrees (converted to motor pulse units) you would like the motor to move

if operating_mode == 'extended_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(dxl_info.ids, com_num, operating_mode)
elif operating_mode == 'current_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(dxl_info.ids, com_num, operating_mode, current_goal, current_limit)
else:
    print('invalid operating mode')


# dxl_present_position = move(dxl_info.ids, dxl_goal_position, SYS, ADDR, LEN, [dxl_start_position[0]-427, dxl_start_position[0]+427])
# move(dxl_info.ids, dxl_goal_position, SYS, ADDR, LEN, True)

# tension finger sequentially
tension_seq(dxl_info.ids, SYS, ADDR, LEN)

start_pos = dxl_read_pos(dxl_info.ids, SYS, ADDR, LEN)

# read finger position when moving finger
while True:
    keypress = getch()
    if keypress == 'p':
        pres_pos = dxl_read_pos(dxl_info.ids, SYS, ADDR, LEN)
        print(pres_pos - start_pos)
    if keypress == 't':
        tension_seq(dxl_info.ids, SYS, ADDR, LEN)
    if keypress == chr(0x1b):
        break

time.sleep(1)
shut_down(dxl_info.ids, SYS, ADDR, LEN, askAction=False)