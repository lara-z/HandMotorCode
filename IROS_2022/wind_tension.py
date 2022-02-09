# automatically tensions full finger

import numpy as np
from functions import *

com_num = '/dev/ttyUSB0'

# comment out one of the lines below to change operating mode
operating_mode = 'current_position' # current-based position control
# operating_mode = 'extended_position'

# specify current goal and limit if using current-based position control
current_limit = amps2curr(1.2) # input the number of amps and it will convert to motor units
current_goal = amps2curr(1.2) # input the goal number of amps

# inputs = input('What is the ID of the motor you would like to control?  ').split(", ")
# 5,2,8,6

class dxl_info:
    # ids = [int(id) for id in inputs]
    ids = [5,2,8,6]
    joint_num = [1, -1, 2, -2]
    joint_torque = [0.25, 0.2]
TORQUE_MARGIN = 0.01

ROTATE_AMOUNT = deg2pulse(30) # the increment in degrees (converted to motor pulse units) you would like the motor to move

if operating_mode == 'extended_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(dxl_info.ids, com_num, operating_mode)
elif operating_mode == 'current_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(dxl_info.ids, com_num, operating_mode, current_goal, current_limit)
else:
    print('invalid operating mode')


# dxl_present_position = move(dxl_info.ids, dxl_goal_position, SYS, ADDR, LEN, [dxl_start_position[0]-427, dxl_start_position[0]+427])
# move(dxl_info.ids, dxl_goal_position, SYS, ADDR, LEN, True)

# tension finger sequentially
dxl_goal_position = tension_seq(dxl_info, SYS, ADDR, LEN)
time.sleep(1)

# move finger
amt = deg2pulse(10)
new_rel_goal = np.array([amt, 0])
grasp = np.array([amt,amt])

# read move finger up or retension
print('***')
print('press ''f'' to flex finger, ''e'' to extend, ''t'' to tension, or ESC to shut down motors')
print('***')
while True:
    keypress = getch()
    # if keypress == 'm':
    #     dxl_present_position, _ = move_rel(dxl_info, new_rel_goal, dxl_goal_position, SYS, ADDR, LEN)
    #     # tension_seq(dxl_info, SYS, ADDR, LEN)
    if keypress == 'f':
        _, dxl_goal_position = move_rel(dxl_info, grasp, dxl_goal_position, SYS, ADDR, LEN)
        dxl_goal_position = tension_seq(dxl_info, SYS, ADDR, LEN, -1)
    elif keypress == 'e':
        _, dxl_goal_position = move_rel(dxl_info, -grasp, dxl_goal_position, SYS, ADDR, LEN)
        dxl_goal_position = tension_seq(dxl_info, SYS, ADDR, LEN, 1)
    elif keypress == 't':
        dxl_goal_position = tension_seq(dxl_info, SYS, ADDR, LEN)
    elif keypress == chr(0x1b):
        break

shut_down(dxl_info.ids, SYS, ADDR, LEN, askAction=True)