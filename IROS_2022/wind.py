# winds and unwinds a single motor
# motor id specified by user as command prompt input
# useful when initially assembling manipulator and need to wind cables individually

import numpy as np
from functions import *

com_num = '/dev/ttyUSB0'

# comment out one of the lines below to change operating mode
# operating_mode = 'current_position' # current-based position control
operating_mode = 'extended_position'

# specify current goal and limit if using current-based position control
current_limit = amps2curr(1.2) # input the number of amps and it will convert to motor units
current_goal = amps2curr(1.0) # input the goal number of amps

inputs = input('What is the ID of the motor you would like to control? You may enter a list  ').split(", ")
motor_id = [int(id) for id in inputs]
ROTATE_AMOUNT = deg2pulse(15) # the increment in degrees (converted to motor pulse units) you would like the motor to move

if operating_mode == 'extended_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(motor_id, com_num, operating_mode)
elif operating_mode == 'current_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(motor_id, com_num, operating_mode, current_goal, current_limit)
else:
    print('invalid operating mode')

print('Press "w" to wind the motor(s), "u" to unwind the motor(s), "c" to change motors, or ESC to exit')

while 1:
    # get command from keypress
    keypress = getch()
    if keypress == chr(0x1b):
        break
    elif keypress == 'w':
        direction = 1
    elif keypress == 'u':
        direction = -1
    elif keypress == 'c':
        print('You asked to change motors. Shut down the old one?')
        shut_down(motor_id, SYS, ADDR, LEN, askAction=True)
        inputs = input('What is the ID of the motor you would like to control? You may enter a list  ').split(", ")
        motor_id = [int(id) for id in inputs]
    else:
        direction = 0

    dxl_goal_position = dxl_read_pos(motor_id, SYS, ADDR, LEN)
    # write new goal position
    dxl_goal_position += direction*ROTATE_AMOUNT

    # move motor
    dxl_present_position = move(motor_id, dxl_goal_position, SYS, ADDR, LEN, [dxl_start_position[0]-427, dxl_start_position[0]+427])

    # print current and voltage readings
    read_curr_volt(motor_id, True, SYS, ADDR)
    # print(dxl_read_pos(motor_id, SYS, ADDR, LEN))
    print('Moved. Press "w" to wind the motor, "u" to unwind the motor, or ESC to exit')
    print('')

count = 0
while count < 50:
    _, curr = calc_torque(motor_id, False, SYS, ADDR)
    print(curr)
    count += 1

shut_down(motor_id, SYS, ADDR, LEN, askAction=False)
