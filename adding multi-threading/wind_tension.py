# winds and unwinds a single motor
# motor id specified by user as command prompt input
# useful when initially assembling manipulator and need to wind cables individually

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
motor_id = [int(id) for id in inputs]

DXL_JOINT_NUM = [1, -1, 2, -2]
JOINT_TORQUE = [0.25, 0.2]
TORQUE_MARGIN = 0.01
joint_ratios = [1, 1, 1]

ROTATE_AMOUNT = deg2pulse(5) # the increment in degrees (converted to motor pulse units) you would like the motor to move

if operating_mode == 'extended_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(motor_id, com_num, operating_mode)
elif operating_mode == 'current_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(motor_id, com_num, operating_mode, current_goal, current_limit)
else:
    print('invalid operating mode')

# move motor
dxl_goal_position += ROTATE_AMOUNT
# dxl_present_position = move(motor_id, dxl_goal_position, SYS, ADDR, LEN, [dxl_start_position[0]-427, dxl_start_position[0]+427])
move(motor_id, dxl_goal_position, SYS, ADDR, LEN, True)

# tension cable
# tension(dxl_start_position, motor_id, DXL_JOINT_NUM, JOINT_TORQUE, TORQUE_MARGIN, SYS, ADDR, LEN)

# tension finger sequentially
# tension_seq(motor_id, DXL_JOINT_NUM, JOINT_TORQUE, SYS, ADDR, LEN)

# print current and voltage readings
# print_curr_volt(motor_id, SYS, ADDR)
# print(dxl_read_pos(motor_id, SYS, ADDR, LEN))

time.sleep(1)
shut_down(motor_id, SYS, ADDR, LEN, askAction=False)