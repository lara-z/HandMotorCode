from utils import *
import os
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import numpy as np

com_num = '/dev/ttyUSB1' # '/dev/ttyUSB0'
operating_mode = 'current_position'
current_des = amps2curr(2.0)	# for current-based position control
current_lim = amps2curr(2.5)	# for current-based position control
rotate_amount = deg2pulse(5)	# amount that motor rotates to close grasp around egg
rotate_limit_ag = 650 # !!! 350
rotate_limit_an = deg2pulse(30)                                # !!! update
motor_ids = [4,7,1,3]
motor_direction = [-1,1,-1,-1] # make values -1 in case agonist or antagonist cables were switched during assembly
dxl_limits = [0]*len(motor_ids)

motor_pos, goal_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_ids, com_num, operating_mode, current_des, current_lim)
# reboot(packetHandler, portHandler, motor_ids, com_num, operating_mode)
# print('**rebooted')
# _, _, _, _, _, _, _, _ = initialize(motor_ids, com_num, operating_mode, current_des, current_lim)
shut_down(motor_ids, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=False)