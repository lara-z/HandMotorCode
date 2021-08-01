# place bottle opening to the left (when looking at the palm)

import time
import airobot as ar
from airobot import Robot
import numpy as np
from utils import *

DXL_on = True
sensor_on = False
COM_PORT = 'COM6' # !!! update
start_pos = [] # !!! find good start position
wrist_ind = 0
tilt_options = ['level', 'little', 'lots']
tilt_thresh = [0, 10, 20] # !!! calibrate
tilt_num = int(input('Enter tilt integer from 0 (level) to 2 (very tilted)'))
threshold = tilt_thresh[tilt_num]
grasp_threshold = 10 # !!! change to real value
num_fing = 3
motor_ids = [] #!!! add motor ids
grasp_rotate_amount = 10 # !!! change to real value
rotate_amount = 10 # !!! change to real value

def calc_pres():
	# read pressure in each finger
	# 0 is the top finger
	pressure = [0]*num_fing
	for finger in range(0,num_fing):
		pressure[finger] = 0 # !!! update with pressure reading
	pressure[1] -= pressure[0]
	pressure[2] -= pressure[0]
	return pres

def adjust(pressure, thresh)
	# might need to make the pressure difference a percentage of pressure: as liquid leaves the bottle...
	while abs(pressure[1] - pressure[2]) <= thresh:
		if pressure[1] <= pressure[2]:
			# bottle is tilted wrong way
			motor_pos[0] += 3*rotate_amount # move top finger left
			motor_pos[1] += 3*rotate_amount # move left finger down
			motor_pos[2] += -3*rotate_amount # move left finger down
		elif pressure[1] - pressure[2] > thresh:
			# bottle is tilted too much
			motor_pos[0] += -rotate_amount # move top finger left
			motor_pos[1] += -rotate_amount # move left finger down
			motor_pos[2] += rotate_amount # move left finger down
		elif pressure[1] - pressure[2] < thresh:
			# bottle is tilted too little
			# bottle is tilted wrong way
			motor_pos[0] += rotate_amount # move top finger left
			motor_pos[1] += rotate_amount # move left finger down
			motor_pos[2] += -rotate_amount # move left finger down
		print('press y to move the finger or press ESC to stop')
		keypress = getch()
		if keypress == chr(0x1b):
			print('Escaped from grasping')
			break
		elif keypress == 'y':
			motor_pos = move(motor_ids, motor_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN): # !!! update function syntax

robot = ar.Robot('ur5e_2f140', pb=False, use_cam=False)

packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(DXL_TOTAL, COM_PORT)

motor_pos = dxl_get_pos(motor_ids, packetHandler, groupBulkRead, ADDR, LEN)

if UR5_on == True:
	# UR5 arm go to start position
	robot.arm.set_jpos(start_pos, wait=True)

pres = calc_pres()

# close fingers around egg until pressure threshold is reached
while any(i <= grasp_threshold for i in pres[1::]):
	keypress = getch()
	print('press y to move the fingers or press ESC to stop')
	if keypress == chr(0x1b):
        print('Escaped from grasping')
        break
    elif keypress == 'y':
		for i in range(1,len(motor_pos)):
			# change only the bottom two fingers for the grasp: the top one should stay
			if pres[i] <= grasp_threshold:
				motor_pos[i] += grasp_rotate_amount
		motor_pos = move(motor_ids, motor_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN): # !!! update function syntax
		# read pressure in each finger
		for finger in range(0,num_fing):
			pres[finger] = 0 # !!! update with pressure reading

while True:
	# specify tilt amount and bottle will (hopefully) achieve it
	keypress = getch()
	print('press ESC to end code, c to change tilt level, p to pour, or v to rotate the bottle to vertical')
	if keypress == chr(0x1b):
        break
    elif keypress == 'c':
    	# change desired tilt
		tilt_num = input(int('Enter tilt integer from 0 (level) to 2 (very tilted):  '))
		threshold = tilt_thresh[tilt_num]
	elif keypress == 'p':
		adjust(pres, threshold)
	elif keypress == 'v':
		# rotate UR5 arm to bring bottle to vertical?
		goal_pos = start_pos
		goal_pos[wrist_ind] += np.pi/2
		# UR5 arm go to start position
		robot.arm.set_jpos(goal_pos, wait=True)
		
shut_down(packetHandler, portHandler, groupBulkRead, ADDR, LEN)