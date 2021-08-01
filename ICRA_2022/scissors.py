#!!! determine where in the room the cut_direction_index will cause the arm to move (if it's in the positive or negative direction)

import time
import airobot as ar
from airobot import Robot
import numpy as np
import keyboard
from utils import *

UR5_on = True
DXL_on = True
sensor_on = False
COM_PORT = 'COM6' # !!! update
num_fing = 3
motor_ids = [] # !!! [prox thumb joint, dist thumb joint, prox pointer joint, dist pointer joint, prox middle finger joint, dist middle finger joint]
rotate_right_angle = 10 # !!! update with correct number, amount to rotate distal joints 90 degrees
rotate_support = 1 # !!! update with correct number
rotate_open = 10 # !!! update with correct number, amount to move thumb proximal joint to open scissors
rotate_close = -1 # !!! update with correct number, amount to move to incrementally try to close the scissors
threshold_suppert = 10 # !!! update threshold pressure to ensure that two side fingers are spread enough not to drop scissors
threshold_closed = 10 # pressure required to say scissors are fully closed
scissor_blade_length = int(input('Enter the scissor blade length in the same coordinates as UR5 Cartesian coordinates:  ')) # !!!
cut_direction_index = 0 # axis (x, y, z) that the UR5 arm should move in to cut (right now x axis)

# !!! make a function to adjust distal joint when proximal joint moves for a specified joint

robot = ar.Robot('ur5e_2f140', pb=False, use_cam=False)

if DXL_on == True:
	packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(DXL_TOTAL, COM_PORT)

	motor_pos = dxl_get_pos(motor_ids, packetHandler, groupBulkRead, ADDR, LEN)

if DXL_on == True:
	# mount scissors
	print('Once the scissors are positioned correctly over the manipulator, press m to move the joints to mount the scissors')
	keypress = getch()
	if keypress == 'm':
		# close distal joints
		for i in range(1,len(motor_pos),2)
			motor_pos[i] += rotate_right_angle
		pres = 0 # !!! read pressure for each finger from sensors, outputs [palmar thumb, dorsal thumb, palmar pointer, palmar middle finger]
		motor_pos = move(motor_ids, motor_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN): # !!! update with syntax
	
		# spread the two fingers so the scissors won't drop
		# !!! change to multithread so fingers can move simultaneously?
		while any(i <= threshold_suppert for i in pres[2::]):
			keypress = getch()		
			print('press y to move the fingers or press ESC to stop')
			if keypress == chr(0x1b):
				print('Escaped from grasping')
				break
			elif keypress == 'y':
				if pres[2] < threshold_suppert:
					motor_pos[2] += rotate_support
				if pres[3] < threshold_suppert:
					motor_pos[4] += rotate_support
				motor_pos = move(motor_ids, motor_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN): # !!! update with syntax	
			pres = 0 # !!! measure pressure		

	# open scissors
	# !!! need to change this programming to also adjust distal joint so it doesn't open when thumb moves
	motor_pos[0] += rotate_open

if UR5_on == True:
	# move arm forward length of scissor blade
	ee_xyz, quat, rot, euler = robot.arm.get_ee_pose()
	goal_ee = ee_xyz
	goal_ee[cut_direction_index] += scissor_blade_length

if DXL_on == True:
	# close scissors
	# !!! need to change this programming to also adjust distal joint so it doesn't open when thumb moves
	while pres[0] < threshold_closed
		keypress = getch()
		print('press y to move the fingers or press ESC to stop')
		if keypress == chr(0x1b):
			print('Escaped from grasping')
			break
		elif keypress == 'y':
			motor_pos[0] += rotate_close
			motor_pos = move(motor_ids, motor_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN): # !!! update with syntax

if UR5_on == True:
	# move arm back to starting position
	print('Move arm back to starting position?')
	keypress = getch()
	if keypress == 'y':
		robot.arm.move_ee_xyz(ee_xyz, wait=True)

if DXL_on == True:	
	shut_down(packetHandler, portHandler, groupBulkRead, ADDR, LEN)