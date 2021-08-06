import time
import airobot as ar
from airobot import Robot
import numpy as np
import keyboard
from utils import *

# !!! import motor stuff

UR5_on = True
DXL_on = True
sensor_on = False
COM_PORT = 'COM3' # !!! update
ee_start_pos = [] # !!! find good start position
rotate_amount = 5 # !!! change once gear ratio and units have been established
rotate_open = 10 # !!! update
motor_ids = [] # !!! add
num_fing = 4
pres = [0]*num_fing
threshold = 10 # !!! update with appropriate threshold

# !!! change to multithread so fingers can move simultaneously?

robot = ar.Robot('ur5e', pb=False, use_cam=False)

if DXL_on == True:
	packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(DXL_TOTAL, COM_PORT)

	motor_pos = dxl_read(DXL_IDS, packetHandler, groupBulkRead, ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)

# UR5 arm go to start position
if UR5_on == True:
	robot.arm.set_jpos(ee_start_pos, wait=True)

# read pressure in each finger
for finger in range(0,num_fing):
	if sensor_on == True:
	else:
		pres[finger] = 0 # !!! update with pressure reading

# close fingers around egg until pressure threshold is reached
if DXL_on == True:
	while any(i <= threshold for i in pres):
		keypress = getch()
		print('press y to move the fingers or press ESC to stop')
		if keypress == chr(0x1b):
			print('Escaped from grasping')
			break
		elif keypress == 'y':
			for i in range(0,len(motor_pos)):
				if pres[i] <= threshold:
					motor_pos[i] += rotate_amount
			motor_pos = move(motor_ids, motor_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN): # !!! update function syntax
			# read pressure in each finger
			for finger in range(0,num_fing):
				pres[finger] = 0 # !!! update with pressure reading

	print('Finished grasping')

if UR5_on == True:
	# calculate end effector Cartesian position from joint coords
	pos, quat, rot, euler = robot.arm.get_ee_pose()
	ee_xyz = pos # !!! did I interpret the line above correctly?

	# lift egg
	ee_xyz[2] += 0.1 #!!! what units are these xyz in? meters?
	robot.arm.move_ee_xyz(ee_xyz, wait=True)

if DXL_on == True:
	# open grasp
	for i in range(0,len(motor_pos)):
		motor_pos[i] += rotate_open
	keypress = getch()
	print('press y to move the fingers or press ESC to stop')
	if keypress == chr(0x1b):
		print('Escaped from grasping')
		break
	elif keypress == 'y':
		motor_pos = move(motor_ids, motor_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN): # !!! update function syntax

	shut_down(packetHandler, portHandler, groupBulkRead, ADDR, LEN)
