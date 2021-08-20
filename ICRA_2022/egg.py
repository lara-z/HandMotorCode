import time
# import airobot as ar
# from airobot import Robot
import numpy as np
from utils import *
from utils_sensor import *

# !!! import motor stuff

UR5_on = False
DXL_on = True
sensor_on = False
visualize = False
COM_PORT = 'COM3' # !!! update
operating_mode = 'extended_position'
ee_start_pos = [] # !!! find good start position
rotate_amount = deg2pulse(5) # !!! change once gear ratio and units have been established
rotate_open = deg2pulse(60) # !!! update
motor_ids = [1,3,7] # !!! add
motor_direction = [1]*len(motor_ids) # make values -1 in case agonist or antagonist cables were switched during assembly
num_fing = len(motor_ids)
pres = [0]*num_fing
t_data = 0.5 # during for pressure reading
threshold = 10 # !!! update with appropriate threshold

# !!! change to multithread so fingers can move simultaneously?

# robot = ar.Robot('ur5e', pb=False, use_cam=False)

if DXL_on == True:
	motor_pos, goal_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_ids, COM_PORT, operating_mode)

if sensor_on == True:
	args, ser, p_zero, f_zero = initialize_sensor(COM_PORT, visualize)

# UR5 arm go to start position
if UR5_on == True:
	robot.arm.set_jpos(ee_start_pos, wait=True)

# read pressure in each finger
for finger in range(0,num_fing):
	if sensor_on == True:
		_, pres[finger], _ = read_pres(p_zero, f_zero, t_data, args, ser)
	else:
		pres[finger] = 0 # !!! update with pressure reading

print('Fingers will close around egg')
# close fingers around egg until pressure threshold is reached
if DXL_on == True:
	while any(i <= threshold for i in pres):
		print('press y to move the fingers or press ESC to stop')
		keypress = getch()
		if keypress == chr(0x1b):
			print('Escaped from grasping')
			break
		elif keypress == 'y':
			for i in range(0,len(goal_pos)):
				if pres[i] <= threshold:
					goal_pos[i] += motor_direction[i]*rotate_amount
			_ = move(motor_ids, goal_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
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

print('Fingers will release egg')
if DXL_on == True:
	# open grasp
	for i in range(0,len(goal_pos)):
		goal_pos[i] -= motor_direction[i]*rotate_open
	print('press y to move the fingers or press ESC to stop')
	keypress = getch()
	if keypress == chr(0x1b):
		print('Escaped from grasping')
	elif keypress == 'y':
		motor_pos = move(motor_ids, goal_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)

	shut_down(motor_ids, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=False)
