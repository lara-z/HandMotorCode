# scissor manipulator must be placed so that fingers are exactly vertical as starting position
#!!! determine where in the room the cut_direction_index will cause the arm to move (if it's in the positive or negative direction)

import time
# import airobot as ar
# from airobot import Robot
import numpy as np
from utils import *
from utils_sensor import *

UR5_on = False
DXL_on = True
sensor_on = False
cutting = True

com_port_dxl = 'COM3'
com_port_sensor = 'COM1'

# establish UR5 variables
scissor_blade_length = float(0.06) # float(input('Enter the scissor blade length in the same coordinates as UR5 Cartesian coordinates:  ')) # !!!
cut_direction_index = 0 # axis (x, y, z) that the UR5 arm should move in to cut (right now x axis)

# establish motor variables
operating_mode = 'current_position'
current_des = amps2curr(2.8)	# for current-based position control
current_lim = amps2curr(3.0)	# for current-based position control
motor_ids = [8,11,5,2,6,10] # !!! [prox thumb joint, dist thumb joint, prox pointer joint, dist pointer joint, prox middle finger joint, dist middle finger joint]
motor_direction = [1,0,1,-1,1,0] # make values -1 in case agonist or antagonist cables were switched during assembly
gear_ratio = -1.1
rotate_right_angle = 375 # !!! update with correct number, amount to rotate distal joints 90 degrees
rotate_support = deg2pulse(5) # !!! update with correct number
rotate_open = deg2pulse(20) # !!! update with correct number, amount to move thumb proximal joint to open scissors
rotate_close_rapid = -deg2pulse(10) # !!! update with correct number, amount to move to incrementally try to close the scissors
rotate_close_increment = -deg2pulse(5) # !!! update with correct number, amount to move to incrementally try to close the scissors
rotate_limit = 600
dxl_limits = [0]*len(motor_ids)

# establish sensor variables
visualize = False
num_fing = len(motor_ids)
threshold_support = 10 # !!! update threshold pressure to ensure that two side fingers are spread enough not to drop scissors
threshold_closed = 10 # pressure required to say scissors are fully closed
class read_pts:
	# order: palmar thumb, dorsal thumb, palmar pointer, palmar middle finger
    x_start = [2,0,6,4]
    x_end   = [4,2,8,6]
    y_start = [0,3,6,10]
    y_end   = [3,6,9,12]

# !!! make a function to adjust distal joint when proximal joint moves for a specified joint

def get_pres():
	# get pressure reading from sensor or zero reading if sensor isn't attached
	if sensor_on == True:
		mean_pres, max_pres, force = read_pres(p_zero, f_zero, args, ser, read_pts, print_pres=False)
	else:
		mean_pres = np.zeros(num_fing)
		max_pres = np.zeros(num_fing)
		force = np.zeros(num_fing)
	return mean_pres, max_pres, force

def move_dxl(dxl_goal):
	# convenient handle to move motors
	_ = move(motor_ids, dxl_goal, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN, print_currvolt=False, limits = dxl_limits)

# initialize everything
if UR5_on == True:
	robot = ar.Robot('ur5e', pb=False, use_cam=False)
if DXL_on == True:
	# initialize dynamixels with fingers in ending position
	motor_pos, goal_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_ids, com_port_dxl, operating_mode, current_des, current_lim)
	# set joint limits
	for i in range(num_fing):
		dxl_limits[i] = sorted([int(motor_pos[i]) - motor_direction[i]*rotate_limit, int(motor_pos[i]) + motor_direction[i]*rotate_limit])
if sensor_on == True:
	args, ser, p_zero, f_zero = initialize_sensor(com_port_sensor, visualize, read_pts)
else:
	p_zero = np.zeros(num_fing)
	f_zero = np.zeros(num_fing)

# read pressure if sensor is on
if sensor_on == True:
	_, pres, force = get_pres()
else:
	pres = np.zeros(num_fing)

# move fingers together to allow scissors to be mounted
print('Moving fingers to prepare for scissor mounting. Press y to continue or ESC to skip')
keypress = getch()
if keypress == chr(0x1b):
	print('Escaped from moving')
elif keypress == 'y':
	# goal_pos[0] += motor_direction[0]*rotate_open
	# goal_pos[1] += motor_direction[1]*gear_ratio*rotate_close_rapid
	goal_pos[2] += motor_direction[2]*rotate_close_rapid
	goal_pos[3] += motor_direction[3]*gear_ratio*rotate_close_rapid
	goal_pos[4] += motor_direction[4]*rotate_close_rapid
	goal_pos[5] += motor_direction[5]*gear_ratio*rotate_close_rapid
	move_dxl(goal_pos)

if DXL_on == True:
	# mount scissors
	print('Once the scissors are positioned correctly over the manipulator, press m to move the joints to mount the scissors')
	keypress = getch()
	if keypress == 'm':
		# close distal joints
		for i in range(1,len(goal_pos),2):
			goal_pos[i] += motor_direction[i]*rotate_right_angle
		mean_pres, max_pres, force = get_pres()
		move_dxl(goal_pos)

		# spread the two fingers so the scissors won't drop
		# !!! change to multithread so fingers can move simultaneously?
		while any(i <= threshold_support for i in max_pres[2::]):
			print('press y to move the fingers or press ESC to stop')
			keypress = getch()
			if keypress == chr(0x1b):
				print('Escaped from grasping')
				break
			elif keypress == 'y':
				for i in [1,2]:
					if max_pres[i] < threshold_support:
						goal_pos[2*i] += motor_direction[2*i]*rotate_support
						goal_pos[2*i+1] += motor_direction[2*i+1]*gear_ratio*rotate_support
				move_dxl(goal_pos)
mean_pres, max_pres, force = get_pres()

while True:
	if DXL_on == True:
		# open scissors
		print('press y to open the scissors')
		keypress = getch()
		if keypress == 'y':
			goal_pos[0] += motor_direction[0]*rotate_open
			goal_pos[1] += motor_direction[1]*gear_ratio*rotate_open
			move_dxl(goal_pos)

	if UR5_on == True:
		# move arm forward length of scissor blade
		goal_ee = [0]*3
		goal_ee[cut_direction_index] += scissor_blade_length
		print('Move arm forward tp prepare for cut')
		print('Press y to continue')
		keypress = getch()
		if keypress == 'y':
			robot.arm.move_ee_xyz(goal_ee, wait=True)

	if DXL_on == True:
		# close scissors
		# !!! need to change this programming to also adjust distal joint so it doesn't open when thumb moves
		while max_pres[0] < threshold_closed:
			print('press y to close the scissors or press ESC to stop')
			keypress = getch()
			if keypress == chr(0x1b):
				print('Escaped from grasping')
				break
			elif keypress == 'y':
				goal_pos[0] += rotate_close_increment
				move_dxl(goal_pos)

	if UR5_on == True:
		# move arm back to starting position
		print('Move arm back to starting position?')
		keypress = getch()
		if keypress == 'y':
			robot.arm.move_ee_xyz(-goal_ee, wait=True)
	
	print('To shut down motors, press ESC, otherwise press any other key')
	keypress = getch()
	if keypress == chr(0x1b):
		print('Escaped from grasping')
		break

if DXL_on == True:
	shut_down(motor_ids, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=False)
