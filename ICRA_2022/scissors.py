# hand will accept scissors for mounting, open scissors, UR5 arm will move forward,
# scissors will close if it cutting paper (requires lower pressure), open, ur5 arm will
# move forward, cutting will repeat
# scissor manipulator must be placed so that fingers are exactly vertical as starting position

import time
# import airobot as ar
# from airobot import Robot
import numpy as np
from utils import *
from utils_sensor import *

UR5_on = False
DXL_on = True
sensor_on = True
cutting = True

# check port using:    python -m serial.tools.list_ports
# ls /dev/ttyUSB*
com_port_dxl = '/dev/ttyUSB0'
com_port_sensor = '/dev/ttyUSB1'

# establish UR5 variables
start_ur5 = [1.5286102294921875, -2.384364744225973, -1.4049272537231445, -0.9217203420451661, 0.04831552505493164, 0.6629600524902344]
scissor_blade_length = float(0.06) # float(input('Enter the scissor blade length in the same coordinates as UR5 Cartesian coordinates:  ')) # !!!
cut_direction_index = 0 # axis (x, y, z) that the UR5 arm should move in to cut (right now x axis)

# establish motor variables
operating_mode = 'current_position'
current_des = amps2curr(2.8)	# for current-based position control
current_lim = amps2curr(3.0)	# for current-based position control
motor_ids = [8,11,5,2,6,10] # !!! [prox thumb joint, dist thumb joint, prox pointer joint, dist pointer joint, prox middle finger joint, dist middle finger joint]
motor_direction = [1,-1,1,-1,1,-1] # make values -1 in case agonist or antagonist cables were switched during assembly
gear_ratio = -1.1
rotate_right_angle = 375 # !!! update with correct number, amount to rotate distal joints 90 degrees
rotate_support = deg2pulse(5) # !!! update with correct number
rotate_open = deg2pulse(20) # !!! update with correct number, amount to move thumb proximal joint to open scissors
rotate_close_rapid = -deg2pulse(12) # !!! update with correct number, amount to move to incrementally try to close the scissors
rotate_close_increment = -deg2pulse(4) # !!! update with correct number, amount to move to incrementally try to close the scissors
rotate_limit = 600
dxl_limits = [0]*len(motor_ids)

# establish sensor variables
visualize = False
num_joints = len(motor_ids)
thresh_support_p = 80 # 40 # ensure that two side fingers are spread enough not to drop scissors
thresh_closed_p = 150 # pressure required to say scissors are fully closed
thresh_closed_f = 150 # force required to say scissors are fully closed
class read_pts:
	# order: palmar thumb, dorsal thumb, palmar pointer, palmar middle finger
    x_start = [4,6,2,0]
    x_end   = [6,8,4,2]
    y_start = [8,8,4,0]
    y_end   = [12,12,8,4]

# !!! make a function to adjust distal joint when proximal joint moves for a specified joint

def get_pres():
	# get pressure reading from sensor or zero reading if sensor isn't attached
	if sensor_on == True:
		mean_pres, max_pres, force, _ = read_pres(p_zero, f_zero, x_zero, args, ser, read_pts, print_pres=True)
	else:
		mean_pres = np.zeros(num_joints)
		max_pres = np.zeros(num_joints)
		force = np.zeros(num_joints)
	return mean_pres, max_pres, force

def move_dxl(dxl_goal):
	# convenient handle to move motors
	pres_pos = move(motor_ids, dxl_goal, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN, print_currvolt=False)
	return pres_pos

# initialize everything
if UR5_on == True:
	robot = ar.Robot('ur5e', pb=False, use_cam=False)
if DXL_on == True:
	# initialize dynamixels with fingers in ending position
	motor_pos, goal_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_ids, com_port_dxl, operating_mode, current_des, current_lim)
	# set joint limits
	for i in range(num_joints):
		dxl_limits[i] = sorted([int(motor_pos[i]) - motor_direction[i]*rotate_limit, int(motor_pos[i]) + motor_direction[i]*rotate_limit])
if sensor_on == True:
	args, ser, p_zero, f_zero, x_zero = initialize_sensor(com_port_sensor, visualize, read_pts)
else:
	p_zero = np.zeros(num_joints)
	f_zero = np.zeros(num_joints)

# read pressure
_, pres, force = get_pres()

# move fingers together to allow scissors to be mounted
print('Move fingers to prepare for scissor mounting. Press y to continue or ESC to skip')
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
	goal_pos = move_dxl(goal_pos)

if DXL_on == True:
	# mount scissors
	print('Once the scissors are positioned correctly over the manipulator, press y to move the joints to mount the scissors')
	keypress = getch()
	if keypress == 'y':
		# close distal joints
		print(goal_pos)
		for i in range(1,len(goal_pos),2):
			print(i)
			goal_pos[i] += motor_direction[i]*rotate_right_angle
		print(goal_pos)
		mean_pres, max_pres, force = get_pres()
		goal_pos = move_dxl(goal_pos)

		# spread the two fingers so the scissors won't drop
		# !!! change to multithread so fingers can move simultaneously?
		while any(i <= thresh_support_p for i in max_pres[2::]):
			print('press y to move the fingers or press ESC to stop')
			keypress = getch()
			if keypress == chr(0x1b):
				print('Escaped from grasping')
				break
			elif keypress == 'y':
				for i in [1,2]:
					if max_pres[i] < thresh_support_p:
						goal_pos[2*i] += motor_direction[2*i]*rotate_support
						goal_pos[2*i+1] += motor_direction[2*i+1]*gear_ratio*rotate_support
				goal_pos = move_dxl(goal_pos)
				_, max_pres, force = get_pres()
		print('Scissors have been grasped')

mean_pres, max_pres, force = get_pres()

while True:
	if DXL_on == True:
		# open scissors
		print('press y to open the scissors')
		keypress = getch()
		if keypress == 'y':
			goal_pos[0] = motor_pos[0] + motor_direction[0]*rotate_open
			goal_pos[1] = motor_pos[1] + motor_direction[1]*gear_ratio*rotate_open
			goal_pos = move_dxl(goal_pos)
			_, max_pres, force = get_pres()

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
		goal_pos = dxl_read(motor_ids, packetHandler, groupBulkRead, ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)
		# !!! need to change this programming to also adjust distal joint so it doesn't open when thumb moves
		while (max_pres[0] < thresh_closed_p) or (force[0] < thresh_closed_f):
			print('press y to close the scissors or press ESC to stop')
			keypress = getch()
			if keypress == chr(0x1b):
				print('Escaped from grasping')
				break
			elif keypress == 'y':
				goal_pos[0] += rotate_close_increment
				goal_pos[1] = goal_pos[1] + motor_direction[1]*gear_ratio*rotate_close_increment
				goal_pos = move_dxl(goal_pos)
				_, max_pres, force = get_pres()
		print('Scissors have been closed')

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
