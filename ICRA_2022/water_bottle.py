# place bottle opening to the left (when looking at the palm)

import time
# import airobot as ar
# from airobot import Robot
import numpy as np
from utils import *
from utils_sensor import *

UR5_on = False
DXL_on = True
sensor_on = True
mode = 'manual' # auto

com_port_dxl = '/dev/ttyUSB2' # !!! update
com_port_sensor = '/dev/ttyUSB0' # !!! update

# establish UR5 variables
start_pos = [] # !!! find good start position in joint coordinates
wrist_ind = 0

# establish dxl variables
operating_mode = 'current_position'
current_des = amps2curr(1.1)	# for current-based position control
current_lim = amps2curr(1.2)	# for current-based position control
motor_ids = [12,13,14]
motor_direction = [-1, 1, -1]
adduct_ind = 1
rotate_amount = 15
rotate_adduct = 3
rotate_limit = 400
rotate_adduct_limit = 120

# establish sensor variables
visualize = True
num_fing = len(motor_ids)
pres = [0]*num_fing
class read_pts:
	# order: abduction/adduction finger, right finger, left finger
    x_start = [10,0,20]
    x_end   = [22,12,32]
    y_start = [3,0,6]
    y_end   = [6,3,9]

# establish tilt variables
tilt_options = ['level', 'little', 'lots']
tilt_thresh = [0, 10, 20] # !!! calibrate
tilt_num = 1 #int(input('Enter tilt integer from 0 (level) to 2 (very tilted):  '))
threshold = tilt_thresh[tilt_num]
grasp_thresh_p = 200 # !!! change to real value
grasp_thresh_f = 200 # !!! change to real value

def calc_pres():
	# read pressure in each finger
	if sensor_on == True:
		# 0 is the top finger
		mean_pres, max_pres, force = read_pres(p_zero, f_zero, args, ser, read_pts, print_pres=True)
		# pressure[1] -= pressure[0]
		# pressure[2] -= pressure[0]
	else:
		mean_pres = np.zeros(num_fing)
		max_pres = np.zeros(num_fing)
		force = np.zeros(num_fing)
	return mean_pres, max_pres, force

def move_dxl(dxl_goal):
	# move motors to goal position
	_ = move(motor_ids, dxl_goal, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN, print_currvolt=True, limits = dxl_limits)

def adjust(dxl_goal, pressure, thresh):
	# might need to make the pressure difference a percentage of pressure: as liquid leaves the bottle...
	while True: # abs(pressure[1] - pressure[2]) <= thresh: # !!! change this line to force and if lines
		if pressure[1] <= pressure[2]:
			# bottle is tilted wrong way
			dxl_goal[0] += 3*motor_direction[0]*rotate_adduct # move top finger left
			dxl_goal[1] += 3*motor_direction[1]*rotate_amount # move left finger down
			dxl_goal[2] += -3*motor_direction[2]*rotate_amount # move left finger down
		elif pressure[1] - pressure[2] > thresh:
			# bottle is tilted too much
			dxl_goal[0] += -motor_direction[0]*rotate_adduct # move top finger left
			dxl_goal[1] += -motor_direction[1]*rotate_amount # move left finger down
			dxl_goal[2] += motor_direction[2]*rotate_amount # move left finger down
		elif pressure[1] - pressure[2] < thresh:
			# bottle is tilted too little
			# bottle is tilted wrong way
			dxl_goal[0] += motor_direction[0]*rotate_adduct # move top finger left
			dxl_goal[1] += motor_direction[1]*rotate_amount # move left finger down
			dxl_goal[2] += -motor_direction[2]*rotate_amount # move left finger down
		print('press y to move the finger or press ESC to stop')
		keypress = getch()
		if keypress == chr(0x1b):
			print('Escaped from grasping')
			break
		elif keypress == 'y':
			move_dxl(dxl_goal)
			mean_pres, max_pres, force = calc_pres()

# initialize everything
if UR5_on == True:
	robot = ar.Robot('ur5e', pb=False, use_cam=False)
	# UR5 arm go to start position
	robot.arm.set_jpos(start_pos, wait=True)
if DXL_on == True:
	motor_pos, goal_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_ids, com_port_dxl, operating_mode, current_des, current_lim)
	motor_pos = dxl_read(motor_ids, packetHandler, groupBulkRead, ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)
if sensor_on == True:
	args, ser, p_zero, f_zero = initialize_sensor(com_port_sensor, visualize, read_pts)
else:
	p_zero = np.zeros(num_fing)
	f_zero = np.zeros(num_fing)

# set joint limits
dxl_limits = [0]*num_fing
for i in range(num_fing):
	dxl_limits[i] = sorted([int(motor_pos[i]) - motor_direction[i]*rotate_limit, int(motor_pos[i]) + motor_direction[i]*1.5*rotate_limit])

mean_pres, max_pres, force = calc_pres()

# open hand to put in bottle
for i in range(1,num_fing):
	goal_pos[i] -= motor_direction[i]*0.43*rotate_limit
move_dxl(goal_pos)

# close fingers around bottle until pressure threshold is reached
while (np.sum(force[1::] >= grasp_thresh_f) < 2) or (np.sum(max_pres[1::] >= grasp_thresh_p) < 2):
	# print('press y to move the fingers or press ESC to stop closing grasp')
	# keypress = getch()
	# if keypress == chr(0x1b):
	# 	print('Escaped from grasping')
	# 	break
	# elif keypress == 'y':
	for i in range(1,num_fing):
		# change only the bottom two fingers for the grasp: the top one should stay
		if force[i] <= grasp_thresh_f:
			goal_pos[i] += motor_direction[i]*rotate_amount
	move_dxl(goal_pos)
	mean_pres, max_pres, force = calc_pres()
print('The bottle has been grasped')

while True:
	# specify tilt amount and bottle will (hopefully) achieve it
	print('press ESC to end code, c to change tilt level, p to pour, or v to rotate the bottle to vertical')
	keypress = getch()
	if keypress == chr(0x1b):
		break
	elif keypress == 'c':
    	# change desired tilt
		tilt_num = int(input('Enter tilt integer from 0 (level) to 2 (very tilted):  '))
		threshold = tilt_thresh[tilt_num] # !!! change threshold to force
	elif keypress == 'p':
		adjust(goal_pos, pres, threshold)
	elif keypress == 'v':
		# rotate UR5 arm to bring bottle to vertical?
		goal_pos = start_pos
		goal_pos[wrist_ind] += np.pi/2
		# UR5 arm go to start position
		robot.arm.set_jpos(goal_pos, wait=True)

shut_down(motor_ids, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=False)
