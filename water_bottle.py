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

# check port using:    python -m serial.tools.list_ports
# ls /dev/ttyUSB*
com_port_dxl = '/dev/ttyUSB0'
com_port_sensor = '/dev/ttyUSB1'

# establish UR5 variables
start_pos = [] # !!! find good start position in joint coordinates
wrist_ind = 0
x_ind = 0 # index for z-axis
z_ind = 2 # index for z-axis
table_height = 0.1 # height of gripper from table
withdraw_dist = 0.1 # amount to withdraw arm after releasing bottle

# establish dxl variables
operating_mode = 'current_position'
current_des = amps2curr(2.5)	# for current-based position control
current_lim = amps2curr(2.8)	# for current-based position control
motor_ids = [12,13,14]
motor_direction = [-1, 1, -1]
adduct_ind = 1
rotate_amount = 20
rotate_adduct = 3
rotate_limit = 1000
rotate_open = 200
rotate_adduct_limit = 120

# establish sensor variables
visualize = False
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
full_thresh_p = 12
bottle_p = 0 # average maximum pressure the bottle exerts, compared to full_thres_p to determine if full or empty
grasp_thresh_p = 70 # !!! change to real value
grasp_thresh_f = 200 # !!! change to real value
bottle_status = '' # options are 'full' or 'empty' once bottle weight is measured

def calc_pres():
	# read pressure in each finger
	if sensor_on == True:
		# 0 is the top finger
		mean_pres, max_pres, force, _ = read_pres(p_zero, f_zero, x_zero, args, ser, read_pts, print_pres=True)
		# pressure[1] -= pressure[0]
		# pressure[2] -= pressure[0]
	else:
		mean_pres = np.zeros(num_fing)
		max_pres = np.zeros(num_fing)
		force = np.zeros(num_fing)
	return mean_pres, max_pres, force

def move_dxl(dxl_goal):
	# move motors to goal position
	_ = move(motor_ids, dxl_goal, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN, print_currvolt=False)

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
	args, ser, p_zero, f_zero, x_zero = initialize_sensor(com_port_sensor, visualize, read_pts)
else:
	p_zero = np.zeros(num_fing)
	f_zero = np.zeros(num_fing)

# set joint limits
dxl_limits = [0]*num_fing
for i in range(num_fing):
	dxl_limits[i] = sorted([int(motor_pos[i]) - motor_direction[i]*rotate_limit, int(motor_pos[i]) + motor_direction[i]*1.5*rotate_limit])

mean_pres, max_pres, force = calc_pres()

# # open hand to put in bottle
# for i in range(1,num_fing):
# 	goal_pos[i] -= motor_direction[i]*rotate_open
# move_dxl(goal_pos)

# check if bottle is full
if sensor_on == True:
	print('press any key to indicate that the bottle has been mounted')
	getch()
	print('Detecting whether bottle is full or empty. One moment...')
	data_pts = 20
	for i in range(data_pts):
		_, max_pres, _ = calc_pres()
		bottle_p += max_pres
	bottle_p = bottle_p/data_pts
	if (np.sum(bottle_p[1::] >= full_thresh_p) == 2):
		bottle_status = 'full'
	else:
		bottle_status = 'empty'
else:
	bottle_status = 'empty'

print('')
print('bottle status determined: ', bottle_status)
print('')

# close fingers around bottle until pressure threshold is reached
while (np.sum(max_pres[1::] >= grasp_thresh_p) < 2):
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
grasp_pos = goal_pos.copy() # remember this pose to return bottle to level later
print('The bottle has been grasped')

if bottle_status == 'full':
	# pour water
	goal_pos[0] += motor_direction[0]*rotate_adduct # move top finger left
	goal_pos[1] += motor_direction[1]*rotate_amount # move left finger down
	move_dxl(goal_pos)
	time.sleep(3.0)
print('poured water')
move_dxl(grasp_pos)

if UR5_on == True:
	# set bottle on table
	goal_pos = start_pos.copy()
	# make bottle vertical
	goal_pos[wrist_ind] += np.pi/2
	robot.arm.set_jpos(goal_pos, wait=True)
	# set on table
	ee_xyz = [0]*3
	ee_xyz[z_ind] = -table_height
	robot.arm.move_ee_xyz(ee_xyz, wait=True)
	# release bottle
	for i in range(1,num_fing):
		goal_pos[i] -= motor_direction[i]*rotate_open
	move_dxl(goal_pos)
	# withdraw arm
	ee_xyz = [0]*3
	ee_xyz[x_ind] = -withdraw_dist
	robot.arm.move_ee_xyz(ee_xyz, wait=True)


shut_down(motor_ids, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=False)
