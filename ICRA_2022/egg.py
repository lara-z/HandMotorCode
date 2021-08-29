import time
# import airobot as ar
# from airobot import Robot
import numpy as np
from utils import *
from utils_sensor import *

UR5_on = False
DXL_on = True
sensor_on = True

com_port_dxl = '/dev/ttyUSB2' # !!! update
com_port_sensor = '/dev/ttyUSB1' # !!! update

# establish UR5 variables
ee_start_pos = [] # !!! find good start position
lift_ind = 2
lift_amount = 0.2
ee_xyz = [0]*3
ee_xyz[lift_ind] = lift_amount

# establish dxl variables
operating_mode = 'current_position'
current_des = amps2curr(2.8)	# for current-based position control
current_lim = amps2curr(3.0)	# for current-based position control
rotate_amount = deg2pulse(5)	# amount that motor rotates to close grasp around egg
rotate_limit_ag = 450 # !!! 350
rotate_limit_an = deg2pulse(30) # !!! update
motor_ids = [4,7,1,3]
motor_direction = [-1,1,-1,-1] # make values -1 in case agonist or antagonist cables were switched during assembly
dxl_limits = [0]*len(motor_ids)

# establish sensor variables
visualize = False
num_fing = len(motor_ids)
pres = [0]*num_fing
class read_pts:
	# order: small left, big left, big right, small right finger
    x_start = [2,0,6,4]
    x_end   = [4,2,8,6]
    y_start = [0,3,6,10]
    y_end   = [3,6,9,12]
p_thresh = 20
f_thresh = 30

# !!! change to multithread so fingers can move simultaneously?

def get_pres():
	mean_pres, max_pres, force = read_pres(p_zero, f_zero, args, ser, read_pts, print_pres=False)
	return mean_pres, max_pres, force

def move_dxl(dxl_goal):
	_ = move(motor_ids, dxl_goal, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN, print_currvolt=True, limits = dxl_limits)

# robot = ar.Robot('ur5e', pb=False, use_cam=False)

# UR5 arm go to start position
if UR5_on == True:
	robot.arm.set_jpos(ee_start_pos, wait=True)

if sensor_on == True:
	args, ser, p_zero, f_zero = initialize_sensor(com_port_sensor, visualize, read_pts)

# initialize dynamixel and open fingers
if DXL_on == True:
	# initialize dynamixels with fingers in ending position
	motor_pos, goal_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_ids, com_port_dxl, operating_mode, current_des, current_lim)
	# set joint limits
	for i in range(num_fing):
		dxl_limits[i] = sorted([int(motor_pos[i]) - motor_direction[i]*rotate_limit_an, int(motor_pos[i]) + motor_direction[i]*rotate_limit_ag])
	# open fingers to grasp around egg
	for i in range(0,len(goal_pos)):
		goal_pos[i] -= int(motor_direction[i]*rotate_limit_an)
	move_dxl(goal_pos)

# UR5 arm move to egg
if UR5_on == True:
	robot.arm.move_ee_xyz(-ee_xyz, wait=True)

# read pressures
if sensor_on == True:
	_, pres, force = get_pres()
else:
	pres = np.zeros(num_fing)

# close fingers around egg until pressure threshold is reached
print('Fingers will close around egg')
if DXL_on == True:
	print('press y to move the fingers or any other key to stop')
	keypress = getch()
	if keypress == 'y':
		# while np.any(pres <= p_thresh) or np.any(force <= f_thresh):
		while (np.sum(pres <= p_thresh)>0.25*num_fing) or (np.sum(force <= f_thresh) > 0.25*num_fing):
			# keypress = getch()
			# if keypress == chr(0x1b):
				# print('Escaped from grasping')
				# break
			# elif keypress == 'y':
				diff = [0]*len(goal_pos)
				for i in range(0,len(goal_pos)):
					# check that pressure is below thresholds
					if (pres[i] <= p_thresh) and (force[i] <= f_thresh):
						# if (pres[i] <= (pres.mean() - p_margin)) and (force[i] <= (force.mean() - f_margin)):
						goal_pos[i] += motor_direction[i]*rotate_amount
						diff[i] += motor_direction[i]*rotate_amount
				print(diff)
				move_dxl(goal_pos)
				# read pressures
				_, pres, force = get_pres()

		print('Finished grasping')

# lift egg, pause, put back down
if UR5_on == True:
	robot.arm.move_ee_xyz(ee_xyz, wait=True)
	time.sleep(1.0)
	robot.arm.move_ee_xyz(-ee_xyz, wait=True)

# release egg
print('Fingers will release egg')
if DXL_on == True:
	# open grasp
	for i in range(0,len(goal_pos)):
		goal_pos[i] = motor_pos[i] - motor_direction[i]*rotate_limit_an
	print('press y to move the fingers or press ESC to stop')
	keypress = getch()
	if keypress == chr(0x1b):
		print('Escaped from grasping')
	elif keypress == 'y':
		move_dxl(goal_pos)

		time.sleep(1.0)
		print('Move motors away from egg. Fingers will now return to original position')

		move_dxl(motor_pos)

	shut_down(motor_ids, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=False)