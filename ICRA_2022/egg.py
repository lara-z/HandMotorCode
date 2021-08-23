import time
# import airobot as ar
# from airobot import Robot
import numpy as np
from utils import *
from utils_sensor import *

UR5_on = False
DXL_on = True
sensor_on = True

ee_start_pos = [] # !!! find good start position
lift_ind = 2
lift_amount = 0.2
ee_xyz = []*3
ee_xyz[lift_ind] = lift_amount

com_port_dxl = '/dev/ttyUSB0' # !!! update
com_port_sensor = '/dev/ttyUSB1' # !!! update
operating_mode = 'extended_position'

rotate_amount = deg2pulse(5) # !!! change once gear ratio and units have been established
rotate_open = deg2pulse(60) # !!! update
motor_ids = [7,4,3,1]
motor_direction = [-1,-1,1,1] # make values -1 in case agonist or antagonist cables were switched during assembly
dxl_limits = [0]*len(motor_ids)

visualize = False
num_fing = len(motor_ids)
pres = [0]*num_fing
class read_pts:
	# order: small left, big left, big right, small right finger
    x_start = [2,0,6,4]
    x_end   = [4,2,8,6]
    y_start = [0,3,6,10]
    y_end   = [3,6,9,12]
p_thresh = 50
f_thresh = 50

# !!! change to multithread so fingers can move simultaneously?

def get_pres():
	mean_pres, max_pres, force = read_pres(p_zero, f_zero, args, ser, read_pts, print_pres=True)
	return mean_pres, max_pres, force

def move_dxl():
	_ = move(motor_ids, goal_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN, limits = dxl_limits)

# robot = ar.Robot('ur5e', pb=False, use_cam=False)

# UR5 arm go to start position
if UR5_on == True:
	robot.arm.set_jpos(ee_start_pos, wait=True)

# initialize dynamixel and open fingers
if DXL_on == True:
	# initialize dynamixels with fingers in ending position
	motor_pos, goal_pos, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_ids, com_port_dxl, operating_mode)
	# set joint limits
	for i in range(num_fing):
		dxl_limits[i] = [int(motor_pos[i]) - deg2pulse(30), int(motor_pos[i]) + deg2pulse(30)]
		dxl_limits[i] = [int(motor_pos[i]) - 2*deg2pulse(30), int(motor_pos[i])]
	# open fingers to grasp around egg
	for i in range(0,len(goal_pos)):
		goal_pos[i] -= int(motor_direction[i]*rotate_open/2)
	move_dxl()

if sensor_on == True:
	args, ser, p_zero, f_zero = initialize_sensor(com_port_sensor, visualize, read_pts)

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
	print('press y to move the fingers or press ESC to stop')
	while np.any(pres <= p_thresh) or np.any(force <= f_thresh):
	# while any(i <= p_thresh for i in pres):
		keypress = getch()
		if keypress == chr(0x1b):
			print('Escaped from grasping')
			break
		elif keypress == 'y':
			for i in range(0,len(goal_pos)):
				if (pres[i] <= p_thresh) or (force[i] <= f_thresh):
					goal_pos[i] += motor_direction[i]*rotate_amount
			move_dxl()
			# read pressures
			_, pres, _ = get_pres()

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
		goal_pos[i] -= motor_direction[i]*rotate_open
	print('press y to move the fingers or press ESC to stop')
	keypress = getch()
	if keypress == chr(0x1b):
		print('Escaped from grasping')
	elif keypress == 'y':
		move_dxl()

	shut_down(motor_ids, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=False)