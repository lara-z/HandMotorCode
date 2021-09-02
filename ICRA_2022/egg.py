import time
# import airobot as ar
# from airobot import Robot
import numpy as np
from utils import *
from utils_sensor import *

UR5_on = True
DXL_on = True
sensor_on = False

# check port using:    python -m serial.tools.list_ports
# ls /dev/ttyUSB*
com_port_dxl = '/dev/ttyS0' # '/dev/ttyUSB0'
com_port_sensor = '/dev/ttyUSB1'

# establish UR5 variables
ee_start_pos = [1.3510046005249023, -2.42978634456777, -1.0763025283813477, -1.229790524845459, 1.5926413536071777, 0.5379843711853027]
ee_twist_pos = [1.802238941192627, -2.489427228967184, -0.715703010559082, -2.9041978321471156, 0.32659387588500977, 2.0060067176818848+0.2]
ee_twist_pos_rot = ee_twist_pos
ee_twist_pos_rot[-1] -= np.pi/2
lift_ind = 2
lift_amount = -0.109
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
	mean_pres, max_pres, force, _ = read_pres(p_zero, f_zero, x_zero, args, ser, read_pts, print_pres=True)
	return mean_pres, max_pres, force

def move_dxl(dxl_goal):
	_ = move(motor_ids, dxl_goal, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN, print_currvolt=True, limits = dxl_limits)

def shake():
	# shake ur5 arm to show egg can't drop
    sleep=False
    move_ur5(0.3*ee_xyz,sleep)
    move_ur5(-0.3*ee_xyz,sleep)
    move_ur5(0.3*ee_xyz,sleep)
    move_ur5(-0.3*ee_xyz,sleep)

def move_ur5(ur5_goal,sleep=True):
	if len(ur5_goal) == 3:
		robot.arm.move_ee_xyz(ur5_goal, wait=True)
	else:
		robot.arm.set_jpos(ur5_goal, wait=True)
	if sleep == True:
		time.sleep(1.5)

# robot = ar.Robot('ur5e', pb=False, use_cam=False)

# UR5 arm go to start position
if UR5_on == True:
	move_ur5(ee_start_pos)

if sensor_on == True:
	args, ser, p_zero, f_zero, x_zero = initialize_sensor(com_port_sensor, visualize, read_pts)

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
	move_ur5(ee_xyz)

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

# lift egg, pause, rotate, shake, put back down
if UR5_on == True:
	# lift
	move_ur5(-ee_xyz)
	# twist to show egg bottom
	move_ur5(ee_twist_pos)
	shake()
	# rotate wrist 90 deg
	move_ur5(ee_twist_pos)
	shake()
	# move wrist back to starting position
	move_ur5(ee_start_pos)
	# move arm down
	robot.arm.move_ee_xyz(ee_xyz)

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

# move arm back up to starting position
move_ur5(ee_start_pos)