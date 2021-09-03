import time
import airobot as ar
from airobot import Robot
import numpy as np
from utils import *
from utils_sensor import *

UR5_on = True
DXL_on = True
sensor_on = True

# check port using:    dmesg
com_port_dxl = '/dev/ttyUSB1' # '/dev/ttyUSB0'
com_port_sensor = '/dev/ttyUSB0'

# establish UR5 variables
egg_pos = 'holder' # options are 'holder' or 'ground'
class holder:
	start_pos1 = [1.3283896446228027, -2.2250029049315394, -1.8629846572875977, -2.2158452473082484, -1.740652863179342, -0.7816255728351038]
	start_pos2 = [1.3281621932983398, -2.5985170803465785, -1.7919988632202148, -1.9135986767210902, -1.7407367865191858, -0.783447567616598] # arm goes down level with the table
	start_pos3 = [1.3709359169006348, -2.702686449090475, -1.4833993911743164, -2.117897172967428, -1.6987865606891077, -0.7813981215106409] # arm moves up to egg
ground_start_pos = [1.355161190032959, -2.4329525432982386, -1.095463752746582, -1.1849048894694825, 1.5603728294372559, -2.5464335123645228]
ee_twist_pos_1 = [1.8218579292297363, -2.4279991588988246, -0.9796571731567383, -2.820000787774557, 0.255401611328125, -0.8206332365619105]
ee_twist_pos_2 = np.copy(ee_twist_pos_1)
ee_twist_pos_2[-1] += np.pi/2
ee_twist_pos_3 = np.copy(ee_twist_pos_1)
ee_twist_pos_3[-1] += np.pi
lift_ind = 2
lift_amount = -10.4/100
ee_xyz = np.zeros(3)
ee_xyz[lift_ind] = lift_amount

# establish dxl variables
operating_mode = 'current_position'
current_des = amps2curr(2.0)	# for current-based position control
current_lim = amps2curr(2.5)	# for current-based position control
rotate_amount = deg2pulse(5)	# amount that motor rotates to close grasp around egg
rotate_limit_ag = 650 # !!! 350
rotate_limit_an = deg2pulse(30)                                # !!! update
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
	_ = move(motor_ids, dxl_goal, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN, print_currvolt=False, limits = dxl_limits)

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

# UR5 arm go to start position
if UR5_on == True:
	robot = ar.Robot('ur5e', pb=False, use_cam=False)
	if egg_pos == 'holder':
		move_ur5(holder.start_pos1)
	elif egg_pos == 'ground':
		move_ur5(ground_start_pos)

# initialize sensors
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
	robot = ar.Robot('ur5e', pb=False, use_cam=False)
	if egg_pos == 'holder':
		move_ur5(holder.start_pos2)
		move_ur5(holder.start_pos3)
	elif egg_pos == 'ground':
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
	lifted_pos = robot.arm.get_jpos()
	# twist to show egg bottom
	move_ur5(ee_twist_pos_1)
	shake()
	# rotate wrist 90 deg
	move_ur5(ee_twist_pos_2)
	shake()
	# rotate wrist 180 deg
	move_ur5(ee_twist_pos_3)
	shake()
	# move wrist back to pre-twisted position
	move_ur5(lifted_pos)
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
		# reset motors because they usually error can therefore won't release the egg
		reboot(packetHandler, portHandler, motor_ids, com_port_dxl, operating_mode)
		shut_down(motor_ids, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=False)
		_, _, packetHandler, portHandler, groupBulkWrite, groupBulkRead, _, _ = initialize(motor_ids, com_port_dxl, operating_mode, current_des, current_lim)
		# release egg
		move_dxl(goal_pos)

if UR5_on:
	# move arm back up to starting position
	move_ur5(ee_start_pos)

if DXL_on:
	# move fingers back to start position
	move_dxl(motor_pos)
	shut_down(motor_ids, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=False)