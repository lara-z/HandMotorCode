import time
import airobot as ar
from airobot import Robot
import numpy as np
# from utils import *
from utils_sensor import *

def check_pressure(hist):
	_, _, force, _, hist = read_pres(zeros, hist, args, ser, read_pts, print_pres=True)
	if ((pres >= thresh_tight) or (force >= force_tight)):
		tightening = False
		print('detected that screw has been tightened')
	else:
		print('checked pressure')
		tightening = True
	return tightening
	

# check port using:    python -m serial.tools.list_ports
# ls /dev/ttyUSB*
COM_PORT = '/dev/ttyUSB2'

sensor_on = False
visualize = False
class read_pts:	# indices of points to be read
    x_start = 0
    x_end = 3
    y_start = 28
    y_end = 32

wrist_ind = -1
rotate = np.pi*30/180
thresh_contact = 5 # threshold that there is contact
thresh_tight = 49 # threshold that nut is tight
force_tight = 60
ang_min = -np.pi/2 # must be more than -pi
rotate_reset = 1.3*np.pi
ang_max = ang_min + rotate_reset # must be less than +pi
z_ind = 2 # index for z-axis
z_lift_initial = 0.95 # vertical lift in meters
z_lift = 0.05 # vertical lift in meters
start_pos = [1.7633118629455566, -2.49069943050527, -1.2172584533691406, -0.9916332525065918, 1.575446605682373, ang_min]
# screw_pos = [-1.2097957769977015, -2.5426417789854945, -1.3618993759155273, -0.8092869085124512, 1.5712904930114746, ang_min]
move = False
tightening = True

robot = ar.Robot('ur5e', pb=False, use_cam=False)

# set start position and calibrate the sensor
robot.arm.set_jpos(start_pos, wait=True)
time.sleep(1.0)
args, ser, zeros, hist = initialize_sensor(com_port_sensor, visualize, read_pts)

# lower to screw
ee_xyz = [0]*3
ee_xyz[z_ind] = -z_lift_initial
robot.arm.move_ee_xyz(ee_xyz, wait=True)
time.sleep(1.0)
goal_pos = robot.arm.get_jpos()

_, pres, _, hist = read_pres(zeros, hist, args, ser, read_pts)

# keep tightening screw until tightness is achieved
while tightening:
	# motion to tighten screw until wrist limit is reached
	print('tightening screw')
	while goal_pos[wrist_ind] <= (ang_max - rotate):
	# while goal_pos[wrist_ind] >= (ang_max + rotate):
		tightening = check_pressure()
		if tightening == False:
			break
		goal_pos[wrist_ind] += rotate
		robot.arm.set_jpos(goal_pos, wait=True)
		time.sleep(0.4)
		tightening = check_pressure()
		if tightening == False:
			break

	# lift arm
	print('lifting arm')
	# ee_xyz, quat, rot, euler = robot.arm.get_ee_pose()
	ee_xyz = [0]*3
	ee_xyz[z_ind] = z_lift
	robot.arm.move_ee_xyz(ee_xyz, wait=True)

	if tightening == False:
		break
	else:
		time.sleep(1.0)

		# rotate back (workaround for wrist limit constraint)
		print('rotating wrist')
		goal_pos = robot.arm.get_jpos()
		goal_pos[wrist_ind] = ang_min
		robot.arm.set_jpos(goal_pos, wait=True)

		time.sleep(1.0)

		# lower arm
		print('lowering arm')
		ee_xyz = [0]*3
		ee_xyz[z_ind] = -z_lift
		robot.arm.move_ee_xyz(ee_xyz, wait=True)

		time.sleep(1.0)

		# read position and pressure
		goal_pos = robot.arm.get_jpos()
		_, pres, _, hist = read_pres(zeros, hist, args, ser, read_pts)
	
print('The screw has been tightened')

# while True:
# 	# read max or average pressure
# 	_, pres, _ = read_pres(p_zero, f_zero, t_data, args, ser)

# 	if (pres > thresh_contact) & (pres < thresh_tight):
# 		goal_pos[wrist_ind] += rotate
# 		move = True
# 	elif pres > thresh_tight:
# 		goal_pos[wrist_ind] -= rotate
# 		move = True
# 	if move == True:
# 		if goal_pos[wrist_ind] >= np.pi:
# 			goal_pos[wrist_ind] = 0.95*np.pi
# 		elif goal_pos[wrist_ind] <= -np.pi:
# 			goal_pos[wrist_ind] = -0.95*np.pi
# 		robot.arm.set_jpos(goal_pos, wait=True)
# 		goal_pos = robot.arm.get_jpos()
# 		move = False