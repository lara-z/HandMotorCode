import time
import airobot as ar
from airobot import Robot
import numpy as np
# from utils import *
from utils_sensor import *

def check_pressure():
	_, pres, _ = read_pres(p_zero, f_zero, args, ser)
	if pres > thresh_tight:
		tightening = False
	else:
		tightening = True
	return tightening
	

COM_PORT = '/dev/ttyUSB1'

sensor_on = False
visualize = False
t_data = 0.3 # during for pressure reading

wrist_ind = -1
rotate = np.pi*30/180
thresh_contact = 5 # threshold that there is contact
thresh_tight = 200 # threshold that nut is tight
ang_min = -np.pi/2 # must be more than -pi
rotate_reset = np.pi
ang_max = ang_min + rotate_reset # must be less than +pi
z_ind = 2 # index for z-axis
z_lift = 0.15 # vertical lift in meters
start_pos = [-1.2077434698687952, -2.5244132481017054, -1.352097511291504, -0.8730543416789551, 1.5568766593933105, ang_min]
move = False
tightening = True

robot = ar.Robot('ur5e', pb=False, use_cam=False)
args, ser, p_zero, f_zero = initialize_sensor(COM_PORT, visualize)

# set start position
robot.arm.set_jpos(start_pos, wait=True)
goal_pos = start_pos
pos, quat, rot, euler = robot.arm.get_ee_pose()
rot = np.round(rot)
robot.arm.set_ee_pose(ori=rot, wait=True)

_, pres, _ = read_pres(p_zero, f_zero, args, ser)

# keep tightening screw until tightness is achieved
while tightening:
	# motion to tighten screw until wrist limit is reached
	print('tightening screw')
	while goal_pos[wrist_ind] <= (ang_max - rotate):
		tightening = check_pressure()
		print(tightening)
		if tightening == False:
			break
		goal_pos[wrist_ind] += rotate
		robot.arm.set_jpos(goal_pos, wait=True)
		_, pres, _ = read_pres(p_zero, f_zero, args, ser)

	# lift arm
	print('lifting arm')
	# ee_xyz, quat, rot, euler = robot.arm.get_ee_pose()
	ee_xyz = [0]*3
	ee_xyz[z_ind] = z_lift
	robot.arm.move_ee_xyz(ee_xyz, wait=True)

	check_pressure()
	if tightening == False:
		break
	else:
		time.sleep(1.0)

		# rotate back (workaround for wrist limit constraint)
		print('rotating wrist')
		goal_pos = robot.arm.get_jpos()
		goal_pos[wrist_ind] = ang_min
		robot.arm.set_jpos(goal_pos, wait=True)

	check_pressure()
	if tightening == False:
		break
	else:
		time.sleep(1.0)

		# lower arm
		print('lowering arm')
		ee_xyz = [0]*3
		ee_xyz[z_ind] = -z_lift
		robot.arm.move_ee_xyz(ee_xyz, wait=True)

	check_pressure()
	if tightening == False:
		break
	else:
		time.sleep(1.0)

		# read position and pressure
		goal_pos = robot.arm.get_jpos()
		_, pres, _ = read_pres(p_zero, f_zero, args, ser)
	
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