import time
import airobot as ar
from airobot import Robot
import numpy as np
import keyboard
from utils import *
from utils_sensor import *

COM_PORT = ''
sensor_on = False
visualize = False
t_data = 0.5 # during for pressure reading
start_pos = [] # !!! find good start position in joint coordinates
goal_pos = start_pos
wrist_ind = -1
rotate = np.pi*5/180
threshold = 10 # !!! change based on sensor

robot = ar.Robot('ur5e', pb=False, use_cam=False)
args, ser, p_zero, f_zero = initialize_sensor(COM_PORT, visualize)

# go to start position
robot.arm.set_jpos(start_pos, wait=True)

# read max or average pressure
 # !!! put in sensor reading
if sensor_on == True:
	_, pres, _ = read_pres(p_zero, f_zero, t_data, args, ser)
else:
	pres = 0

while pres < threshold:
	keypress = getch()
	print('press y to continue or press ESC to stop')
	if keypress == chr(0x1b):
		print('Escaped from grasping')
		break
	elif keypress == 'y':
		goal_pos[wrist_ind] += rotate
		robot.arm.set_jpos(goal_pos, wait=True)    	
	pres = 0 # !!! measure pressure
    	
# calculate end effector Cartesian position from joint coords
pos, quat, rot, euler = robot.arm.get_ee_pose()
ee_xyz = pos

# lift arm
ee_xyz[2] += 0.1 #!!! what units are these xyz in? meters?
robot.arm.move_ee_xyz(ee_xyz, wait=True)