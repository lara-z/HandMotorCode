import airobot as ar
from airobot import Robot
import numpy as np
import time
robot = ar.Robot('ur5e', pb=False, use_cam=False)

def move_ur5(ur5_goal,sleep=True):
	if len(ur5_goal) == 3:
		robot.arm.move_ee_xyz(ur5_goal, wait=True)
	else:
		robot.arm.set_jpos(ur5_goal, wait=True)
	if sleep == True:
		time.sleep(1.5)

def shake():
	# shake ur5 arm to show egg can't drop
    sleep=False
    shake_coords = np.array([0,0,0.02])
    move_ur5(shake_coords,sleep)
    move_ur5(shake_coords,sleep)
    move_ur5(shake_coords,sleep)
    move_ur5(shake_coords,sleep)