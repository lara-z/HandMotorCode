# print joint position and test movement
import time
import numpy as np
import copy
import airobot as ar
from airobot import Robot
# from utils import *

robot = ar.Robot('ur5e', pb=False, use_cam=False)
# print('Current joint coordinates:')
# print(robot.arm.get_jpos())

def move_ur5(ur5_goal,sleep=True):
	if len(ur5_goal) == 3:
		robot.arm.move_ee_xyz(ur5_goal, wait=True)
	else:
		robot.arm.set_jpos(ur5_goal, wait=True)
	if sleep == True:
		time.sleep(1.5)

def getch():
	import os

	if os.name == 'nt':
		import msvcrt
		return msvcrt.getch().decode()
	else:
		import sys, tty, termios
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

start_pos = [1.759509563446045, -2.468517919579977, -1.2424640655517578, -2.234706541100973, 0.22429990768432617, 0.45635557174682617]
move_ur5(start_pos)

print('press y to move arm')
keypress = getch()
if keypress == chr(0x1b):
	print('Escaped from moving')
elif keypress == 'y':
	move_ur5([0,+0.09,0])

print('press y to move arm')
keypress = getch()
if keypress == chr(0x1b):
	print('Escaped from moving')
elif keypress == 'y':
	move_ur5([0,-0.09,0])