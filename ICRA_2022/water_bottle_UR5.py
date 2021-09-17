# print joint position and test movement
import time
import numpy as np
import copy
import airobot as ar
from airobot import Robot
# from utils import *

robot = ar.Robot('ur5e', pb=False, use_cam=False)

def move_ur5(ur5_goal,sleep=False):
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

pos_start = [1.8013062477111816, -2.5462728939452113, -1.1996850967407227, -2.473063131371969, 0.2938375473022461, 0.7388815879821777]
pos_drop = [2.0392022132873535, -2.3850838146605433, -1.3258895874023438, -1.757352968255514, 0.7269549369812012, 0.09564733505249023]

print('Moving arm to start position')
move_ur5(pos_start)

print('press y to move arm to deposit bottle')
keypress = getch()
if keypress == chr(0x1b):
	print('Escaped from moving')
elif keypress == 'y':
    move_ur5(pos_drop)

print('press y to move arm back to starting position')
keypress = getch()
if keypress == chr(0x1b):
	print('Escaped from moving')
elif keypress == 'y':
    move_ur5(pos_start)