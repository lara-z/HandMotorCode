import airobot as ar
from airobot import Robot
	
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

robot = ar.Robot('ur5e', pb=False, use_cam=False)
z_move = 0.05
wrist_index = -1
wrist_move = 0.1

pos, quat, rot, euler = robot.arm.get_ee_pose()
print('this is the robot\'s current Cartesian pose:')
print(pos)

print('')

print('press \'m\' to move arm up by a small amount to location printed below or ESCAPE to pass')
pos[2] += z_move
print(pos)
keypress = getch()
if keypress == chr(0x1b):
	print('Escaped from moving')
elif keypress == 'm':
	robot.arm.move_ee_xyz(pos, wait=True)
	
print('moved')
print('')

jpos = robot.arm.get_jpos()
print('this is the robot\'s current joint angle pose:')
print(jpos)
	
print('press \'r\' to rotate the wrist by a small amount to location printed below or ESCAPE to pass')
jpos[wrist_index] += wrist_move
print(jpos)
keypress = getch()
if keypress == chr(0x1b):
	print('Escaped from moving')
elif keypress == 'r':
	robot.arm.set_jpos(jpos, wait=True)