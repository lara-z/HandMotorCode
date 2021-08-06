import airobot as ar
from airobot import Robot

robot = ar.Robot('ur5e', pb=False, use_cam=False)
z_move = 0.05 # !!! verify number
wrist_index = -1 # !!! verify number
wrist_move = 0.1 # !!! verify number

pos, quat, rot, euler = robot.arm.get_ee_pose()
print('this is the robot\'s current Cartesian pose:)
print(pos)

print('press \'m\' to move arm up by a small amount or ESCAPE to pass)
keypress = getch()
if keypress == chr(0x1b):
	print('Escaped from moving')
	break
elif keypress == 'm':
	pos[2] += z_move
	robot.arm.move_ee_xyz(pos, wait=True)
	
print('press \'r\' to rotate the wrist by a small amount or ESCAPE to pass)
if keypress == chr(0x1b):
	print('Escaped from moving')
	break
elif keypress == 'm':
	rot[wrist_index] += wrist_move
	robot.arm.set_jpos(rot, wait=True)
	
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
		return ch}

