import os

def getch():
	# get keyboard stroke based on mac or windows operating system

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

def toggle_control(mode):
	# in manual mode, allow user to incrementally check in on task if mode is true
	# in automatic mode, continue without asking
	# example use, place in while loop to ask to continue to move motors to close around object until sensor reading is achieved
	# inteded to help catch motion errors before disasters happen
	run = False
	if mode == 'manual':
		print('Press ''y'' to continue or ESC to skip')
		keypress = getch()
		if keypress == chr(0x1b):
			print('Escaped from grasping')
			run = False
		elif keypress == 'y':
			run = True
	else:
		run = True
	return run