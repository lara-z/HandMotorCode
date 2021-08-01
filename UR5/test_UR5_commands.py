import airobot as ar
from airobot import Robot


robot = ar.Robot('ur5e_2f140', pb=False, use_cam=False)
z_move = 0.1 # !!! verify number
wrist_index = 0 # !!! verify number
wrist_move = 0.1 # !!! verify number

pos, quat, rot, euler = robot.arm.get_ee_pose()
print('this is the robot\'s current Cartesian pose:)
print(pos)

print('press \'m\' to move arm up by a small amount or ESCAPE to pass)
if keypress == b'\x1b':
	print('Escaped from moving')
	break
elif keypress == b'm':
	pos[2] += z_move
	robot.arm.move_ee_xyz(pos, wait=True)
	
print('press \'r\' to rotate the wrist by a small amount or ESCAPE to pass)
if keypress == b'\x1b':
	print('Escaped from moving')
	break
elif keypress == b'm':
	rot[wrist_index] += wrist_move
	robot.arm.set_jpos(rot, wait=True)

