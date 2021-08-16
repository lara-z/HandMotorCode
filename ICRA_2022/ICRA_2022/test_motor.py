from utils import *

COM_PORT = '/dev/ttyUSB0'
motor_id = [int(input('Enter the ID of the motor you would like to move:  '))]
rotate_amount = deg2pulse(20) # !!! change value

packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_id, COM_PORT)

def make_move(direction):
	pres_position = dxl_read(motor_id, packetHandler, groupBulkRead, ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)
	goal_position = [pres_position[0] + direction*rotate_amount]
	move(motor_id, goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
	print('moved')

print('Press \'c\' to change the motor id, \'f\' to flex, \'e\' to extend, or ESC to end')
while True:
	keypress = getch()
	if keypress == chr(0x1b):
		# end program, shut down motors
		shut_down(motor_id, packetHandler, portHandler, groupBulkRead, ADDR, LEN)
		break
	elif keypress == 'c':
		# change active motor, shut down previously used motor
		shut_down(motor_id, packetHandler, portHandler, groupBulkRead, ADDR, LEN)
		motor_id = [int(input('Enter the ID of the motor you would like to move:  '))]
		packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_id, COM_PORT)
	elif keypress == 'f':
		# flex finger
		make_move(1)
	elif keypress == 'e':
		# extend finger
		make_move(-1)
	else:
	    print('Invalid key. Press a different key.')
