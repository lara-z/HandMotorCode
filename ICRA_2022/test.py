from utils import *

COM_PORT = 'COM3'
motor_id = [int(input('Enter the ID of the motor you would like to move:  '))]
rotate_amount = 100 # !!! change value

packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_id, COM_PORT)


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
		pres_position = dxl_get_pos(motor_id, packetHandler, groupBulkRead, ADDR, LEN)
		goal_position = [pres_position[0] + rotate_amount]
		move(motor_id, goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
	elif keypress == 'e':
		# extend finger
		pres_position = dxl_get_pos(motor_id, packetHandler, groupBulkRead, ADDR, LEN)
		goal_position = [pres_position[0] - rotate_amount]
		move(motor_id, goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
	else:
	    print('Invalid key. Press a different key.')
