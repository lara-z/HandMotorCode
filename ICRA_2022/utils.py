from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# initialize dynamixels
# Set goal position as present position
# Return a bunch of handles necessary to accessing dynamixels
def initialize(DXL_TOTAL, com_num):
	import os

# 	if os.name == 'nt':
# 		import msvcrt
# 		def getch():
# 			return msvcrt.getch().decode()
# 	else:
# 		import sys, tty, termios
# 		fd = sys.stdin.fileno()
# 		old_settings = termios.tcgetattr(fd)
# 		def getch():
# 			try:
# 				tty.setraw(sys.stdin.fileno())
# 				ch = sys.stdin.read(1)
# 			finally:
# 				termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
# 			return ch

	os.sys.path.append('../dynamixel_functions_py')             # Path setting

	import keyboard
	import numpy as np

	manual_control              = True

	# Control table address
	class ADDR:
		OPERATING_MODE         = 11
		PRO_TORQUE_ENABLE      = 64
		PRO_LED_RED            = 65
		PRO_GOAL_POSITION      = 116
		PRO_PRESENT_POSITION   = 132
		PRO_CURRENT_LIMIT      = 38
		PRO_GOAL_CURRENT       = 102
		PRO_PRESENT_CURRENT    = 126
		PRO_GOAL_PWM           = 100
		PRO_HARDWARE_ERROR     = 70
		PRO_DRIVE_MODE         = 10
		PRO_VELOCITY_PROFILE   = 112

	# Data Byte Length
	class LEN:
		PRO_GOAL_POSITION       = 4
		PRO_PRESENT_POSITION    = 4
		PRO_CURRENT             = 2
		PRO_PWM                 = 2

	# Protocol version
	PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

	BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
	DEVICENAME                  = com_num            # Check which port is being used on your controller

	OPERATION_MODE              = 0x02
	EXT_POSITION_CONTROL_MODE   = 4                 # Value for extended position control mode (operating mode)
	CURRENT_BASED_POSITION      = 5                 # Value for current-based position control mode (operating mode)
	VELOCITY_BASED_PROFILE      = 0
	TORQUE_ENABLE               = 1                 # Value for enabling the torque
	TORQUE_DISABLE              = 0                 # Value for disabling the torque
	DXL_MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold

	GOAL_TORQUE                 = 0.45
	JOINT_TORQUE                = [0.4, 0.4, 0.4, 0.4, 0.02, 0.02]  # Nm, desired torque for each joint motor ordered following DXL_TENSIONED
	# TORQUE_MARGIN               = 0.06      # the maximum value above the desired torque that is still an acceptable torque value
	# DESIRED_CURRENT             = torque2current(GOAL_TORQUE)            # desired current based on desired torque and converted to
	# CURRENT_LIMIT               = torque2current(3.0)            # desired current based on desired torque and converted to
	# DESIRED_PWM                 = int(100/0.113)     # desired PWM value in units of 0.113%
	VELOCITY_LIMIT              = int(3/0.229)
	ROTATE_AMOUNT               = deg2pulse(30)
	DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold
	# DXL_CURRENT_THRESHOLD       = torque2current(0.08)

	# Initialize PortHandler instance
	# Set the port path
	# Get methods and members of PortHandlerLinux or PortHandlerWindows
	portHandler = PortHandler(DEVICENAME)

	# Initialize PacketHandler instance
	# Set the protocol version
	# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	packetHandler = PacketHandler(PROTOCOL_VERSION)

	# Initialize GroupBulkWrite instance
	groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

	# Initialize GroupBulkRead instace for Present Position
	groupBulkRead = GroupBulkRead(portHandler, packetHandler)

	# Open port
	if portHandler.openPort():
		print("Succeeded to open the port")
	else:
		print("Failed to open the port")
		print("Press any key to terminate...")
		getch()
		quit()

	# Set port baudrate
	if portHandler.setBaudRate(BAUDRATE):
		print("Succeeded to change the baudrate")
	else:
		print("Failed to change the baudrate")
		print("Press any key to terminate...")
		getch()
		quit()

	# set up motors
	setup = [0]*7
	# Set operating mode to extended position control mode
	for motor_id in DXL_TOTAL:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR.OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		else:
			setup[0] +=1

	if setup[0] == len(DXL_TOTAL):
		print("All dynamixel operating modes have been successfully changed")
	else:
		print('Error(s) encountered. Shutting down motors...')
		shut_down(DXL_TOTAL, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=False)
		print('PLEASE RE-RUN CODE. The motor error should now be fixed (torque has been turned off)')
		quit()

	# # Set drive mode
	# for motor_id in DXL_TOTAL:
	# 	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR.PRO_DRIVE_MODE, VELOCITY_BASED_PROFILE)
	# 	if dxl_comm_result != COMM_SUCCESS:
	# 		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	# 	elif dxl_error != 0:
	# 		print("%s" % packetHandler.getRxPacketError(dxl_error))
	# 	else:
	# 		setup[4] +=1
	#
	# if setup[4] == len(DXL_TOTAL):
	# 	print("All dynamixel drive modes have been successfully changed to velocity-based")

	# Enable Torque
	for motor_id in DXL_TOTAL:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR.PRO_TORQUE_ENABLE, TORQUE_ENABLE)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		else:
			setup[5] +=1

	if setup[5] == len(DXL_TOTAL):
		print("All dynamixels now have torque enabled")

	# Add parameter storage for present position
	for motor_id in DXL_TOTAL:
		dxl_addparam_result = groupBulkRead.addParam(motor_id, ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupBulkRead addparam failed" % motor_id)
			quit()

	# set initial goal position to current position
	dxl_present_position = dxl_get_pos(DXL_TOTAL, packetHandler, groupBulkRead, ADDR, LEN)
	dxl_goal_position = dxl_present_position.copy()

	return packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN

def shut_down(DXL_TOTAL, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=True):
	TORQUE_DISABLE = 0                 # Value for disabling the torque

	# Clear bulkread parameter storage
	groupBulkRead.clearParam()

	if  askAction==True:
		print('Disable torque? y/n')
		keypress = getch()
	else:
		keypress = 'y'

	if keypress == 'y':
		for motor_id in DXL_TOTAL:
			dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR.PRO_TORQUE_ENABLE, TORQUE_DISABLE)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % packetHandler.getRxPacketError(dxl_error))
		print("torques disabled")
	elif keypress == 'n':
		print('motors still on')

	# Close port
	portHandler.closePort()

def move(DXL_TOTAL, goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN):
	# will line 249 error with DXL_LOBYTE, etc.?
    # move the finger to a goal position that's defined in the code

	DXL_MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold

	param_goal_position = [0]*len(DXL_TOTAL)
	count = 0
	for motor_id in DXL_TOTAL:
		# Allocate goal position value into byte array
		param_goal_position[count] = [DXL_LOBYTE(DXL_LOWORD(goal_position[count])), DXL_HIBYTE(DXL_LOWORD(goal_position[count])), DXL_LOBYTE(DXL_HIWORD(goal_position[count])), DXL_HIBYTE(DXL_HIWORD(goal_position[count]))]

        # Add Dynamixel goal position value to the Bulkwrite parameter storage
		dxl_addparam_result = groupBulkWrite.addParam(motor_id, ADDR.PRO_GOAL_POSITION, LEN.PRO_GOAL_POSITION, param_goal_position[count])
		if dxl_addparam_result != True:
			print("[ID:%03d] groupBulkWrite addparam failed" % motor_id)
			quit()
			count += 1

    # Bulkwrite goal position
	dxl_comm_result = groupBulkWrite.txPacket()
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear bulkwrite parameter storage
	groupBulkWrite.clearParam()

	moving = True
	while moving:
		print('moving')
        # get present position
		dxl_present_position = dxl_get_pos(DXL_TOTAL, packetHandler, groupBulkRead, ADDR, LEN)

		for count in range(0,len(dxl_present_position)):
			if not (abs(goal_position[count] - dxl_present_position[count]) > DXL_MOVING_STATUS_THRESHOLD):
				moving = False
	print('***')
	return dxl_present_position

def dxl_get_pos(DXL_IDS, packetHandler, groupBulkRead, ADDR, LEN):
	# Bulkread present positions
	dxl_comm_result = groupBulkRead.txRxPacket()
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

	# Check if groupbulkread data is available
	for motor_id in DXL_IDS:
		dxl_getdata_result = groupBulkRead.isAvailable(motor_id, ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupBulkRead getdata failed" % motor_id)
			quit()

	# Get present position value
	motor_pos = [0]*len(DXL_IDS)
	count = 0
	for motor_id in DXL_IDS:
		motor_pos[count] = groupBulkRead.getData(motor_id, ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)
		count += 1
	return motor_pos

# get keyboard stroke based on mac or windows operating system
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

def deg2pulse(deg):
    # converts from degrees of joint displacement to motor pulses
    ratio = 1
    return int(deg*(ratio/0.088))
