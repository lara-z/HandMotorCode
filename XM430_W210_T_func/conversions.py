import os

if os.name == 'nt':
	import msvcrt
	def getch():
		return msvcrt.getch().decode()
else:
	import sys, tty, termios
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	def getch():
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

os.sys.path.append('../dynamixel_functions_py')             # Path setting

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import keyboard
import numpy as np

def initialize(DXL_TOTAL, com_num):
	import os

	if os.name == 'nt':
		import msvcrt
		def getch():
			return msvcrt.getch().decode()
	else:
		import sys, tty, termios
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		def getch():
			try:
				tty.setraw(sys.stdin.fileno())
				ch = sys.stdin.read(1)
			finally:
				termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
			return ch

	os.sys.path.append('../dynamixel_functions_py')             # Path setting

	# from dynamixel_sdk import *                    # Uses Dynamixel SDK library
	# import keyboard
	# import numpy as np

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
	DXL_MOVING_STATUS_THRESHOLD = 5                 # Dynamixel moving status threshold

	GOAL_TORQUE                 = 0.45
	JOINT_TORQUE                = [0.4, 0.4, 0.4, 0.4, 0.02, 0.02]  # Nm, desired torque for each joint motor ordered following DXL_TENSIONED
	TORQUE_MARGIN               = 0.06      # the maximum value above the desired torque that is still an acceptable torque value
	DESIRED_CURRENT             = torque2current(GOAL_TORQUE)            # desired current based on desired torque and converted to
	CURRENT_LIMIT               = torque2current(3.0)            # desired current based on desired torque and converted to
	DESIRED_PWM                 = int(100/0.113)     # desired PWM value in units of 0.113%
	VELOCITY_LIMIT              = int(3/0.229)
	ROTATE_AMOUNT               = deg2pulse(30)
	DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold
	DXL_CURRENT_THRESHOLD       = torque2current(0.08)

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
	# Set operating mode to current-based position control mode
	for motor_id in DXL_TOTAL:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR.OPERATING_MODE, CURRENT_BASED_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
			print("ID%d %s" % (motor_id, packetHandler.getTxRxResult(dxl_comm_result)))
		elif dxl_error != 0:
			print("ID%d %s" % (motor_id, packetHandler.getRxPacketError(dxl_error)))
		else:
			setup[0] +=1

	if setup[0] == len(DXL_TOTAL):
		print("All dynamixel operating modes have been successfully changed")

	# Set current limit
	for motor_id in DXL_TOTAL:
		dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR.PRO_CURRENT_LIMIT, CURRENT_LIMIT)
		if dxl_comm_result != COMM_SUCCESS:
			print("ID%d %s" % (motor_id, packetHandler.getTxRxResult(dxl_comm_result)))
		elif dxl_error != 0:
			print("ID%d %s" % (motor_id, packetHandler.getRxPacketError(dxl_error)))
		else:
			setup[1] +=1

	if setup[1] == len(DXL_TOTAL):
		print("All dynamixel current limit has been successfully changed")

	# Set desired current
	for motor_id in DXL_TOTAL:
		dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR.PRO_GOAL_CURRENT, DESIRED_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("ID%d %s" % (motor_id, packetHandler.getTxRxResult(dxl_comm_result)))
		elif dxl_error != 0:
			print("ID%d %s" % (motor_id, packetHandler.getRxPacketError(dxl_error)))
		else:
			setup[2] +=1

	if setup[2] == len(DXL_TOTAL):
		print("All dynamixel goal current values have been successfully changed")

	# Set desired PWM
	for motor_id in DXL_TOTAL:
		dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR.PRO_GOAL_PWM, DESIRED_PWM)
		if dxl_comm_result != COMM_SUCCESS:
			print("ID%d %s" % (motor_id, packetHandler.getTxRxResult(dxl_comm_result)))
		elif dxl_error != 0:
			print("ID%d %s" % (motor_id, packetHandler.getRxPacketError(dxl_error)))
		else:
			setup[3] +=1

	if setup[3] == len(DXL_TOTAL):
		print("All dynamixel goal PWM values have been successfully changed")

	# Set drive mode
	for motor_id in DXL_TOTAL:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR.PRO_DRIVE_MODE, VELOCITY_BASED_PROFILE)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		else:
			setup[4] +=1

	if setup[4] == len(DXL_TOTAL):
		print("All dynamixel drive modes have been successfully changed to velocity-based")

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
	dxl_present_position = [0]*max(DXL_TOTAL)
	dxl_comm_result = [0]*max(DXL_TOTAL)
	dxl_error = [0]*max(DXL_TOTAL)
	dxl_goal_position =  [0]*max(DXL_TOTAL)
	for motor_id in DXL_TOTAL:
		dxl_present_position[motor_id-1], dxl_comm_result[motor_id-1], dxl_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR.PRO_PRESENT_POSITION)
	dxl_goal_position = dxl_present_position.copy()

	return dxl_present_position, dxl_goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN

def shut_down(DXL_TOTAL, packetHandler, portHandler, groupBulkRead, ADDR, LEN):
	TORQUE_DISABLE = 0                 # Value for disabling the torque

	# Clear bulkread parameter storage
	groupBulkRead.clearParam()

	print('Disable torque? y/n')
	keypress = msvcrt.getch()
	if keypress == b'y':
		for motor_id in DXL_TOTAL:
			dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR.PRO_TORQUE_ENABLE, TORQUE_DISABLE)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % packetHandler.getRxPacketError(dxl_error))
		print("torques disabled")
	elif keypress == b'n':
		print('motors still on')

	# Close port
	portHandler.closePort()

def calc_torque(DXLS, ifprint, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN):
    # calculate the torque based on measured current
    error_status = False
    dxl_present_current = [0]*max(DXLS)
    torque = [0]*max(DXLS)
    for motor_id in DXLS:
        # Read present current
        dxl_present_current[motor_id-1], dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, motor_id, ADDR.PRO_PRESENT_CURRENT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("[ID:%-2d]: %s" % (motor_id, packetHandler.getRxPacketError(dxl_error)))
            dxl_error_message, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, motor_id, ADDR.PRO_HARDWARE_ERROR)
            error_status = True

        # convert current unit to amps
        dxl_present_current[motor_id-1] = curr2Amps(dxl_present_current[motor_id-1])

        if dxl_present_current[motor_id-1] > 150:
            # when the cables are slack, the current is really large, but toque is actually zero so set to zero
            torque[motor_id-1] = 0
        else:
            torque[motor_id-1] = current2torque(dxl_present_current[motor_id-1])
    if ifprint == True:
        measurements = ''
        for motor_id in DXLS:
            # measurements = measurements + ("    [ID:%02d] PresTorque:%3.2fNm, PresCurrent:%8.4fA" % (DXL_TENSIONED[i], torque[i], dxl_present_current[i]))
            measurements = measurements + ("    [ID:%02d] Torque: %3.2fNm" % (motor_id, torque[motor_id-1]))
        print(measurements)
    return torque, dxl_present_current, error_status

def move(DXL_TOTAL, dxl_present_position, goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN):
    # move the finger to a goal position that's defined in the code

    DXL_MOVING_STATUS_THRESHOLD = 5                 # Dynamixel moving status threshold

    param_goal_position = [0]*max(DXL_TOTAL)
    for motor_id in DXL_TOTAL:
        # Allocate goal position value into byte array
        param_goal_position[motor_id-1] = [DXL_LOBYTE(DXL_LOWORD(goal_position[motor_id-1])), DXL_HIBYTE(DXL_LOWORD(goal_position[motor_id-1])), DXL_LOBYTE(DXL_HIWORD(goal_position[motor_id-1])), DXL_HIBYTE(DXL_HIWORD(goal_position[motor_id-1]))]

        # Add Dynamixel goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = groupBulkWrite.addParam(motor_id, ADDR.PRO_GOAL_POSITION, LEN.PRO_GOAL_POSITION, param_goal_position[motor_id-1])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % motor_id)
            quit()

    # Bulkwrite goal position
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()

    moving = True
    while moving:
        # Bulkread present positions
        dxl_comm_result = groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        # print('Bulkread present positions')

        # Check if groupbulkread data is available
        for motor_id in DXL_TOTAL:
            dxl_getdata_result = groupBulkRead.isAvailable(motor_id, ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % motor_id)
                quit()

        # Get present position value
        for motor_id in DXL_TOTAL:
            dxl_present_position[motor_id-1] = groupBulkRead.getData(motor_id, ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)

        torque, dxl_present_current, error_status = calc_torque(DXL_TOTAL, False, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
        if error_status == True:
            moving = False
        # print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position : %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))

        for motor_id in DXL_TOTAL:
            if not (abs(goal_position[motor_id-1] - dxl_present_position[motor_id-1]) > DXL_MOVING_STATUS_THRESHOLD):
                moving = False
    return dxl_present_position

def tension(dxl_present_position, dxl_goal_position, DXL_TOTAL, DXL_TENSIONED, JOINT_TORQUE, TORQUE_MARGIN, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN):
    # keep moving the finger to the desired torque until the desired torque is achieved

    dxl_comm_result = [0]*max(DXL_TOTAL)
    dxl_error = [0]*max(DXL_TOTAL)

    for motor_id in DXL_TOTAL:
        dxl_present_position[motor_id-1], dxl_comm_result[motor_id-1], dxl_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR.PRO_PRESENT_POSITION)

    ROTATE_AMOUNT = 5
    torque, dxl_present_current, error_status = calc_torque(DXL_TOTAL, True, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)

    num_tensioned = 0;
    print('Start tensioning')
    while (num_tensioned < len(DXL_TENSIONED)) & (error_status == False):
        # write new goal position
        num_tensioned = 0;
        count = 0
        for motor_id in DXL_TENSIONED:
            if round(torque[motor_id-1],2) > (JOINT_TORQUE[count] + 0.2):
                new_goal = - 3*ROTATE_AMOUNT
            elif round(torque[motor_id-1],2) > (JOINT_TORQUE[count] + TORQUE_MARGIN):
                new_goal = - ROTATE_AMOUNT
            elif round(torque[motor_id-1],2) < 0.05:
                new_goal =  5*ROTATE_AMOUNT
            elif round(torque[motor_id-1],2) < JOINT_TORQUE[count]:
                new_goal = ROTATE_AMOUNT
            else:
                new_goal = 0
                num_tensioned += 1
            dxl_goal_position[motor_id-1] = dxl_goal_position[motor_id-1] + new_goal
            count = count + 1

        dxl_present_position = move(DXL_TOTAL, dxl_present_position, dxl_goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
        torque, dxl_present_current, error_status = calc_torque(DXL_TOTAL, False, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
    if error_status == True:
        print('Error. Stopped tensioning.')
    else:
        print('Finished tensioning')

    return dxl_present_position, dxl_goal_position

def read_pres_pos(dxl_present_position, DXL_TOTAL, packetHandler, portHandler, ADDR):
    # read the present motor position for all motors
    dxl_comm_result = [0]*max(DXL_TOTAL)
    dxl_error = [0]*max(DXL_TOTAL)

    for motor_id in DXL_TOTAL:
        dxl_present_position[motor_id-1], dxl_comm_result[motor_id-1], dxl_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR.PRO_PRESENT_POSITION)
    return dxl_present_position

####### math conversions ######
def deg2pulse(deg):
    # converts from degrees of joint displacement to motor pulses
    ratio = 0.45
    return int(deg*(ratio/0.088))

def curr2Amps(current_reading):
    return float(current_reading*2.69/1000)

def torque2current(torque):
    # convert based on tau = Kt*I
    current_amps = (torque*0.95 + 0.1775)
    # convert to current units used by motor
    return int(current_amps*1000/2.69)

def current2torque(current_amps):
    return float((current_amps - 0.1775)/0.95)
