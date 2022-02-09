import os
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import numpy as np
from .conversions import *
from .misc import *

"""
	DXL_IDS
	format: list
	what: id numbers of each motor
	established: input from user
"""
"""
	com_num
	format: string
	what: usb port that dynamixel motors are plugged into
	established: input from user
"""
"""
	operating mode
	format: string
	what: indicates desired operating mode of dynamixel motors
	options: 'extended_position', 'current_position'
	established: input from user
"""
"""
	SYS
	format: class
	what: low-level functions to communicate with dynamixel motors
	contains functions: portHandler, packetHandler, groupSyncWrite, groupBulkRead
	established: in utils_motor/initialize()
"""
"""
	ADDR
	format: class
	what: dynamixel control table addresses, see initialize()
	established: in utils_motor/initialize()
"""
"""
	LEN
	format: class
	what: integer byte lengths corresponding to dynamixel control table addresses, see initialize()
	established: in utils_motor/initialize()
"""

def initialize(DXL_IDS, com_num, operating_mode, DESIRED_CURRENT=500, CURRENT_LIMIT=1193):
	# initialize motors
	# returns dxl_present_position, dxl_goal_position, SYS, ADDR, LEN
	# operating_mode accepts two inputs: current_position, extended_position

	os.sys.path.append('../dynamixel_functions_py')             # Path setting

	# Control table address
	class ADDR:
		OPERATING_MODE         = 11
		PRO_MOVING_THRES	   = 24
		PRO_TORQUE_ENABLE      = 64
		PRO_LED_RED            = 65
		PRO_HARDWARE_ERROR     = 70
		PRO_GOAL_POSITION      = 116
		PRO_PRESENT_POSITION   = 132
		PRO_CURRENT_LIMIT      = 38
		PRO_GOAL_CURRENT       = 102
		PRO_MOVING			   = 122
		PRO_PRESENT_CURRENT    = 126
		PRO_GOAL_PWM           = 100
		PRO_PRESENT_PWM        = 124
		PRO_PRESENT_INPUT_VOLTAGE      = 144

	# Data Byte Length
	class LEN:
		PRO_GOAL_POSITION       = 4
		PRO_PRESENT_POSITION    = 4
		PRO_PMOVING_THRES	    = 4
		PRO_CURRENT             = 2
		PRO_PWM                 = 2
		PRO_VOLT                = 2
		PRO_MOVING				= 1

	# Protocol version
	PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

	BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
	DEVICENAME                  = com_num            # Check which port is being used on your controller

	OPERATION_MODE              = 0x02
	EXT_POSITION_CONTROL_MODE   = 4                 # Value for extended position control mode (operating mode)
	CURRENT_BASED_POSITION      = 5                 # Value for current-based position control mode (operating mode)
	TORQUE_ENABLE               = 1                 # Value for enabling the torque

	MOVING_THRESHOLD			= 4
	DESIRED_PWM                 = int(100/0.113)     # desired PWM value in units of 0.113%

	class SYS:
		# Initialize PortHandler instance
		# Set the port path
		# Get methods and members of PortHandlerLinux or PortHandlerWindows
		portHandler = PortHandler(DEVICENAME)

		# Initialize PacketHandler instance
		# Set the protocol version
		# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
		packetHandler = PacketHandler(PROTOCOL_VERSION)

		# Initialize GroupSyncWrite instance
		groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR.PRO_GOAL_POSITION, LEN.PRO_GOAL_POSITION)

		# Initialize GroupBulkRead instace for Present Position
		groupBulkRead = GroupBulkRead(portHandler, packetHandler)

	# Open port
	if SYS.portHandler.openPort():
		print("Succeeded to open the port")
	else:
		print("Failed to open the port")
		print("Press any key to terminate...")
		getch()
		quit()

	# Set port baudrate
	if SYS.portHandler.setBaudRate(BAUDRATE):
		print("Succeeded to change the baudrate")
	else:
		print("Failed to change the baudrate")
		print("Press any key to terminate...")
		getch()
		quit()

	# set up motors
	setup = [0]*6
	# Set operating mode to current-based position control mode
	for motor_id in DXL_IDS:
		if operating_mode == 'extended_position':
			dxl_comm_result, dxl_error = SYS.packetHandler.write1ByteTxRx(SYS.portHandler, motor_id, ADDR.OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
		elif operating_mode == 'current_position':
			dxl_comm_result, dxl_error = SYS.packetHandler.write1ByteTxRx(SYS.portHandler, motor_id, ADDR.OPERATING_MODE, CURRENT_BASED_POSITION)
		setup[0] += check4TxRxerror(motor_id, SYS, ADDR, dxl_comm_result, dxl_error)

	if setup[0] == len(DXL_IDS):
		if operating_mode == 'extended_position':
			print("All dynamixel operating modes have been successfully changed to extended position control")
		elif operating_mode == 'current_position':
			print("All dynamixel operating modes have been successfully changed to current-based position control")
	else:
		print('Error(s) encountered. Rebooting and shutting down motors...')
		reboot(SYS, DXL_IDS)
		shut_down(DXL_IDS, SYS, ADDR, LEN, askAction=False)
		print('PLEASE RE-RUN CODE. The motor error should now be fixed')
		quit()

	for motor_id in DXL_IDS:
		if operating_mode == 'current_position':
			# Set current limit
			dxl_comm_result, dxl_error = SYS.packetHandler.write2ByteTxRx(SYS.portHandler, motor_id, ADDR.PRO_CURRENT_LIMIT, CURRENT_LIMIT)
			setup[1] += check4TxRxerror(motor_id, SYS, ADDR, dxl_comm_result, dxl_error)
			# Set desired current
			dxl_comm_result, dxl_error = SYS.packetHandler.write2ByteTxRx(SYS.portHandler, motor_id, ADDR.PRO_GOAL_CURRENT, DESIRED_CURRENT)
			setup[2] += check4TxRxerror(motor_id, SYS, ADDR, dxl_comm_result, dxl_error)
			# Set desired PWM
			dxl_comm_result, dxl_error = SYS.packetHandler.write2ByteTxRx(SYS.portHandler, motor_id, ADDR.PRO_GOAL_PWM, DESIRED_PWM)
			setup[3] += check4TxRxerror(motor_id, SYS, ADDR, dxl_comm_result, dxl_error)
		# reduce moving threshold
		dxl_comm_result, dxl_error = SYS.packetHandler.write4ByteTxRx(SYS.portHandler, motor_id, ADDR.PRO_MOVING_THRES, MOVING_THRESHOLD)
		setup[4] += check4TxRxerror(motor_id, SYS, ADDR, dxl_comm_result, dxl_error)
		# Enable Torque
		dxl_comm_result, dxl_error = SYS.packetHandler.write1ByteTxRx(SYS.portHandler, motor_id, ADDR.PRO_TORQUE_ENABLE, TORQUE_ENABLE)
		setup[5] += check4TxRxerror(motor_id, SYS, ADDR, dxl_comm_result, dxl_error)

		# Add parameter storage for present position
		dxl_addparam_result = SYS.groupBulkRead.addParam(motor_id, ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupBulkRead addparam failed to add PRESENT POSITION" % motor_id)
	print('All dynamixels have been successfully initialized')

	# set initial goal position to current position
	dxl_present_position = dxl_read_pos(DXL_IDS, SYS, ADDR, LEN)
	dxl_goal_position = dxl_present_position.copy().astype(int)

	return dxl_present_position, dxl_goal_position, SYS, ADDR, LEN

def shut_down(DXL_IDS, SYS, ADDR, LEN, askAction=True):
	# shut down all motors
	# returns nothing

	TORQUE_DISABLE = 0                 # Value for disabling the torque

	# Clear bulkread parameter storage
	SYS.groupBulkRead.clearParam()

	if  askAction==True:
		print('Disable torque? y/n')
		keypress = getch()
	else:
		keypress = 'y'

	if keypress == 'y':
		for motor_id in DXL_IDS:
			dxl_comm_result, dxl_error = SYS.packetHandler.write1ByteTxRx(SYS.portHandler, motor_id, ADDR.PRO_TORQUE_ENABLE, TORQUE_DISABLE)
			check4TxRxerror(motor_id, SYS, ADDR, dxl_comm_result, dxl_error)
		print("torques disabled")
	elif keypress == 'n':
		print('motors still on')

	# Close port
	SYS.portHandler.closePort()

def reboot(SYS, DXL_IDS):
	# reboots all motors
	# returns nothing

	for id in DXL_IDS:
		SYS.packetHandler.reboot(SYS.portHandler,id)

def calc_torque(DXL_IDS, ifprint, SYS, ADDR):
	# calculate the torque based on measured current
	# returns torque, dxl_present_current
	# ifprint: True/False, determines if function prints torque and current in the command line
	dxl_present_current = curr2amps(dxl_get_elec(DXL_IDS, SYS, ADDR.PRO_PRESENT_CURRENT, ADDR))
	torque = current2torque(dxl_present_current)

	if ifprint == True:
		measurements = ["    [ID:%02d] Torque: %3.2fNm, Current: %3.2fA" % (DXL_IDS[i], torque[i], dxl_present_current[i]) for i in range(len(DXL_IDS))]
		print(measurements)

	return torque, dxl_present_current

def dxl_read_pos(DXL_IDS, SYS, ADDR, LEN):
	# read value motor present position
	# returns position values as numpy array corresponding to motors in DXL_IDS

	# Syncread present values
	dxl_comm_result = SYS.groupBulkRead.txRxPacket()
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % SYS.packetHandler.getTxRxResult(dxl_comm_result))

	# Check if groupbulkread data is available
	for motor_id in DXL_IDS:
		dxl_getdata_result = SYS.groupBulkRead.isAvailable(motor_id, ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupBulkRead getdata failed" % motor_id)

	# Get value
	read_value = np.zeros(len(DXL_IDS))
	for i in range(len(DXL_IDS)):
		read_value[i] = SYS.groupBulkRead.getData(DXL_IDS[i], ADDR.PRO_PRESENT_POSITION, LEN.PRO_PRESENT_POSITION)

	return read_value

def dxl_get_elec(DXL_IDS, SYS, ADDR_read, ADDR):
	# read current and voltage values
	# current, voltage, and pwm are 2 bytes and require function below
	# special function to read two's complement because the way the current values are stored
	# ADDR_read: the specific control table address to be read
	present_value = [0]*len(DXL_IDS) # created as a list because it will store strings of binary numbers
	dxl_present_read = np.zeros(len(DXL_IDS))
	for i in range(len(DXL_IDS)):
		# Read present electricity
		present_value, dxl_comm_result, dxl_error = SYS.packetHandler.read2ByteTxRx(SYS.portHandler, DXL_IDS[i], ADDR_read)
		check4TxRxerror(DXL_IDS[i], SYS, ADDR, dxl_comm_result, dxl_error)
		# find two's complement
		if ADDR_read == ADDR.PRO_PRESENT_CURRENT:
			# convert number to binary
			bin_num = bin(present_value)[2:]
			while len(bin_num)<16:
				bin_num = '0'+bin_num
			if bin_num[0] == '0':
				twos_complement = int(bin_num, 2)
			else:
				twos_complement =  -1 * (int(''.join('1' if x == '0' else '0' for x in bin_num), 2) + 1)

			dxl_present_read[i] = twos_complement
			# dxl_present_current[motor_id-1] = min([dxl_present_current[motor_id-1], -abs(65535 - dxl_present_current[motor_id-1])])
		else:
			dxl_present_read[i] = present_value
	
	return dxl_present_read

def read_curr_volt(DXL_IDS, ifprint, SYS, ADDR):
	# print current, voltage, pwm present limit and goal values
	# returns cur_pres, pwm_pres, volt_pres
	# ifprint: True/False, determines if function prints current, pwm, volts in the command line

	cur_lim = np.around(curr2amps(dxl_get_elec(DXL_IDS, SYS, ADDR.PRO_CURRENT_LIMIT, ADDR)),2)
	cur_pres = np.around(curr2amps(dxl_get_elec(DXL_IDS, SYS, ADDR.PRO_PRESENT_CURRENT, ADDR)),2)
	cur_goal = np.around(curr2amps(dxl_get_elec(DXL_IDS, SYS, ADDR.PRO_GOAL_CURRENT, ADDR)),2)
	pwm_goal = np.around(pwm2pcnt(dxl_get_elec(DXL_IDS, SYS, ADDR.PRO_GOAL_PWM, ADDR)),1)
	pwm_pres = np.around(pwm2pcnt(dxl_get_elec(DXL_IDS, SYS, ADDR.PRO_PRESENT_PWM, ADDR)),1)
	volt_pres = np.around(volt2volts(dxl_get_elec(DXL_IDS, SYS, ADDR.PRO_PRESENT_INPUT_VOLTAGE, ADDR)),1)

	if ifprint:
		# print('cur_limt: ', cur_lim, 'A,  cur_psnt: ', cur_pres, 'A,  cur_goal: ', cur_goal, 'pwm_goal: ', pwm_goal, '%, pwm_psnt:', pwm_pres, '%, vlt_inpt: ', volt_pres, 'V')
		print('cur_psnt: ', cur_pres, 'A,  cur_goal: ', cur_goal, ', pwm_psnt:', pwm_pres, '%, vlt_inpt: ', volt_pres, 'V')

	return cur_pres, pwm_pres, volt_pres

def check4TxRxerror(DXL_ID, SYS, ADDR, dxl_comm_result, dxl_error):
	# used to check for error after calling SYS.packetHandler.write#ByteTxRx functions (for any byte length #)
	# dxl_comm_result, dxl_error outputted from SYS.packetHandler.write#ByteTxRx
	# see initialize() for example of use
	# returns 0 (failure) or 1 (success)

	if dxl_comm_result != COMM_SUCCESS:
		print("ID%d %s" % (DXL_ID, SYS.packetHandler.getTxRxResult(dxl_comm_result)))
		return 0
	elif dxl_error != 0:
		print("ID%d %s" % (DXL_ID, SYS.packetHandler.getRxPacketError(dxl_error)))
		dxl_error_message, dxl_comm_result, dxl_error = SYS.packetHandler.read1ByteTxRx(SYS.portHandler, DXL_ID, ADDR.PRO_HARDWARE_ERROR)
		return 0
	else:
		return 1