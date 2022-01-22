import os
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import numpy as np
from .utils_motor import *
from .conversions import *

def move(DXL_IDS, goal_position, SYS, ADDR, LEN, wait_finish=True, print_currvolt=False, limits=False):
    # move the finger to a goal position that's defined in the code

	DXL_MOVING_STATUS_THRESHOLD = deg2pulse(5)                # Dynamixel moving status threshold
	goal_position = goal_position.astype(int)

	param_goal_position = [0]*len(DXL_IDS)

	for i in range(len(DXL_IDS)):
		# Allocate goal position value into byte array
		param_goal_position[i] = [DXL_LOBYTE(DXL_LOWORD(goal_position[i])), DXL_HIBYTE(DXL_LOWORD(goal_position[i])), DXL_LOBYTE(DXL_HIWORD(goal_position[i])), DXL_HIBYTE(DXL_HIWORD(goal_position[i]))]

        # Add Dynamixel goal position value to the Syncwrite parameter storage
		dxl_addparam_result = SYS.groupSyncWrite.addParam(DXL_IDS[i], param_goal_position[i])
		if dxl_addparam_result != True:
			print("[ID:%03d] groupSyncWrite addparam failed" % DXL_IDS[i])
			shut_down(DXL_IDS, SYS, ADDR, LEN, askAction=False)

    # Syncwrite goal position
	dxl_comm_result = SYS.groupSyncWrite.txPacket()
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % SYS.packetHandler.getTxRxResult(dxl_comm_result))

    # Clear bulkwrite parameter storage
	SYS.groupSyncWrite.clearParam()
	
	if wait_finish:
		print('Waiting to finish')
		time.sleep(0.02)
		moving = len(DXL_IDS)
		while moving != 0:
			moving = 0
			for motor_id in DXL_IDS:
				present_value, dxl_comm_result, dxl_error = SYS.packetHandler.read1ByteTxRx(SYS.portHandler, motor_id, ADDR.PRO_MOVING)
				check4TxRxerror(DXL_IDS[i], SYS, ADDR, dxl_comm_result, dxl_error)
				moving += present_value
			# print(moving)

	# get present position, current, pwm, voltage
	dxl_present_position = dxl_read_pos(DXL_IDS, SYS, ADDR, LEN)
	cur_pres, pwm_pres, volt_pres = read_curr_volt(DXL_IDS, print_currvolt, SYS, ADDR)

	return dxl_present_position, cur_pres, pwm_pres, volt_pres

def tension(DXL_IDS, torque_des, SYS, ADDR, LEN):
    # used in open loop tendon drive
	# keep moving the finger to the desired torque until the desired torque is achieved

	# distal joints may require lower tension
	# joint_torque is torque for each numbereth joint (1 is proximal, higher number is distal). If the highest dof finger has 3 joints, joint_torque is a list of 3 numbers
	# DXL_JOINT_NUM is the numbereth joint from the 1st most proximal joint that each motor corresponds to
	# DXL_JOINT_NUM is the same shape as DXL_IDS where the first entry corresponds to the first motor, etc.
	# DXL_JOINT_NUM is negative if agonist, positive if antagonist

	#! modify function so that pairs of motors tension sequentially from proximal to distal joint
	dxl_present_position = dxl_read_pos(DXL_IDS, SYS, ADDR, LEN)
	dxl_goal_position = dxl_present_position.copy()

	ROTATE_AMOUNT = deg2pulse(0.5) # incremental amount motors rotate for tensioning
	TORQUE_MARGIN = 0.03
	torque, _ = calc_torque(DXL_IDS, True, SYS, ADDR)

	print('Starting to tension selected cables')
	num_tensioned = 0
	count = 0
	while (num_tensioned < len(DXL_IDS)):
		num_tensioned = 0 # zero tensioning so same motor can't be counted twice

		# write new goal positions
		for motor_entry in range(len(DXL_IDS)):
			if round(torque[motor_entry],2) > (torque_des + 0.2):
				new_goal = 6*ROTATE_AMOUNT
				# print('torque way too high')
			elif round(torque[motor_entry],2) > (torque_des + TORQUE_MARGIN):
				new_goal = ROTATE_AMOUNT
				count += 1
				# print('torque too high')
			elif round(torque[motor_entry],2) < 0.08:
				new_goal = - 20*ROTATE_AMOUNT
			elif round(torque[motor_entry],2) < torque_des:
				new_goal = - ROTATE_AMOUNT
				count += 1
			else:
				new_goal = 0
				num_tensioned += 1
			dxl_goal_position[motor_entry] = dxl_goal_position[motor_entry] + new_goal
		
		# move motors and measure new torque
		dxl_present_position, cur_pres, _, _ = move(DXL_IDS, dxl_goal_position, SYS, ADDR, LEN, False)
		torque = current2torque(cur_pres)
		# if count % 1 == 0:
		# 	print('Torque: ', np.round(torque, decimals=2))
		if count > 10*len(DXL_IDS):
			print('Motors are taking too long to achieve desired tension')
			break
	print('Finished tensioning selected cables. Torque: ', np.round(torque, decimals=2))
	
	return dxl_present_position, dxl_goal_position

def move_pair(joint_num, rotate_amount, DXL_IDS, DXL_JOINT_NUM, joint_ratios, JOINT_TORQUE, SYS, ADDR, LEN):
	# move a pair of agonist and antagonist cables in a joint
	# cables for more distal joints also get moved so that more distal part of finger maintains position and appears unaffected by proximal joint movement

	dxl_present_position = dxl_read_pos(DXL_IDS, SYS, ADDR, LEN)
	dxl_goal_position = dxl_present_position.copy()

	# calculate new goal positions
	for i in range(0,len(DXL_IDS)):
		if abs(DXL_JOINT_NUM[i]) >= joint_num:
			if (abs(DXL_JOINT_NUM[i]) - joint_num) % 2 == 1:
				comp_dir = -1
			else:
				comp_dir = 1
			dxl_goal_position[i] += np.sign(DXL_JOINT_NUM[i])*comp_dir*rotate_amount

	# finger
	dxl_present_position = move(DXL_IDS, dxl_goal_position, SYS, ADDR, LEN)

	# retension finger
	# dxl_present_position= tension_seq(DXL_IDS, DXL_JOINT_NUM, JOINT_TORQUE, SYS, ADDR, LEN)

	return dxl_present_position

def tension_seq(DXL_IDS, DXL_JOINT_NUM, JOINT_TORQUE, SYS, ADDR, LEN):
	# sequentially joints of each finger from most proximal to most distal

	for joint in np.unique(np.absolute(DXL_JOINT_NUM)):
		# find the motors ids corresponding to this joint number
		joint_dxl = np.array(DXL_IDS)[np.where(np.absolute(DXL_JOINT_NUM) == joint)[0]]
		torque_des = JOINT_TORQUE[joint-1]
		# tension all cables corresponding to this joint number
		tension(joint_dxl, torque_des, SYS, ADDR, LEN)

	dxl_present_position = dxl_read_pos(DXL_IDS, SYS, ADDR, LEN)
	print('Finished tensioning all joints')
	print('')
	return dxl_present_position