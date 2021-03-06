import os
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import numpy as np
from .utils_motor import *
from .conversions import *
import time

def move_abs(DXL_IDS, goal_position, SYS, ADDR, LEN, wait_finish=True, print_currvolt=False, limits=False):
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
		# print('Waiting to finish')
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

def move_rel(dxl_info, new_rel_goal, dxl_goal_position, SYS, ADDR, LEN):
	# move a pair of agonist and antagonist cables in a joint
	# cables for more distal joints also get moved so that more distal part of finger maintains position and appears unaffected by proximal joint movement

	dxl_goal_position = new_goal_pos_comp(new_rel_goal, dxl_info, dxl_goal_position)
	dxl_present_position, _, _, _ = move_abs(dxl_info.ids, dxl_goal_position, SYS, ADDR, LEN)

	return dxl_present_position, dxl_goal_position

def new_goal_pos_comp(new_rel_goal, dxl_info, dxl_pres_position):
	# calculate new goal positions
	# compensate more distal joints for movement in proximal joints so that distal joints don't change relative angle from proximal joint
	# rotate amount: a list of positions relative to current position. numpy array is 1xnumber-of-joints, and first element in list corresponds to most proximal joint.
	dxl_goal_position = dxl_pres_position.copy()

	for this_joint_num in range(1,len(new_rel_goal)+1):
		rotate_amount = new_rel_goal[this_joint_num-1]
		for i in range(0,len(dxl_info.ids)):
			if abs(dxl_info.joint_num[i]) >= this_joint_num:
				if (abs(dxl_info.joint_num[i]) - this_joint_num) % 2 == 1:
					comp_dir = -1
				else:
					comp_dir = 1
				dxl_goal_position[i] += np.sign(dxl_info.joint_num[i])*comp_dir*rotate_amount

	return dxl_goal_position

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
	torque, _ = calc_torque(DXL_IDS, False, SYS, ADDR)

	# print('Starting to tension selected cables')
	num_tensioned = 0
	count = 0
	while (num_tensioned < len(DXL_IDS)):
		num_tensioned = 0 # zero tensioning so same motor can't be counted twice

		# write new goal positions
		for motor_entry in range(len(DXL_IDS)):
			if round(torque[motor_entry],2) > (torque_des + 0.2):
				new_goal = - 6*ROTATE_AMOUNT
				# print('torque way too high')
			elif round(torque[motor_entry],2) > (torque_des + TORQUE_MARGIN):
				new_goal = - ROTATE_AMOUNT
				count += 1
				# print('torque too high')
			elif round(torque[motor_entry],2) < 0.08:
				new_goal = 20*ROTATE_AMOUNT
			elif round(torque[motor_entry],2) < torque_des:
				new_goal = ROTATE_AMOUNT
				count += 1
			else:
				new_goal = 0
				num_tensioned += 1
			dxl_goal_position[motor_entry] = dxl_goal_position[motor_entry] + new_goal
		
		# move motors and measure new torque
		dxl_present_position, cur_pres, _, _ = move_abs(DXL_IDS, dxl_goal_position, SYS, ADDR, LEN, False)
		torque = current2torque(cur_pres)
		# if count % 1 == 0:
		# 	print('Torque: ', np.round(torque, decimals=2))
		if count > 10*len(DXL_IDS):
			print('Motors are taking too long to achieve desired tension')
			break
	# print('Finished tensioning selected cables')
	# print('Finished tensioning selected cables. Torque: ', np.round(torque, decimals=2))
	
	return dxl_present_position, dxl_goal_position

def tension_seq(dxl_info, SYS, ADDR, LEN, which='both'):
	# sequentially joints of each finger from most proximal to most distal
	# which specifies if dynamixels tensioned should be only agonistic (+1), only antagonistic (-1), or both

	# print('Starting to tension cables')
	for joint in np.unique(np.absolute(dxl_info.joint_num)):
		# find the motors ids corresponding to this joint number
		if which == 'both':
			joint_dxl = np.array(dxl_info.ids)[np.where(np.absolute(dxl_info.joint_num) == joint)[0]]
		elif which == 1 or which == -1:
			joint_dxl = np.array(dxl_info.ids)[np.where(dxl_info.joint_num == which*joint)[0]]
		else:
			print('Invalid ''which''. Valid entries are: ''both'', 1, -1')
			quit()
		torque_des = dxl_info.joint_torque[joint-1]
		# tension all cables corresponding to this joint number
		tension(joint_dxl, torque_des, SYS, ADDR, LEN)

	dxl_present_position = dxl_read_pos(dxl_info.ids, SYS, ADDR, LEN)
	print('Finished tensioning all joints')
	print('')
	return dxl_present_position