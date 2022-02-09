from .conversions import *
from .utils_motor import *
from .moving import *
from .utils_sensor import *
from .misc import *

# COMMON VARIABLES SHARED ACROSS MOTOR FUNCTIONS
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
	format: class of integers
	what: dynamixel control table addresses as integers, see initialize() for all values
	established: in utils_motor/initialize()
"""
"""
	LEN
	format: class of integers
	what: integer byte lengths corresponding to dynamixel control table addresses, see initialize() for all values
	established: in utils_motor/initialize()
"""
"""
	dxl_info
	format: class of lists
	contains: ids, joint_num, joint_torque
	ids: list of motor ids in any order
	joint_num: list of joint numbers (1 is most proximal, number increases as it becomes distal) for each motor
		entries ordered according to the motor order in dxl_info.ids and size should match dxl_info.ids
	joint_torque: list of desired joint torques for each joint, first is most proximal joint and last is most distal joint
		for multiple fingers, it is assumed that all 1st joints are tensioned to the same value, all 2nd joints to another value, etc.
	established: in user code
"""
"""
	dxl_pres_position
	format: numpy array of the length of the number of motors (length of dxl_info.ids)
	what: present positions of all motors, entries ordered according to the motor order in dxl_info.ids
	established: output of dxl_read_pos()
"""
"""
	goal_position, dxl_goal_position
	format: numpy array of the length of the number of motors (length of dxl_info.ids)
	what: absolute goal position for each motor, entries ordered according to the motor order in dxl_info.ids
	established: in user code
"""
"""
	new_rel_goal
	format: numpy array of integers containing as many entries as there are joints in one finger
	what: the new relative positions of the motors (how much the motors should move from their current position)
			from the most proximal to the most distal joints in the finger
	units: motor encoder units
	established: in user code
"""
"""
	torque_des
	format: integer
	what: desired torque
	established: in user code, likely as an entry in dxl_info.joint_torque
"""
"""
	dxl_info
	format: class of lists
	contains: ids, joint_num, joint_torque
	ids: list of motor ids in any order
	joint_num: list of joint numbers (1 is most proximal, number increases as it becomes distal) for each motor
		entries ordered according to the motor order in dxl_info.ids and size should match dxl_info.ids
	joint_torque: list of desired joint torques for each joint, first is most proximal joint and last is most distal joint
		for multiple fingers, it is assumed that all 1st joints are tensioned to the same value, all 2nd joints to another value, etc.
	established: in user code
"""
"""
	dxl_pres_position
	format: numpy array of the length of the number of motors (length of dxl_info.ids)
	what: present positions of all motors, entries ordered according to the motor order in dxl_info.ids
	established: output of dxl_read_pos()
"""
"""
	goal_position, dxl_goal_position
	format: numpy array of the length of the number of motors (length of dxl_info.ids)
	what: absolute goal position for each motor, entries ordered according to the motor order in dxl_info.ids
	established: in user code
"""
"""
	new_rel_goal
	format: numpy array of integers containing as many entries as there are joints in one finger
	what: the new relative positions of the motors (how much the motors should move from their current position)
			from the most proximal to the most distal joints in the finger
	units: motor encoder units
	established: in user code
"""
"""
	torque_des
	format: integer
	what: desired torque
	established: in user code, likely as an entry in dxl_info.joint_torque
"""