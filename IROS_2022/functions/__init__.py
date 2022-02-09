from .conversions import *
from .utils_motor import *
from .moving import *
from .utils_sensor import *
from .misc import *

# COMMON VARIABLES SHARED ACROSS FUNCTIONS
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