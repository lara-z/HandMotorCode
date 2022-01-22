#!/usr/bin/env python
# -*- coding: utf-8 -*-

# this was copied and pasted from an official dynamixel example, except functions have been changed to use dynamixel_sdk library instead of dynamixel_functions library

import os, sys
from dynamixel_sdk import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(sys.stdin.fileno())
    def getch():
        return sys.stdin.read(1)

os.sys.path.append('../dynamixel_functions_py')             # Path setting
print('')

# import dynamixel_functions as dynamixel                     # Uses DYNAMIXEL SDK library

# Control table address
ADDR_PRO_TORQUE_ENABLE       = 64                          # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116
ADDR_PRO_PRESENT_POSITION    = 132

# Protocol version
PROTOCOL_VERSION            = 2.0                             # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 10                             # Dynamixel ID: 1
BAUDRATE                    = 1000000
DEVICENAME                  = '/dev/ttyUSB0'                # Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150                       # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150                        # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

index = 0
dxl_comm_result = COMM_TX_FAIL                              # Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

dxl_error = 0                                               # Dynamixel error
dxl_present_position = 0                                    # Present position

# Open port
if portHandler.openPort():
    print("Succeeded to open the port!", end="\n", file=sys.stdout, flush=False)
else:
    print("Failed to open the port!")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    print("Press any key to terminate...")
    getch()
    quit()


# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(ESC_ASCII_VALUE):
        break

    # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # while 1:
    #     # Read present position
    #     dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))

    #     # print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))

    #     if abs(dxl_goal_position[index] - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD:
    #         break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0


# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
