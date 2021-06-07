# can wind, unwind, move up, or move down any number of agonist/antagonist num_pairs
# wind, unwind, move up, or move down based on user input specified in the command prompt
# agonist/antagonist pairs specified in command prompt

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Bulk Read and Bulk Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
# Be sure that Dynamixel PRO properties are already set as %% ID : 1 and 2 / Baudnum : 1 (Baudrate : 57600)
#

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

# Control table address
ADDR_OPERATING_MODE         = 11
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_LED_RED            = 65
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_PRO_PRESENT_CURRENT    = 126

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
# DXL_TOTAL                   = list(range(1,9)) #list(range(1,int(input("How many motors are there?"))+1))
num_pairs                   = int(input("How many agonist, antagonist pairs do you want to move simultaneously?  "))
DXLAG_ID                    = list(map(int,input("Enter the agonist motor numbers (separated by a space):  ").strip().split()))[:num_pairs]
DXLAN_ID                    = list(map(int,input("Enter the antagonist motor numbers (in the same order, separated by a space):  ").strip().split()))[:num_pairs]
DXL_MOVE                    = [x for y in zip(DXLAG_ID, DXLAN_ID) for x in y]
DXL_TOTAL                   = DXL_MOVE + list(map(int,input("Any other motors to turn on?:  ").strip().split()))

BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM3'            # Check which port is being used on your controller

OPERATION_MODE              = 0x02
EXT_POSITION_CONTROL_MODE   = 4                 # Value for extended position control mode (operating mode)
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_START_POSITION_VALUE    = 0                 # Dynamixel will rotate between this value
ROTATE_AMOUNT               = 50
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 1

def curr2Amps(current_reading):
    return current_reading*2.69/1000
    return int(current_amps*1000/2.69)

def current2torque(current_amps):
    return float((current_amps - 0.1775)/0.95)

def calc_torque(ifprint):
    # calculate the torque based on measured current
    dxl_present_current = [0]*len(DXL_MOVE)
    torque = [0]*len(DXL_MOVE)
    count = 0
    for motor_id in DXL_MOVE:
        # Read present current
        dxl_present_current[count], dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_CURRENT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("[ID:%-2d]: %s" % (motor_id, packetHandler.getRxPacketError(dxl_error)))

        # convert current unit to amps
        dxl_present_current[count] = curr2Amps(dxl_present_current[count])
        torque[count] = current2torque(dxl_present_current[count])
        count = count+1
    if ifprint == True:
        measurements = ''
        for i in range(len(DXL_MOVE)):
            # measurements = measurements + ("    [ID:%02d] PresTorque:%3.2fNm, PresCurrent:%8.4fA" % (DXL_TENSIONED[i], torque[i], dxl_present_current[i]))
            measurements = measurements + ("    [ID:%02d] PresTorque:%3.2fNm" % (DXL_MOVE[i], torque[i]))
        print(measurements)
    return torque, dxl_present_current

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

# Set operating mode to extended position control mode in both motors
count = [0,0]
for motor_id in DXL_TOTAL:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        count[0] +=1

if count[0] == DXL_TOTAL[-1]:
    print("Operating mode changed to extended position control mode forall dynamixels")

# Enable Torque
for motor_id in DXL_TOTAL:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        count[1] +=1

if count[1] == DXL_TOTAL[-1]:
    print("All dynamixels has been successfully connected")

# Add parameter storage for present position
for motor_id in DXL_TOTAL:
    dxl_addparam_result = groupBulkRead.addParam(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkRead addparam failed" % motor_id)
        quit()

# set initial goal position to current position
dxl_present_position = [0]*max(DXL_TOTAL)
dx_comm_result = [0]*max(DXL_TOTAL)
dx_error = [0]*max(DXL_TOTAL)
dxl_goal_position =  [0]*max(DXL_TOTAL)
for motor_id in DXL_TOTAL:
    dxl_present_position[motor_id-1], dx_comm_result[motor_id-1], dx_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_POSITION)
    dxl_goal_position[motor_id-1] = [dxl_present_position[motor_id-1], dxl_present_position[motor_id-1]]

print("*********")

print('Press "u" to move the finger(s) up, "d" to move the finger(s) down')
print('Press "b" to pull all moving motors, "r" to release all moving motors')
print('Press "c" to change which motors you manipulate')
print('Press ESC to exit')

while 1:
    direction = 0
    keypress = msvcrt.getch()
    if keypress == b'\x1b':
        break
    elif keypress == b'u':
        direction = [1,-1]*num_pairs
    elif keypress == b'd':
        direction = [-1,1]*num_pairs
    elif keypress == b'b':
        direction = [1,1]*num_pairs
    elif keypress == b'r':
        direction = [-1,-1]*num_pairs
    elif keypress == b'c':
        num_pairs = int(input("How many agonist, antagonist pairs do you want to move simultaneously?  "))
        DXLAG_ID  = list(map(int,input("Enter the agonist motor numbers (separated by a space):  ").strip().split()))[:num_pairs]
        DXLAN_ID  = list(map(int,input("Enter the antagonist motor numbers (in the same order, separated by a space):  ").strip().split()))[:num_pairs]
        DXL_MOVE  = [x for y in zip(DXLAG_ID, DXLAN_ID) for x in y]
        direction = [0,0]*num_pairs
        print('Motors changed.')
    else:
        direction = [0,0]*num_pairs
        print("Invalid key")

    # write new goal position
    count = 0
    for motor_id in DXL_MOVE:
        new_goal  = (dxl_goal_position[motor_id-1][1] + direction[count]*ROTATE_AMOUNT)
        dxl_goal_position[motor_id-1] = [dxl_goal_position[motor_id-1][1], new_goal]
        count = count + 1

    # move the finger to a goal position that's defined in the code
    param_goal_position = [0]*max(DXL_TOTAL)
    for motor_id in DXL_TOTAL:
        # Allocate goal position value into byte array
        param_goal_position[motor_id-1] = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[motor_id-1][1])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[motor_id-1][1])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[motor_id-1][1])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[motor_id-1][1]))]

        # Add Dynamixel#1 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = groupBulkWrite.addParam(motor_id, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position[motor_id-1])
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
            dxl_getdata_result = groupBulkRead.isAvailable(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % motor_id)
                quit()

        # Get present position value
        for motor_id in DXL_TOTAL:
            dxl_present_position[motor_id-1] = groupBulkRead.getData(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position : %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))

        for motor_id in DXL_TOTAL:
            # print(motor_id)
            if not (abs(dxl_goal_position[motor_id-1][1] - dxl_present_position[motor_id-1]) > DXL_MOVING_STATUS_THRESHOLD):
                moving = False

    # measure torque after finishing moving
    torque, dxl_present_current = calc_torque(True)


# Clear bulkread parameter storage
groupBulkRead.clearParam()

print('Disable torque? y/n')
keypress = msvcrt.getch()
if keypress == b'y':
    for motor_id in DXL_TOTAL:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    print("torques disabled")
elif keypress == b'n':
    print('motors still on')

# Close port
portHandler.closePort()
