# will repeatedly move specified joint back and forth until motor is unplugged
# assumes cables are already tensioned properly and torque is enabled. Suggested to use either tensioning_antagonistic_CP.py or tensioning_antagonistic.py to tension cables before running this file
# change joint rotation amount in line 76 ROTATE_AMOUNT
# change pause between movements in line 95 WAIT_TIME

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

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM3'            # Check which port is being used on your controller

OPERATION_MODE              = 0x02
EXT_POSITION_CONTROL_MODE   = 4                 # Value for extended position control mode (operating mode)
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_START_POSITION_VALUE    = 0                 # Dynamixel will rotate between this value
ROTATE_AMOUNT_DEG           = 15                # degrees, will rotate this amount on either side of the starting position for specified joint
ROTATE_AMOUNT               = int(ROTATE_AMOUNT_DEG/0.088)       # convert from degrees to pulses
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

# Default setting
DXLAG_ID                    = list(map(int,input("Enter the agonist motor numbers from proximal to distal separated by only a space:  ").strip().split()))
DXLAN_ID                    = list(map(int,input("Enter the antagonist motor numbers from proximal to distal separated by only a space:  ").strip().split()))
DXL_TOTAL                   = DXLAG_ID + DXLAN_ID
num_move                    = int(input("Which joint do you want to move? (Joints numbered from 0 starting distally)  "))
DXL_MOVE                    = [DXLAG_ID[num_move], DXLAN_ID[num_move]]
DXL_DISTAL_ODD              = list(range(num_move+1,len(DXLAG_ID),2))
DXL_DISTAL_EVEN             = list(range(num_move+2,len(DXLAG_ID),2))
POS_MOTORS                  = [DXLAG_ID[i] for i in DXL_DISTAL_EVEN] + [DXLAN_ID[i] for i in DXL_DISTAL_ODD]    # this is used to know which motors to move in a positive direction to compensate cable length in distal joints as proximal joint moves
NEG_MOTORS                  = [DXLAG_ID[i] for i in DXL_DISTAL_ODD] + [DXLAN_ID[i] for i in DXL_DISTAL_EVEN]    # this is used to know which motors to move in a negative direction to compensate cable length in distal joints as proximal joint moves

r_m                         = 11.78 # mm, radius of pulley jointed to motor where cable is wound
r_j                         = 4.5 + 0.7/2 # mm, radius of the joint
compensation                = int(ROTATE_AMOUNT*(-0.4*r_m/r_j)) # 0.2, the amount that more distal motors must move to keep the finger straight

WAIT_TIME                   = 0.7 # seconds

index = 1

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

# # Set operating mode to extended position control mode in both motors
# count = [0,0]
# for motor_id in DXL_TOTAL:
#     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))
#     else:
#         count[0] +=1
#
# if count[0] == DXL_TOTAL[-1]:
#     print("Operating mode changed to extended position control mode forall dynamixels")
#
# # Enable Torque
# for motor_id in DXL_TOTAL:
#     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))
#     else:
#         count[1] +=1
#
# if count[1] == DXL_TOTAL[-1]:
#     print("All dynamixels has been successfully connected")

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

count = 0
for motor_id in DXL_TOTAL:
    dxl_present_position[motor_id-1], dx_comm_result[motor_id-1], dx_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_POSITION)
    if motor_id == DXLAG_ID[num_move]:
        dxl_goal_position[motor_id-1] = [dxl_present_position[motor_id-1] - ROTATE_AMOUNT, dxl_present_position[motor_id-1] + ROTATE_AMOUNT]
    elif motor_id == DXLAN_ID[num_move]:
        dxl_goal_position[motor_id-1] = [dxl_present_position[motor_id-1] + ROTATE_AMOUNT, dxl_present_position[motor_id-1] - ROTATE_AMOUNT]
    else:
        dxl_goal_position[motor_id-1] = [dxl_present_position[motor_id-1], dxl_present_position[motor_id-1]]
    count = count + 1

count = 0
for motor_id in POS_MOTORS:
    # compensate distally for proximal movement (i.e., maintain distal joint angles during proximal movement)
    dxl_goal_position[motor_id-1] = [dxl_goal_position[motor_id-1][1] + compensation, dxl_goal_position[motor_id-1][1] - compensation]

for motor_id in NEG_MOTORS:

    # compensate distally for proximal movement (i.e., maintain distal joint angles during proximal movement)
    dxl_goal_position[motor_id-1] = [dxl_goal_position[motor_id-1][1] - compensation, dxl_goal_position[motor_id-1][1] + compensation]

print("*********")

# print('Press "c" to change which motors you manipulate')
# print('Press ESC to exit')
# print('Press any other key to move the motors')

while 1:
    # uncomment code below to manually prompt each movement for debugging
    # keypress = msvcrt.getch()
    # if keypress == b'\x1b':
    #     break
    # elif keypress == b'c':
    #     num_pairs = int(input("How many agonist, antagonist pairs do you want to move simultaneously?  "))
    #     DXLAG_ID  = list(map(int,input("Enter the agonist motor numbers (separated by a space):  ").strip().split()))[:num_pairs]
    #     DXLAN_ID  = list(map(int,input("Enter the antagonist motor numbers (in the same order, separated by a space):  ").strip().split()))[:num_pairs]
    #     DXL_MOVE  = [x for y in zip(DXLAG_ID, DXLAN_ID) for x in y]
    #     print('Motors changed')
    # else:
    #     print('moving...')

    param_goal_position = [0]*max(DXL_TOTAL)
    count = 0
    for motor_id in DXL_TOTAL:
        # Allocate goal position value into byte array
        param_goal_position[count] = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[motor_id-1][index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[motor_id-1][index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[motor_id-1][index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[motor_id-1][index]))]

        # Add Dynamixel goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = groupBulkWrite.addParam(motor_id, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position[count])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % motor_id)
            quit()
        count = count + 1

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
        count = 0
        for motor_id in DXL_TOTAL:
            dxl_present_position[motor_id-1] = groupBulkRead.getData(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            count = count + 1

        # print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position : %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))
        count = 0
        for motor_id in DXL_TOTAL:
            # print(motor_id)
            if not (abs(dxl_goal_position[motor_id-1][index] - dxl_present_position[motor_id-1]) > DXL_MOVING_STATUS_THRESHOLD):
                moving = False
            count = count + 1

    if index == 1:
        index = 0
    elif index == 0:
        index = 1

    time.sleep(WAIT_TIME)

# Clear bulkread parameter storage
groupBulkRead.clearParam()

print('Disable torque? y/n')
keypress = msvcrt.getch()
if keypress == b'y':
    for motor_id in range(1,9):
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
