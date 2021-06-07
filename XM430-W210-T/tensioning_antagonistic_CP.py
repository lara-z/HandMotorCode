# can wind, unwind, move up, or move down any number of agonist/antagonist num_pairs
# wind, unwind, move up, or move down based on user input specified in the command prompt
# agonist/antagonist pairs specified in command prompt
# will automatically tension cable to desired torque (line 105, DESRIED_TORQUE)

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
ADDR_PRO_CURRENT_LIMIT      = 38
ADDR_PRO_GOAL_CURRENT       = 102
ADDR_PRO_PRESENT_CURRENT    = 126
ADDR_PRO_GOAL_PWM           = 100
ADDR_PRO_HARDWARE_ERROR     = 70

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_CURRENT             = 2
LEN_PRO_PWM                 = 2

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
# DXL_TOTAL                   = list(range(1,9)) #list(range(1,int(input("How many motors are there?"))+1))
joint_move                  = int(input("Which joint would you like to move (most proximal joint is 0)?  "))
DXLAG_ID                    = list(map(int,input("Enter the agonist motor numbers (separated by a space, proximal to distal):  ").strip().split()))[:len(DXLAG_ID)]
DXLAN_ID                    = list(map(int,input("Enter the antagonist motor numbers (separated by a space, proximal to distal):  ").strip().split()))[:len(DXLAG_ID)]
DXL_MOVE                    = [DXLAG_ID[joint_move], DXLAN_ID[joint_move]]
DXL_TENSIONED               = [x for y in zip(DXLAG_ID[:(1+joint_move)], DXLAN_ID[:(1+joint_move)]) for x in y]
DXL_TOTAL                   = DXLAG_ID + DXLAN_ID

BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM3'            # Check which port is being used on your controller

OPERATION_MODE              = 0x02
EXT_POSITION_CONTROL_MODE   = 4                 # Value for extended position control mode (operating mode)
CURRENT_BASED_POSITION      = 5                 # Value for current-based position control mode (operating mode)
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

def curr2Amps(current_reading):
    return current_reading*2.69/1000

def torque2current(torque):
    # convert based on tau = Kt*I
    current_amps = (torque*0.95 + 0.1775)
    # convert to current units used by motor
    return int(current_amps*1000/2.69)

def current2torque(current_amps):
    return float((current_amps - 0.1775)/0.95)

DESIRED_TORQUE              = 0.4              # desired torque in Nm
TORQUE_MARGIN               = 0.06      # the maximum value above the desired torque that is still an acceptable torque value
DESIRED_CURRENT             = torque2current(0.9)      # desired current based on desired torque and converted to
CURRENT_LIMIT               = torque2current(2.0)      # desired current based on desired torque and converted to
DESIRED_PWM                 = int(93/0.113)     # desired PWM value in units of 0.113%
DXL_START_POSITION_VALUE    = 0                 # Dynamixel will rotate between this value
ROTATE_AMOUNT               = 100
DXL_MOVING_STATUS_THRESHOLD = 40                # Dynamixel moving status threshold
DXL_CURRENT_THRESHOLD       = torque2current(0.08)

index = 1

def calc_torque(ifprint):
    # calculate the torque based on measured current
    dxl_present_current = [0]*len(DXL_TENSIONED)
    torque = [0]*len(DXL_TENSIONED)
    count = 0
    for motor_id in DXL_TENSIONED:
        # Read present current
        dxl_present_current[count], dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_CURRENT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("[ID:%-2d]: %s" % (motor_id, packetHandler.getRxPacketError(dxl_error)))
            dxl_error_message, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, motor_id, ADDR_PRO_HARDWARE_ERROR)
            # print(dxl_error_message)
            # print(dxl_comm_result)
            # print(dxl_error)

        # convert current unit to amps
        dxl_present_current[count] = curr2Amps(dxl_present_current[count])

        if dxl_present_current[count] > 150:
            # when the cables are slack, the current is really large, but toque is actually zero so set to zero
            torque[count] = 0
        else:
            torque[count] =current2torque(dxl_present_current[count])
        count = count+1
    if ifprint == True:
        measurements = ''
        for i in range(len(DXL_TENSIONED)):
            # measurements = measurements + ("    [ID:%02d] PresTorque:%3.2fNm, PresCurrent:%8.4fA" % (DXL_TENSIONED[i], torque[i], dxl_present_current[i]))
            measurements = measurements + ("    [ID:%02d] Torque: %3.2fNm" % (DXL_TENSIONED[i], torque[i]))
        print(measurements)
    return torque, dxl_present_current

def setCurrent():
    # move the finger to a goal current that's defined in the code
    param_goal_current = [0]*max(DXL_TOTAL)
    dxl_present_current = [0]*max(DXL_TOTAL)
    for motor_id in DXL_TOTAL:
        # Allocate goal current value into byte array
        param_goal_current[motor_id-1] = [DXL_LOBYTE(DXL_LOWORD(DESIRED_CURRENT)), DXL_HIBYTE(DXL_LOWORD(DESIRED_CURRENT)), DXL_LOBYTE(DXL_HIWORD(DESIRED_CURRENT)), DXL_HIBYTE(DXL_HIWORD(DESIRED_CURRENT))]

        # Add Dynamixel#1 goal current value to the Bulkwrite parameter storage
        dxl_addparam_result = groupBulkWrite.addParam(motor_id, ADDR_PRO_GOAL_CURRENT, LEN_PRO_CURRENT, param_goal_current[motor_id-1])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % motor_id)
            quit()

    # Bulkwrite goal current?
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()

    moving = True
    while moving:
        # Bulkread present current?
        dxl_comm_result = groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        # print('Bulkread present positions')

        # Check if groupbulkread data is available
        for motor_id in DXL_TOTAL:
            dxl_getdata_result = groupBulkRead.isAvailable(motor_id, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_CURRENT)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % motor_id)
                quit()

        # Get present position value
        for motor_id in DXL_TOTAL:
            dxl_present_current[motor_id-1] = groupBulkRead.getData(motor_id, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_CURRENT)

        torque, dxl_present_current = calc_torque(True)
        # print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position : %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))

        for motor_id in DXL_TOTAL:
                if not (abs(DESIRED_CURRENT - dxl_present_current[motor_id-1]) > DXL_CURRENT_THRESHOLD):
                moving = False

def move(goal_position):
    # move the finger to a goal position that's defined in the code
    param_goal_position = [0]*max(DXL_TOTAL)
    for motor_id in DXL_TOTAL:
        # Allocate goal position value into byte array
        param_goal_position[motor_id-1] = [DXL_LOBYTE(DXL_LOWORD(goal_position[motor_id-1][1])), DXL_HIBYTE(DXL_LOWORD(goal_position[motor_id-1][1])), DXL_LOBYTE(DXL_HIWORD(goal_position[motor_id-1][1])), DXL_HIBYTE(DXL_HIWORD(goal_position[motor_id-1][1]))]

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

        torque, dxl_present_current = calc_torque(True)
        # print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position : %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))

        for motor_id in DXL_TOTAL:
            if not (abs(goal_position[motor_id-1][1] - dxl_present_position[motor_id-1]) > DXL_MOVING_STATUS_THRESHOLD):
                moving = False

def tension():
    # keep moving the finger to the desired torque until the desired torque is achieved
    ROTATE_AMOUNT = 5
    torque, dxl_present_current = calc_torque(True)

    while any(x < DESIRED_TORQUE for x in torque) | any(x > (DESIRED_TORQUE + TORQUE_MARGIN) for x in torque):
        # write new goal position
        count = 0
        for motor_id in DXL_TENSIONED:
            if torque[count] > (DESIRED_TORQUE + TORQUE_MARGIN):
                new_goal  = (dxl_goal_position[motor_id-1][1] - ROTATE_AMOUNT)
            elif torque[count] < DESIRED_TORQUE:
                new_goal  = (dxl_goal_position[motor_id-1][1] + ROTATE_AMOUNT)
            else:
                new_goal = dxl_goal_position[motor_id-1][1]
            dxl_goal_position[motor_id-1] = [dxl_goal_position[motor_id-1][1], new_goal]
            count = count + 1

        move(dxl_goal_position)
        torque, dxl_present_current = calc_torque(False)

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

setup = [0]*4
# Set operating mode to extended position control mode
for motor_id in DXL_TOTAL:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        setup[0] +=1

if setup[0] == DXL_TOTAL[-1]:
    print("All dynamixel operating modes have been successfully changed")

# Set current limit
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_PRO_CURRENT_LIMIT, CURRENT_LIMIT)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    setup[1] +=1

if setup[1] == DXL_TOTAL[-1]:
    print("All dynamixel current limit has been successfully changed")

# Set desired current
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_PRO_GOAL_CURRENT, DESIRED_CURRENT)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    setup[1] +=1

if setup[1] == DXL_TOTAL[-1]:
    print("All dynamixel goal current values have been successfully changed")

# Set desired PWM
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_PRO_GOAL_PWM, DESIRED_PWM)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    setup[2] +=1

if setup[2] == DXL_TOTAL[-1]:
    print("All dynamixel goal PWM values have been successfully changed")

# Enable Torque
for motor_id in DXL_TOTAL:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        setup[3] +=1

if setup[3] == DXL_TOTAL[-1]:
    print("All dynamixels now have torque enabled")

# Add parameter storage for present position
for motor_id in DXL_TOTAL:
    dxl_addparam_result = groupBulkRead.addParam(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkRead addparam failed" % motor_id)
        quit()

# set initial goal position to current position
dxl_present_position = [0]*len(DXL_TOTAL)
dx_comm_result = [0]*len(DXL_TOTAL)
dx_error = [0]*len(DXL_TOTAL)
dxl_goal_position =  [0]*len(DXL_TOTAL)
for motor_id in DXL_TOTAL:
    dxl_present_position[motor_id-1], dx_comm_result[motor_id-1], dx_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_POSITION)
    dxl_goal_position[motor_id-1] = [dxl_present_position[motor_id-1], dxl_present_position[motor_id-1]]

print("*********")
print('Please wait while the system tensions itself')

print('tensioning...')
tension()
print('tensioned')

print('At any time...')
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
        direction = [1,-1]*int(len(DXL_MOVE)/2)
        print('u')
    elif keypress == b'd':
        direction = [-1,1]*int(len(DXL_MOVE)/2)
        print('d')
    elif keypress == b'b':
        direction = [1,1]*int(len(DXL_MOVE)/2)
    elif keypress == b'r':
        direction = [-1,-1]*int(len(DXL_MOVE)/2)
    elif keypress == b'c':
        joint_move    = int(input("Which joint would you like to move (most proximal joint is 0)?  "))
        DXL_MOVE      = [DXLAG_ID[joint_move], DXLAN_ID[joint_move]]
        DXL_TENSIONED = [x for y in zip(DXLAG_ID[:(1+joint_move)], DXLAN_ID[:(1+joint_move)]) for x in y]
        direction = [0,0]*int(len(DXL_MOVE)/2)
        print('Moving joint changed.')
    else:
        direction = [0,0]*int(len(DXL_MOVE)/2)
        print("Invalid key")

    # write new goal position
    count = 0
    for motor_id in DXL_MOVE:
        new_goal  = (dxl_goal_position[motor_id-1][1] + direction[count]*ROTATE_AMOUNT)
        dxl_goal_position[motor_id-1] = [dxl_goal_position[motor_id-1][1], new_goal]
        count = count + 1

    move(dxl_goal_position)
    tension()


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
