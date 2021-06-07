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
num_pairs                   = 1 #int(input("How many agonist, antagonist pairs do you want to move simultaneously?  "))
DXLAG_ID                    = [5] #list(map(int,input("Enter the agonist motor numbers (separated by a space):  ").strip().split()))[:num_pairs]
DXLAN_ID                    = [8] #list(map(int,input("Enter the antagonist motor numbers (in the same order, separated by a space):  ").strip().split()))[:num_pairs]
DXL_MOVE                    = [x for y in zip(DXLAG_ID, DXLAN_ID) for x in y]
DXL_MOVE_ORDER              = DXL_MOVE          # this order will be swapped in the code so that the one cable always loses tension before the other cable increases tension
DXL_TOTAL                   = DXL_MOVE #+ list(map(int,input("Any other motors to turn on? (Press enter if none)  ").strip().split()))

# DXLAG_ID                    = list(map(int,input("Enter the agonist motor numbers from proximal to distal separated by only a space:  ").strip().split()))
# DXLAN_ID                    = list(map(int,input("Enter the antagonist motor numbers from proximal to distal separated by only a space:  ").strip().split()))
# DXL_TOTAL                   = [x for y in zip(DXLAG_ID, DXLAN_ID) for x in y]
# num_pairs                   = len(DXLAG_ID)
# DXL_MOVE                    = [DXLAG_ID[num_move], DXLAN_ID[num_move]]
# DXL_DISTAL_ODD_0            = list(range(0+1,len(DXLAG_ID),2))
# DXL_DISTAL_EVEN_0           = list(range(0+2,len(DXLAG_ID),2))
# POS_MOTORS_0                = [DXLAG_ID[i] for i in DXL_DISTAL_EVEN_0] + [DXLAN_ID[i] for i in DXL_DISTAL_ODD_0]    # this is used to know which motors to move in a positive direction to compensate cable length in distal joints as proximal joint moves
# NEG_MOTORS_0                = [DXLAG_ID[i] for i in DXL_DISTAL_ODD_0] + [DXLAN_ID[i] for i in DXL_DISTAL_EVEN_0]    # this is used to know which motors to move in a negative direction to compensate cable length in distal joints as proximal joint moves
#
# DXL_DISTAL_ODD_1            = list(range(1+1,len(DXLAG_ID),2))
# DXL_DISTAL_EVEN_1           = list(range(1+2,len(DXLAG_ID),2))
# POS_MOTORS_1                = [DXLAG_ID[i] for i in DXL_DISTAL_EVEN_1] + [DXLAN_ID[i] for i in DXL_DISTAL_ODD_1]    # this is used to know which motors to move in a positive direction to compensate cable length in distal joints as proximal joint moves
# NEG_MOTORS_1                = [DXLAG_ID[i] for i in DXL_DISTAL_ODD_1] + [DXLAN_ID[i] for i in DXL_DISTAL_EVEN_1]    # this is used to know which motors to move in a negative direction to compensate cable length in distal joints as proximal joint moves


BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM3'            # Check which port is being used on your controller

OPERATION_MODE              = 0x02
EXT_POSITION_CONTROL_MODE   = 4                 # Value for extended position control mode (operating mode)
CURRENT_BASED_POSITION      = 5                 # Value for current-based position control mode (operating mode)
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
ROTATE_AMOUNT               = 50
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

def deg2pulse(deg):
    ratio = 2.4
    return int(deg*(ratio/0.088))

def curr2Amps(current_reading):
    return current_reading*2.69/1000
    return int(current_amps*1000/2.69)

def torque2current(torque):
    # convert based on tau = Kt*I
    current_amps = (torque*0.95 + 0.1775)
    # convert to current units used by motor
    return int(current_amps*1000/2.69)

def current2torque(current_amps):
    return float((current_amps - 0.1775)/0.95)

GOAL_TORQUE                 = 0.35
TORQUE_MARGIN               = 0.06      # the maximum value above the desired torque that is still an acceptable torque value
JOINT_TORQUE               = [0.4, 0.4, 0.3, 0.3, 0.1, 0.1]  # Nm, desired torque for each joint motor ordered following DXL_TENSIONED
TORQUE_MARGIN               = 0.06      # the maximum value above the desired torque that is still an acceptable torque value
DESIRED_CURRENT             = torque2current(GOAL_TORQUE)            # desired current based on desired torque and converted to
CURRENT_LIMIT               = torque2current(2.0)            # desired current based on desired torque and converted to
DESIRED_PWM                 = int(93/0.113)     # desired PWM value in units of 0.113%
ROTATE_AMOUNT               = 100
DXL_MOVING_STATUS_THRESHOLD = 40                # Dynamixel moving status threshold
DXL_CURRENT_THRESHOLD       = torque2current(0.08)

def calc_torque(ifprint):
    # calculate the torque based on measured current
    dxl_present_current = [0]*max(DXL_MOVE_ORDER)
    torque = [0]*max(DXL_MOVE_ORDER)
    count = 0
    for motor_id in DXL_MOVE_ORDER:
        # Read present current
        dxl_present_current[motor_id-1], dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_CURRENT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("[ID:%-2d]: %s" % (motor_id, packetHandler.getRxPacketError(dxl_error)))

        # convert current unit to amps
        dxl_present_current[motor_id-1] = curr2Amps(dxl_present_current[motor_id-1])
        if dxl_present_current[motor_id-1] > 15:
            torque[motor_id-1] = 0
        else:
            torque[motor_id-1] = current2torque(dxl_present_current[motor_id-1])
        count = count+1
    if ifprint == True:
        measurements = ''
        for motor_id in DXL_MOVE_ORDER:
            # measurements = measurements + ("    [ID:%02d] PresTorque:%3.2fNm, PresCurrent:%8.4fA" % (DXL_TENSIONED[i], torque[i], dxl_present_current[i]))
            measurements = measurements + ("    [ID:%02d] PresTorque:%3.2fNm" % (motor_id, torque[motor_id-1]))
        print(measurements)
    return torque, dxl_present_current

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

        torque, dxl_present_current = calc_torque(False)
        # print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position : %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))

        for motor_id in DXL_TOTAL:
            if not (abs(goal_position[motor_id-1][1] - dxl_present_position[motor_id-1]) > DXL_MOVING_STATUS_THRESHOLD):
                moving = False

def tension():
    print('Starting tensioning...')
    # keep moving the finger to the desired torque until the desired torque is achieved
    ROTATE_AMOUNT = 5
    torque, dxl_present_current = calc_torque(True)

    num_tensioned = 0;
    while num_tensioned < len(DXL_MOVE_ORDER):
        # write new goal position
        num_tensioned = 0;
        count = 0
        for motor_id in DXL_MOVE_ORDER:
            if torque[motor_id-1] > (GOAL_TORQUE + 0.3):
                new_goal  = (dxl_goal_position[motor_id-1][1] - 1*ROTATE_AMOUNT)
            elif torque[motor_id-1] > (GOAL_TORQUE + TORQUE_MARGIN):
                new_goal  = (dxl_goal_position[motor_id-1][1] - ROTATE_AMOUNT)
            elif torque[motor_id-1] < 0.05:
                new_goal  = (dxl_goal_position[motor_id-1][1] + 3*ROTATE_AMOUNT)
            elif torque[motor_id-1] < GOAL_TORQUE:
                new_goal  = (dxl_goal_position[motor_id-1][1] + ROTATE_AMOUNT)
            else:
                new_goal = dxl_goal_position[motor_id-1][1]
                num_tensioned += 1
            dxl_goal_position[motor_id-1] = [dxl_goal_position[motor_id-1][1], new_goal]
            count = count + 1

        move(dxl_goal_position)
        torque, dxl_present_current = calc_torque(False)
    print('Finished tensioning')

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
setup = [0]*4
# Set operating mode to current-based position control mode
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
dxl_present_position = [0]*max(DXL_TOTAL)
dx_comm_result = [0]*max(DXL_TOTAL)
dx_error = [0]*max(DXL_TOTAL)
dxl_goal_position =  [0]*max(DXL_TOTAL)
for motor_id in DXL_TOTAL:
    dxl_present_position[motor_id-1], dx_comm_result[motor_id-1], dx_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_POSITION)
    dxl_goal_position[motor_id-1] = [dxl_present_position[motor_id-1], dxl_present_position[motor_id-1]]

# define desired trajectory
TRAJECTORY_J1               = list(range(0,90,10))
TRAJECTORY_J1               = [deg2pulse(step) for step in TRAJECTORY_J1]
TRAJECTORY_J2               = [0]*len(TRAJECTORY_J1)    # should be 0-90 in opposite direction compared to joint 1
TRAJECTORY_J3               = [0]*len(TRAJECTORY_J1)    # should be 0-90 in same direction as joint 2 but with delay in start time
TRAJECTORY                  = [2*[TRAJECTORY_J1],2*[TRAJECTORY_J2],2*[[TRAJECTORY_J3]]     # goal positions time series for joints proximal to distal
traj_ind                    = 0                             # index used to keep track of which timestep of the trajectory
traj_move                   = False                         # toggles between following trajectory or tuning tension/movement with ROTATE_AMOUNT

# calculate the trajectory to compensation for motion in more proximal joints
# direction = [-1,1]*num_pairs
# count = 0
# for motor_id in DXL_TOTAL:
#     new_goal  = [(basepoint[motor_id-1] + direction[count]*TRAJECTORY[motor_id-1][traj_point]) for traj_point in TRAJECTORY[motor_id-1]]
#     if motor_id in POS_MOTORS_0:
#         new_goal = [new_goal[traj_point] + compensation_0 for traj_point in new_goal]
#     if motor_id in NEG_MOTORS_0:
#         new_goal = [new_goal[traj_point] - compensation_0 for traj_point in new_goal]
#     if motor_id in POS_MOTORS_1:
#         new_goal = [new_goal[traj_point] - compensation_1 for traj_point in new_goal]
#     if motor_id in NEG_MOTORS_1:
#         new_goal = [new_goal[traj_point] + compensation_1 for traj_point in new_goal]
#     print(new_goal)
#     dxl_goal_position[motor_id-1] = new_goal
#     count = count + 1

print("*********")

print('Press "u" to move the finger(s) up, "d" to move the finger(s) down')
print('Press "b" to pull all moving motors, "r" to release all moving motors')
print('Press "c" to change which motors you manipulate')
print('Press "t" to tension selected motors to goal torque')
print('Press "n" to go to the next move')
print('Press ESC to exit')

while 1:
    DXL_MOVE_ORDER = DXL_MOVE
    keypress = msvcrt.getch()
    if keypress == b'\x1b':
        break
    elif keypress == b'u':
        direction = [1,-1]*num_pairs
        DXL_MOVE_ORDER = DXL_MOVE[::-1]
    elif keypress == b'd':
        direction = [1,-1]*num_pairs
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
    elif keypress == b'n':
        direction = [1,-1]*num_pairs
        DXL_MOVE_ORDER = DXL_MOVE[::-1]
        joint_move = 0 # int(input("Which joint are you moving?  "))
        traj_move = True
        if traj_ind == 0:
            basepoint = dxl_present_position
        if traj_ind >= len(TRAJECTORY):
            print('Invalid. End of movement has been reached.')
            traj_ind = len(TRAJECTORY) - 1
        else:
            print('Moving by %d' % TRAJECTORY[joint_move][traj_ind])
    elif keypress == b't':
        tension()
        direction = [0,0]*num_pairs
    else:
        direction = [0,0]*num_pairs
        print("Invalid key")

    # write new goal position
    count = 0
    for motor_id in DXL_MOVE_ORDER:
        if traj_move == True:
            new_goal  = (basepoint[motor_id-1] + direction[count]*TRAJECTORY[joint_move][traj_ind])
            traj_move = False
            traj_ind += 1
        else:
            new_goal  = (dxl_goal_position[motor_id-1][1] + direction[count]*ROTATE_AMOUNT)
        dxl_goal_position[motor_id-1] = [dxl_goal_position[motor_id-1][1], new_goal]
        count += 1

    move(dxl_goal_position)
    torque, dxl_present_current = calc_torque(False)
    time.sleep(0.5)
    tension()

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
