# makes finger follow desired trajectory path for RSS paper

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

# import os
# 
# if os.name == 'nt':
#     import msvcrt
#     def getch():
#         return msvcrt.getch().decode()
# else:
#     import sys, tty, termios
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     def getch():
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch
# 
# os.sys.path.append('../dynamixel_functions_py')             # Path setting
# 
# from dynamixel_sdk import *                    # Uses Dynamixel SDK library
# import keyboard
# from conversions import *
# import numpy as np
# 
# manual_control              = True
# 
# # Control table address
# ADDR_OPERATING_MODE         = 11
# ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
# ADDR_PRO_LED_RED            = 65
# ADDR_PRO_GOAL_POSITION      = 116
# ADDR_PRO_PRESENT_POSITION   = 132
# ADDR_PRO_CURRENT_LIMIT      = 38
# ADDR_PRO_GOAL_CURRENT       = 102
# ADDR_PRO_PRESENT_CURRENT    = 126
# ADDR_PRO_GOAL_PWM           = 100
# ADDR_PRO_HARDWARE_ERROR     = 70
# ADDR_PRO_DRIVE_MODE         = 10
# ADDR_PRO_VELOCITY_PROFILE   = 112
# 
# # Data Byte Length
# LEN_PRO_GOAL_POSITION       = 4
# LEN_PRO_PRESENT_POSITION    = 4
# LEN_PRO_CURRENT             = 2
# LEN_PRO_PWM                 = 2
# 
# # Protocol version
# PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
# 
# BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
# DEVICENAME                  = 'COM3'            # Check which port is being used on your controller
# 
# OPERATION_MODE              = 0x02
# EXT_POSITION_CONTROL_MODE   = 4                 # Value for extended position control mode (operating mode)
# CURRENT_BASED_POSITION      = 5                 # Value for current-based position control mode (operating mode)
# VELOCITY_BASED_PROFILE      = 0
# TORQUE_ENABLE               = 1                 # Value for enabling the torque
# TORQUE_DISABLE              = 0                 # Value for disabling the torque
# DXL_MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold
# 
# GOAL_TORQUE                 = 0.45
# JOINT_TORQUE                = [0.4, 0.4, 0.4, 0.4, 0.02, 0.02]  # Nm, desired torque for each joint motor ordered following DXL_TENSIONED
# TORQUE_MARGIN               = 0.06      # the maximum value above the desired torque that is still an acceptable torque value
# DESIRED_CURRENT             = torque2current(GOAL_TORQUE)            # desired current based on desired torque and converted to
# CURRENT_LIMIT               = torque2current(3.0)            # desired current based on desired torque and converted to
# DESIRED_PWM                 = int(100/0.113)     # desired PWM value in units of 0.113%
# VELOCITY_LIMIT              = int(3/0.229)
# ROTATE_AMOUNT               = deg2pulse(30)
# DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold
# DXL_CURRENT_THRESHOLD       = torque2current(0.08)

# Default setting

DXL_TOTAL                   = list(range(1,9)) #list(range(1,int(input("How many motors are there?"))+1))
DXLAG_ID                    = list([2, 8, 1]) #list(map(int,input("Enter the agonist motor numbers from proximal to distal separated by only a space:  ").strip().split()))
DXLAN_ID                    = list([5, 7, 3]) #list(map(int,input("Enter the antagonist motor numbers from proximal to distal separated by only a space:  ").strip().split()))
num_pairs                   = len(DXLAG_ID)
DXL_MOVE                    = [x for y in zip(DXLAG_ID, DXLAN_ID) for x in y]
DXL_DISTAL_ODD_0            = list(range(0+1,len(DXLAG_ID),2))
DXL_DISTAL_EVEN_0           = list(range(0+2,len(DXLAG_ID),2))
POS_MOTORS_0                = [DXLAG_ID[i] for i in DXL_DISTAL_EVEN_0] + [DXLAN_ID[i] for i in DXL_DISTAL_ODD_0]    # this is used to know which motors to move in a positive direction to compensate cable length in distal joints as proximal joint moves
NEG_MOTORS_0                = [DXLAG_ID[i] for i in DXL_DISTAL_ODD_0] + [DXLAN_ID[i] for i in DXL_DISTAL_EVEN_0]    # this is used to know which motors to move in a negative direction to compensate cable length in distal joints as proximal joint moves

DXL_DISTAL_ODD_1            = list(range(1+1,len(DXLAG_ID),2))
DXL_DISTAL_EVEN_1           = list(range(1+2,len(DXLAG_ID),2))
POS_MOTORS_1                = [DXLAG_ID[i] for i in DXL_DISTAL_EVEN_1] + [DXLAN_ID[i] for i in DXL_DISTAL_ODD_1]    # this is used to know which motors to move in a positive direction to compensate cable length in distal joints as proximal joint moves
NEG_MOTORS_1                = [DXLAG_ID[i] for i in DXL_DISTAL_ODD_1] + [DXLAN_ID[i] for i in DXL_DISTAL_EVEN_1]    # this is used to know which motors to move in a negative direction to compensate cable length in distal joints as proximal joint moves

joint_move                  = 0
DXL_TENSIONED               = [x for y in zip(DXLAG_ID[:(1+joint_move)], DXLAN_ID[:(1+joint_move)]) for x in y]

r_m                         = 11.78 # mm, radius of pulley jointed to motor where cable is wound
r_j                         = 4.5 + 0.7/2 # mm, radius of the joint
compensation_0              = -0.475*r_m/r_j # the amount that more distal motors must move to keep the finger straight about joint 0
compensation_1              = -0.2*r_m/r_j # the amount that more distal motors must move to keep the finger straight about joint 1

# define desired trajectory
num_steps                   = 6
deg2pulse_vec               = np.vectorize(deg2pulse)
# TRAJECTORY_J1               = np.append(np.linspace(0,90,num_steps),5) # list(range(0,90,10))
TRAJECTORY_J1               = np.array([0,90,0])
TRAJECTORY_J1               = deg2pulse_vec(TRAJECTORY_J1) # [deg2pulse(step) for step in TRAJECTORY_J1]
# TRAJECTORY_J2               = [0]*len(TRAJECTORY_J1)    # should be 0-90 in opposite direction compared to joint 1
TRAJECTORY_J2               = np.array([0,-90,0])
# TRAJECTORY_J2               = np.append(np.linspace(0,-90,num_steps),5) # list(range(0,90,10))
TRAJECTORY_J2               = deg2pulse_vec(TRAJECTORY_J2) # [deg2pulse(step) for step in TRAJECTORY_J2]
# TRAJECTORY_J3               = [0]*len(TRAJECTORY_J1)    # should be 0-90 in same direction as joint 2 but with delay in start time
TRAJECTORY_J3               = np.array([0,90,0])
# TRAJECTORY_J3               = np.append(np.linspace(0,90,num_steps),0) # list(range(0,90,10))
TRAJECTORY_J3               = deg2pulse_vec(TRAJECTORY_J3) # [deg2pulse(step) for step in TRAJECTORY_J2]
TRAJECTORY                  = np.matrix([TRAJECTORY_J1,TRAJECTORY_J1,TRAJECTORY_J2, TRAJECTORY_J2,TRAJECTORY_J3, TRAJECTORY_J3])     # goal positions time series for joints proximal to distal
TRAJECTORY_ORDERED          = np.zeros([len(DXL_TOTAL),len(TRAJECTORY_J1)])
traj_ind                    = 0                             # index used to keep track of which timestep of the trajectory
traj_move                   = False                         # toggles between following trajectory or tuning tension/movement with ROTATE_AMOUNT

# def calc_torque(ifprint):
#     # calculate the torque based on measured current
#     error_status = False
#     dxl_present_current = [0]*max(DXL_TOTAL)
#     torque = [0]*max(DXL_TOTAL)
#     for motor_id in DXL_TOTAL:
#         # Read present current
#         dxl_present_current[motor_id-1], dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_CURRENT)
#         if dxl_comm_result != COMM_SUCCESS:
#             print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#         elif dxl_error != 0:
#             print("[ID:%-2d]: %s" % (motor_id, packetHandler.getRxPacketError(dxl_error)))
#             dxl_error_message, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, motor_id, ADDR_PRO_HARDWARE_ERROR)
#             error_status = True
# 
#         # convert current unit to amps
#         dxl_present_current[motor_id-1] = curr2Amps(dxl_present_current[motor_id-1])
# 
#         if dxl_present_current[motor_id-1] > 150:
#             # when the cables are slack, the current is really large, but toque is actually zero so set to zero
#             torque[motor_id-1] = 0
#         else:
#             torque[motor_id-1] = current2torque(dxl_present_current[motor_id-1])
#     if ifprint == True:
#         measurements = ''
#         for motor_id in DXL_TENSIONED:
#             # measurements = measurements + ("    [ID:%02d] PresTorque:%3.2fNm, PresCurrent:%8.4fA" % (DXL_TENSIONED[i], torque[i], dxl_present_current[i]))
#             measurements = measurements + ("    [ID:%02d] Torque: %3.2fNm" % (motor_id, torque[motor_id-1]))
#         print(measurements)
#     return torque, dxl_present_current, error_status

# def move(goal_position):
#     # move the finger to a goal position that's defined in the code
# 
#     param_goal_position = [0]*max(DXL_TOTAL)
#     for motor_id in DXL_TOTAL:
#         # Allocate goal position value into byte array
#         param_goal_position[motor_id-1] = [DXL_LOBYTE(DXL_LOWORD(goal_position[motor_id-1])), DXL_HIBYTE(DXL_LOWORD(goal_position[motor_id-1])), DXL_LOBYTE(DXL_HIWORD(goal_position[motor_id-1])), DXL_HIBYTE(DXL_HIWORD(goal_position[motor_id-1]))]
# 
#         # Add Dynamixel goal position value to the Bulkwrite parameter storage
#         dxl_addparam_result = groupBulkWrite.addParam(motor_id, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position[motor_id-1])
#         if dxl_addparam_result != True:
#             print("[ID:%03d] groupBulkWrite addparam failed" % motor_id)
#             quit()
# 
#     # Bulkwrite goal position
#     dxl_comm_result = groupBulkWrite.txPacket()
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# 
#     # Clear bulkwrite parameter storage
#     groupBulkWrite.clearParam()
# 
#     moving = True
#     while moving:
#         # Bulkread present positions
#         dxl_comm_result = groupBulkRead.txRxPacket()
#         if dxl_comm_result != COMM_SUCCESS:
#             print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#         # print('Bulkread present positions')
# 
#         # Check if groupbulkread data is available
#         for motor_id in DXL_TOTAL:
#             dxl_getdata_result = groupBulkRead.isAvailable(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
#             if dxl_getdata_result != True:
#                 print("[ID:%03d] groupBulkRead getdata failed" % motor_id)
#                 quit()
# 
#         # Get present position value
#         for motor_id in DXL_TOTAL:
#             dxl_present_position[motor_id-1] = groupBulkRead.getData(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
# 
#         torque, dxl_present_current, error_status = calc_torque(False)
#         if error_status == True:
#             moving = False
#         # print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position : %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))
# 
#         for motor_id in DXL_TOTAL:
#             if not (abs(goal_position[motor_id-1] - dxl_present_position[motor_id-1]) > DXL_MOVING_STATUS_THRESHOLD):
#                 moving = False
#     return dxl_present_position
# 
# def tension():
#     # keep moving the finger to the desired torque until the desired torque is achieved
#     # why does it keep shaking back and forth if I change goal position to present position???
#     for motor_id in DXL_TOTAL:
#         dxl_present_position[motor_id-1], dx_comm_result[motor_id-1], dx_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_POSITION)
# 
#     ROTATE_AMOUNT = 5
#     torque, dxl_present_current, error_status = calc_torque(True)
# 
#     num_tensioned = 0;
#     print('Start tensioning')
#     while (num_tensioned < len(DXL_TENSIONED)) & (error_status == False):
#         # write new goal position
#         num_tensioned = 0;
#         count = 0
#         for motor_id in DXL_TENSIONED:
#             if round(torque[motor_id-1],2) > (JOINT_TORQUE[count] + 0.2):
#                 new_goal = - 3*ROTATE_AMOUNT
#             elif round(torque[motor_id-1],2) > (JOINT_TORQUE[count] + TORQUE_MARGIN):
#                 new_goal = - ROTATE_AMOUNT
#             elif round(torque[motor_id-1],2) < 0.05:
#                 new_goal =  5*ROTATE_AMOUNT
#             elif round(torque[motor_id-1],2) < JOINT_TORQUE[count]:
#                 new_goal = ROTATE_AMOUNT
#             else:
#                 new_goal = 0
#                 num_tensioned += 1
#             dxl_goal_position[motor_id-1] = dxl_goal_position[motor_id-1] + new_goal
#             count = count + 1
# 
#         move(dxl_goal_position)
#         torque, dxl_present_current, error_status = calc_torque(False)
#     if error_status == True:
#         print('Error. Stopped tensioning.')
#     else:
#         print('Finished tensioning')

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
# portHandler = PortHandler(DEVICENAME)
# 
# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
# packetHandler = PacketHandler(PROTOCOL_VERSION)
# 
# Initialize GroupBulkWrite instance
# groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
# 
# Initialize GroupBulkRead instace for Present Position
# groupBulkRead = GroupBulkRead(portHandler, packetHandler)
# 
# Open port
# if portHandler.openPort():
#     print("Succeeded to open the port")
# else:
#     print("Failed to open the port")
#     print("Press any key to terminate...")
#     getch()
#     quit()
# 
# Set port baudrate
# if portHandler.setBaudRate(BAUDRATE):
#     print("Succeeded to change the baudrate")
# else:
#     print("Failed to change the baudrate")
#     print("Press any key to terminate...")
#     getch()
#     quit()
# 
# set up motors
# setup = [0]*7
# Set operating mode to current-based position control mode
# for motor_id in DXL_TOTAL:
#     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))
#     else:
#         setup[0] +=1
# 
# if setup[0] == len(DXL_TOTAL):
#     print("All dynamixel operating modes have been successfully changed")
# 
# Set current limit
# for motor_id in DXL_TOTAL:
#     dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_PRO_CURRENT_LIMIT, CURRENT_LIMIT)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))
#     else:
#         setup[1] +=1
# 
# if setup[1] == len(DXL_TOTAL):
#     print("All dynamixel current limit has been successfully changed")
# 
# Set desired current
# for motor_id in DXL_TOTAL:
#     dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_PRO_GOAL_CURRENT, DESIRED_CURRENT)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))
#     else:
#         setup[2] +=1
# 
# if setup[2] == len(DXL_TOTAL):
#     print("All dynamixel goal current values have been successfully changed")
# 
# Set desired PWM
# for motor_id in DXL_TOTAL:
#     dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_PRO_GOAL_PWM, DESIRED_PWM)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))
#     else:
#         setup[3] +=1
# 
# if setup[3] == len(DXL_TOTAL):
#     print("All dynamixel goal PWM values have been successfully changed")
# 
# Set drive mode
# for motor_id in DXL_TOTAL:
#     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_PRO_DRIVE_MODE, VELOCITY_BASED_PROFILE)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))
#     else:
#         setup[4] +=1
# 
# if setup[4] == len(DXL_TOTAL):
#     print("All dynamixel drive modes have been successfully changed to velocity-based")
# 
# Enable Torque
# for motor_id in DXL_TOTAL:
#     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))
#     else:
#         setup[5] +=1
# 
# if setup[5] == len(DXL_TOTAL):
#     print("All dynamixels now have torque enabled")
# 
# Add parameter storage for present position
# for motor_id in DXL_TOTAL:
#     dxl_addparam_result = groupBulkRead.addParam(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
#     if dxl_addparam_result != True:
#         print("[ID:%03d] groupBulkRead addparam failed" % motor_id)
#         quit()
# 
# set initial goal position to current position
# dxl_present_position = [0]*max(DXL_TOTAL)
# dx_comm_result = [0]*max(DXL_TOTAL)
# dx_error = [0]*max(DXL_TOTAL)
# dxl_goal_position =  [0]*max(DXL_TOTAL)
# for motor_id in DXL_TOTAL:
#     dxl_present_position[motor_id-1], dx_comm_result[motor_id-1], dx_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_POSITION)
# dxl_goal_position = dxl_present_position.copy()

packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize('COM3', DXL_TOTAL)

# tension cables
joint_move    = int(input("Up to which joint would you like to tension (most proximal joint is 0)?  "))
DXL_TENSIONED = [x for y in zip(DXLAG_ID[:(1+joint_move)], DXLAN_ID[:(1+joint_move)]) for x in y]
tension()

for motor_id in DXL_TOTAL:
    dxl_present_position[motor_id-1], dx_comm_result[motor_id-1], dx_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_POSITION)
dxl_goal_position = dxl_present_position.copy()

# Set velocity profile value (so hand moves more slowly)
for motor_id in DXL_TOTAL:
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_PRO_VELOCITY_PROFILE, VELOCITY_LIMIT)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        setup[6] +=1

if setup[6] == len(DXL_TOTAL):
    print("All dynamixels now have velocity profile speed set")

print("*********")

running = True
while 1:
    DXL_MOVE_ORDER = DXL_MOVE
    if manual_control == False:
        traj_move = True
        if traj_ind == 0:

            # calculate trajectory using current motor position now that cables are num_tensioned
            direction = [-1,1]*num_pairs
            count = 0
            basepoint = dxl_present_position
            for motor_id in DXL_MOVE:
                TRAJECTORY[count,:]  = (basepoint[motor_id - 1] + direction[count]*TRAJECTORY[count,:]).astype(int)
                if motor_id == 7:
                    TRAJECTORY[count,:] += (compensation_0*TRAJECTORY_J1).astype(int)
                elif motor_id == 8:
                    TRAJECTORY[count,:] += - (compensation_0*TRAJECTORY_J1).astype(int)
                elif motor_id == 1:
                    TRAJECTORY[count,:] += (- compensation_0*TRAJECTORY_J1 + compensation_1*TRAJECTORY_J2).astype(int)
                elif motor_id == 3:
                    TRAJECTORY[count,:] += (compensation_0*TRAJECTORY_J1 - compensation_1*TRAJECTORY_J2).astype(int)
                TRAJECTORY_ORDERED[motor_id-1,:] = TRAJECTORY[count,:]
                count = count + 1
            TRAJECTORY_ORDERED = TRAJECTORY_ORDERED.astype(int)
            print('Will start moving')
            time.sleep(1.5)
        if traj_ind >= len(TRAJECTORY_J1):
            traj_ind = 0
            traj_move = False
            direction = [0,0]*num_pairs
            manual_control = True
    else:
        print('Press "u" to move the finger(s) up, "d" to move the finger(s) down')
        print('Press "b" to pull all moving motors, "r" to release all moving motors')
        print('Press "c" to change which motors you manipulate')
        print('Press "t" to tension selected motors to goal torque')
        print('Press "n" to go to the next move')
        print('Press "a" to switch to automatic motion')
        print('Press ESC to exit')

        keypress = msvcrt.getch()
        if keypress == b'\x1b':
            break
        elif keypress == b'u':
            DXL_MOVE_ORDER = DXL_MOVE[::-1]
            direction = [0,0]*num_pairs
            direction[2*joint_move] = 1
            direction[2*joint_move+1] = -1
        elif keypress == b'd':
            direction = [0,0]*num_pairs
            direction[2*joint_move] = 1
            direction[2*joint_move+1] = -1
        elif keypress == b'b':
            direction = [0,0]*num_pairs
            direction[2*joint_move] = 1
            direction[2*joint_move+1] = 1
        elif keypress == b'r':
            direction = [0,0]*num_pairs
            direction[2*joint_move] = -1
            direction[2*joint_move+1] = -1
        elif keypress == b'c':
            joint_move = int(input("Which joint would you like to move (most proximal joint is 0)?  "))
            DXL_TENSIONED = [x for y in zip(DXLAG_ID[:(1+joint_move)], DXLAN_ID[:(1+joint_move)]) for x in y]
            direction = [0,0]*num_pairs
            direction[2*joint_move] = 1
            direction[2*joint_move+1] = 1
        elif keypress == b'n':
            direction = [-1,1]*num_pairs
            # DXL_MOVE_ORDER = DXL_MOVE[::-1]
            traj_move = True
            if traj_ind == 0:
                # calculate the trajectory to compensation for motion in more proximal joints
                for motor_id in DXL_TOTAL:
                    dxl_present_position[motor_id-1], dx_comm_result[motor_id-1], dx_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_POSITION)
                basepoint = dxl_present_position.copy()

                count = 0
                direction = [-1,1]*num_pairs
                for motor_id in DXL_MOVE:
                    TRAJECTORY[count,:]  = (basepoint[motor_id - 1] + direction[count]*TRAJECTORY[count,:]).astype(int)
                    if motor_id == 7:
                        TRAJECTORY[count,:] += (compensation_0*TRAJECTORY_J1).astype(int)
                    elif motor_id == 8:
                        TRAJECTORY[count,:] += - (compensation_0*TRAJECTORY_J1).astype(int)
                    elif motor_id == 1:
                        TRAJECTORY[count,:] += (- compensation_0*TRAJECTORY_J1 + compensation_1*TRAJECTORY_J2).astype(int)
                    elif motor_id == 3:
                        TRAJECTORY[count,:] += (compensation_0*TRAJECTORY_J1 - compensation_1*TRAJECTORY_J2).astype(int)
                    TRAJECTORY_ORDERED[motor_id-1,:] = TRAJECTORY[count,:]
                    count = count + 1
                TRAJECTORY_ORDERED = TRAJECTORY_ORDERED.astype(int)
            if traj_ind >= len(TRAJECTORY_J1):
                print('Invalid. End of movement has been reached.')
                traj_ind = len(TRAJECTORY_J1) - 1
        elif keypress == b't':
            joint_move    = int(input("Which joint would you like to move (most proximal joint is 0)?  "))
            DXL_TENSIONED = [x for y in zip(DXLAG_ID[:(1+joint_move)], DXLAN_ID[:(1+joint_move)]) for x in y]
            direction = [0,0]*num_pairs
            tension()
        elif keypress == b'a':
            manual_control = False
            joint_move = 1
            direction = [0,0]*num_pairs
        else:
            direction = [0,0]*num_pairs
            print("Invalid key")

    if traj_move == True:
        # print('Completed %d %% of movement' % (100*(traj_ind+1)/len(TRAJECTORY_J1)))
        for motor_id in DXL_TOTAL:
            dxl_present_position[motor_id-1], dx_comm_result[motor_id-1], dx_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_POSITION)

        dxl_goal_position = [TRAJECTORY_ORDERED[count,traj_ind] for count in range(0,len(DXL_TOTAL))]
        traj_ind += 1
    else:        # calculate goal positions for each motor
        for motor_id in DXL_TOTAL:
            dxl_present_position[motor_id-1], dx_comm_result[motor_id-1], dx_error[motor_id-1] = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_POSITION)
        dxl_goal_position = dxl_present_position.copy()
        count = 0
        for motor_id in DXL_MOVE_ORDER:
            dxl_goal_position[motor_id-1] = dxl_goal_position[motor_id-1] + direction[count]*ROTATE_AMOUNT
            # if motor_id == 7:
            #     dxl_goal_position[motor_id-1] += int(compensation_0*ROTATE_AMOUNT)
            # elif motor_id == 8:
            #     dxl_goal_position[motor_id-1] += - int(compensation_0*ROTATE_AMOUNT)
            # elif motor_id == 1:
            #     dxl_goal_position[motor_id-1] += - int(compensation_1*ROTATE_AMOUNT) + int(compensation_1*ROTATE_AMOUNT)
            # elif motor_id == 3:
            #     dxl_goal_position[motor_id-1] += + int(compensation_1*ROTATE_AMOUNT) - int(compensation_1*ROTATE_AMOUNT)
            count += 1

    if dxl_goal_position != dxl_present_position:
        move(DXL_TOTAL, dxl_goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
        print('Moved')
        if traj_move == True:
            time.sleep(6)
    traj_move = False
    torque, dxl_present_current, error_status = calc_torque(DXL_TOTAL, True, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
    # tension()

shut_down(packetHandler, portHandler, groupBulkRead, ADDR, LEN)

# Clear bulkread parameter storage
# groupBulkRead.clearParam()
# 
# print('Disable torque? y/n')
# keypress = msvcrt.getch()
# if keypress == b'y':
#     for motor_id in DXL_TOTAL:
#         dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
#         if dxl_comm_result != COMM_SUCCESS:
#             print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#         elif dxl_error != 0:
#             print("%s" % packetHandler.getRxPacketError(dxl_error))
#     print("torques disabled")
# elif keypress == b'n':
#     print('motors still on')
# 
# Close port
# portHandler.closePort()
