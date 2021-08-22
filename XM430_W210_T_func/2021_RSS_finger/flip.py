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

import sys
sys.path.append('./2021_RSS_finger')

from conversions import *

manual_control              = True

DEVICENAME                  = 'COM3'            # Check which port is being used on your controller

# GOAL_TORQUE                 = 0.45
JOINT_TORQUE                = [0.4, 0.4, 0.4, 0.4, 0.02, 0.02]  # Nm, desired torque for each joint motor ordered following DXL_TENSIONED
TORQUE_MARGIN               = 0.06      # the maximum value above the desired torque that is still an acceptable torque value
# DESIRED_CURRENT             = torque2current(GOAL_TORQUE)            # desired current based on desired torque and converted to
# CURRENT_LIMIT               = torque2current(3.0)            # desired current based on desired torque and converted to
# DESIRED_PWM                 = int(100/0.113)     # desired PWM value in units of 0.113%
VELOCITY_LIMIT              = int(3/0.229)
ROTATE_AMOUNT               = deg2pulse(30)
# DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold
# DXL_CURRENT_THRESHOLD       = torque2current(0.08)
#
# Default setting
DXL_TOTAL                   = list(range(1,9)) #list(range(1,int(input("How many motors are there?"))+1))
DXL_TOTAL.remove(6)
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
TRAJECTORY_ORDERED          = np.zeros([max(DXL_TOTAL),len(TRAJECTORY_J1)])
traj_ind                    = 0                             # index used to keep track of which timestep of the trajectory
traj_move                   = False                         # toggles between following trajectory or tuning tension/movement with ROTATE_AMOUNT

dxl_comm_result = [0]*max(DXL_TOTAL)
dxl_error = [0]*max(DXL_TOTAL)


dxl_present_position, dxl_goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(DXL_TOTAL, 'COM3')

# tension cables
joint_move    = int(input("Up to which joint would you like to tension (most proximal joint is 0)?  "))
DXL_TENSIONED = [x for y in zip(DXLAG_ID[:(1+joint_move)], DXLAN_ID[:(1+joint_move)]) for x in y]
dxl_present_position, dxl_goal_position = tension(dxl_present_position, dxl_goal_position, DXL_TOTAL, DXL_TENSIONED, JOINT_TORQUE, TORQUE_MARGIN, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)

dxl_present_position = read_pres_pos(dxl_present_position, DXL_TOTAL, packetHandler, portHandler, ADDR)
dxl_goal_position = dxl_present_position.copy()

# Set velocity profile value (so hand moves more slowly)
setup = 0
for motor_id in DXL_TOTAL:
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR.PRO_VELOCITY_PROFILE, VELOCITY_LIMIT)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        setup +=1

if setup == len(DXL_TOTAL):
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
                dxl_present_position = read_pres_pos(dxl_present_position, DXL_TOTAL, packetHandler, portHandler, ADDR)
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
            dxl_present_position, dxl_goal_position = tension(dxl_present_position, dxl_goal_position, DXL_TOTAL, DXL_TENSIONED, JOINT_TORQUE, TORQUE_MARGIN, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
        elif keypress == b'a':
            manual_control = False
            joint_move = 1
            direction = [0,0]*num_pairs
        else:
            direction = [0,0]*num_pairs
            print("Invalid key")

    if traj_move == True:
        # print('Completed %d %% of movement' % (100*(traj_ind+1)/len(TRAJECTORY_J1)))
        dxl_present_position = read_pres_pos(dxl_present_position, DXL_TOTAL, packetHandler, portHandler, ADDR)

        dxl_goal_position = [TRAJECTORY_ORDERED[count,traj_ind] for count in range(0,max(DXL_TOTAL))]
        traj_ind += 1
    else:        # calculate goal positions for each motor
        for motor_id in DXL_TOTAL:
            dxl_present_position = read_pres_pos(dxl_present_position, DXL_TOTAL, packetHandler, portHandler, ADDR)
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
        dxl_present_position = move(DXL_TOTAL, dxl_present_position, dxl_goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
        print('Moved')
        if traj_move == True:
            time.sleep(6)
    traj_move = False
    torque, dxl_present_current, error_status = calc_torque(DXL_TOTAL, True, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
    # tension()

shut_down(DXL_TOTAL, packetHandler, portHandler, groupBulkRead, ADDR, LEN)
