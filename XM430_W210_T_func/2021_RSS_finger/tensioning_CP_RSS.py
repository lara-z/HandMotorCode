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

import sys
sys.path.append('./')

from conversions import *

# Default setting
DXL_TOTAL                   = list(range(1,9)) #list(range(1,int(input("How many motors are there?"))+1))
DXL_TOTAL.remove(6)
joint_move                  = 2 #int(input("Which joint would you like to move (most proximal joint is 0)?  "))
DXLAG_ID                    = [2, 8, 3] #list(map(int,input("Enter the agonist motor numbers (separated by a space, proximal to distal):  ").strip().split()))[:len(DXLAG_ID)]
DXLAN_ID                    = [5, 7, 1] #list(map(int,input("Enter the antagonist motor numbers (separated by a space, proximal to distal):  ").strip().split()))[:len(DXLAG_ID)]
DXL_MOVE                    = [DXLAG_ID[joint_move], DXLAN_ID[joint_move]]
DXL_TENSIONED               = [x for y in zip(DXLAG_ID[:(1+joint_move)], DXLAN_ID[:(1+joint_move)]) for x in y]

BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM3'            # Check which port is being used on your controller

OPERATION_MODE              = 0x02
EXT_POSITION_CONTROL_MODE   = 4                 # Value for extended position control mode (operating mode)
CURRENT_BASED_POSITION      = 5                 # Value for current-based position control mode (operating mode)
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

JOINT_TORQUE                = [0.1]*6
# JOINT_TORQUE               = [0.4, 0.4, 0.3, 0.3, 0.2, 0.2]  # Nm, desired torque for each joint motor ordered following DXL_TENSIONED
TORQUE_MARGIN               = 0.06      # the maximum value above the desired torque that is still an acceptable torque value
DESIRED_CURRENT             = torque2current(0.4)            # desired current based on desired torque and converted to
CURRENT_LIMIT               = torque2current(3.0)            # desired current based on desired torque and converted to
DESIRED_PWM                 = int(100/0.113)     # desired PWM value in units of 0.113%
DXL_START_POSITION_VALUE    = 0                 # Dynamixel will rotate between this value
ROTATE_AMOUNT               = 100
DXL_MOVING_STATUS_THRESHOLD = 40                # Dynamixel moving status threshold
DXL_CURRENT_THRESHOLD       = torque2current(0.08)

index = 1

dxl_present_position, dxl_goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(DXL_TOTAL, DEVICENAME)


print("*********")
print('Please wait while the system tensions itself')

print('tensioning...')
dxl_present_position, dxl_goal_position = tension(dxl_present_position, dxl_goal_position, DXL_TOTAL, DXL_TENSIONED, JOINT_TORQUE, TORQUE_MARGIN, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
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
        dxl_goal_position[motor_id-1]  = (dxl_goal_position[motor_id-1] + direction[count]*ROTATE_AMOUNT)
        count += 1

    dxl_present_position = move(DXL_TOTAL, dxl_present_position, dxl_goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
    dxl_present_position, dxl_goal_position = tension(dxl_present_position, dxl_goal_position, DXL_TOTAL, DXL_TENSIONED, JOINT_TORQUE, TORQUE_MARGIN, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)
    error_status = False

shut_down(DXL_TOTAL, packetHandler, portHandler, groupBulkRead, ADDR, LEN)
