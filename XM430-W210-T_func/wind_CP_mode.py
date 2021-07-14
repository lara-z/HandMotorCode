# winds a single motor
# motor id specified by user as command prompt input

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

# import dynamixel_functions as dynamixel                     # Uses DYNAMIXEL SDK library
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import keyboard

# Control table address
ADDR_OPERATING_MODE         = 11
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_CURRENT_LIMIT          = 38
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
DXL_ID                      = int(input("What is the ID of this dynamixel motor?  "))
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
    return float((abs(current_amps) - 0.1775)/0.95)

DESIRED_CURRENT             = torque2current(0.6)               # desired current value in units of 2.69mA
DESIRED_PWM                 = int(80/0.113)   # desired PWM value in units of 0.113%
DXL_START_POSITION_VALUE    = 0                 # Dynamixel will rotate between this value
ROTATE_AMOUNT               = 20
DXL_MOVING_STATUS_THRESHOLD = 40                # Dynamixel moving status threshold
DXL_CURRENT_THRESHOLD       = torque2current(0.005)
print(curr2Amps(DXL_CURRENT_THRESHOLD))

index = 1

def calc_torque(ifprint):
    # calculate the torque based on measured current
    # Read goal current
    dxl_goal_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("[ID:%-2d]: %s" % (DXL_ID, packetHandler.getRxPacketError(dxl_error)))
        dxl_error_message, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_HARDWARE_ERROR)

    # Read present current
    dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_CURRENT)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("[ID:%-2d]: %s" % (DXL_ID, packetHandler.getRxPacketError(dxl_error)))
        dxl_error_message, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_HARDWARE_ERROR)

    # convert current unit to amps

    torque = current2torque(curr2Amps(dxl_present_current))

    if ifprint == True:
        print("[ID:%02d] Torque: %3.3fNm, PresCurrent: %3.3fA, GoalCurrent: %3.3fA" % (DXL_ID, torque, curr2Amps(dxl_present_current), curr2Amps(DESIRED_CURRENT)))
        # print("Raw Current: %8.2f, Current (Amps): %3.3fA, Raw Goal Current: %8.2f, Goal Current (Amps): %3.3fA" % (dxl_present_current, curr2Amps(dxl_present_current),dxl_goal_current, curr2Amps(dxl_goal_current)))


    return torque, dxl_present_current

def setCurrent():
    # move the finger to a goal current that's defined in the code

    # Write goal current
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, DESIRED_CURRENT)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print('start current adjustment')
    while 1:
        calc_torque(True)
        # Read present position
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        if not abs(DESIRED_CURRENT - dxl_present_current) > DXL_CURRENT_THRESHOLD:
            break

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)
# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

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

# Set operating mode to extended position control mode
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode changed to extended position control mode.")

# Set desired current
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, DESIRED_CURRENT)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Desired current set")

# Set desired PWM
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, DESIRED_PWM)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Desired PWM set")

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

print("*********")

setCurrent()

dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
dxl_goal_position = [dxl_present_position, dxl_present_position]

print('Press "w" to wind the motor, "u" to unwind the motor, or ESC to exit')

calc_torque(True)

while 1:
    if getch() == chr(0x1b):
        break
    elif keyboard.is_pressed('w'):
        direction = 1
    elif keyboard.is_pressed('u'):
        direction = -1
    else:
        direction = 0

    start_pos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)

    # write new goal position
    new_goal  = (dxl_goal_position[1] + direction*ROTATE_AMOUNT)
    dxl_goal_position = [dxl_goal_position[1], new_goal]

    # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print('start move')
    while 1:
        calc_torque(True)
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break

    end_pos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)

    print('Finished move. Goal movement amount: %d. Moved %d.' % (ROTATE_AMOUNT, end_pos - start_pos))
    calc_torque(True)

# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
print("torque disabled")

# Close port
portHandler.closePort()
