# winds and unwinds a single motor
# motor id specified by user as command prompt input
# useful when initially assembling manipulator and need to wind cables individually

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


from conversions import *

com_num = '/dev/ttyUSB0'

# comment out one of the lines below to change operating mode
operating_mode = 'current_position' # current-based position control
# operating_mode = 'extended_position'

# specify current goal and limit if using current-based position control
current_limit = amps2curr(3.0) # input the number of amps and it will convert to motor units
current_goal = amps2curr(1.0) # input the goal number of amps

motor_id = [int(input('What is the ID of the motor you would like to control?  '))]
ROTATE_AMOUNT = deg2pulse(10) # the increment in degrees (converted to motor pulse units) you would like the motor to move

if operating_mode == 'extended_position':
    dxl_present_position, dxl_goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_id, com_num, operating_mode)
elif operating_mode == 'current_position':
    dxl_present_position, dxl_goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN = initialize(motor_id, com_num, operating_mode, current_goal, current_limit)
else:
    print('invalid operating mode')
print('Press "w" to wind the motor, "u" to unwind the motor, or ESC to exit')

while 1:
    # get command from keypress
    keypress = getch()
    if keypress == chr(0x1b):
        break
    elif keypress == 'w':
        direction = 1
    elif keypress == 'u':
        direction = -1
    else:
        direction = 0

    # write new goal position
    dxl_goal_position[0] += direction*ROTATE_AMOUNT

    # move motor
    dxl_present_position = move(motor_id, dxl_goal_position, packetHandler, portHandler, groupBulkWrite, groupBulkRead, ADDR, LEN)

    # print current and voltage readings
    print_curr_volt(motor_id, 0, portHandler, packetHandler, groupBulkRead, ADDR, LEN)

    print('Moved. Press "w" to wind the motor, "u" to unwind the motor, or ESC to exit')
    print('')

shut_down(motor_id, packetHandler, portHandler, groupBulkRead, ADDR, LEN, askAction=False)
