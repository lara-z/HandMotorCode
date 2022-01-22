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


import numpy as np
from functions import *
import time
import threading
import queue

com_num = '/dev/ttyUSB0'

# comment out one of the lines below to change operating mode
operating_mode = 'current_position' # current-based position control
# operating_mode = 'extended_position'

# specify current goal and limit if using current-based position control
current_limit = amps2curr(1.2) # input the number of amps and it will convert to motor units
current_goal = amps2curr(1.0) # input the goal number of amps

# inputs = input('What is the ID of the motor you would like to control?  ').split(", ")
# motor_id = [int(id) for id in inputs]
motor_id = [6, 2, 5, 8]

DXL_JOINT_NUM = [1, -1, 2, -2]
JOINT_TORQUE = [0.25, 0.2]
TORQUE_MARGIN = 0.01
joint_ratios = [1, 1, 1]

ROTATE_AMOUNT = deg2pulse(5) # the increment in degrees (converted to motor pulse units) you would like the motor to move

def cycle():
    start_time = time.time()
    count = 0
    while q.empty():
        move_pair(1, -350, motor_id, DXL_JOINT_NUM, joint_ratios, JOINT_TORQUE, SYS, ADDR, LEN)
        time.sleep(0.1)
        move_pair(1, 350, motor_id, DXL_JOINT_NUM, joint_ratios, JOINT_TORQUE, SYS, ADDR, LEN)
        count += 1
        if count % 20 == 0:
            text = 'Cycles: ' + str(count)
            print(text, end="\r")
        if count % 20 == 0:
            print('')
            tension_seq(motor_id, DXL_JOINT_NUM, JOINT_TORQUE, SYS, ADDR, LEN)
    elapsed_time = time.time() - start_time
    duration = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))
    print('Duration:  '+str(duration)+', '+str(count)+' cycles')

if operating_mode == 'extended_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(motor_id, com_num, operating_mode)
elif operating_mode == 'current_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(motor_id, com_num, operating_mode, current_goal, current_limit)
else:
    print('invalid operating mode')

q = queue.Queue()
x = threading.Thread(target=cycle)

tension_seq(motor_id, DXL_JOINT_NUM, JOINT_TORQUE, SYS, ADDR, LEN)
time.sleep(2)

print('Starting run')
print('press y to move the finger or press ESC to stop')
x.start()

while True:
    keypress = getch()
    if keypress == chr(0x1b):
        q.put('stop!')
        x.join()
        print('This run has been stopped')
        break

shut_down(motor_id, SYS, ADDR, LEN, askAction=False)
