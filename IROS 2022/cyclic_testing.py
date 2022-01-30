# cycling testing on base joint

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

class dxl_info:
    ids = [5,2,8,6]
    joint_num = [1, -1, 2, -2]
    joint_torque = [0.25, 0.2]
TORQUE_MARGIN = 0.01

ROTATE_AMOUNT = deg2pulse(5) # the increment in degrees (converted to motor pulse units) you would like the motor to move

def cycle():
    start_time = time.time()
    count = 0
    while q.empty():
        move_pair(-350, 1, dxl_info, SYS, ADDR, LEN)
        time.sleep(0.1)
        move_pair(350, 1, dxl_info, SYS, ADDR, LEN)
        count += 1
        if count % 20 == 0:
            text = 'Cycles: ' + str(count)
            print(text, end="\r")
        if count % 20 == 0:
            print('')
            tension_seq(dxl_info, SYS, ADDR, LEN)
    elapsed_time = time.time() - start_time
    duration = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))
    print('Duration:  '+str(duration)+', '+str(count)+' cycles')

if operating_mode == 'extended_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(dxl_info.ids, com_num, operating_mode)
elif operating_mode == 'current_position':
    dxl_start_position, dxl_goal_position, SYS, ADDR, LEN = initialize(dxl_info.ids, com_num, operating_mode, current_goal, current_limit)
else:
    print('invalid operating mode')

q = queue.Queue()
x = threading.Thread(target=cycle)

tension_seq(dxl_info, SYS, ADDR, LEN)
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

shut_down(dxl_info.ids, SYS, ADDR, LEN, askAction=False)
