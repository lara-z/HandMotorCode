from functions import *
import numpy as np
import time

class read_pts:
	# order: abduction/adduction finger, right finger, left finger
    x_start = [0]
    x_end   = [32]
    y_start = [0]
    y_end   = [32]
visualize = True
com_port_sensor = '/dev/ttyUSB0'

th_wait, q_sens, events = collect_pres(com_port_sensor, visualize, read_pts, 'test_data', 'test', print_pres=False)

while True:
    if events.finished.is_set() == True:
        end_sens(th_wait, events, False)
        break