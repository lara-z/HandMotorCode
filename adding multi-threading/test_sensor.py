from functions import *
import numpy as np
import time

class read_pts:
	# order: abduction/adduction finger, right finger, left finger
    x_start = [0]
    x_end   = [32]
    y_start = [0]
    y_end   = [32]
visualize = False
com_port_sensor = '/dev/ttyUSB0'

th_wait, q_sens, finished = collect_pres(com_port_sensor, visualize, read_pts, 'test_data', 'test', print_pres=False)
i = 0
while i < 30:
    max_pres, _ = pres_from_q(q_sens)
    print(max_pres, end='\r')
    i += 1
end_sens(th_wait, finished)