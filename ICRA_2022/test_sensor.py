from utils_sensor import *
import numpy as np
import time

class read_pts:
	# order: abduction/adduction finger, right finger, left finger
    x_start = [10,0,20]
    x_end   = [22,12,32]
    y_start = [3,0,6]
    y_end   = [6,3,9]
visualize = True

args, ser, zeros, hist = initialize_sensor('/dev/ttyUSB0', visualize, read_pts)

t_start = time.time()

# while (time.time() - t_start) < 30:
while True:
    _,_,_,_,hist = read_pres(zeros, hist, args, ser, read_pts, print_pres=True)

# save_data(hist,"egg")