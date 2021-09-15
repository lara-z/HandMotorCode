from utils_sensor import *
import numpy as np
import time

class read_pts:
	# order: abduction/adduction finger, right finger, left finger
    x_start = [4,6,2,0]
    x_end   = [6,8,4,2]
    y_start = [8,8,4,0]
    y_end   = [12,12,8,4]
visualize = True

args, ser, zeros, hist = initialize_sensor('/dev/ttyUSB0', visualize, read_pts)

t_start = time.time()

# while (time.time() - t_start) < 30:
while True:
    _,_,_,_,hist = read_pres(zeros, hist, args, ser, read_pts, print_pres=True)

# save_data(hist,"egg")