from utils_sensor import *
import numpy as np
import time

class read_pts:
	# order: palmar thumb, dorsal thumb, palmar pointer, palmar middle finger
    x_start = [4,6,2,0]
    x_end   = [6,8,4,2]
    y_start = [8,8,4,0]
    y_end   = [12,12,8,4]
visualize = True

args, ser, zeros, hist = initialize_sensor('/dev/ttyUSB1', visualize, read_pts)

t_start = time.time()

while (time.time() - t_start) < 30:
    _,_,_,_,hist = read_pres(zeros, hist, args, ser, read_pts, print_pres=False)

# save_data(hist,"egg")