from utils_sensor import *
import numpy as np
import time

class read_pts:
	# order: small left, big left, big right, small right finger
    x_start = [2,0,6,4]
    x_end   = [4,2,8,6]
    y_start = [0,3,6,10]
    y_end   = [3,6,9,12]
visualize = False

args, ser, zeros, hist = initialize_sensor('/dev/ttyUSB0', visualize, read_pts)

t_start = time.time()

while (time.time() - t_start) < 40:
    _,_,_,_,hist = read_pres(zeros, hist, args, ser, read_pts, print_pres=False)
    print(time.time() - t_start)

np.savetxt("hist_force_3.csv",hist.force,delimiter=",")
np.savetxt("hist_pres_3.csv",hist.max_press,delimiter=",")