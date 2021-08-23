from utils_sensor import *
class read_pts:
    x_start = [2,0,6,4]
    x_end   = [4,2,8,6]
    y_start = [0,3,6,10]
    y_end   = [3,6,9,12]
visualize = False
args, ser, p_zero, f_zero = initialize_sensor('/dev/ttyUSB1', visualize, read_pts)
while True:
    read_pres(p_zero, f_zero, args, ser, read_pts)
