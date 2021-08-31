from utils_sensor import *
class read_pts:
    x_start = [10,0,20]
    x_end   = [22,12,32]
    y_start = [3,0,6]
    y_end   = [6,3,9]
visualize = True
args, ser, p_zero, f_zero, x_zero = initialize_sensor('/dev/ttyUSB1', visualize, read_pts)
while True:
    read_pres(p_zero, f_zero, x_zero, args, ser, read_pts, print_pres=True)
