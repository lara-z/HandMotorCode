from utils_sensor import *
class read_pts:
    x_start = [4,6,2,0]
    x_end   = [6,8,4,2]
    y_start = [8,8,4,0]
    y_end   = [12,12,8,4]
visualize = True
args, ser, p_zero, f_zero = initialize_sensor('/dev/ttyUSB0', visualize, read_pts)
while True:
    read_pres(p_zero, f_zero, args, ser, read_pts, print_pres=True)
