from utils_sensor import *
class read_pts:
    x_start = [0]
    x_end = [3]
    y_start = [28]
    y_end = [32]
visualize = True
args, ser, p_zero, f_zero = initialize_sensor('/dev/ttyUSB0', visualize, read_pts)
while True:
    read_pres(p_zero, f_zero, args, ser, read_pts)
