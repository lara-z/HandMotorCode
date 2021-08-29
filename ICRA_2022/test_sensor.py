from utils_sensor import *
class read_pts:
    x_start = [0]
    x_end   = [31]
    y_start = [0]
    y_end   = [31]
visualize = True
# '/dev/ttyUSB1'
args, ser, p_zero, f_zero = initialize_sensor('COM4', visualize, read_pts)
while True:
    read_pres(p_zero, f_zero, args, ser, read_pts, print_pres=True)
